// m5-stopwatch-earth: Blue Marble on the StopWatch's round AMOLED, orthographic, software-rendered.
//
// Each frame is drawn in horizontal strips into two DMA buffers in internal RAM: while one
// strip is on its way to the panel the CPU fills the other. Every texel comes from the
// 1024x512 RGB565 texture embedded in flash.
//
// Modes:
//   SPIN  - the globe turns by itself, touch is not read over I2C. Measures drawing alone.
//           The touch controller's interrupt line still switches to TOUCH on the first contact.
//   TOUCH - touch is read once per frame, dragging turns the globe. Measures drawing with
//           touch, and the delay from the touch sample to the end of the frame that shows it.
//           The left button goes back to SPIN.
// Both the button and the touch line are caught by GPIO interrupts: a frame takes hundreds of
// milliseconds in the first cut, and polling once per frame missed short presses.
//
// Every 2 s a STAT line goes to USB serial (see tools/serial_log.py).
#include <M5Unified.h>
#include <esp_heap_caps.h>

#include "globe_math.h"

extern const uint8_t tex_start[] asm("_binary_data_earth_1024x512_rgb565be_start");
extern const uint8_t tex_end[] asm("_binary_data_earth_1024x512_rgb565be_end");

namespace {

constexpr int TW = 1024, TH = 512;
constexpr int kStripLines = 16;
constexpr int kButtonLeft = 2;  // KEYA, yellow
constexpr int kTouchInt = 13;   // TP_INT, low while the controller has a report
volatile bool g_button_edge = false;
volatile bool g_touch_edge = false;
void IRAM_ATTR on_button() { g_button_edge = true; }
void IRAM_ATTR on_touch_int() { g_touch_edge = true; }
constexpr float kDeg = globe::kPi / 180.0f;

const uint16_t* g_tex = nullptr;  // big-endian RGB565, as the panel takes it
int g_w = 0, g_h = 0;
globe::Disc g_disc;
uint16_t* g_buf[2] = {nullptr, nullptr};

enum class Mode { Spin, Touch };
Mode g_mode = Mode::Spin;

float g_lat0 = 20.0f * kDeg;
float g_lon0 = 135.0f * kDeg;

// Statistics over one reporting window.
struct Stats {
  uint32_t frames = 0;
  uint64_t render_us = 0;   // CPU time spent computing texels
  uint64_t frame_us = 0;    // wall time per frame, start to DMA done
  uint64_t touch_us = 0;    // time in the touch read
  uint32_t touched_frames = 0;
  uint32_t lat_n = 0;       // frames that carried a new touch position
  uint64_t lat_sum_us = 0;
  uint32_t lat_min_us = UINT32_MAX;
  uint32_t lat_max_us = 0;
  uint32_t t0_ms = 0;
} g_stats;

// One strip: rows [y0, y0+lines) into buf, full width, black outside the disc.
void IRAM_ATTR render_strip(const globe::View& v, int y0, int lines, uint16_t* buf) {
  for (int r = 0; r < lines; ++r) {
    const int j = y0 + r;
    uint16_t* row = buf + r * g_w;
    int i0, i1;
    if (!g_disc.row_span(j, g_w, &i0, &i1)) {
      memset(row, 0, g_w * 2);
      continue;
    }
    if (i0 > 0) memset(row, 0, i0 * 2);
    if (i1 < g_w - 1) memset(row + i1 + 1, 0, (g_w - 1 - i1) * 2);
    const float py = (g_disc.cy - (j + 0.5f)) * g_disc.inv_r;
    const float py2 = py * py;
    for (int i = i0; i <= i1; ++i) {
      const float px = (i + 0.5f - g_disc.cx) * g_disc.inv_r;
      float rem = 1.0f - px * px - py2;
      if (rem < 0.0f) rem = 0.0f;  // span edge rounding
      const float pz = sqrtf(rem);
      row[i] = g_tex[globe::view_to_texel_fast<TW, TH>(v, px, py, pz)];
    }
  }
}

// Draws one whole frame. Returns the CPU time spent rendering, in microseconds.
uint32_t draw_frame(const globe::View& v) {
  uint32_t render = 0;
  int b = 0;
  M5.Display.startWrite();
  for (int y = 0; y < g_h; y += kStripLines) {
    const int lines = (y + kStripLines <= g_h) ? kStripLines : (g_h - y);
    const uint32_t t = micros();
    render_strip(v, y, lines, g_buf[b]);
    render += micros() - t;
    // pushImageDMA waits for the previous transfer, so buffer b^1 is free to fill next.
    M5.Display.pushImageDMA(0, y, g_w, lines, reinterpret_cast<const lgfx::swap565_t*>(g_buf[b]));
    b ^= 1;
  }
  M5.Display.waitDMA();
  M5.Display.endWrite();
  return render;
}

// Touch state carried between frames.
bool g_was_down = false;
int g_last_x = 0, g_last_y = 0;

void report(uint32_t now_ms) {
  Stats& s = g_stats;
  const float secs = (now_ms - s.t0_ms) / 1000.0f;
  const float n = s.frames ? s.frames : 1;
  Serial.printf(
      "STAT mode=%s frames=%u fps=%.2f render_ms=%.2f frame_ms=%.2f touch_ms=%.3f touched=%u "
      "lat_n=%u lat_avg_ms=%.2f lat_min_ms=%.2f lat_max_ms=%.2f heap=%u psram=%u\n",
      g_mode == Mode::Spin ? "spin" : "touch", s.frames, s.frames / secs, s.render_us / n / 1000.0f,
      s.frame_us / n / 1000.0f, s.touch_us / n / 1000.0f, s.touched_frames, s.lat_n,
      s.lat_n ? s.lat_sum_us / (float)s.lat_n / 1000.0f : 0.0f, s.lat_n ? s.lat_min_us / 1000.0f : 0.0f,
      s.lat_max_us / 1000.0f, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  s = Stats{};
  s.t0_ms = now_ms;
}

// Boot-time microbenchmark: one full disc at a fixed view, three variants, one variable each.
//   math   - texel index only, no memory read
//   flash  - plus the read from the texture in flash (through the MMU cache)
//   psram  - plus the read from a copy of the texture in PSRAM
void IRAM_ATTR bench_pass(const globe::View& v, const uint16_t* tex, uint32_t* out_sum, uint32_t* out_us) {
  uint32_t sum = 0;
  const uint32_t t0 = micros();
  for (int j = 0; j < g_h; ++j) {
    int i0, i1;
    if (!g_disc.row_span(j, g_w, &i0, &i1)) continue;
    const float py = (g_disc.cy - (j + 0.5f)) * g_disc.inv_r;
    const float py2 = py * py;
    for (int i = i0; i <= i1; ++i) {
      const float px = (i + 0.5f - g_disc.cx) * g_disc.inv_r;
      float rem = 1.0f - px * px - py2;
      if (rem < 0.0f) rem = 0.0f;
      const uint32_t idx = globe::view_to_texel_fast<TW, TH>(v, px, py, sqrtf(rem));
      sum += tex ? tex[idx] : idx;
    }
  }
  *out_us = micros() - t0;
  *out_sum = sum;
}

void run_bench() {
  long pixels = 0;
  for (int j = 0; j < g_h; ++j) {
    int i0, i1;
    if (g_disc.row_span(j, g_w, &i0, &i1)) pixels += i1 - i0 + 1;
  }
  uint16_t* psram_tex = static_cast<uint16_t*>(heap_caps_malloc(TW * TH * 2, MALLOC_CAP_SPIRAM));
  if (psram_tex) memcpy(psram_tex, g_tex, TW * TH * 2);
  const globe::View v = globe::View::make(g_lat0, g_lon0);
  struct {
    const char* name;
    const uint16_t* tex;
  } variants[] = {{"math", nullptr}, {"flash", g_tex}, {"psram", psram_tex}};
  for (auto& var : variants) {
    if (var.tex == nullptr && strcmp(var.name, "math") != 0) continue;
    uint32_t sum, us;
    bench_pass(v, var.tex, &sum, &us);  // warm-up
    bench_pass(v, var.tex, &sum, &us);
    Serial.printf("BENCH %s pixels=%ld ms=%.2f cycles_per_px=%.1f sum=%u\n", var.name, pixels, us / 1000.0f,
                  us * (float)getCpuFrequencyMhz() / pixels, (unsigned)sum);
  }
  heap_caps_free(psram_tex);
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  pinMode(kButtonLeft, INPUT_PULLUP);
  attachInterrupt(kButtonLeft, on_button, FALLING);

  g_w = M5.Display.width();
  g_h = M5.Display.height();
  g_disc = globe::Disc::make(g_w, g_h);
  g_tex = reinterpret_cast<const uint16_t*>(tex_start);

  for (auto& b : g_buf) {
    b = static_cast<uint16_t*>(heap_caps_malloc(g_w * kStripLines * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  }
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setSwapBytes(false);

  Serial.printf("BOOT board=%d display=%dx%d texture_bytes=%u expected=%u strip_lines=%d cpu_mhz=%u\n",
                (int)M5.getBoard(), g_w, g_h, (unsigned)(tex_end - tex_start), (unsigned)(TW * TH * 2), kStripLines,
                (unsigned)getCpuFrequencyMhz());
  if (!g_buf[0] || !g_buf[1]) Serial.println("ERROR strip buffer allocation failed");
  delay(1500);  // let a serial reader attach before the benchmark lines go out
  run_bench();
  // After M5.begin, so the touch driver's own pin setup does not replace the handler.
  pinMode(kTouchInt, INPUT_PULLUP);
  attachInterrupt(kTouchInt, on_touch_int, FALLING);
  g_touch_edge = false;
  g_button_edge = false;
  g_stats.t0_ms = millis();
}

void loop() {
  // Left button: back to SPIN. First contact while spinning: to TOUCH.
  Mode next = g_mode;
  if (g_button_edge) {
    g_button_edge = false;
    next = Mode::Spin;
    Serial.println("EVENT button");
  }
  if (g_touch_edge) {
    g_touch_edge = false;
    if (g_mode == Mode::Spin && next == Mode::Spin) next = Mode::Touch;
  }
  if (next != g_mode) {
    report(millis());  // close the window of the old mode
    g_mode = next;
    g_was_down = false;
    Serial.printf("MODE %s\n", g_mode == Mode::Spin ? "spin" : "touch");
  }

  const uint32_t f0 = micros();
  uint32_t touch_sample_us = 0;
  bool new_touch = false;

  if (g_mode == Mode::Spin) {
    g_lon0 += 1.0f * kDeg;
  } else {
    lgfx::touch_point_t tp;
    const uint32_t t = micros();
    const int n = M5.Display.getTouch(&tp, 1);
    touch_sample_us = micros();
    g_stats.touch_us += touch_sample_us - t;
    if (n > 0) {
      ++g_stats.touched_frames;
      if (g_was_down && (tp.x != g_last_x || tp.y != g_last_y)) {
        // Drag: the point under the finger follows it, roughly (radians per pixel at the centre).
        const float k = 1.0f / g_disc.r;
        g_lon0 -= (tp.x - g_last_x) * k;
        g_lat0 += (tp.y - g_last_y) * k;
        if (g_lat0 > 89.0f * kDeg) g_lat0 = 89.0f * kDeg;
        if (g_lat0 < -89.0f * kDeg) g_lat0 = -89.0f * kDeg;
        new_touch = true;
      }
      g_last_x = tp.x;
      g_last_y = tp.y;
      g_was_down = true;
    } else {
      g_was_down = false;
    }
  }

  const globe::View v = globe::View::make(g_lat0, g_lon0);
  g_stats.render_us += draw_frame(v);
  const uint32_t done = micros();
  g_stats.frame_us += done - f0;
  ++g_stats.frames;
  if (new_touch) {
    const uint32_t lat = done - touch_sample_us;
    ++g_stats.lat_n;
    g_stats.lat_sum_us += lat;
    if (lat < g_stats.lat_min_us) g_stats.lat_min_us = lat;
    if (lat > g_stats.lat_max_us) g_stats.lat_max_us = lat;
  }

  const uint32_t now = millis();
  if (now - g_stats.t0_ms >= 2000) report(now);
}
