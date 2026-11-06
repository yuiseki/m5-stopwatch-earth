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
#include <lgfx/v1/panel/Panel_CO5300.hpp>  // Panel_AMOLED and its framebuffer (needs LGFX_USE_QSPI, set by M5GFX on ESP32-S3)

#include "globe_math.h"

extern const uint8_t tex_start[] asm("_binary_data_earth_1024x512_rgb565be_start");
extern const uint8_t tex_end[] asm("_binary_data_earth_1024x512_rgb565be_end");

namespace {

// On the StopWatch, M5.Display is a PSRAM framebuffer (Panel_AMOLED_Framebuffer) in front of
// the CO5300 panel. Writes land in the framebuffer, and every endWrite() then sends the whole
// 438 KB to the panel, blocking for about 22 ms. Strips are sent to the panel underneath
// instead, so each strip's DMA runs while the next strip is being rendered.
// The framebuffer keeps that panel in a protected member; a member pointer taken inside a
// derived class reaches it without casting the object.
struct FbPeek : lgfx::Panel_AMOLED_Framebuffer {
  static lgfx::Panel_AMOLED* underlying(lgfx::Panel_AMOLED_Framebuffer* fb) { return fb->*(&FbPeek::_panel); }
};
lgfx::LGFX_Device g_direct;
lgfx::LGFX_Device* g_out = nullptr;  // &g_direct, or &M5.Display if the panel could not be reached
bool g_direct_ok = false;

constexpr int TW = 1024, TH = 512;
constexpr int kStripLines = 16;
// Adaptive row interpolation (see globe::render_row_adaptive). On the host, seg 32 / tol 0.5
// never strays more than one texel from the exact mapping and needs 0.17 exact evaluations per
// pixel (tools/interp_eval.cpp).
constexpr int kSeg = 32;
constexpr float kTol = 0.5f;
// Quadratic rows (globe::render_row_quadratic): tol in screen pixels. seg 64 / tol 0.25 never
// strays more than one pixel on screen and needs 0.157 exact evaluations per pixel.
constexpr int kQuadSeg = 64;
constexpr float kQuadTol = 0.25f;
bool g_quadratic = true;
constexpr int kButtonLeft = 2;  // KEYA, yellow
constexpr int kButtonRight = 1; // KEYB, blue: toggles one or two rendering cores
volatile bool g_right_edge = false;
void IRAM_ATTR on_right() { g_right_edge = true; }
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
  uint64_t render_us = 0;   // wall time spent computing texels (both cores, until both are done)
  uint64_t push_us = 0;     // wall time blocked in the display transfer calls
  uint64_t frame_us = 0;    // wall time per frame, start to DMA done
  uint64_t touch_us = 0;    // time in the touch read
  uint32_t touched_frames = 0;
  uint32_t lat_n = 0;       // frames that carried a new touch position
  uint64_t lat_sum_us = 0;
  uint32_t lat_min_us = UINT32_MAX;
  uint32_t lat_max_us = 0;
  uint32_t t0_ms = 0;
} g_stats;

// Rows [r0, r1) of the strip that starts at screen row y0, into buf (full width, black
// outside the disc).
void IRAM_ATTR render_rows(const globe::View& v, int y0, int r0, int r1, uint16_t* buf) {
  for (int r = r0; r < r1; ++r) {
    const int j = y0 + r;
    uint16_t* row = buf + r * g_w;
    int i0, i1;
    if (!g_disc.row_span(j, g_w, &i0, &i1)) {
      memset(row, 0, g_w * 2);
      continue;
    }
    if (i0 > 0) memset(row, 0, i0 * 2);
    if (i1 < g_w - 1) memset(row + i1 + 1, 0, (g_w - 1 - i1) * 2);
    if (g_quadratic) {
      globe::render_row_quadratic<TW, TH>(v, g_disc, j, i0, i1, kQuadSeg, kQuadTol, g_tex, row);
    } else {
      globe::render_row_adaptive<TW, TH>(v, g_disc, j, i0, i1, kSeg, kTol, g_tex, row);
    }
  }
}

// Second rendering core. The loop task runs on core 1; this worker sits on core 0 and renders
// the lower half of each strip while the loop renders the upper half.
int g_cores = 2;
TaskHandle_t g_worker = nullptr;
TaskHandle_t g_loop_task = nullptr;
struct Job {
  globe::View v;
  int y0, r0, r1;
  uint16_t* buf;
} g_job;

void worker_main(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    render_rows(g_job.v, g_job.y0, g_job.r0, g_job.r1, g_job.buf);
    xTaskNotifyGive(g_loop_task);
  }
}

// Draws one whole frame. Adds to g_stats.render_us and g_stats.push_us.
void draw_frame(const globe::View& v) {
  int b = 0;
  g_out->startWrite();
  for (int y = 0; y < g_h; y += kStripLines) {
    const int lines = (y + kStripLines <= g_h) ? kStripLines : (g_h - y);
    const uint32_t t = micros();
    if (g_cores == 2 && lines > 1) {
      const int split = lines / 2;
      g_job = Job{v, y, split, lines, g_buf[b]};
      xTaskNotifyGive(g_worker);
      render_rows(v, y, 0, split, g_buf[b]);
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } else {
      render_rows(v, y, 0, lines, g_buf[b]);
    }
    const uint32_t t1 = micros();
    g_stats.render_us += t1 - t;
    // pushImageDMA waits for the previous transfer, so buffer b^1 is free to fill next.
    g_out->pushImageDMA(0, y, g_w, lines, reinterpret_cast<const lgfx::swap565_t*>(g_buf[b]));
    g_stats.push_us += micros() - t1;
    b ^= 1;
  }
  const uint32_t t2 = micros();
  g_out->waitDMA();
  g_out->endWrite();
  g_stats.push_us += micros() - t2;
}

// Touch state carried between frames.
bool g_was_down = false;
int g_last_x = 0, g_last_y = 0;

void report(uint32_t now_ms) {
  Stats& s = g_stats;
  const float secs = (now_ms - s.t0_ms) / 1000.0f;
  const float n = s.frames ? s.frames : 1;
  Serial.printf(
      "STAT mode=%s cores=%d frames=%u fps=%.2f render_ms=%.2f push_ms=%.2f frame_ms=%.2f touch_ms=%.3f touched=%u "
      "lat_n=%u lat_avg_ms=%.2f lat_min_ms=%.2f lat_max_ms=%.2f heap=%u psram=%u\n",
      g_mode == Mode::Spin ? "spin" : "touch", g_cores, s.frames, s.frames / secs, s.render_us / n / 1000.0f,
      s.push_us / n / 1000.0f, s.frame_us / n / 1000.0f, s.touch_us / n / 1000.0f, s.touched_frames, s.lat_n,
      s.lat_n ? s.lat_sum_us / (float)s.lat_n / 1000.0f : 0.0f, s.lat_n ? s.lat_min_us / 1000.0f : 0.0f,
      s.lat_max_us / 1000.0f, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  s = Stats{};
  s.t0_ms = now_ms;
}

// Boot-time microbenchmark: one full disc at a fixed view, rendered row by row into a scratch
// row (no display transfer). Exact vs adaptive, texture in flash vs a copy in PSRAM.
void IRAM_ATTR bench_pass(const globe::View& v, const uint16_t* tex, int seg, uint16_t* rowbuf, uint32_t* out_sum,
                          uint32_t* out_us) {
  uint32_t sum = 0;
  const uint32_t t0 = micros();
  for (int j = 0; j < g_h; ++j) {
    int i0, i1;
    if (!g_disc.row_span(j, g_w, &i0, &i1)) continue;
    if (seg <= 1) {
      globe::render_row<TW, TH>(v, g_disc, j, i0, i1, 1, 0.0f, tex, rowbuf);
    } else if (seg == kQuadSeg) {
      globe::render_row_quadratic<TW, TH>(v, g_disc, j, i0, i1, kQuadSeg, kQuadTol, tex, rowbuf);
    } else {
      globe::render_row_adaptive<TW, TH>(v, g_disc, j, i0, i1, seg, kTol, tex, rowbuf);
    }
    sum += rowbuf[(i0 + i1) / 2];
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
  uint16_t* rowbuf = static_cast<uint16_t*>(heap_caps_malloc(g_w * 2, MALLOC_CAP_INTERNAL));
  struct {
    const char* name;
    const uint16_t* tex;
    int seg;
  } variants[] = {{"exact_flash", g_tex, 1}, {"exact_psram", psram_tex, 1}, {"adaptive_flash", g_tex, kSeg},
                  {"adaptive_psram", psram_tex, kSeg},
                  {"quadratic_flash", g_tex, kQuadSeg}, {"quadratic_psram", psram_tex, kQuadSeg}};
  for (auto& var : variants) {
    if (var.tex == nullptr) continue;
    uint32_t sum, us;
    bench_pass(v, var.tex, var.seg, rowbuf, &sum, &us);  // warm-up
    bench_pass(v, var.tex, var.seg, rowbuf, &sum, &us);
    Serial.printf("BENCH %s pixels=%ld ms=%.2f cycles_per_px=%.1f sum=%u\n", var.name, pixels, us / 1000.0f,
                  us * (float)getCpuFrequencyMhz() / pixels, (unsigned)sum);
  }
  // The fill loop alone: no exact evaluation, u and v stepped like an accepted segment.
  for (const uint16_t* tex : {g_tex, (const uint16_t*)psram_tex}) {
    if (!tex) continue;
    uint32_t sum = 0;
    const uint32_t t0 = micros();
    for (int j = 0; j < g_h; ++j) {
      int i0, i1;
      if (!g_disc.row_span(j, g_w, &i0, &i1)) continue;
      int32_t U = (j * 3) << 16, V = (j * TH / g_h) << 16;
      const int32_t dU = 46000, dV = 3000;  // about 0.7 and 0.05 texel per pixel
      for (int i = i0; i <= i1; ++i) {
        const int iu = (U >> 16) & (TW - 1);
        int iv = V >> 16;
        if (iv > TH - 1) iv = TH - 1;
        if (iv < 0) iv = 0;
        rowbuf[i] = tex[iv * TW + iu];
        U += dU;
        V += dV;
      }
      sum += rowbuf[(i0 + i1) / 2];
    }
    const uint32_t us = micros() - t0;
    Serial.printf("BENCH fill_only_%s ms=%.2f cycles_per_px=%.1f sum=%u\n", tex == g_tex ? "flash" : "psram",
                  us / 1000.0f, us * (float)getCpuFrequencyMhz() / pixels, (unsigned)sum);
  }
  heap_caps_free(rowbuf);

  // Display transfer: how long does pushImageDMA block, and how long until the DMA is done?
  memset(g_buf[0], 0, g_w * kStripLines * 2);
  for (int lines : {1, 4, 16}) {
    uint32_t call_us = 0, total_us = 0;
    const int reps = 20;
    g_out->startWrite();
    for (int k = 0; k < reps; ++k) {
      const uint32_t t0 = micros();
      g_out->pushImageDMA(0, 0, g_w, lines, reinterpret_cast<const lgfx::swap565_t*>(g_buf[0]));
      const uint32_t t1 = micros();
      g_out->waitDMA();
      const uint32_t t2 = micros();
      call_us += t1 - t0;
      total_us += t2 - t0;
    }
    g_out->endWrite();
    const float bytes = g_w * lines * 2.0f;
    // Back to back with a gap: if the transfer runs in the background, the second call after a
    // 3 ms gap returns as fast as the first.
    g_out->startWrite();
    g_out->pushImageDMA(0, 0, g_w, lines, reinterpret_cast<const lgfx::swap565_t*>(g_buf[0]));
    delayMicroseconds(3000);
    const uint32_t g0 = micros();
    g_out->pushImageDMA(0, 0, g_w, lines, reinterpret_cast<const lgfx::swap565_t*>(g_buf[0]));
    const uint32_t g1 = micros();
    g_out->pushImageDMA(0, 0, g_w, lines, reinterpret_cast<const lgfx::swap565_t*>(g_buf[0]));
    const uint32_t g2 = micros();
    g_out->endWrite();
    const uint32_t g3 = micros();
    Serial.printf("PUSHGAP lines=%d second_call_after_gap_us=%u third_call_back_to_back_us=%u endwrite_us=%u\n", lines,
                  (unsigned)(g1 - g0), (unsigned)(g2 - g1), (unsigned)(g3 - g2));
    Serial.printf("PUSH lines=%d bytes=%.0f call_us=%.1f total_us=%.1f mbyte_per_s=%.1f\n", lines, bytes,
                  call_us / (float)reps, total_us / (float)reps, bytes / (total_us / (float)reps));
  }
  heap_caps_free(psram_tex);
}

}  // namespace

#ifdef DIAG_COLORS
// Diagnostic: which way of writing reaches the glass? Each colour fills the whole screen for
// 3 s, in a loop: red via the framebuffer (known good), blue via g_direct.pushImageDMA strips
// (the renderer's path), green via setWindow/start_qspi/writeBytes on the underlying panel
// (the framebuffer's own flush, called by hand), white via the framebuffer again.
lgfx::Panel_AMOLED* g_panel = nullptr;
void fill_strips_direct(uint16_t be) {
  for (int k = 0; k < g_w * kStripLines; ++k) g_buf[0][k] = be;
  g_direct.startWrite();
  for (int y = 0; y < g_h; y += kStripLines) {
    const int lines = (y + kStripLines <= g_h) ? kStripLines : (g_h - y);
    g_direct.pushImageDMA(0, y, g_w, lines, reinterpret_cast<const lgfx::swap565_t*>(g_buf[0]));
  }
  g_direct.waitDMA();
  g_direct.endWrite();
}
void fill_strips_manual(uint16_t be) {
  for (int k = 0; k < g_w * kStripLines; ++k) g_buf[0][k] = be;
  auto* bus = g_panel->getBus();
  g_panel->startWrite();
  for (int y = 0; y < g_h; y += kStripLines) {
    const int lines = (y + kStripLines <= g_h) ? kStripLines : (g_h - y);
    g_panel->setWindow(0, y, g_w - 1, y + lines - 1);
    g_panel->start_qspi();
    bus->writeBytes(reinterpret_cast<const uint8_t*>(g_buf[0]), g_w * lines * 2, false, true);
  }
  bus->wait();
  g_panel->endWrite();
}
void diag_loop() {
  for (int round = 0;; ++round) {
    M5.Display.fillScreen(TFT_RED);
    M5.Display.display();
    Serial.printf("DIAG %d red framebuffer\n", round);
    delay(3000);
    fill_strips_direct(0x1F00);  // blue 0x001F, big-endian
    Serial.printf("DIAG %d blue direct pushImageDMA\n", round);
    delay(3000);
    fill_strips_manual(0xE007);  // green 0x07E0, big-endian
    Serial.printf("DIAG %d green manual setWindow/start_qspi/writeBytes\n", round);
    delay(3000);
    M5.Display.fillScreen(TFT_WHITE);
    M5.Display.display();
    Serial.printf("DIAG %d white framebuffer\n", round);
    delay(3000);
  }
}
#endif

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  pinMode(kButtonLeft, INPUT_PULLUP);
  attachInterrupt(kButtonLeft, on_button, FALLING);
  pinMode(kButtonRight, INPUT_PULLUP);
  attachInterrupt(kButtonRight, on_right, FALLING);
  g_loop_task = xTaskGetCurrentTaskHandle();
  xTaskCreatePinnedToCore(worker_main, "render0", 4096, nullptr, 1, &g_worker, 0);

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.display();
  g_out = &M5.Display;
  if (M5.getBoard() == m5::board_t::board_M5StopWatch) {
    auto* fb = static_cast<lgfx::Panel_AMOLED_Framebuffer*>(M5.Display.getPanel());
    lgfx::Panel_AMOLED* panel = fb ? FbPeek::underlying(fb) : nullptr;
    if (panel) {
      g_direct.setPanel(panel);
      // Not init()ed (the panel already is), so the clip rectangle is still empty and every
      // write would be clipped away. Open it to the whole panel.
      int32_t cx, cy, cw, ch;
      g_direct.getClipRect(&cx, &cy, &cw, &ch);
      Serial.printf("BOOT direct clip before=%d,%d,%d,%d\n", (int)cx, (int)cy, (int)cw, (int)ch);
      g_direct.clearClipRect();
      g_direct.getClipRect(&cx, &cy, &cw, &ch);
      Serial.printf("BOOT direct clip after=%d,%d,%d,%d panel=%dx%d\n", (int)cx, (int)cy, (int)cw, (int)ch, (int)g_direct.width(),
                    (int)g_direct.height());
      g_out = &g_direct;
      g_direct_ok = true;
    }
  }
  g_w = g_out->width();
  g_h = g_out->height();
  g_disc = globe::Disc::make(g_w, g_h);
  g_tex = reinterpret_cast<const uint16_t*>(tex_start);

  for (auto& b : g_buf) {
    b = static_cast<uint16_t*>(heap_caps_malloc(g_w * kStripLines * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  }
  g_out->fillScreen(TFT_BLACK);

  Serial.printf("BOOT direct_panel=%d fb=%dx%d\n", g_direct_ok ? 1 : 0, (int)M5.Display.width(), (int)M5.Display.height());
  Serial.printf("BOOT board=%d display=%dx%d texture_bytes=%u expected=%u strip_lines=%d cpu_mhz=%u\n",
                (int)M5.getBoard(), g_w, g_h, (unsigned)(tex_end - tex_start), (unsigned)(TW * TH * 2), kStripLines,
                (unsigned)getCpuFrequencyMhz());
  if (!g_buf[0] || !g_buf[1]) Serial.println("ERROR strip buffer allocation failed");
#ifdef DIAG_COLORS
  {
    auto* fb = static_cast<lgfx::Panel_AMOLED_Framebuffer*>(M5.Display.getPanel());
    g_panel = FbPeek::underlying(fb);
    diag_loop();
  }
#endif
  delay(1500);  // let a serial reader attach before the benchmark lines go out
  run_bench();
  // After M5.begin, so the touch driver's own pin setup does not replace the handler.
  pinMode(kTouchInt, INPUT_PULLUP);
  attachInterrupt(kTouchInt, on_touch_int, FALLING);
  g_touch_edge = false;
  g_button_edge = false;
  g_right_edge = false;
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
  if (g_right_edge) {
    g_right_edge = false;
    report(millis());
    g_cores = (g_cores == 2) ? 1 : 2;
    Serial.printf("CORES %d\n", g_cores);
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
  draw_frame(v);
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
