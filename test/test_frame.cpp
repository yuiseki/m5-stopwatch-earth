// Host tests for include/frame.h: how a frame is cut into strips and shared between the two
// cores, and the touch latency statistics. Build and run with `make test`.
#include "frame.h"

#include <cstdint>
#include <cstdio>
#include <vector>

using namespace frame;

static int g_failures = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__);    \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
      ++g_failures;                                       \
    }                                                     \
  } while (0)

// Every screen row is rendered exactly once, by one core, whatever the height, strip size and
// number of cores. This is the loop of draw_frame in src/main.cpp.
static void test_strips_and_cores_cover_every_row_once() {
  for (int h : {1, 15, 16, 17, 31, 466, 468, 480}) {
    for (int strip : {1, 2, 16, 32}) {
      for (int cores : {1, 2}) {
        std::vector<int> seen(h, 0);
        int strips = 0;
        for (int y = 0; y < h; y += strip) {
          const int lines = strip_lines(y, h, strip);
          CHECK(lines >= 1 && lines <= strip, "h %d strip %d y %d: %d lines", h, strip, y, lines);
          const int split = core_split(lines, cores);
          CHECK(split >= 1 && split <= lines, "lines %d cores %d: split %d", lines, cores, split);
          for (int r = 0; r < split; ++r) ++seen[y + r];       // this core
          for (int r = split; r < lines; ++r) ++seen[y + r];   // the worker
          ++strips;
        }
        int bad = 0;
        for (int v : seen) bad += v != 1;
        CHECK(bad == 0, "h %d strip %d cores %d: %d rows not drawn exactly once", h, strip, cores, bad);
        CHECK(strips == (h + strip - 1) / strip, "h %d strip %d: %d strips", h, strip, strips);
      }
    }
  }
}

static void test_core_split() {
  CHECK(core_split(16, 1) == 16, "one core renders the whole strip");
  CHECK(core_split(16, 2) == 8, "two cores halve a 16-row strip, got %d", core_split(16, 2));
  CHECK(core_split(5, 2) == 2, "odd strip: the worker takes the larger half, got %d", core_split(5, 2));
  CHECK(core_split(1, 2) == 1, "a one-row strip is not split, got %d", core_split(1, 2));
}

static void test_last_strip_is_short() {
  CHECK(strip_lines(0, 466, 16) == 16, "first strip");
  CHECK(strip_lines(464, 466, 16) == 2, "last strip of 466 rows, got %d", strip_lines(464, 466, 16));
  CHECK(strip_lines(448, 464, 16) == 16, "exact fit, got %d", strip_lines(448, 464, 16));
}

static void test_latency_stats() {
  Latency l;
  CHECK(l.n == 0 && l.avg_us() == 0.0f && l.min_us_or_zero() == 0 && l.max_us == 0, "empty window reports zeros");
  l.add(50000);
  l.add(62000);
  l.add(48000);
  CHECK(l.n == 3, "n %u", l.n);
  CHECK(l.min_us_or_zero() == 48000 && l.max_us == 62000, "min %u max %u", l.min_us_or_zero(), l.max_us);
  CHECK(l.avg_us() == 160000.0f / 3.0f, "avg %g", l.avg_us());
  l = Latency{};
  CHECK(l.n == 0 && l.min_us_or_zero() == 0, "reset by assigning a fresh window");
}

int main() {
  test_strips_and_cores_cover_every_row_once();
  test_core_split();
  test_last_strip_is_short();
  test_latency_stats();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("all frame tests passed\n");
  return 0;
}
