// How a frame is cut into strips and shared between the two cores, and the touch latency
// statistics. Header-only and free of Arduino, so the host tests can check it (test/test_frame.cpp).
#pragma once

#include <cstdint>

namespace frame {

// Rows in the strip that starts at screen row y: `strip`, or fewer for the last one.
inline int strip_lines(int y, int height, int strip) { return y + strip <= height ? strip : height - y; }

// The calling core renders rows [0, split) of a strip and the worker on the other core renders
// [split, lines). With one core, or a one-row strip, split is lines and the worker is not used.
inline int core_split(int lines, int cores) { return cores == 2 && lines > 1 ? lines / 2 : lines; }

// Delay from a touch sample to the end of the frame that shows it, over one reporting window.
struct Latency {
  uint32_t n = 0;
  uint64_t sum_us = 0;
  uint32_t min_us = UINT32_MAX;
  uint32_t max_us = 0;

  void add(uint32_t us) {
    ++n;
    sum_us += us;
    if (us < min_us) min_us = us;
    if (us > max_us) max_us = us;
  }
  float avg_us() const { return n ? static_cast<float>(sum_us) / static_cast<float>(n) : 0.0f; }
  uint32_t min_us_or_zero() const { return n ? min_us : 0; }
};

}  // namespace frame
