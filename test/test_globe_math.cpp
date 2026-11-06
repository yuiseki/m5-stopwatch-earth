// Host tests for include/globe_math.h. Build and run with `make test`.
#include "globe_math.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace globe;

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

constexpr float kDeg = kPi / 180.0f;
constexpr int TW = 1024, TH = 512;
static uint32_t tex_ref(float lat, float lon) { return latlon_to_texel<TW, TH>(lat, lon); }
static uint32_t tex_fast(const View& v, float x, float y, float z) { return view_to_texel_fast<TW, TH>(v, x, y, z); }

static void test_atan2_accuracy() {
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> d(-1.0f, 1.0f);
  float worst = 0;
  for (int k = 0; k < 200000; ++k) {
    const float y = d(rng), x = d(rng);
    const float e = std::fabs(fast_atan2(y, x) - std::atan2(y, x));
    if (e > worst) worst = e;
  }
  // 2e-5 rad is 0.003 texel across a 1024-wide texture.
  CHECK(worst < 2e-5f, "fast_atan2 worst error %g rad", worst);
  std::printf("fast_atan2 worst error: %.3g rad\n", worst);
  CHECK(fast_atan2(0, 0) == 0.0f, "atan2(0,0) must be 0");
  CHECK(std::fabs(fast_atan2(1, 0) - kPi / 2) < 1e-5f, "atan2(1,0)");
  CHECK(std::fabs(fast_atan2(0, -1) - kPi) < 1e-5f, "atan2(0,-1)");
}

static void test_centre_shows_view_centre() {
  // The disc centre (0, 0, 1) must land on (lat0, lon0).
  const float cases[][2] = {{0, 0}, {35.7f, 139.7f}, {-33.9f, 151.2f}, {89.0f, -45.0f}, {-60.0f, 179.9f}};
  for (auto& c : cases) {
    View v = View::make(c[0] * kDeg, c[1] * kDeg);
    float lat, lon;
    view_to_latlon_ref(v, 0, 0, 1, &lat, &lon);
    CHECK(std::fabs(lat - c[0] * kDeg) < 1e-5f, "lat %g -> %g", c[0], lat / kDeg);
    CHECK(std::fabs(lon - c[1] * kDeg) < 1e-5f, "lon %g -> %g", c[1], lon / kDeg);
  }
}

static void test_up_is_north() {
  // From the equator, the top of the disc (0, 1, 0) is the north pole,
  // and the right edge (1, 0, 0) is 90 deg east of the centre.
  View v = View::make(0, 20 * kDeg);
  float lat, lon;
  view_to_latlon_ref(v, 0, 1, 0, &lat, &lon);
  CHECK(std::fabs(lat - kPi / 2) < 1e-5f, "top should be +90, got %g", lat / kDeg);
  view_to_latlon_ref(v, 1, 0, 0, &lat, &lon);
  CHECK(std::fabs(lat) < 1e-5f && std::fabs(lon - 110 * kDeg) < 1e-5f, "right edge should be (0,110), got (%g,%g)",
        lat / kDeg, lon / kDeg);
}

static void test_texel_corners() {
  // (lat +90, lon -180) is row 0 col 0; just south of the equator at lon 0 is the middle.
  CHECK(tex_ref(kPi / 2, -kPi) == 0, "NW corner");
  CHECK(tex_ref(-0.001f, 0.001f) == (TH / 2) * TW + TW / 2, "centre");
  CHECK(tex_ref(-kPi / 2, kPi - 1e-4f) == (TH - 1) * TW + (TW - 1), "SE corner clamps");
  // lon = +180 wraps to column 0.
  CHECK(tex_ref(0.1f, kPi) % TW == 0, "lon +180 wraps to col 0");
}

static void test_fast_matches_ref_over_disc() {
  // Every pixel of a 466 disc at several view centres: the fast path may differ from the
  // reference only where the reference sits within a hair of a texel boundary.
  const Disc disc = Disc::make(466, 466);
  const float centres[][2] = {{0, 0}, {35.7f, 139.7f}, {-80, -170}, {60, 10}};
  long total = 0, mismatched = 0;
  for (auto& c : centres) {
    View v = View::make(c[0] * kDeg, c[1] * kDeg);
    for (int j = 0; j < 466; ++j)
      for (int i = 0; i < 466; ++i) {
        float x, y, z;
        if (!disc.pixel_to_view(i, j, &x, &y, &z)) continue;
        float lat, lon;
        view_to_latlon_ref(v, x, y, z, &lat, &lon);
        const uint32_t a = tex_ref(lat, lon);
        const uint32_t b = tex_fast(v, x, y, z);
        ++total;
        if (a != b) {
          ++mismatched;
          const int du = std::abs(int(a % TW) - int(b % TW));
          const int dv = std::abs(int(a / TW) - int(b / TW));
          CHECK((du <= 1 || du == TW - 1) && dv <= 1, "pixel (%d,%d) off by (%d,%d) texels", i, j, du, dv);
        }
      }
  }
  std::printf("fast vs ref: %ld of %ld pixels differ by one texel\n", mismatched, total);
  CHECK(mismatched * 1000 < total, "too many mismatches: %ld / %ld", mismatched, total);
}

static void test_accumulated_lon_wraps() {
  // Dragging east many turns must give the same picture as the wrapped longitude.
  View a = View::make(10 * kDeg, 30 * kDeg);
  View b = View::make(10 * kDeg, 30 * kDeg + 50 * 2 * kPi);
  const Disc disc = Disc::make(466, 466);
  int diff = 0, n = 0;
  for (int j = 0; j < 466; j += 7)
    for (int i = 0; i < 466; i += 7) {
      float x, y, z;
      if (!disc.pixel_to_view(i, j, &x, &y, &z)) continue;
      ++n;
      if (tex_fast(a, x, y, z) != tex_fast(b, x, y, z)) ++diff;
    }
  // float lon0 loses precision after 50 turns; allow a few one-texel flips at boundaries.
  CHECK(diff * 50 < n, "%d of %d pixels changed after 50 turns", diff, n);
}

static void test_row_span_matches_pixel_test() {
  const Disc disc = Disc::make(466, 466);
  for (int j = 0; j < 466; ++j) {
    int i0 = -1, i1 = -1;
    const bool any = disc.row_span(j, 466, &i0, &i1);
    for (int i = 0; i < 466; ++i) {
      float x, y, z;
      const bool in = disc.pixel_to_view(i, j, &x, &y, &z);
      const bool span = any && i >= i0 && i <= i1;
      CHECK(in == span, "row %d col %d: pixel test %d, span %d [%d,%d]", j, i, in, span, i0, i1);
    }
  }
}

static void test_adaptive_rows_stay_within_one_texel() {
  // The device default (seg 32, tol 0.5) against the exact renderer: never more than one texel
  // off, at random views and looking straight down at each pole.
  std::vector<uint32_t> tex(TW * TH);
  for (uint32_t k = 0; k < tex.size(); ++k) tex[k] = k;
  const int n = 468;
  const Disc disc = Disc::make(n, n);
  std::vector<uint32_t> a(n), b(n);
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> uni(0.0f, 1.0f);
  std::vector<View> views = {View::make(89.5f * kDeg, 0.1f), View::make(-89.5f * kDeg, -3.0f), View::make(0, 0)};
  for (int k = 0; k < 12; ++k) views.push_back(View::make(std::asin(2 * uni(rng) - 1), (2 * uni(rng) - 1) * kPi));
  long px = 0, evals = 0;
  int worst = 0;
  for (const View& v : views)
    for (int j = 0; j < n; ++j) {
      int i0, i1;
      if (!disc.row_span(j, n, &i0, &i1)) continue;
      render_row<TW, TH>(v, disc, j, i0, i1, 1, 0, tex.data(), a.data());
      evals += render_row_adaptive<TW, TH>(v, disc, j, i0, i1, 32, 0.5f, tex.data(), b.data());
      for (int i = i0; i <= i1; ++i) {
        ++px;
        int du = std::abs(int(a[i] % TW) - int(b[i] % TW));
        if (du > TW / 2) du = TW - du;
        const int dv = std::abs(int(a[i] / TW) - int(b[i] / TW));
        if (du > worst) worst = du;
        if (dv > worst) worst = dv;
      }
    }
  CHECK(worst <= 1, "adaptive rows drift %d texels from exact", worst);
  CHECK(evals * 4 < px, "adaptive rows evaluated %ld times for %ld pixels", evals, px);
  std::printf("adaptive seg32 tol0.5: worst %d texel, %.3f evals/px\n", worst, double(evals) / px);
}

static void test_adaptive_single_pixel_rows() {
  // The top and bottom rows of a disc can be one or two pixels wide.
  std::vector<uint32_t> tex(TW * TH);
  for (uint32_t k = 0; k < tex.size(); ++k) tex[k] = k;
  const Disc disc = Disc::make(468, 468);
  const View v = View::make(0.3f, 1.0f);
  std::vector<uint32_t> a(468, 0xFFFFFFFFu), b(468, 0xFFFFFFFFu);
  for (int w = 0; w < 3; ++w) {
    const int i0 = 200, i1 = 200 + w;
    render_row<TW, TH>(v, disc, 100, i0, i1, 1, 0, tex.data(), a.data());
    render_row_adaptive<TW, TH>(v, disc, 100, i0, i1, 32, 0.5f, tex.data(), b.data());
    for (int i = i0; i <= i1; ++i) CHECK(a[i] == b[i], "width %d col %d: %u vs %u", w + 1, i, a[i], b[i]);
    CHECK(b[i1 + 1] == 0xFFFFFFFFu, "wrote past i1 for width %d", w + 1);
  }
}

int main() {
  test_atan2_accuracy();
  test_centre_shows_view_centre();
  test_up_is_north();
  test_texel_corners();
  test_fast_matches_ref_over_disc();
  test_accumulated_lon_wraps();
  test_row_span_matches_pixel_test();
  test_adaptive_rows_stay_within_one_texel();
  test_adaptive_single_pixel_rows();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
