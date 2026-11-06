// Host tests for include/globe_math.h. Build and run with `make test`.
#include "globe_math.h"

#include <algorithm>
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

static void test_fast_scalar_ops() {
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> d(1e-6f, 2.0f);
  float worst_sqrt = 0, worst_recip = 0;
  for (int k = 0; k < 200000; ++k) {
    const float x = d(rng);
    worst_sqrt = std::max(worst_sqrt, std::fabs(fast_sqrt(x) - std::sqrt(x)) / std::sqrt(x));
    worst_recip = std::max(worst_recip, std::fabs(fast_recip(x) * x - 1.0f));
  }
  std::printf("fast_sqrt worst rel error %.3g, fast_recip worst rel error %.3g\n", worst_sqrt, worst_recip);
  CHECK(worst_sqrt < 1e-5f, "fast_sqrt rel error %g", worst_sqrt);
  CHECK(worst_recip < 2e-6f, "fast_recip rel error %g", worst_recip);
  CHECK(fast_sqrt(0.0f) == 0.0f && fast_sqrt(-1.0f) == 0.0f, "fast_sqrt of 0 and negatives is 0");
  for (float x : {-2.5f, -2.0f, -0.5f, -0.0f, 0.0f, 0.5f, 1.0f, 1023.999f, 1024.0f, -1024.0f})
    CHECK(fast_floor(x) == (int)std::floor(x), "fast_floor(%g) = %d", x, fast_floor(x));
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

static void test_quadratic_rows_stay_within_one_pixel() {
  // The device default (seg 64, tol 0.25 px). Error is judged on screen: the texel distance
  // from the exact renderer divided by the local squeeze (texels per pixel between exact
  // neighbours, at least 1). Near the limb many texels share a pixel.
  std::vector<uint32_t> tex(TW * TH);
  for (uint32_t k = 0; k < tex.size(); ++k) tex[k] = k;
  const int n = 468;
  const Disc disc = Disc::make(n, n);
  std::vector<uint32_t> a(n), b(n);
  std::mt19937 rng(11);
  std::uniform_real_distribution<float> uni(0.0f, 1.0f);
  std::vector<View> views = {View::make(89.5f * kDeg, 0.1f), View::make(-89.5f * kDeg, -3.0f), View::make(0, 0)};
  for (int k = 0; k < 12; ++k) views.push_back(View::make(std::asin(2 * uni(rng) - 1), (2 * uni(rng) - 1) * kPi));
  auto dist = [](uint32_t p, uint32_t q) {
    int du = std::abs(int(p % TW) - int(q % TW));
    if (du > TW / 2) du = TW - du;
    const int dv = std::abs(int(p / TW) - int(q / TW));
    return du > dv ? du : dv;
  };
  long px = 0, evals = 0;
  float worst = 0;
  for (const View& v : views)
    for (int j = 0; j < n; ++j) {
      int i0, i1;
      if (!disc.row_span(j, n, &i0, &i1)) continue;
      render_row<TW, TH>(v, disc, j, i0, i1, 1, 0, tex.data(), a.data());
      evals += render_row_quadratic<TW, TH>(v, disc, j, i0, i1, 64, 0.25f, tex.data(), b.data());
      for (int i = i0; i <= i1; ++i) {
        ++px;
        int sq = 1;
        if (i > i0) sq = std::max(sq, dist(a[i], a[i - 1]));
        if (i < i1) sq = std::max(sq, dist(a[i], a[i + 1]));
        worst = std::max(worst, float(dist(a[i], b[i])) / float(sq));
      }
    }
  CHECK(worst <= 1.0f, "quadratic rows drift %.2f pixels on screen", worst);
  CHECK(evals * 5 < px, "quadratic rows evaluated %ld times for %ld pixels", evals, px);
  std::printf("quadratic seg64 tol0.25px: worst %.2f px on screen, %.3f evals/px\n", worst, double(evals) / px);

  // Rows one to three pixels wide, and nothing written past i1.
  const View v = View::make(0.3f, 1.0f);
  for (int w = 0; w < 3; ++w) {
    std::fill(a.begin(), a.end(), 0xFFFFFFFFu);
    std::fill(b.begin(), b.end(), 0xFFFFFFFFu);
    render_row<TW, TH>(v, disc, 100, 200, 200 + w, 1, 0, tex.data(), a.data());
    render_row_quadratic<TW, TH>(v, disc, 100, 200, 200 + w, 64, 0.25f, tex.data(), b.data());
    for (int i = 200; i <= 200 + w; ++i) CHECK(a[i] == b[i], "width %d col %d", w + 1, i);
    CHECK(b[201 + w] == 0xFFFFFFFFu, "wrote past i1 for width %d", w + 1);
  }
}

static void test_tiled_layouts_render_the_same_picture() {
  // A texture of texel ids in row-major order, and the same ids re-laid out in tiles. The
  // quadratic renderer must produce identical rows from either, for every tile size.
  std::vector<uint32_t> lin(TW * TH);
  for (uint32_t k = 0; k < lin.size(); ++k) lin[k] = k;
  auto tiled = [&](auto addr) {
    std::vector<uint32_t> t(TW * TH, 0xFFFFFFFFu);
    for (int iv = 0; iv < TH; ++iv)
      for (int iu = 0; iu < TW; ++iu) t[addr(iu, iv)] = uint32_t(iv) * TW + iu;
    return t;
  };
  const auto t4 = tiled([](int u, int v) { return texel_addr<TW, 4>(u, v); });
  const auto t8 = tiled([](int u, int v) { return texel_addr<TW, 8>(u, v); });
  const auto t16 = tiled([](int u, int v) { return texel_addr<TW, 16>(u, v); });
  for (const auto* t : {&t4, &t8, &t16})
    for (uint32_t x : *t) CHECK(x != 0xFFFFFFFFu, "tiled layout left a hole");
  const int n = 468;
  const Disc disc = Disc::make(n, n);
  std::vector<uint32_t> a(n), b(n);
  long px = 0, bad = 0;
  for (const View& v : {View::make(0.3f, 1.0f), View::make(89.0f * kDeg, -2.0f), View::make(-70.0f * kDeg, 3.0f)})
    for (int j = 0; j < n; j += 3) {
      int i0, i1;
      if (!disc.row_span(j, n, &i0, &i1)) continue;
      render_row_quadratic<TW, TH>(v, disc, j, i0, i1, 64, 0.25f, lin.data(), a.data());
      render_row_quadratic<TW, TH, uint32_t, 4>(v, disc, j, i0, i1, 64, 0.25f, t4.data(), b.data());
      for (int i = i0; i <= i1; ++i) bad += a[i] != b[i];
      render_row_quadratic<TW, TH, uint32_t, 8>(v, disc, j, i0, i1, 64, 0.25f, t8.data(), b.data());
      for (int i = i0; i <= i1; ++i) bad += a[i] != b[i];
      render_row_quadratic<TW, TH, uint32_t, 16>(v, disc, j, i0, i1, 64, 0.25f, t16.data(), b.data());
      for (int i = i0; i <= i1; ++i) bad += a[i] != b[i];
      px += 3 * (i1 - i0 + 1);
    }
  CHECK(bad == 0, "%ld of %ld pixels differ between layouts", bad, px);
}

static void centre_and_top(const View& v, float* clat, float* clon, float* tlat) {
  float lon;
  view_to_latlon_ref(v, 0, 0, 1, clat, clon);
  view_to_latlon_ref(v, 0, 1, 0, tlat, &lon);
}

static void test_drag_goes_over_the_pole_to_upside_down() {
  const float r = 234.0f;
  View v = View::make(0, 0);
  // Drag down (screen y grows) a quarter turn in small steps: the north pole comes to the centre.
  for (int k = 0; k < 100; ++k) v.drag(0, kPi / 2 * r / 100, r);
  float clat, clon, tlat;
  centre_and_top(v, &clat, &clon, &tlat);
  CHECK(std::fabs(clat - kPi / 2) < 1e-3f, "quarter turn down: centre lat %g", clat / kDeg);
  // Another quarter turn: over the pole, the far side of the globe, upside down.
  for (int k = 0; k < 100; ++k) v.drag(0, kPi / 2 * r / 100, r);
  centre_and_top(v, &clat, &clon, &tlat);
  CHECK(std::fabs(clat) < 1e-3f, "half turn down: centre lat %g", clat / kDeg);
  CHECK(std::fabs(std::fabs(clon) - kPi) < 1e-3f, "half turn down: centre lon %g", clon / kDeg);
  CHECK(std::fabs(tlat + kPi / 2) < 1e-3f, "half turn down: top of the disc should be the south pole, lat %g",
        tlat / kDeg);
}

static void test_drag_right_brings_the_west() {
  const float r = 234.0f;
  View v = View::make(0, 0);
  v.drag(kPi / 2 * r, 0, r);
  float clat, clon, tlat;
  centre_and_top(v, &clat, &clon, &tlat);
  CHECK(std::fabs(clat) < 1e-4f && std::fabs(clon + kPi / 2) < 1e-4f, "drag right: centre (%g, %g)", clat / kDeg,
        clon / kDeg);
  CHECK(std::fabs(tlat - kPi / 2) < 1e-4f, "drag right keeps north up, top lat %g", tlat / kDeg);
}

static void test_spin_equals_turning_longitude() {
  View a = View::make(20 * kDeg, 135 * kDeg);
  a.spin(10 * kDeg);
  const View b = View::make(20 * kDeg, 145 * kDeg);
  for (int i = 0; i < 9; ++i) CHECK(std::fabs(a.m[i] - b.m[i]) < 1e-5f, "spin m[%d] %g vs %g", i, a.m[i], b.m[i]);
}

static void test_many_drags_stay_a_rotation() {
  std::mt19937 rng(5);
  std::uniform_real_distribution<float> d(-6.0f, 6.0f);
  View v = View::make(0.4f, 1.2f);
  for (int k = 0; k < 10000; ++k) {
    v.drag(d(rng), d(rng), 234.0f);
    v.spin(0.017f);
  }
  float worst = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const float dot = v.m[i * 3] * v.m[j * 3] + v.m[i * 3 + 1] * v.m[j * 3 + 1] + v.m[i * 3 + 2] * v.m[j * 3 + 2];
      worst = std::max(worst, std::fabs(dot - (i == j ? 1.0f : 0.0f)));
    }
  CHECK(worst < 1e-5f, "after 10000 drags M M^T is off by %g", worst);
}

static void test_equator_reset_keeps_the_facing_longitude() {
  View v = View::make(0.3f, 2.0f);
  for (int k = 0; k < 37; ++k) v.drag(9.0f, 13.0f, 234.0f);  // somewhere odd, maybe upside down
  float clat, clon, tlat;
  centre_and_top(v, &clat, &clon, &tlat);
  const View r = v.equator_reset();
  float rlat, rlon, rtlat;
  centre_and_top(r, &rlat, &rlon, &rtlat);
  CHECK(std::fabs(rlat) < 1e-5f, "reset centre lat %g", rlat / kDeg);
  CHECK(std::fabs(rlon - clon) < 1e-4f, "reset keeps lon %g, got %g", clon / kDeg, rlon / kDeg);
  CHECK(std::fabs(rtlat - kPi / 2) < 1e-5f, "reset puts north on top, top lat %g", rtlat / kDeg);
  // From a pole at the centre: lon 0.
  const View p = View::make(kPi / 2, 1.0f).equator_reset();
  centre_and_top(p, &rlat, &rlon, &rtlat);
  CHECK(std::fabs(rlat) < 1e-5f && std::fabs(rtlat - kPi / 2) < 1e-5f, "reset from the pole");
}

int main() {
  test_atan2_accuracy();
  test_fast_scalar_ops();
  test_centre_shows_view_centre();
  test_up_is_north();
  test_texel_corners();
  test_fast_matches_ref_over_disc();
  test_accumulated_lon_wraps();
  test_row_span_matches_pixel_test();
  test_adaptive_rows_stay_within_one_texel();
  test_adaptive_single_pixel_rows();
  test_quadratic_rows_stay_within_one_pixel();
  test_tiled_layouts_render_the_same_picture();
  test_drag_goes_over_the_pole_to_upside_down();
  test_drag_right_brings_the_west();
  test_spin_equals_turning_longitude();
  test_many_drags_stay_a_rotation();
  test_equator_reset_keeps_the_facing_longitude();
  if (g_failures) {
    std::printf("%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
