// Orthographic globe: map a screen pixel to a texel of an equirectangular texture.
//
// Header-only and free of Arduino, so the same code runs on the device and in the
// host tests (test/test_globe_math.cpp) and the host renderer (tools/host_render.cpp).
//
// Frames
//   view  : x to the right, y up, z towards the viewer. The visible hemisphere is z >= 0.
//   world : the globe after rotating by the view centre (lat0, lon0). The pixel at the
//           disc centre shows (lat0, lon0).
//
// Texture
//   Equirectangular, width TW (a power of two) and height TH. Column 0 starts at
//   longitude -180 deg, row 0 at latitude +90 deg.
#pragma once

#include <cmath>
#include <cstdint>

namespace globe {

constexpr float kPi = 3.14159265358979323846f;

// atan(t) for t in [0, 1]. Minimax odd polynomial, max error about 1e-5 rad,
// which is 0.002 texel on a 1024-wide texture.
inline float atan01(float t) {
  const float t2 = t * t;
  return t * (0.99997726f +
              t2 * (-0.33262347f + t2 * (0.19354346f + t2 * (-0.11643287f + t2 * (0.05265332f + t2 * -0.01172120f)))));
}

// atan2(y, x) built on atan01. Returns 0 for (0, 0) instead of NaN.
inline float fast_atan2(float y, float x) {
  const float ax = std::fabs(x);
  const float ay = std::fabs(y);
  const float mx = ax > ay ? ax : ay;
  const float mn = ax > ay ? ay : ax;
  if (mx == 0.0f) return 0.0f;
  float a = atan01(mn / mx);
  if (ay > ax) a = 0.5f * kPi - a;
  if (x < 0.0f) a = kPi - a;
  return y < 0.0f ? -a : a;
}

// The view centre, with its sine and cosine cached once per frame.
struct View {
  float lat0;  // radians
  float lon0;  // radians
  float sin_lat0;
  float cos_lat0;

  static View make(float lat0, float lon0) {
    return View{lat0, lon0, std::sin(lat0), std::cos(lat0)};
  }
};

// Unit-sphere point in the view frame -> (lat, lon) in radians, lon in (-pi, pi].
// Uses libm. This is the reference the fast path is tested against.
inline void view_to_latlon_ref(const View& v, float x, float y, float z, float* lat, float* lon) {
  const float wx = x;
  const float wy = y * v.cos_lat0 + z * v.sin_lat0;
  const float wz = z * v.cos_lat0 - y * v.sin_lat0;
  *lat = std::atan2(wy, std::sqrt(wx * wx + wz * wz));
  float l = v.lon0 + std::atan2(wx, wz);
  while (l > kPi) l -= 2.0f * kPi;
  while (l <= -kPi) l += 2.0f * kPi;
  *lon = l;
}

// Texel index for a (lat, lon). The column wraps, the row clamps.
template <int TW, int TH>
inline uint32_t latlon_to_texel(float lat, float lon) {
  static_assert((TW & (TW - 1)) == 0, "texture width must be a power of two");
  const float u = (lon + kPi) * (TW / (2.0f * kPi));
  const float vv = (0.5f * kPi - lat) * (TH / kPi);
  // floor, then wrap; u can be slightly negative or >= TW after rounding.
  int iu = static_cast<int>(std::floor(u)) & (TW - 1);
  int iv = static_cast<int>(vv);
  if (iv < 0) iv = 0;
  if (iv > TH - 1) iv = TH - 1;
  return static_cast<uint32_t>(iv) * TW + static_cast<uint32_t>(iu);
}

// The per-pixel hot path: view-frame point -> texel index, with fast_atan2.
// Longitude is carried in turns-free radians and wrapped by the power-of-two mask,
// so lon0 may be any value (it accumulates while dragging).
template <int TW, int TH>
inline uint32_t view_to_texel_fast(const View& v, float x, float y, float z) {
  const float wx = x;
  const float wy = y * v.cos_lat0 + z * v.sin_lat0;
  const float wz = z * v.cos_lat0 - y * v.sin_lat0;
  const float horiz = std::sqrt(wx * wx + wz * wz);
  const float lat = fast_atan2(wy, horiz);
  const float lon = v.lon0 + fast_atan2(wx, wz);
  const float u = (lon + kPi) * (TW / (2.0f * kPi));
  const float vv = (0.5f * kPi - lat) * (TH / kPi);
  // u may be far outside [0, TW) when lon0 has accumulated; floor then mask keeps it right
  // for |u| < 2^24, which is about 16000 turns of the globe.
  int iu = static_cast<int>(std::floor(u)) & (TW - 1);
  int iv = static_cast<int>(vv);
  if (iv < 0) iv = 0;
  if (iv > TH - 1) iv = TH - 1;
  return static_cast<uint32_t>(iv) * TW + static_cast<uint32_t>(iu);
}

// Screen geometry: a disc of radius r centred at (cx, cy) in pixel coordinates.
// Pixel (i, j) samples at its centre.
struct Disc {
  float cx, cy, r, inv_r;
  static Disc make(int width, int height) {
    const float r = 0.5f * static_cast<float>(width < height ? width : height);
    return Disc{0.5f * width, 0.5f * height, r, 1.0f / r};
  }
  // Fills x, y (view frame) for pixel (i, j). Returns false outside the disc.
  bool pixel_to_view(int i, int j, float* x, float* y, float* z) const {
    const float px = (static_cast<float>(i) + 0.5f - cx) * inv_r;
    const float py = (cy - (static_cast<float>(j) + 0.5f)) * inv_r;
    const float r2 = px * px + py * py;
    if (r2 > 1.0f) return false;
    *x = px;
    *y = py;
    *z = std::sqrt(1.0f - r2);
    return true;
  }
  // First and last pixel column inside the disc on row j, or false if the row misses it.
  bool row_span(int j, int width, int* i0, int* i1) const {
    const float py = (cy - (static_cast<float>(j) + 0.5f)) * inv_r;
    const float rem = 1.0f - py * py;
    if (rem < 0.0f) return false;
    const float half = std::sqrt(rem) * r;  // in pixels
    int a = static_cast<int>(std::ceil(cx - half - 0.5f));
    int b = static_cast<int>(std::floor(cx + half - 0.5f));
    if (a < 0) a = 0;
    if (b > width - 1) b = width - 1;
    if (a > b) return false;
    *i0 = a;
    *i1 = b;
    return true;
  }
};

}  // namespace globe
