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

// Square root, reciprocal and floor from multiplies only.
//
// On the ESP32-S3, GCC turns sqrtf, float division and floorf into library calls (sqrtf
// handles errno, division is soft-float __divsf3), and those calls were most of the 550 cycles
// an exact pixel cost. These versions start from a bit-level estimate and refine it with
// Newton steps, so they compile to FPU multiply-adds on the device.

// Bit reinterpretation through a union (defined behaviour in GCC and Clang). std::memcpy was
// not inlined by the ESP32-S3 toolchain and became a real call in the per-pixel loop.
union FloatBits {
  float f;
  uint32_t u;
};

// 1/sqrt(x) for x > 0. Relative error below 1e-5 after two Newton steps (measured 4.8e-6;
// float rounding in the steps keeps it above the 1e-9 the iteration alone would reach).
inline float fast_rsqrt(float x) {
  FloatBits b;
  b.f = x;
  b.u = 0x5f375a86u - (b.u >> 1);
  float y = b.f;
  const float hx = 0.5f * x;
  y = y * (1.5f - hx * y * y);
  y = y * (1.5f - hx * y * y);
  return y;
}

// sqrt(x) for x >= 0 (0 for x <= 0).
inline float fast_sqrt(float x) { return x > 0.0f ? x * fast_rsqrt(x) : 0.0f; }

// 1/x for x > 0. Relative error below 1e-7 after three Newton steps.
inline float fast_recip(float x) {
  FloatBits b;
  b.f = x;
  b.u = 0x7ef311c7u - b.u;
  float y = b.f;
  y = y * (2.0f - x * y);
  y = y * (2.0f - x * y);
  y = y * (2.0f - x * y);
  return y;
}

// floor(x) for |x| < 2^31.
inline int fast_floor(float x) {
  const int i = static_cast<int>(x);
  return i - (x < static_cast<float>(i) ? 1 : 0);
}

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
  float a = atan01(mn * fast_recip(mx));
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
  const float horiz = fast_sqrt(wx * wx + wz * wz);
  const float lat = fast_atan2(wy, horiz);
  const float lon = v.lon0 + fast_atan2(wx, wz);
  const float u = (lon + kPi) * (TW / (2.0f * kPi));
  const float vv = (0.5f * kPi - lat) * (TH / kPi);
  // u may be far outside [0, TW) when lon0 has accumulated; floor then mask keeps it right
  // for |u| < 2^24, which is about 16000 turns of the globe.
  int iu = fast_floor(u) & (TW - 1);
  int iv = static_cast<int>(vv);
  if (iv < 0) iv = 0;
  if (iv > TH - 1) iv = TH - 1;
  return static_cast<uint32_t>(iv) * TW + static_cast<uint32_t>(iu);
}

// Continuous texel coordinates for a view-frame point: u in [0, TW) (wrapped), v in [0, TH].
// Column iu = floor(u), row iv = min(floor(v), TH - 1) reproduce view_to_texel_fast.
template <int TW, int TH>
inline void view_to_uv(const View& v, float x, float y, float z, float* u, float* vv) {
  const float wx = x;
  const float wy = y * v.cos_lat0 + z * v.sin_lat0;
  const float wz = z * v.cos_lat0 - y * v.sin_lat0;
  const float horiz = fast_sqrt(wx * wx + wz * wz);
  const float lat = fast_atan2(wy, horiz);
  const float lon = v.lon0 + fast_atan2(wx, wz);
  float uu = (lon + kPi) * (TW / (2.0f * kPi));
  uu -= TW * static_cast<float>(fast_floor(uu * (1.0f / TW)));
  if (uu >= TW) uu -= TW;  // rounding can land exactly on TW
  *u = uu;
  *vv = (0.5f * kPi - lat) * (TH / kPi);
}

// Texel index from the continuous coordinates of view_to_uv.
template <int TW, int TH>
inline uint32_t latlon_texel_from_uv(float u, float vv) {
  const int iu = static_cast<int>(u) & (TW - 1);
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

// One row by adaptive subdivision, pixels [i0, i1] of row j, written to row[i0..i1].
//
// The row is cut into segments of `seg` pixels (a power of two). For each segment the exact
// mapping is evaluated at both ends and at the midpoint. If the midpoint lies within `tol`
// texels of the straight line between the ends, the two halves are filled by linear
// interpolation (16.16 fixed point) through the three exact points; otherwise each half is
// handled the same way. Halves of two pixels or fewer are computed exactly.
//
// The midpoint test measures how much the mapping bends, which is what makes linear
// interpolation wrong; it bends near the poles and along the limb. How fast u and v move does
// not matter: they move about 0.7 texel per pixel at the disc centre already.
//
// Returns the number of exact evaluations, for cost accounting.
template <int TW, int TH, typename Pixel>
inline int render_row_adaptive(const View& v, const Disc& d, int j, int i0, int i1, int seg, float tol,
                               const Pixel* tex, Pixel* row) {
  const float py = (d.cy - (static_cast<float>(j) + 0.5f)) * d.inv_r;
  const float py2 = py * py;
  int evals = 0;
  struct UV {
    float u, v;
  };
  auto exact = [&](int i) {
    const float px = (static_cast<float>(i) + 0.5f - d.cx) * d.inv_r;
    float rem = 1.0f - px * px - py2;
    if (rem < 0.0f) rem = 0.0f;  // span edge rounding
    UV r;
    view_to_uv<TW, TH>(v, px, py, fast_sqrt(rem), &r.u, &r.v);
    ++evals;
    return r;
  };
  // u difference taken the short way round the texture.
  auto wrap = [](float du) {
    if (du > 0.5f * TW) du -= TW;
    if (du < -0.5f * TW) du += TW;
    return du;
  };
  auto put = [&](int i, const UV& p) { row[i] = tex[latlon_texel_from_uv<TW, TH>(p.u, p.v)]; };
  // Linear fill of [a, b) from p (at a) towards q (at b).
  auto lerp_fill = [&](int a, int b, const UV& p, const UV& q) {
    const float inv = fast_recip(static_cast<float>(b - a));
    int32_t U = static_cast<int32_t>(p.u * 65536.0f);
    int32_t V = static_cast<int32_t>(p.v * 65536.0f);
    const int32_t dU = static_cast<int32_t>(wrap(q.u - p.u) * inv * 65536.0f);
    const int32_t dV = static_cast<int32_t>((q.v - p.v) * inv * 65536.0f);
    for (int i = a; i < b; ++i) {
      const int iu = (U >> 16) & (TW - 1);
      int iv = V >> 16;
      if (iv > TH - 1) iv = TH - 1;
      if (iv < 0) iv = 0;
      row[i] = tex[iv * TW + iu];
      U += dU;
      V += dV;
    }
  };
  // Fills [a, b) given exact values at a and b. Explicit stack: depth is log2(seg).
  struct Job {
    int a, b;
    UV p, q;
  };
  Job stack[24];
  UV pa = exact(i0);
  for (int a = i0; a < i1;) {
    const int b = (a + seg < i1) ? a + seg : i1;
    const UV pb = exact(b);
    int sp = 0;
    stack[sp++] = Job{a, b, pa, pb};
    while (sp) {
      const Job job = stack[--sp];
      const int n = job.b - job.a;
      if (n <= 2) {
        put(job.a, job.p);
        if (n == 2) put(job.a + 1, exact(job.a + 1));
        continue;
      }
      const int m = (job.a + job.b) / 2;
      const UV pm = exact(m);
      const float t = static_cast<float>(m - job.a) * fast_recip(static_cast<float>(n));
      const float eu = std::fabs(wrap(pm.u - job.p.u) - t * wrap(job.q.u - job.p.u));
      const float ev = std::fabs((pm.v - job.p.v) - t * (job.q.v - job.p.v));
      if (eu <= tol && ev <= tol) {
        lerp_fill(job.a, m, job.p, pm);
        lerp_fill(m, job.b, pm, job.q);
      } else {
        stack[sp++] = Job{m, job.b, pm, job.q};
        stack[sp++] = Job{job.a, m, job.p, pm};
      }
    }
    pa = pb;
    a = b;
  }
  put(i1, pa);
  return evals;
}

// One row of the disc, pixels [i0, i1] of row j, written to row[i0..i1].
//
// The exact mapping (view_to_texel_fast) costs two atan2, two sqrt and a division per pixel.
// Here it is evaluated only at anchors every `step` pixels; between anchors the texel
// coordinates are interpolated linearly in 16.16 fixed point. A segment is computed exactly
// instead when u or v moves faster than `max_rate` texels per pixel, which is where the
// mapping bends hard: around the poles (longitude sweeps) and along the limb.
//
// step = 1 is the exact renderer.
template <int TW, int TH, typename Pixel>
inline void render_row(const View& v, const Disc& d, int j, int i0, int i1, int step, float max_rate,
                       const Pixel* tex, Pixel* row) {
  const float py = (d.cy - (static_cast<float>(j) + 0.5f)) * d.inv_r;
  const float py2 = py * py;
  auto view_z = [&](int i, float* px) {
    *px = (static_cast<float>(i) + 0.5f - d.cx) * d.inv_r;
    float rem = 1.0f - *px * *px - py2;
    if (rem < 0.0f) rem = 0.0f;  // span edge rounding
    return fast_sqrt(rem);
  };
  if (step <= 1) {
    for (int i = i0; i <= i1; ++i) {
      float px;
      const float pz = view_z(i, &px);
      row[i] = tex[view_to_texel_fast<TW, TH>(v, px, py, pz)];
    }
    return;
  }
  float px;
  float ua, va;
  {
    const float pz = view_z(i0, &px);
    view_to_uv<TW, TH>(v, px, py, pz, &ua, &va);
  }
  for (int a = i0; a <= i1;) {
    int b = a + step;
    if (b > i1) b = i1;
    const int len = b - a;
    if (len == 0) {  // single pixel row, or the last pixel
      row[a] = tex[latlon_texel_from_uv<TW, TH>(ua, va)];
      break;
    }
    float ub, vb;
    {
      const float pz = view_z(b, &px);
      view_to_uv<TW, TH>(v, px, py, pz, &ub, &vb);
    }
    float du = ub - ua;
    if (du > 0.5f * TW) du -= TW;
    if (du < -0.5f * TW) du += TW;
    const float dv = vb - va;
    const float inv_len = 1.0f / static_cast<float>(len);
    const float rate_u = std::fabs(du) * inv_len;
    const float rate_v = std::fabs(dv) * inv_len;
    if (rate_u > max_rate || rate_v > max_rate) {
      for (int i = a; i < b; ++i) {
        const float pz = view_z(i, &px);
        row[i] = tex[view_to_texel_fast<TW, TH>(v, px, py, pz)];
      }
    } else {
      int32_t U = static_cast<int32_t>(ua * 65536.0f);
      int32_t V = static_cast<int32_t>(va * 65536.0f);
      const int32_t dU = static_cast<int32_t>(du * inv_len * 65536.0f);
      const int32_t dV = static_cast<int32_t>(dv * inv_len * 65536.0f);
      for (int i = a; i < b; ++i) {
        const int iu = (U >> 16) & (TW - 1);
        int iv = V >> 16;
        if (iv > TH - 1) iv = TH - 1;
        if (iv < 0) iv = 0;
        row[i] = tex[iv * TW + iu];
        U += dU;
        V += dV;
      }
    }
    if (b == i1) {
      row[b] = tex[latlon_texel_from_uv<TW, TH>(ub, vb)];
      break;
    }
    ua = ub;
    va = vb;
    a = b;
  }
}

}  // namespace globe
