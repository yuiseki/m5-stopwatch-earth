// How far does the adaptive interpolating renderer drift from the exact one, and what does it cost?
//
//   make build/interp_eval && ./build/interp_eval
//
// The "texture" holds each texel's own index, so a rendered pixel says which texel it took.
// Views are drawn uniformly at random over the sphere. For every (seg, tol) pair this prints
// the share of pixels that took a different texel, the share more than one texel away, the
// worst distance, and exact evaluations per pixel (the cost; the exact renderer is 1.0).
#include "globe_math.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace globe;
constexpr int TW = 1024, TH = 512, N = 468;

int main() {
  std::vector<uint32_t> tex(TW * TH);
  for (uint32_t k = 0; k < tex.size(); ++k) tex[k] = k;
  const Disc disc = Disc::make(N, N);

  std::mt19937 rng(20261106);
  std::uniform_real_distribution<float> uni(0.0f, 1.0f);
  std::vector<View> views;
  for (int k = 0; k < 40; ++k) {
    const float lat = std::asin(2.0f * uni(rng) - 1.0f);
    const float lon = (2.0f * uni(rng) - 1.0f) * kPi;
    views.push_back(View::make(lat, lon));
  }
  // Plus the hard cases on purpose: looking straight at a pole.
  views.push_back(View::make(89.0f * kPi / 180, 0.3f));
  views.push_back(View::make(-89.0f * kPi / 180, 2.0f));

  std::vector<uint32_t> exact(N), approx(N);
  std::printf(" seg   tol  differ%%  >1texel%%  worst  evals/px\n");
  for (int seg : {8, 16, 32, 64}) {
    for (float tol : {0.25f, 0.5f, 1.0f}) {
      long px = 0, differ = 0, far = 0, evals = 0;
      int worst = 0;
      for (const View& v : views) {
        for (int j = 0; j < N; ++j) {
          int i0, i1;
          if (!disc.row_span(j, N, &i0, &i1)) continue;
          render_row_exact<TW, TH>(v, disc, j, i0, i1, tex.data(), exact.data());
          evals += render_row_adaptive<TW, TH>(v, disc, j, i0, i1, seg, tol, tex.data(), approx.data());
          for (int i = i0; i <= i1; ++i) {
            ++px;
            if (exact[i] == approx[i]) continue;
            ++differ;
            int du = std::abs(int(exact[i] % TW) - int(approx[i] % TW));
            if (du > TW / 2) du = TW - du;
            const int dv = std::abs(int(exact[i] / TW) - int(approx[i] / TW));
            const int dd = du > dv ? du : dv;
            if (dd > 1) ++far;
            if (dd > worst) worst = dd;
          }
        }
      }
      std::printf("%4d %5.2f  %7.3f  %8.4f  %5d  %8.3f\n", seg, tol, 100.0 * differ / px, 100.0 * far / px, worst,
                  double(evals) / px);
    }
  }

  // Quadratic rows. Error in screen pixels: texel error divided by the local squeeze (texels per
  // pixel, from the exact neighbours), floored at one texel per pixel.
  std::printf("\nquadratic: seg   tol  differ%%  >1texel%%  worst_tex  worst_px  >1px%%  evals/px\n");
  for (int seg : {32, 64, 128}) {
    for (float tol : {0.25f, 0.5f}) {
      long px = 0, differ = 0, far = 0, evals = 0, far_px = 0;
      int worst = 0;
      float worst_px = 0;
      for (const View& v : views) {
        for (int j = 0; j < N; ++j) {
          int i0, i1;
          if (!disc.row_span(j, N, &i0, &i1)) continue;
          render_row_exact<TW, TH>(v, disc, j, i0, i1, tex.data(), exact.data());
          evals += render_row_quadratic<TW, TH>(v, disc, j, i0, i1, seg, tol, tex.data(), approx.data());
          auto dist = [](uint32_t a, uint32_t b) {
            int du = std::abs(int(a % TW) - int(b % TW));
            if (du > TW / 2) du = TW - du;
            const int dv = std::abs(int(a / TW) - int(b / TW));
            return du > dv ? du : dv;
          };
          for (int i = i0; i <= i1; ++i) {
            ++px;
            if (exact[i] == approx[i]) continue;
            ++differ;
            const int dd = dist(exact[i], approx[i]);
            if (dd > 1) ++far;
            if (dd > worst) worst = dd;
            int sq = 1;
            if (i > i0) sq = std::max(sq, dist(exact[i], exact[i - 1]));
            if (i < i1) sq = std::max(sq, dist(exact[i], exact[i + 1]));
            const float e_px = float(dd) / float(sq);
            if (e_px > 1.0f) ++far_px;
            if (e_px > worst_px) worst_px = e_px;
          }
        }
      }
      std::printf("           %4d %5.2f  %7.3f  %8.4f  %9d  %8.2f  %5.3f  %8.3f\n", seg, tol, 100.0 * differ / px,
                  100.0 * far / px, worst, worst_px, 100.0 * far_px / px, double(evals) / px);
    }
  }
  return 0;
}
