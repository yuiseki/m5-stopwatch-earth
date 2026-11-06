// Render the globe on the host with the firmware's own math, to look at before flashing.
//
//   make build/host_render
//   ./build/host_render data/earth_1024x512.rgb565be LAT LON SIZE out.ppm
#include "globe_math.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

constexpr int TW = 1024, TH = 512;

int main(int argc, char** argv) {
  if (argc != 6) {
    std::fprintf(stderr, "usage: %s texture.rgb565be lat lon size out.ppm\n", argv[0]);
    return 2;
  }
  std::vector<uint16_t> tex(TW * TH);
  FILE* f = std::fopen(argv[1], "rb");
  if (!f || std::fread(tex.data(), 2, tex.size(), f) != tex.size()) {
    std::fprintf(stderr, "cannot read %d texels from %s\n", TW * TH, argv[1]);
    return 1;
  }
  std::fclose(f);
  const float lat = std::atof(argv[2]) * globe::kPi / 180.0f;
  const float lon = std::atof(argv[3]) * globe::kPi / 180.0f;
  const int n = std::atoi(argv[4]);

  const globe::View v = globe::View::make(lat, lon);
  const globe::Disc disc = globe::Disc::make(n, n);
  FILE* o = std::fopen(argv[5], "wb");
  std::fprintf(o, "P6\n%d %d\n255\n", n, n);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      unsigned char rgb[3] = {0, 0, 0};
      float x, y, z;
      if (disc.pixel_to_view(i, j, &x, &y, &z)) {
        const uint16_t be = tex[globe::view_to_texel_fast<TW, TH>(v, x, y, z)];
        const uint16_t c = static_cast<uint16_t>((be >> 8) | (be << 8));  // file is big-endian
        rgb[0] = ((c >> 11) & 31) * 255 / 31;
        rgb[1] = ((c >> 5) & 63) * 255 / 63;
        rgb[2] = (c & 31) * 255 / 31;
      }
      std::fwrite(rgb, 1, 3, o);
    }
  std::fclose(o);
  return 0;
}
