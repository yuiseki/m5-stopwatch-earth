# m5-stopwatch-earth

NASA Blue Marble on the round 1.75" AMOLED of the M5Stack StopWatch (ESP32-S3R8, CO5300, 468x468
as M5GFX reports it), as an orthographic globe drawn by a software renderer. Drag to turn it.

The texture is a 1024x512 equirectangular RGB565 image embedded in the app image in flash
(`data/earth_1024x512.rgb565be`, 1 MiB). Every screen pixel inside the disc is mapped back to a
latitude and longitude and takes its texel.

## Build, flash, measure

    make test                       # host tests of the projection math (include/globe_math.h)
    tools/flash.sh                  # build, copy to pi4-deck, write over USB (port pinned by MAC)
    LOG_SECS=15 tools/flash.sh      # ... and print the serial output for 15 s after the reset
    ENV=diag tools/flash.sh         # colour test of the display write paths

`tools/serial_log.py PORT SECONDS` reads the serial port without resetting the board (see below).

The firmware prints `BENCH` lines once at boot and a `STAT` line every 2 s:
fps, render time, time blocked on the display transfer, touch read time, and the delay from a
touch sample to the end of the frame that shows it.

- Spin mode (at boot): the globe turns by 1 deg per frame and touch is not read over I2C.
- Touch mode: entered on the first contact (the touch controller's INT line, G13). Dragging
  turns the globe like a trackball, with no limit: over a pole and on to upside down. The
  orientation is a rotation matrix (`globe::View`), not a latitude and longitude. The left button (yellow) goes back to spin.
- The left button (yellow) also turns the globe north up with the equator at the centre,
  keeping the longitude that faces the viewer.
- The right button (blue) switches between rendering on one core and on both.

## How it renders

- Strips of 16 rows go to two DMA buffers in internal RAM. While one strip is sent, the next is
  rendered. Core 1 renders the upper half of each strip, a worker task on core 0 the lower half.
- Each row is rendered by `globe::render_row_quadratic`. The exact mapping (two atan2 and a
  square root) runs only at the ends, midpoint and quarter points of 64-pixel segments. A
  parabola through those points fills the segment by forward differences. A segment that fails
  the check is halved. The tolerance is 0.25 px on screen: towards the limb, where many texels
  share a pixel, a larger texel error is allowed. The host tests hold every view within one
  pixel of the exact mapping. About 0.16 exact evaluations per pixel.
- sqrt, reciprocal and floor are computed with multiplies only (bit-level estimate plus Newton
  steps). On this toolchain `sqrtf`, float division and `floorf` are library calls.

## Results (2026-11-06, view at latitude 20 deg)

| Step | Spin fps | Render ms/frame |
|---|---:|---:|
| First cut: exact mapping, one core, through M5.Display | 2.42 | 391 |
| Adaptive linear interpolation along rows | 7.65 | 108 |
| Two cores | 10.9 | 70 |
| Strips sent straight to the panel (see below) | 14.4 | 69 |
| Multiply-only sqrt / reciprocal / floor | 17.8 | 55 |
| Quadratic rows, screen-space tolerance | 19.7 | 50 |
| Texture copied to PSRAM in 4x4 tiles | 20.5 | 48 |

With a finger dragging, after the tiled texture: 16.1-20.3 fps, touch read 0.42 ms, touch sample
to frame done 48-62 ms, also while dragging around both poles. (Before the tiled texture,
looking near a pole dipped to 13 fps and 123 ms.) This latency starts at the touch *sample*:
the time from the finger to the sample (up to a frame) and the panel's own refresh come on top
and are not measured.

Cycles per pixel by view latitude and texture layout (one core, quadratic rows):

| View latitude | Flash, row-major | PSRAM, row-major | PSRAM, 4x4 tiles | 8x8 | 16x16 |
|---|---:|---:|---:|---:|---:|
| 0 | 100.5 | 96.0 | 105.9 | 105.9 | 105.7 |
| 45 | 130.9 | 111.5 | 113.8 | 114.4 | 114.5 |
| 70 | 201.6 | 143.4 | 122.9 | 127.6 | 129.5 |
| 89 | 223.0 | 152.2 | 126.8 | 130.2 | 132.3 |

The number of exact evaluations hardly changes with latitude (0.147 to 0.169 per pixel), so
the extra cost is texture reads: near a pole one screen row walks down the texture as much as
across it, and row-major reads miss the cache on almost every pixel. The renderer now reads a
copy in PSRAM laid out in 4x4 tiles (`globe::texel_addr`), built from the flash texture at boot;
the worst case drops from 223 to 127 cycles for 5% more at the equator.

## Things found on the way

- On the StopWatch, `M5.Display` is a PSRAM framebuffer (`Panel_AMOLED_Framebuffer`) in front of
  the CO5300. `pushImageDMA` into it is a memcpy, and every `endWrite()` sends all 438 KB to the
  panel, blocking for about 22 ms. The renderer reaches the panel underneath (a protected member,
  read through a member pointer) and gives it to its own `lgfx::LGFX_Device`.
- That device is never `init()`ed (the panel already is), so its clip rectangle stays empty and
  every write is silently clipped away until `clearClipRect()`.
- A 16-row strip (15 KB) goes out in about 390 us, about 38 MB/s on the 80 MHz QSPI bus.
- `std::memcpy` used for float bit casts was not inlined by this toolchain and became 29 real
  calls in the hot loop, which made the multiply-only version slower until a union replaced it.
- Moving the hot function to IRAM made no difference.
- The ESP32-S3 here talks through its built-in USB-Serial/JTAG, where DTR/RTS drive the reset.
  On Linux, cdc_acm raises both whenever the port is opened at a non-zero baud rate, so every
  serial capture reset the chip (`rst:0x15 USB_UART_CHIP_RESET`). Clearing them from pyserial
  after the open is too late. `serial_log.py` sets the port to baud 0, raw and `-hupcl` and
  reads it with `os.read`; opening it then leaves the chip alone.
- Twice the screen stayed black after such a reset, through every write path including the
  framebuffer, while the firmware ran normally; the power button brought it back. It happened
  only with the direct-DMA renderer, which keeps a panel write in flight about a fifth of the
  time, so the working explanation is a reset landing mid-write and leaving the panel in a state
  that M5GFX's 8 ms reset pulse does not clear. The firmware now switches the panel supply
  (M5IOE1 PYG8) and reset (PYG5) off for 200 ms before `M5.begin()`. Once so far, that brought
  a black panel back without the power button; it has not been shown to prevent every case.

## Texture

`data/earth_1024x512.rgb565be` is made by `tools/make_texture.py` from NASA's Blue Marble
(Land Surface, Shallow Water, and Shaded Topography, 2002), public domain, taken as the 3840 px
rendition of `File:Blue_Marble_2002.png` on Wikimedia Commons (tagged PD-USGov-NASA there).
Image credit: NASA Goddard Space Flight Center, Reto Stöckli (land surface, shallow water,
clouds) and Robert Simmon (ocean colour, compositing), NASA Visible Earth.

## Licence

The code is under the MIT licence (`LICENSE`). The texture in `data/` is a work of NASA and in
the public domain; the MIT licence does not apply to it.

