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
  turns the globe. The left button (yellow) goes back to spin.
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

With a finger dragging: 19.7 fps, touch read 0.43 ms, touch sample to frame done 46-56 ms
(one dip to 13 fps, max 123 ms, while looking near a pole). This latency starts at the touch
*sample*: the time from the finger to the sample (up to a frame) and the panel's own refresh
come on top and are not measured.

Cost per pixel by view latitude (one core, quadratic rows):

| View latitude | Texture in flash | Copy in PSRAM |
|---|---:|---:|
| 0 | 99.5 cycles | 94.8 |
| 45 | 129.9 | 110.3 |
| 70 | 200.8 | 142.2 |
| 89 | 222.3 | 151.0 |

The number of exact evaluations hardly changes with latitude (0.147 to 0.169 per pixel), so
the extra cost is texture reads: near a pole one screen row crosses many texture rows.

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
- The ESP32-S3 here talks through its built-in USB-Serial/JTAG. pyserial raises DTR/RTS when it
  opens a port, which can leave the chip in ROM download mode, so `serial_log.py` clears both
  lines before opening.
- Once, the screen stayed black (every write path, the framebuffer included) until the power
  button restarted the board. The cause is not known.

## Texture

`data/earth_1024x512.rgb565be` is made by `tools/make_texture.py` from NASA's Blue Marble
(Land Surface, Shallow Water, and Shaded Topography, 2002), public domain, taken as the 3840 px
rendition of `File:Blue_Marble_2002.png` on Wikimedia Commons.
