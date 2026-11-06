#!/usr/bin/env python3
"""Turn an equirectangular image into the RGB565 texture the firmware embeds.

    python3 -I tools/make_texture.py SOURCE.png data/earth_1024x512.rgb565be [--width 1024]

Output is raw big-endian RGB565 (the byte order the panel takes, so the firmware can hand
texels to DMA without swapping), row 0 = latitude +90, column 0 = longitude -180,
exactly width*height*2 bytes. A PNG decoded back from the RGB565 is written next to it
(.preview.png) so the quantisation can be looked at.
"""
import argparse
import sys

from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("source")
    ap.add_argument("out")
    ap.add_argument("--width", type=int, default=1024)
    a = ap.parse_args()

    w = a.width
    h = w // 2
    if w & (w - 1):
        sys.exit("width must be a power of two")

    img = Image.open(a.source).convert("RGB")
    if abs(img.width - 2 * img.height) > 2:
        sys.exit(f"source is {img.width}x{img.height}, not 2:1 equirectangular")
    img = img.resize((w, h), Image.LANCZOS)

    raw = bytearray(w * h * 2)
    preview = Image.new("RGB", (w, h))
    px = img.load()
    pv = preview.load()
    k = 0
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y]
            v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            raw[k] = v >> 8
            raw[k + 1] = v & 0xFF
            k += 2
            pv[x, y] = (((v >> 11) & 31) * 255 // 31, ((v >> 5) & 63) * 255 // 63, (v & 31) * 255 // 31)

    with open(a.out, "wb") as f:
        f.write(raw)
    preview.save(a.out + ".preview.png")
    print(f"wrote {a.out}: {w}x{h}, {len(raw)} bytes")


if __name__ == "__main__":
    main()
