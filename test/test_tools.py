"""Host tests for the Python tools. Run with `make test` (python3 -I test/test_tools.py)."""
import os
import pty
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAKE_TEXTURE = os.path.join(ROOT, "tools", "make_texture.py")
SERIAL_LOG = os.path.join(ROOT, "tools", "serial_log.py")


def run(*args):
    return subprocess.run([sys.executable, "-I", *args], capture_output=True, text=True, timeout=60)


class MakeTextureTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def path(self, name):
        return os.path.join(self.dir.name, name)

    def source(self, w, h, fill):
        p = self.path(f"src_{w}x{h}.png")
        img = Image.new("RGB", (w, h))
        img.putdata([fill(x, y) for y in range(h) for x in range(w)])
        img.save(p)
        return p

    def test_packs_big_endian_rgb565_row_0_north_column_0_west(self):
        # Four quadrants of flat colour, at the output size, so resizing keeps them exact.
        def fill(x, y):
            if y < 4:
                return (255, 0, 0) if x < 8 else (0, 255, 0)
            return (0, 0, 255) if x < 8 else (255, 255, 255)

        src = self.source(16, 8, fill)
        out = self.path("t.rgb565be")
        r = run(MAKE_TEXTURE, src, out, "--width", "16")
        self.assertEqual(r.returncode, 0, r.stderr)
        with open(out, "rb") as f:
            data = f.read()
        self.assertEqual(len(data), 16 * 8 * 2)

        def texel(x, y):
            k = (y * 16 + x) * 2
            return data[k] << 8 | data[k + 1]

        self.assertEqual(texel(0, 0), 0xF800)    # red, north-west
        self.assertEqual(texel(15, 0), 0x07E0)   # green, north-east
        self.assertEqual(texel(0, 7), 0x001F)    # blue, south-west
        self.assertEqual(texel(15, 7), 0xFFFF)   # white, south-east
        self.assertEqual(data[0:2], b"\xf8\x00", "big-endian: high byte first")
        self.assertTrue(os.path.exists(out + ".preview.png"))

    def test_preview_decodes_the_quantised_colours(self):
        src = self.source(16, 8, lambda x, y: (200, 100, 50))
        out = self.path("t.rgb565be")
        self.assertEqual(run(MAKE_TEXTURE, src, out, "--width", "16").returncode, 0)
        r, g, b = Image.open(out + ".preview.png").getpixel((3, 3))
        # 200 -> 25/31, 100 -> 25/63, 50 -> 6/31, scaled back to 0..255.
        self.assertEqual((r, g, b), (25 * 255 // 31, 25 * 255 // 63, 6 * 255 // 31))

    def test_resizes_to_the_requested_width(self):
        src = self.source(64, 32, lambda x, y: (x * 4, y * 8, 0))
        out = self.path("t.rgb565be")
        self.assertEqual(run(MAKE_TEXTURE, src, out, "--width", "32").returncode, 0)
        self.assertEqual(os.path.getsize(out), 32 * 16 * 2)

    def test_rejects_a_source_that_is_not_2_to_1(self):
        src = self.source(30, 10, lambda x, y: (0, 0, 0))
        r = run(MAKE_TEXTURE, src, self.path("t.rgb565be"), "--width", "16")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("not 2:1", r.stderr)
        self.assertFalse(os.path.exists(self.path("t.rgb565be")))

    def test_rejects_a_width_that_is_not_a_power_of_two(self):
        src = self.source(48, 24, lambda x, y: (0, 0, 0))
        r = run(MAKE_TEXTURE, src, self.path("t.rgb565be"), "--width", "48")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("power of two", r.stderr)


class SerialLogTest(unittest.TestCase):
    """serial_log.py against a pseudo-terminal standing in for the USB serial port."""

    def test_prints_complete_lines_with_a_timestamp_and_the_tail_at_the_end(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        port = os.ttyname(slave)

        def feed():
            time.sleep(0.4)
            os.write(master, b"STAT fps=19.5\r\nBOOT tex")
            time.sleep(0.2)
            os.write(master, b"ture=psram_tile4\nno newline")

        t = threading.Thread(target=feed)
        t.start()
        r = run(SERIAL_LOG, port, "1.5")
        t.join()
        self.assertEqual(r.returncode, 0, r.stderr)
        lines = r.stdout.splitlines()
        self.assertEqual(len(lines), 3, r.stdout)
        self.assertRegex(lines[0], r"^\d\d:\d\d:\d\d STAT fps=19\.5$")
        self.assertRegex(lines[1], r"^\d\d:\d\d:\d\d BOOT texture=psram_tile4$", "a line split across reads")
        self.assertEqual(lines[2], "no newline", "the unterminated tail is printed as is")

    def test_leaves_the_port_at_baud_0_without_hangup(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        port = os.ttyname(slave)
        self.assertEqual(run(SERIAL_LOG, port, "0.2").returncode, 0)
        st = subprocess.run(["stty", "-F", port, "-a"], capture_output=True, text=True).stdout
        self.assertIn("speed 0 baud", st)
        self.assertIn("-hupcl", st.split())
        self.assertIn("clocal", st.split())


if __name__ == "__main__":
    unittest.main(verbosity=1)
