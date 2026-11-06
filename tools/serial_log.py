#!/usr/bin/env python3
"""Read the StopWatch's USB serial for a while without resetting it.

    python3 serial_log.py PORT SECONDS

The StopWatch talks through the ESP32-S3's built-in USB-Serial/JTAG, where DTR/RTS drive the
chip's reset. On Linux, cdc_acm raises DTR and RTS whenever the port is opened with a non-zero
baud rate, and that reset the chip on every open (rst:0x15 USB_UART_CHIP_RESET), including
while it was mid-way through a panel transfer. Clearing DTR/RTS from pyserial after the open
comes too late.

So the port is set to baud 0 (the kernel then leaves DTR/RTS alone on open), raw, without
hang-up on close, and read with plain os.read so termios is never touched again. Baud means
nothing to USB-Serial/JTAG. The first call after the board enumerates may still reset it,
because stty itself opens the port while the baud rate is still non-zero.
"""
import os
import select
import subprocess
import sys
import time


def main():
    port, secs = sys.argv[1], float(sys.argv[2])
    subprocess.run(["stty", "-F", port, "0", "raw", "-echo", "-hupcl", "clocal"], check=True)
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    end = time.time() + secs
    buf = b""
    try:
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                time.sleep(0.05)
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                print(f"{time.strftime('%H:%M:%S')} {line.decode(errors='replace').rstrip()}", flush=True)
    finally:
        os.close(fd)
    if buf:
        print(buf.decode(errors="replace"), flush=True)


if __name__ == "__main__":
    main()
