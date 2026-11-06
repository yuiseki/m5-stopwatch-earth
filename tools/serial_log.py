#!/usr/bin/env python3
"""Read the StopWatch's USB serial for a while without resetting it.

    python3 serial_log.py PORT SECONDS

The StopWatch talks through the ESP32-S3's built-in USB-Serial/JTAG. On that bridge DTR/RTS
are reset and boot-mode lines, and pyserial raises both when it opens a port, which can drop
the chip into ROM download mode. So the port object is built unopened, both lines are cleared,
and only then is it opened.
"""
import sys
import time

import serial


def main():
    port, secs = sys.argv[1], float(sys.argv[2])
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    end = time.time() + secs
    buf = b""
    while time.time() < end:
        buf += ser.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            print(f"{time.strftime('%H:%M:%S')} {line.decode(errors='replace').rstrip()}", flush=True)
    if buf:
        print(buf.decode(errors="replace"), flush=True)


if __name__ == "__main__":
    main()
