#!/usr/bin/env bash
# Build here, copy the merged image to the host the StopWatch is plugged into, and write it.
#
#   tools/flash.sh            # build, then flash through pi4-deck
#   HOST=other tools/flash.sh
#   LOG_SECS=15 tools/flash.sh   # and print serial output for 15 s after the reset
#
# The port is pinned by serial number so another ESP32 on the same host is never written.
set -euo pipefail
cd "$(dirname "$0")/.."

HOST=${HOST:-pi4-deck}
PORT=${PORT:-/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_C4:9E:7E:12:A8:F8-if00}
ESPTOOL=${ESPTOOL:-'python3 ~/.platformio/packages/tool-esptoolpy/esptool.py'}
REMOTE_DIR=${REMOTE_DIR:-'~/m5-stopwatch-earth'}

~/.platformio/penv/bin/pio run -e stopwatch
IMG=.pio/build/stopwatch/firmware.factory.bin
[ -f "$IMG" ] || { echo "missing $IMG" >&2; exit 1; }

ssh "$HOST" "mkdir -p $REMOTE_DIR"
scp -q "$IMG" "$HOST:$REMOTE_DIR/firmware.factory.bin"
scp -q tools/serial_log.py "$HOST:$REMOTE_DIR/serial_log.py"
ssh "$HOST" "test -e '$PORT' || { echo 'StopWatch not on $PORT' >&2; exit 1; }; \
  ! fuser '$PORT' 2>/dev/null || { echo 'something holds $PORT' >&2; exit 1; }; \
  $ESPTOOL --chip esp32s3 -p '$PORT' write_flash 0x0 $REMOTE_DIR/firmware.factory.bin"

# LOG_SECS=N: keep reading serial for N seconds right after the reset, so boot lines are caught.
if [ "${LOG_SECS:-0}" -gt 0 ]; then
  ssh "$HOST" "python3 $REMOTE_DIR/serial_log.py '$PORT' $LOG_SECS"
fi
