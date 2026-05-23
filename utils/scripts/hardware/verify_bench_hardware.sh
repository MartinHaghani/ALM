#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

RUTX_IP="${RUTX_IP:-192.168.1.1}"
GPS_PORT="${GPS_PORT:-/dev/ttyAMA2}"
GPS_BAUD="${GPS_BAUD:-921600}"
LSM6DSO_BUS="${LSM6DSO_BUS:-1}"
LSM6DSO_ADDRESS="${LSM6DSO_ADDRESS:-0x6B}"

section() {
  printf '\n== %s ==\n' "$1"
}

section "Network interfaces"
ip -brief addr || true
ip route || true

section "RUTX reachability"
if ping -c 1 -W 1 "$RUTX_IP" >/dev/null 2>&1; then
  echo "RUTX responds at $RUTX_IP"
else
  echo "No ping response from $RUTX_IP. If the RUTX is factory-default, check Pi eth0 DHCP/static IP and subnet conflicts."
fi

if command -v curl >/dev/null 2>&1; then
  curl -k --connect-timeout 2 --max-time 4 --head "https://$RUTX_IP/" 2>/dev/null | sed -n '1,3p' || \
    curl --connect-timeout 2 --max-time 4 --head "http://$RUTX_IP/" 2>/dev/null | sed -n '1,3p' || true
fi

section "I2C bus"
if [ ! -e "/dev/i2c-$LSM6DSO_BUS" ]; then
  echo "/dev/i2c-$LSM6DSO_BUS does not exist. Enable I2C on the Raspberry Pi host before testing the LSM6DSO."
else
  if command -v i2cdetect >/dev/null 2>&1; then
    i2cdetect -y "$LSM6DSO_BUS" || true
  fi
  if ! python3 "$REPO_DIR/utils/scripts/hardware/read_lsm6dso.py" \
      --bus "$LSM6DSO_BUS" \
      --address "$LSM6DSO_ADDRESS" \
      --samples 3; then
    echo "LSM6DSO check failed. Continue with the remaining bench checks, then return to I2C wiring/power/address."
  fi
fi

section "F9P serial traffic"
if [ ! -e "$GPS_PORT" ]; then
  echo "$GPS_PORT does not exist. Check the Pi UART overlay, jumpers, and whether the F9P is on USB instead."
else
  stty -F "$GPS_PORT" "$GPS_BAUD" raw -echo -echoe -echok -echoctl -echoke -ixon -ixoff -crtscts || true
  echo "Reading up to 128 bytes from $GPS_PORT at $GPS_BAUD. No antenna is needed for a wiring-only traffic check."
  if command -v timeout >/dev/null 2>&1 && command -v hexdump >/dev/null 2>&1; then
    timeout 4 dd if="$GPS_PORT" bs=1 count=128 status=none 2>/dev/null | hexdump -C || true
  else
    echo "Install coreutils/bsdmainutils or use: timeout 4 dd if=$GPS_PORT bs=1 count=128 status=none | hexdump -C"
  fi
fi

section "USB devices"
if command -v lsusb >/dev/null 2>&1; then
  lsusb || true
else
  echo "lsusb is not installed."
fi

if [ -d /dev/serial/by-id ]; then
  ls -l /dev/serial/by-id || true
else
  echo "/dev/serial/by-id does not exist yet."
fi

section "Done"
echo "Bench checks completed. Treat any motor/ESC connection as a separate safety step."
