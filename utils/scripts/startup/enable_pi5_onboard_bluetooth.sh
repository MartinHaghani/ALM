#!/bin/bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: sudo utils/scripts/startup/enable_pi5_onboard_bluetooth.sh [--release-uart0]

Re-enable the Raspberry Pi 5 onboard Bluetooth adapter for the mower-side
controller path. By default this keeps UART0 on GPIO14/15 available for the
serial console/debug header and only removes the boot settings that hide the
Bluetooth adapter from the kernel.

Options:
  --release-uart0   Also stop using UART0 as a header/console UART. Use this
                    only if the default Bluetooth re-enable still leaves
                    bluetoothctl reporting "No default controller available".
EOF
}

release_uart0=false
for arg in "$@"; do
  case "$arg" in
    --release-uart0)
      release_uart0=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  echo "This script edits boot config and must be run with sudo." >&2
  exit 1
fi

config_file="${OPEN_MOWER_PI_BOOT_CONFIG:-/boot/firmware/config.txt}"
cmdline_file="${OPEN_MOWER_PI_CMDLINE:-/boot/firmware/cmdline.txt}"
timestamp="$(date +%Y%m%d-%H%M%S)"

if [ ! -f "$config_file" ]; then
  echo "Boot config not found: $config_file" >&2
  exit 1
fi

cp -a "$config_file" "$config_file.openmower-bt-$timestamp.bak"

python3 - "$config_file" "$release_uart0" <<'PY'
import sys
from pathlib import Path

config = Path(sys.argv[1])
release_uart0 = sys.argv[2] == "true"
lines = config.read_text().splitlines()
out = []
has_active_krnbt_on = False

for line in lines:
    stripped = line.strip()
    active = stripped and not stripped.startswith("#")

    if active and stripped in ("dtoverlay=disable-bt", "dtoverlay=disable-bt-pi5"):
        out.append("# open_mower_bt: disabled to expose onboard Bluetooth")
        out.append("# " + line)
        continue

    if active and stripped == "dtparam=krnbt=off":
        out.append("dtparam=krnbt=on")
        has_active_krnbt_on = True
        continue

    if release_uart0 and active and stripped == "dtoverlay=uart0-pi5":
        out.append("# open_mower_bt: disabled to release UART0 from GPIO14/15")
        out.append("# " + line)
        continue

    if active and stripped == "dtparam=krnbt=on":
        has_active_krnbt_on = True

    out.append(line)

if not has_active_krnbt_on:
    out.append("")
    out.append("# open_mower_bt: allow the kernel to probe onboard Bluetooth")
    out.append("dtparam=krnbt=on")

config.write_text("\n".join(out) + "\n")
PY

if $release_uart0; then
  if [ -f "$cmdline_file" ]; then
    cp -a "$cmdline_file" "$cmdline_file.openmower-bt-$timestamp.bak"
    python3 - "$cmdline_file" <<'PY'
import sys
from pathlib import Path

cmdline = Path(sys.argv[1])
tokens = cmdline.read_text().strip().split()
drop = {"console=ttyAMA0,115200", "console=serial0,115200"}
cmdline.write_text(" ".join(token for token in tokens if token not in drop) + "\n")
PY
  fi
  systemctl disable --now serial-getty@ttyAMA0.service >/dev/null 2>&1 || true
  systemctl mask serial-getty@ttyAMA0.service >/dev/null 2>&1 || true
fi

systemctl enable bluetooth.service >/dev/null 2>&1 || true

echo "Updated $config_file"
if $release_uart0; then
  echo "UART0 console/header overlay was also released."
else
  echo "UART0 header/console settings were preserved."
fi
echo "Reboot the mower, then verify with:"
echo "  bluetoothctl show"
echo "  curl -s http://mowrator.local:8080/next/"
