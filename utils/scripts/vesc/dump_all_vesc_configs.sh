#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VESC_TOOL_CLI="${VESC_TOOL_CLI:-$SCRIPT_DIR/vesc_tool_cli.sh}"
OUTPUT_DIR="${1:-$HOME/vesc-config-dumps/current}"

LEFT_DRIVE_PORT="${LEFT_DRIVE_PORT:-/dev/ttyAMA5}"
RIGHT_DRIVE_PORT="${RIGHT_DRIVE_PORT:-/dev/ttyAMA3}"
MOWER_PORT="${MOWER_PORT:-/dev/ttyAMA4}"

mkdir -p "$OUTPUT_DIR"

dump_pair() {
  local name port
  name="$1"
  port="$2"

  "$VESC_TOOL_CLI" --vescPort "$port" --getMcConf "$OUTPUT_DIR/${name}-mcconf.xml"
  "$VESC_TOOL_CLI" --vescPort "$port" --getAppConf "$OUTPUT_DIR/${name}-appconf.xml"
}

dump_pair "left-drive" "$LEFT_DRIVE_PORT"
dump_pair "right-drive" "$RIGHT_DRIVE_PORT"
dump_pair "mower" "$MOWER_PORT"

echo "Saved VESC config snapshots to $OUTPUT_DIR"
