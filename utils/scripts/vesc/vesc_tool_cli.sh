#!/bin/bash

set -euo pipefail

VESC_TOOL_DIR="${VESC_TOOL_DIR:-$HOME/tools/vesc_tool}"

find_vesc_tool_bin() {
  find "$VESC_TOOL_DIR/build/lin" -maxdepth 1 -type f -name 'vesc_tool*' -perm -111 | sort | tail -n 1
}

if [ ! -d "$VESC_TOOL_DIR" ]; then
  echo "VESC_TOOL_DIR does not exist: $VESC_TOOL_DIR" >&2
  echo "Build it first with utils/scripts/vesc/build_vesc_tool.sh." >&2
  exit 1
fi

VESC_TOOL_BIN="${VESC_TOOL_BIN:-$(find_vesc_tool_bin)}"

if [ -z "${VESC_TOOL_BIN:-}" ] || [ ! -x "$VESC_TOOL_BIN" ]; then
  echo "Could not find a built VESC Tool binary under $VESC_TOOL_DIR/build/lin" >&2
  echo "Build it first with utils/scripts/vesc/build_vesc_tool.sh." >&2
  exit 1
fi

exec "$VESC_TOOL_BIN" --offscreen "$@"
