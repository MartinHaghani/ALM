#!/bin/bash
set -euo pipefail

# Build the local Pi dev image from the current checkout so the runtime does not
# depend on the broken prebuilt arm64 image.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/_open_mower_env.sh"

open_mower_resolve_env

docker build \
  -f "$OPEN_MOWER_REPO_DIR/docker/Dockerfile.PiDev" \
  -t "${OPEN_MOWER_IMAGE:-open_mower_ros:pi-dev}" \
  "$OPEN_MOWER_REPO_DIR"
