#!/bin/bash
set -euo pipefail

# Run the local checkout on the Pi. Compile first with compile_open_mower.sh.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/_open_mower_env.sh"

open_mower_resolve_env
open_mower_require_env
open_mower_ensure_image

docker rm -f "$OPEN_MOWER_CONTAINER_NAME" >/dev/null 2>&1 || true

open_mower_docker_run \
  --name "$OPEN_MOWER_CONTAINER_NAME" \
  --entrypoint /opt/open_mower_ros/docker/openmower_entrypoint.pi.sh \
  "$OPEN_MOWER_IMAGE" \
  roslaunch open_mower open_mower.launch --screen
