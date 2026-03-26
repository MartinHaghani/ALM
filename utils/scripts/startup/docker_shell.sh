#!/bin/bash
set -euo pipefail

# Open a shell inside the Pi runtime image with the local checkout mounted in.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/_open_mower_env.sh"

open_mower_resolve_env
open_mower_require_env
open_mower_ensure_image

open_mower_docker_run \
  --rm \
  --entrypoint /bin/bash \
  "$OPEN_MOWER_IMAGE"
