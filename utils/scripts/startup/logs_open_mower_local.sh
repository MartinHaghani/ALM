#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/_open_mower_env.sh"

open_mower_resolve_env

docker logs -f "$OPEN_MOWER_CONTAINER_NAME"
