#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${OPEN_MOWER_REPO_DIR:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
NODE_IMAGE="${OPEN_MOWER_WEBUI_NODE_IMAGE:-node:22-bookworm-slim}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is required to build the next WebUI with $NODE_IMAGE." >&2
  exit 1
fi

docker run \
  --rm \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp/open_mower_webui \
  -v "$REPO_DIR:/workspace" \
  -w /workspace/webui \
  "$NODE_IMAGE" \
  bash -lc 'npm ci && npm run typecheck && npm run build'
