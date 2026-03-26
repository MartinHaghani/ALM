#!/bin/bash
set -euo pipefail

# Compile the local checkout inside the toolchain container so the Pi host stays lean.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/_open_mower_env.sh"

open_mower_resolve_env
open_mower_require_env
open_mower_ensure_image

BUILD_STAMP_FILE="$OPEN_MOWER_REPO_DIR/build/.open_mower_image"
if [ -d "$OPEN_MOWER_REPO_DIR/build" ]; then
  if [ ! -f "$BUILD_STAMP_FILE" ] || [ "$(cat "$BUILD_STAMP_FILE")" != "$OPEN_MOWER_IMAGE" ]; then
    echo "Detected build artifacts from a different or unstamped image. Removing build and devel for a clean rebuild." >&2
    rm -rf "$OPEN_MOWER_REPO_DIR/build" "$OPEN_MOWER_REPO_DIR/devel"
  fi
fi

DOCKER_USER_ARGS=()
if [ "$OPEN_MOWER_IMAGE" = "open_mower_ros:pi-dev" ]; then
  DOCKER_USER_ARGS=(
    --user "$(id -u):$(id -g)"
    --env "HOME=/tmp/open_mower_compile"
  )
fi

if open_mower_docker_run \
  "${DOCKER_USER_ARGS[@]}" \
  --rm \
  --entrypoint= \
  "$OPEN_MOWER_IMAGE" \
  bash -lc "MISSING_PACKAGES=''; for package in libasio-dev libcrypto++-dev python3-venv python3.8-venv; do if ! dpkg -s \"\$package\" >/dev/null 2>&1; then MISSING_PACKAGES=\"\$MISSING_PACKAGES \$package\"; fi; done; if [ -n \"\$MISSING_PACKAGES\" ]; then DISABLED_FILES=\$(grep -rl 'packages.ros.org/ros/ubuntu' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true); for file in \$DISABLED_FILES; do mv \"\$file\" \"\$file.codex-disabled\"; done; apt-get update; apt-get install -y --no-install-recommends \$MISSING_PACKAGES; for file in \$DISABLED_FILES; do mv \"\$file.codex-disabled\" \"\$file\"; done; fi && source /opt/ros/noetic/setup.bash && cd /opt/open_mower_ros && if [ -f build/CMakeCache.txt ] && grep -q '^CATKIN_BLACKLIST_PACKAGES:STRING=.*slic3r_coverage_planner' build/CMakeCache.txt; then echo 'Detected cached slic3r_coverage_planner blacklist; removing build and devel for a clean rebuild.'; rm -rf build devel; fi && catkin_make"; then
  mkdir -p "$OPEN_MOWER_REPO_DIR/build"
  printf '%s\n' "$OPEN_MOWER_IMAGE" > "$BUILD_STAMP_FILE"
fi
