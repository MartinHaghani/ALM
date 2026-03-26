#!/bin/bash

open_mower_default_image() {
  echo "open_mower_ros:pi-dev"
}

open_mower_resolve_env() {
  local script_dir repo_default
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  repo_default="$(cd "$script_dir/../../.." && pwd)"

  export OPEN_MOWER_IMAGE="${OPEN_MOWER_IMAGE:-$(open_mower_default_image)}"
  export OPEN_MOWER_REPO_DIR="${OPEN_MOWER_REPO_DIR:-$repo_default}"
  export OPEN_MOWER_CONFIG_FILE="${OPEN_MOWER_CONFIG_FILE:-$HOME/mower_config.sh}"
  export OPEN_MOWER_ROS_HOME="${OPEN_MOWER_ROS_HOME:-$HOME/.ros}"
  export OPEN_MOWER_CONTAINER_NAME="${OPEN_MOWER_CONTAINER_NAME:-open_mower_local}"
  export OPEN_MOWER_ROSCONSOLE_CONFIG="${OPEN_MOWER_ROSCONSOLE_CONFIG:-$OPEN_MOWER_REPO_DIR/docker/assets/rosconsole.config}"
}

open_mower_require_env() {
  if [ ! -d "$OPEN_MOWER_REPO_DIR" ]; then
    echo "OPEN_MOWER_REPO_DIR does not exist: $OPEN_MOWER_REPO_DIR" >&2
    exit 1
  fi

  if [ ! -f "$OPEN_MOWER_CONFIG_FILE" ]; then
    echo "OPEN_MOWER_CONFIG_FILE does not exist: $OPEN_MOWER_CONFIG_FILE" >&2
    echo "Create it from config/mower_config.sh.example before running the Pi workflow." >&2
    exit 1
  fi

  if [ ! -f "$OPEN_MOWER_ROSCONSOLE_CONFIG" ]; then
    echo "OPEN_MOWER_ROSCONSOLE_CONFIG does not exist: $OPEN_MOWER_ROSCONSOLE_CONFIG" >&2
    exit 1
  fi

  mkdir -p "$OPEN_MOWER_ROS_HOME"
  open_mower_ensure_version_info
}

open_mower_ensure_image() {
  if [ "$OPEN_MOWER_IMAGE" != "open_mower_ros:pi-dev" ]; then
    return
  fi

  if docker image inspect "$OPEN_MOWER_IMAGE" >/dev/null 2>&1; then
    return
  fi

  echo "Docker image $OPEN_MOWER_IMAGE is not present. Building it from docker/Dockerfile.PiDev..." >&2
  "$OPEN_MOWER_REPO_DIR/utils/scripts/startup/build_open_mower_pi_image.sh"
}

open_mower_ensure_version_info() {
  local version_info_file version

  version_info_file="$OPEN_MOWER_REPO_DIR/version_info.env"
  if [ -f "$version_info_file" ]; then
    return
  fi

  version="v0.0.0"
  if command -v git >/dev/null 2>&1 && git -C "$OPEN_MOWER_REPO_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    version="$(git -C "$OPEN_MOWER_REPO_DIR" describe --tags 2>/dev/null || git -C "$OPEN_MOWER_REPO_DIR" describe --always 2>/dev/null || echo "v0.0.0")"
  fi

  printf 'export OM_SOFTWARE_VERSION=%s\n' "$version" > "$version_info_file"
}

open_mower_docker_run() {
  if [ -t 0 ] && [ -t 1 ]; then
    docker run \
      -it \
      --privileged \
      -v /dev:/dev \
      -v "$OPEN_MOWER_CONFIG_FILE:/config/mower_config.sh:ro" \
      -v "$OPEN_MOWER_ROSCONSOLE_CONFIG:/config/rosconsole.config:ro" \
      -v "$OPEN_MOWER_REPO_DIR:/opt/open_mower_ros" \
      -v "$OPEN_MOWER_ROS_HOME:/root/.ros" \
      --network host \
      "$@"
  else
    docker run \
      --privileged \
      -v /dev:/dev \
      -v "$OPEN_MOWER_CONFIG_FILE:/config/mower_config.sh:ro" \
      -v "$OPEN_MOWER_ROSCONSOLE_CONFIG:/config/rosconsole.config:ro" \
      -v "$OPEN_MOWER_REPO_DIR:/opt/open_mower_ros" \
      -v "$OPEN_MOWER_ROS_HOME:/root/.ros" \
      --network host \
      "$@"
  fi
}
