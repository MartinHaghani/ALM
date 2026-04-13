#!/bin/bash

set -euo pipefail

VESC_TOOL_DIR="${VESC_TOOL_DIR:-$HOME/tools/vesc_tool}"
VESC_TOOL_REPO_URL="${VESC_TOOL_REPO_URL:-https://github.com/vedderb/vesc_tool.git}"
VESC_TOOL_QMAKE_FLAGS="${VESC_TOOL_QMAKE_FLAGS:-CONFIG += release_lin build_original exclude_fw}"
VESC_TOOL_BUILD_JOBS="${VESC_TOOL_BUILD_JOBS:-$(nproc)}"

apt_install() {
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y "$@"
  else
    apt-get update
    apt-get install -y "$@"
  fi
}

if ! command -v git >/dev/null 2>&1; then
  echo "git is required to clone VESC Tool." >&2
  exit 1
fi

apt_install \
  build-essential \
  git \
  libqt5gamepad5-dev \
  libqt5serialport5-dev \
  libqt5svg5-dev \
  qml-module-qt-labs-platform \
  qml-module-qt-labs-settings \
  qml-module-qtquick-controls2 \
  qml-module-qtquick-extras \
  qml-module-qt3d \
  qt5-qmake \
  qtbase5-dev \
  qtbase5-private-dev \
  qtconnectivity5-dev \
  qtdeclarative5-dev \
  qtdeclarative5-private-dev \
  qtmultimedia5-dev \
  qtpositioning5-dev \
  qtquickcontrols2-5-dev

mkdir -p "$(dirname "$VESC_TOOL_DIR")"

if [ ! -d "$VESC_TOOL_DIR/.git" ]; then
  git clone --depth 1 "$VESC_TOOL_REPO_URL" "$VESC_TOOL_DIR"
else
  git -C "$VESC_TOOL_DIR" pull --ff-only
fi

cd "$VESC_TOOL_DIR"
qmake -config release "$VESC_TOOL_QMAKE_FLAGS"
make -j"$VESC_TOOL_BUILD_JOBS"
