#!/bin/bash
set -euo pipefail

# The plain-Pi dev workflow bind-mounts a local checkout into runtime images that
# may lag behind ALM's workspace dependencies. Install small missing
# runtime libs in-container before delegating to the legacy entrypoint.

restore_disabled_sources() {
  local file
  for file in ${DISABLED_APT_FILES:-}; do
    if [ -f "$file.codex-disabled" ]; then
      mv "$file.codex-disabled" "$file"
    fi
  done
}

install_runtime_packages() {
  local missing_packages package file
  missing_packages=""

  for package in libcrypto++6 nginx python3-dbus python3-gi; do
    if ! dpkg -s "$package" >/dev/null 2>&1; then
      missing_packages="$missing_packages $package"
    fi
  done

  if [ -z "$missing_packages" ]; then
    return
  fi

  DISABLED_APT_FILES="$(grep -rl 'packages.ros.org/ros/ubuntu' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true)"
  trap restore_disabled_sources EXIT

  for file in $DISABLED_APT_FILES; do
    mv "$file" "$file.codex-disabled"
  done

  apt-get update
  apt-get install -y --no-install-recommends $missing_packages

  restore_disabled_sources
  trap - EXIT
}

if [ "${OM_SKIP_RUNTIME_PACKAGE_INSTALL:-False}" != "True" ] && [ "${OM_SKIP_RUNTIME_PACKAGE_INSTALL:-False}" != "true" ]; then
  install_runtime_packages
fi

configure_web_services() {
  mkdir -p /var/lib/nginx /run/nginx /var/log/nginx
  rm -rf /etc/nginx/sites-enabled/*
  cp /opt/open_mower_ros/docker/assets/nginx.conf /etc/nginx/conf.d/default.conf
}

start_web_services() {
  service nginx start
}

configure_web_services
start_web_services

export ROSCONSOLE_CONFIG_FILE="${ROSCONSOLE_CONFIG_FILE:-/config/rosconsole.config}"

exec /opt/open_mower_ros/docker/openmower_entrypoint.legacy.sh "$@"
