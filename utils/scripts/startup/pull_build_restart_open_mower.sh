#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/_open_mower_env.sh"

open_mower_resolve_env
open_mower_require_env

if ! git -C "$OPEN_MOWER_REPO_DIR" diff --quiet --; then
  echo "Refusing to pull because tracked files in $OPEN_MOWER_REPO_DIR have local changes." >&2
  echo "Commit, stash, or discard them before running pull_build_restart_open_mower.sh." >&2
  exit 1
fi

if ! git -C "$OPEN_MOWER_REPO_DIR" diff --cached --quiet --; then
  echo "Refusing to pull because staged changes exist in $OPEN_MOWER_REPO_DIR." >&2
  echo "Commit, stash, or discard them before running pull_build_restart_open_mower.sh." >&2
  exit 1
fi

if ! (cd "$OPEN_MOWER_REPO_DIR" && git submodule foreach --recursive "git diff --quiet -- && git diff --cached --quiet --" >/dev/null); then
  echo "Refusing to pull because one or more submodules have local changes." >&2
  echo "Commit, stash, or discard submodule changes before running pull_build_restart_open_mower.sh." >&2
  exit 1
fi

git -C "$OPEN_MOWER_REPO_DIR" pull --ff-only
git -C "$OPEN_MOWER_REPO_DIR" submodule update --init --recursive

"$SCRIPT_DIR/compile_open_mower.sh"
"$SCRIPT_DIR/stop_open_mower_local.sh"
"$SCRIPT_DIR/start_open_mower_local.sh"
