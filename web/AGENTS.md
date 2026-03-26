# Web bundle guide

Purpose: guidance for the checked-in `web/` directory.

## What this directory looks like

- The directory contains compiled Flutter-style web assets such as `main.dart.js`, `flutter.js`, `flutter_bootstrap.js`, and asset manifests.
- `docker/development/docker-compose.yaml` mounts this directory read-only into nginx.
- No matching editable source tree for this bundle is present in this repository.

## Working rules

- Treat `web/` as generated or built output by default.
- Do not hand-edit compiled assets unless the change is intentional, narrowly scoped, and called out in your summary.
- If the source for this bundle is introduced later, update this file and [../docs/REPO_MAP.md](../docs/REPO_MAP.md).
