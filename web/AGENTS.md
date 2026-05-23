# Web bundle guide

Purpose: guidance for the checked-in `web/` directory.

## What this directory looks like

- The directory contains compiled Flutter web assets such as `main.dart.js`, `flutter.js`, `flutter_bootstrap.js`, and asset manifests.
- `docker/development/docker-compose.yaml` mounts this directory read-only into nginx.
- The editable source for this bundle lives in the separate `OpenMowerApp` Flutter repository, not in this directory.
- The upstream history in this repo and the sibling build script both point to `OpenMowerApp` as the source of truth for UI changes.

## Working rules

- Treat `web/` as generated or built output by default.
- Do not hand-edit compiled assets unless the change is intentional, narrowly scoped, and called out in your summary.
- Make feature changes in the Flutter source repo, then rebuild and deploy the bundle back into `web/`.
- The current gamepad behavior for the served mower UI is implemented in the Flutter source repo. As of the `Mowrator` fork flow, gamepad mapping is:
  - left stick drives in `AREA_RECORDING`
  - the remote-control screen has an on-screen press-and-hold blade button
  - `L1 + R1` also runs the blade while held on a gamepad
  - both blade hold paths reuse the existing manual-mowing actions and therefore follow the active mower profile's blade control mode
  - the mower logic side should keep `start_manual_mowing` and `stop_manual_mowing` action availability stable while a hold is active; use a brief stop-chatter guard in `mower_logic` instead of hiding the stop action
  - blade control also requires `OM_ENABLE_MOWER=true` in the active mower config
- If the build pipeline changes, keep this file aligned so future agents know where the real source lives.
