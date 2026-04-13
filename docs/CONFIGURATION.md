# Configuration

Purpose: explain the repo’s actual configuration model, sources of truth, and known drift points.

## Configuration sources of truth

- Source of truth: `config/mower_config.schema.json`
- Deprecated but still maintained: `config/mower_config.sh.example`
- Redirect stub only: `src/open_mower/config/mower_config.sh.example`
- Runtime loading logic: `src/open_mower/launch/include/_params.launch`
- Hardware-specific defaults and YAML overlays: `src/open_mower/params/hardware_specific/**/*`
- Runtime entrypoint behavior: `docker/openmower_entrypoint.sh` and `docker/openmower_entrypoint.legacy.sh`

## Current files and what they mean

### `config/mower_config.schema.json`

- This is the authoritative structured config artifact in the repo.
- It groups configuration into top-level sections and records environment-variable mappings with `x-environment-variable`.
- The observed top-level groups are:
  - `important_settings` with title `Hardware Settings`
  - `gps_settings` with title `GPS Settings`
  - `mower_logic_settings` with title `Mower Logic Settings`
  - `external_mqtt_broker` with title `External MQTT Broker`
  - `sound_settings` with title `Sound Settings`
  - `custom_environment` with title `Additional Environment Variables`

### `config/mower_config.sh.example`

- This file is explicitly marked with a deprecation notice.
- Its header says changes must be mirrored into the schema.
- It still matters because the legacy runtime entrypoint sources `/config/mower_config.sh`.

### `src/open_mower/config/mower_config.sh.example`

- This file is not a second real config file.
- Its only content is a redirect note telling readers to use the root `config/` copy.

## Loading model observed in launch and entrypoint files

## Default or OSv2-style flow

Observed from `src/open_mower/launch/include/_params.launch` and `docker/openmower_entrypoint.sh`:

- When `OM_LEGACY_CONFIG_MODE` is not set, `_params.launch` follows the newer YAML-first path.
- For `MOWER=CUSTOM`, it loads `$(env PARAMS_PATH)/custom_params.yaml`.
- For predefined mower types, it loads:
  - `src/open_mower/params/openmower_defaults_v2.yaml`
  - `src/open_mower/params/hardware_specific/$(env MOWER)/params_v2.yaml`
  - `$(env PARAMS_PATH)/mower_params.yaml`
- If `HARDWARE_PLATFORM=1`, it also loads hardware-specific comms YAML for v1 low-level settings.
- `docker/openmower_entrypoint.sh` does not source `mower_config.sh`; it sources ROS setup, the built workspace, and `version_info.env`, then relies on runtime environment variables that are expected to come from outside the image.

The exact OSv2-side mechanism that provides `MOWER`, `PARAMS_PATH`, `HARDWARE_PLATFORM`, and related variables is not fully described inside this repo, so do not claim more than the files show.

## Legacy flow

Observed from `docker/openmower_entrypoint.legacy.sh` and `_params.launch`:

- The legacy entrypoint sources `/config/mower_config.sh`.
- If `OM_V2` is truthy, it sets `HARDWARE_PLATFORM=2`.
- Otherwise it sets `HARDWARE_PLATFORM=1` and `OM_LEGACY_CONFIG_MODE=True`.
- It exports:
  - `ESC_TYPE=$OM_MOWER_ESC_TYPE`
  - `MOWER=$OM_MOWER`
- It sources `src/open_mower/params/hardware_specific/$MOWER/default_environment.sh`.
- It sets:
  - `RECORDINGS_PATH=$HOME`
  - `PARAMS_PATH=$HOME`
- In legacy mode, `_params.launch` maps many environment variables directly onto the ROS parameter server for mower comms, GPS, xbot positioning, correction input, mower logic, monitoring, and snapshot features.
- `_comms.launch` starts the built-in NTRIP client only when `OM_USE_NTRIP=True` and `OM_USE_RTCM_TCP` is not truthy.
- `_comms.launch` starts `open_mower/scripts/rtcm_tcp_bridge.py` when `OM_USE_RTCM_TCP=True`, publishing raw TCP RTCM into the same `/ll/position/gps/rtcm` path used by the GPS driver.

## Schema grouping summary

The schema is large. Do not paste it into docs. Use it as the reference and summarize by section.

### Hardware settings

Observed examples:

- `OM_HARDWARE_VERSION`
- `OM_MOWER`
- `ESC_TYPE`
- `OM_MOWER_GAMEPAD`
- `OM_IGNORE_CHARGING_CURRENT`

### GPS settings

Observed examples:

- datum coordinates via `OM_DATUM_LAT` and `OM_DATUM_LONG`
- correction-source selection via `OM_USE_NTRIP` and `OM_USE_RTCM_TCP`
- NTRIP connection values
- raw RTCM TCP host and port values
- advanced GPS transport and protocol settings such as serial versus TCP, `OM_GPS_PROTOCOL`, and relative-position controls

Choose one correction source in normal operation:

- `OM_USE_NTRIP=True` for the built-in NTRIP client
- `OM_USE_RTCM_TCP=True` for the raw TCP bridge

If both are truthy, the current launch wiring gives raw TCP precedence and skips the NTRIP client.

### Mower logic settings

Observed examples:

- docking and undocking distances and timing
- tool width
- `OM_ENABLE_MOWER`
- `OM_RANDOMIZE_MOWER_DIRECTION`
- battery voltage thresholds
- mower motor temperature thresholds
- GPS wait and timeout settings
- automatic mode
- rain handling
- recording and snapshot settings

### External MQTT broker

Observed examples:

- `OM_MQTT_ENABLE`
- broker host, port, username, password, and topic prefix

### Sound settings

Observed examples:

- `OM_DFP_IS_5V`
- `OM_LANGUAGE`
- `OM_VOLUME`
- `OM_BACKGROUND_SOUNDS`

### Additional environment variables

- The schema reserves a `custom_environment` section.
- No predefined environment-variable mappings were observed in that section during this documentation pass.

## Observed mismatches and drift

### `ESC_TYPE` versus `OM_MOWER_ESC_TYPE`

- The schema exposes `ESC_TYPE`.
- The deprecated shell example uses `OM_MOWER_ESC_TYPE`.
- The legacy entrypoint resolves this by exporting `ESC_TYPE=$OM_MOWER_ESC_TYPE`.

Document this mismatch rather than hiding it. If you change one side, update all three places and the docs together.

### `Sabo` hardware preset drift

- `src/open_mower/params/hardware_specific/Sabo/` exists and includes `default_environment.sh` plus `params_v2.yaml`.
- `OM_MOWER` in both `config/mower_config.schema.json` and `config/mower_config.sh.example` now lists `Mowrator`, `YardForce500`, `YardForceSA650`, and `CUSTOM`, but still does not expose `Sabo`.

Treat this as observed fork-specific or repo-internal drift until the artifacts are brought back into alignment.

### README path drift

- `README.md` still tells users to copy `src/open_mower/config/mower_config.sh.example`.
- The package-local file is now only a stub redirect.
- The real deprecated shell example is `config/mower_config.sh.example`.

## Sync rules for contributors

- When config semantics change, update:
  - `config/mower_config.schema.json`
  - `config/mower_config.sh.example`
  - [BUILD_AND_RUN.md](BUILD_AND_RUN.md) if the workflow changes
  - this document
- If a change affects launch-time loading, also update the nearest launch or agent doc that explains it.
- If a mismatch is intentional for compatibility reasons, document that explicitly rather than leaving it implicit.
