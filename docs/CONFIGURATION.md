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
  - `lidar_settings` with title `LIDAR Settings`
  - `slam_settings` with title `Passive SLAM Settings`
  - `imu_settings` with title `IMU Settings`
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
- If `HARDWARE_PLATFORM=1` and `MOWER=Mowrator`, it loads Mowrator direct hardware YAML under `/hw/services/...`.
- If `HARDWARE_PLATFORM=1` and `MOWER` is not `Mowrator`, it still loads legacy low-level-board comms YAML under `/ll/...`.
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
- In legacy mode with `MOWER=Mowrator`, `_params.launch` maps the useful environment variables onto `/hw/services/...`, disables runtime rain/charger/perimeter assumptions, and leaves low-level-board-only settings unused.
- In legacy mode with non-Mowrator presets, `_params.launch` keeps the old `/ll/...` low-level-board mapping.
- `_comms.launch` starts the built-in NTRIP client only when `OM_USE_NTRIP=True` and `OM_USE_RTCM_TCP` is not truthy.
- `_comms.launch` starts `open_mower/scripts/rtcm_tcp_bridge.py` when `OM_USE_RTCM_TCP=True`, publishing raw TCP RTCM into `/hw/position/gps/rtcm` for the supported Mowrator runtime.
- `_comms.launch` starts `open_mower/scripts/lsm6dso_imu_node.py` for Mowrator by default, publishing the Raspberry Pi I2C LSM6DSO to `/hw/imu/data_raw`.
- `_comms.launch` starts `open_mower/scripts/battery_voltage_logger.py` for Mowrator by default, subscribing to `/hw/power` and appending a fsynced CSV row every second for left drive, right drive, and mower/blade ESC voltages. It rotates at 10 MiB x 5 files by default. Set `OM_NO_BATTERY_VOLTAGE_LOG=True` to disable it.

## Schema grouping summary

The schema is large. Do not paste it into docs. Use it as the reference and summarize by section.

### Hardware settings

Observed examples:

- `OM_HARDWARE_VERSION`
- `OM_MOWER`
- `ESC_TYPE`
- `OM_MOWER_GAMEPAD`
- `OM_MANUAL_INPUT_DEFAULT_SOURCE`
- `OM_DIRECT_GAMEPAD_PROFILE`
- `OM_DIRECT_GAMEPAD_DEVICE`
- `OM_NO_DIRECT_GAMEPAD`
- `OM_NO_BLUETOOTH_GAMEPAD_MANAGER`
- `OM_IGNORE_CHARGING_CURRENT`

Manual input defaults to `web_gamepad`, so existing browser/root WebUI control remains the active source after launch. Select `direct_bluetooth` from `/next/` after pairing a mower-side controller, or set `OM_MANUAL_INPUT_DEFAULT_SOURCE=direct_bluetooth` only for controlled bench/test workflows where direct controller authority should come up active.

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

Multi-lawn map selection uses the configured absolute GPS datum as one shared `map` frame for all saved maps. Do not expect v1 map selection to switch GPS datum per lawn. Keep `OM_DATUM_LAT` and `OM_DATUM_LONG` stable for the mower's service region, and tune `OM_MAP_SELECTOR_MAX_START_DISTANCE_M` only if the default 50 m start guard is too tight for a recorded lawn.

### IMU settings

Observed examples:

- `OM_USE_LSM6DSO_IMU`
- `OM_LSM6DSO_I2C_BUS`
- `OM_LSM6DSO_I2C_ADDRESS`
- `OM_LSM6DSO_RATE_HZ`
- `OM_LSM6DSO_ACCEL_RANGE_G`
- `OM_LSM6DSO_GYRO_RANGE_DPS`
- `OM_LSM6DSO_AXIS_CONFIG`
- `OM_LSM6DSO_FRAME_ID`
- `OM_XBOT_POSITIONING_MAX_GYRO_CALIBRATION_ABS_OFFSET`
- `OM_XBOT_POSITIONING_MAX_GYRO_CALIBRATION_STDDEV`
- `OM_XBOT_POSITIONING_MAX_GYRO_CALIBRATION_RANGE`

These settings are for the current Raspberry Pi I2C SparkFun LSM6DSO replacement path. For `Mowrator`, the IMU publisher is default-on and publishes under `/hw/imu/data_raw`. The `OM_XBOT_POSITIONING_MAX_GYRO_CALIBRATION_*` values guard operational `map -> base_link` gyro recalibration so a transient raw-gyro burst cannot replace a good zero-rate offset.

### LIDAR settings

Observed examples:

- `OM_USE_C1_LIDAR`
- `OM_C1_SERIAL_PORT`
- `OM_C1_SERIAL_BAUDRATE`
- `OM_C1_FRAME_ID`
- `OM_C1_SCAN_TOPIC`
- `OM_C1_SCAN_FREQUENCY`
- `OM_C1_SCAN_MODE`
- `OM_C1_INVERTED`
- `OM_C1_ANGLE_COMPENSATE`
- `OM_C1_X`, `OM_C1_Y`, and `OM_C1_Z`
- `OM_C1_ROLL`, `OM_C1_PITCH`, and `OM_C1_YAW`

These settings start the optional Slamtec C1 driver and static `base_link` to LIDAR transform. Offsets are meters and angles are radians. The `/next/` sensor viewer can display this LaserScan alongside raw IMU output; it does not change localization, mapping, planning, or navigation authority.

The supported Mowrator C1 scan topic default is `/hw/lidar`. The Mowrator default environment places the C1 at the footprint center, `OM_C1_X=0.41` and `OM_C1_Y=0.0`, unless those values are explicitly overridden.

### Passive SLAM settings

Observed examples:

- `OM_USE_PASSIVE_SLAM`
- `OM_SLAM_RAW_SCAN_TOPIC`
- `OM_SLAM_SCAN_TOPIC`
- `OM_SLAM_NAMESPACE`
- `OM_SLAM_MAP_FRAME`
- `OM_SLAM_ODOM_FRAME`
- `OM_SLAM_ORIGIN_FRAME`
- `OM_SLAM_BASE_FRAME`
- `OM_SLAM_LIDAR_FRAME`
- `OM_SLAM_MAP_TOPIC`
- `OM_SLAM_ODOM_TOPIC`
- `OM_SLAM_TWIST_TOPIC`
- `OM_SLAM_IMU_TOPIC`
- `OM_SLAM_ODOM_GYRO_CALIBRATION_SECONDS`
- `OM_SLAM_ODOM_GYRO_STATIONARY_VX_THRESHOLD`
- `OM_SLAM_ODOM_GYRO_STATIONARY_WZ_THRESHOLD`
- `OM_SLAM_ODOM_GYRO_WARNING_YAW_RATE_THRESHOLD`
- `OM_SLAM_ODOM_GYRO_WARNING_SECONDS`
- `OM_SLAM_MAP_RESOLUTION`
- `OM_SLAM_MAX_LASER_RANGE`
- `OM_SLAM_START_ENABLED`
- `OM_SLAM_ALIGNMENT_STATUS_TOPIC`
- `OM_SLAM_ALIGNMENT_GPS_TOPIC`
- `OM_SLAM_ALIGNMENT_NAVSAT_TOPIC`
- `OM_SLAM_ALIGNMENT_BOUNDARY_SAMPLE_TOPIC`
- `OM_SLAM_ALIGNMENT_MIN_TRAVEL_M`
- `OM_SLAM_ALIGNMENT_MAX_RESIDUAL_M`
- `OM_SLAM_ALIGNMENT_TF_BUFFER_DURATION`
- `OM_SLAM_ALIGNMENT_MAX_BOUNDARY_SAMPLES`
- `OM_SLAM_ALIGNMENT_MAX_STATUS_BOUNDARY_PAIRS`
- `OM_SLAM_ALIGNMENT_POSE_SYNC_MAX_LAG`
- `OM_SLAM_ODOM_GYRO_CALIBRATION_MAX_ABS_OFFSET`
- `OM_SLAM_ODOM_GYRO_CALIBRATION_MAX_STDDEV`
- `OM_SLAM_ODOM_GYRO_CALIBRATION_MAX_RANGE`
- `OM_ENABLE_SLAM_RECORDING`

These settings start the passive SLAM manager, a SLAM-only local odometry helper, and the passive alignment helper used by the combined `/next/` map. Mapping starts stopped by default so the operator can choose the first map origin from the WebUI. When mapping is started, the manager resets the SLAM-only odometry origin and starts `slam_toolbox`; the alignment helper clears old samples and learns a visualization-only `map -> slam_map` transform. The passive odom helper estimates gyro yaw-rate offset during its startup calibration window and exposes `/passive_slam_odom/calibrate_gyro`; operational `map -> base_link` heading is still owned by `xbot_positioning`, which exposes `/xbot_positioning/recalibrate_gyro`. The `/next/` Sensors tab calls both services for manual gyro recalibration while the mower is still; calibration windows are rejected if the raw gyro mean, standard deviation, or range is implausible. Stationary gyro warnings are diagnostic only. It prefers saved mowing-boundary samples from `/area_recorder/boundary_samples`, then falls back to synchronized RTK-fixed `map -> base_link` and `slam_map -> slam_base_link` motion samples if no usable boundary path exists. Boundary samples default to a larger retention window than generic motion samples so long yard recordings are not truncated, and the status topic sends a decimated calibration trace for the WebUI. Live GPS and SLAM robot status poses are time-synchronized when their TF stamps are close enough, reducing motion-only marker separation caused by GPS/fusion latency. The default passive tree while mapping is `map -> slam_map -> slam_odom -> slam_base_link -> slam_lidar`; the raw C1 scan from `/hw/lidar` is republished as `/slam_toolbox/scan` in `slam_lidar`. The mower's existing `map -> base_link -> lidar` localization remains separate and authoritative for normal mower behavior, so passive SLAM does not feed costmaps, planning, or control.

Area recording now saves mowing and obstacle geometry from the full costmap footprint swept along the selected recording pose. The default selected pose is `/localization_fusion/pose`; `/next/` exposes a runtime toggle back to the legacy `/xbot_positioning/xb_pose` GPS path while no polygon is actively recording. Mowing outlines use the largest exterior boundary of that swept union. Obstacles require a closed loop and use the largest interior hole boundary of the swept union. Navigation areas remain the existing `base_link` breadcrumb polygon. Polygon recording skips new trusted geometry when the selected pose is stale or outside its configured quality limits; those gaps start a new swept segment so the final saved polygon does not bridge through bad data. Boundary samples for GPS/LIDAR alignment remain RTK-fixed GPS truth only and are not generated from LIDAR-derived fused pose. Existing maps should be cleared and rerecorded after deploying this behavior. See [AREA_RECORDING_SWEEP.md](AREA_RECORDING_SWEEP.md) before tuning the area-recording params.

The map selector keeps one selected active map for display, recording, and mowing. Create or select the intended map while `mower_logic` is idle, then start area recording; recorded mowing, navigation, obstacle, and docking data are saved into that selected map. Map create/select/rename/delete controls are intentionally guarded by `mower_logic`, and Start Mowing is blocked when the current fused pose is farther than `/mower_logic/map_selector/max_start_distance_m` from the selected map bounds.

### Localization confidence settings

Observed examples:

- `OM_USE_LOCALIZATION_CONFIDENCE`
- `OM_LOCALIZATION_CONFIDENCE_STATUS_TOPIC`
- `OM_GPS_QUALITY_TOPIC`
- `OM_LOCALIZATION_CONFIDENCE_RATE_HZ`
- `OM_CONFIDENCE_MAX_TOPIC_AGE_SEC`
- `OM_CONFIDENCE_SCAN_MAX_POINTS`
- `OM_CONFIDENCE_SCAN_MATCH_NEAR_M`
- `OM_CONFIDENCE_SCAN_MATCH_FAR_M`

These settings start a read-only confidence monitor. The GPS driver publishes receiver-quality JSON on `/hw/position/gps/quality`, including carrier phase, satellite count, pDOP, reported accuracy, RTCM freshness, and parser health when UBX telemetry is available. The monitor publishes `/localization_confidence/status` with conservative GPS and LIDAR trust scores for display in `/next/`. GPS motion self-consistency uses a rolling receiver-only position/velocity check with receiver speed and timing uncertainty so normal driving does not look like a GPS fault. LIDAR confidence uses scan-to-map fit, estimated scan pose correction, observability, scan motion distortion, and passive alignment quality so rotation/scan-warp conditions can reduce LIDAR trust before fusion. The scores are diagnostics only; they do not affect mower localization, planning, costmaps, or control.

### Localization fusion settings

Observed examples:

- `OM_USE_LOCALIZATION_FUSION`
- `OM_LOCALIZATION_FUSION_POSE_TOPIC`
- `OM_LOCALIZATION_FUSION_ODOM_TOPIC`
- `OM_LOCALIZATION_FUSION_STATUS_TOPIC`
- `OM_LOCALIZATION_FUSION_BASE_FRAME`
- `OM_LOCALIZATION_FUSION_RATE_HZ`
- `OM_LOCALIZATION_FUSION_MAX_TOPIC_AGE_SEC`
- `OM_LOCALIZATION_FUSION_TF_MAX_AGE_SEC`
- `OM_LOCALIZATION_FUSION_PUBLISH_TF`

These settings start a shadow publisher for the unified localization marker in `/next/` and the default area-recording geometry source. It publishes `/localization_fusion/pose`, `/localization_fusion/odom`, `/localization_fusion/status`, and `map -> fused_base_link`. It uses the operational fused GPS/base pose from `/xbot_positioning/xb_pose` for the GPS position, uses raw GPS only as an RTK-fixed quality gate, and uses the globally aligned passive LIDAR pose with the confidence monitor's sigma/confidence values. RTK float is rejected as a GPS fusion source, so the shadow pose becomes LIDAR-only when the aligned LIDAR pose is available. It does not replace `map -> base_link` and does not feed navigation, planning, costmaps, or control.

### Mower logic settings

Observed examples:

- docking and undocking distances and timing
- tool width
- `OM_ENABLE_MOWER`
- `OM_RANDOMIZE_MOWER_DIRECTION`
- battery voltage thresholds
- drive ESC battery voltage mismatch warning
- battery voltage CSV logging controls: `OM_NO_BATTERY_VOLTAGE_LOG`, `OM_BATTERY_VOLTAGE_LOG_PATH`, `OM_BATTERY_VOLTAGE_LOG_PERIOD_SEC`, `OM_BATTERY_VOLTAGE_LOG_FSYNC`, `OM_BATTERY_VOLTAGE_LOG_MAX_BYTES`, and `OM_BATTERY_VOLTAGE_LOG_MAX_FILES`
- mower motor temperature thresholds
- GPS wait and timeout settings
- multi-lawn map-selector start-distance guard: `OM_MAP_SELECTOR_MAX_START_DISTANCE_M`
- automatic mode
- swept area recording controls: `OM_AREA_RECORDING_USE_LOCALIZATION_FUSION`, `OM_AREA_RECORDING_FUSED_POSE_TOPIC`, `OM_AREA_RECORDING_LEGACY_POSE_TOPIC`, `OM_AREA_RECORDING_MAX_POSE_AGE_SEC`, `OM_AREA_RECORDING_MAX_FUSED_POSITION_ACCURACY_M`, `OM_AREA_RECORDING_MAX_FUSED_YAW_ACCURACY_RAD`, `OM_AREA_RECORDING_POSE_STEP_M`, `OM_AREA_RECORDING_YAW_STEP_RAD`, `OM_AREA_RECORDING_SIMPLIFY_EPSILON_M`, and `OM_AREA_RECORDING_MIN_POLYGON_AREA_M2`
- legacy rain settings, ignored by the supported Mowrator runtime
- recording and snapshot settings

### External MQTT broker

Observed examples:

- `OM_MQTT_ENABLE`
- broker host, port, username, password, and topic prefix

### Sound settings

Sound settings are legacy OpenMower low-level-board settings. The supported Mowrator runtime has no sound board.

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
