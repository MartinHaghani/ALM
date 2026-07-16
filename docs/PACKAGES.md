# Packages

Purpose: inventory the workspace packages and clarify which ones are first-party runtime code versus external or shared infrastructure.

## Direct packages under `src/`

- `src/open_mower`: orchestration package. It provides launch files, parameter YAML, hardware-specific presets, RViz configs, the raw TCP RTCM bridge script, the Mowrator battery-voltage CSV logger, the Pi CPU temperature publisher, optional C1 LIDAR launch wiring, optional passive `slam_toolbox` manager/alignment launch wiring, and the optional Raspberry Pi I2C LSM6DSO IMU publisher used by the main comms launch flow. The LSM6DSO node publishes both raw IMU data and the IMU die temperature.
- `src/mower_hardware`: supported Mowrator direct hardware bridge. It owns the `/hw` runtime namespace, drives left/right/blade ESCs with the xESC driver, and publishes clean hardware status/power/emergency telemetry, including per-ESC battery voltage, without the OpenMower low-level-board serial protocol.
- `src/mower_comms_v1`: legacy v1 low-level-board mower comms executable for non-Mowrator `HARDWARE_PLATFORM=1` presets.
- `src/mower_comms_v2`: legacy/simulation v2 ROS bridge built on xBot service interfaces. It consumes the shared JSON definitions under `services/`.
- `src/mower_logic`: high-level mower behavior package. It defines the `mower_logic` executable, dynamic reconfigure configs, and a separate `monitoring` executable. The monitoring executable republishes mower, power, GPS accuracy, ESC temperature, Pi CPU temperature, IMU temperature, and GNSS temperature telemetry as xBot sensor topics. On startup it clears stale xBot behavior-action registrations before re-registering the active state so the WebUI action set tracks the current behavior after node restarts. Each behavior entry now also clears the previous behavior prefixes again before advertising its own actions, so stale `Pause`/`Start` combinations do not linger after failed state transitions. Manual `Start Mowing` runs now clear any stored mowing checkpoint before re-entering `MowingBehavior`, while paused semiautomatic resumes still keep their saved progress. Map catalog create/select/rename/delete and saved-map edit calls are guarded here so they run only while idle, and mowing checkpoints include the selected map identity/hash before restore. Its mowing pause recovery now re-checks live fused GPS directly, and the node uses more than one callback thread so UI/state timers keep updating while safety callbacks are busy. During mowing it now publishes both the full current-area plan and the live remaining path to the shared xBot map-overlay channel so the existing WebUI can draw path progress on top of the stored map. Area recording uses `SweptAreaRecorder`, a Clipper2-backed helper that reconstructs saved mowing/obstacle polygons from the full swept mower footprint; paused recording can queue draft edit strokes before the area is saved. See [AREA_RECORDING_SWEEP.md](AREA_RECORDING_SWEEP.md).
- `src/mower_map`: map service package. It defines messages and services, publishes active-map topics, persists a saved-map catalog under `maps/`, preserves the legacy selected `map.json` compatibility copy, exposes an RPC provider, and owns Clipper2-backed selected-map edit geometry for paint/erase/polygon-replace tools.
- `src/mower_msgs`: shared mower-specific ROS messages and services such as `HwStatus`, `HwPower`, `ESCStatus`, `Emergency`, `HighLevelStatus`, and `HighLevelControlSrv`.
- `src/mower_simulation`: simulator-side low-level service implementation. It defines `mower_simulation` and dynamic reconfigure for simulation.
- `src/mower_utils`: helper package with `planner_test`, `xbot_pose_converter`, a planner test launch file, and RViz configs.

## Important packages under `src/lib/`

### Planning and navigation

- `src/lib/ftc_local_planner`: follow-the-carrot local planner plugin with dynamic reconfigure, debug topics, and recovery behavior. ALM's live mower defaults enable obstacle checking against the map edge during mowing and use a tighter `max_follow_distance` than the historical source so the controller bails out sooner instead of wandering far off the planned path before declaring a crash. Observed from its README and CMake targets.
- `src/lib/slic3r_coverage_planner`: coverage-planning package used by `open_mower.launch` and planner-related tooling.

### Positioning, monitoring, and remote control

- `src/lib/rplidar_ros`: Vendored/External. Slamtec ROS driver version 2.1.5 with RPLIDAR C1 support, used by optional C1 LIDAR launch wiring.
- `src/open_mower/scripts/passive_slam_odom.py`: first-party helper for passive SLAM only. It republishes the raw C1 scan into `slam_lidar`, estimates/manual-recalibrates gyro yaw-rate offset, warns when corrected gyro yaw rate is nonzero while wheel/twist motion appears stationary, and integrates measured twist plus IMU yaw rate into `slam_odom -> slam_base_link`, keeping SLAM independent from RTK heading jumps.
- `src/open_mower/scripts/passive_slam_alignment.py`: first-party helper for passive SLAM visualization only. It estimates and publishes `map -> slam_map`, preferring saved mowing-boundary front-right corner samples before falling back to synchronized RTK-fixed GPS/fused and SLAM pose pairs, so `/next/` can overlay the SLAM map without feeding mower localization or navigation.
- `src/open_mower/scripts/localization_confidence.py`: first-party read-only confidence monitor. It publishes `/localization_confidence/status` from GPS receiver telemetry, receiver-only GPS motion consistency, passive SLAM scan-to-map fit, scan pose-correction and motion-distortion estimates, and passive alignment quality without feeding mower localization, planning, costmaps, or control.
- `src/open_mower/scripts/localization_fusion.py`: first-party shadow fusion publisher for `/next/` and default area-recording geometry. It fuses the operational `/xbot_positioning/xb_pose` base pose with the globally aligned passive LIDAR pose, while using raw GPS only to require RTK fixed before GPS can contribute. If GPS drops to RTK float, GPS is ignored and the shadow pose follows LIDAR-only when LIDAR is globally available. It does not replace operational `map -> base_link` or feed navigation, planning, costmaps, or control.
- `src/lib/xbot_driver_gps`: External/Submodule. High-performance u-blox GPS driver with RTCM, IMU, and wheel-tick support, based on its README. ALM also publishes a raw `sensor_msgs/NavSatFix` topic beside the existing xBot absolute-pose topic so `/next/` can display the GPS antenna on a satellite map without inverse datum conversion, plus read-only GPS quality JSON and GNSS receiver temperature topics for confidence diagnostics.
- `src/lib/xbot_monitoring`: monitoring package providing `xbot_monitoring`, `heatmap_generator`, and an example sensor node. Its teleop bridge now suppresses redundant neutral `Twist` frames so an idle connected client does not keep `twist_mux` pinned away from autonomous navigation. In the main launch flow its remote velocity output is remapped to `/web_joy_vel`, then gated by `mower_input_router` before reaching `/joy_vel`. It also mirrors the registered WebUI action set as latched JSON on `xbot_monitoring/actions_json` so the React `/next/` UI can render enabled mower actions through rosbridge without depending on MQTT.
- `src/lib/xbot_msgs`: shared xBot message and service package.
- `src/lib/xbot_positioning`: localization package providing the `xbot_positioning` executable and pose-related services.
- `src/lib/xbot_remote`: remote command-velocity bridge used by the main launch flow. Like `xbot_monitoring`, it publishes a single zero command on teleop release instead of streaming repeated neutral frames from an idle client. In the main launch flow its output is remapped to `/web_joy_vel` so it cannot bypass manual input source selection.
- `src/lib/xbot_rpc`: RPC library plus ROS messages and services used by `mower_map`.

### Service framework and hardware interface libraries

- `src/lib/ntrip_client`: External/Submodule. ROS NTRIP client used via `_ntrip_client.launch`.
- `src/lib/xbot_framework`: External/Submodule. Shared xBot service framework, code generation, and runtime support. Contains nested submodules under `ext/`.
- `src/lib/xesc_ros/xesc_msgs`: xESC message package.
- `src/lib/xesc_ros/xesc_interface`: shared xESC interface library.
- `src/lib/xesc_ros/xesc_driver`: xESC driver library and node target.
- `src/lib/xesc_ros/xesc_2040_driver`: RP2040-specific xESC driver library.
- `src/lib/xesc_ros/xesc_yfr4_driver`: YardForce Rev4 adapter-oriented xESC driver library.
- `src/lib/xesc_ros/vesc_driver`: VESC compatibility driver library.
- `src/lib/xesc_ros/xesc`: top-level xESC package container with package metadata.

## Shared service contracts

- `services/diff_drive_service.json`
- `services/emergency_service.json`
- `services/gps_service.json`
- `services/imu_service.json`
- `services/input_service.json`
- `services/mower_service.json`
- `services/mower_ui_service.json`
- `services/power_service.json`

These JSON files are not catkin packages. They are shared service-definition inputs used directly by `mower_comms_v2` and `mower_simulation`, and they live in an External/Submodule directory.

## Simulation note

- `src/mower_simulation/README.md` says the simulator replaces the mower comms node.
- The current `src/open_mower/launch/sim_mower_logic.launch` still starts `mower_comms_v2`.
- The observed runtime shape is therefore: `mower_simulation` stands in for the low-level mower service side, while `mower_comms_v2` remains active as the ROS bridge in the simulation launch.

## Support directories that are not packages

- `src/lib/json`: support directory included from `mower_map` via `add_subdirectory`. Not a catkin package.

## Related docs

- [REPO_MAP.md](REPO_MAP.md)
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [SIMULATION.md](SIMULATION.md)
