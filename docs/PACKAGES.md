# Packages

Purpose: inventory the workspace packages and clarify which ones are first-party runtime code versus external or shared infrastructure.

## Direct packages under `src/`

- `src/open_mower`: orchestration package. It provides launch files, parameter YAML, hardware-specific presets, RViz configs, and the raw TCP RTCM bridge script used by the main comms launch flow.
- `src/mower_comms_v1`: v1 low-level mower comms executable for `HARDWARE_PLATFORM=1`.
- `src/mower_comms_v2`: v2 ROS bridge built on xBot service interfaces. It consumes the shared JSON definitions under `services/`.
- `src/mower_logic`: high-level mower behavior package. It defines the `mower_logic` executable, dynamic reconfigure configs, and a separate `monitoring` executable.
- `src/mower_map`: map service package. It defines messages and services, publishes map topics, persists map data, and exposes an RPC provider.
- `src/mower_msgs`: shared mower-specific ROS messages and services such as `Status`, `Power`, `HighLevelStatus`, and `HighLevelControlSrv`.
- `src/mower_simulation`: simulator-side low-level service implementation. It defines `mower_simulation` and dynamic reconfigure for simulation.
- `src/mower_utils`: helper package with `planner_test`, `xbot_pose_converter`, a planner test launch file, and RViz configs.

## Important packages under `src/lib/`

### Planning and navigation

- `src/lib/ftc_local_planner`: follow-the-carrot local planner plugin with dynamic reconfigure, debug topics, and recovery behavior. Observed from its README and CMake targets.
- `src/lib/slic3r_coverage_planner`: coverage-planning package used by `open_mower.launch` and planner-related tooling.

### Positioning, monitoring, and remote control

- `src/lib/xbot_driver_gps`: External/Submodule. High-performance u-blox GPS driver with RTCM, IMU, and wheel-tick support, based on its README.
- `src/lib/xbot_monitoring`: monitoring package providing `xbot_monitoring`, `heatmap_generator`, and an example sensor node.
- `src/lib/xbot_msgs`: shared xBot message and service package.
- `src/lib/xbot_positioning`: localization package providing the `xbot_positioning` executable and pose-related services.
- `src/lib/xbot_remote`: remote command-velocity bridge used by the main launch flow.
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
