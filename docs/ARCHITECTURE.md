# Architecture

Purpose: explain the verified runtime shape of this workspace without speculating beyond repo evidence.

## What this document is based on

This summary is grounded in:

- `src/open_mower/launch/open_mower.launch`
- launch includes under `src/open_mower/launch/include/`
- package CMake targets and package metadata under `src/`
- `src/mower_map/src/mower_map_service.cpp`
- `src/mower_simulation/launch/_mower_simulation.launch`
- `src/mower_simulation/src/mower_simulation.cpp`

## Runtime composition

`src/open_mower/launch/open_mower.launch` composes the primary mower runtime from smaller launch fragments plus a set of core nodes.

### Launch includes

- `_params.launch`: loads parameters from YAML and or environment variables, depending on legacy mode and hardware platform.
- `_comms.launch`: starts `mower_hardware` for the supported `Mowrator` direct-Pi hardware path, starts the separate Mowrator battery-voltage CSV logger unless disabled, keeps `mower_comms_v1` available only for legacy non-Mowrator `HARDWARE_PLATFORM=1` presets, or starts `mower_comms_v2` for `HARDWARE_PLATFORM=2`. It also starts the Raspberry Pi I2C LSM6DSO IMU publisher and chooses either the built-in NTRIP client or the raw TCP RTCM bridge for correction input.
- `_c1_lidar.launch`: optionally starts the vendored Slamtec C1 driver and publishes a static `base_link` to LIDAR transform when `OM_USE_C1_LIDAR=True`.
- `_passive_slam.launch`: optionally starts the passive SLAM manager, SLAM-only odometry helper, and passive alignment helper when `OM_USE_PASSIVE_SLAM=True`. The manager starts mapping disabled by default, resets the local `slam_odom -> slam_base_link` odometry origin when mapping is started, then runs `slam_toolbox` under `/slam_toolbox`. The alignment helper estimates a visualization-only `map -> slam_map` transform, preferring saved mowing-boundary GPS/LIDAR corner pairs before falling back to synchronized RTK-fixed motion pose pairs, without changing mower localization or navigation authority.
- `_move_base.launch`: starts `mbf_costmap_nav` plus the legacy relay shim, loading costmap and planner YAML from `src/open_mower/params/`.
- `_localization.launch`: starts `xbot_positioning`.
- `_teleop.launch`: starts joystick input and teleop mapping based on the selected gamepad.
- `_record.launch`: enables rosbag recording or snapshot buffering when the matching environment variables are set.

### Core nodes launched directly

- `mower_map_service` from `mower_map`
- `mower_logic` and `monitoring` from `mower_logic`
- `slic3r_coverage_planner`
- `twist_mux`
- `xbot_monitoring`
- `heatmap_generator`, conditionally
- `xbot_remote`

## High-level data and control flow

1. Parameters are assembled first from launch-time YAML and environment inputs.
2. The supported Mowrator hardware layer exposes direct hardware state and control topics under `/hw/...`, including `/hw/power` battery voltage telemetry from the left drive, right drive, and mower/blade ESCs.
3. `xbot_positioning` consumes GPS, IMU, and measured twist data to produce the mower pose.
4. `mower_map_service` provides map storage, occupancy-grid publication, docking and mowing-area services, and an RPC method named `map.replace`.
5. `mower_logic` coordinates mower behaviors such as idle, mowing, parking at the recorded docking point, and area recording, using `mower_map`, `slic3r_coverage_planner`, MBF actions, and `/hw` services.
6. Navigation runs through `mbf_costmap_nav` with configuration loaded from `src/open_mower/params/`.
7. Operator and UI-facing pieces include teleop input, `xbot_monitoring`, `xbot_remote`, optional heatmap generation, rosbridge, the existing Flutter UI at `/`, and the React combined GPS/LIDAR map plus sensor viewer at `/next/`.

## Package role split

- `open_mower`: orchestration package. It provides launch, params, RViz assets, a small Python RTCM bridge script for raw TCP correction sources, a small Python LSM6DSO IMU publisher for the current Pi-I2C bench hardware path, the separate battery-voltage CSV logger, and optional C1 LIDAR launch wiring.
- `mower_hardware`: supported Mowrator direct hardware bridge. It drives left/right/blade ESCs through the xESC driver and publishes `/hw/status`, `/hw/power`, `/hw/emergency`, measured drive telemetry, and per-ESC battery voltage without the OpenMower low-level-board protocol.
- `mower_logic`: high-level decision-making and mower state transitions.
- `mower_map`: map storage and retrieval plus occupancy-grid and marker publication.
- `mower_comms_v1` and `mower_comms_v2`: legacy ROS bridges to OpenMower/YardForce low-level hardware or xBot services. They are retained for legacy/simulation context but are not the supported Mowrator runtime path.
- `mower_msgs`: shared ROS message and service definitions.
- `mower_simulation`: simulator-side low-level service implementation.
- `mower_utils`: helper tools for testing and pose conversion.

## Map and persistence

Observed from `src/mower_map/src/mower_map_service.cpp`:

- The current map file name is `map.json`.
- A legacy file name `map.bag` is still defined in code.
- The map service is the in-memory source of truth for loaded map data during runtime.
- The service publishes JSON, occupancy-grid, visualization, and map-size topics under `mower_map_service/...`.

The exact runtime storage directory is not explicitly set in the files inspected here, so avoid claiming a fixed on-disk location without further verification.

## Configuration boundary

The main config boundary is between:

- structured and documented config artifacts under `config/`
- launch-time loading logic in `src/open_mower/launch/include/_params.launch`
- hardware-specific defaults in `src/open_mower/params/hardware_specific/`
- runtime container entrypoints in `docker/openmower_entrypoint.sh` and `docker/openmower_entrypoint.legacy.sh`

See [CONFIGURATION.md](CONFIGURATION.md) for the exact loading model.

## Simulation

Simulation is built from:

- `src/open_mower/launch/sim_mower_logic.launch`
- `src/open_mower/launch/sim_navigation.launch`
- `src/mower_simulation/launch/_mower_simulation.launch`
- `src/mower_simulation/src/mower_simulation.cpp`

Observed behavior:

- `mower_simulation` starts low-level xBot services for emergency, diff drive, mower, IMU, power, and GPS.
- `sim_mower_logic.launch` still starts `mower_comms_v2`, so the current sim path appears to keep the v2 ROS bridge while replacing the physical low-level service side.
- `mower_simulation/README.md` says the simulator “basically replaces the mower_comms node.” The launch files show a more precise arrangement than that README summary.

See [SIMULATION.md](SIMULATION.md) for the full simulation notes.

## Safety-sensitive areas

Treat these as high-risk change zones:

- `src/mower_logic/**/*`
- `src/mower_hardware/**/*`
- `src/mower_comms_v1/**/*`
- `src/mower_comms_v2/**/*`
- `src/open_mower/launch/**/*`
- `src/open_mower/params/hardware_specific/**/*`
- `docker/openmower_entrypoint.sh`
- `docker/openmower_entrypoint.legacy.sh`

These files influence motion, docking, emergency handling, battery thresholds, GPS behavior, or runtime startup assumptions.
