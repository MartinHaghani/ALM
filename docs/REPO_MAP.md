# Repository map

Purpose: explain the directory tree, ownership boundaries, and which areas are source of truth versus external or generated.

## Top-level directories

- `.devcontainer/`: VS Code devcontainer config. Development-only.
- `.github/`: GitHub Actions and release-drafter config. Includes the observed image-build workflow.
- `config/`: configuration artifacts. `config/mower_config.schema.json` is the structured Source of truth. `config/mower_config.sh.example` is Deprecated but still maintained.
- `devenv/`: Dockerized ROS Noetic desktop development environment with helper scripts and compose file.
- `docker/`: runtime images, entrypoints, runtime configs, and a development-only compose file for companion services.
- `img/`: image assets referenced by the README.
- `services/`: xBot service-definition JSON files such as `diff_drive_service.json` and `gps_service.json`. External/Submodule.
- `src/`: catkin workspace packages and supporting libraries.
- `utils/`: startup, debugging, firmware, and helper scripts.
- `web/`: built web output. The root contains the observed Flutter bundle, and `web/next/` is generated from `webui/`.
- `webui/`: Vite + React + TypeScript source for the `/next/` WebUI.

## Important files at the root

- `README.md`: upstream-facing overview and getting-started text. Contains some stale path and container notes, so use the docs layer for repo-specific guidance.
- `.gitmodules`: verifies the repo-level submodules.
- `.pre-commit-config.yaml`: configures pre-commit hooks and excludes `config/`, `web/`, and `src/lib/` from broad formatting/checking scope.
- `AGENTS.md`: root Codex guidance.
- `CLAUDE.md`: root Claude Code guidance.
- `CONTRIBUTING.md`: human contributor guide.

## `src/` layout

- `src/open_mower/`: main orchestration package. Contains launch files, parameter YAML, hardware-specific presets, RViz configs, and small runtime helper scripts such as the battery-voltage logger.
- `src/mower_logic/`: high-level mower state machine and monitoring node. Safety-critical.
- `src/mower_hardware/`: supported Mowrator direct hardware bridge under `/hw`. Safety-critical.
- `src/mower_comms_v1/`: legacy v1 low-level-board comms executable. Safety-critical when touched, but not the supported Mowrator runtime.
- `src/mower_comms_v2/`: legacy/simulation v2 xBot-service bridge. Safety-critical when touched.
- `src/mower_map/`: map service, RPC entrypoints, occupancy-grid publication, and map persistence.
- `src/mower_msgs/`: repo-specific messages and services shared across packages.
- `src/mower_simulation/`: simulation-side low-level service implementation.
- `src/mower_utils/`: helper binaries and launch files for testing and visualization.
- `src/lib/`: mixed external libraries, vendored packages, and submodules. Do not assume first-party ownership.

## `src/lib/` ownership notes

- `src/lib/ntrip_client/`: External/Submodule.
- `src/lib/rplidar_ros/`: Vendored/External Slamtec ROS driver used for optional C1 LIDAR bring-up.
- `src/lib/xbot_driver_gps/`: External/Submodule.
- `src/lib/xbot_framework/`: External/Submodule. Also contains nested submodules under `ext/`.
- `src/lib/ftc_local_planner/`, `src/lib/slic3r_coverage_planner/`, `src/lib/xbot_monitoring/`, `src/lib/xbot_msgs/`, `src/lib/xbot_positioning/`, `src/lib/xbot_remote/`, `src/lib/xbot_rpc/`, and `src/lib/xesc_ros/`: repo-local library packages shipped inside this workspace, but still not the place for casual broad formatting.
- `src/lib/json/`: support directory added via `add_subdirectory` from `mower_map`. Not a catkin package.

## Runtime and config boundaries

- `src/open_mower/launch/open_mower.launch`: primary runtime composition entrypoint.
- `src/open_mower/launch/include/_params.launch`: key source for how YAML and environment configuration are loaded.
- `src/open_mower/params/hardware_specific/`: mower presets and defaults. Includes `Mowrator/`, `YardForce500/`, `YardForceSA650/`, and `CUSTOM/`, plus an observed `Sabo/` preset that is still not exposed in the root config schema.
- `src/open_mower/config/mower_config.sh.example`: redirect stub pointing contributors to `config/`.

## Generated, external, and deprecated areas

- Generated: `web/`, including `web/next/`.
- External/Submodule: `services/`, `src/lib/ntrip_client/`, `src/lib/xbot_driver_gps/`, `src/lib/xbot_framework/`, plus nested submodules inside `src/lib/xbot_framework/ext/`.
- Vendored/External: `src/lib/rplidar_ros/`.
- Deprecated: `config/mower_config.sh.example` and the stub `src/open_mower/config/mower_config.sh.example`.
- Source of truth: `config/mower_config.schema.json` for structured config, and the launch plus param files under `src/open_mower/` for runtime composition.

## Related docs

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [PACKAGES.md](PACKAGES.md)
- [CONFIGURATION.md](CONFIGURATION.md)
- [DOCKER.md](DOCKER.md)
