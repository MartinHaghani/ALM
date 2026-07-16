# Source tree guide

Purpose: guidance for editing the catkin workspace under `src/`.

## Package inventory summary

- `open_mower`: orchestration package with launch files, params, and RViz configs.
- `mower_logic`: high-level mower behavior and monitoring. Safety-critical.
- `mower_hardware`: supported Mowrator direct hardware bridge. Safety-critical.
- `mower_comms_v1` and `mower_comms_v2`: low-level comms bridges. Safety-critical.
- `mower_map`: map service and map persistence.
- `mower_msgs`: shared mower messages and services.
- `mower_simulation`: simulator-side low-level services.
- `mower_utils`: helper binaries and test-style launch assets.
- `lib/`: mixed external, vendored, and shared library packages, including the vendored Slamtec `rplidar_ros` driver.

## Working rules

- Respect ROS package boundaries. Prefer local package changes over cross-package rewrites.
- Verify launch and package usage before moving interfaces or renaming nodes.
- Treat `lib/` as mixed external territory. Do not do broad formatting or cleanup there.
- Treat `lib/rplidar_ros` as vendored third-party source unless a task explicitly targets the C1 driver itself.
- Treat `mower_logic`, `mower_hardware`, `mower_comms_*`, and `open_mower/launch` as safety-sensitive.

## Documentation rule

- If package behavior, package ownership, or package interfaces change, update [../docs/PACKAGES.md](../docs/PACKAGES.md).
- If launch composition changes, update [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md) and [../docs/BUILD_AND_RUN.md](../docs/BUILD_AND_RUN.md).
