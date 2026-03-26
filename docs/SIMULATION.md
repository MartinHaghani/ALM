# Simulation

Purpose: document the simulation support that is verified from the current repo files.

## Verification status

- Verified from repo files: launch files, `mower_simulation` source, `mower_simulation/README.md`, and `mower_utils/launch/planner_test.launch`.
- Not yet runtime-verified in this documentation pass: no ROS launch commands were executed here.

## Verified simulation entrypoints

- `src/open_mower/launch/sim_mower_logic.launch`
- `src/open_mower/launch/sim_navigation.launch`
- `src/mower_simulation/launch/_mower_simulation.launch`
- `src/mower_utils/launch/planner_test.launch`

## What `mower_simulation` does

Observed from `src/mower_simulation/src/mower_simulation.cpp`:

- Starts the `mower_simulation` node.
- Creates low-level xBot services for emergency, diff drive, mower, IMU, power, and GPS.
- Starts a simulated robot object.
- Waits briefly, then queries `mower_map_service/get_docking_point` and uses that pose to initialize the simulated robot.

Observed from `src/mower_simulation/README.md`:

- The package is intended to simulate an OpenMower on a PC without additional hardware.
- The README says it “basically replaces the mower_comms node.”

## Observed launch behavior

### `sim_mower_logic.launch`

Observed composition:

- includes `_params.launch`
- includes `_mower_simulation.launch`
- includes `_move_base.launch`
- includes `_teleop.launch`
- loads `src/open_mower/params/simulation_params.yaml`
- starts RViz with `src/open_mower/rviz/sim_mower_logic.rviz` when `OM_START_RVIZ` is true
- starts `rqt_reconfigure` when `OM_START_RVIZ` is true
- starts `mower_map_service`
- starts `mower_logic`
- starts `slic3r_coverage_planner`
- starts `mower_utils/xbot_pose_converter`
- starts `twist_mux`
- starts `mower_comms_v2`
- starts `xbot_monitoring`, `xbot_remote`, `monitoring`, and conditional `heatmap_generator`

### `sim_navigation.launch`

Observed composition:

- includes `_params.launch`
- includes `_mower_simulation.launch`
- includes `_move_base.launch`
- includes `_teleop.launch`
- starts RViz with `src/open_mower/rviz/sim_navigation_test.rviz`
- starts `mower_map_service`
- starts `twist_mux`

### `planner_test.launch`

Observed composition:

- starts RViz with `src/mower_utils/rviz/planner_test.rviz`
- starts `mower_map_service`
- starts `slic3r_coverage_planner`
- conditionally starts `mower_utils/planner_test`

## How simulation differs from the real mower path

Observed difference:

- The real runtime uses `_comms.launch` to choose between `mower_comms_v1` and `mower_comms_v2`.
- The simulation runtime starts `mower_simulation` and still starts `mower_comms_v2` in `sim_mower_logic.launch`.

Best grounded interpretation:

- `mower_simulation` appears to replace the physical low-level mower service side.
- `mower_comms_v2` remains active as the ROS bridge in the current simulation launch.

That is more precise than simply saying “simulation replaces comms.”

## Inferred versus observed

- Observed: `mower_simulation` exposes the low-level xBot services and initializes itself from the docking point service.
- Observed: the sim launch still includes `mower_comms_v2`.
- Inferred: the simulator is standing in for the hardware-facing service implementation while preserving much of the production ROS graph.

## Safe change notes

- Changes to simulation launches can affect both developer workflows and the assumptions encoded in docs.
- If you change `mower_simulation` behavior, also review `docs/ARCHITECTURE.md`, `docs/PACKAGES.md`, and `docs/BUILD_AND_RUN.md`.
- Be careful not to describe inferred behavior as verified behavior unless you confirm it from code or a real launch.
