# Mowrator migration

Purpose: track the staged hardware migration from the current YardForce-based rover profile to the custom `Mowrator` mower build.

## Stage 1: wheel motion bring-up

Stage 1 exists only to prove that the left and right Flipsky 75100 Pro V2 drive ESCs can move the mower through the existing joystick and web teleop path while the mower is in `AREA_RECORDING`.

Current stage-1 assumptions:

- profile name: `Mowrator`
- ESC transport: UART via the existing `xesc_mini` VESC path
- low-level board: `/dev/ttyAMA0`
- low-level stop/lift/tilt sensor inputs are currently disconnected on this rover, so the `Mowrator` profile sets `ignore_low_level_emergency_inputs=true` and ignores low-level emergency flags during bring-up
- left drive ESC: `/dev/ttyAMA5`
- right drive ESC: `/dev/ttyAMA3`
- mower ESC: `/dev/ttyAMA4`
- left and right drive motors: 14 poles, configured as `motor_pole_pairs: 7`
- stage 1 overrides the inherited YardForce right-drive inversion, so both drive ESCs use `invert_direction: false`
- mower blade remains wired in the profile
- measured `Mowrator` chassis values now replace the copied YardForce geometry defaults: `wheel_distance_m=0.58`, `antenna_offset_x=0.72`, `antenna_offset_y=0.0`, `OM_TOOL_WIDTH=0.40`, and the shared costmap footprint uses the current rectangular `Mowrator` outline
- the current plain-Pi runtime still starts in legacy-config mode unless `OM_V2` is enabled, so a Pi-side `~/mower_config.sh` that sources `config/mower_config.sh.example` must explicitly override `OM_TOOL_WIDTH=0.40` for `Mowrator`
- `ticks_per_m` is now calibrated to `374.1` from a 2026-04-05 RTK-backed straight-drive measurement (`6.64 m` run, left/right estimates `372.0` / `376.2`)
- `utils/scripts/vesc/calibrate_ticks_per_m.py` can now estimate the real `ticks_per_m` from a short straight RTK-backed drive instead of leaving the inherited placeholder in place
- the current repo VESC UART path uses `115200` baud, so the Flipsky drive ESC UART settings must match that existing driver expectation
- the `xesc_mini` drive path now rate-limits wheel-duty changes in `mower_comms_v1`; the current `Mowrator` profile starts with `drive_command_ramp_up_seconds=0.8` and `drive_command_ramp_down_seconds=0.25`
- the current `Mowrator` profile also slows VESC state polling to `100 ms` on the drive ESCs and `50 ms` on the blade ESC to avoid UART frame churn during bring-up on the Pi UARTs
- the patched VESC UART driver now drops a timed-out partial frame instead of waiting on it forever, so one truncated read can resynchronize on the next valid VESC frame
- the drive cap now lives in `mower_comms_v1`, so it applies to every movement source: `drive_command_scale=0.3` generally and `mowing_drive_command_scale=0.2` when the high-level state is `MOWING`

Current manual blade-control bring-up:

- `OM_ENABLE_MOWER=true` is required for live blade control
- `OM_RANDOMIZE_MOWER_DIRECTION=false` should be set for `Mowrator` blade bring-up so each enable uses a consistent spin direction
- the blade is controlled through the existing `mower_logic:area_recording/start_manual_mowing` and `stop_manual_mowing` actions
- the `Mowrator` profile currently uses `duty` mode again because the experimental `15A` current-mode bring-up produced slow, unstable blade startup
- current-mode and RPM-mode retuning are still deferred until the blade can start cleanly under load
- the April 2, 2026 live blade VESC readback reported `4` motor poles and no configured motor temperature sensor, so the `Mowrator` profile should use `motor_pole_pairs: 2` and `has_motor_temp: false`
- the RPM target and reported RPM still need real blade-speed validation under load before the final thresholds are trusted
- the current web/gamepad mapping is:
  - left stick drives in `AREA_RECORDING`
  - the remote-control screen has an on-screen press-and-hold blade button
  - `L1 + R1` also runs the blade while held
  - both blade hold paths reuse the active manual-mowing backend on `Mowrator`
- the Flutter source for that behavior lives in the separate `OpenMowerApp` repository; the compiled `web/` bundle in this repo is deployment output only

Recommended stage-1 config:

```bash
export OM_MOWER="Mowrator"
export OM_MOWER_ESC_TYPE="xesc_mini"
export OM_ENABLE_MOWER=true
export OM_RANDOMIZE_MOWER_DIRECTION=false
export OM_AUTOMATIC_MODE=0
```

Bench-test expectations:

- keep the mower immobilized and the drive wheels lifted or otherwise made safe
- start the mower into `AREA_RECORDING`
- verify joystick or web commands still flow to `/ll/cmd_vel`
- verify the left and right ESCs connect on `/dev/ttyAMA5` and `/dev/ttyAMA3`
- verify the mower ESC is visible on `/dev/ttyAMA4`
- verify the blade only runs while the on-screen blade button or `L1 + R1` are held in the remote-control UI
- if a drive wheel spins backward, correct direction in VESC Tool or wiring before changing code
- if a drive ESC does not connect, validate UART and VESC Tool setup before changing the ROS driver path
- if the drive Flipskys stay disconnected while the mower ESC connects, validate that the drive ESCs are powered, not held off by dock or charging state, and configured for normal VESC UART behavior at `115200`
- if a drive wheel direction is wrong, change the `invert_direction` flag in the `Mowrator` profile before touching the shared VESC transport code

## Stage 2: full behavior migration

These items are still intentionally deferred until stage 1 wheel motion is proven.

- calibrate the final `ticks_per_m` from the new drive motors and drivetrain
- re-measure docking and undocking distances if the fork keeps docking; this may become unnecessary if docking is removed later on
- add the final mower blade motor details, including pole pairs and safe RPM thresholds
- revalidate odometry, straight-line tracking, low-speed control, and braking
- revalidate outline recording, mowing overlap, obstacle clearance, and docking behavior
- decide only after stage 1 whether a dedicated ESC label such as `flipsky_75100_v2` is worth adding; do not add it just for the first UART bring-up
