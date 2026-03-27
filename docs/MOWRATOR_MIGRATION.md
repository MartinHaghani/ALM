# Mowrator migration

Purpose: track the staged hardware migration from the current YardForce-based rover profile to the custom `Mowrator` mower build.

## Stage 1: wheel motion bring-up

Stage 1 exists only to prove that the left and right Flipsky 75100 Pro V2 drive ESCs can move the mower through the existing joystick and web teleop path while the mower is in `AREA_RECORDING`.

Current stage-1 assumptions:

- profile name: `Mowrator`
- ESC transport: UART via the existing `xesc_mini` VESC path
- low-level board: `/dev/ttyAMA0`
- left drive ESC: `/dev/ttyAMA5`
- right drive ESC: `/dev/ttyAMA3`
- mower ESC: `/dev/ttyAMA4`
- left and right drive motors: 14 poles, configured as `motor_pole_pairs: 7`
- mower blade remains wired in the profile, but stage 1 keeps `OM_ENABLE_MOWER=false`
- geometry, wheel calibration, and GPS antenna offsets remain at the current `YardForce500` values for now
- the current repo VESC UART path uses `115200` baud, so the Flipsky drive ESC UART settings must match that existing driver expectation

Recommended stage-1 config:

```bash
export OM_MOWER="Mowrator"
export OM_MOWER_ESC_TYPE="xesc_mini"
export OM_ENABLE_MOWER=false
export OM_AUTOMATIC_MODE=0
```

Bench-test expectations:

- keep the mower immobilized and the drive wheels lifted or otherwise made safe
- start the mower into `AREA_RECORDING`
- verify joystick or web commands still flow to `/ll/cmd_vel`
- verify the left and right ESCs connect on `/dev/ttyAMA5` and `/dev/ttyAMA3`
- verify the mower ESC may be visible on `/dev/ttyAMA4`, but the blade does not spin because mowing is disabled
- if a drive wheel spins backward, correct direction in VESC Tool or wiring before changing code
- if a drive ESC does not connect, validate UART and VESC Tool setup before changing the ROS driver path
- if the drive Flipskys stay disconnected while the mower ESC connects, validate that the drive ESCs are powered, not held off by dock or charging state, and configured for normal VESC UART behavior at `115200`

## Stage 2: full behavior migration

These items are intentionally deferred until stage 1 wheel motion is proven.

- measure the final wheel center-to-center distance and replace the temporary `wheel_distance_m`
- calibrate the final `ticks_per_m` from the new drive motors and drivetrain
- measure the final GPS antenna offsets and replace the temporary `antenna_offset_x` and `antenna_offset_y`
- measure the final mower footprint and update costmap radius or footprint if the chassis size changed enough to matter
- update `OM_TOOL_WIDTH` to match the real blade or deck geometry
- re-measure docking and undocking distances if the new chassis changes bumper-to-axle or antenna-to-dock geometry
- add the final mower blade motor details, including pole pairs and safe RPM thresholds
- revalidate odometry, straight-line tracking, low-speed control, and braking
- revalidate outline recording, mowing overlap, obstacle clearance, and docking behavior
- decide only after stage 1 whether a dedicated ESC label such as `flipsky_75100_v2` is worth adding; do not add it just for the first UART bring-up
