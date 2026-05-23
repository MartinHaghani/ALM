# VESC maintenance

Purpose: document the verified headless VESC Tool workflow for reading and writing VESC configuration from the Raspberry Pi.

## What this guide is for

This guide covers the current `Mowrator` Pi workflow where:

- the mower runtime normally owns the VESC UARTs
- the official VESC Tool is built on the Pi from upstream source
- maintenance runs over SSH without a GUI by using VESC Tool's CLI and `--offscreen`

This is the supported way for agents and contributors to inspect or change VESC settings without unplugging the controllers from the mower wiring.

## Verification status

- Verified from official VESC Tool source:
  - `README.md` documents the Linux build flow and the expected output under `build/lin/`
  - `main.cpp` exposes the CLI flags `--offscreen`, `--vescPort`, `--getMcConf`, `--setMcConf`, `--getAppConf`, and `--setAppConf`
- Runtime-verified on the rover Pi:
  - the mower runtime can be stopped cleanly before maintenance
  - the official VESC Tool was installed and built on `rpi4.local`
  - headless motor-config reads succeeded on all three mower UARTs:
    - `/dev/ttyAMA5` left drive
    - `/dev/ttyAMA3` right drive
    - `/dev/ttyAMA4` mower blade
  - a headless app-config read succeeded on the right drive ESC over `/dev/ttyAMA3`
  - April 2, 2026 firmware readback on `rpi4.local` showed:
    - left drive `/dev/ttyAMA5`: `FW: V6.02 (no_hw_limits), Hw: 75_300_R2`
    - right drive `/dev/ttyAMA3`: `FW: V6.02 (no_hw_limits), Hw: 75_300_R2`
    - mower blade `/dev/ttyAMA4`: `FW: V6.06 (no_hw_limits), Hw: 75_300_R2`

## Safety rules

- Stop `open_mower_local` before touching VESC configuration. The mower runtime owns the same UARTs.
- Stop the mosquitto sidecar too if you want the Pi fully quiet during maintenance.
- Keep the mower immobilized and the blade disabled while testing or changing ESC config.
- Do not change UART baudrate or app input mode casually; the current mower runtime expects standard VESC UART behavior at `115200`.

## Current `Mowrator` UART map

Observed from the active hardware profile:

- low-level board: `/dev/ttyAMA0`
- left drive ESC: `/dev/ttyAMA5`
- right drive ESC: `/dev/ttyAMA3`
- mower ESC: `/dev/ttyAMA4`

See [Mowrator ESC params](../src/open_mower/params/hardware_specific/Mowrator/comms_xesc_mini_params.yaml) for the profile that currently drives these assignments.

## Current live current-budget profile

As of the latest repo-tracked snapshot in [vesc_configs/](vesc_configs/), the live mower is using the shared-pack "later stronger tune" current budget:

- left drive ESC:
  - `l_current_max=40`
  - `l_current_min=-40`
  - `l_abs_current_max=60`
  - `l_in_current_max=14`
  - `l_in_current_min=0`
- right drive ESC:
  - `l_current_max=40`
  - `l_current_min=-40`
  - `l_abs_current_max=60`
  - `l_in_current_max=14`
  - `l_in_current_min=0`
- mower blade ESC:
  - `l_current_max=50`
  - `l_current_min=-50`
  - `l_abs_current_max=75`
  - `l_in_current_max=20`
  - `l_in_current_min=0`

This profile is intended for a shared `56V` nominal, `18Ah`, `3C` battery pack. If the pack, BMS, or wiring limits change, rework the current budget before raising these values further.

## One-time install on the Pi

From the repo root on the Pi:

```bash
~/open_mower_ros/utils/scripts/vesc/build_vesc_tool.sh
```

Observed behavior of that helper:

- installs the Qt and build dependencies needed by the official VESC Tool source tree
- clones or updates `https://github.com/vedderb/vesc_tool.git` into `~/tools/vesc_tool`
- runs the Linux build path from the official project:

```bash
qmake -config release "CONFIG += release_lin build_original exclude_fw"
make -j"$(nproc)"
```

## Stop the mower runtime first

```bash
~/open_mower_ros/utils/scripts/startup/stop_open_mower_local.sh
docker stop open_mower_mosquitto >/dev/null 2>&1 || true
```

If you skip this step, VESC Tool and the mower stack will fight over the UART device.

## Run VESC Tool headlessly

The repo helper wraps the built binary and always adds `--offscreen`:

```bash
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh --version
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh --help
```

To dump all three ESCs in one pass:

```bash
~/open_mower_ros/utils/scripts/vesc/dump_all_vesc_configs.sh ~/vesc-config-dumps/current
```

The repo keeps the latest known live snapshots under [vesc_configs/](vesc_configs/). If you write a new config to any ESC, refresh that directory in the same change.

The VESC Tool motor wizard can rewrite more than the obvious FOC detection values. After running it, always re-dump both `mcconf` and `appconf` for the affected ESC before reviewing or documenting the result.

## Read motor and app config from an ESC

Create a dump directory on the Pi:

```bash
mkdir -p ~/vesc-config-dumps
```

Read the right drive ESC config:

```bash
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA3 \
  --getMcConf ~/vesc-config-dumps/right-drive-mcconf.xml

~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA3 \
  --getAppConf ~/vesc-config-dumps/right-drive-appconf.xml
```

Equivalent examples for the other ESCs:

```bash
# Left drive
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA5 \
  --getMcConf ~/vesc-config-dumps/left-drive-mcconf.xml

# Mower blade
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA4 \
  --getMcConf ~/vesc-config-dumps/mower-mcconf.xml
```

## Write config back to an ESC

```bash
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA3 \
  --setMcConf ~/vesc-config-dumps/right-drive-mcconf.xml

~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA3 \
  --setAppConf ~/vesc-config-dumps/right-drive-appconf.xml
```

Use the matching UART path for the ESC you are targeting.

## Confirm the dump worked

After a successful read, confirm the files exist and are non-empty:

```bash
ls -lh ~/vesc-config-dumps
sed -n '1,40p' ~/vesc-config-dumps/right-drive-mcconf.xml
sed -n '1,40p' ~/vesc-config-dumps/right-drive-appconf.xml
```

## Recover after a firmware reinstall

If an ESC firmware reinstall wipes its config, use this recovery flow:

1. Stop the mower runtime.
2. Query the ESC firmware first:

```bash
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA4 \
  --queryDeviceFwParams
```

3. If the saved repo snapshot was produced on an older firmware layout, read back fresh XML from the newly flashed controller and use that as the base file shape:

```bash
~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA4 \
  --getAppConf /tmp/mower-app-readback.xml

~/open_mower_ros/utils/scripts/vesc/vesc_tool_cli.sh \
  --vescPort /dev/ttyAMA4 \
  --getMcConf /tmp/mower-mc-readback.xml
```

4. Reapply the mower-specific values to those XMLs, then write them back with `--setAppConf` and `--setMcConf`.
5. Read the config back again and store the final XML under [vesc_configs/](vesc_configs/) so the repo matches the live ESC.

Observed on April 2, 2026:

- both drive VESCs responded as `FW: V6.02 (no_hw_limits), Hw: 75_300_R2`
- the blade VESC responded as `FW: V6.06 (no_hw_limits), Hw: 75_300_R2`
- writing the older saved XML directly worked for key fields, but VESC Tool reported skipped unknown parameters
- the safer recovery path was to read back fresh `6.06` XML, patch the mower-critical fields, and then re-write that updated XML
- the fresh blade `mcconf` readback reported `si_motor_poles=4` and `m_motor_temp_sens_type=0`, so the ROS runtime should use `motor_pole_pairs: 2` and `has_motor_temp: false`
- on April 3, 2026 the left drive snapshot was normalized to `m_motor_temp_sens_type=0` and a sane `foc_temp_comp_base_temp=25` to match the `Mowrator` drive wiring, which does not expose a drive motor temp sensor in ROS

## Restart the mower runtime after maintenance

```bash
~/open_mower_ros/utils/scripts/startup/start_open_mower_local.sh
```

If the mower was previously using the mosquitto sidecar, `start_open_mower_local.sh` will bring it back as part of the standard Pi workflow.

## Troubleshooting

### `Could not find a built VESC Tool binary`

Run:

```bash
~/open_mower_ros/utils/scripts/vesc/build_vesc_tool.sh
```

### ESC read fails on a port

Check in this order:

1. The mower runtime is stopped.
2. The ESC is powered.
3. RX and TX are crossed correctly for that ESC.
4. The ESC is configured for normal VESC UART behavior.
5. You are targeting the correct port:
   - `/dev/ttyAMA5` left drive
   - `/dev/ttyAMA3` right drive
   - `/dev/ttyAMA4` mower

### `Could not load package archive resource`

VESC Tool can print:

```text
Could not load package archive resource. Please update the archive in order to use VESC packages.
```

That warning was observed during successful `--getMcConf` and `--getAppConf` runs on the Pi. It does not block ordinary UART config reads or writes.

### Mower comms break after VESC changes

Most likely causes:

- UART baudrate changed away from `115200`
- app input settings were changed away from the expected UART mode
- a config was written to the wrong ESC

If that happens, stop the mower runtime, read back the config again, and restore a known-good XML file with `--setMcConf` or `--setAppConf`.

## Agent sync rule

If you change live VESC settings, update all of these together:

- [vesc_configs/](vesc_configs/)
- [VESC_MAINTENANCE.md](VESC_MAINTENANCE.md) if the procedure changed
- any hardware-migration doc that depends on the changed values

## Related docs

- [RASPBERRY_PI.md](RASPBERRY_PI.md)
- [MOWRATOR_MIGRATION.md](MOWRATOR_MIGRATION.md)
- [DOCKER.md](DOCKER.md)
- [vesc_configs/README.md](vesc_configs/README.md)
