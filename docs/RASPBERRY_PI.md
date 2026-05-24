# Raspberry Pi

Purpose: get this repo running on a fresh Raspberry Pi OS host with a fast manual pull, rebuild, and restart workflow for hardware testing.

## What this guide is for

This guide targets:

- a fresh Raspberry Pi running Raspberry Pi OS
- SSH access as the main control path
- a checked-out local repo on the Pi
- manual `git pull` plus rebuild and restart
- the repo’s legacy-style container startup path, because a plain Pi does not provide the OSv2 web and MQTT services expected by `docker/Dockerfile`

In the current Pi-dev workflow, the main container provides `nginx` and `rosbridge`, and the startup script launches a host-networked `eclipse-mosquitto` sidecar so the rover can expose the checked-in web bundle directly from the Pi.

This guide does not assume OpenMower OS v2 is already installed on the Pi.

For the current custom `Mowrator` bench hardware sequence, use [MOWRATOR_BENCH_BRINGUP.md](MOWRATOR_BENCH_BRINGUP.md) before connecting ESCs or motors.

## Recommended layout on the Pi

- repo checkout: `~/open_mower_ros`
- config file: `~/mower_config.sh`
- ROS logs: `~/.ros`
- Mowrator battery voltage CSV log: `~/.ros/battery_voltage_log.csv`

The startup scripts under `utils/scripts/startup/` now default to exactly that layout.

## Battery cutoff voltage log

For `MOWER=Mowrator`, `open_mower.launch` starts a separate battery-voltage logger by default. It subscribes to `/hw/power`, appends one CSV row per second, and fsyncs every row so the most recent left drive, right drive, and mower/blade ESC voltages should survive a sudden BMS power cutoff. The active CSV rotates at 10 MiB and keeps 5 files by default, capping the log set at about 50 MiB.

In the plain-Pi workflow, the container path `/root/.ros/battery_voltage_log.csv` is bind-mounted to the host as `~/.ros/battery_voltage_log.csv`. After recharging and rebooting, inspect the final complete rows:

```bash
tail -n 40 ~/.ros/battery_voltage_log.csv
```

Use `OM_BATTERY_VOLTAGE_LOG_PERIOD_SEC`, `OM_BATTERY_VOLTAGE_LOG_PATH`, `OM_BATTERY_VOLTAGE_LOG_FSYNC`, `OM_BATTERY_VOLTAGE_LOG_MAX_BYTES`, `OM_BATTERY_VOLTAGE_LOG_MAX_FILES`, or `OM_NO_BATTERY_VOLTAGE_LOG` in `~/mower_config.sh` only if you need to change the default logger behavior.

## First boot checklist

These are host-side setup steps, not repo-provided commands.

1. Install Raspberry Pi OS and enable SSH before or during first boot.
2. Install git and Docker on the Pi.
3. Make sure your user can run Docker commands.
4. SSH into the Pi and clone this repo:

```bash
git clone https://github.com/MartinHaghani/open_mower_ros.git ~/open_mower_ros
cd ~/open_mower_ros
git remote add upstream https://github.com/ClemensElflein/open_mower_ros.git
git submodule update --init --recursive
```

Optional hardening for the manual-update workflow:

```bash
git config pull.ff only
```

## Host Wi-Fi profile notes

Wi-Fi credentials for the Pi live on the Raspberry Pi OS host, not in this repo and not in `~/mower_config.sh`. On a default Raspberry Pi OS setup, NetworkManager stores the saved SSID profile under `/etc/NetworkManager/system-connections/`.

For a mesh network, keep the saved profile free of a fixed BSSID so the Pi can roam between extenders. If outdoor range and consistency matter more than peak throughput, prefer a 2.4 GHz-only client profile:

```bash
sudo nmcli connection modify "YOUR_SSID" 802-11-wireless.band bg
sudo nmcli connection down "YOUR_SSID"
sudo nmcli connection up "YOUR_SSID"
```

Notes:

- `connection.autoconnect=yes` keeps the saved profile reconnecting on the next boot.
- leaving `802-11-wireless.bssid` unset keeps roaming mesh-friendly instead of pinning the Pi to one extender.
- use the host Wi-Fi profile for SSID, password, and band selection; do not expect runtime mower config files in this repo to manage host networking.

## Create the Pi config file

The legacy-style plain-Pi workflow expects a config file at `~/mower_config.sh`.

```bash
cp ~/open_mower_ros/config/mower_config.sh.example ~/mower_config.sh
```

Then edit `~/mower_config.sh` for your mower and hardware.

This guide intentionally uses the legacy-style config mount `/config/mower_config.sh`, because that is what `docker/openmower_entrypoint.legacy.sh` expects on a plain Pi workflow.

## Pi runtime image

The supported plain-Pi workflow now defaults to a local image tag:

- `open_mower_ros:pi-dev`

That image is built from `docker/Dockerfile.PiDev`, which derives from the official `ros:noetic-ros-base-focal` image and installs this repo's ROS dependencies locally on the Pi.

Why this matters:

- the prebuilt arm64 image `ghcr.io/clemenselflein/open_mower_ros:releases-testing` was observed to make even a minimal `roscpp` process grow until the kernel killed it on this rover Pi
- a control test on the same Pi using the official `ros:noetic-ros-core-focal` image stayed stable
- the Pi workflow therefore defaults to the local `open_mower_ros:pi-dev` image and only uses the GHCR image if you explicitly override `OPEN_MOWER_IMAGE`

Build the local Pi image once before the first compile, or let the other Pi scripts build it automatically if it is missing:

```bash
~/open_mower_ros/utils/scripts/startup/build_open_mower_pi_image.sh
```

## Correction source options

The rover can receive RTCM corrections in two file-verified ways:

- NTRIP via the built-in `ntrip_client` launch path
- raw RTCM 3 over TCP via `open_mower/scripts/rtcm_tcp_bridge.py`

For a LAN-only raw TCP source, disable NTRIP and set:

```bash
export OM_USE_NTRIP=False
export OM_USE_RTCM_TCP=True
export OM_RTCM_TCP_HOSTNAME="rp3s.local"
export OM_RTCM_TCP_PORT=5016
```

For a normal NTRIP source, leave `OM_USE_RTCM_TCP` unset or false and configure the `OM_NTRIP_*` values instead.

## Supported startup scripts

The supported Pi workflow scripts live under `utils/scripts/startup/`:

- `compile_open_mower.sh`
- `start_open_mower_local.sh`
- `stop_open_mower_local.sh`
- `logs_open_mower_local.sh`
- `pull_build_restart_open_mower.sh`
- `docker_shell.sh`

## Script contract

These scripts support the same environment variables:

- `OPEN_MOWER_IMAGE`
  - default: `open_mower_ros:pi-dev`
  - override this only if you intentionally want a different runtime image
- `OPEN_MOWER_MQTT_IMAGE`
  - default: `eclipse-mosquitto:latest`
- `OPEN_MOWER_MQTT_CONFIG`
  - default: `$OPEN_MOWER_REPO_DIR/docker/assets/mosquitto.conf`
- `OPEN_MOWER_REPO_DIR`
  - default: repo root derived from the script location
- `OPEN_MOWER_CONFIG_FILE`
  - default: `$HOME/mower_config.sh`
- `OPEN_MOWER_ROS_HOME`
  - default: `$HOME/.ros`
- `OPEN_MOWER_CONTAINER_NAME`
  - default: `open_mower_local`
- `OPEN_MOWER_MQTT_CONTAINER_NAME`
  - default: `open_mower_mosquitto`

They also intentionally:

- use host networking
- mount `/dev` into the container instead of hardcoding a short `ttyAMA` list
- mount the local repo into `/opt/open_mower_ros`
- mount the legacy config file into `/config/mower_config.sh`
- start the local-checkout runtime through `docker/openmower_entrypoint.pi.sh` from the mounted repo
- let `docker/openmower_entrypoint.pi.sh` install small missing runtime libraries before delegating to `docker/openmower_entrypoint.legacy.sh`
- serve the checked-in `web/` bundle through nginx on port `8080`
- start an `eclipse-mosquitto` sidecar with host networking so MQTT is available on port `1883` and MQTT-over-WebSockets is available on port `9001`
- start `rosbridge` by default through `open_mower.launch` unless `OM_NO_ROSBRIDGE=True`
- generate a local `version_info.env` in the repo root when the bind-mounted checkout does not already have one

## First run on the Pi

From the repo root or any shell:

```bash
~/open_mower_ros/utils/scripts/startup/build_open_mower_pi_image.sh
~/open_mower_ros/utils/scripts/startup/compile_open_mower.sh
~/open_mower_ros/utils/scripts/startup/start_open_mower_local.sh
```

If you want a shell inside the mounted runtime image:

```bash
~/open_mower_ros/utils/scripts/startup/docker_shell.sh
```

If you want logs from an already running container:

```bash
~/open_mower_ros/utils/scripts/startup/logs_open_mower_local.sh
```

Expected service endpoints after startup:

- `http://rpi4.local:8080/` for the checked-in web bundle
- `ws://rpi4.local:9090/` for rosbridge
- `ws://rpi4.local:9002/` for `xbot_remote`
- `mqtt://rpi4.local:1883` for MQTT
- `ws://rpi4.local:9001/` for MQTT-over-WebSockets

Current `Mowrator` web/gamepad mapping:

- the editable source of the served UI lives in the separate `OpenMowerApp` Flutter repository; do not hand-edit the compiled `web/` bundle for feature work
- in `AREA_RECORDING`, the left stick drives without a gamepad deadman
- the remote-control screen exposes a press-and-hold blade button
- holding `L1 + R1` also runs the blade while held
- both blade hold paths reuse the active mower profile's blade control mode
- the hold path in `mower_logic` now exposes a stable `start_manual_mowing` / `stop_manual_mowing` pair to the WebUI and gamepad, with a brief 50 ms-polled stop-chatter guard so noisy start/stop bursts do not toggle the blade during a hold
- blade enable changes from manual hold are applied immediately in `mower_logic`, so they do not wait for the 0.5 s safety timer
- the blade command is no longer tied to the drive `cmd_vel` timeout path in `mower_hardware`
- in duty mode, `mower_hardware` ramps the blade command up instead of stepping straight to full duty in one cycle; the current `Mowrator` profile keeps the dead-start path duty-first with a full-duty startup floor because the live blade responds more cleanly to direct duty than to a Pi-side startup current override
- on blade disable, `mower_hardware` can send a short VESC brake-current pulse before returning to zero command, which sharpens blade spin-down without changing the steady-state control mode
- blade hold-control only works if the active rover config has `OM_ENABLE_MOWER=true`
- the `Mowrator` runtime now uses `/hw` and `mower_hardware`; the OpenMower low-level board, sound/rain/UI board, charger-contact sensing, and low-level stop/lift/tilt inputs are not modeled
- drive ESC `DISCONNECTED` or `ERROR` status now latches emergency; a left/right drive voltage mismatch over 1.0V warns but does not emergency-stop
- if a map exists but no docking point has been recorded yet, `mower_logic` now stays in `IDLE` instead of force-jumping back into `AREA_RECORDING`; low battery with no docking point stops blade/motion and idles
- the `IDLE` behavior now latches `start_mowing` and `start_area_recording` requests with atomics before the main behavior loop consumes them, which avoids flaky lost WebUI or MQTT action presses on the Pi
- the normal undocked `IDLE` state now keeps GPS enabled so RTK stays warm before a mowing start; only the docked idle variant disables GPS
- in `IDLE`, the start-mowing action is now only enabled when GPS is actually usable (or GPS errors are explicitly ignored); this prevents a manual start from immediately falling into a failed mow-plan attempt and then docking retries on a bad fix
- if `MowingBehavior` finishes or aborts on a rover with no recorded docking point, it now returns to `IDLE` instead of entering parking; this keeps no-dock Mowrator test loops local to mowing and localization rather than falling into impossible park retries

## VESC maintenance

For headless VESC Tool install, config dumps, and safe UART maintenance from the Pi, use [VESC_MAINTENANCE.md](VESC_MAINTENANCE.md).

To stop the runtime container:

```bash
~/open_mower_ros/utils/scripts/startup/stop_open_mower_local.sh
```

## Daily development loop

For the normal “change code, test on Pi” loop, use:

```bash
~/open_mower_ros/utils/scripts/startup/pull_build_restart_open_mower.sh
```

Observed behavior:

- it refuses to pull if tracked files or submodules have local modifications
- it runs `git pull --ff-only`
- it refreshes submodules
- it rebuilds in the container
- it restarts the runtime container

This is intentionally manual. There is no unattended background `git pull`.

## Optional boot-time runtime service

If you want the Pi to start the local checkout automatically on boot, use the example service at:

- `utils/systemd/open_mower_local.service.example`

One straightforward install flow is:

```bash
sudo cp ~/open_mower_ros/utils/systemd/open_mower_local.service.example /etc/systemd/system/open_mower_local.service
sudo editor /etc/systemd/system/open_mower_local.service
sudo systemctl daemon-reload
sudo systemctl enable --now open_mower_local.service
```

Replace every `<user>` placeholder before enabling the service.

To inspect the service after boot:

```bash
sudo systemctl status open_mower_local.service
sudo journalctl -u open_mower_local.service -f
```

This service is only for boot-time process management. Keep updates manual by continuing to use `pull_build_restart_open_mower.sh`.

## Important limitations

- This guide does not claim the OSv2 provisioning model for web, MQTT, or YAML parameter injection on a plain Pi host.
- The default image under `docker/Dockerfile` still expects external OSv2 services and is not the first bring-up path for this guide.
- `utils/scripts/startup/start_open_mower.sh` remains an older image-only helper. For Pi development, prefer the local-checkout scripts listed above.

## Related docs

- [BUILD_AND_RUN.md](BUILD_AND_RUN.md)
- [DOCKER.md](DOCKER.md)
- [CONFIGURATION.md](CONFIGURATION.md)
