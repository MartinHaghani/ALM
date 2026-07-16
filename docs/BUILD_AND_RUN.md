# Build and run

Purpose: document the build, launch, and container workflows that are verified from repo files.

## Verification status

- Verified from repo files: commands and paths in this document are grounded in `README.md`, `docker/Dockerfile*`, `devenv/README.md`, helper scripts, launch files, and GitHub workflow config.
- Not runtime-verified in this documentation pass: `rosdep`, `catkin_make`, and `roslaunch` are not installed in the current environment, so the commands below are file-verified rather than execution-verified.

## Prerequisites referenced by the repo

- ROS Noetic, referenced in `README.md` and the runtime images.
- `python3-rosdep`, referenced in `README.md`.
- git submodules, referenced in `README.md` and `.gitmodules`.
- Docker, referenced by `docker/`, `devenv/`, `.devcontainer/`, and helper scripts.

## Local catkin workspace flow

### Install and initialize rosdep

From `README.md`:

```bash
sudo apt install python3-rosdep
sudo rosdep init
```

### Fetch dependencies

Run from the repo root:

```bash
rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
```

### Build the workspace

```bash
catkin_make
source devel/setup.bash
```

### Launch the main runtime

```bash
roslaunch open_mower open_mower.launch
```

For `MOWER=Mowrator`, the launch flow also starts the battery-voltage CSV logger by default. It writes one row per second to `~/.ros/battery_voltage_log.csv` unless `OM_BATTERY_VOLTAGE_LOG_PATH` overrides the location, rotates at 10 MiB x 5 files by default, and can be disabled with `OM_NO_BATTERY_VOLTAGE_LOG=True`.

## Configuration note before launching

The root README and maintained workflows use the real deprecated shell example:

```bash
config/mower_config.sh.example
```

The similarly named file under `src/open_mower/config/` is only a compatibility
redirect. See [CONFIGURATION.md](CONFIGURATION.md) before relying on legacy shell
configuration or OSv2 YAML and env loading.

## Development container workflows

### `devenv/`

`devenv/README.md` documents two helper commands:

```bash
./devenv/start_devenv.sh
./devenv/attach.sh
```

Observed behavior:

- `devenv/Dockerfile` builds a ROS Noetic desktop-full development image.
- `devenv/docker-compose.yaml` mounts the repo into `/workspace`, forwards X11, and runs the container with host networking.
- This environment is for development tooling such as `catkin_make`, RViz, IDE usage, and debugging. It is not the runtime image layout described in `docker/`.

### `.devcontainer/`

Observed from `.devcontainer/devcontainer.json`:

- The devcontainer points at `../devenv/docker-compose.yaml`.
- The workspace folder is `/workspace`.
- The post-create command sources ROS Noetic in the shell profile.

## Runtime Docker workflows

### Default image

Observed from `docker/Dockerfile` and `.github/workflows/build-image.yaml`:

- The default image uses `docker/Dockerfile`.
- It builds the ROS stack and expects OSv2 or external system services to provide web and MQTT.
- The default container entrypoint is `docker/openmower_entrypoint.sh`.
- The default container command runs:

```bash
roslaunch open_mower open_mower.launch --screen
```

### Legacy image

Observed from `docker/Dockerfile.Legacy` and `.github/workflows/build-image.yaml`:

- The legacy image uses `docker/Dockerfile.Legacy`.
- It embeds nginx and mosquitto because older OS v1 installs do not provide them externally.
- The legacy container entrypoint is `docker/openmower_entrypoint.legacy.sh`.
- The legacy container command starts nginx, starts mosquitto, and then launches `open_mower`.

## Raspberry Pi workflow

For a plain Raspberry Pi OS bring-up with a local checkout on the Pi, use [RASPBERRY_PI.md](RASPBERRY_PI.md).

The supported Pi scripts are:

- `utils/scripts/web/build_next_webui.sh`
- `utils/scripts/startup/build_open_mower_pi_image.sh`
- `utils/scripts/startup/compile_open_mower.sh`
- `utils/scripts/startup/start_open_mower_local.sh`
- `utils/scripts/startup/stop_open_mower_local.sh`
- `utils/scripts/startup/logs_open_mower_local.sh`
- `utils/scripts/startup/pull_build_restart_open_mower.sh`
- `utils/scripts/startup/docker_shell.sh`

Observed plain-Pi detail:

- `build_next_webui.sh` runs `npm ci`, `npm run typecheck`, and `npm run build` inside `node:22-bookworm-slim`, generating `web/next/` from the React source under `webui/`.
- `build_open_mower_pi_image.sh` builds the local `open_mower_ros:pi-dev` image from `docker/Dockerfile.PiDev`.
- `docker/Dockerfile.PiDev` derives from `ros:noetic-ros-base-focal`, installs this repo's apt-level ROS dependencies, and includes `nginx` for the plain-Pi workflow.
- `start_open_mower_local.sh` launches through the repo's `docker/openmower_entrypoint.pi.sh` inside the bind-mounted checkout.
- `docker/openmower_entrypoint.pi.sh` installs small missing runtime libraries when needed, starts `nginx`, then delegates to `docker/openmower_entrypoint.legacy.sh` so legacy `mower_config.sh` values become the `MOWER`, `ESC_TYPE`, and related runtime environment expected by `open_mower.launch`.
- `start_open_mower_local.sh` also starts an `eclipse-mosquitto:latest` sidecar with host networking and `docker/assets/mosquitto.conf`.
- `open_mower.launch` now includes `rosbridge` by default in this workflow unless `OM_NO_ROSBRIDGE=True`.
- The Mowrator comms launch publishes diagnostic temperature topics for Pi CPU temperature at `/hw/pi/temperature`, LSM6DSO IMU die temperature at `/hw/imu/temperature`, and u-blox `UBX-MON-SYS` receiver temperature at `/hw/position/gps/temperature` when UBX GPS telemetry is available.
- The Pi startup helper bind-mounts `/run/dbus/system_bus_socket` when it exists so the mower-side Bluetooth manager can use host BlueZ. Enable the host service with `sudo systemctl enable --now bluetooth.service` before pairing controllers from `/next/`.
- `open_mower.launch` includes passive C1 SLAM only when `OM_USE_PASSIVE_SLAM=True`; that launch path now starts the passive SLAM manager, SLAM-only odometry helper, and GPS/LIDAR alignment helper for `/next/` visualization. Clear old mower/SLAM maps after deploying swept-footprint recording and rerecord mowing outlines by driving the mower footprint around the boundary. The area recorder defaults to `/localization_fusion/pose` for saved geometry, can fall back to legacy GPS positioning from `/next/`, skips new swept geometry while the selected pose is stale or outside its configured quality limits, and only emits GPS/LIDAR alignment samples during RTK-fixed GPS-truth intervals.
- `open_mower.launch` starts the read-only localization confidence monitor by default with `OM_USE_LOCALIZATION_CONFIDENCE=True`. It publishes `/localization_confidence/status` for `/next/` display only and does not affect mower localization, planning, costmaps, or control.
- `open_mower.launch` starts the passive manual path recorder only when `OM_ENABLE_MANUAL_PATH_RECORDER=True`. Path capture is controlled from `/next/`; optional raw bag capture is selected per session and is stopped with the capture.
- `docker/assets/nginx.conf` serves the existing Flutter UI at `/` and the generated React UI at `/next/`.

## Development companion services under `docker/`

Observed from `docker/development/docker-compose.yaml`:

- `nginx` serves content from `../../web` using `docker/assets/nginx.conf`.
- `mosquitto` uses `docker/assets/mosquitto.conf`.
- `etherbridge` is a separate helper container with host networking and device access.

This compose file is a development or integration helper, not the primary runtime image definition.

## Simulation entrypoints

Verified simulation entrypoints include:

- `src/open_mower/launch/sim_mower_logic.launch`
- `src/open_mower/launch/sim_navigation.launch`
- `src/mower_simulation/launch/_mower_simulation.launch`
- `src/mower_utils/launch/planner_test.launch`

See [SIMULATION.md](SIMULATION.md) for the observed behavior and caveats.

## Older image-only helper

`utils/scripts/startup/start_open_mower.sh` still exists as an older image-only helper.

For Raspberry Pi development on a local checkout, prefer the parameterized local-repo scripts documented in [RASPBERRY_PI.md](RASPBERRY_PI.md).

## Not yet verified from execution

Do not claim the following without a live runtime check:

- the exact host-side config file layout expected by an OSv2 deployment
- whether every helper script still matches the current image tags and runtime layout
- whether the local host already satisfies ROS Noetic dependencies
