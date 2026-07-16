# Docker

Purpose: explain the runtime and development container layout that exists in this repository.

## Runtime images under `docker/`

Observed runtime image files:

- `docker/Dockerfile`
- `docker/Dockerfile.Legacy`
- `docker/openmower_entrypoint.sh`
- `docker/openmower_entrypoint.legacy.sh`
- `docker/openmower_entrypoint.pi.sh`
- `docker/assets/mosquitto.conf`
- `docker/assets/nginx.conf`
- `docker/assets/openmower-bashrc.sh`
- `docker/assets/rosconsole.config`

## Default versus legacy images

### Default image

Observed from `docker/Dockerfile`:

- Based on `ros:noetic-ros-base-focal`.
- Does not install nginx or mosquitto.
- Expects the surrounding OSv2 deployment to provide web and MQTT services.
- Builds the workspace inside the image and blacklists `slic3r_coverage_planner` because a prebuilt install is staged separately.
- Copies `docker/openmower_entrypoint.sh` as the entrypoint.
- Runs `roslaunch open_mower open_mower.launch --screen` as the default command.
- The mower-side Bluetooth manager uses the host BlueZ daemon over `/run/dbus/system_bus_socket`. Pi runtime helpers mount that socket when it exists, and the images install `python3-dbus` plus `python3-gi` for D-Bus and pairing-agent support.

### Legacy image

Observed from `docker/Dockerfile.Legacy`:

- Also based on `ros:noetic-ros-base-focal`.
- Installs nginx and mosquitto directly in the image.
- Copies the runtime nginx and mosquitto configs from `docker/assets/`.
- Copies `docker/openmower_entrypoint.legacy.sh` as the entrypoint.
- Starts nginx, starts mosquitto, and then launches `open_mower`.

## What the entrypoints do

### `docker/openmower_entrypoint.sh`

Observed behavior:

- sources `/opt/ros/$ROS_DISTRO/setup.bash`
- sources `/opt/open_mower_ros/devel/setup.bash`
- sources `/opt/open_mower_ros/version_info.env`
- toggles ROS console behavior based on `DEBUG`
- sets line-buffered logging and unbuffered Python output

It does not source `mower_config.sh`. That is consistent with the default image expecting external OSv2 config and service provisioning.

### `docker/openmower_entrypoint.legacy.sh`

Observed behavior:

- sources ROS and workspace setup
- sources `/opt/open_mower_ros/version_info.env`
- sources `/config/mower_config.sh`
- sets `HARDWARE_PLATFORM` based on `OM_V2`
- enables `OM_LEGACY_CONFIG_MODE` when running the older legacy path
- maps `OM_MOWER_ESC_TYPE` into `ESC_TYPE`
- maps `OM_MOWER` into `MOWER`
- sources `open_mower/params/hardware_specific/$MOWER/default_environment.sh`
- sets `RECORDINGS_PATH` and `PARAMS_PATH` to `$HOME`

This file is the key bridge between the deprecated shell config and the runtime parameter-loading logic.

### `docker/openmower_entrypoint.pi.sh`

Observed behavior:

- intended for the plain-Pi local-checkout workflow under `utils/scripts/startup/`
- installs small missing runtime packages when the pulled image lags behind the checked-out workspace
- copies the runtime nginx config from `docker/assets/`
- starts nginx inside the Pi-dev container before launching ROS
- relies on the Pi startup scripts to launch an `eclipse-mosquitto` sidecar with `docker/assets/mosquitto.conf`
- then delegates to `docker/openmower_entrypoint.legacy.sh`

## Build workflow evidence

Observed from `.github/workflows/build-image.yaml`:

- both default and legacy images are built
- both amd64 and arm64 builds are configured
- the default image uses `docker/Dockerfile`
- the legacy image uses `docker/Dockerfile.Legacy`
- pre-commit and repository-policy checks run once in the separate always-triggered
  `Project policy` workflow rather than four times inside the Docker matrix
- third-party workflow actions are pinned to immutable commit SHAs and maintained by
  Dependabot
- pushed image builds request BuildKit `mode=max` provenance and an SBOM; pull
  request validation builds do not publish those attestations
- package-write permission is scoped to the image build/merge jobs rather than the
  entire workflow

## Development-only container setup

### `devenv/`

- `devenv/Dockerfile` builds a ROS Noetic desktop-full development image with SSH, sudo, git, zsh, gdb, and rsync.
- `devenv/docker-compose.yaml` mounts the repo into `/workspace`, forwards X11, and uses host networking.
- `devenv/README.md` documents `./devenv/start_devenv.sh` and `./devenv/attach.sh`.

### `.devcontainer/`

- `.devcontainer/devcontainer.json` points at `../devenv/docker-compose.yaml`.
- The devcontainer is for editor integration and development convenience, not for the runtime image contract.

### `docker/development/docker-compose.yaml`

Observed services:

- `nginx`, serving the checked-in `web/` bundle read-only
- `mosquitto`
- `etherbridge`

This file looks like a companion integration setup for development or testing, not the main runtime image definition.

## Pi-dev image notes

Observed from `docker/Dockerfile.PiDev` and `docker/openmower_entrypoint.pi.sh`:

- The Pi-dev image installs `nginx` directly.
- The Pi-dev entrypoint starts nginx before launching `open_mower`.
- `utils/scripts/startup/start_open_mower_local.sh` also starts an `eclipse-mosquitto:latest` sidecar using `docker/assets/mosquitto.conf`.
- The checked-in `web/` bundle is therefore reachable from the Pi-dev runtime through nginx on port `8080`, with the existing Flutter UI at `/` and the generated React UI at `/next/`.
- `docker/assets/nginx.conf` also needs to serve Flutter `.wasm` assets with `application/wasm`; newer Flutter web bundles under `web/canvaskit/` will blank-screen in the browser if nginx falls back to a generic MIME type.
- `docker/assets/nginx.conf` routes `/next/` to `web/next/index.html` so React routes resolve without interfering with the root Flutter fallback.
- `utils/scripts/web/build_next_webui.sh` builds `webui/` into `web/next/` inside `node:22-bookworm-slim`.
- MQTT is available from the sidecar on port `1883`, and MQTT-over-WebSockets is available from the sidecar on port `9001`.
- `open_mower.launch` also conditionally includes `rosbridge` unless `OM_NO_ROSBRIDGE=True`.
- `open_mower.launch` starts the manual input router, direct gamepad mapper, and Bluetooth gamepad manager by default. Set `OM_NO_DIRECT_GAMEPAD=True` or `OM_NO_BLUETOOTH_GAMEPAD_MANAGER=True` to disable those pieces for a container that does not mount `/dev/input` or the host D-Bus socket.

## Cautions when editing

- Preserve the default versus legacy split. The repo and CI explicitly depend on it.
- Treat entrypoint changes as high-risk because they change config loading, environment variables, and runtime behavior.
- Keep development-container changes separate from runtime image changes when possible.
- If you change a Dockerfile, also check whether `docs/BUILD_AND_RUN.md`, `docs/CONFIGURATION.md`, and `docker/AGENTS.md` need updates.
- The supported plain-Pi workflow now uses the parameterized local-checkout scripts under `utils/scripts/startup/`.
- `utils/scripts/startup/start_open_mower.sh` remains the older image-only helper; do not treat it as the primary Pi dev path.
