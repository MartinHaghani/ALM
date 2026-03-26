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
- then delegates to `docker/openmower_entrypoint.legacy.sh`

## Build workflow evidence

Observed from `.github/workflows/build-image.yaml`:

- both default and legacy images are built
- both amd64 and arm64 builds are configured
- the default image uses `docker/Dockerfile`
- the legacy image uses `docker/Dockerfile.Legacy`
- pre-commit runs as part of the image build workflow before Docker build and push

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

## Cautions when editing

- Preserve the default versus legacy split. The repo and CI explicitly depend on it.
- Treat entrypoint changes as high-risk because they change config loading, environment variables, and runtime behavior.
- Keep development-container changes separate from runtime image changes when possible.
- If you change a Dockerfile, also check whether `docs/BUILD_AND_RUN.md`, `docs/CONFIGURATION.md`, and `docker/AGENTS.md` need updates.
- The supported plain-Pi workflow now uses the parameterized local-checkout scripts under `utils/scripts/startup/`.
- `utils/scripts/startup/start_open_mower.sh` remains the older image-only helper; do not treat it as the primary Pi dev path.
