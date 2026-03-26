# Docker guide

Purpose: guidance for editing runtime and development container files under `docker/`.

## Current split

- `Dockerfile`: default runtime image for newer OSv2-style deployments.
- `Dockerfile.Legacy`: legacy runtime image with nginx and mosquitto inside the container.
- `openmower_entrypoint.sh`: default runtime entrypoint.
- `openmower_entrypoint.legacy.sh`: legacy runtime entrypoint and compatibility bridge for shell config.
- `development/docker-compose.yaml`: development or integration helper services, not the primary runtime image definition.

## Working rules

- Preserve the default versus legacy behavior split unless the task explicitly changes the support model.
- Treat entrypoints as high-risk because they control config loading, runtime environment, and startup behavior.
- Distinguish runtime-image changes from development-container changes.
- If runtime behavior changes, update [../docs/DOCKER.md](../docs/DOCKER.md) and [../docs/BUILD_AND_RUN.md](../docs/BUILD_AND_RUN.md).
