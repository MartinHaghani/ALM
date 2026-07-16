# Contributing

Purpose: practical contributor guidance for this fork of `open_mower_ros`.

## Start here

- Read [docs/README.md](docs/README.md) for the documentation map.
- Read [docs/BUILD_AND_RUN.md](docs/BUILD_AND_RUN.md) before attempting to build or launch.
- Read [docs/CONFIGURATION.md](docs/CONFIGURATION.md) before changing config, params, or environment handling.
- Read [docs/UPSTREAM_SYNC.md](docs/UPSTREAM_SYNC.md) before mixing fork-only changes with upstream sync work.
- Read [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md) and
  [docs/AGENT_OPERATIONS.md](docs/AGENT_OPERATIONS.md) before starting or resuming a
  tracked workstream.

## Environment setup

This repository is a ROS Noetic catkin workspace. The setup commands below are verified from repo files such as `README.md` and Dockerfiles, but they were not executed in this documentation pass.

```bash
rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
catkin_make
source devel/setup.bash
```

To launch the default runtime entrypoint described in the README:

```bash
roslaunch open_mower open_mower.launch
```

For containerized development, the repo also includes:

```bash
./devenv/start_devenv.sh
./devenv/attach.sh
```

## Submodules and external code

- Initialize submodules before assuming the workspace is complete.
- `services/`, `src/lib/ntrip_client`, `src/lib/xbot_driver_gps`, and `src/lib/xbot_framework` are verified git submodules.
- `src/lib/xbot_framework/ext/cpputest` and `src/lib/xbot_framework/ext/ulog` are nested submodules.
- Do not casually reformat or refactor code under `src/lib/`, `services/`, or `web/`.
- If you intentionally edit submodule or generated content, call that out clearly in your summary or PR description.

## Build and run expectations

- Keep commands exact and package-aware. This is a catkin workspace, not a mixed build-system repo.
- Prefer local, package-scoped changes over cross-cutting rewrites.
- Update docs when a launch file, config model, package boundary, or runtime assumption changes.
- If you change behavior that depends on the default versus legacy container split, update [docs/DOCKER.md](docs/DOCKER.md) and [docs/BUILD_AND_RUN.md](docs/BUILD_AND_RUN.md).

## Robotics and safety-sensitive changes

Be especially careful when changing:

- `src/mower_logic`
- `src/mower_comms_v1`
- `src/mower_comms_v2`
- `src/mower_hardware`
- `src/open_mower/launch`
- `src/open_mower/params/hardware_specific`
- `docker/openmower_entrypoint.sh`
- `docker/openmower_entrypoint.legacy.sh`

Expected behavior in those areas is part of the runtime safety envelope. If a change affects mower movement, emergency behavior, docking, undocking, battery handling, GPS handling, or low-level comms, explain the impact and validation steps explicitly.

## Config changes

- Treat `config/mower_config.schema.json` as the authoritative structured config artifact.
- `config/mower_config.sh.example` is deprecated but still expected to stay aligned while it exists.
- `src/open_mower/config/mower_config.sh.example` is a redirect stub, not the real config file.
- When config semantics change, update the schema, the deprecated shell example, and [docs/CONFIGURATION.md](docs/CONFIGURATION.md) together.
- Preserve observed repo drift notes instead of hiding them. Current examples include `ESC_TYPE` versus `OM_MOWER_ESC_TYPE` and the `Sabo` hardware preset under `src/open_mower/params/hardware_specific/`.

## Review and style expectations

- Keep diffs narrow and explain intent clearly.
- Do not invent workflows or commands that are not present in the repo.
- Use relative links in Markdown docs.
- Keep docs high-signal and avoid repeating the same long explanation across files.
- If a path is generated, deprecated, external, or not yet verified, label it that way.
- Use a topic branch and linked GitHub issue for substantive work. Complex or
  multi-session changes also require an active ExecPlan under
  `docs/exec-plans/active/`.
- Use Conventional Commit subjects; substantive commits include rationale,
  validation, and `Refs: #<issue>` in the body.
- Open a draft PR with the repository template. Do not merge, force-push, deploy,
  change live VESC settings, or perform a physical mower test without the required
  human approval.

## What not to edit casually

- `src/lib/**/*`
- `services/**/*`
- `web/**/*`
- hardware-specific params under `src/open_mower/params/hardware_specific/**/*`
- runtime entrypoints under `docker/`

## Docs expectations

Update the docs layer when you change:

- launch composition or runtime wiring
- configuration structure or environment variables
- package inventory or ownership boundaries
- container behavior
- fork-specific divergences that future contributors need to know

Use [docs/DOCS_MAINTENANCE.md](docs/DOCS_MAINTENANCE.md) to decide which file needs an update.

## Fork and upstream expectations

- This checkout has both `origin` and `upstream` remotes configured.
- Keep upstream sync work isolated from fork-local customization.
- Preserve clean commit boundaries between sync work and local changes.
- Record meaningful fork-specific drift in the docs instead of leaving it tribal knowledge.
