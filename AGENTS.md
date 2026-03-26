# Repository guide

Purpose: operational guidance for Codex and other repo-aware agents working in this `open_mower_ros` fork.

This repository is a ROS Noetic catkin workspace for OpenMower. It combines launch/orchestration under `src/open_mower`, first-party mower packages under `src/`, external libraries and submodules under `src/lib/`, runtime and development container assets under `docker/`, `devenv/`, and `.devcontainer/`, config artifacts under `config/`, shared xBot service-definition JSON under `services/`, and an observed built web bundle under `web/`.

## Repo map

- `src/open_mower`: main orchestration package with launch files, params, hardware presets, and RViz configs.
- `src/mower_logic`: high-level mower state machine and monitoring node. Treat as safety-sensitive.
- `src/mower_comms_v1` and `src/mower_comms_v2`: low-level comms bridges. Treat as safety-sensitive.
- `src/mower_map`: map service, map persistence, occupancy-grid publication, RPC entrypoints.
- `src/mower_simulation`: simulation-side low-level service implementation.
- `src/mower_msgs`: repo-specific ROS messages and services.
- `src/mower_utils`: helper binaries such as `planner_test` and `xbot_pose_converter`.
- `src/lib`: mixed third-party, vendored, and submodule code. Do not treat as first-party by default.
- `services`: xBot service-definition JSON files used by `mower_comms_v2` and `mower_simulation`. External/Submodule.
- `config`: config schema and deprecated shell example. `config/mower_config.schema.json` is the structured source of truth.
- `docker`: runtime images and entrypoints.
- `devenv` and `.devcontainer`: development-only container setup.
- `web`: observed compiled web bundle. Avoid hand-editing unless the change is intentional and documented.
- `utils`: helper scripts for startup, debugging, firmware upload, and button actions.

## Verified commands

These commands are verified from repo files such as `README.md`, `docker/Dockerfile*`, `devenv/README.md`, and launch files. They were not executed in this documentation pass.

### Local workspace setup

```bash
rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
catkin_make
source devel/setup.bash
roslaunch open_mower open_mower.launch
```

### Development container helpers

```bash
./devenv/start_devenv.sh
./devenv/attach.sh
```

## Safe working rules

- Verify from repo files before assuming behavior, paths, or commands.
- Keep ROS package boundaries intact. Do not introduce a different build system or flatten package ownership.
- Treat `src/lib/ntrip_client`, `src/lib/xbot_driver_gps`, `src/lib/xbot_framework`, and `services` as submodule/external boundaries unless the task explicitly targets them.
- Treat nested `src/lib/xbot_framework/ext/cpputest` and `src/lib/xbot_framework/ext/ulog` as nested submodules.
- Do not do broad formatting or cleanup edits in `src/lib/`, `services/`, or `web/`.
- Treat `web/` as generated/build output unless direct evidence in the repo says otherwise.
- Treat `mower_logic`, `mower_comms_*`, launch wiring, hardware-specific params, and container entrypoints as safety-sensitive.
- `config/mower_config.schema.json` is the authoritative structured config artifact. If config semantics change, update the schema, the deprecated shell example, and docs together.
- `config/mower_config.sh.example` is deprecated but still expected to stay aligned while it exists.
- `src/open_mower/config/mower_config.sh.example` is only a stub redirect. Do not expand it into the real config file.
- Keep fork-specific changes easy to separate from upstream sync work. Prefer narrow, well-described edits.

## Directory-specific notes

### `src/`

- Start with [docs/PACKAGES.md](docs/PACKAGES.md), [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), and [src/AGENTS.md](src/AGENTS.md).
- `src/open_mower/launch/open_mower.launch` is the primary runtime composition point.
- `src/open_mower/launch/include/_params.launch` is the key source for config loading behavior.
- `src/open_mower/params/hardware_specific/` contains mower presets. An observed `Sabo` preset exists here even though it is not exposed in the root schema/example.

### `config/`

- Start with [docs/CONFIGURATION.md](docs/CONFIGURATION.md) and [config/AGENTS.md](config/AGENTS.md).
- Watch the observed mismatch between `ESC_TYPE` in the schema and `OM_MOWER_ESC_TYPE` in the deprecated shell example plus legacy entrypoint.

### `docker/`

- Start with [docs/DOCKER.md](docs/DOCKER.md) and [docker/AGENTS.md](docker/AGENTS.md).
- Preserve the default versus legacy image split.
- Do not casually change `docker/openmower_entrypoint.sh` or `docker/openmower_entrypoint.legacy.sh`; they define runtime assumptions.

### `web/`

- Start with [web/AGENTS.md](web/AGENTS.md).
- The directory currently contains compiled Flutter-style assets such as `main.dart.js`, `flutter.js`, and asset manifests.

## Verification checklist

### Docs-only changes are done when

- New or updated docs match observed repo structure and paths.
- Relative links resolve.
- Deprecated, generated, external, and not-yet-verified areas are labeled explicitly.
- `AGENTS.md`, `CLAUDE.md`, `.claude/rules/`, and detailed docs do not contradict one another.

### Code changes are done when

- The relevant package, launch, config, or container docs are updated with the same change.
- Verification is proportional to risk. At minimum, confirm referenced paths and commands still exist.
- Safety-sensitive changes call out risk and validation clearly in the summary.
- Submodule or generated-area edits are explicit and justified.

## Fork maintenance

- This fork has both `origin` and `upstream` remotes configured.
- Keep upstream sync work isolated from fork-specific customization.
- When a fork-only behavior matters to contributors or agents, document it in [docs/UPSTREAM_SYNC.md](docs/UPSTREAM_SYNC.md) or the nearest relevant reference doc.
- Do not silently normalize observed drift such as hardware presets or config naming mismatches. Document it.

## Documentation map

- [CLAUDE.md](CLAUDE.md): Claude Code startup guidance.
- [CONTRIBUTING.md](CONTRIBUTING.md): human contributor guide.
- [docs/README.md](docs/README.md): doc index and reading order.
- [docs/REPO_MAP.md](docs/REPO_MAP.md): directory structure and ownership boundaries.
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): launch-composed runtime and package roles.
- [docs/BUILD_AND_RUN.md](docs/BUILD_AND_RUN.md): build, launch, container, and dev-environment flows.
- [docs/CONFIGURATION.md](docs/CONFIGURATION.md): schema, legacy shell config, YAML/env loading, and drift notes.
- [docs/PACKAGES.md](docs/PACKAGES.md): package inventory.
- [docs/DOCKER.md](docs/DOCKER.md): runtime versus development container guidance.
- [docs/SIMULATION.md](docs/SIMULATION.md): simulation entrypoints and observed behavior.
- [docs/UPSTREAM_SYNC.md](docs/UPSTREAM_SYNC.md): fork maintenance guidance.
- [docs/CODE_REVIEW.md](docs/CODE_REVIEW.md): review checklist.
- [docs/DOCS_MAINTENANCE.md](docs/DOCS_MAINTENANCE.md): how to keep the documentation layer aligned.
