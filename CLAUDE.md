# Project memory

Purpose: concise startup guidance for Claude Code in this `open_mower_ros` fork.

Before editing, read `docs/PROJECT_STATE.md`, inspect the current branch/worktree
and dirty state, and follow the applicable active plan under
`docs/exec-plans/active/`. Substantial, multi-session, cross-package, or
safety-sensitive work follows `PLANS.md`. GitHub Issues are the authoritative
outstanding-work tracker; do not create new standalone TODO documents.

This repository is a ROS Noetic catkin workspace fork of `open_mower_ros`. The main orchestration lives in `src/open_mower`, first-party runtime packages live in `src/`, mixed external and submodule code lives in `src/lib/`, structured config lives in `config/`, runtime images live in `docker/`, development containers live in `devenv/` and `.devcontainer/`, shared xBot service definitions live in `services/`, and `web/` is observed built output.

## Critical commands

These commands are verified from repo files, not from execution in this environment.

```bash
rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
catkin_make
source devel/setup.bash
roslaunch open_mower open_mower.launch
```

Development container helpers from `devenv/README.md`:

```bash
./devenv/start_devenv.sh
./devenv/attach.sh
```

## Critical guardrails

- Verify from repo files before assuming commands, paths, or behavior.
- Consult a scoped rule in `.claude/rules/` before editing a matching path.
- Preserve ROS package boundaries and catkin conventions.
- Treat `mower_logic`, `mower_hardware`, `mower_comms_*`, launch/config wiring, hardware-specific params, and entrypoints as safety-sensitive.
- Treat `src/lib/ntrip_client`, `src/lib/xbot_driver_gps`, `src/lib/xbot_framework`, and `services` as external/submodule boundaries unless the task explicitly targets them.
- Treat `web/` as generated/build output by default.
- `config/mower_config.schema.json` is the authoritative structured config artifact.
- `config/mower_config.sh.example` is deprecated but must stay aligned while retained.
- `src/open_mower/config/mower_config.sh.example` is only a redirect stub.
- Do not silently smooth over repo drift. Document observed mismatches such as `ESC_TYPE` versus `OM_MOWER_ESC_TYPE` and the `Sabo` hardware preset.
- Preserve unrelated dirty work by using a separate issue, topic branch, and worktree. Routine commits, pushes, issues, and draft PRs are allowed for an authorized change; merge, force-push, deployment, and live mower/VESC actions remain human-gated.
- Proactively delegate bounded independent exploration, tests, and review while keeping one writer per file/workstream.

## Rules to consult

- `.claude/rules/ros-workspace.md`
- `.claude/rules/config-and-env.md`
- `.claude/rules/generated-and-external.md`
- `.claude/rules/docker-runtime.md`
- `.claude/rules/docs-style.md`

## Fast path by area

- `src/**/*`: read `.claude/rules/ros-workspace.md` and `src/AGENTS.md`.
- `config/**/*` or `*mower_config*`: read `.claude/rules/config-and-env.md` and `config/AGENTS.md`.
- `docker/**/*` or `.devcontainer/**/*`: read `.claude/rules/docker-runtime.md` and `docker/AGENTS.md`.
- `src/lib/**/*`, `services/**/*`, or `web/**/*`: read `.claude/rules/generated-and-external.md`.
- `**/*.md`: read `.claude/rules/docs-style.md`.

## Detailed references

- [AGENTS.md](AGENTS.md)
- [docs/PROJECT_STATE.md](docs/PROJECT_STATE.md)
- [PLANS.md](PLANS.md)
- [CONTRIBUTING.md](CONTRIBUTING.md)
- [docs/README.md](docs/README.md)
