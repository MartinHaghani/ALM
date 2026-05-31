# Documentation index

Purpose: index of the documentation layer for agents and human contributors.

## Start here

For a fast orientation pass, read in this order:

1. [../AGENTS.md](../AGENTS.md) if you are an agent or want the shortest operational summary.
2. [../CLAUDE.md](../CLAUDE.md) if you are using Claude Code.
3. [../CONTRIBUTING.md](../CONTRIBUTING.md) for contributor workflow and safety expectations.
4. [BUILD_AND_RUN.md](BUILD_AND_RUN.md) for setup, build, launch, and container usage.
5. [RASPBERRY_PI.md](RASPBERRY_PI.md) if you want a plain Raspberry Pi bring-up and fast local-repo workflow.
6. [MOWRATOR_BENCH_BRINGUP.md](MOWRATOR_BENCH_BRINGUP.md) before touching the current custom mower bench hardware.
7. [CONFIGURATION.md](CONFIGURATION.md) before touching config, params, or environment handling.
8. [PACKAGES.md](PACKAGES.md) and [ARCHITECTURE.md](ARCHITECTURE.md) for codebase structure.

## Agent-facing entrypoints

- [../AGENTS.md](../AGENTS.md): Codex-first repo operating guide.
- [../CLAUDE.md](../CLAUDE.md): concise Claude Code startup memory.
- [../src/AGENTS.md](../src/AGENTS.md): package-boundary guidance for `src/`.
- [../config/AGENTS.md](../config/AGENTS.md): config source-of-truth and sync rules.
- [../docker/AGENTS.md](../docker/AGENTS.md): runtime image and entrypoint guardrails.
- [../web/AGENTS.md](../web/AGENTS.md): generated-web guidance.
- [../webui/AGENTS.md](../webui/AGENTS.md): React `/next/` WebUI guidance.
- [../.claude/rules/ros-workspace.md](../.claude/rules/ros-workspace.md): scoped ROS workspace rule.
- [../.claude/rules/config-and-env.md](../.claude/rules/config-and-env.md): scoped config rule.
- [../.claude/rules/generated-and-external.md](../.claude/rules/generated-and-external.md): external/generated rule.
- [../.claude/rules/docker-runtime.md](../.claude/rules/docker-runtime.md): Docker/runtime rule.
- [../.claude/rules/docs-style.md](../.claude/rules/docs-style.md): Markdown style rule.

## Human-facing reference docs

- [REPO_MAP.md](REPO_MAP.md): top-level tree and ownership boundaries.
- [ARCHITECTURE.md](ARCHITECTURE.md): launch-composed runtime and package roles.
- [BUILD_AND_RUN.md](BUILD_AND_RUN.md): local build, launch, development containers, and runtime images.
- [RASPBERRY_PI.md](RASPBERRY_PI.md): plain Raspberry Pi OS bring-up and manual update loop.
- [MOWRATOR_MIGRATION.md](MOWRATOR_MIGRATION.md): staged Flipsky-based hardware migration for the custom `Mowrator` profile.
- [MOWRATOR_BENCH_BRINGUP.md](MOWRATOR_BENCH_BRINGUP.md): current one-by-one bench checks for the RUTX11, LSM6DSO, F9P, and Slamtec C1 hardware.
- [AREA_RECORDING_SWEEP.md](AREA_RECORDING_SWEEP.md): swept-footprint mower map recording behavior and tuning notes.
- [VESC_MAINTENANCE.md](VESC_MAINTENANCE.md): headless VESC Tool install and UART config workflow on the Pi.
- [vesc_configs/README.md](vesc_configs/README.md): repo-tracked live VESC XML snapshots for the current mower.
- [CONFIGURATION.md](CONFIGURATION.md): config schema, deprecated shell example, YAML and env loading.
- [PACKAGES.md](PACKAGES.md): package inventory for `src/` and important `src/lib/` packages.
- [DOCKER.md](DOCKER.md): runtime versus development container behavior.
- [SIMULATION.md](SIMULATION.md): simulation entrypoints and observed runtime shape.
- [COVERAGE_PLANNER_LAB.md](COVERAGE_PLANNER_LAB.md): laptop-only Fields2Cover route-planning evaluation setup.
- [COVERAGE_PLANNER_ROADMAP.md](COVERAGE_PLANNER_ROADMAP.md): prioritized coverage planner work list (P0–P9), per-priority status, and edge case catalogue.
- [UPSTREAM_SYNC.md](UPSTREAM_SYNC.md): fork maintenance guidance.
- [CODE_REVIEW.md](CODE_REVIEW.md): reusable review checklist.
- [TODO.md](TODO.md): deferred fork-specific implementation ideas.
- [DOCS_MAINTENANCE.md](DOCS_MAINTENANCE.md): how to keep this doc layer aligned.
