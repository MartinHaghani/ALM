# Documentation index

Purpose: index of the documentation layer for agents and human contributors.

## Start here

For a fast orientation pass, read in this order:

1. [../AGENTS.md](../AGENTS.md) if you are an agent or want the shortest operational summary.
2. [PROJECT_STATE.md](PROJECT_STATE.md) for the current baseline, active workstreams, and authoritative handoffs.
3. [../CLAUDE.md](../CLAUDE.md) if you are using Claude Code.
4. [../CONTRIBUTING.md](../CONTRIBUTING.md) for contributor workflow and safety expectations.
5. [BUILD_AND_RUN.md](BUILD_AND_RUN.md) for setup, build, launch, and container usage.
6. [RASPBERRY_PI.md](RASPBERRY_PI.md) if you want a plain Raspberry Pi bring-up and fast local-repo workflow.
7. [MOWRATOR_BENCH_BRINGUP.md](MOWRATOR_BENCH_BRINGUP.md) before touching the current custom mower bench hardware.
8. [CONFIGURATION.md](CONFIGURATION.md) before touching config, params, or environment handling.
9. [PACKAGES.md](PACKAGES.md) and [ARCHITECTURE.md](ARCHITECTURE.md) for codebase structure.

## Agent-facing entrypoints

- [../AGENTS.md](../AGENTS.md): Codex-first repo operating guide.
- [PROJECT_STATE.md](PROJECT_STATE.md): concise current-state router; not the backlog.
- [../PLANS.md](../PLANS.md): policy for self-contained, living execution plans.
- [exec-plans/](exec-plans/): active implementation handoffs and completed plan archive.
- [decisions/](decisions/): durable architecture and operating decisions.
- [AGENT_OPERATIONS.md](AGENT_OPERATIONS.md): maintainer runbook for skills, hooks, subagents, validation, GitHub governance, scheduling, and evaluations.
- [agent-evals/README.md](agent-evals/README.md): fresh-agent orientation and handoff evaluation suite.
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

## Documentation types

The repository is migrating incrementally toward the Diátaxis distinction without
moving established paths in one disruptive change:

- **How-to and runbooks:** [BUILD_AND_RUN.md](BUILD_AND_RUN.md),
  [RASPBERRY_PI.md](RASPBERRY_PI.md),
  [MOWRATOR_BENCH_BRINGUP.md](MOWRATOR_BENCH_BRINGUP.md),
  [VESC_MAINTENANCE.md](VESC_MAINTENANCE.md), and [DOCKER.md](DOCKER.md).
- **Reference:** [REPO_MAP.md](REPO_MAP.md), [PACKAGES.md](PACKAGES.md),
  [CONFIGURATION.md](CONFIGURATION.md), [SIMULATION.md](SIMULATION.md), and
  [FTC_EXECUTOR_LIMITATIONS.md](FTC_EXECUTOR_LIMITATIONS.md).
- **Explanation and design:** [ARCHITECTURE.md](ARCHITECTURE.md), hardware and
  recording architecture documents, coverage-planner research/design documents,
  and [UPSTREAM_SYNC.md](UPSTREAM_SYNC.md).
- **Learning tutorials:** add these only when a newcomer needs a guided learning
  experience distinct from an operational runbook. Do not relabel safety procedures
  as tutorials.
- **Plans and governance:** `PROJECT_STATE`, ExecPlans, roadmaps, ADRs, and review
  policy are operational project records and remain separate from those four
  reader-documentation categories.

Classify new durable documentation before creating it. Improve existing documents
in place, preserve links, and move a file only when the benefit exceeds the link and
history disruption.

## Documentation catalogue

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
- [COVERAGE_PLANNER_RESEARCH.md](COVERAGE_PLANNER_RESEARCH.md): durable source ledger for V2 coverage-planner research.
- [COVERAGE_PLANNER_V2_DESIGN.md](COVERAGE_PLANNER_V2_DESIGN.md): fresh mower-first coverage planner architecture, path contract, and runtime redesign direction.
- [COVERAGE_PLANNER_V2_ALGORITHM_PLAN.md](COVERAGE_PLANNER_V2_ALGORITHM_PLAN.md): comprehensive algorithm plan for task extraction, candidate generation, route optimization, and verification.
- [COVERAGE_PLANNER_V2_PROTOTYPE_PLAN.md](COVERAGE_PLANNER_V2_PROTOTYPE_PLAN.md): implementation plan for the first V2 map-conditioning and macro-zone classification prototype.
- [COVERAGE_PLANNER_ROADMAP.md](COVERAGE_PLANNER_ROADMAP.md): prioritized coverage planner work list (P0–P9), per-priority status, and edge case catalogue.
- [PATH_RECORDING_ARCHITECTURE.md](PATH_RECORDING_ARCHITECTURE.md): manual-mowing path recording architecture for GPS, LIDAR, and fused teach-path/debug artifacts.
- [FTC_EXECUTOR_LIMITATIONS.md](FTC_EXECUTOR_LIMITATIONS.md): current `PlanPath`/FTC executor limitations that teach-path recording must preserve rather than hide.
- [UPSTREAM_SYNC.md](UPSTREAM_SYNC.md): fork maintenance guidance.
- [CODE_REVIEW.md](CODE_REVIEW.md): reusable review checklist.
- [TODO.md](TODO.md): legacy pointer to the authoritative GitHub Issues backlog; do not add tasks there.
- [legacy-todos.json](legacy-todos.json): machine-validated issue ownership for
  pre-policy first-party TODO/FIXME markers; not a backlog for new work.
- [DOCS_MAINTENANCE.md](DOCS_MAINTENANCE.md): how to keep this doc layer aligned.

## Project governance

- [PROJECT_STATE.md](PROJECT_STATE.md): short current snapshot and active-work router.
- [../PLANS.md](../PLANS.md): when and how to maintain an ExecPlan.
- [exec-plans/](exec-plans/): living implementation records and their archive.
- [decisions/](decisions/): ADR index and accepted decisions.
- [templates/exec-plan.md](templates/exec-plan.md) and [templates/adr.md](templates/adr.md): canonical templates.
- [GitHub Issues](https://github.com/MartinHaghani/open_mower_ros/issues): authoritative dynamic backlog.
- GitHub Project: target status/priority/area/risk view; rollout is tracked in [issue #10](https://github.com/MartinHaghani/open_mower_ros/issues/10).
