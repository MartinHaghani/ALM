# Project state

Purpose: give a newly started agent a compact, evidence-based snapshot and route it
to the active source for each workstream.

This file is **not the backlog**, an architecture reference, or a history log.
Outstanding work belongs in GitHub Issues; Project views may organize those issues;
implementation detail belongs in an active ExecPlan; stable behavior belongs in
the relevant reference documentation. Keep this page short enough to read at every
task start.

## Current Baseline

- Last verified: 2026-07-15.
- Canonical GitHub repository: [MartinHaghani/ALM](https://github.com/MartinHaghani/ALM), a public standalone repository.
- Branch used for this migration: `codex/agent-operating-system`.
- Inspected baseline commit: `329726f` (`webui: compact map selector controls`).
- Runtime baseline: the launch-composed ROS Noetic workspace described in
  [ARCHITECTURE.md](ARCHITECTURE.md); the laptop coverage lab does not replace the
  runtime `slic3r_coverage_planner`.
- Safety boundary: mower logic, low-level communications, hardware configuration,
  launch wiring, live VESC settings, and container entrypoints require
  risk-proportional validation and explicit review.

Always verify `git status`, `git branch --show-current`, `git worktree list`, and
`git rev-parse --short HEAD` before relying on this snapshot. A checked-in state
page cannot see uncommitted changes in another worktree.

## Active Workstreams

| Workstream | Status | Snapshot | Authoritative next-step source |
|---|---|---|---|
| Standalone ALM repository migration | active | Public standalone repo, curated refs, issues #1–#23, draft PR #22, governance, preservation, GPS publication, recursive-clone acceptance, and scheduled-automation migration are complete; canonical-doc CI and confidence-period closeout remain | [issue #23](https://github.com/MartinHaghani/ALM/issues/23), [ADR 0003](decisions/0003-standalone-alm-project-identity.md), and the [active migration ExecPlan](exec-plans/active/alm-standalone-repository-migration.md) |
| Agent documentation, context, and Git operating system | active | Draft PR #22 passed policy gates in the former repository at `474a8a1`; the ALM rerun, full candidate cohort, and external rollout remain | [PR #22](https://github.com/MartinHaghani/ALM/pull/22), [issue #1](https://github.com/MartinHaghani/ALM/issues/1), and the [active migration ExecPlan](exec-plans/active/agent-operating-system-migration.md) |
| Existing coverage planner lab | planned | P0, P1, P3, P10, P11, P12, and P13 are recorded as landed; P5 is the next sequence item | [roadmap](COVERAGE_PLANNER_ROADMAP.md) and [issue #3](https://github.com/MartinHaghani/ALM/issues/3) |
| Coverage Planner V2 exploration | planned | M1 evidence and substantial M2.x local prototypes exist; M2 acceptance reconciliation and M3 candidate routing are outstanding | [algorithm status](COVERAGE_PLANNER_V2_ALGORITHM_PLAN.md#current-implementation-status) and [issue #8](https://github.com/MartinHaghani/ALM/issues/8) |
| Passive SLAM confidence weighting | planned | No active implementation plan | [issue #2](https://github.com/MartinHaghani/ALM/issues/2) |
| Parallel Mowrator slope branches | planned | Audit required before integration or worktree cleanup | [issue #12](https://github.com/MartinHaghani/ALM/issues/12) |

## Blockers and Risks

The original `/Users/martinhaghani/Code/open_mower_ros` worktree had 23 modified or
untracked paths across several unrelated workstreams at the 2026-07-15 snapshot.
This migration uses a separate clean worktree and must not stage, reset, or rewrite
that state.

The inspection also found additional unmerged worktrees for Mowrator slope
reliability at commits `f56dd9a` and `bb84e454`, plus three Claude worktrees. Branch
names and commits prove that parallel state exists; they do not prove completion or
integration. Inspect `git worktree list --porcelain`, each worktree's status, and its
branch-local handoff before using or deleting any of them. Do not treat results on
those branches as part of the baseline until they are reviewed and merged.

The public standalone ALM repository now exists and the curated migration refs are
recoverable from the independently verified preservation bundle at
`/Users/martinhaghani/Code/ALM_migration_backup_20260715T210630-0400`. Commit
`bd05076` in `src/lib/xbot_driver_gps` is published on
`codex/pi-rover-fixes`; a fresh recursive ALM clone retrieved it and every nested
submodule successfully. The former fork and all active worktrees remain intact for
the confidence period.

The current tree retains every path present at the historical fork point, including
293 byte-identical files. ALM branding is accepted, but inherited and third-party
license notices must remain until documented rights or clean-room provenance prove
otherwise. Package-license normalization and machine-readable provenance remain
tracked migration work.

Tracked rollout gaps are:

- GitHub Issues are authoritative and issues #1–#23 plus draft PR #22 retain the
  migrated numbering and provenance.
  Every legacy first-party source TODO/FIXME marker has exact-content ownership in
  the machine-validated [legacy register](legacy-todos.json); new markers require
  inline issue references. The Projects v2 board still requires separate OAuth
  `project` scope and is tracked
  by [issue #10](https://github.com/MartinHaghani/ALM/issues/10).
- ALM `main` protection is active without required CI checks; add the stable policy
  gate only after the workflow lands on `main` and passes, also under issue #10.
- The manual fresh-agent candidate cohort required before merge is outstanding in
  [issue #11](https://github.com/MartinHaghani/ALM/issues/11); stochastic
  trials are evidence, not a required per-PR CI check.
- Full-tree pre-commit debt is ratcheted to added/modified files and tracked by
  [issue #21](https://github.com/MartinHaghani/ALM/issues/21).
- The React `/next/` source identifies ALM; the generated legacy Flutter root UI
  still requires a source-owned rebuild under
  [issue #25](https://github.com/MartinHaghani/ALM/issues/25).
- Parallel slope branches require the branch-by-branch audit in issue #12.

These are migration gaps, not a replacement backlog. Remove a bullet when its
linked issue is completed; do not add implementation checklists here.

## Next Actions

1. Push this canonical-documentation checkpoint to
   [draft PR #22](https://github.com/MartinHaghani/ALM/pull/22), wait for the ALM
   policy checks, and record the result in
   [issue #23](https://github.com/MartinHaghani/ALM/issues/23).
2. Complete the remaining license/provenance and confidence-period acceptance items
   in issue #23; do not
   archive the former fork without explicit maintainer approval.
3. Run and record the full pre-merge candidate cohort under
   [issue #11](https://github.com/MartinHaghani/ALM/issues/11) at an
   immutable reviewed head of [draft PR #22](https://github.com/MartinHaghani/ALM/pull/22).
4. Review draft PR #22; preserve its isolated worktree until the PR is merged or
   explicitly abandoned.
5. Complete the external Project/required-check rollout in issue #10 after the
   workflow lands.
6. Resume planner and slope work only through their linked issues and active plans;
   preserve every unmerged worktree until issue #12 proves it is safe to clean.

## Routing

- Repository layout and ownership: [REPO_MAP.md](REPO_MAP.md) and
  [PACKAGES.md](PACKAGES.md).
- Runtime composition: [ARCHITECTURE.md](ARCHITECTURE.md).
- Build and launch commands: [BUILD_AND_RUN.md](BUILD_AND_RUN.md).
- Mowrator bench and live-hardware safety:
  [MOWRATOR_BENCH_BRINGUP.md](MOWRATOR_BENCH_BRINGUP.md) and
  [VESC_MAINTENANCE.md](VESC_MAINTENANCE.md).
- Agent planning policy: [../PLANS.md](../PLANS.md).
- Durable decisions: [decisions/README.md](decisions/README.md).
- ALM repository migration: [active ALM ExecPlan](exec-plans/active/alm-standalone-repository-migration.md).
- Documentation update rules: [DOCS_MAINTENANCE.md](DOCS_MAINTENANCE.md).

## Refresh Contract

Update this file in the same change whenever an active workstream starts, finishes,
becomes blocked, changes its authoritative plan, or establishes a new baseline that
would change a fresh agent's first action. Each row must link to evidence. Remove
finished work instead of turning this page into a changelog.
