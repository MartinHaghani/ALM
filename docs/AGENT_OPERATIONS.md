# Agent operations

Purpose: operate and maintain the repository's agent context, documentation, Git,
subagent, validation, and scheduled-audit system.

## What loads for a new agent

Codex automatically loads the root `AGENTS.md` and the closest nested `AGENTS.md`
files. The root guide routes the agent to three startup sources:

1. [PROJECT_STATE.md](PROJECT_STATE.md) for the compact current snapshot;
2. [PLANS.md](../PLANS.md) and the applicable active ExecPlan for substantial work;
3. the `project-operations` skill for the task lifecycle.

The skill uses progressive disclosure: Codex sees its name and description at
startup and loads its full workflow only when a repository change, review, handoff,
or tracking task matches. Stable architecture and operations docs are read only when
the task needs them.

## Sources of truth

| Information | Authority |
|---|---|
| Durable agent rules | root and nested `AGENTS.md` |
| Current cross-project snapshot | [PROJECT_STATE.md](PROJECT_STATE.md) |
| Complex in-flight implementation | `docs/exec-plans/active/` |
| Durable decision and rationale | `docs/decisions/` |
| Outstanding work | [GitHub Issues](https://github.com/MartinHaghani/open_mower_ros/issues) |
| Current system behavior | stable docs and implementation |
| Implementation/validation history | commits and pull requests |

Never turn `PROJECT_STATE` into a backlog or copy volatile issue status into stable
reference docs. Preserve decision rationale, alternatives, consequences, evidence,
and uncertainty; do not store raw chain-of-thought or uncurated transcripts.

## Local setup

Install and enable pre-commit in a development environment:

```bash
python3 -m pip install pre-commit
pre-commit install
pre-commit install --hook-type commit-msg
```

Project hooks under `.codex/hooks.json` are code and run only after the repository
is trusted. In Codex CLI, use `/hooks` to inspect the exact definitions and trust
their current hash. Review them again whenever `.codex/hooks.json` or its referenced
scripts change.

The hooks are intentionally bounded:

- `SessionStart` reports branch/status plus the current-state and plan routes.
- `SubagentStart` injects the evidence and one-writer handoff boundary.
- `Stop` runs deterministic hygiene checks and returns actionable warnings. It does
  not edit prose, stage, commit, push, merge, or delete anything.

CI is authoritative because local hooks can be skipped or untrusted.

## Run the policy checks

From the repository root, use the commands documented by the project-operations
skill:

```bash
python3 scripts/agent/check_project_hygiene.py --root . --scope all
python3 -m unittest discover -s scripts/agent/tests -p 'test_*.py'
python3 scripts/agent/evaluate_agent_context.py
python3 .github/scripts/test_validate_pr.py
pre-commit run
git diff --check
```

For a task branch, the validator infers its base from

1. explicit `--base`;
2. `PROJECT_HYGIENE_BASE`;
3. the uniquely matching active ExecPlan baseline;
4. the default branch.

Use an explicit base when inference is ambiguous:

```bash
python3 scripts/agent/check_project_hygiene.py \
  --root . \
  --scope changed \
  --base <task-baseline-commit>
```

The validator checks project-state and ExecPlan structure, ADR indexing, local
Markdown links, issue-linked TODO/FIXME markers, and required documentation impact
for sensitive path families. It validates structure and evidence routing; a human or
review agent still evaluates whether the semantics are true.

Pre-commit is a changed-file ratchet while [issue #21](https://github.com/MartinHaghani/open_mower_ros/issues/21)
owns inherited full-tree debt. Stage only the intended paths, then run
`pre-commit run`; CI compares the event base with `HEAD`. Do not use an all-files
run to autoformat unrelated baseline code. The JSON formatter deliberately skips
the generator/schema-owned config schema and package lock while `check-json` still
validates their syntax.

## Agent roles and delegation

Project roles live under `.codex/agents/`:

- `explorer`: read-only repository evidence gathering;
- `test_analyst`: focused test/build/log selection and interpretation;
- `correctness_reviewer`: read-only correctness and safety review;
- `docs_state_auditor`: read-only source-of-truth and handoff consistency review.

The parent agent keeps requirements, decisions, integration, and final reporting.
Use subagents for bounded independent exploration, testing, and review. Assign one
writer per file/workstream; concurrent writers require separate worktrees and a
single integrator. The checked-in Codex config caps concurrency at four threads and
nested delegation at one level.

When adding a role, give it one job, explicit write permissions, a required output
format, and a prohibition on recursive delegation. Do not create overlapping roles
to simulate a large team where one focused agent is enough.

## Issue and pull-request workflow

Issue forms require outcome, evidence, scope, safety, acceptance, dependencies, and
verification. Discovered work outside the active task becomes a linked issue rather
than hidden scope growth or an unowned TODO.

Pull requests require:

- a Conventional Commit-style title;
- linked issue and ExecPlan or a reason the plan is unnecessary;
- outcome and key decisions;
- documentation impact;
- safety and rollback;
- exact verification with observed results;
- known limitations and follow-up issues.

Routine topic-branch creation, commits, pushes, and draft PRs are automatic for an
authorized implementation task. Merge, direct default-branch push, force-push,
history rewriting, deployment, and live mower/VESC actions remain human-gated.

## Current GitHub governance

Verified and applied on 2026-07-15 to `MartinHaghani/open_mower_ros`:

- Issues enabled and the initial backlog plus legacy TODO ownership migrated to
  issues #1–#21;
- squash-only merges using PR title/body;
- automatic deletion of merged branches;
- `main` requires a PR, linear history, and resolved conversations;
- admin enforcement is enabled; force-push and branch deletion are disabled;
- zero required approvals because the repository currently has only one
  collaborator and GitHub does not permit approving one's own PR.

The checked-in policy workflow exposes the stable `Project policy / policy-gate`
check. Do not make it required until this workflow is present on the protected
branch and has passed representative PRs. Track that rollout in
[issue #10](https://github.com/MartinHaghani/open_mower_ros/issues/10).

Before requiring it, also establish an independent trust boundary for changes to
the workflow, validators, hooks, skills, and agent instructions. A PR-controlled
workflow can otherwise weaken its own checks while keeping the same status name.
Issue #10 owns the ruleset, trusted-workflow, or independent-review solution.

CODEOWNERS identifies safety and governance ownership, but mandatory CODEOWNER
approval must wait for an independent trusted reviewer; otherwise every PR becomes
unmergeable.

## Legacy TODO ratchet

[legacy-todos.json](legacy-todos.json) maps each pre-policy first-party marker to an
assigned issue by exact source text. It is a bounded migration register, not an
alternative backlog:

- new TODO/FIXME markers require an inline issue reference;
- changing or deleting a registered marker makes hygiene fail until its register
  entry is updated or removed;
- adding a register exception for new work is prohibited;
- when a source marker gains an inline issue link, remove its register entry.

Third-party and generated trees remain outside this ownership policy.

## GitHub Project rollout

The intended Projects v2 view uses:

- Status: Backlog, Ready, In progress, Review, Blocked, Done;
- Priority: P0, P1, P2, P3;
- Area: agent operations, coverage planner, localization, hardware, WebUI, and
  other repository domains as needed;
- Risk: normal, high, safety-critical;
- Owner.

Configure automatic addition of repository issues and PRs plus status movement for
opened/closed/merged items. Project creation requires the GitHub CLI token's separate
`project` scope; repository administration alone is insufficient. Refresh the scope
and complete issue #10 from an authenticated maintainer session:

```bash
gh auth refresh -s read:project -s project
```

Do not duplicate Project status in Markdown after rollout. `PROJECT_STATE` should
show only the few active workstreams and their authoritative links.

## Scheduled gardening

The Codex desktop automation `OpenMower project hygiene` runs weekly on Monday in an
isolated worktree. It audits documentation, project state, active plans, ADRs,
issues/PRs, worktrees, and deterministic policy checks.

`OpenMower agent context regression` runs every four weeks on Wednesday in an
isolated worktree. It samples at least three risk-balanced evaluation cases with
three trials per case when the environment supports them, records raw measures, and
updates issue #11. This periodic sample does not replace the full candidate cohort
required before merging material context-policy changes.

It may open issues or a draft PR for bounded low-risk repairs. It must not merge,
force-push, delete unique worktrees, change mower runtime code, deploy, alter live
VESC settings, or perform physical tests. Review the first several runs and refine
the prompt if it produces false positives or broad changes. Archive old scheduled
runs so their worktrees do not accumulate.

## Evaluate performance

Use [agent-evals/README.md](agent-evals/README.md) and its ten representative cases
to compare fresh-agent orientation, restart success, unsupported claims, human
corrections, documentation drift, end-state correctness, tool use, and tokens.
Deterministic structural checks run in CI; stochastic model trials run manually or
on a schedule in disposable worktrees. Never treat a single model sample as a merge
gate.

## Change the operating system

Update `AGENTS.md` only for durable rules or repeated failure patterns. Put a
repeatable workflow in the skill, mechanical enforcement in validators/hooks/CI,
volatile status in `PROJECT_STATE` or an active plan, durable choices in an ADR, and
outstanding work in an issue.

Material changes to instructions, plans, hooks, roles, or hygiene checks require:

1. an issue and, when complex, an active ExecPlan;
2. documentation and backward-compatibility review;
3. deterministic policy tests;
4. a documentation-state audit and correctness review;
5. representative cases from the agent evaluation suite;
6. a draft PR with rollback instructions.
