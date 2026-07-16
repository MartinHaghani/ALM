# ExecPlan: Establish the repository agent operating system

- Status: Active
- Owner: coordinating Codex agent
- Created: 2026-07-15
- Last updated: 2026-07-15
- Issue: [#1](https://github.com/MartinHaghani/open_mower_ros/issues/1)
- Branch/worktree: `codex/agent-operating-system` at `/Users/martinhaghani/Code/open_mower_ros_agent_ops`
- Baseline commit: `329726f`
- Related ADRs: [ADR 0001](../../decisions/0001-agent-documentation-and-context-model.md), [ADR 0002](../../decisions/0002-git-autonomy-and-safety-boundary.md)

This plan follows [PLANS.md](../../../PLANS.md). It is the self-contained handoff for
implementing the documentation, context, tracking, subagent, and Git automation
requested by the maintainer.

## Purpose and Intended Outcome

A newly started agent should orient itself, select or create the correct workstream,
use subagents appropriately, preserve decisions and evidence, update outstanding
work, perform safe routine Git operations, and leave a reviewable handoff without
the maintainer repeating workflow prompts.

The outcome is a repository-enforced workflow, not merely more prose. Checked-in
instructions and skills define agent behavior; state, plans, ADRs, and issues have
separate sources of truth; deterministic local and CI checks detect omissions; Git
rules preserve human control over merge, destructive history, deployment, and live
hardware operations.

## Progress

- [x] (2026-07-15) Audited existing documentation, Git history, dirty state, and
  parallel worktrees; identified strong domain docs but no universal current-state,
  decision, or handoff layer.
- [x] (2026-07-15) Created isolated branch `codex/agent-operating-system` and
  worktree from baseline `329726f` so unrelated source-worktree changes remain
  untouched.
- [x] (2026-07-15) Added the documentation foundation: planning policy, project
  state router, ADR index and initial decisions, templates, and plan lifecycle.
- [x] (2026-07-15) Added concise repository operating instructions and a project operations skill
  that automatically performs startup, planning, documentation, subagent, tracking,
  Git, and closeout duties.
- [x] (2026-07-15) Added bounded custom subagent roles and project hooks for context injection and
  deterministic stop-time hygiene checks.
- [x] (2026-07-15) Completed validators and their self-tests, then executed the
  changed-file pre-commit/CI path for links, state, ExecPlans, ADRs, documentation
  impact, and commit/PR policy. Full-tree debt is tracked in issue #21.
- [x] (2026-07-15) Added issue/PR templates, CODEOWNERS, pinned workflow actions,
  least-privilege workflow permissions, and policy CI without weakening the Docker
  build or Release Drafter.
- [x] (2026-07-15) Enabled Issues, created labels and issues #1–#12, configured
  squash-only merging and branch cleanup, and protected `main` with PR-only linear
  history, resolved conversations, and force-push/deletion blocks.
- [x] (2026-07-15) Added active isolated Codex automations for weekly project
  hygiene and four-week rotating fresh-agent regression sampling.
- [ ] Create the Projects v2 board after the GitHub credential receives `project`
  scope; add the policy gate as required only after it lands and passes.
- [x] (2026-07-15) Validated deterministic fresh-agent orientation, hooks, skill
  structure, configuration parsing, documentation-only/code/safety documentation
  impact, PR policy, and changed-file pre-commit behavior in the isolated clean
  worktree.
- [x] (2026-07-15) Reconciled migration artifacts and assigned every legacy
  first-party source TODO/FIXME marker through an exact-content validated register;
  issues #13–#20 capture the newly discovered work.
- [ ] Run and record the required manual fresh-agent candidate cohort under issue
  #11 before merge.
- [ ] Prepare and review the draft PR, complete the Projects v2/required-check
  rollout in issue #10, then move this plan to `completed/` after merge.

Exact next action: publish the reviewed migration branch as a draft PR linked to
issue #1.

## Surprises & Discoveries

- The repository already contains detailed architecture, hardware, planner, and
  maintenance documentation. The primary gap is state ownership and enforcement,
  not lack of domain prose.
- The source worktree contains unrelated uncommitted changes spanning multiple
  workstreams. An isolated worktree is required to avoid accidental staging or
  rewriting of user work.
- Several local worktrees and unmerged slope branches exist. A branch name is not
  completion evidence; every agent must inspect worktree state and branch-local
  handoffs before integration or cleanup.
- Coverage Planner V2 status is internally inconsistent: the newer prototype plan
  points beyond classification, while the older algorithm plan still calls M1 the
  immediate next step. The migration must track reconciliation rather than silently
  selecting one claim.
- Recent commits often have concise subjects without explanatory bodies, reducing
  the usefulness of Git history as a handoff ledger. Commit policy must preserve
  problem, decision, validation, risk, and follow-up context for substantive work.
- The fork had Issues disabled and no branch protection. Both are now enabled; the
  sole-collaborator setup makes a mandatory independent approval impossible until a
  second trusted reviewer is added.
- GitHub Projects v2 uses a separate OAuth scope not present on the current `gh`
  token. Repository administration access does not imply Projects access.
- The repository contained 15 legacy source TODO/FIXME sites without ownership.
  Eight focused issues (#13–#20) now own them; the navigation-completion marker was
  linked to the already related issue #6.
- The former pre-commit exclusion regex effectively skipped the repository. A true
  all-files run exposed inherited formatter and executable-bit debt and attempted
  broad unrelated rewrites. Those side effects were removed, CI now ratchets
  added/modified files, and issue #21 owns deliberate full-tree cleanup.

## Decision Log

- 2026-07-15 — Separate instructions, current state, plans, decisions, stable docs,
  backlog, and Git history into purpose-specific artifacts. This reduces stale
  duplication and startup context. See ADR 0001.
- 2026-07-15 — Make GitHub Issues/Projects the target authoritative backlog; retain
  existing roadmaps as design and sequencing references during deliberate migration.
- 2026-07-15 — Permit routine branch, worktree, commit, push, issue, Project, and
  draft-PR actions while reserving merge, history rewriting, deployment, and live
  hardware changes for explicit human approval. See ADR 0002.
- 2026-07-15 — Use deterministic hooks and CI to validate structure and required
  evidence. Automation may flag missing semantic documentation but must not invent
  decisions or rewrite prose automatically.
- 2026-07-15 — Parallelize bounded read-heavy exploration, testing, and review;
  assign one write owner per file/workstream and use isolated worktrees for
  concurrent implementation.

## Outcomes & Retrospective

The local operating system is complete and internally consistent in this worktree:
layered documentation, the behavioral skill, bounded custom agents, hooks,
validators, policy CI, GitHub templates, evaluation cases, and Git conventions all
have deterministic coverage. The migration also exposed and assigned every legacy
first-party source TODO instead of silently carrying it forward.

Verification passed for the agent-policy and PR-policy suites, seven fresh-agent
assertions, full and strict changed-scope hygiene, action workflow parsing,
JSON/TOML parsing, hook output parsing, changed-file pre-commit, and whitespace.
No ROS build or live hardware validation was required because runtime behavior and
first-party source are unchanged; legacy markers are mapped through the validated
register rather than comment-only source edits.

The remaining rollout is deliberately external: Projects v2 needs one-time OAuth
scope, and the policy gate cannot safely become a required check until its workflow
has landed and passed. Issue #10 owns both steps. The manual candidate cohort in
issue #11 is also required before merge, while full-tree formatting cleanup remains
separate in issue #21. This plan remains active through PR review and merge, then
moves to `completed/`.

## Context and Orientation

This repository is a ROS Noetic catkin workspace. Root `AGENTS.md` and nested agent
guides define repository boundaries; [docs/README.md](../../README.md) indexes stable
documentation. Safety-sensitive areas include `src/mower_logic`,
`src/mower_comms_v1`, `src/mower_comms_v2`, hardware-specific parameters, launch
wiring, container entrypoints, and live VESC settings.

The operating-system implementation belongs on `codex/agent-operating-system` in
the isolated worktree named above. Do not stage or clean files in the source
worktree. Before editing, inspect `git status --short --branch`, all worktrees, the
active plan, and file ownership assigned to parallel agents.

The documentation sources of truth are defined by ADR 0001. The autonomy boundary
is defined by ADR 0002. Existing build workflows under `.github/workflows/` must be
preserved; new policy checks should call the same deterministic validation entrypoint
as local hooks and pre-commit.

## Plan of Work

First, establish the documentation foundation and routing contract without moving
existing domain docs. Then encode the lifecycle in concise agent instructions and a
focused repository skill so it triggers automatically for implementation work.

Next, add custom subagents for exploration, testing, review, and documentation
audit. Add project hooks only for context routing and deterministic hygiene checks;
the owning agent remains responsible for semantic decisions and edits.

Add one reusable validator that checks internal links, required ExecPlan and ADR
structure, project-state freshness fields, documentation impact declarations, and
prohibited tracking patterns. Run it from pre-commit and a required GitHub Actions
workflow. Add issue and PR templates, CODEOWNERS for safety-sensitive paths, and
document the repository settings that cannot be enforced by checked-in files alone.

Finally, test onboarding and closeout behavior in a clean checkout and across three
representative scenarios. Reconcile evidence and outstanding items, then complete
and archive this plan while preparing a draft PR for human review.

## Concrete Steps

Work only in the isolated worktree:

```bash
cd /Users/martinhaghani/Code/open_mower_ros_agent_ops
git status --short --branch
git worktree list --porcelain
```

Inspect documentation and policy references before modifying them:

```bash
sed -n '1,240p' AGENTS.md
sed -n '1,240p' docs/PROJECT_STATE.md
sed -n '1,260p' PLANS.md
```

After implementation, run the repository-provided policy validator and existing
relevant checks:

```bash
python3 scripts/agent/check_project_hygiene.py --root . --scope all
python3 -m unittest discover -s scripts/agent/tests -p 'test_*.py'
python3 scripts/agent/evaluate_agent_context.py
python3 .github/scripts/test_validate_pr.py
# After staging only the intended paths:
pre-commit run
git diff --check
git status --short
```

Expected success is a zero exit status, no broken repository-local links, no missing
required sections, and a diff containing only the intended operating-system files.

## Validation and Acceptance

The migration is accepted only when all of the following are demonstrated:

- A fresh agent can identify the repository baseline, active workstream, exact next
  action, safety boundary, and validation commands from checked-in sources without
  access to the originating conversation.
- Implementation tasks automatically invoke the project workflow, while simple
  questions do not create unnecessary plans, branches, or issues.
- Independent read-heavy tasks are delegated; concurrent agents cannot silently
  write the same workstream.
- A discovered out-of-scope item is created or queued as a structured issue and is
  included in the final outstanding-work report.
- A substantive change missing its required plan, documentation-impact record,
  validation evidence, or PR linkage fails locally and in CI with an actionable
  message.
- Topic-branch commit, push, and draft-PR preparation can run without repeated user
  prompts, while merge, force-push, direct default-branch push, deployment, and live
  hardware operations remain human-gated.
- Existing repository build workflows still parse and the new checks run on pull
  requests. Documentation links and templates validate on a clean checkout.
- `git diff --check` and all added automated tests pass. Any environment-dependent
  check that cannot run is identified with exact reason and follow-up issue.

## Idempotence and Recovery

All new files are additive or narrow policy updates, so the migration can be rerun
and reviewed without touching source work. Validators must be read-only and
idempotent. Hooks may stop a task with an actionable diagnostic but may not modify
prose, stage files, commit, push, merge, or delete worktrees.

If interrupted, start from this plan's `Progress` and verify Git/worktree state
before editing. Preserve partial commits on the topic branch. If parallel edits
overlap, stop the losing writer, inspect both diffs, and integrate deliberately
instead of resetting user work. Do not remove the isolated worktree or branch until
the draft PR is merged or the maintainer explicitly abandons it after confirming no
unique commits remain.

External GitHub configuration should be applied only after the checked-in workflow
is reviewable. If API access or permissions are unavailable, record the exact
settings and pending action in this plan and an issue rather than treating the
migration as complete.

## Artifacts and Interfaces

- Planning policy: `PLANS.md`.
- Current-state router: `docs/PROJECT_STATE.md`.
- Active/completed plan lifecycle: `docs/exec-plans/`.
- Templates: `docs/templates/exec-plan.md` and `docs/templates/adr.md`.
- Durable decisions: `docs/decisions/`.
- Agent behavior: root/nested `AGENTS.md` and the repository operations skill added
  by this migration.
- Agent roles and hooks: `.codex/agents/` and project hook configuration added by
  this migration.
- Deterministic checks: repository validation scripts, pre-commit, and GitHub Actions
  added by this migration.
- Tracking/review: GitHub Issues, Project items, pull-request template, CODEOWNERS,
  draft PR, and protected-branch settings.

## Plan Change Log

- 2026-07-15 — Created the self-contained migration plan and recorded the initial
  documentation foundation, decisions, evidence, and remaining milestones.
- 2026-07-15 — Completed the local automation and governance layers, applied
  repository settings, migrated backlog/TODO ownership to issues #1–#20, recorded
  the Projects OAuth boundary in issue #10, and captured verification evidence.
- 2026-07-15 — Replaced attempted full-tree formatting with a changed-file ratchet,
  tracked inherited debt in issue #21, added the exact legacy-marker register, and
  incorporated independent correctness and documentation review findings.
