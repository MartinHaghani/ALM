# ADR 0002: Git autonomy and safety boundary

- Status: Accepted
- Date: 2026-07-15
- Decision owners: repository maintainers
- Related plan: [Agent operating-system migration](../exec-plans/active/agent-operating-system-migration.md)

## Context

The maintainer wants agents to manage routine documentation, tracking, and Git work
without repeated prompts. The repository also controls a physical mower and includes
safety-critical logic, hardware configuration, live VESC settings, and runtime
entrypoints. Unrestricted automation would reduce coordination cost but could merge
unreviewed behavior, destroy recoverable history, or change live hardware state.

Multiple worktrees and unrelated uncommitted changes are normal in the current
repository. Safe automation therefore needs explicit ownership and isolation rules,
not merely permission to run Git commands.

## Decision

Agents may perform the following routine actions without asking for a new prompt
when they are necessary to complete an already authorized repository task:

- inspect repository, branch, remote, issue, and worktree state;
- create or resume a topic branch and isolated worktree;
- stage only the files owned by the current workstream;
- create narrow, logical commits with meaningful descriptions and validation;
- push topic branches without force;
- create and update issues, Project items, and draft pull requests;
- update documentation, ExecPlans, ADRs, and tracking state in the same change;
- remove a clean task worktree after its work is merged and recoverability is
  confirmed.

The following actions require explicit human approval at the point of action:

- merging a pull request or pushing directly to the default branch;
- force-pushing, rebasing shared published history, deleting a branch containing
  unmerged work, or otherwise reducing recoverability;
- bypassing required checks, reviews, rulesets, or code-owner approval;
- deploying to a mower or production environment;
- writing live VESC settings or changing live hardware configuration;
- destructive filesystem, map-data, or device operations outside the task's clear
  scope.

Agents must use one task/issue per topic branch and worktree when parallel edits or
unrelated dirty state exist. Only one agent owns writes to a shared file or
workstream at a time. Read-heavy exploration, testing, and review may run in
parallel; their findings return to the owning agent for integration.

The default branch must be protected by required CI checks. Safety-sensitive paths
must have CODEOWNERS or equivalent required review. A push or API failure is recorded
in the active plan and final report; it is not silently treated as completion.

## Alternatives considered

### Ask before every Git or tracking action

Rejected because it shifts routine coordination back to the maintainer and makes
documentation and issue state lag behind implementation.

### Permit agents to merge when checks pass

Rejected for this repository because passing automated checks does not establish
physical mower safety, operational readiness, or acceptance of user-visible tradeoffs.

### Never allow agents to push or create issues

Rejected because local-only work is difficult to recover and invisible to project
automation, while untracked discoveries are easily lost.

## Consequences

- Routine work can reach a reviewable draft PR without repeated Git instructions.
- Branch protection, CI, issue templates, and CODEOWNERS become required parts of
  the operating system rather than optional conventions.
- Agents must inspect dirty state and ownership before editing or staging files.
- The maintainer retains final control over merging, destructive history changes,
  deployment, and live hardware effects.
- Offline or unavailable GitHub operations may leave a locally complete task in a
  pending state; the plan and final report must identify the exact missing action.

## Acceptance evidence

This decision is implemented when repository instructions encode the boundary,
templates and checks require issue/plan/validation linkage, topic-branch automation
can create a draft PR, and protected actions cannot occur without human approval.
