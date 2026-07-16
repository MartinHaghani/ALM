# Execution plans

Purpose: define how agents and contributors plan substantial work so another
person or agent can resume it without reconstructing the task from chat history.

An execution plan (ExecPlan) is a self-contained, living implementation record.
It explains the intended outcome, the relevant repository context, the current
state, decisions already made, concrete next steps, and how completion will be
proved. It is not a design essay, a terminal transcript, or the project backlog.

The canonical template is [docs/templates/exec-plan.md](docs/templates/exec-plan.md).
Active plans live in [docs/exec-plans/active/](docs/exec-plans/active/) and completed
plans move to [docs/exec-plans/completed/](docs/exec-plans/completed/).

## When an ExecPlan is required

Create or adopt an ExecPlan before editing when any of these conditions applies:

- the work is expected to span more than one focused session;
- the change crosses package, runtime, configuration, container, or UI boundaries;
- the change is safety-critical or changes live-mower behavior;
- the implementation has meaningful design uncertainty or multiple milestones;
- more than one agent or worktree will contribute;
- failure recovery, migration, or compatibility behavior must be coordinated.

A narrow, reversible, single-package change with obvious validation does not need
a checked-in plan. Its issue and pull request can carry the implementation record.
If a supposedly small change discovers one of the conditions above, create the
plan at that point rather than continuing with undocumented scope growth.

## Required properties

Every ExecPlan must be:

- **Self-contained.** Define repository-specific terms, name paths, and link to
  stable references. A new agent must not need the originating conversation.
- **Living.** Update it whenever progress, discoveries, decisions, validation, or
  the next action changes. Do not wait until the end of the task.
- **Outcome-oriented.** Describe observable behavior and acceptance evidence, not
  only files to edit.
- **Honest about evidence.** Distinguish observed, tested, inferred, and unverified
  claims. Include commands and concise results for completed validation.
- **Safe to resume.** End each update with a precise next action, current blockers,
  and any recovery or cleanup needed.
- **Bounded.** Link durable architecture and operating documentation instead of
  copying it. Link discovered follow-up issues rather than absorbing unrelated work.

Record decision rationale and evidence, not private chain-of-thought or a complete
chronological transcript. Preserve alternatives only when they clarify a tradeoff
that a future implementer may otherwise reopen.

## Required sections

Each plan must contain all of the following, even when a section currently says
that there is nothing to report:

1. **Purpose and intended outcome** — the user-visible result and why it matters.
2. **Progress** — timestamped checkboxes reflecting reality, plus the exact next
   action. Split partially completed work rather than marking it complete.
3. **Surprises & Discoveries** — unexpected repository facts, constraints, defects,
   or results with evidence.
4. **Decision Log** — decisions, dates, rationale, and any superseding ADR.
5. **Outcomes & Retrospective** — completed results, remaining gaps, and lessons.
6. **Context and Orientation** — relevant packages, entrypoints, boundaries, terms,
   branch, issue, and dependencies.
7. **Plan of Work** — milestone order and the behavior each milestone establishes.
8. **Concrete Steps** — exact working directory, commands, and expected outcomes.
9. **Validation and Acceptance** — tests, reviews, evidence, and success criteria.
10. **Idempotence and Recovery** — how reruns, rollback, interruption, and cleanup
    are handled safely.
11. **Artifacts and Interfaces** — files, APIs, schemas, topics, commands, reports,
    and external tracking records created or changed.

## Lifecycle

1. Copy the template into `docs/exec-plans/active/<issue-or-topic>.md`.
2. Set its owner, issue, branch/worktree, baseline commit, status, and creation date.
3. Update `docs/PROJECT_STATE.md` with one short row linking the active plan.
4. Work milestone by milestone. Update `Progress`, discoveries, decisions, and
   validation before switching agents, stopping, or changing scope.
5. Put durable architectural decisions in `docs/decisions/`; keep the plan's
   Decision Log as the dated implementation trail and link the ADR.
6. Put new out-of-scope work in the authoritative issue tracker with evidence and
   acceptance criteria. Link it from the plan; do not create a second backlog here.
7. At completion, reconcile stable docs, issue/PR state, commits, and acceptance
   evidence. Fill in `Outcomes & Retrospective` and remove obsolete next actions.
8. Move the plan to `docs/exec-plans/completed/` in the completion change, then
   remove its active row from `PROJECT_STATE.md`.

Completed plans are historical evidence. Correct factual errors with an explicit
dated note; do not rewrite past decisions to make the implementation look linear.

## Update discipline

- Use UTC dates in `YYYY-MM-DD` form; add times when ordering same-day handoffs
  matters.
- Reference commits with at least seven hexadecimal characters and link issues or
  PRs when their URLs exist.
- Do not mark work complete merely because code was written. Completion requires
  the validation and documentation named in the plan.
- Do not leave placeholders such as "investigate later" without an owner, issue,
  acceptance condition, or explicit reason for deferral.
- If repository evidence contradicts the plan, record the contradiction under
  `Surprises & Discoveries` and resolve the status before proceeding.
- If a plan becomes invalid, record why, mark it superseded or abandoned, link its
  replacement or issue, and move it to `completed/`; never silently delete it.
