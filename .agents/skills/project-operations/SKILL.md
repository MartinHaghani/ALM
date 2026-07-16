---
name: project-operations
description: Keep repository work, execution plans, documentation, outstanding issues, validation, and Git state synchronized. Use for every change, build, fix, refactor, documentation edit, code review, migration, or multi-step investigation in this repository; also use when resuming prior work, handing work to another agent, discovering follow-up work, preparing commits or pull requests, or delegating repository tasks to subagents.
---

# Project Operations

Operate every repository task as a recoverable workstream. Preserve decisions and evidence, keep dynamic work in its designated source of truth, and leave the next agent an exact continuation point.

## Start the workstream

1. Read the applicable `AGENTS.md` files, then `docs/PROJECT_STATE.md` and the relevant active plan or issue. Read `PLANS.md` before creating or changing an ExecPlan.
2. Inspect the current branch, status, diff, recent commits, and linked worktrees. Treat pre-existing modifications as user work until proven otherwise.
3. Identify the task's issue, branch, worktree, acceptance criteria, safety classification, and canonical documents. State any assumption that could materially change the result.
4. Use or create an ExecPlan for work that is multi-step, spans packages or sessions, changes architecture or safety-sensitive behavior, or has meaningful uncertainty. Small, isolated changes may use the issue and final report instead.
5. For an authorized implementation task, create a narrow topic branch or isolated worktree when the current tree contains unrelated work. Never mix workstreams to save time.

## Delegate deliberately

Proactively delegate independent, bounded, read-heavy work when parallelism will materially improve speed or quality. Prefer the `explorer`, `test_analyst`, `correctness_reviewer`, and `docs_state_auditor` project agents.

- Keep requirements, final decisions, integration, and user communication with the parent agent.
- Give each subagent one concrete question, scope boundary, and output contract.
- Assign at most one writer to a file or shared worktree. Use separate worktrees for concurrent implementation.
- Ask subagents for distilled conclusions, exact file references, commands, evidence, uncertainty, and recommended follow-up—not raw logs.
- Do not delegate a task that is faster to perform locally than to explain and integrate.

## Keep the record current

Update an active ExecPlan at every meaningful milestone and before pausing. Record checked progress, unexpected discoveries, durable decisions and tradeoffs, validation evidence, blockers, and the exact next action.

Keep each kind of information in its authoritative location:

- Current cross-project snapshot: `docs/PROJECT_STATE.md`
- In-flight detail: `docs/exec-plans/active/`
- Durable decisions: `docs/decisions/`
- Outstanding work: linked GitHub issues and Project items
- Current behavior: stable package and operations documentation
- Implementation evidence: commits and pull requests

When new work is useful but outside the active acceptance criteria, do not silently expand scope. Create or update a linked GitHub issue with evidence, risk, dependencies, and acceptance criteria. If GitHub is unavailable, record a clearly identified pending issue in the active ExecPlan and surface it in the final report; migrate it to GitHub as soon as access returns.

Store decisions, rationale, alternatives, consequences, and evidence. Do not store hidden chain-of-thought, raw conversation transcripts, or uncurated terminal output.

## Change safely

- Keep changes narrow and preserve package, generated-code, vendored, submodule, and safety boundaries from `AGENTS.md`.
- Update stable documentation in the same change when behavior, interfaces, configuration, deployment, safety assumptions, or operator workflows change.
- Add an ADR only for a durable decision with meaningful alternatives or consequences. Never rewrite an accepted ADR to make history look current; supersede it.
- Run proportional checks early and again after the final edit. Capture exact commands and outcomes in the ExecPlan or pull request.
- Never perform live mower, VESC, deployment, or other physical safety operations without explicit authorization and the repository's required human review.

## Close or hand off

1. Review the complete diff and reconcile code, docs, ADRs, ExecPlan progress, project state, and linked issues.
2. Run `python3 scripts/agent/check_project_hygiene.py --scope changed` plus the task-specific tests. Resolve errors and assess every warning.
3. Create logical commits containing only this workstream. Use a Conventional Commit subject and a substantive body describing problem, decision, validation, risk, and follow-up when applicable.
4. When repository policy grants Git autonomy and credentials are available, push only the topic branch and create or update a draft pull request without requesting a separate prompt. Never merge, force-push, rewrite shared history, or push the default branch automatically.
5. Update or close the linked issue only when its acceptance criteria are actually satisfied. Move a completed ExecPlan to the completed directory and record outcomes.
6. Report the outcome, files or commits, verification, residual risks, outstanding linked issues, and exact next action. A new agent must be able to resume without reconstructing the session.
