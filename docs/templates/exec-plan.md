# ExecPlan: Outcome-oriented title

- Status: Proposed | Active | Blocked | Completed | Superseded | Abandoned
- Owner: agent or person responsible for integration
- Created: YYYY-MM-DD
- Last updated: YYYY-MM-DD
- Issue: URL or `Not yet created — reason`
- Branch/worktree: branch and path, or `Not created`
- Baseline commit: seven-or-more-character SHA
- Related ADRs: links or `None`

This plan follows [PLANS.md](../../PLANS.md). It must remain self-contained and be
updated whenever progress, decisions, discoveries, validation, blockers, or the
next action changes.

## Purpose and Intended Outcome

Explain what will be observably different for the user or operator and why it
matters. Define success without relying on the originating conversation.

## Progress

- [ ] (YYYY-MM-DD) First factual milestone.
- [ ] (YYYY-MM-DD) Second factual milestone.

Exact next action: name one concrete, safe action a new owner can take immediately.

## Surprises & Discoveries

- Observation with repository path, command result, test output, or commit evidence.
- If none: `None recorded yet.`

## Decision Log

- YYYY-MM-DD — Decision. Rationale and consequences. Link an ADR when durable.

## Outcomes & Retrospective

Record completed results against the intended outcome, remaining gaps with issue
links, and lessons that should change future work. While active, say what remains.

## Context and Orientation

Name the relevant packages, entrypoints, documents, terminology, safety boundaries,
dependencies, issue, branch, and worktree. Explain enough for a repository newcomer
to navigate the work safely.

## Plan of Work

Describe the ordered milestones in prose. For each, state the behavior or evidence
it establishes and how it prepares the next milestone.

## Concrete Steps

Give the working directory and exact commands. Include expected signals that
distinguish success from failure. Update this section when commands change.

```bash
cd /absolute/path/to/worktree
# command
```

## Validation and Acceptance

List observable acceptance criteria, tests, reviews, documentation checks, and
artifact inspection. For completed checks, record the command and concise result;
do not merely say "tests pass."

## Idempotence and Recovery

Explain which steps can be rerun, how partial work is recognized, how to recover
from interruption, how to roll back safely, and what cleanup must wait until merge.

## Artifacts and Interfaces

List files, APIs, ROS topics/services, schemas, commands, generated reports, issues,
PRs, and external systems created or changed. Identify the source of truth for each.

## Plan Change Log

- YYYY-MM-DD — Created plan.
