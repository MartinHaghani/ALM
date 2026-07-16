# ADR 0001: Layered documentation and context model

- Status: Accepted
- Date: 2026-07-15
- Decision owners: repository maintainers
- Related plan: [Agent operating-system migration](../exec-plans/active/agent-operating-system-migration.md)

## Context

The repository has substantial architecture, package, hardware, and planner
documentation, but instructions, current status, implementation plans, decisions,
and outstanding work are not consistently separated. Some long planning documents
contain stale status beside useful design detail, and unmerged work can exist in
multiple worktrees. A new agent therefore has to search broadly and infer which
statements are current.

A single comprehensive memory file would be easy to start but expensive to keep
accurate. It would also place stable reference material, volatile status, and task
history into every agent's initial context whether relevant or not.

## Decision

Adopt a layered, progressively disclosed documentation model with one responsibility
per artifact:

- `AGENTS.md` files contain short, durable operating instructions and route readers
  to deeper sources. They do not carry workstream status or history.
- `docs/PROJECT_STATE.md` is a compact current-state cache and router. It links to
  evidence and active plans but is neither the backlog nor an architecture manual.
- `docs/exec-plans/active/` contains self-contained, living plans for substantial
  work. Completed plans move to `docs/exec-plans/completed/` as historical evidence.
- GitHub Issues and Projects become the authoritative dynamic backlog. Roadmaps may
  explain sequencing, but every actionable outstanding item must have a tracked
  owner, state, evidence, and acceptance criteria.
- `docs/decisions/` contains immutable ADRs for durable decisions and consequences.
- Existing domain documentation remains authoritative for how the system currently
  works. It should evolve gradually toward clear tutorial, how-to, reference, and
  explanation purposes rather than being reorganized in one disruptive pass.
- Git commits and pull requests form the implementation ledger and carry validation,
  review, risk, and issue linkage.

Agents record concise decision rationale, alternatives, evidence, assumptions, and
verification. They do not persist private chain-of-thought, indiscriminate chat
transcripts, or full terminal logs.

## Alternatives considered

### One repository memory document

Rejected because volatile status and durable knowledge would become interleaved,
causing frequent conflicts, duplicated facts, and excessive startup context.

### Use only issues and pull requests

Rejected because issues do not reliably provide the self-contained repository
orientation and recovery detail required for long-running, multi-agent work, and
they are unavailable when working offline.

### Keep the existing informal roadmaps and TODO files

Rejected as the target model because prose trackers do not provide uniform owners,
dependencies, automated status changes, or reliable closure. Existing trackers are
inputs to a deliberate issue migration and remain readable history.

## Consequences

- A new agent has a small deterministic reading set: repository instructions,
  `PROJECT_STATE.md`, and the relevant active ExecPlan.
- Volatile status must be updated at task boundaries and checked by automation.
- Work discovered outside the active scope must become an issue instead of a buried
  TODO or unplanned scope expansion.
- Some information will be linked across artifacts, but factual duplication should
  decrease because each artifact has a defined owner and purpose.
- Existing planning documents require gradual reconciliation; adopting this model
  does not make their current status claims correct automatically.
- Deterministic checks can validate structure, links, and required updates, while
  semantic correctness still requires review.

## Acceptance evidence

This decision is implemented when the repository contains the documented layers,
new substantial work uses the ExecPlan lifecycle, actionable discoveries are linked
to the issue tracker, and automated checks detect missing or inconsistent state.
