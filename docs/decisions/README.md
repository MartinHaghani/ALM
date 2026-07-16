# Architecture decision records

Purpose: index durable technical and operating decisions whose rationale must
survive individual tasks, agents, and branches.

An architecture decision record (ADR) captures one significant decision, its
context, meaningful alternatives, and consequences. ADRs do not track execution
progress or outstanding work. Use an ExecPlan for implementation state and an issue
for backlog state.

## Status values

- `Proposed`: under active review and not yet governing work.
- `Accepted`: current policy or architecture.
- `Deprecated`: retained for compatibility but should not guide new work.
- `Superseded`: replaced; the record must link its successor.
- `Rejected`: considered and deliberately not adopted.

Accepted ADRs are immutable historical records. When a decision changes, add a new
ADR and mark the old one superseded rather than rewriting its original rationale.
Small factual corrections may be added as dated amendments.

## Index

| ADR | Status | Decision |
|---|---|---|
| [0001](0001-agent-documentation-and-context-model.md) | Accepted | Use a layered, progressively disclosed documentation and context model |
| [0002](0002-git-autonomy-and-safety-boundary.md) | Accepted | Automate routine Git work while preserving review gates for destructive and safety-sensitive actions |

Use [the ADR template](../templates/adr.md) for new decisions. Allocate the next
four-digit number without renumbering existing records.
