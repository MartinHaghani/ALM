# Docs maintenance

Purpose: explain how the documentation layer should evolve with the codebase.

## Keep the layers distinct

- `AGENTS.md` and `CLAUDE.md` are entrypoints, not long-form references.
- `.claude/rules/*.md` should stay short, scoped, and path-aware.
- `docs/*.md` hold durable explanations for humans and agents.
- nested `AGENTS.md` files should only exist where they materially reduce mistakes.
- [PROJECT_STATE.md](PROJECT_STATE.md) is a concise current snapshot and router, not a backlog or changelog.
- [exec-plans/](exec-plans/) hold self-contained living implementation state for substantial work.
- [decisions/](decisions/) hold durable decisions and consequences, not task progress.
- GitHub Issues are the source of truth for actionable outstanding work; the GitHub Project is the intended status/priority view. Stable docs may link to issues but should not duplicate their state.
- Commits and pull requests are the implementation and validation ledger, not a substitute for current reference docs.

Record decisions, rationale, alternatives that affected the outcome, evidence, and
remaining uncertainty. Do not preserve private chain-of-thought, indiscriminate chat
transcripts, or complete terminal output as project documentation.

## Update the right file for the right change

- Change in launch composition or runtime wiring: update [ARCHITECTURE.md](ARCHITECTURE.md), [BUILD_AND_RUN.md](BUILD_AND_RUN.md), and the nearest agent guide.
- Change in config schema, env vars, or parameter loading: update [CONFIGURATION.md](CONFIGURATION.md), `config/AGENTS.md`, and any affected startup doc.
- Change in package inventory or ownership boundaries: update [PACKAGES.md](PACKAGES.md), [REPO_MAP.md](REPO_MAP.md), and `src/AGENTS.md`.
- Change in Dockerfiles, entrypoints, or development containers: update [DOCKER.md](DOCKER.md), [BUILD_AND_RUN.md](BUILD_AND_RUN.md), and `docker/AGENTS.md`.
- Change in simulation launches or behavior: update [SIMULATION.md](SIMULATION.md) and any affected architecture notes.
- Change in fork-versus-upstream behavior: update [UPSTREAM_SYNC.md](UPSTREAM_SYNC.md) and the nearest reference doc.
- Change in active owner, state, blocker, or authoritative plan: update [PROJECT_STATE.md](PROJECT_STATE.md) with one evidence-linked row.
- Substantial multi-session, cross-package, or safety-sensitive work: create or update an ExecPlan under [exec-plans/active/](exec-plans/active/) according to [PLANS.md](../PLANS.md).
- Durable architecture or operating decision: add an ADR under [decisions/](decisions/) and link it from the active plan.
- Newly discovered out-of-scope work: create a structured issue with evidence and acceptance criteria; link it rather than adding an unowned prose TODO.
- Pre-policy first-party markers: keep the exact-content ownership map in
  [legacy-todos.json](legacy-todos.json) aligned. New markers must link issues
  inline and must not be added to the legacy register.

## Avoid duplication

- Keep commands in one or two canonical docs and link to them elsewhere.
- Keep root agent files concise and point to deeper docs instead of repeating full explanations.
- If a note is only relevant to one path family, put it in a scoped Claude rule or nested `AGENTS.md` rather than the root files.
- Keep volatile status out of stable reference docs. Link to the active plan or issue instead.
- Do not copy a plan's progress or decision log into `PROJECT_STATE.md`; keep only the current state and authoritative link.

## Keep document purposes distinct

Use the Diátaxis distinction for stable reader documentation:

- tutorial: a guided learning experience;
- how-to: steps for a concrete operational goal;
- reference: exact facts, interfaces, parameters, and commands;
- explanation: architecture, concepts, rationale, and tradeoffs.

ExecPlans, ADRs, project state, issues, and PRs are project-governance records, not
additional Diátaxis categories. Avoid mixing a live task checklist into a reference
page or copying reference material into a plan. Reclassify and improve existing
files incrementally; do not perform a mass move that breaks links and obscures Git
history.

## How to handle repeated mistakes

- If agents repeatedly edit the wrong directory, add or tighten a nested `AGENTS.md` or Claude rule.
- If contributors repeatedly miss a repo-specific drift point, document it in the closest durable reference doc.
- If a repo inconsistency is intentional, document the compatibility reason rather than leaving it implicit.
- If a plan or state claim contradicts repository evidence, record the discrepancy and resolve it before adding more policy.

## Labels to preserve

Keep these labels explicit where they matter:

- Source of truth
- Deprecated
- External/Submodule
- Generated
- Not yet verified
- Safety-critical

## Minimum verification for doc updates

- Check that every referenced path exists.
- Check that relative links resolve.
- Do not claim commands were executed unless they really were.
- If the repo files disagree, say so plainly instead of choosing one silently.
- Confirm every active ExecPlan has all sections required by [PLANS.md](../PLANS.md), a precise next action, and current validation evidence.
- Confirm every `PROJECT_STATE.md` workstream row points to its authoritative current source and remove completed rows.
- Confirm new ADR numbers are unique and the [decision index](decisions/README.md) matches their status.

## Cross-layer alignment checklist

- Root entrypoints do not contradict `docs/*.md`.
- Scoped Claude rules match the actual path boundaries in the repo.
- Nested `AGENTS.md` files match the durable docs and do not invent new policy.
- README links still point readers into the maintained docs layer.
- `PROJECT_STATE.md`, active ExecPlans, issues, commits, and stable docs agree about what is current, outstanding, and verified.
