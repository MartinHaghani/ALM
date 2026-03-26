# Docs maintenance

Purpose: explain how the documentation layer should evolve with the codebase.

## Keep the layers distinct

- `AGENTS.md` and `CLAUDE.md` are entrypoints, not long-form references.
- `.claude/rules/*.md` should stay short, scoped, and path-aware.
- `docs/*.md` hold durable explanations for humans and agents.
- nested `AGENTS.md` files should only exist where they materially reduce mistakes.

## Update the right file for the right change

- Change in launch composition or runtime wiring: update [ARCHITECTURE.md](ARCHITECTURE.md), [BUILD_AND_RUN.md](BUILD_AND_RUN.md), and the nearest agent guide.
- Change in config schema, env vars, or parameter loading: update [CONFIGURATION.md](CONFIGURATION.md), `config/AGENTS.md`, and any affected startup doc.
- Change in package inventory or ownership boundaries: update [PACKAGES.md](PACKAGES.md), [REPO_MAP.md](REPO_MAP.md), and `src/AGENTS.md`.
- Change in Dockerfiles, entrypoints, or development containers: update [DOCKER.md](DOCKER.md), [BUILD_AND_RUN.md](BUILD_AND_RUN.md), and `docker/AGENTS.md`.
- Change in simulation launches or behavior: update [SIMULATION.md](SIMULATION.md) and any affected architecture notes.
- Change in fork-versus-upstream behavior: update [UPSTREAM_SYNC.md](UPSTREAM_SYNC.md) and the nearest reference doc.

## Avoid duplication

- Keep commands in one or two canonical docs and link to them elsewhere.
- Keep root agent files concise and point to deeper docs instead of repeating full explanations.
- If a note is only relevant to one path family, put it in a scoped Claude rule or nested `AGENTS.md` rather than the root files.

## How to handle repeated mistakes

- If agents repeatedly edit the wrong directory, add or tighten a nested `AGENTS.md` or Claude rule.
- If contributors repeatedly miss a repo-specific drift point, document it in the closest durable reference doc.
- If a repo inconsistency is intentional, document the compatibility reason rather than leaving it implicit.

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

## Cross-layer alignment checklist

- Root entrypoints do not contradict `docs/*.md`.
- Scoped Claude rules match the actual path boundaries in the repo.
- Nested `AGENTS.md` files match the durable docs and do not invent new policy.
- README links still point readers into the maintained docs layer.
