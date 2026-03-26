# Upstream sync

Purpose: maintenance guidance for this fork of `open_mower_ros`.

## Observed fork facts

- This checkout has both `origin` and `upstream` remotes configured.
- `origin` points at `MartinHaghani/open_mower_ros`.
- `upstream` points at `ClemensElflein/open_mower_ros`.
- The repo contains fork-local drift that contributors should not miss, including:
  - the observed `Sabo` hardware preset under `src/open_mower/params/hardware_specific/`
  - config naming drift around `ESC_TYPE` versus `OM_MOWER_ESC_TYPE`
  - a doc layer that now explains repo-specific boundaries and workflows

## No repo-enforced sync policy was observed

This repository does not encode a required merge-versus-rebase policy in the files inspected during this documentation pass.

The guidance below is repo guidance for this fork, not a verified upstream mandate.

## Recommended sync model for this fork

- Keep upstream sync work in dedicated branches, PRs, or commit ranges.
- Keep fork-only customization in separate commits from sync work.
- Avoid combining upstream sync, local feature work, docs cleanup, and generated or submodule updates in one diff.
- Re-run `git submodule update --init --recursive` after sync work that touches submodule pointers.

## Document fork-specific divergences

Record a fork-specific divergence when it changes:

- build or launch expectations
- config names or sources of truth
- supported mower presets
- Docker runtime assumptions
- external or generated boundaries

Preferred places to record drift:

- [CONFIGURATION.md](CONFIGURATION.md) for config or env behavior
- [BUILD_AND_RUN.md](BUILD_AND_RUN.md) for build, launch, or container workflow differences
- [PACKAGES.md](PACKAGES.md) or [REPO_MAP.md](REPO_MAP.md) for package or ownership changes
- [DOCS_MAINTENANCE.md](DOCS_MAINTENANCE.md) if the divergence changes how docs should be maintained

## Commit boundary guidance

- Upstream sync commits should be easy to identify and revert independently.
- Fork-local behavior changes should explain why they exist and what upstream assumption they override.
- If a sync changes submodule pointers, call that out explicitly.
- If a fork-only patch touches safety-sensitive areas, keep its rationale in the commit message or PR description and update the docs.

## Review checklist for sync work

- Confirm whether any submodule pointer changed.
- Confirm whether launch, config, or container docs now need updates.
- Confirm whether a previously documented drift has been resolved and should be removed from the docs.
- Confirm whether the sync changed generated or built assets under `web/`.

## Stop-and-ask cases

- Upstream sync and fork-local behavior edits are interleaved in the same diff.
- A submodule update pulls in behavior changes that are not obvious from the pointer bump.
- A sync changes config semantics without corresponding schema or docs updates.
- A sync appears to invalidate the default versus legacy container split.
