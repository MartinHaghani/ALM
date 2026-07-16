# Historical OpenMower comparison and imports

Purpose: explain ALM's lineage and the safe process for deliberately importing
changes from historical OpenMower repositories.

## Canonical repository identity

- [MartinHaghani/ALM](https://github.com/MartinHaghani/ALM) is the canonical,
  standalone ALM repository. GitHub reports it as a non-fork repository.
- `MartinHaghani/open_mower_ros` is the former fork and remains available during
  the migration confidence period as a rollback and historical reference.
- `ClemensElflein/open_mower_ros` is a historical comparison source, not ALM's
  governing upstream.
- Existing migration worktrees may temporarily keep the former fork as `origin`
  and ALM as `alm`. Inspect `git remote -v` before fetching, pushing, or changing a
  branch's tracking configuration; remote names alone are not authoritative.

ALM retains useful Git history, inherited GPLv3 provenance, and applicable
third-party notices. A standalone GitHub identity does not make inherited code a
clean-room implementation or authorize removing notices.

## Import model

- Treat every historical import as an intentional change, not a routine upstream
  synchronization.
- Use a dedicated branch, issue, and PR or a clearly isolated commit range.
- State the exact source repository and commit range in the plan and PR.
- Keep imported changes separate from ALM-specific behavior, documentation cleanup,
  generated output, and unrelated submodule updates.
- Reconcile imported assumptions with ALM's current runtime, safety boundaries,
  configuration, and hardware support instead of assuming compatibility.
- Re-run `git submodule update --init --recursive` when an import changes gitlinks.

## Document compatibility divergences

Record a divergence when it affects:

- build or launch expectations;
- configuration names or sources of truth;
- supported mower presets or hardware behavior;
- Docker and deployment assumptions;
- ROS, MQTT, D-Bus, schema, image, or persisted-data compatibility;
- external, generated, or submodule boundaries.

Use [CONFIGURATION.md](CONFIGURATION.md) for configuration behavior,
[BUILD_AND_RUN.md](BUILD_AND_RUN.md) for build and launch differences,
[PACKAGES.md](PACKAGES.md) or [REPO_MAP.md](REPO_MAP.md) for ownership changes, and
[DOCS_MAINTENANCE.md](DOCS_MAINTENANCE.md) for documentation policy.

## Review checklist

- Confirm the exact imported commits and licenses.
- Confirm whether submodule pointers changed and whether every pinned commit is
  remotely retrievable.
- Review safety-sensitive launch, motion, blade, emergency, hardware, and container
  behavior explicitly.
- Update configuration, architecture, build, or container documentation when the
  imported assumptions differ from ALM.
- Confirm whether generated or built assets under `web/` changed intentionally.
- Preserve compatibility-sensitive names unless a dedicated migration supplies
  aliases, versioned readers, rollback, and validation.

## Stop-and-ask cases

- Historical imports and ALM-specific behavior are interleaved so reviewers cannot
  isolate them.
- A submodule update introduces behavior not explained by the gitlink change.
- An import changes configuration semantics without the schema and docs updates.
- An import affects a safety boundary without risk, rollback, and controlled
  validation evidence.
- A proposed cleanup would remove inherited or third-party notices without
  documented rights or clean-room provenance.
