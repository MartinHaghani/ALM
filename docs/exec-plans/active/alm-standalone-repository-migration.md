# ExecPlan: Establish ALM as the standalone canonical repository

- Status: Active
- Owner: coordinating Codex agent
- Created: 2026-07-15
- Last updated: 2026-07-15
- Issue: [#23](https://github.com/MartinHaghani/ALM/issues/23)
- Branch/worktree: `codex/agent-operating-system` at `/Users/martinhaghani/Code/open_mower_ros_agent_ops`
- Baseline commit: `329726f` (migration base)
- Related ADRs: [ADR 0003](../../decisions/0003-standalone-alm-project-identity.md), [ADR 0002](../../decisions/0002-git-autonomy-and-safety-boundary.md)

This plan follows [PLANS.md](../../../PLANS.md). It is the authoritative living
handoff for preserving the former fork and establishing ALM without losing active
work, history, metadata, dependencies, or operational knowledge.

## Purpose and Intended Outcome

ALM will be an independently created GitHub repository, not a GitHub fork. It will
own the canonical product identity, repository URL, governance, and release surface
while retaining the full useful implementation history and applicable provenance.
A fresh agent or contributor must be able to clone ALM recursively, identify the
current workstreams and decisions, run the documented checks, and continue work
without consulting the archived fork or the originating conversation.

The migration changes repository identity and public branding. It must not silently
change mower runtime behavior, ROS API identities, deployed configuration contracts,
or licensing terms.

## Progress

- [x] (2026-07-15) The maintainer selected `ALM` as the new product and repository
  name and selected a standalone repository rather than continued fork identity.
- [x] (2026-07-15) Audited local remotes, worktrees, branches, tags, dirty state,
  GitHub metadata, branding surfaces, licensing boundaries, and submodule
  reachability.
- [x] (2026-07-15) Created issue #23 as the authoritative work item and accepted
  ADR 0003 for the durable identity and provenance decision.
- [x] (2026-07-15) Selected the public personal repository
  `MartinHaghani/ALM`.
- [x] (2026-07-15) Snapshotted every dirty worktree and created a complete old-ref
  bundle, submodule bundles, GitHub metadata exports, and recovery artifacts under
  `/Users/martinhaghani/Code/ALM_migration_backup_20260715T210630-0400`.
- [x] (2026-07-15) Published the two `xbot_driver_gps` commits through
  `codex/pi-rover-fixes`; the pinned `bd05076` gitlink is remotely retrievable.
- [x] (2026-07-15) Preserved draft PR #22 by recreating its head/base refs and
  draft PR in ALM, then retargeted it from the identical integration ref to protected
  `main`. ALM `main` remains at `329726f`, so migration has not implicitly merged
  the draft agent-operations work.
- [x] (2026-07-15) Created the public, empty, non-fork ALM repository with Actions
  disabled, then imported only the reviewed active branches and no historical tags.
- [x] (2026-07-15) Recreated labels, issues #1–#21, draft PR #22, and issue #23 with
  provenance and original numbering; reproduced repository protection, security,
  merge, and least-privilege Actions settings.
- [x] (2026-07-15) Verified an independent fresh recursive clone, `git fsck`, every
  top-level and nested submodule gitlink, and a clean `main` checkout.
- [x] (2026-07-15) Updated product-facing canonical documentation, exported
  post-migration GitHub target records, regenerated and verified `SHA256SUMS`, and
  re-verified the complete 53-ref source bundle.
- [x] (2026-07-15) Left the former fork active for the confidence period while
  changing its repository description and homepage to direct visitors to ALM.
- [x] (2026-07-15) Migrated both active Codex automations in place to ALM names,
  canonical `alm/main` reads, explicit `alm` pushes, ALM issue/PR targets, and
  fail-closed remote validation while preserving their schedules and safety gates.
- [x] (2026-07-15) Recorded successful ALM push and pull-request policy runs
  `29464674958` and `29464692054` at `9d8aa7e`.
- [x] (2026-07-15) Imported the narrowly scoped `input`-group fix from historical
  comparison commit `c7d1b715`; ALM Build run `29465378523` passed all four
  default/legacy, amd64/arm64 jobs at `dde57b3`.
- [ ] Confirm policy and container checks again on the immutable head selected for
  maintainer review; exact run evidence belongs in PR #22 and issue #23 rather than
  a self-invalidating status-only commit.
- [ ] Normalize license/package metadata and add machine-readable third-party and
  license validation without removing inherited notices.
- [ ] Complete build/governance/rollback acceptance and the confidence period;
  archive the former fork only after explicit maintainer approval.

Exact next action: confirm the ALM policy gate and all four container validation
builds for the current head of draft PR #22, then hand the still-draft PR to the
maintainer for review without merging it absent explicit approval.

## Surprises & Discoveries

- Seven linked worktrees, twelve local branches, eighteen origin branches, and seven
  tags exist. The main worktree has 23 modified or untracked paths and one Claude
  worktree has five modified documentation paths.
- No local commit is unreachable from all origin refs, but origin contains 113
  commits reachable only through remote branch refs. Pushing only local branches or
  the default branch would lose historical heads.
- The superproject pins `src/lib/xbot_driver_gps` at `bd05076`. Publishing its two
  pre-existing local commits to `codex/pi-rover-fixes` made the exact gitlink
  retrievable; a fresh recursive clone verified it.
- All 376 paths present at the fork point remain in the current tree. Of those, 293
  are byte-identical and 83 modified; the project is substantially extended but not
  a clean-room rewrite.
- First-party manifests still claim `CC BY-NC-SA 4.0` while the root and source
  history are GPLv3-only; `src/mower_utils/package.xml` says `TODO`. This is metadata
  debt to correct, not authority to remove notices.
- The initial ref import must not enable tag-triggered Actions accidentally. Existing
  version tags match the container publication workflow and could publish historical
  images under ALM.
- The migration therefore imported no historical tags into ALM. They remain in the
  verified preservation bundle and former repository.
- ALM `main` intentionally remains at `329726f`; moving the agent-operations branch
  to `main` would merge draft PR #22 without the required maintainer approval.
- Recreated PR #22 initially retained its historical integration-branch base. That
  branch and `main` both pointed to `329726f`, so retargeting the draft to protected
  `main` changed neither its diff nor its merge status and restored the intended
  review boundary.
- Enabling normal repository automation caused Dependabot to open
  [PR #24](https://github.com/MartinHaghani/ALM/pull/24) for Vite `8.1.4`.
  `npm audit` identifies the pinned `8.0.14` as the one current high-severity
  development dependency; keep that automated PR separate and review its build
  evidence rather than silently absorbing it into the identity migration.
- The inherited `clang-format` pre-commit hook's default file types included JSON
  and JavaScript, so a two-line schema branding edit initially rewrote hundreds of
  unrelated JSON lines. Restoring both JSON files from the branch baseline and
  overriding the hook to compiled-language types removed the churn and made the
  changed-file ratchet deterministic for schema and WebUI metadata.
- The root Flutter UI under `web/` is generated from a separate `OpenMowerApp`
  source and still displays legacy branding. Direct edits would violate the
  source/output boundary, so [issue #25](https://github.com/MartinHaghani/ALM/issues/25)
  owns the source-side rebrand, reproducible rebuild, and browser smoke test.
- Repository branding is much smaller than runtime compatibility migration. ROS
  package names, `OM_*`, `OPEN_MOWER_*`, schema identifiers, MQTT defaults, D-Bus
  paths, container paths, and systemd names need deliberate aliases or versioned
  transitions rather than global replacement.
- ALM Build run `29464676667` did not report four independent failures: the
  arm64/default job reached `docker/Dockerfile` lines 85–89 and failed because the
  minimal ROS base omitted the `input` group; fail-fast then canceled the other
  three matrix jobs. Historical comparison commit `c7d1b715` adds that missing
  group at the OSv2 GID `996`, and its original four-architecture GitHub matrix
  completed successfully. Import only that hunk and re-run ALM's complete matrix.
- The current SHA-pinned checkout and Docker actions emit GitHub's Node 20 runtime
  deprecation warning. [Issue #26](https://github.com/MartinHaghani/ALM/issues/26)
  owns supported-runtime upgrades so the container portability fix stays isolated.

## Decision Log

- 2026-07-15 — Name the independent project and repository ALM. See ADR 0003.
- 2026-07-15 — Create a new non-fork repository rather than renaming or detaching
  the existing fork; preserve the old repository as a rollback and historical
  reference until verified cutover.
- 2026-07-15 — Preserve curated full Git history. A clean GitHub identity does not
  require a fabricated clean-room history.
- 2026-07-15 — Separate ALM product branding from ROS, deployment, configuration,
  schema, and persisted-data identifiers. Compatibility changes require their own
  reviewed migrations.
- 2026-07-15 — Keep GPL-3.0-only and applicable inherited/third-party notices for
  initial migration. Any future relicensing requires documented rights or clean-room
  replacement evidence.
- 2026-07-15 — Import only reviewed active branches, not historical tags or abandoned
  refs. Preserve the complete old namespace in the verified bundle and former repo.
- 2026-07-15 — Keep ALM `main` at `329726f` and recreate PR #22 as draft rather than
  treating repository migration as approval to merge it.
- 2026-07-15 — Retarget draft PR #22 to protected `main` after verifying its former
  base resolves to the same `329726f`; preserve the diff while avoiding an
  unprotected integration path.
- 2026-07-15 — Import only the `input`-group creation hunk from historical
  comparison commit `c7d1b715`, with the exact source and independent upstream
  matrix evidence recorded. Do not merge unrelated historical changes into ALM.

## Outcomes & Retrospective

The standalone public ALM repository, curated active refs, work-item numbering,
draft PR #22, governance, security settings, preservation artifacts, and dependency
reachability now exist. A fresh recursive clone of ALM `main` at `329726f` completed
with every nested submodule and passed `git fsck`. No active worktree was rewritten,
the former fork remains available for rollback, and PR #22 remains unmerged.

ALM policy-check evidence now exists. The initial ALM container matrix exposed a
missing base-image group in the default image, and the focused fix passed all four
jobs at `dde57b3`. Any later reviewed head must retain green policy and container
evidence; license/package normalization and the confidence-period closeout remain.
Archiving the former fork remains explicitly human-gated.

Validation evidence recorded on 2026-07-15:

- `git clone --recurse-submodules git@github.com:MartinHaghani/ALM.git
  /Users/martinhaghani/Code/ALM_acceptance_20260715` — passed at `329726f`; all six
  top-level and nested gitlinks were retrieved and the checkout was clean.
- `git fsck --full` in the acceptance clone — passed with no diagnostics.
- `python3 scripts/agent/check_project_hygiene.py --root . --scope all --strict`
  and changed scope against `329726f` — both passed with zero errors and warnings.
- Agent-policy unit tests — 29 passed, including commit-range/exception/bot bypass
  cases; PR-policy unit tests — 9 passed; fresh-agent context evaluation — 8/8
  assertions passed.
- Pinned pre-commit hooks on the changed files and `git diff --check` — passed after
  the `clang-format` file-type correction; a before/after diff digest confirmed the
  final hook run made no changes.
- Downloaded `actionlint` 1.7.12 for Darwin arm64, verified its release checksum,
  and ran it against the workflows — passed with no diagnostics.
- ALM `Project policy` runs `29464674958` (push) and `29464692054` (pull request) —
  passed at `9d8aa7e`.
- ALM Build run `29464676667` — arm64/default failed after the ROS build because
  `adduser openmower input` referenced a missing group; the remaining three matrix
  jobs were canceled by fail-fast. All four jobs for the exact fix in historical
  comparison commit `c7d1b715` passed in run `24735999486`; ALM current-head
  confirmation followed in Build run `29465378523`, where all four jobs passed at
  `dde57b3`.
- `npm ci`, `npm run typecheck`, and `npm run build` under local Node `24.13.1` —
  passed; the ignored `web/next/` output contains the ALM title/brand. Vite retained
  its existing bundle-size warning and `config.js` warning.
- `./utils/scripts/web/build_next_webui.sh` — not run to completion because the
  local Docker daemon was unavailable; the supported Node 22 container build remains
  part of final acceptance.
- `npm audit --json` — one high-severity Vite development dependency finding at
  `8.0.14`; Dependabot PR #24 proposes the non-major fix `8.1.4`.
- `shasum -a 256 -c SHA256SUMS` — every preservation artifact passed; `git bundle
  verify` reported the 53-ref bundle complete.

GitHub ran the non-publishing container matrix and exposed the missing ARM64
base-image group recorded above; the current head still requires a complete green
matrix. No local ROS build, Pi validation, deployment, image publication, or
live-hardware operation was run in this repository-identity checkpoint.

## Context and Orientation

The former source worktree is `/Users/martinhaghani/Code/open_mower_ros`. During the
staged migration its shared local `origin` remains `MartinHaghani/open_mower_ros`,
`upstream` remains `ClemensElflein/open_mower_ros`, and `alm` points to the canonical
`MartinHaghani/ALM`. The isolated agent-governance worktree is
`/Users/martinhaghani/Code/open_mower_ros_agent_ops` on
`codex/agent-operating-system`. Do not stage, clean, reset, or rewrite another
worktree while preparing this migration.

Read [PROJECT_STATE.md](../../PROJECT_STATE.md), [ADR 0003](../../decisions/0003-standalone-alm-project-identity.md), issue #23, and the branch-local status before
acting. `mower_logic`, communications, hardware configuration, launch composition,
container entrypoints, and live mower operations remain safety-sensitive even when a
change appears to be a rename.

The migration has four distinct surfaces:

1. Git object/ref and dirty-work preservation.
2. GitHub repository identity, settings, and work-item migration.
3. Public ALM branding and canonical URLs.
4. Optional compatibility-sensitive ROS, configuration, deployment, or schema
   migration, which is not part of the initial cutover unless separately planned.

Generated `web/`, `src/lib/**`, and `services/**` remain generated or external
boundaries. Do not bulk-edit their branding or notices.

## Plan of Work

First, establish an immutable preservation ledger. Record every worktree status,
branch/ref/tag SHA, submodule gitlink and remote reachability, dirty patch or commit,
and GitHub metadata/settings export. Create a full old-repository Git bundle before
changing remotes. Resolve the unpublished GPS submodule commits and document the
disposition of draft PR #22.

The public ALM repository was created without GitHub's fork action. Actions remained
disabled while curated refs were imported so historical tags could not trigger image
publication. The target ref manifest and default branch were then checked before
protection and security settings were applied.

Issues, the draft PR, labels, settings, protections, workflow permissions, empty
secret/variable inventories, and scheduled automations have been migrated. Finish
repository-facing ALM branding, URLs, badges, and clone instructions. Preserve
compatibility-sensitive identifiers until a separate migration supplies aliases and
versioned readers.

Finally, normalize license declarations and add machine-readable SPDX/REUSE and
third-party checks. Verify a clean recursive clone and supported builds, test agent
orientation and GitHub policy, audit external state, and retain rollback evidence.
Only after the confidence period and explicit maintainer approval should the old
fork be archived with a permanent ALM link.

## Concrete Steps

Operate from the isolated worktree or a new dedicated ALM migration worktree; never
from a dirty feature worktree:

```bash
cd /Users/martinhaghani/Code/open_mower_ros_agent_ops
git status --short --branch
git worktree list --porcelain
git for-each-ref --format='%(refname) %(objectname)'
git submodule status --recursive
```

Before creating ALM, write the reviewed worktree/ref/submodule/GitHub inventory and
create an independently restorable bundle. Record hashes rather than claiming that
a push preserved history by name alone.

After ALM exists, import only the reviewed refs while Actions are disabled. Do not
run a blind `git push --mirror`: remote-tracking, pull-request, temporary Claude,
and tag-triggering refs need deliberate treatment.

Clone the destination into a new directory for validation rather than reusing an
existing object store:

```bash
git clone --recurse-submodules git@github.com:MartinHaghani/ALM.git /tmp/alm-acceptance
cd /tmp/alm-acceptance
git fsck --full
python3 scripts/agent/check_project_hygiene.py --root . --scope all
python3 -m unittest discover -s scripts/agent/tests -p 'test_*.py'
python3 scripts/agent/evaluate_agent_context.py
pre-commit run --all-files
git diff --check
```

Use the documented ROS, container, and WebUI environments for their respective
builds. Do not claim a Pi, GHCR, or live-hardware check unless it was actually run.

## Validation and Acceptance

- The source bundle restores all inventoried old refs without network access.
- Every worktree's dirty state is preserved by a reviewed commit, patch artifact, or
  explicit documented abandonment; no agent infers abandonment from age or branch
  name.
- Every ALM branch and tag SHA matches its reviewed manifest, and historical tags
  do not trigger unintended publication.
- `gh api repos/MartinHaghani/ALM --jq '{fork,parent,source}'` reports `fork: false` and no
  fork parent/source.
- `git clone --recurse-submodules` succeeds from an empty local object cache.
- The repository hygiene, agent policy, fresh-agent, pre-commit, link, workflow,
  whitespace, ROS, container, and WebUI checks appropriate to changed scope pass
  with exact results recorded.
- Issues and outstanding work retain source links and acceptance criteria; the old
  draft PR has either landed, been recreated with a handoff, or remains permanently
  linked from the archived repository.
- Default-branch protection, least-privilege Actions permissions, branch cleanup,
  CODEOWNERS, merge policy, secrets, and automations match the documented operating
  model.
- Product surfaces identify ALM. Technical identifiers retained for compatibility
  are explicitly documented; renamed interfaces have backward-compatible readers or
  aliases and dedicated validation.
- GPLv3-only, third-party notices, package metadata, generated notices, and
  machine-readable license checks agree.
- The archived old fork contains a visible ALM link and the preservation bundle and
  rollback procedure remain available through the confidence period.

## Idempotence and Recovery

Inventories, bundles, clone verification, and destination comparisons are
repeatable and read-only. Repository creation, issue recreation, secret placement,
and old-repository archival are external state changes; query the destination first
and record created IDs so retries update rather than duplicate state.

If migration is interrupted, leave old remotes and the old GitHub repository active.
Do not delete worktrees, branches, or tags merely because a destination ref exists.
Restore from the source bundle when a ref is missing. Disable ALM publication
workflows until ref import and secret review are complete. If a fresh recursive
clone cannot retrieve a submodule commit, stop cutover and repair reachability rather
than substituting an unreviewed dependency commit.

Rollback before archive means keeping the old repository canonical and marking ALM
as incomplete. After archive, unarchive the old repository and restore its canonical
notice if a critical build, dependency, metadata, or deployment gap appears. No
history rewrite or repository deletion is part of this plan.

## Artifacts and Interfaces

- Durable decision: `docs/decisions/0003-standalone-alm-project-identity.md`.
- Active execution state: this file.
- Outstanding work: [issue #23](https://github.com/MartinHaghani/ALM/issues/23).
- Source GitHub repository: `MartinHaghani/open_mower_ros`.
- Canonical destination repository: `MartinHaghani/ALM`.
- Historical source remote: `ClemensElflein/open_mower_ros`.
- Critical dependency boundary: `src/lib/xbot_driver_gps` gitlink `bd05076`, now
  published through its `codex/pi-rover-fixes` branch and verified by fresh clone.
- Preservation artifacts: worktree ledger, Git ref manifest, restorable bundle,
  source and target GitHub metadata/settings exports, submodule reachability report, and recovery
  scripts under `/Users/martinhaghani/Code/ALM_migration_backup_20260715T210630-0400`.
- Scheduled automations: `ALM project hygiene` weekly and `ALM agent context
  regression` every four weeks. Both resolve the canonical `alm` remote explicitly
  while the saved Codex project remains the shared confidence-period checkout.
- Generated legacy-root-UI rebrand and rebuild:
  [issue #25](https://github.com/MartinHaghani/ALM/issues/25).
- SHA-pinned action supported-runtime upgrade:
  [issue #26](https://github.com/MartinHaghani/ALM/issues/26).
- Host-aware, least-privilege runtime input-device mapping:
  [issue #27](https://github.com/MartinHaghani/ALM/issues/27).
- Compatibility interfaces not implicitly renamed: ROS packages/messages/topics,
  `OM_*`, `OPEN_MOWER_*`, schema IDs, MQTT/D-Bus names, container paths, image names,
  systemd units, and persisted configuration.

## Plan Change Log

- 2026-07-15 — Created the ALM standalone-repository migration plan from repository,
  GitHub, license, branding, and submodule audits.
- 2026-07-15 — Recorded the public repository creation, curated-ref import,
  work-tracking/governance migration, submodule publication, preservation backup,
  and successful independent recursive-clone acceptance. Kept the former repository
  active and PR #22 draft pending explicit approval.
- 2026-07-15 — Migrated both scheduled Codex jobs in place to ALM names, canonical
  remote handling, issue/PR destinations, current frontier model, and fail-closed
  safety behavior without duplicating their schedules.
- 2026-07-15 — Kept generated `web/` out of the branding diff and created issue #25
  for a traceable source-side Flutter rebrand and rebuild.
- 2026-07-15 — Recorded ALM policy evidence, the precise first container-matrix
  failure, its one-hunk historical source, and issue #26 for the separate action
  runtime upgrade.
