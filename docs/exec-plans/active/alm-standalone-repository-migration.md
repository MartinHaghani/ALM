# ExecPlan: Establish ALM as the standalone canonical repository

- Status: Active
- Owner: coordinating Codex agent
- Created: 2026-07-15
- Last updated: 2026-07-15
- Issue: [#23](https://github.com/MartinHaghani/open_mower_ros/issues/23)
- Branch/worktree: `codex/agent-operating-system` at `/Users/martinhaghani/Code/open_mower_ros_agent_ops`
- Baseline commit: `2256dfd`
- Related ADRs: [ADR 0003](../../decisions/0003-standalone-alm-project-identity.md), [ADR 0002](../../decisions/0002-git-autonomy-and-safety-boundary.md)

This plan follows [PLANS.md](../../../PLANS.md). It is the authoritative living
handoff for preserving the current fork and establishing ALM without losing active
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
- [ ] Record the repository owner and visibility. `MartinHaghani/ALM` and
  `MME4487/ALM` appeared available during the 2026-07-15 read-only check.
- [ ] Snapshot and reconcile every dirty worktree, then create a complete old-ref
  bundle and reviewed migration manifest.
- [ ] Publish or replace the two local-only `xbot_driver_gps` commits so the pinned
  superproject gitlink is retrievable by a fresh clone.
- [ ] Complete or preserve draft PR #22 and ensure the agent operating system is
  present on the intended ALM default branch.
- [ ] Create the empty non-fork ALM repository with Actions disabled during initial
  ref import, then push the reviewed branches and deliberate tag set.
- [ ] Migrate GitHub work tracking, settings, protections, Actions, secrets,
  automations, and product-facing branding.
- [ ] Normalize license/package metadata and add machine-readable third-party and
  license validation without removing inherited notices.
- [ ] Verify a fresh recursive clone, builds, governance, metadata, and rollback;
  archive the old fork only after maintainer cutover approval.

Exact next action: record whether ALM will live under `MartinHaghani` or an
organization and whether it will remain public.

## Surprises & Discoveries

- Seven linked worktrees, twelve local branches, eighteen origin branches, and seven
  tags exist. The main worktree has 23 modified or untracked paths and one Claude
  worktree has five modified documentation paths.
- No local commit is unreachable from all origin refs, but origin contains 113
  commits reachable only through remote branch refs. Pushing only local branches or
  the default branch would lose historical heads.
- Superproject HEAD pins `src/lib/xbot_driver_gps` at `bd05076`, while its local
  `codex/pi-rover-fixes` branch is two commits ahead of GitHub. The pinned commit is
  not remotely retrievable.
- All 376 paths present at the fork point remain in the current tree. Of those, 293
  are byte-identical and 83 modified; the project is substantially extended but not
  a clean-room rewrite.
- First-party manifests still claim `CC BY-NC-SA 4.0` while the root and source
  history are GPLv3-only; `src/mower_utils/package.xml` says `TODO`. This is metadata
  debt to correct, not authority to remove notices.
- The initial ref import must not enable tag-triggered Actions accidentally. Existing
  version tags match the container publication workflow and could publish historical
  images under ALM.
- Repository branding is much smaller than runtime compatibility migration. ROS
  package names, `OM_*`, `OPEN_MOWER_*`, schema identifiers, MQTT defaults, D-Bus
  paths, container paths, and systemd names need deliberate aliases or versioned
  transitions rather than global replacement.

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

## Outcomes & Retrospective

The decision, initial evidence, blockers, and migration acceptance criteria are now
durable and issue-backed. No ALM repository has been created, no active worktree has
been rewritten, and no remote has been changed. Owner/visibility selection,
worktree preservation, submodule publication, and PR #22 disposition remain before
the external cutover can safely begin.

## Context and Orientation

The source repository is `/Users/martinhaghani/Code/open_mower_ros`. Its local
`origin` is `MartinHaghani/open_mower_ros`; `upstream` is
`ClemensElflein/open_mower_ros`. The isolated agent-governance worktree is
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

Next, create an empty ALM repository under the selected owner without using GitHub's
fork action. Keep Actions disabled while importing curated refs so historical tags
cannot trigger image publication. Compare source and destination object/ref
manifests before selecting and protecting the ALM default branch.

Then migrate issue/backlog content, labels, settings, protections, workflows,
secrets, automations, and other external state. Update repository-facing ALM
branding, URLs, badges, clone/deployment instructions, maintainers, and generated
asset sources. Preserve compatibility-sensitive identifiers until a separate
migration supplies aliases and versioned readers.

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
git clone --recurse-submodules git@github.com:OWNER/ALM.git /tmp/alm-acceptance
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
- `gh api repos/OWNER/ALM --jq '{fork,parent,source}'` reports `fork: false` and no
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
- Outstanding work: [issue #23](https://github.com/MartinHaghani/open_mower_ros/issues/23).
- Source GitHub repository: `MartinHaghani/open_mower_ros`.
- Destination repository: `OWNER/ALM`, not yet created.
- Historical source remote: `ClemensElflein/open_mower_ros`.
- Critical dependency boundary: `src/lib/xbot_driver_gps` gitlink `bd05076` and its
  two local-only commits.
- Preservation artifacts to create: worktree ledger, Git ref manifest, restorable
  bundle, GitHub metadata/settings export, submodule reachability report, and
  post-cutover verification report.
- Compatibility interfaces not implicitly renamed: ROS packages/messages/topics,
  `OM_*`, `OPEN_MOWER_*`, schema IDs, MQTT/D-Bus names, container paths, image names,
  systemd units, and persisted configuration.

## Plan Change Log

- 2026-07-15 — Created the ALM standalone-repository migration plan from repository,
  GitHub, license, branding, and submodule audits.
