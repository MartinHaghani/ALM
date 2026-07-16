# ADR 0003: Establish ALM as a standalone project identity

- Status: Accepted
- Date: 2026-07-15
- Decision owners: Martin Haghani
- Related issue/plan: [issue #23](https://github.com/MartinHaghani/ALM/issues/23) and the [ALM migration ExecPlan](../exec-plans/active/alm-standalone-repository-migration.md)
- Supersedes: None

## Context

GitHub records `MartinHaghani/open_mower_ros` as a fork of
`ClemensElflein/open_mower_ros`. The maintained product has developed a distinct
identity and needs its own canonical repository, release surface, governance, and
agent context without GitHub presenting it as an upstream fork.

The repository is substantially changed but not a clean-room replacement. At the
current fork point, all 376 tracked baseline paths remain in the current tree: 293
are byte-identical and 83 are modified. The root license is GPLv3-only, inherited
OpenMower copyright and license notices remain in first-party source, and vendored
or submodule dependencies carry MIT, BSD, Apache-2.0, AGPLv3, Boost, and other
independent obligations. Product branding and repository identity can therefore
change independently of provenance and licensing; a new name does not establish
ownership of inherited expression or authorize notice removal.

The current repository also contains active state that cannot be reconstructed from
one branch: multiple dirty worktrees, origin-only branch history, a draft pull
request, issues, GitHub settings, and an unpublished submodule commit pinned by the
superproject. A cutover needs explicit preservation and verification rather than a
rename, detach, or single-branch push.

## Decision

Create a new, non-fork GitHub repository named `ALM` and make it the canonical
project repository after a verified migration.

- Preserve a curated full Git history, required branch heads, and tags. Do not
  squash history merely to obscure fork ancestry.
- Make the new ALM repository `origin`. Keep the historical OpenMower repository as
  an explicitly named legacy remote only when ongoing comparison is useful; it is
  not ALM's governing upstream.
- Keep GPL-3.0-only as the initial ALM project license and retain applicable
  inherited and third-party notices. Add ALM modification and ownership notices for
  qualifying new work without replacing earlier notices.
- Permit a different license only for a demonstrably separable work whose ownership
  and clean provenance are documented. Relicensing inherited work requires rights
  from its copyright holders or a clean-room replacement.
- Change repository and product-facing branding to ALM. Treat ROS package names,
  message identities, topics, configuration keys, container paths, persisted schema
  identifiers, and deployment names as compatibility interfaces; do not rename them
  through a global replacement.
- Resolve every dirty worktree, remote-only ref, GitHub work item, setting, secret,
  automation, and submodule reachability risk before cutover.
- Archive the old fork with a permanent migration notice only after a fresh clone,
  build, governance, and metadata audit succeeds and the maintainer approves the
  cutover.

## Alternatives considered

### Rename the current GitHub repository

A rename would preserve the fork-network relationship and therefore would not meet
the independent-identity requirement.

### Detach or delete and recreate the fork

GitHub's detach path is permanent and does not retain important repository metadata.
Deleting the current fork before a verified migration would also remove the safest
rollback and historical issue/PR reference surface.

### Start with a squashed snapshot and remove prior notices

A snapshot would discard useful blame, decision, and contributor evidence without
changing the licensing status of inherited code in the current tree. The current
repository has no copyright assignment or contributor-license mechanism that grants
the maintainer unilateral relicensing authority.

### Rename every OpenMower-derived technical identifier immediately

This would create unnecessary ROS, configuration, deployment, and persisted-data
breakage. ALM can have an independent public identity while compatibility-sensitive
interfaces are migrated separately with aliases and versioned readers.

## Consequences

- ALM gains an independent GitHub identity, canonical remote, and release surface.
- The migration must preserve and reconcile more than Git commits: issues, PR
  handoffs, labels, settings, branch protections, Actions, secrets, automations,
  submodules, generated outputs, and deployment documentation.
- Public branding can change before runtime identifiers, producing a deliberate
  compatibility period in which ALM still contains names such as `open_mower`,
  `mower_msgs`, `OM_*`, and `OPEN_MOWER_*`.
- License metadata must be normalized. First-party `package.xml` files currently
  conflict with the GPLv3-only root/source state, and `mower_utils` still declares a
  placeholder license.
- ALM needs machine-readable license and third-party inventory checks so future
  agents can distinguish first-party branding from protected notices and external
  boundaries automatically.
- The old fork remains available as a read-only historical redirect through a
  confidence period; it is not deleted as part of the initial migration.

## Acceptance evidence

- GitHub reports the new canonical repository as `ALM` with `fork: false`.
- A reviewed ref manifest proves that all intended branches, tags, and commits were
  preserved, and every dirty worktree has an explicit disposition.
- A clean recursive clone retrieves every submodule and passes documented build,
  agent-policy, link, workflow, and repository-hygiene validation.
- ALM product surfaces and canonical URLs use the new name while compatibility
  identifiers are documented rather than silently rewritten.
- GPLv3-only and third-party license data, applicable notices, package metadata, and
  automated license checks agree.
- The old fork is archived with a visible ALM migration link only after cutover
  approval.
