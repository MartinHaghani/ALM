# Code review

Purpose: reusable review checklist for human reviewers and agents working in this repo.

## Review priorities

- Safety-sensitive behavior comes first.
- Launch and configuration changes come before style or cleanup comments.
- External, submodule, and generated boundaries must be checked before accepting broad edits.

## Safety-sensitive checklist

- Does the change affect `mower_logic`, `mower_hardware`, legacy `mower_comms_*`, launch wiring, entrypoints, or hardware-specific params?
- Could the change alter mower motion, parking/docking, emergency handling, battery thresholds, GPS behavior, or hardware comms?
- Are default values, thresholds, or parameter names changing?
- Is the validation story strong enough for the risk level?

## Launch and config checklist

- Are launch file includes, node names, or remaps changing?
- Does `src/open_mower/launch/include/_params.launch` still match the documented config model?
- If config semantics changed, were `config/mower_config.schema.json`, `config/mower_config.sh.example`, and [CONFIGURATION.md](CONFIGURATION.md) updated together?
- If a change touches legacy compatibility, does it account for `docker/openmower_entrypoint.legacy.sh`?

## External, generated, and submodule checklist

- Does the diff touch `src/lib/`, `services/`, or `web/`?
- If yes, is the edit intentional and clearly called out?
- Is the change a submodule pointer update or a direct content edit?
- Is there any broad formatting churn in external or generated areas that should be rejected?

## Package and architecture checklist

- Does the change cross ROS package boundaries unnecessarily?
- Does it keep catkin conventions intact?
- If package responsibilities moved, were [PACKAGES.md](PACKAGES.md), [REPO_MAP.md](REPO_MAP.md), or [ARCHITECTURE.md](ARCHITECTURE.md) updated?

## Docs and contributor impact checklist

- Will a new contributor understand the updated workflow?
- Did the change invalidate any verified commands or paths in the docs layer?
- Were ALM-specific, lineage, or compatibility divergences documented instead of hidden?

## Validation checklist

- Were changed paths and commands actually verified?
- For docs-only changes, were links and relative paths checked?
- For behavior changes, was the level of verification proportional to the risk?
- If runtime validation was not performed, does the summary say that clearly?

## Stop-and-ask triggers

- The change alters safety-sensitive logic without clear validation.
- Config artifacts drift further apart instead of being reconciled or documented.
- A diff mixes historical-source imports, ALM feature work, and submodule or generated edits.
- A reviewer cannot tell whether `web/` or `src/lib/` edits are intentional.
- The runtime effect of a Docker or launch-file change is unclear.
