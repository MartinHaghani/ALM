# Mowrator slope reliability working path

Purpose: track the staged implementation that makes the `Mowrator` mower terrain-aware on steep slopes without mutating the user-recorded map geometry.

## Scope

- Target hardware: `Mowrator` only for the first implementation cycle.
- Primary goal: improve path-tracking consistency and mowing reliability on steep cross-slopes, uphills, and downhills.
- Non-goal: replace the existing map workflow in pass 1 or pass 2.
- Constraint: keep `map.json` as the user-authored source of truth and store learned terrain evidence separately in `terrain_memory.json`.

## Working branch

- Branch: `codex/mowrator-slope-reliability-working-path`
- Base commit: `6e5b9f2a751ccdc675f485e9ce7c371609438426`
- This document is the authoritative handoff log for each pass.

## Pass plan

### Pass 0: docs and branch bootstrap

Status: planned

Deliverables:

- add this working-path document
- link it from the docs index
- reserve commit boundaries for the terrain implementation work

Commit target:

- `docs: add mowrator slope reliability working path`

### Pass 1: control foundation and terrain telemetry

Status: planned

Deliverables:

- increase navigation controller cadence so slope correction is not bottlenecked by a `1 Hz` loop
- launch a filtered-IMU path and a new terrain observer node
- publish `mower_logic/terrain_state`
- extend robot-state monitoring with terrain telemetry
- add replay-friendly debug signals for steep-slope runs

Acceptance:

- repeatable terrain telemetry during steep-slope runs
- no control-loop starvation
- enough evidence to identify where drift begins and in which direction

Commit target:

- `nav: add terrain observer and higher-rate control foundation`

### Pass 2: adaptive slope-aware controller and recovery

Status: planned

Deliverables:

- subscribe to terrain state in `ftc_local_planner`
- schedule speed, lookahead, lateral authority, and heading bias from terrain state
- add bounded terrain recovery before generic abort handling
- surface terrain recovery state through monitoring

Acceptance:

- no off-map excursion during repeated steep-edge traversals
- no `Robot is far away from global plan` aborts on the validation edge
- no blade-on stationary wait caused by terrain tracking loss

Commit target:

- `nav: add adaptive slope tracking and recovery`

### Pass 3: learned terrain intelligence and plan-time modifiers

Status: planned

Deliverables:

- persist terrain evidence in `terrain_memory.json`
- preview upcoming terrain risk during runtime
- synthesize transient caution or exclusion holes from learned high-risk cells before coverage planning
- keep learned exclusions derived and clearable without editing `map.json`

Acceptance:

- repeated runs pre-compensate before the known steep edge
- previously troublesome segments stop needing reactive recovery
- the test lawn can be completed without the mower drifting off the map

Commit target:

- `nav: add learned terrain memory and plan modifiers`

## Validation log

Record each field session here after implementation commits land.

### Template

- date:
- branch commit:
- lawn:
- scenario:
- max cross-track error:
- terrain recovery count:
- completion rate:
- left mapped area:
- parameter overrides used:
- notes:

## Open questions for future passes

- whether Mowrator-specific defaults should stay entirely in `params_v2.yaml` or partly migrate into shared defaults after validation
- whether terrain memory needs an explicit UI action for clearing, or whether file-level tooling is enough after first validation
- whether learned high-risk cells should only slow the mower at first, or be allowed to create transient plan holes immediately
