# Coverage Planner V2 Prototype Plan

Status: historical implementation plan. The initial prototype and later M1/M2.x
diagnostics are present from `a0b5896` onward. Current advancement and acceptance
work is tracked in [issue #8](https://github.com/MartinHaghani/open_mower_ros/issues/8)
and the status block in
[COVERAGE_PLANNER_V2_ALGORITHM_PLAN.md](COVERAGE_PLANNER_V2_ALGORITHM_PLAN.md).

Purpose: implementation plan for the first lab-only V2 prototype: read an OpenMower `map.json`, condition it at mower scale, classify macro-zones/corridors/pockets/unreachable features, and write an inspectable geometry report. This prototype intentionally does **not** replace the existing planner or produce live mower-ready paths.

Related context:

- [COVERAGE_PLANNER_V2_DESIGN.md](COVERAGE_PLANNER_V2_DESIGN.md): overall V2 architecture and future maneuver-aware contract.
- [COVERAGE_PLANNER_LAB.md](COVERAGE_PLANNER_LAB.md): current lab command and artifact conventions.
- [../tools/coverage_lab/AGENTS.md](../tools/coverage_lab/AGENTS.md): offline lab guardrails.

## Scope

This milestone builds the V2 **coverage problem model**, not the route planner.

In scope:

- Add a lab-only V2 command that reads the same `map.json` format as the current lab.
- Build cleaned/conditioned mower-scale geometry from each selected mow area.
- Compute footprint-safe drivable geometry from the configured mower footprint and safety margin.
- Classify meaningful macro-regions:
  - wide mowable body zones;
  - narrow corridors;
  - small pockets;
  - unreachable or too-tight notches;
  - boundary wrinkles removed during conditioning.
- Write JSON metrics and an HTML/SVG report.
- Run on `tools/coverage_lab/data/maps/current/map.json` and tracked examples.

Out of scope for this milestone:

- Stripe generation.
- Turn planning.
- Endpoint routing.
- `coverage_plan_v2.json` route segments beyond a stub/schema shell.
- ROS service/message changes.
- Live mower reverse, pivot, or blade-state execution.

The important architectural rule is that all future route output must be maneuver-aware. The first prototype may only output classified geometry, but it should name future segment fields consistently with the V2 design.

## Proposed CLI

Add a new command to `tools/coverage_lab/coverage_lab.py`:

```bash
tools/coverage_lab/bin/coverage_lab classify-v2 \
  --map tools/coverage_lab/data/maps/current/map.json \
  --output tools/coverage_lab/runs/v2-current-map-classify
```

Options:

- `--map`: required OpenMower map JSON.
- `--config`: default `tools/coverage_lab/configs/default.yaml`.
- `--output`: optional run directory.
- `--area-index`: classify one active mow area.
- `--repair-rings`: same meaning as current commands.
- `--conditioning-profile`: `conservative | normal | aggressive`, default `normal`.
- `--simplify-tolerance-m`: optional override for mower-scale simplification.
- `--min-zone-area-m2`: optional override.
- `--corridor-width-factor`: optional override, default tied to mower footprint width and tool width.

Batch can come later. For the first implementation, run the command manually on the recorded map and examples.

## New Files

Preferred new files:

- `tools/coverage_lab/v2_geometry.py`
  - mower-scale conditioning and classification helpers;
  - host-testable without Fields2Cover imports.
- `tools/coverage_lab/v2_report.py`
  - V2-only SVG/HTML rendering helpers if the main `coverage_lab.py` would become too large.
- `docs/COVERAGE_PLANNER_V2_PROTOTYPE_PLAN.md`
  - this plan.

Allowed changes to existing files:

- `tools/coverage_lab/coverage_lab.py`
  - add CLI command;
  - reuse `parse_map`, `load_config`, `write_json`, and run-dir helpers;
  - delegate V2 geometry to `v2_geometry.py`.
- `tools/coverage_lab/README.md`
  - add one short V2 prototype command once implemented.
- `docs/COVERAGE_PLANNER_V2_DESIGN.md`
  - link this plan.
- `docs/COVERAGE_PLANNER_LAB.md`
  - mention the new lab-only command once implemented.

Do not change mower runtime packages for this milestone.

## Output Artifacts

Each `classify-v2` run should write:

- `v2_geometry_report.html`: primary human report.
- `v2_geometry.svg`: static layer view.
- `v2_geometry.json`: detailed classified geometry, preliminary zones, local-width samples, ridge diagnostics, neck candidates, and metadata.
- `v2_metrics.json`: summary counts, areas, local-width statistics, ridge counts, and neck-candidate status/reason counts.
- `source_map_snapshot.json`: copied input map.
- `config_snapshot.json`: effective V2 config.

Optional later:

- `coverage_plan_v2.json`: schema stub with `segments: []`, useful once downstream route code starts.

Do not write `planpath_compat.json` in this milestone. That output implies route compatibility, and this prototype is not producing a route.

Update: a later V2 bridge now writes a lossy `planpath_compat.json` from eligible forward, blade-on V2 task-path segments so the new planner can be compared against the old mower-facing shape. That bridge is diagnostic/backward-compatible output, not the final maneuver-aware route contract.

## Geometry Conditioning Pipeline

For each selected lawn:

1. **Raw polygon construction**
   - Convert OpenMower lawn and holes to Shapely polygons.
   - Preserve the raw outline and holes in the report.
   - Validate area, ring closure, and hole containment.

2. **Mower model extraction**
   - Read `footprint`, `tool_width`, `tool_center_offset`, and `safety_margin_m` from config.
   - Compute:
     - physical footprint length/width;
     - safety footprint bbox;
     - `base_link` all-yaw radius for V2 maneuver diagnostics;
     - tool-center all-yaw radius for comparison with the old Fields2Cover wrapper;
     - cutter radius `tool_width / 2`.

3. **Boundary conditioning**
   - Remove near-duplicate and tiny zig-zag segments smaller than a configurable mower-scale tolerance.
   - Use Shapely simplify with topology preservation as a first pass.
   - Use open/close morphology carefully:
     - small inward notches below footprint scale can be marked unreachable or smoothed out;
     - narrow corridors wider than the safe footprint must be preserved.
   - Record every area delta:
     - raw area;
     - conditioned area;
     - removed wrinkle area;
     - filled/smoothed notch area.

4. **Drivable region**
   - Erode conditioned lawn by the configured V2 boundary clearance, defaulting to `v2_drivable_boundary_clearance_m` (`0.10 m` on the current lab config). This is the main drivable/coverage interior shown to the operator.
   - Separately compute the more conservative `base_link` footprint-disk erosion as an `all_yaw_safe_region` diagnostic. That larger inset is useful for arbitrary pivots and unconstrained-yaw maneuvers, but it should not define the whole drivable area.
   - Split resulting `MultiPolygon` into drivable islands.
   - Mark islands below `min_zone_area_m2` as pockets or unreachable.

5. **Coverage band**
   - Compute boundary band as `conditioned_lawn - drivable_region`.
   - Classify it into:
     - normal headland band;
     - unreachable notch band;
     - corridor shoulder;
     - tiny wrinkle removed/ignored.
   - This directly addresses the current 61% coverage symptom by making the lost band visible before route generation.

## Macro-Zone Classification

The first version should be deliberately simple and explainable.

Recommended first-pass classification:

- **Main body zone:** a drivable polygon component with area above `min_zone_area_m2` and both major dimensions comfortably larger than the mower footprint.
- **Corridor:** region where local width is narrow but still drivable; candidate criteria:
  - short-axis extent below `corridor_width_factor * footprint_width`;
  - long-axis extent several times larger than short-axis extent.
- **Pocket:** small drivable component or lobe that can fit the mower but may not be worth stripe routing.
- **Unreachable:** raw/conditioned lawn area where the footprint-safe drivable erosion disappears.
- **Wrinkle/noise:** area removed by conditioning because it is below mower-scale tolerance.

Implementation options in order:

1. Start with polygon components plus oriented minimum rotated rectangle metrics.
2. Add negative/positive buffer neck detection to identify corridors:
   - erode by a corridor probe radius;
   - observe where components split;
   - classify necks between larger components.
3. Later, add medial-axis/skeleton analysis if simple geometry is not enough.

Do not overfit the first version. The report should show enough geometry for us to decide whether the classification is useful.

## JSON Shape

`v2_geometry.json` should be stable enough to build on:

```json
{
  "schema": "open_mower.coverage_lab.v2_geometry.v0",
  "map": "...",
  "frame_id": "map",
  "mower_model": {
    "footprint": [[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]],
    "tool_width_m": 0.4,
    "tool_center_offset": [0.41, 0.0],
    "safety_margin_m": 0.05,
    "v2_drivable_boundary_clearance_m": 0.10,
    "base_link_all_yaw_radius_m": 0.9535,
    "tool_center_all_yaw_radius_m": 0.6031
  },
  "areas": [
    {
      "area_index": 0,
      "source_id": "...",
      "raw_area_m2": 71.88,
      "conditioned_area_m2": 0.0,
      "drivable_area_m2": 0.0,
      "coverage_band_area_m2": 0.0,
      "zones": [
        {
          "id": "zone-0-main",
          "kind": "main_body",
          "preliminary": true,
          "area_m2": 0.0,
          "polygon": [[0.0, 0.0]],
          "oriented_bounds": {
            "length_m": 0.0,
            "width_m": 0.0,
            "angle_deg": 0.0
          }
        }
      ],
      "width_samples": [],
      "ridge_points": [],
      "ridge_links": [],
      "neck_candidates": [],
      "features": [
        {
          "id": "feature-0",
          "kind": "unreachable_notch",
          "area_m2": 0.0,
          "polygon": [[0.0, 0.0]],
          "reason": "below footprint-safe width"
        }
      ],
      "warnings": []
    }
  ]
}
```

Polygons may use a compact ring format. If holes are needed, use:

```json
{
  "outer": [[x, y], ...],
  "holes": [[[x, y], ...]]
}
```

## Metrics

`v2_metrics.json` should summarize:

- raw mow area;
- conditioned mow area;
- drivable area;
- all-yaw-safe area;
- boundary band area;
- maneuver-limited area, meaning drivable area that is not safe for arbitrary-yaw pivots;
- unreachable area;
- removed wrinkle/noise area;
- zone counts by kind;
- largest zone area;
- number of drivable islands;
- min/median/max zone width;
- warnings.

For the recorded current map, include a comparison block with the current baseline:

```json
{
  "baseline_current_planner": {
    "coverage_percent": 61.12425212188674,
    "cells": 13,
    "connector_length_m": 122.78850445043075
  }
}
```

This comparison is informational. The classifier is not expected to beat coverage yet because it does not route.

## HTML Report

The first report should be visual, not fancy.

Required layers:

- raw map outline;
- conditioned outline;
- footprint-safe drivable region;
- macro-zones colored by kind;
- unreachable/notch/noise features;
- boundary coverage band;
- mower footprint scale marker;
- legend and metrics table.

Interactions can be minimal:

- layer checkboxes;
- static SVG pan/zoom can come later;
- link to raw JSON artifacts.

The report must make the recorded-map failure obvious: if the map boundary has many tiny wrinkles or tight notches, the report should show which are preserved, smoothed, or declared unreachable.

## Acceptance Criteria

First implementation is accepted when:

- `python3 -m py_compile tools/coverage_lab/coverage_lab.py tools/coverage_lab/v2_geometry.py` passes.
- `python3 -m doctest tools/coverage_lab/v2_geometry.py` passes if doctests are added.
- `COVERAGE_LAB_NO_OPEN=1 tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/data/maps/current/map.json --output tools/coverage_lab/runs/v2-current-map-classify` completes.
- The run writes `v2_geometry_report.html`, `v2_geometry.svg`, `v2_geometry.json`, `v2_metrics.json`, `source_map_snapshot.json`, and `config_snapshot.json`.
- The recorded map report shows fewer macro-zones than the current planner's 13 cells unless the geometry genuinely requires more.
- The report shows local-width, ridge, candidate neck-cut, and preliminary-zone layers. Candidate cuts are diagnostic evidence only and do not rewrite zones yet.
- Every unreachable or removed area is quantified in `v2_metrics.json`.
- No route, reverse, pivot, or blade execution is implied by the report.

Secondary validation:

```bash
COVERAGE_LAB_NO_OPEN=1 tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/examples/rectangle_map.json
COVERAGE_LAB_NO_OPEN=1 tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/examples/l_shape_map.json
COVERAGE_LAB_NO_OPEN=1 tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/examples/narrow_pivot_map.json
COVERAGE_LAB_NO_OPEN=1 tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/examples/obstacle_map.json
```

Expected behavior:

- rectangle: one main body, no meaningful unreachable features;
- L-shape: one or two macro-zones depending on neck classification, not many tiny cells;
- narrow pivot: corridor classification should be visible;
- obstacle: obstacle-driven drivable split or corridor/pocket features should be visible without route warnings.

## Implementation Order

1. Add `v2_geometry.py` with pure geometry helpers:
   - polygon conversion;
   - mower model extraction;
   - conditioning config;
   - oriented bounds;
   - geometry-to-JSON serialization.
2. Add `classify_lawn_v2(lawn, area_index, config, options)` returning a serializable dict.
3. Add `classify_map_v2(model, map_path, config, selected, options)`.
4. Add basic `v2_metrics.json` generation.
5. Add simple SVG/HTML report.
6. Add `classify-v2` parser and command in `coverage_lab.py`.
7. Run on recorded map and examples.
8. Review the generated report before adding route generation.

## Design Notes For The Next Milestone

Once the classifier is useful, the next milestone should generate candidate swaths per macro-zone and score them by cutter-swept area. It should still output maneuver-aware segment shells, even if all segments are forward-only at first:

```json
{
  "kind": "SWATH",
  "blade_state": "ON",
  "direction": "FORWARD",
  "controller_mode": "FOLLOW_PATH",
  "base_link_poses": []
}
```

Reverse, pivot-wheel turns, and blade-off phases should enter as lab-only segment types after endpoint routing exists. Live mower support remains a separate runtime milestone.
