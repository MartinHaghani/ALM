# Coverage Planner Lab

Purpose: describe the laptop-only Fields2Cover evaluation setup in `tools/coverage_lab/`.

This lab is intentionally separate from mower runtime code. It does not change launch files, ROS packages, map service behavior, Docker runtime entrypoints, or `mower_logic`. It exists to test Fields2Cover on real and synthetic OpenMower maps before any mower-side migration.

## Workflow

Build the isolated Docker image and verify the Python binding:

```bash
tools/coverage_lab/bin/coverage_lab build
tools/coverage_lab/bin/coverage_lab smoke
```

Copy a mower map manually into the ignored local data directory:

```bash
scp <mower-user>@<mower-host>:~/.ros/map.json tools/coverage_lab/data/maps/current/map.json
```

Or convert Google Earth KML polygons into the same JSON shape. Name each KML polygon with a case-insensitive `mow:`, `obstacle:`, or `nav:` prefix:

```bash
tools/coverage_lab/bin/coverage_lab convert-kml --kml ~/Downloads/front_yard.kml
tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/google_earth/front_yard.json
```

For a directory of KML files:

```bash
tools/coverage_lab/bin/coverage_lab convert-kml --kml-dir ~/Downloads/lawn_kmls
tools/coverage_lab/bin/coverage_lab batch --maps tools/coverage_lab/data/maps/google_earth
```

Validate and plan:

```bash
tools/coverage_lab/bin/coverage_lab validate-map --map tools/coverage_lab/data/maps/current/map.json
tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/current/map.json
```

Generate the V2 diagnostic report, optionally with operator task-boundary annotations exported from the report. In the report, use **Annotations** -> **Start drawing**, click the two ends of the task mouth cut, then click the service side to keep. The panel keeps draft cuts visible, stores them in the browser for that map, and shows the rerun command after export.

```bash
tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/data/maps/current/map.json
tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/data/maps/current/map.json --task-annotations v2_task_annotations.json
```

After successful `plan`, `batch`, or `render` commands, the wrapper opens the generated HTML report on the laptop. Use `COVERAGE_LAB_NO_OPEN=1` when running unattended:

```bash
COVERAGE_LAB_NO_OPEN=1 tools/coverage_lab/bin/coverage_lab batch --maps tools/coverage_lab/examples
```

Tracked examples can be exercised without a private mower map:

```bash
tools/coverage_lab/bin/coverage_lab batch --maps tools/coverage_lab/examples
```

## Current State

The coverage lab is current as an offline planning/evaluation workspace. It is not current as a mower integration, and that separation is intentional.

The main implementation lives in `tools/coverage_lab/coverage_lab.py`; use `tools/coverage_lab/bin/coverage_lab` for normal operation so the command runs inside the isolated Fields2Cover Docker image. The CLI currently supports:

- `validate-map`: validate OpenMower-style JSON maps.
- `classify-v2`: run the lab-only V2 map-conditioning and macro-zone classification prototype.
- `convert-kml`: convert Google Earth KML polygons into OpenMower-style JSON maps.
- `plan`: generate primary and comparison profile artifacts.
- `render`: regenerate SVG/HTML/preview artifacts for an existing run directory.
- `batch`: run planning over a directory of map JSON files and write an index.

The current primary profile is `mowrator_zero_turn`. Fields2Cover is still used for swath generation and ordering, but the lab owns the mower-specific wrapper around it: footprint-disk headlands, obstacle-aware cell decomposition, dominant-direction stripe-angle selection, tight-cell angle overrides, explicit transits/fill bridges, and footprint-checked turn maneuvers. The comparison profile is `f2c_tiny_radius`, which runs Fields2Cover's built-in path planner with a small turn radius for side-by-side evaluation.

Private real maps and generated reports live under ignored paths such as `tools/coverage_lab/data/maps/` and `tools/coverage_lab/runs/`. A future agent may inspect local ignored artifacts to understand the current debugging case, but must not commit them unless the user explicitly requests a scrubbed sample.

## Inputs And Outputs

The input is the current `mower_map_service` JSON shape: `areas[]` with `properties.type` values such as `mow`, `nav`, and `obstacle`, plus optional `docking_stations[]`.

The planner reads active `mow` areas as lawns and active `obstacle` areas as holes when the obstacle centroid lies inside that lawn. Docking stations are preserved in the source snapshot but are not used for planning.

Each run writes ignored artifacts under `tools/coverage_lab/runs/`:

- `plan.svg`: visual overlay of lawn boundary, obstacles, generated swaths, headlands, and path.
- `plan.html`: evaluation report with an interactive slicer-style mower footprint preview, including pan/zoom/rotate controls.
- `v2_geometry_report.html`: when using `classify-v2`, a lab-only geometry report with raw/conditioned outlines, the configured mowable/drivable inset, a separate all-yaw-safe diagnostic inset, local-width heatmap, prototype clearance-ridge centerline, service-region-grown task proposals, operator annotation cuts, service-ownership adjustments, mouth cuts, dead-end terminal caps, M2 local task-path prototypes, M2.1/M2.5 service-axis arrows, M2.6 side-task axis-candidate scoring diagnostics, M2.7 BODY centerline-axis scoring, M2.2 turnaround envelopes, M2.3 annotation-QA overlays comparing automatic task evidence to operator annotations, M2.4 body-side portal candidate search metadata, candidate neck cuts, preliminary zones, boundary band, and unreachable/noise features. The report has map zoom/pan controls, simplified primary/advanced layer groups, and hover `i` explanations for each layer. It does not imply route or live mower support.
- `v2_annotation_qa.json`: when `classify-v2` is run with `--task-annotations`, a diagnostic comparison of operator-drawn task mouths against the automatic task proposals, including matched, missed, type-mismatch, mouth-misaligned, partial, and unmatched-automatic records.
- `v2_task_paths.json`: when using `classify-v2`, diagnostic local task-path prototypes for visible task proposals. These paths carry service-axis, stripe-safe region, supplemental BODY pocket service, lane-anchor, exit-mode, turnaround-feasibility, direction, blade, local coverage, and unsafe-footprint metadata, but they are not globally ordered.
- `v2_planpath_compat_summary.json`: when using `classify-v2`, diagnostic summary of the lossy bridge from V2 task paths into the old `planpath_compat.json` shape. The default bridge exports only forward, blade-on `base_link` segments and reports skipped reverse, rotate, or blade-off segments.
- `lawn_preview.png`: static raster grass-finish preview rendered from `simulation_preview.json`; overgrown grass, mowed footprint/cutting disk, wheel-track marks, and obstacle holes are composited in pose order.
- `planpath_compat.json`: `PlanPath`-like JSON with map-frame `paths[]`.
- `metrics.json`: approximate coverage and safety metrics, plus swath length, headland, cell, transit, fill-bridge, turn-planner, and per-cell-angle summaries. Metrics are scoped to the selected lawn when `--area-index` is used.
- `source_map_snapshot.json`: reproducibility copy of the input map.
- `planning_debug.json`: additional generated geometry for re-rendering and agent inspection.
- `simulation_preview.json`: flattened `base_link` execution timeline, per-pose footprint safety data, and lab-only maneuver metadata for wheel-anchor turns.
- `config_snapshot.json`: effective lab config used for the run.

Each primary run is the `mowrator_zero_turn` profile. Comparison output for Fields2Cover's built-in small-radius path planner is written under `profiles/f2c_tiny_radius/`. Batch runs also write `batch.html`, a small index that links to each generated `plan.html`.

## Defaults

The default config is `tools/coverage_lab/configs/default.yaml`.

It mirrors current Mowrator planning assumptions where possible:

- `tool_width: 0.4`
- `tool_center_offset: [0.41, 0.0]`
- `headland_strategy: footprint_disk` (P0). The headland centreline is the boundary of `lawn ⊖ disk(footprint_disk_radius)`, computed by [tools/coverage_lab/lab_geometry.py](../tools/coverage_lab/lab_geometry.py). The disk radius bakes in `safety_margin_m` so the safety footprint is guaranteed inside the lawn at every yaw. Legacy `f2c` strategy is preserved for comparison.
- `cell_decomposition: true` (P1). The mainland is split into hole-free Boustrophedon cells before Fields2Cover swath generation; cells are stitched together with explicit transit segments that walk the eroded boundary.
- `dominant_direction_smoothing_deg: 3.0` (P12). The global stripe angle is selected from a smoothed, length-weighted edge-angle histogram on the eroded boundary when that histogram has a clear peak.
- `tight_cell_stripe_threshold_factor: 2.0` (P13). Cells whose short-axis extent is less than this factor times `tool_width` can override the global stripe angle and align with the cell's long axis.
- `outline_count: 1`
- `outline_clearance_m: auto`, which derives the first outline inset from the mower footprint plus configured safety margin (only used when `headland_strategy: f2c`; the `footprint_disk` strategy computes its own clearance directly)
- `outline_offset: 0.0`
- `v2_outline_generation_mode: footprint_side_fit`, which emits executable `base_link` poses whose right footprint side tracks the conditioned boundary as closely as the configured clearance and sampled turn geometry allow.
- `v2_outline_right_footprint_clearance_m: auto`, which uses the outline footprint clearance and falls back to `safety_margin_m`; set `v2_outline_footprint_clearance_m: 0.0` only for a no-extra-clearance diagnostic run against a trusted boundary.
- `v2_outline_yaw_window_m: auto`, used only by the legacy V2 outline modes that convert tool-center samples to `base_link`.
- footprint: `[[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]]`
- `drive_model: zero_turn`
- `pivot_yaw_step_degrees: 10`
- `wheel_track_m: 0.58`
- `wheel_contact_x_m: 0.0`
- `turn_planner: wheel_anchor`
- `turn_cutting_mode: off`
- `turn_reverse_enabled: true`, for lab output only
- `turn_anchor_pivot_min_degrees: 90`
- `turn_anchor_pivot_max_degrees: 180`
- `turn_anchor_angle_step_degrees: 2.5`
- `turn_anchor_target_tolerance_m: 0.03`
- `turn_anchor_refine_tolerance_m: 0.01`
- `turn_anchor_reverse_max_spacing_factor: 4.0`
- `turn_fallback_planners: ["forward_u_turn", "omega", "three_point_y"]`
- `omega_radius_m: 0.6`
- `omega_overshoot_m: 0.4`
- `y_turn_forward_m: 0.5`
- `y_turn_pivot_max_degrees: 150`
- `y_turn_pivot_step_degrees: 5`
- `in_place_pivot_enabled: false`, because the current mower-facing FTC path follower cannot execute duplicate-position yaw-only pivots
- `turn_lattice_step_m: 0.10`
- `turn_lattice_yaw_step_degrees: 15.0`
- `turn_lattice_pivot_step_degrees: 12.0`
- `turn_lattice_min_radius_m: 0.28`
- `turn_lattice_terminal_max_distance_m: 1.8`
- `turn_lattice_search_margin_m: 1.2`
- `turn_forward_extent_spacing_factor: 1.0`, for forward U-turn fallback only
- `safety_margin_m: 0.05`
- `v2_drivable_boundary_clearance_m: 0.40`, used by the V2 geometry classifier's main mowable/drivable area and task-fill stripe region. The footprint-side V2 outline has its own right-footprint clearance, while the larger footprint-disk erosion remains a separate all-yaw-safe diagnostic layer.
- V2 task-evidence thresholds such as `v2_task_dead_end_min_depth_m: 1.8`, `v2_task_dead_end_min_area_m2: 1.0`, `v2_task_region_merge_overlap_ratio: 0.45`, `v2_task_portal_cut_buffer_m: 0.03`, and `v2_task_pocket_min_visible_area_m2: 0.25`, used only to grow diagnostic seeds into service regions and distinguish shallow notches from single-entry dead ends.
- M2/M2.1/M2.2/M2.3/M2.4/M2.6 prototype knobs such as `v2_task_path_spacing_factor: 0.90`, `v2_task_path_min_pass_length_m: 0.35`, `v2_task_path_body_min_pass_length_m: 2.00`, `v2_task_path_stripe_boundary_clearance_factor: 1.0`, `v2_task_path_body_pocket_outline_band_m: 0.50`, `v2_task_path_body_pocket_min_area_m2: 0.50`, `v2_task_path_sample_step_m: 0.10`, `v2_task_path_reverse_out_enabled: true`, `v2_task_path_reverse_cutting_enabled: false`, `v2_task_path_terminal_angle_tolerance_deg: 35`, `v2_task_path_side_lane_min_length_factor: 0.55`, `v2_task_path_notch_max_strokes: 3`, `v2_task_path_service_axis_ray_pad_m: 0.50`, `v2_task_path_axis_candidate_scoring_enabled: true`, `v2_task_path_axis_candidate_sweep_degrees: 25`, `v2_task_path_axis_candidate_step_degrees: 12.5`, `v2_task_path_axis_candidate_max_count: 24`, `v2_task_annotation_cut_buffer_m: 0.03`, `v2_task_ownership_short_body_pass_m: 1.20`, `v2_task_path_dead_end_turnaround_enabled: true`, automatic portal-search knobs such as `v2_task_portal_search_enabled: true`, `v2_task_portal_search_m: 0.80`, `v2_task_portal_search_step_m: 0.10`, and annotation-QA thresholds such as `v2_annotation_qa_min_operator_overlap: 0.25`, `v2_annotation_qa_good_overlap: 0.65`, and `v2_annotation_qa_max_portal_distance_m: 0.80`, used only by the diagnostic local path/report layer.
- Fields2Cover `v2.0.0`
- built-in Fields2Cover headland, swath, and route-order primitives, with mower-specific footprint-side `base_link` outline generation and sampled wheel-anchored stripe-to-stripe turns
- comparison profile using Fields2Cover Dubins path planning with `min_turning_radius: 0.10`

## Wheel-Anchored Turns

The default lab turn model is `wheel_anchor`. For each adjacent swath pair, the planner chooses the wheel on the next-stripe side as the initial pivot wheel, pivots between `90 deg` and `180 deg`, reverses along that selected angle, and then pivots about the opposite wheel's target anchor so the final `base_link` pose and yaw exactly match the next swath start. Candidate turns are rejected before scoring if any physical or safety-margin footprint sample leaves the lawn or intersects a hole, if the reverse distance is invalid, if the outside wheel misses its target tolerance, or if later phases sweep farther outward than the initial pivot envelope.

The HTML report includes toggleable left-wheel track, right-wheel track, pivot-anchor, target-anchor, and swept-footprint layers. These layers are intentionally separate from the route color/style: the coverage route should stay readable, while the wheel overlays explain whether the maneuver geometry itself is plausible.

`simulation_preview.json` carries maneuver-aware metadata such as `maneuvers[]`, `maneuver_id`, `phase`, `direction`, `pivot_wheel`, wheel positions, `blade_enabled`, `target_wheel_error_m`, `min_clearance_m`, `reverse_distance_m`, and `pivot_angle_deg`. `planpath_compat.json` does not carry those fields because it remains a compatibility artifact for the current mower-facing `PlanPath` shape.

The earlier lattice planner is still available manually with `--turn-planner lattice` for comparison/debugging, but it is not the default fallback. The default fallback list is intentionally explicit and finite so unsafe or unclear connectors become visible gaps.

## Landed Planner Work

Recent Claude-agent work moved the lab past the original wheel-anchor-only planner. The current implementation includes:

- P0/P1: footprint-disk headlands and obstacle-aware cell decomposition in [tools/coverage_lab/lab_geometry.py](../tools/coverage_lab/lab_geometry.py).
- P10/P11: explicit within-lawn bridges plus critical-vertex BCD cuts so path pieces no longer silently teleport across a lawn.
- P3: turn fallback diversity after wheel-anchor failure: forward U-turn, omega/keyhole, and three-point-Y. The in-place pivot primitive remains disabled by default because the live FTC controller cannot preserve it.
- P12/P13: robust dominant-direction stripe angle selection and local tight-cell stripe-angle overrides.

The roadmap remains the source of truth for status, accepted tradeoffs, and baseline numbers.

## Guardrails

Do not add automatic SSH or mower-control behavior to the lab. The map copy step is manual by design.

Do not commit private maps or run outputs. `tools/coverage_lab/data/maps/**/*.json`, `tools/coverage_lab/data/maps/**/*.kml`, `tools/coverage_lab/data/maps/**/*.bag`, and `tools/coverage_lab/runs/**` are ignored.

The first mower migration target is compatibility with the existing planner service concept, not immediate execution. Treat V2's `planpath_compat.json` as a bridge artifact for future work: it keeps the old JSON shape, but it is lossy until the mower-side contract can carry direction, blade state, and maneuver metadata.

The HTML preview is an offline path preview, not a dynamic controller simulation. It animates the mower footprint along generated `base_link` poses so unsafe connectors, pivots, and boundary collisions can be inspected visually. The map and slider stay in the same preview panel, and the SVG can be zoomed from the toolbar or mouse wheel, panned by dragging, and rotated from the toolbar.

`lawn_preview.png` uses the same timeline for a different visual check: the renderer interpolates poses densely, paints the physical footprint and active cutter disk in mowed-grass color, paints rear wheel rectangles after the footprint so current wheel tracks remain visible, then clips the result to the union of lawn polygons minus holes. This is only a display artifact; `metrics.json` remains the source for coverage and safety numbers.

## Map Recording Relationship

The lab itself still does not change mower runtime behavior. Separately, [AREA_RECORDING_SWEEP.md](AREA_RECORDING_SWEEP.md) documents mower-side swept-footprint area recording. That runtime change saves mowing and obstacle polygons from the full costmap footprint swept along trusted poses instead of a single corner breadcrumb. Once old maps are cleared and rerecorded, new map inputs should better match the coverage lab's footprint-first planning assumptions.

## Runtime Semantics

The future mower-facing planner must preserve the current runtime's frame convention: exported path poses are executable `base_link` poses in the `map` frame.

On the current Mowrator setup, `base_link` is the rear-center point of the physical footprint. The default footprint is `[[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]]`, so the front center is `[0.82, 0.0]`. The cutter/tool center is separate at `[0.41, 0.0]`.

Current mower execution flow:

- `mower_logic` asks `slic3r_coverage_planner/plan_path` for `Path[]`.
- For each path segment, `mower_logic` sends the first `path.path.poses[]` pose to Move Base Flex as a `MoveBaseGoal` with `controller = "FTCPlanner"`.
- Once that first pose is reached, `mower_logic` enables the mower and sends the remaining poses as an MBF `ExePathGoal` with `controller = "FTCPlanner"`.
- Progress is read from `/move_base_flex/FTCPlanner/planner_get_progress` and mapped back into the current `Path` index.

`FTCPlanner` is a follow-the-carrot controller. It stores the path as `global_plan`, interpolates a moving control pose along adjacent path poses, transforms that control pose from `map` into `base_link`, then computes longitudinal, lateral, and yaw error. Those errors drive `cmd_vel.linear.x` and `cmd_vel.angular.z`. It does not solve a geometric curve fit, and it does not choose a footprint-safe turn shape.

For the direct Mowrator hardware bridge, the twist command is converted into differential drive commands with the configured wheel distance:

```text
right = linear.x + 0.5 * wheel_distance_m * angular.z
left  = linear.x - 0.5 * wheel_distance_m * angular.z
```

Implications for the lab:

- `planpath_compat.json` should keep representing `base_link`, not cutter center.
- Coverage metrics should be computed from the cutter/tool path, while safety metrics should be computed from the full footprint at the exported `base_link` poses.
- Inter-segment first-pose moves are not coverage stripes. Keep marking them as segment gaps in the preview.
- Same-position yaw-only poses are not reliable in the existing `FTCPlanner` follow state because duplicate-position points are skipped. The current primary profile may include pivot samples in the HTML/simulation metadata, but that is lab-only maneuver planning. Live mower support still needs an execution contract that preserves direction, blade state, and pivot semantics.
- Reverse turn poses are offline lab metadata only. The mower-side `PlanPath` service and `MowingBehavior` still lack explicit direction and blade-state semantics, so this lab work must not be treated as live reverse-mowing support.
- A future live implementation should use a maneuver-aware planner message or service with explicit per-segment direction, blade state, pivot mode, and wheel-anchor data. Do not overload `nav_msgs/Path` with hidden semantics; a centerline path cannot faithfully describe a pivot wheel, reverse phase, or blade-off turn.

## Known Gaps

For the fresh-planner direction that keeps this lab as a baseline instead of continuing to stack local fixes on it, see [COVERAGE_PLANNER_V2_DESIGN.md](COVERAGE_PLANNER_V2_DESIGN.md).

P0/P1/P3/P10/P11/P12/P13 have landed in the lab. The following are the remaining gaps that the prioritized roadmap continues to address:

- coverage of the band between the eroded headland centreline and the lawn boundary (P5: multi-headland and stripe overrun);
- remaining stripe splits inside narrow cells where the current wheel-anchor/forward-U/omega/Y-turn library still cannot find a safe transition; skip-stripe ordering is the missing piece;
- visual stripe direction continuity is preserved across cells (P1 carries this from P2) but stripe-end alignment, blade scheduling, and rotation-of-direction memory are still open (remainder of P2);
- mower-side execution semantics for reverse, pivot wheel, and blade state are still unencoded (P4: maneuver-aware planner contract).
- FTC-truthful simulation is still absent: the preview follows exact planned poses rather than simulating controller tracking error (P9).

Do not hide these by weakening validation. The next planning work should make small, testable changes and use the HTML preview plus tracked sample maps to compare behavior.

The prioritized work list and the per-priority implementation plans live in [COVERAGE_PLANNER_ROADMAP.md](COVERAGE_PLANNER_ROADMAP.md). Start there before touching the lab. The roadmap is the source of truth for what to build next, in what order, and what acceptance criteria each step must meet. Update its status table when work lands.
