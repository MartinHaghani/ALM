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
- `convert-kml`: convert Google Earth KML polygons into OpenMower-style JSON maps.
- `plan`: generate primary and comparison profile artifacts.
- `render`: regenerate SVG/HTML/preview artifacts for an existing run directory.
- `batch`: run planning over a directory of map JSON files and write an index.

The current primary profile is `mowrator_zero_turn`. Fields2Cover is responsible for headland generation, swath generation, and swath ordering. The lab then converts cutter-center swaths into Mowrator `base_link` poses and inserts footprint-checked wheel-anchored stripe-to-stripe maneuvers. The comparison profile is `f2c_tiny_radius`, which runs Fields2Cover's built-in path planner with a small turn radius for side-by-side evaluation.

Private real maps and generated reports live under ignored paths such as `tools/coverage_lab/data/maps/` and `tools/coverage_lab/runs/`. A future agent may inspect local ignored artifacts to understand the current debugging case, but must not commit them unless the user explicitly requests a scrubbed sample.

## Inputs And Outputs

The input is the current `mower_map_service` JSON shape: `areas[]` with `properties.type` values such as `mow`, `nav`, and `obstacle`, plus optional `docking_stations[]`.

The planner reads active `mow` areas as lawns and active `obstacle` areas as holes when the obstacle centroid lies inside that lawn. Docking stations are preserved in the source snapshot but are not used for planning.

Each run writes ignored artifacts under `tools/coverage_lab/runs/`:

- `plan.svg`: visual overlay of lawn boundary, obstacles, generated swaths, headlands, and path.
- `plan.html`: evaluation report with an interactive slicer-style mower footprint preview, including pan/zoom/rotate controls.
- `planpath_compat.json`: `PlanPath`-like JSON with map-frame `paths[]`.
- `metrics.json`: approximate coverage and safety metrics, scoped to the selected lawn when `--area-index` is used.
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
- `outline_count: 1`
- `outline_clearance_m: auto`, which derives the first outline inset from the mower footprint plus configured safety margin
- `outline_offset: 0.0`
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
- `turn_fallback_planners: ["forward_u_turn"]`
- `turn_lattice_step_m: 0.10`
- `turn_lattice_yaw_step_degrees: 15.0`
- `turn_lattice_pivot_step_degrees: 12.0`
- `turn_lattice_min_radius_m: 0.28`
- `turn_lattice_terminal_max_distance_m: 1.8`
- `turn_lattice_search_margin_m: 1.2`
- `turn_forward_extent_spacing_factor: 1.0`, for forward U-turn fallback only
- `safety_margin_m: 0.05`
- Fields2Cover `v2.0.0`
- built-in Fields2Cover headland, swath, and route-order primitives, with mower-specific footprint-safe outline centerline selection and sampled wheel-anchored stripe-to-stripe turns
- comparison profile using Fields2Cover Dubins path planning with `min_turning_radius: 0.10`

## Wheel-Anchored Turns

The default lab turn model is `wheel_anchor`. For each adjacent swath pair, the planner chooses the wheel on the next-stripe side as the initial pivot wheel, pivots between `90 deg` and `180 deg`, reverses along that selected angle, and then pivots about the opposite wheel's target anchor so the final `base_link` pose and yaw exactly match the next swath start. Candidate turns are rejected before scoring if any physical or safety-margin footprint sample leaves the lawn or intersects a hole, if the reverse distance is invalid, if the outside wheel misses its target tolerance, or if later phases sweep farther outward than the initial pivot envelope.

The HTML report includes toggleable left-wheel track, right-wheel track, pivot-anchor, target-anchor, and swept-footprint layers. These layers are intentionally separate from the route color/style: the coverage route should stay readable, while the wheel overlays explain whether the maneuver geometry itself is plausible.

`simulation_preview.json` carries maneuver-aware metadata such as `maneuvers[]`, `maneuver_id`, `phase`, `direction`, `pivot_wheel`, wheel positions, `blade_enabled`, `target_wheel_error_m`, `min_clearance_m`, `reverse_distance_m`, and `pivot_angle_deg`. `planpath_compat.json` does not carry those fields because it remains a compatibility artifact for the current mower-facing `PlanPath` shape.

The earlier lattice planner is still available manually with `--turn-planner lattice` for comparison/debugging, but it is not the default fallback. The default fallback list is intentionally narrow so unsafe or unclear connectors become visible gaps.

## Guardrails

Do not add automatic SSH or mower-control behavior to the lab. The map copy step is manual by design.

Do not commit private maps or run outputs. `tools/coverage_lab/data/maps/**/*.json`, `tools/coverage_lab/data/maps/**/*.kml`, `tools/coverage_lab/data/maps/**/*.bag`, and `tools/coverage_lab/runs/**` are ignored.

The first mower migration target is compatibility with the existing planner service concept, not immediate execution. Treat `planpath_compat.json` as a bridge artifact for future work.

The HTML preview is an offline path preview, not a dynamic controller simulation. It animates the mower footprint along generated `base_link` poses so unsafe connectors, pivots, and boundary collisions can be inspected visually. The map and slider stay in the same preview panel, and the SVG can be zoomed from the toolbar or mouse wheel, panned by dragging, and rotated from the toolbar.

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

The lab currently exposes several unresolved planning problems on complex real maps:

- unsafe footprint samples near boundaries and obstacles;
- stripes or turns that can leave the mow area;
- obstacle interactions that need stricter clipping and validation;
- turn candidates that are geometrically impossible for the configured wheel track and stripe spacing;
- uncertainty about how best to encode pivots, reverse, and blade state for the existing FTC controller.

Do not hide these by weakening validation. The next planning work should make small, testable changes and use the HTML preview plus tracked sample maps to compare behavior.

Likely next implementation areas:

- stricter swath clipping against mow polygons and holes;
- controller-aware segment boundaries for headlands, swaths, and turns;
- an execution preview that approximates FTC follow-the-carrot behavior, separate from the current exact-pose and wheel-track preview;
- tracked regression maps for narrow boundaries, obstacle holes, separated lawns, and Google Earth-like outlines;
- eventual mower-side service integration only after the offline output is accepted.
