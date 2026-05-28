# Coverage planner lab guide

Purpose: guardrails for the laptop-only Fields2Cover evaluation lab.

Before starting any planner work, read [docs/COVERAGE_PLANNER_ROADMAP.md](../../docs/COVERAGE_PLANNER_ROADMAP.md). It carries the prioritized work list (P0–P9), per-priority status, per-priority implementation plan docs, and the natural-lawn edge case table. Update the roadmap status row when work lands.

- This directory is for offline route-planning experiments only. Do not wire it into `open_mower.launch`, `mower_logic`, runtime Docker images, or mower startup scripts.
- Do not add automatic SSH or mower-control behavior here. Current mower maps are copied manually into `data/maps/`.
- Treat files under `data/maps/` and `runs/` as private local artifacts. They are ignored on purpose and should not be committed unless a user explicitly asks to publish a scrubbed sample.
- Keep the first migration target compatible with the existing planner concept: map-frame points and `PlanPath`-like `paths[]` output. Do not change the mower-side service until the offline behavior is accepted.
- Prefer adding small tracked example maps under `examples/` for tests and documentation.
- If planner defaults drift from Mowrator params, update `configs/default.yaml`, `README.md`, and `docs/COVERAGE_PLANNER_LAB.md` together.

## Current implementation state

- `coverage_lab.py` is the main laptop-only CLI. It supports `validate-map`, `plan`, `render`, `batch`, and `convert-kml`.
- `lab_geometry.py` is the shapely-backed geometry helper used by P0/P1: footprint-disk Minkowski erosion of the lawn, Boustrophedon Cellular Decomposition aligned to the stripe angle, and ring-walk transit stitching. Doctest-tested on host and in the lab image.
- `bin/coverage_lab` is the Docker-aware wrapper. Prefer using the wrapper for planning because Fields2Cover is installed inside the lab image.
- `configs/default.yaml` is the effective Mowrator planning model for the lab: `tool_width: 0.4`, `tool_center_offset: [0.41, 0.0]`, footprint `[[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]]`, `headland_strategy: footprint_disk` (P0), `cell_decomposition: true` (P1), one outline/headland pass, automatic footprint-derived outline clearance, `wheel_track_m: 0.58`, `wheel_contact_x_m: 0.0`, wheel-anchored turn planner with lab-only reverse metadata, zero-turn primary profile, and `f2c_tiny_radius` comparison profile.
- `metrics.json` carries new top-level sections produced by P0/P1: `headland.{strategy, eroded_polygon_area_m2, mainland_polygon_area_m2, unsafe_footprint_samples}`, `cells.{count, areas_m2}`, and `transit.{count, total_length_m}`. The post-P0/P1 baseline numbers for each tracked example are recorded in [docs/COVERAGE_PLANNER_ROADMAP.md](../../docs/COVERAGE_PLANNER_ROADMAP.md).
- The primary profile is `mowrator_zero_turn`: Fields2Cover produces headlands, swaths, and swath order; lab code converts tool-center poses to `base_link` poses and inserts footprint-checked wheel-anchor maneuvers that land exactly on the next swath start when safe. If no safe turn or configured safe fallback exists, the fill path is split so the preview shows the unplanned segment gap.
- The comparison profile is `f2c_tiny_radius`: the same F2C swaths/order are passed through Fields2Cover's built-in path planner with `min_turning_radius: 0.10`.
- `plan.html` embeds `simulation_preview.json` and works from a local file without a server. It has a slicer-style timeline slider, dimensionally accurate mower footprint, wheel-track and turn-anchor overlays, unsafe-pose coloring, previous/next unsafe buttons, layer toggles, and pan/zoom/rotate view controls.

## Mower runtime assumptions to preserve

- Treat every exported `path.path.poses[]` pose as an executable `base_link` pose in the `map` frame.
- On the current Mowrator setup, `base_link` is the rear-center point of the mower footprint. The cutter center is modeled separately at `[0.41, 0.0]`; do not silently reinterpret path poses as cutter-center or front-center poses.
- `mower_logic` currently moves to the first pose of each path segment using MBF `MoveBaseGoal`, then sends the remaining poses as an MBF `ExePathGoal` with `controller = "FTCPlanner"`.
- The inter-segment move to the first pose is not part of the coverage path. The preview marks segment gaps so they are visible instead of hiding them as normal mowing.
- `FTCPlanner` is a follow-the-carrot controller, not a geometric turn planner. It interpolates a moving control pose along the path, transforms it into `base_link`, and turns longitudinal, lateral, and yaw error into `cmd_vel`.
- Same-position yaw-only points are not a reliable way to request an in-path pivot from `FTCPlanner`: during normal following it skips duplicate-position points. Pivot samples are acceptable in the lab preview only; mower-side execution needs explicit maneuver semantics before live pivot or reverse support.
- Reverse, pivot-wheel, and blade-off turn phases are offline lab metadata. Do not enable real mower reverse execution without adding explicit direction, blade-state, and maneuver semantics to the mower-side runtime.

## Known planner gaps

- The lab is currently useful for finding problems, not for producing mower-ready plans. Complex real Google Earth maps have shown unsafe footprint samples, stripes/turns that cross obstacles or leave mow areas, and turn gaps where the configured wheel geometry cannot produce a safe maneuver.
- Fields2Cover should remain useful for headland and swath generation, but mower-specific logic likely needs to own swath clipping/validation, segment boundaries, stripe-to-stripe turns, and controller-aware execution.
- Do not "fix" unsafe output by relaxing safety checks. Make the report show the issue clearly, add a small tracked sample if possible, then adjust planning logic or evaluation deliberately.
