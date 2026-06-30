# Laptop-Only Fields2Cover Planner Lab

This lab evaluates Fields2Cover against OpenMower map files without changing the mower runtime. It builds a separate Docker image, reads a manually copied `map.json`, and writes visual and JSON artifacts under ignored local run directories.

## Quick Start

Build and smoke-test the isolated image:

```bash
tools/coverage_lab/bin/coverage_lab build
tools/coverage_lab/bin/coverage_lab smoke
```

Copy the mower map manually:

```bash
scp <mower-user>@<mower-host>:~/.ros/map.json tools/coverage_lab/data/maps/current/map.json
```

Or convert Google Earth KML polygons. Name each Google Earth polygon with a `mow:`, `obstacle:`, or `nav:` prefix:

```bash
tools/coverage_lab/bin/coverage_lab convert-kml --kml ~/Downloads/front_yard.kml
tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/google_earth/front_yard.json
```

Use `--plan` to convert and immediately generate the coverage report:

```bash
tools/coverage_lab/bin/coverage_lab convert-kml --kml ~/Downloads/front_yard.kml --plan
```

For larger or awkward KML maps where the diagnostic F2C comparison path is slow, run only the primary Mowrator profile:

```bash
tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/google_earth/front_yard.json --skip-comparisons
```

For detailed Google Earth outlines where automatic swath-angle search is slow, use a fixed angle:

```bash
tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/google_earth/front_yard.json --skip-comparisons --swath-angle-mode fixed --swath-angle-degrees 0
```

If headland generation is the slow part, temporarily inspect fill stripes only:

```bash
tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/google_earth/front_yard.json --skip-comparisons --swath-angle-mode fixed --swath-angle-degrees 0 --outline-count 0
```

If the drawn outline has many tiny segments, simplify it during conversion:

```bash
tools/coverage_lab/bin/coverage_lab convert-kml --kml ~/Downloads/front_yard.kml --simplify-tolerance-m 0.4
```

Validate and plan:

```bash
tools/coverage_lab/bin/coverage_lab validate-map --map tools/coverage_lab/data/maps/current/map.json
tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/current/map.json
```

Generate the lab-only V2 geometry classification report. The main mowable/drivable region defaults to the configured V2 boundary clearance (`0.40 m` in the current lab config); the larger footprint-disk inset is shown separately as an all-yaw-safe maneuver diagnostic.
The report also includes task-evidence diagnostics: local-width samples, prototype skeleton branches, service-region-grown corridor/dead-end/notch/body task proposals, labeled mouth cuts, dead-end terminal caps, M2 local task-path prototypes, neck cuts, and preliminary zones for comparison. Dead-end/notch prototypes use M2.6 axis-candidate scoring: portal-normal, source-centerline, terminal, oriented-bounds, and nearby offset axes are scored by the local paths they produce, so small recorded-map wrinkles at the deepest point do not decide stripe direction by themselves. BODY prototypes also score a ridge-derived body-centerline axis against oriented-bounds alternatives so the main area can follow the lawn's centerline instead of the raw bounding box. BODY and corridor stripe centers are clipped to a boundary-inset stripe-safe region so the outline pass can own the border band, and BODY/corridor lanes use a longer minimum pass length so tiny main-area stripe fragments are suppressed. The default task view includes both a generated V2 outline cutting path and an infill-area outline showing the exact polygon used to generate or clip the visible fill strokes. The V2 outline path now exports a `base_link` trajectory whose right footprint side tracks the conditioned boundary as closely as the configured footprint clearance and sampled turn geometry allow; it carries footprint-safety diagnostics and exports through `planpath_compat.json` with `is_outline: true`. If those clean BODY stripes leave meaningful residual pockets beyond the outline-owned band, the report adds supplemental BODY pocket service strokes instead of letting the main stripe field creep back to the boundary. On simple single-body maps, weak body-core subtraction remnants can be demoted to hidden artifacts so a recorded rectangle does not become a false BODY + NOTCH split; notch evidence from pockets, annotations, skeleton branches, and complex zone structure remains eligible. The default task view hides noisy artifact proposals, seed evidence lines, rejected axis alternatives, and stripe-safe overlays; use `Show artifacts` or the diagnostics/all presets to inspect them. The map view supports zoom/pan, and each layer has a hover `i` explanation. These layers are for deciding future coverage-task rules, not for mower execution.

```bash
tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/data/maps/current/map.json
```

The V2 report can export operator task-boundary annotations. Use the **Annotations** panel, click **Start drawing**, click two mouth/cut points, then click the service side to keep. Draft annotations stay visible on the map, are saved in that browser for the current source map, and can be removed before export. Export the JSON, then rerun with:

```bash
tools/coverage_lab/bin/coverage_lab classify-v2 --map tools/coverage_lab/data/maps/current/map.json --task-annotations v2_task_annotations.json
```

`plan`, `batch`, and `render` automatically open the generated HTML report on the laptop after a successful run. For script or headless use, disable that with:

```bash
COVERAGE_LAB_NO_OPEN=1 tools/coverage_lab/bin/coverage_lab plan --map tools/coverage_lab/data/maps/current/map.json
```

Run the tracked smoke examples:

```bash
tools/coverage_lab/bin/coverage_lab batch --maps tools/coverage_lab/examples
```

## Current Handoff Notes

The lab is caught up as an offline evaluator, not as a mower-ready planner. A future agent should start by reading this file, `tools/coverage_lab/AGENTS.md`, `docs/COVERAGE_PLANNER_LAB.md`, and `docs/COVERAGE_PLANNER_ROADMAP.md`. The roadmap carries the prioritized work list and is where status updates land when a priority is implemented.

The current primary output is `mowrator_zero_turn`. It uses Fields2Cover for swath generation/order, but the lab now owns the mower-specific geometry around it: footprint-disk headlands, obstacle-aware Boustrophedon cells, dominant-direction stripe angle selection, tight-cell stripe-angle overrides, explicit transit/fill bridges, and footprint-checked turn maneuvers. The comparison output `f2c_tiny_radius` keeps Fields2Cover's built-in path planner for contrast.

The important mower convention is that `planpath_compat.json` exports executable `base_link` poses in `map`, not cutter-center poses. On the current Mowrator model, `base_link` is the rear-center of the `0.82 m x 0.68 m` footprint. The cutter center is modeled separately at `[0.41, 0.0]`.

For V2 outline paths, the default `footprint_side_fit` mode is `base_link` first: exterior rings are oriented so the boundary is on the mower's right, straight sections offset the rear-center pose by the right side of the footprint, and corners are solved from the rotated footprint's support against the adjacent boundary half-planes before full-footprint safety sampling.

The actual mower currently uses `mower_logic` plus MBF `FTCPlanner`: it moves to the first pose of a segment with `MoveBaseGoal`, then follows the rest with `ExePathGoal`. `FTCPlanner` is a follow-the-carrot controller, not a true geometric turn planner. It does not guarantee that a requested curve or pivot will be followed exactly by the full mower footprint. Same-position yaw-only poses are especially suspect because the controller skips duplicate-position points during normal following.

Known current planner problems are expected to show up in the HTML preview and metrics: missing coverage along the conservative eroded boundary band, remaining stripe splits where no safe turn/bridge exists, long but explicit headland-walk bridges, and visually imperfect stripe ends. Do not relax the safety checks to hide these. Use the preview, `lawn_preview.png`, and small tracked examples to make the next planner change measurable.

## Outputs

Each `plan` run writes a timestamped directory under `tools/coverage_lab/runs/` unless `--output` is supplied.

- `plan.svg`: boundary, obstacles, headland rings, generated swaths, and final path.
- `plan.html`: a report embedding the SVG, metrics, and an interactive slicer-style mower footprint preview with pan/zoom/rotate controls.
- `lawn_preview.png`: a raster grass-finish preview painted from the execution timeline. Overgrown lawn starts dark green, the mower footprint and active cutting disk repaint it mowed green, wheel tracks are drawn after the footprint so current tracks remain visible, and holes/obstacles stay gray.
- `v2_geometry_report.html`: when using `classify-v2`, a lab-only geometry report showing raw/conditioned boundaries, the configured mowable/drivable inset, local-width heatmap, labeled skeleton/task evidence, operator annotations, service-ownership adjustments, labeled portal candidates, dead-end terminal caps, M2 local task paths, M2.3 annotation-QA overlays comparing automatic task evidence to operator annotations, M2.4 body-side portal candidate search metadata, candidate neck cuts, preliminary zones, boundary band, and unreachable/noise features. This is not a live route output.
- `v2_annotation_qa.json`: when `classify-v2` is run with `--task-annotations`, diagnostic operator-vs-automatic task evidence matches, misses, type mismatches, portal misalignments, partial matches, and unmatched automatic proposals.
- `v2_task_paths.json`: when using `classify-v2`, diagnostic local task-path prototypes for each visible task proposal. It includes per-task service axes, stripe-safe regions for BODY/corridor stripe centers, supplemental BODY pocket service metadata, exit mode, turnaround feasibility, per-segment direction, cutting metadata, lane anchors, derived `base_link` poses, local coverage estimates, reverse distance, and unsafe footprint sample counts.
- `planpath_compat.json`: `PlanPath`-like export with `paths[]`, `is_outline`, and `poses[{x,y,yaw}]` in the existing `map` frame. For `classify-v2`, this is a lossy V2 bridge that defaults to forward, blade-on task-path segments only; reverse, rotate, and blade-off V2 phases stay in `v2_task_paths.json`.
- `v2_planpath_compat_summary.json`: when using `classify-v2`, summary of which V2 task-path segments were exported or skipped by the lossy compatibility bridge.
- `planpath_compat_dry_run_fill3.json`: optional reduced static test export for the current rectangle run. It skips the outline and keeps the first three fill paths so the live mower can dry-run stripe-to-stripe navigation without committing to the full V2 route.
- `metrics.json`: approximate coverage, path length, swath length stats, headland/cell/transit summaries, connector/fill-bridge counts, turn-planner counts, obstacle intersections, and footprint safety samples. When `--area-index` is used, these metrics are scoped to that selected lawn instead of the full multi-lawn map.
- `source_map_snapshot.json`: copied input map for reproducibility. This lives in ignored `runs/` output.
- `planning_debug.json`: extra geometry for inspection and re-rendering.
- `simulation_preview.json`: flattened `base_link` execution timeline, mower footprint geometry, per-pose safety status, wheel-anchor maneuver metadata, and segment-gap metadata used by `plan.html`.
- `config_snapshot.json`: effective lab config used to generate the run.
- `profiles/f2c_tiny_radius/`: comparison output using Fields2Cover's built-in path planner with a small turn radius.

## Important Limits

This is not a mower executor. It does not publish ROS topics, call mower services, alter launch files, or send commands to hardware.

The primary profile uses a footprint-disk headland strategy (`headland_strategy: footprint_disk`) and obstacle-aware cell decomposition (`cell_decomposition: true`) before handing each cell to Fields2Cover for swaths. It then converts tool-center swaths into Mowrator `base_link` poses, optimizes cell visit order, and post-processes chunks so within-lawn path pieces are connected by direct turns or explicit headland-walk transits when safe.

The default turn planner is still `wheel_anchor`. A wheel-anchor turn pivots about the wheel on the next-stripe side, reverses at the selected pivot angle until the opposite wheel reaches its target anchor on the next stripe, then pivots/straightens into the exact next swath start pose. Every sample is checked with both physical and safety-margin footprints. If no safe wheel-anchor candidate exists, the lab tries configured safe fallbacks: `forward_u_turn`, `omega`, and `three_point_y`. If those fail, it leaves a visible gap instead of hiding an unsafe connector. The comparison profile keeps Fields2Cover's built-in path planner so runs can show what the mower-specific wrapper changed.

The default config mirrors the current Mowrator assumptions: `tool_width: 0.4`, a `0.82 m x 0.68 m` footprint, cutter center offset `[0.41, 0.0]`, one outline/headland pass, V2 outline mode `footprint_side_fit`, `headland_strategy: footprint_disk`, `cell_decomposition: true`, dominant-direction stripe angle selection, tight-cell angle overrides, `wheel_track_m: 0.58`, `wheel_contact_x_m: 0.0`, `turn_planner: wheel_anchor`, `turn_cutting_mode: off`, lab-only reverse turns enabled, wheel-anchor pivot search from `90 deg` to `180 deg`, and map-frame coordinates. The forward U-turn fallback still uses `turn_forward_extent_spacing_factor: 1.0`; omega/Y-turn behavior is controlled by `omega_*` and `y_turn_*` settings.

The preview animates the executable `base_link` path, not a physics/controller simulation. The footprint box is dimensionally accurate for the configured mower footprint, with `base_link` at rear-center, the cutter marker `0.41 m` forward, and the front marker `0.82 m` forward for the default Mowrator geometry. Wheel-track, pivot-anchor, target-anchor, and swept-footprint layers matter because a plain centerline can look clean even when the physical wheel geometry cannot execute the maneuver. The slider and map are kept in the same preview panel, and the SVG can be zoomed with the toolbar or mouse wheel, panned by dragging, and rotated from the toolbar. The generated `lawn_preview.png` is a static raster sanity check of what the grass would look like after the exact same pose timeline; it is clipped to lawn polygons, excludes holes, and uses Pillow inside the lab Docker image.

`planpath_compat.json` intentionally remains compatible with the existing mower-facing shape: `paths[]`, `is_outline`, and map-frame `poses[{x,y,yaw}]`. Wheel-anchor phases, direction, blade state, pivot wheel, wheel tracks, and target-anchor error live in `simulation_preview.json` and metrics only. Live mower support should use a future maneuver-aware contract instead of overloading the current `PlanPath` response, because `nav_msgs/Path` alone cannot preserve reverse, blade-off, or pivot-wheel semantics.

For controlled dry-run validation, `tools/coverage_lab/v2_static_planpath_server.py` can temporarily serve a V2 `planpath_compat.json` through the existing `slic3r_coverage_planner/plan_path` service. This is a service shim, not a planner. Start OpenMower with the normal planner disabled, keep `OM_ENABLE_MOWER=False`, source the ROS workspace, and run:

```bash
tools/coverage_lab/bin/v2_static_planpath_server serve \
  --plan tools/coverage_lab/runs/57-whitburn-back-yard-rectangle-v2-compat/planpath_compat.json \
  --skip-outline \
  --max-fill-paths 3
```

The reduced rectangle test can also be written ahead of time:

```bash
tools/coverage_lab/bin/v2_static_planpath_server filter \
  --plan tools/coverage_lab/runs/57-whitburn-back-yard-rectangle-v2-compat/planpath_compat.json \
  --skip-outline \
  --max-fill-paths 3 \
  --output tools/coverage_lab/runs/57-whitburn-back-yard-rectangle-v2-compat/planpath_compat_dry_run_fill3.json
```

The static service only answers `PlanPath` requests with old-shape forward geometry. It cannot command reverse, blade-off phases, pivot-wheel maneuvers, or smarter turns. A dry run with this shim shows how the current mower runtime navigates between separate V2 stripe paths.

For live dry-run debugging, record a passive telemetry trace while the old WebUI runs the static plan:

```bash
tools/coverage_lab/bin/v2_live_trace_recorder \
  --plan tools/coverage_lab/runs/57-whitburn-back-yard-rectangle-v2-straight-heading-diagnostic/planpath_compat_straight_heading_diagnostic.json \
  --output tools/coverage_lab/runs/live-traces/straight-heading-001.jsonl
```

The recorder only subscribes to topics such as `/xbot_positioning/xb_pose`, `/hw/position/gps`, `/hw/position/gps/quality`, `/mower_logic/current_state`, `/hw/diff_drive/measured_twist`, `/hw/cmd_vel`, `/hw/imu/data_raw`, FTC debug/carrot-point topics, and MBF status topics. It does not publish motion commands or change mower state. After the run, compare the recorded path against the plan:

```bash
tools/coverage_lab/bin/v2_trace_compare \
  --plan tools/coverage_lab/runs/57-whitburn-back-yard-rectangle-v2-straight-heading-diagnostic/planpath_compat_straight_heading_diagnostic.json \
  --trace tools/coverage_lab/runs/live-traces/straight-heading-001.jsonl \
  --map tools/coverage_lab/data/maps/recorded/57-whitburn-cres-back-yard-rectangle/map.json \
  --output-dir tools/coverage_lab/runs/live-traces/straight-heading-001-compare
```

This writes `trace_compare.html` and `trace_compare.json` with the planned path, actual `base_link` track, worst cross-track errors, yaw-error metrics, active mower state filtering, GPS quality deltas, and map-boundary context. Use this before changing coverage geometry when the question is whether `FTCPlanner`, heading, GPS timing, or the route shape caused a live dry-run problem.

The earlier lattice planner is still available manually with `--turn-planner lattice` for comparison/debugging, but it is not in the default fallback list. The default fallback list stays explicit and finite so unsafe or unclear connectors become visible gaps.

Runtime swept-footprint map recording is separate from the lab, but it matters to future planner inputs: [docs/AREA_RECORDING_SWEEP.md](../../docs/AREA_RECORDING_SWEEP.md) documents the new mower-side recorder that saves mowing/obstacle polygons from the full footprint swept by trusted poses. That should make newly recorded maps better aligned with the lab's footprint-first assumptions once old corner-breadcrumb maps are rerecorded.

Google Earth KML conversion supports `.kml` files, not `.kmz`. Converted maps are written under `tools/coverage_lab/data/maps/google_earth/` by default and are ignored by git.
