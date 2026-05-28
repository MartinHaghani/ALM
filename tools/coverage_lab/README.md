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

The current primary output is `mowrator_zero_turn`. It uses Fields2Cover for headlands, swath generation, and swath ordering, then converts cutter-center swaths into executable Mowrator `base_link` poses and adds footprint-checked wheel-anchored stripe turns between adjacent swaths. The comparison output `f2c_tiny_radius` keeps Fields2Cover's built-in path planner for contrast.

The important mower convention is that `planpath_compat.json` exports executable `base_link` poses in `map`, not cutter-center poses. On the current Mowrator model, `base_link` is the rear-center of the `0.82 m x 0.68 m` footprint. The cutter center is modeled separately at `[0.41, 0.0]`.

The actual mower currently uses `mower_logic` plus MBF `FTCPlanner`: it moves to the first pose of a segment with `MoveBaseGoal`, then follows the rest with `ExePathGoal`. `FTCPlanner` is a follow-the-carrot controller, not a true geometric turn planner. It does not guarantee that a requested curve or pivot will be followed exactly by the full mower footprint. Same-position yaw-only poses are especially suspect because the controller skips duplicate-position points during normal following.

Known current planner problems are expected to show up in the HTML preview: unsafe footprint samples, obstacle crossings, off-lawn stripes, and turn sections on complex Google Earth maps. Do not relax the safety checks to hide these. Use the preview and small tracked examples to make the next planner change measurable.

## Outputs

Each `plan` run writes a timestamped directory under `tools/coverage_lab/runs/` unless `--output` is supplied.

- `plan.svg`: boundary, obstacles, headland rings, generated swaths, and final path.
- `plan.html`: a report embedding the SVG, metrics, and an interactive slicer-style mower footprint preview with pan/zoom/rotate controls.
- `planpath_compat.json`: `PlanPath`-like export with `paths[]`, `is_outline`, and `poses[{x,y,yaw}]` in the existing `map` frame.
- `metrics.json`: approximate coverage, path length, swath count, connector length, obstacle intersections, and footprint safety samples. When `--area-index` is used, these metrics are scoped to that selected lawn instead of the full multi-lawn map.
- `source_map_snapshot.json`: copied input map for reproducibility. This lives in ignored `runs/` output.
- `planning_debug.json`: extra geometry for inspection and re-rendering.
- `simulation_preview.json`: flattened `base_link` execution timeline, mower footprint geometry, per-pose safety status, wheel-anchor maneuver metadata, and segment-gap metadata used by `plan.html`.
- `config_snapshot.json`: effective lab config used to generate the run.
- `profiles/f2c_tiny_radius/`: comparison output using Fields2Cover's built-in path planner with a small turn radius.

## Important Limits

This is not a mower executor. It does not publish ROS topics, call mower services, alter launch files, or send commands to hardware.

The primary profile uses Fields2Cover for headlands/swaths, then converts tool-center swaths into Mowrator `base_link` poses with sampled wheel-anchored stripe-to-stripe maneuvers. A wheel-anchor turn pivots about the wheel on the next-stripe side, reverses at the selected pivot angle until the opposite wheel reaches its target anchor on the next stripe, then pivots/straightens into the exact next swath start pose. Every sample is checked with both physical and safety-margin footprints. If no safe wheel-anchor candidate exists, the lab tries only configured safe fallbacks such as `forward_u_turn`; if those fail, it splits the fill path so the HTML report shows the unplanned segment gap instead of hiding an unsafe connector. The comparison profile keeps Fields2Cover's built-in path planner so runs can show what the mower-specific wrapper changed.

The default config mirrors the current Mowrator assumptions: `tool_width: 0.4`, a `0.82 m x 0.68 m` footprint, cutter center offset `[0.41, 0.0]`, one outline/headland pass, automatic footprint-derived outline clearance, `wheel_track_m: 0.58`, `wheel_contact_x_m: 0.0`, `turn_planner: wheel_anchor`, `turn_cutting_mode: off`, lab-only reverse turns enabled, wheel-anchor pivot search from `90 deg` to `180 deg`, and map-frame coordinates. The forward U-turn fallback still uses `turn_forward_extent_spacing_factor: 1.0`.

The preview animates the executable `base_link` path, not a physics/controller simulation. The footprint box is dimensionally accurate for the configured mower footprint, with `base_link` at rear-center, the cutter marker `0.41 m` forward, and the front marker `0.82 m` forward for the default Mowrator geometry. Wheel-track, pivot-anchor, target-anchor, and swept-footprint layers matter because a plain centerline can look clean even when the physical wheel geometry cannot execute the maneuver. The slider and map are kept in the same preview panel, and the SVG can be zoomed with the toolbar or mouse wheel, panned by dragging, and rotated from the toolbar.

`planpath_compat.json` intentionally remains compatible with the existing mower-facing shape: `paths[]`, `is_outline`, and map-frame `poses[{x,y,yaw}]`. Wheel-anchor phases, direction, blade state, pivot wheel, wheel tracks, and target-anchor error live in `simulation_preview.json` and metrics only. Live mower support should use a future maneuver-aware contract instead of overloading the current `PlanPath` response, because `nav_msgs/Path` alone cannot preserve reverse, blade-off, or pivot-wheel semantics.

The earlier lattice planner is still available manually with `--turn-planner lattice` for comparison/debugging, but the default fallback list stays narrow so unsafe or unclear connectors become visible gaps.

Google Earth KML conversion supports `.kml` files, not `.kmz`. Converted maps are written under `tools/coverage_lab/data/maps/google_earth/` by default and are ignored by git.
