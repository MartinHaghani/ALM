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

## Outputs

Each `plan` run writes a timestamped directory under `tools/coverage_lab/runs/` unless `--output` is supplied.

- `plan.svg`: boundary, obstacles, headland rings, generated swaths, and final path.
- `plan.html`: a small report embedding the SVG and metrics.
- `planpath_compat.json`: `PlanPath`-like export with `paths[]`, `is_outline`, and `poses[{x,y,yaw}]` in the existing `map` frame.
- `metrics.json`: approximate coverage, path length, swath count, connector length, obstacle intersections, and footprint safety samples. When `--area-index` is used, these metrics are scoped to that selected lawn instead of the full multi-lawn map.
- `source_map_snapshot.json`: copied input map for reproducibility. This lives in ignored `runs/` output.
- `planning_debug.json`: extra geometry for inspection and re-rendering.
- `profiles/f2c_tiny_radius/`: comparison output using Fields2Cover's built-in path planner with a small turn radius.

## Important Limits

This is not a mower executor. It does not publish ROS topics, call mower services, alter launch files, or send commands to hardware.

The primary profile uses Fields2Cover for headlands/swaths, then converts tool-center swaths into Mowrator `base_link` poses with zero-turn connectors. The comparison profile keeps Fields2Cover's built-in path planner so runs can show what the mower-specific wrapper changed.

The default config mirrors the current Mowrator assumptions: `tool_width: 0.4`, a `0.82 m x 0.68 m` footprint, cutter center offset `[0.41, 0.0]`, three outline/headland passes, zero-turn pivots sampled every `10 deg`, and map-frame coordinates.

Google Earth KML conversion supports `.kml` files, not `.kmz`. Converted maps are written under `tools/coverage_lab/data/maps/google_earth/` by default and are ignored by git.
