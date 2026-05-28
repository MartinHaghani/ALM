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

## Inputs And Outputs

The input is the current `mower_map_service` JSON shape: `areas[]` with `properties.type` values such as `mow`, `nav`, and `obstacle`, plus optional `docking_stations[]`.

The planner reads active `mow` areas as lawns and active `obstacle` areas as holes when the obstacle centroid lies inside that lawn. Docking stations are preserved in the source snapshot but are not used for planning.

Each run writes ignored artifacts under `tools/coverage_lab/runs/`:

- `plan.svg`: visual overlay of lawn boundary, obstacles, generated swaths, headlands, and path.
- `plan.html`: evaluation report.
- `planpath_compat.json`: `PlanPath`-like JSON with map-frame `paths[]`.
- `metrics.json`: approximate coverage and safety metrics, scoped to the selected lawn when `--area-index` is used.
- `source_map_snapshot.json`: reproducibility copy of the input map.
- `planning_debug.json`: additional generated geometry for re-rendering and agent inspection.

Each primary run is the `mowrator_zero_turn` profile. Comparison output for Fields2Cover's built-in small-radius path planner is written under `profiles/f2c_tiny_radius/`. Batch runs also write `batch.html`, a small index that links to each generated `plan.html`.

## Defaults

The default config is `tools/coverage_lab/configs/default.yaml`.

It mirrors current Mowrator planning assumptions where possible:

- `tool_width: 0.4`
- `tool_center_offset: [0.41, 0.0]`
- `outline_count: 3`
- `outline_offset: 0.0`
- footprint: `[[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]]`
- `drive_model: zero_turn`
- `pivot_yaw_step_degrees: 10`
- `safety_margin_m: 0.05`
- Fields2Cover `v2.0.0`
- built-in Fields2Cover headland, swath, and route-order primitives
- comparison profile using Fields2Cover Dubins path planning with `min_turning_radius: 0.10`

## Guardrails

Do not add automatic SSH or mower-control behavior to the lab. The map copy step is manual by design.

Do not commit private maps or run outputs. `tools/coverage_lab/data/maps/**/*.json`, `tools/coverage_lab/data/maps/**/*.kml`, `tools/coverage_lab/data/maps/**/*.bag`, and `tools/coverage_lab/runs/**` are ignored.

The first mower migration target is compatibility with the existing planner service concept, not immediate execution. Treat `planpath_compat.json` as a bridge artifact for future work.
