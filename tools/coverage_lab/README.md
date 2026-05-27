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
- `metrics.json`: approximate coverage, path length, swath count, connector length, obstacle intersections, and footprint safety samples.
- `source_map_snapshot.json`: copied input map for reproducibility. This lives in ignored `runs/` output.
- `planning_debug.json`: extra geometry for inspection and re-rendering.

## Important Limits

This is not a mower executor. It does not publish ROS topics, call mower services, alter launch files, or send commands to hardware.

The first version intentionally uses Fields2Cover as directly as possible. The metrics are approximate and are meant to reveal where mower-specific logic is needed, especially body footprint safety, connector routing, uncovered slivers, and headland behavior.

The default config mirrors the current Mowrator assumptions: `tool_width: 0.4`, a `0.82 m x 0.68 m` footprint, three outline/headland passes, and map-frame coordinates.
