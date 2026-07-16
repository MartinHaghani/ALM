# Coverage Planner P0 + P1 Implementation Plan

Status: completed historical plan. P0 and P1 landed in `5602b8e` with subsequent
corrections recorded in
[COVERAGE_PLANNER_ROADMAP.md](COVERAGE_PLANNER_ROADMAP.md). Unchecked boxes below
are preserved from the original implementation checklist and are not active work;
current outstanding planner work belongs in the linked GitHub issues.

Purpose: detailed plan for the two highest-priority coverage planner changes called out in [COVERAGE_PLANNER_ROADMAP.md](COVERAGE_PLANNER_ROADMAP.md): footprint-aware headland (P0) and obstacle-aware swath bridging (P1). Both are localized to the lab in `tools/coverage_lab/coverage_lab.py` and do not touch mower runtime code.

Read [COVERAGE_PLANNER_LAB.md](COVERAGE_PLANNER_LAB.md) first for current lab semantics, defaults, and FTC runtime constraints.

## Scope and non-goals

In scope:

- Replace the inset-polyline headland centerline with a footprint-aware headland computed from a Minkowski erosion of the lawn by the footprint disk, plus corner arc smoothing at the configured minimum turning radius.
- Decompose the mow polygon (lawn minus obstacles) into hole-free sub-regions before swath generation, run F2C per sub-region, and stitch sub-regions with explicit transit segments.
- Track new metrics in `metrics.json` so the regression suite (P8) can see the headland and bridging behavior.
- Update tracked example maps and add at least one new example for obstacle bridging.

Not in scope (deferred to later priorities):

- Stripe aesthetics, perimeter loop closure, blade scheduling (P2).
- New turn maneuver types beyond what wheel-anchor and forward U-turn already support (P3).
- Maneuver-aware live mower contract (P4).
- Increasing absolute coverage% beyond the safety baseline; P5 owns coverage closure.
- Multi-lawn transit or docking (P6).
- Slope or soft zones (P7).
- FTC-truthful preview (P9).

## Why P0 and P1 together

P0 and P1 share the polygon offset and decomposition machinery. The cleanest implementation introduces a single geometry helper module that does both: shrink-by-footprint produces the headland; cut-around-obstacle produces the sub-regions. Doing them together avoids duplicating polygon plumbing and avoids a P0-then-P1 in-between state where the headland is footprint-safe but the path is still fragmented around obstacles.

The two priorities are also each other's acceptance condition: P0 cannot be evaluated on the real Whitburn Cres map without P1, because the same map has both concave corners (drives P0 unsafe samples) and an interior obstacle (drives P1 unplanned turns). Both must work to call either done on real maps.

## P0 — Footprint-aware headland

### Problem recap

The lab generates the headland by inward polygon offset using `outline_clearance + N * tool_width`. `outline_clearance_m: auto` sets the inset to `max(footprint extent) + safety_margin_m`. This is safe on straight boundary segments. It is not safe at concave corners: the rear-center `base_link` follows the inset polyline, but the front-outboard corner of the footprint sweeps wider than the polyline by up to `L * (1 - cos((π - θ) / 2))` where `L` is the front-to-base distance and `θ` is the interior angle. For `L = 0.82 m` and `θ = 90°`, the extra outward sweep is about 24 cm, well past the configured 5 cm safety margin.

Symptom: on Whitburn Cres, 30% (simplified outline) and 49% (raw outline) of headland footprint samples are flagged unsafe, all in the `headland` section, with yaw values that cluster at the concave vertices.

### Options evaluated

| Option | What it does | Pros | Cons | Verdict |
|---|---|---|---|---|
| A. Per-vertex perpendicular offset with bulge compensation | Keep offset polyline, add extra inset per concave vertex | Easiest patch | Bandaid; concave runs with multiple close vertices still fail; does not handle curves; effectively re-derives Minkowski but badly | Reject |
| B. Heading-dependent Minkowski erosion | Shrink lawn by `rotated_footprint(local_heading)` along the tangent | Geometrically exact | Footprint orientation changes per vertex; requires custom polygon clipping; no off-the-shelf library does this directly | Defer to v2 |
| C. Constant-radius Minkowski erosion by footprint circumscribed disk | Shrink lawn by `disk(r)` where `r = sqrt((L/2)^2 + (W/2)^2)` from base_link to the farthest footprint corner | Trivially uses any polygon-offset library; guaranteed safe at every heading | Over-conservative by up to a few cm on straight runs; gives up some edge band | **Accept for v1** |
| D. C plus corner-arc smoothing at minimum turning radius | C, then resample the inset boundary with arcs of radius `>= min_turning_radius` at every corner | Drivable centerline; preserves provable safety | Slightly more code | **Accept for v1.1 inside this same priority** |
| E. Iterative offset adjustment | Use existing inset, sample swept footprint, push polyline inward locally | Minimal new geometry | Slow, fragile, may not converge | Reject |

### Recommended approach

Implement C as the geometric base, then layer D on top in the same priority. The result:

1. The Minkowski-eroded polygon `lawn ⊖ disk(r_footprint)` is the **safe drivable region** for the rear-center `base_link`. Any path that stays inside this region with any yaw is guaranteed to keep the physical footprint inside the lawn.
2. The boundary of that eroded polygon, walked with corner arcs of radius `>= min_turning_radius`, is the headland centerline. Stripes start and end at the headland.
3. The conservatism of using the circumscribed disk costs at most `r - L/2` of inward inset versus the heading-aware version. For the current footprint (`L=0.82, W=0.68`, `base_link` at rear-center, so far corners are at front), `r = sqrt(0.82^2 + 0.34^2) ≈ 0.886 m` vs `L/2 = 0.41 m`. The extra inward inset is about 48 cm in the worst direction (back-to-front), but only because `base_link` is at the rear-center, not the geometric centroid. Mitigation: shift the disk centre to the footprint centroid before erosion, so `r` becomes the centroid-to-farthest-corner distance, which is `sqrt(0.41^2 + 0.34^2) ≈ 0.533 m`. Then the inset is symmetric and minimal under the disk assumption.

### Library choice

The lab is Python (`coverage_lab.py`) inside a Docker image with Fields2Cover. F2C is C++ but exposes Python bindings; under the hood it uses GEOS. We can either:

- Use `shapely` (Python GEOS bindings) directly for the offset. Clean, well-documented, supports `buffer(-r, join_style='mitre' or 'round')`.
- Use F2C's own polygon operations through `f2c.Types.Cells.offset`. Less documented, but already in the docker image and consistent with the rest of the file.

Decision: use `shapely` for both P0 and P1 geometry. F2C's polygon ops are intended for swath generation, not arbitrary CSG. `shapely` is already a dependency transitively through GEOS, and the small additional pip install is acceptable inside the lab docker. Document the dependency in `tools/coverage_lab/docker/Dockerfile` and `tools/coverage_lab/AGENTS.md`.

If `shapely` is undesirable, fall back to F2C's offset; the algorithm is the same.

### Implementation steps for P0

Each step is a separate commit unless noted. Each step keeps the lab runnable.

**P0.1 — Add `shapely` and a geometry helper module.**

- Add `shapely>=2.0` to the lab docker image.
- Create `tools/coverage_lab/lab_geometry.py` with three functions:
  - `footprint_disk_radius(config) -> float`: returns the radius of the smallest disk centred at the footprint centroid that contains every footprint corner.
  - `erode_polygon(lawn_ring, holes, radius) -> list[Polygon]`: applies a negative buffer with `mitre` joins and returns the resulting polygons (a list because erosion can split a polygon).
  - `polygon_boundary_polyline(polygon) -> list[(x, y)]`: returns the closed outer ring, oriented CCW.
- Add unit tests as Python doctests inside `lab_geometry.py` for the rectangle, L-shape, and a polygon with one hole.

**P0.2 — Wire the eroded polygon as the headland source.**

- In `plan_one_lawn_profiles()` ([tools/coverage_lab/coverage_lab.py:2035](../tools/coverage_lab/coverage_lab.py)), replace the F2C `generateHeadlands` call with a call to `lab_geometry.erode_polygon(lawn.ring, lawn.holes, r_footprint + safety_margin_m)`. The result becomes the "mainland" passed to F2C for swath generation.
- The first headland centerline is the boundary of the eroded polygon, plus a `tool_width / 2` inset for the headland pass itself (so the headland pass cuts the band between the eroded boundary and the lawn boundary as a single sweep).
- Additional headland passes (when `outline_count > 1`) are successive `tool_width` insets from the first headland centerline, capped at the eroded polygon boundary.
- Keep the legacy F2C-headland path available behind a config flag `headland_strategy: f2c | footprint_disk` (default `footprint_disk`) so the comparison profile and old behavior remain accessible during evaluation.

**P0.3 — Add corner-arc smoothing.**

- In `lab_geometry.py`, add `smooth_corners(polyline, min_radius) -> list[(x, y)]` that walks the polyline, detects vertices where the interior angle is sharper than the minimum-turn-radius arc can fit, and replaces them with a short arc of radius `min_radius`.
- If a corner is sharper than the minimum radius can fit (the arc would exceed the available straight segment on either side), instead split the headland into two polyline pieces at that corner and emit a `kind: PIVOT_AT_CORNER` placeholder pose pair. v1 of P0 leaves the placeholder in the lab metadata; live mower execution of the corner pivot is gated on P4.
- The eroded polygon already protects footprint safety, so the smoothed centerline cannot become unsafe; document this in the function docstring.

**P0.4 — Update metrics and the HTML preview.**

- Add `metrics.json` fields:
  - `headland.strategy`: `footprint_disk` or `f2c`.
  - `headland.eroded_polygon_area_m2`.
  - `headland.unsafe_footprint_samples_after_erosion`: should be 0 on every map; if it is not, the geometry helper has a bug, and the run is failed loudly.
  - `headland.corner_pivots_count`: how many corner-pivot placeholders were emitted.
- Add an SVG layer toggle for the eroded polygon outline so the centerline relationship to the lawn is visible.

**P0.5 — Refresh example baselines.**

- Re-run `coverage_lab batch --maps tools/coverage_lab/examples` and the tracked Google Earth maps.
- Confirm `unsafe_footprint_samples` in `headland` section is 0 on every example map.
- If coverage drops more than 2 percentage points on any synthetic example, that is expected and is recovered in P5; document the new baselines in the next commit message.

### Acceptance for P0

- All four `tools/coverage_lab/examples/*.json` produce 0 unsafe footprint samples in the headland section.
- Whitburn Cres simplified and unsimplified runs produce 0 unsafe headland samples.
- `headland.unsafe_footprint_samples_after_erosion == 0` on every run.
- HTML preview renders the eroded polygon overlay and the smoothed centerline.
- `tools/coverage_lab/AGENTS.md` and `docs/COVERAGE_PLANNER_LAB.md` mention the new `headland_strategy` flag and the `lab_geometry` helper module.

## P1 — Obstacle-aware swath bridging

### Problem recap

F2C generates one swath per straight line crossing the field. When an obstacle is inside the lawn, the swath gets clipped into two collinear segments on opposite sides of the obstacle. The wheel-anchor planner is asked to bridge anchor poses many metres apart and fails because the maneuver geometry assumes adjacent stripes one `tool_width` apart. On `obstacle_map`, 21 of 33 stripe transitions fail and the resulting path is split into many disconnected fragments.

### Options evaluated

| Option | What it does | Pros | Cons | Verdict |
|---|---|---|---|---|
| A. Boustrophedon Cellular Decomposition (BCD) aligned to stripe direction | Sweep a line in the stripe direction; emit a new cell each time topology changes | Standard CPP-with-obstacles answer; cells align with stripes; each cell is simple | Sweep event handling has corner cases (vertical edges, multiple events at same coordinate) | **Accept for v1** |
| B. Trapezoidal decomposition | Sweep + every event creates a trapezoid | Simple to implement | Produces many small trapezoids; over-decomposes; confuses swath ordering | Reject |
| C. Convex partitioning (Bayazit / Hertel-Mehlhorn) | Greedy convex decomposition | Minimal cell count | Cells do not align with stripe direction; per-cell swath orientation drifts and breaks stripe-direction continuity (P2) | Reject |
| D. Multi-cell F2C `Cells` from manual cuts | Insert "cuts" tangent to obstacles and let F2C plan each Cell | Reuses F2C swath generation; minimal new planner code | Manual cut choice is essentially a custom BCD; just do BCD properly | Equivalent to A |
| E. Skip-stripe global ordering with cross-obstacle transit | Do not decompose; reorder swaths so adjacent route swaths are physically adjacent; add transit segments | Smallest code change | Does not solve the actual coverage gap (the obstacle band is still split by F2C); transit segments must still avoid the obstacle | Reject as primary; useful as a transit policy on top of A |

### Recommended approach

Implement A (BCD), aligned to the chosen stripe direction. Each sub-region (cell) is then a simple polygon with no holes, which is exactly what F2C handles well. F2C plans swaths per cell. The lab stitches cells together with explicit transit segments along the headland.

### Implementation steps for P1

**P1.1 — Add `decompose_mow_polygon` to `lab_geometry.py`.**

Signature: `decompose_mow_polygon(eroded_polygon, stripe_direction_rad) -> list[Polygon]`.

Algorithm (BCD):

1. Rotate the polygon so the stripe direction is the x-axis.
2. Compute event x-coordinates: every vertex of the outer ring and every vertex of each hole, sorted by x.
3. Walk events left to right. Maintain an "open cells" list keyed by their vertical extent at the current x.
4. At each event vertex, determine the event type: OPEN (a new vertical extent starts), CLOSE (an extent ends), SPLIT (one extent becomes two because a hole begins), or MERGE (two extents become one because a hole ends).
5. Emit the closed cells. Each cell is a polygon with edges that are either segments of the input polygon or vertical sweep-line segments.
6. Rotate cells back to the original orientation.
7. Return the cell list.

Open polygons (cells still active when the sweep ends) are flushed at the final x.

This is the canonical CPP-with-obstacles decomposition (Choset 2000). Document the reference in the function docstring.

**P1.2 — Use cells as the F2C planning unit.**

- In `plan_one_lawn_profiles()`, after computing the eroded polygon (P0.2), call `decompose_mow_polygon(eroded_polygon, swath_angle_rad)` to get cells.
- For each cell, build an F2C `Cells` object and call the existing swath generation and ordering helpers.
- Per-cell swath generation respects the global stripe angle: each cell gets the same `swath_angle_rad` so stripes are visually continuous across the lawn even though the cells are separate (P2's "global stripe angle" expectation is honored from P1 onward).

**P1.3 — Stitch cells with explicit transit segments.**

- After each cell's swath block is planned, the final pose of the cell's path is the start of a transit to the next cell.
- Transit segments are planned along the headland centerline, not across the obstacle. The headland centerline is already a closed loop around all obstacles (P0), so any two cells can be connected by walking along the headland in the shorter direction.
- Each transit is recorded as a path segment with the new `kind: TRANSIT` marker in `simulation_preview.json` and `metrics.json.connector_path.transit_runs`. The lab-only blade state on transit is OFF.
- Solve the cell visit order with a greedy TSP: start at the cell closest to the docking station (or the map origin when there is no dock), pick the nearest unvisited cell by transit cost at each step. v1.1 of P1 may swap in a proper TSP solver; v1 keeps it greedy.

**P1.4 — Update metrics and the HTML preview.**

- New `metrics.json` fields:
  - `cells.count`: number of cells produced.
  - `cells.area_m2[]`: area per cell.
  - `transit.count`, `transit.total_length_m`.
- Update SVG to render cell boundaries (toggle layer) and color transit segments distinctly from stripes and headlands.

**P1.5 — Add tracked example maps.**

- Add `tools/coverage_lab/examples/obstacle_off_center_map.json`: a rectangle with one obstacle near a corner.
- Add `tools/coverage_lab/examples/two_obstacles_map.json`: a rectangle with two obstacles forcing two BCD splits.
- Re-run the batch and confirm no warnings and a single connected base-link path on each.

### Acceptance for P1

- `obstacle_map` runs with `warnings == []` and one connected path (verified by `metrics.json.connector_path.unplanned_turn_gaps == 0`).
- Two new tracked example maps pass with the same conditions.
- Whitburn Cres `connector_path.connector_runs` is at most half of the pre-P1 baseline.
- Cell boundaries are rendered in the SVG.

## Combined acceptance and regression

After both P0 and P1 land:

- `coverage_lab batch --maps tools/coverage_lab/examples` returns `ok: true` for every map.
- The synthetic batch summary shows `unsafe_footprint_samples == 0` across all maps.
- Whitburn Cres simplified shows `unsafe_footprint_samples == 0` and a single connected path.
- `metrics.json.warnings` is empty on all tracked example maps.

Record the new metric values in `docs/COVERAGE_PLANNER_ROADMAP.md` as the **post-P0-P1 baseline**. P8 will turn this into a regression test, but the baseline must exist now so reviewers can compare future work against it.

## Commit plan

Each bullet is a separate commit. Each commit keeps the lab runnable; if a commit cannot be runnable on its own, fold it into the next one.

1. Add `shapely` to the lab docker image; add empty `lab_geometry.py` skeleton with module docstring and TODO markers. Build and smoke-test the image.
2. Implement `footprint_disk_radius`, `erode_polygon`, `polygon_boundary_polyline` with doctests. No call sites yet.
3. Wire `erode_polygon` into `plan_one_lawn_profiles` behind `headland_strategy: footprint_disk`. Add metrics fields. Update HTML.
4. Implement `smooth_corners` and the corner-pivot placeholder. Update metrics.
5. Refresh example baselines and update `docs/COVERAGE_PLANNER_ROADMAP.md` status row for P0 to `landed`. Include before/after numbers in the commit message.
6. Implement `decompose_mow_polygon` (BCD) with doctests. No call sites yet.
7. Wire BCD into `plan_one_lawn_profiles`. Implement per-cell swath generation.
8. Implement headland-following cell transit. Add transit metrics. Update HTML.
9. Add two new obstacle example maps. Refresh baselines. Update `docs/COVERAGE_PLANNER_ROADMAP.md` status row for P1 to `landed`. Include before/after numbers in the commit message.

## Risks and how to debug them

- **Erosion produces empty or near-empty polygons on narrow lawns.** Expected for narrow necks (edge case 6 in the roadmap). The lab should detect this and emit a `warning` per-area instead of silently producing no plan. Add an explicit check in P0.2.
- **BCD event ordering with collinear vertices.** The sweep can see multiple events at the same x. Use a stable secondary sort by y. Add a doctest with a deliberately collinear polygon.
- **`shapely.buffer` with `mitre` join can produce sharp slivers on near-180° vertices.** Use `mitre_limit=2.0` and validate the result with `Polygon.is_valid`; if invalid, fall back to `round` joins for that erosion call only.
- **Transit segments that cross an obstacle.** P0's eroded polygon already excludes obstacles, so walking along its boundary is safe by construction. If a transit ever produces an unsafe sample, that is a P0.2 bug, not a transit bug.
- **Performance on dense Google Earth outlines.** The unsimplified Whitburn Cres outline has hundreds of vertices and may produce many BCD events. Document `--simplify-tolerance-m` as the recommended workflow for raw KML, as the README already does. If BCD takes more than a few seconds, add a profiling section to `docs/COVERAGE_PLANNER_LAB.md`.

## Handoff checklist

When P0 and P1 are both landed:

- [ ] `docs/COVERAGE_PLANNER_ROADMAP.md` status rows updated with merge SHAs and post-P0-P1 baseline numbers.
- [ ] `docs/COVERAGE_PLANNER_LAB.md` "Known Gaps" section pruned: remove entries that P0 or P1 resolved, leave the rest, and reference the next plan doc when one is started.
- [ ] `tools/coverage_lab/AGENTS.md` and `tools/coverage_lab/README.md` mention the new `lab_geometry.py` module and the new metrics fields.
- [ ] Two new example maps tracked under `tools/coverage_lab/examples/`.
- [ ] One commit per step, each runnable on its own.
- [ ] Post-P0-P1 baseline metric values recorded in the roadmap for future regression comparison.
