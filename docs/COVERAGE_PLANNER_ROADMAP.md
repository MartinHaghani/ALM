# Coverage Planner Roadmap

Purpose: track the prioritized work needed to turn the laptop-only Fields2Cover lab into a mower-quality coverage planner. This is the index. Per-priority implementation plans live in separate docs and are linked below.

This roadmap is shared between agents and humans. **Update the status table when work lands, do not rewrite the priorities silently.** When a priority is finished, leave the entry, mark it done, and link the merge commit so future agents can see what was implemented and why.

Related context:

- [COVERAGE_PLANNER_LAB.md](COVERAGE_PLANNER_LAB.md): current lab design, defaults, runtime semantics, and known gaps.
- [tools/coverage_lab/README.md](../tools/coverage_lab/README.md): lab CLI usage.
- [tools/coverage_lab/AGENTS.md](../tools/coverage_lab/AGENTS.md): lab guardrails for agents.
- [src/lib/slic3r_coverage_planner/](../src/lib/slic3r_coverage_planner/): the slicer planner still wired into `mower_logic` at runtime. The lab does not yet replace it.

## Background

The mower currently ships the `slic3r_coverage_planner` from `src/lib/`. It covers the field but produces cutting paths that do not match the visual stripe quality expected from a lawnmower. The lab in `tools/coverage_lab/` was built to evaluate Fields2Cover (F2C) as a replacement. F2C is an agricultural CPP library; it is not a lawnmower planner. The lab wraps F2C with mower-specific logic (wheel-anchored stripe turns, footprint safety checks, KML conversion) and surfaces unsafe output instead of hiding it.

The most recent lab runs (`tools/coverage_lab/runs/`) show:

- Synthetic example coverage: 71–81%. Below lawnmower expectations.
- The synthetic `obstacle_map` example produces ~21 stripe-to-stripe turn failures because the swaths on opposite sides of the obstacle are 10+ m apart and the wheel-anchor planner cannot bridge them.
- Real Google Earth backyard maps (Whitburn Cres) reach ~95% coverage but with **30–49% of footprint samples flagged unsafe**, every one of them in the headland section. The headland centerline is generated as a constant inset of the boundary, so the rear-center-mounted footprint sweeps outside the lawn at every concave corner.

These tests motivate the priorities below.

## Status table

Listed in **execution order** (top = do next). IDs are stable per the "do not renumber" rule, so the ID column may jump around when an entry is reordered.

| Exec | ID | Topic | Status | Plan doc | Landed in |
|---|---|---|---|---|---|
| – | P0 | Footprint-aware headland | landed | [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md) | 5602b8e |
| – | P1 | Obstacle-aware swath bridging | landed | [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md) | 5602b8e |
| – | P11 | BCD critical-vertex decomposition (split at concave outer-boundary vertices, not just hole x-extents) | landed | – | 91c4f92 |
| 1 | P3 | Stripe-to-stripe turn diversity (omega, Y-turn, in-place pivot, skip-stripe ordering) | not started — next | – | – |
| 2 | **P12** | Robust dominant-direction stripe angle (replace F2C's `best_swath_length` with a weighted edge-angle histogram so noisy real-world outlines pick the visually-dominant axis, not the longest single segment) | not started | – | – |
| 3 | **P13** | Per-cell stripe angle for tight cells (cells whose short axis < ~2× tool_width along the global angle should rotate stripes to align with the cell's long axis, eliminating impossible U-turns in narrow slivers like obstacle_off_center paths 10–17) | not started | – | – |
| 4 | P10 | Always-connected base-link path (eliminate teleports between paths) | not started | – | – |
| 5 | P6 | Multi-lawn navigation and dock integration | not started | – | – |
| 6 | P5 | Coverage closes to ≥95% on synthetic maps (multi-headland, stripe overrun, gap-map overlay) | not started | – | – |
| 7 | P2 | Stripe aesthetics (single angle, end discipline, blade scheduling, rotation memory, perimeter loop) | not started | – | – |
| 8 | P4 | FTC-aware execution contract | not started | – | – |
| 9 | P9 | Path smoothing and FTC-truthful preview | not started | – | – |
| 10 | P7 | Slope and soft-zone awareness | not started | – | – |
| 11 | P8 | Stripe-quality regression suite | not started | – | – |

Statuses: `not started`, `in progress`, `blocked`, `landed`. When marking `landed`, include the commit SHA or PR link in the last column.

When a priority is in progress, the assigned agent must update its row with a one-line note: "in progress — <agent or branch>".

### Reorder rationale (post-P0/P1 review)

After P0/P1 landed, an output review found that the dominant remaining problem on every map (synthetic and real) is **teleports between consecutive path pieces**, not turn quality or coverage. A path-anatomy pass over the post-P0/P1 baseline counted 12–54 chunk-to-chunk jumps per map, with the largest jump on Whitburn at 29.7 m across the property between two separate lawns. Four distinct teleport categories exist (within-cell stripe-to-stripe failure, ring-to-ring, headland-to-fill, cross-lawn), all with the same root cause: nothing forces consecutive `paths[]` entries to share endpoints.

P10 was added because none of the existing priorities directly required consecutive paths to connect; P3 reduces the *count* of stripe-to-stripe failures but does not stop the path from fragmenting. P11 was added because the existing P1 only decomposes around interior holes, but a hole that touches the eroded outer boundary collapses into a notch and Fields2Cover then fragments every stripe that crosses it. The two are independent: P10 owns path topology, P11 owns swath geometry.

P10 was placed at execution slot 1 because it has the largest immediate visual effect, it is a single localized post-processing pass, and it makes every subsequent priority easier to evaluate (remaining problems become visible long transits instead of confusing teleports). P11 at slot 2 because it closes the second-biggest remaining symptom on the synthetic batch. P3 was moved up to slot 3 because turn quality after P10 becomes the dominant remaining visual issue.

---

## P0 — Footprint-aware headland

**Problem.** The lab generates the headland as a constant inward offset of the lawn boundary by `clearance + outline_count * tool_width`. The `outline_clearance_m: auto` setting derives the inset from `max(footprint extent) + safety_margin_m`. This is safe on straight segments. On every concave corner of a realistic boundary, the front-outboard footprint corner sweeps wider than the inset polyline because `base_link` sits at the rear-center of an 0.82 m × 0.68 m footprint. The result is 30–49% of headland footprint samples leaving the lawn on Whitburn Cres maps.

**Solution.** Plan the headland centerline as the boundary of `lawn ⊖ footprint_disk`, a Minkowski erosion using the footprint's circumscribed disk, then resample with corner-arc smoothing at the minimum turning radius. Document option evaluation and stepwise implementation in [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md).

**Acceptance.** On the tracked example maps and the Whitburn Cres real-world map, `metrics.json.safety.unsafe_footprint_samples == 0` for all headland-section samples. Coverage drop from the more conservative inset is allowed up to a configurable percent and is compensated by P5.

**Outcome (5602b8e).** Landed jointly with P1 because both share the polygon-erosion machinery in [tools/coverage_lab/lab_geometry.py](../tools/coverage_lab/lab_geometry.py). The Fields2Cover constant inset was replaced with `shapely.buffer(-r, join_style="round")` where `r = footprint_disk_radius(config)`. On the synthetic batch and Whitburn Cres, `safety.unsafe_footprint_samples` is now `0` everywhere. The honest safe-coverage on Whitburn dropped from a fake 95.68% (which counted the cutter sweeping while 30% of the footprint was outside the lawn) to 65.08%; P5 owns recovering the lost edge band by allowing multiple headlands and stripe overrun. See post-P0/P1 baseline below.

---

## P1 — Obstacle-aware swath bridging

**Problem.** F2C generates a swath per straight line crossing the polygon. When an obstacle is inside the lawn, swaths get clipped into two collinear segments on opposite sides of the obstacle. The wheel-anchor turn planner is asked to bridge anchor poses that may be many metres apart across the obstacle and fails. On the `obstacle_map` example this produces 21 unplanned turn gaps and a heavily fragmented path.

**Solution.** Decompose the mow polygon (lawn minus obstacles) into sub-regions with no interior holes before swath generation, then run F2C per sub-region and stitch sub-regions together with explicit transit segments. Recommended algorithm: Boustrophedon Cellular Decomposition (BCD) aligned to the chosen stripe direction. Document option evaluation and stepwise implementation in [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md).

**Acceptance.** `obstacle_map` runs with 0 unplanned turn gaps and a single connected base-link path. Whitburn Cres runs show `metrics.json.connector_path.connector_runs` reduced by at least half versus the pre-P1 baseline.

**Outcome (5602b8e).** Boustrophedon Cellular Decomposition aligned to the chosen swath angle: cells are cut at each hole's x-extents in the stripe frame, so every cell is hole-free and Fields2Cover plans clean swaths per cell. Between cells the planner walks the eroded headland boundary as an explicit transit segment, guaranteed footprint-safe by construction. The natural F2C swath angle is probed once on the union mainland, then a fixed angle is used per cell so stripes stay visually parallel across the whole lawn (P2 stripe-direction continuity gets a free win here).

On `obstacle_map`, the pre-P1 21 hard-fail turn warnings dropped to 13, and 9 of those 13 now fall back gracefully to the forward U-turn fallback instead of leaving a gap. Whitburn Cres `connector_path.connector_runs` dropped from 42 to 4 (an 89% reduction), well past the 50% target.

**P1 v1 sweep direction was wrong; corrected at 6883754.** The original BCD cut along x (vertical columns), but for stripes-along-x the correct sweep is along y (horizontal bands). The X-cut version forced every stripe to be ≤ column width, producing 149 short stripes and 30 warnings on `two_obstacles_map` where the geometry should naturally support full-width stripes wherever no obstacle blocks. The Y-cut fix gives cell shapes that match the boustrophedon flow: bands at obstacle-free y values produce full-width long stripes; bands intersecting an obstacle split into left/right sub-cells only in that obstacle's y-range. Same total cell count, very different cell shapes and stripe counts. A `swath_length` section was also added to `metrics.json` (mean/median/min/max/short_count) so this class of regression is visible without manual inspection in the future.

**P1 v2 visit-order optimisation landed at 9aeccc3.** The original P1 assembled cells in the arbitrary order BCD returned them, always inserted a headland transit between consecutive cells, and never tried a direct stripe-to-stripe turn even when the cells were y-adjacent. That produced visible "teleports" the user flagged at obstacle_map pose 1889 (a transit chunk where a wheel-anchor turn would have continued the path), pose 3929 (a 197-pose perimeter transit because Cell 3 was visited last in forward direction, far from Cell 2's end), and the obstacle_off_center 3-section sequencing problem.

The fix is three small changes in `_plan_one_lawn_profiles_footprint_disk`:

- `optimize_cell_visit_order` runs a **multi-start greedy nearest-neighbour**: try every (starting cell, starting direction) pair, greedy-walk from each, keep the lowest-total-transit order. Each cell can be visited F2C-forward or pose-reversed. This consistently picks a starting cell whose F2C-default endpoint sits near the rest of the cells; single-start greedy was locking the planner into corner exits.
- `try_direct_inter_cell_connector` calls the existing `plan_swath_turn_base_poses` between consecutive cells; a footprint-safe wheel-anchor or forward U-turn becomes the connector. Headland transit is now the fallback, not the default.
- Three new metrics: `metrics.json.path_splits.within_cell_path_splits`, `metrics.json.inter_cell.direct_turn_count`, plus `metrics.json.transit.{count,total_length_m}` which now drops as direct turns replace transits. The first one is the user-flagged priority metric.

Results vs the post-direction-fix run:

| map | transits | direct turns | within-cell splits | path total |
|---|---|---|---|---|
| obstacle_map | 3 → **0** | 3 | 2 | unchanged |
| obstacle_off_center_map | 6 → **4** | 2 | 3 | unchanged |
| two_obstacles_map | 6 → **3** | 3 | 2 | unchanged |
| 57-Whitburn-Cres | transit length **75.9 → 21.0 m** (−72 %) | 0 | 37 | base_link 522.8 → **467.9 m** (−11 %) |

The within-cell splits on `obstacle_map` (2), `two_obstacles_map` (2), `obstacle_off_center_map` (3) and `57-Whitburn-Cres` (37) are wheel-anchor failures at stripe_spacing 0.40 m < wheel_track 0.58 m — a geometric impossibility for the current wheel-anchor maneuver, not a planner bug. They belong to P3 (richer turn library: omega, three-point Y-turn, skip-stripe ordering).

---

## Post-P0/P1 baseline

This baseline was recorded at commit 5602b8e and is the reference for P8 regression. Re-running the lab on these maps with the current `tools/coverage_lab/configs/default.yaml` should reproduce the values within rounding noise.

| Map | Coverage % | Unsafe footprint samples | Cells | Transits | Warnings |
|---|---:|---:|---:|---:|---:|
| `examples/rectangle_map.json` | 80.94 | 0 | 1 | 0 | 0 |
| `examples/l_shape_map.json` | 76.85 | 0 | 1 | 0 | 0 |
| `examples/narrow_pivot_map.json` | 71.16 | 0 | 1 | 0 | 1 |
| `examples/obstacle_map.json` | 83.21 | 0 | 4 | 3 | 13 |
| `examples/obstacle_off_center_map.json` | 82.31 | 0 | 1 | 0 | 18 |
| `examples/two_obstacles_map.json` | 83.61 | 0 | 7 | 6 | 30 |
| `data/maps/google_earth/57-Whitburn-Cres-simplified.json` (private) | 65.08 | 0 | 7 | 4 | 64 |

Notes:
- Synthetic example coverage caps at 71–84%. The missing band is the unswept area between the eroded headland boundary and the lawn boundary (the band P0 had to give up to be safe). P5 recovers it.
- `obstacle_off_center_map` shows 1 cell, 0 transits because the inflated obstacle touches the eroded outer boundary and the hole collapses into a notch — correct geometry, no decomposition needed.
- Warning counts above zero are not unsafe; they are wheel-anchor turn failures that fall back to a forward U-turn or split the path at a visible gap. Reducing them is P3 territory.

---

## P10 — Always-connected base-link path

**Problem.** After P0/P1 landed, every test map produces a `paths[]` list where consecutive entries do not share endpoints. The simulation preview interprets each gap as a teleport, including 6–14 m jumps within a single cell and 29.7 m cross-property jumps on Whitburn Cres. Four distinct categories of teleport were measured:

1. *Within-cell stripe-to-stripe failure.* When both the wheel-anchor turn and the forward-U-turn fallback fail for a stripe pair, `build_zero_turn_fill_paths` calls `finish_current()` and starts a new `fill` chunk with no connector. The gap is the lawn width.
2. *Ring-to-ring within a single headland.* The headland builder emits one `path` entry per ring; outer and obstacle-hole rings have no connector between them even though both are footprint-safe by P0 construction.
3. *Headland → first fill stripe.* The last headland pose and the first fill pose are typically several metres apart.
4. *Cross-lawn.* Independent mow areas (e.g. Whitburn's three lawns) are planned in isolation; no connector exists between the last pose of one lawn's path and the first pose of the next.

These teleports also explain the "instant 180° turn" symptom: a path-anatomy pass over the post-P0/P1 baseline measured zero instant yaw jumps > 85° between *adjacent* poses, but many of the chunk-to-chunk gaps have Δyaw ≈ 180° because consecutive stripes alternate direction.

**Solution.** After planning, walk the full `paths[]` and insert an explicit transit segment between any two consecutive entries whose endpoints don't match within `path_sample_step_m`. The transit:

- *Within a lawn, both endpoints lie on the eroded headland*: use `lab_geometry.walk_ring_between` on the largest eroded ring (P0 output is already footprint-safe by construction). This covers categories 1, 2, and 3.
- *Cross-lawn*: emit a straight-line connector tagged with `cross_lawn_transit: true` and a per-segment warning. P6 will replace this with proper nav-polygon routing.
- All inserted poses are tagged `section: "connector"` with `transit: true` and `cutting_enabled: false`, so coverage and turn metrics are not polluted.

This is the same `build_transit_path` machinery already used for inter-cell transit in P1; the change is to *also* run it as a final post-processing pass over the assembled per-profile `paths[]`.

**Acceptance.**

- For every tracked example map and the Whitburn Cres real-world map, the path-anatomy pass returns **0 chunk-to-chunk jumps > `2 × path_sample_step_m`** within a single lawn (i.e. the within-area path is fully connected).
- Cross-lawn jumps still exist on Whitburn but are now explicit `cross_lawn_transit` connectors with warnings, not silent teleports.
- `metrics.json.transit.count` increases (every inserted segment is counted); coverage % and unsafe sample count do not change.

**Why this slot.** It is a self-contained change with the largest single visual improvement available, and it makes every subsequent priority easier to evaluate. P3, P5, P11 all leave teleports in place; only P10 removes them.

---

## P11 — BCD critical-vertex decomposition

**Problem.** The current `lab_geometry.bcd_decompose` only cuts at hole x-extents. When a lawn obstacle sits near the outer boundary, the eroded obstacle merges with the eroded outer boundary and the resulting mainland is hole-free but has a concave "notch" cut out of one side. My BCD then skips decomposition entirely (because there are no interiors), Fields2Cover plans swaths across the notched polygon, and every stripe that crosses the notch gets fragmented into two short stripes with no internal connection. The `obstacle_off_center_map` example exposes this clearly: 1 cell, 31 swaths, 17 fill chunks for what should be ~25 clean stripes plus a small notch region.

**Solution.** Implement proper Boustrophedon Cellular Decomposition with critical-vertex detection on the outer boundary. A critical vertex in the stripe-aligned frame is one where the boundary turns away from the sweep direction (a local extremum in x for stripes along x). At each critical vertex, cut a vertical line through it and intersect with the polygon. Cells are the connected components of `polygon - cut_union`.

Specifically, extend `bcd_decompose` to:

1. Rotate the polygon so stripes run along x (existing behaviour).
2. Walk the outer ring and classify each vertex as a critical vertex if the boundary's x-derivative changes sign across it *and* the vertex is concave (inside angle > 180°).
3. Walk each interior ring and classify each hole vertex the same way (existing behaviour for the leftmost/rightmost cases, extended to all critical vertices for non-convex holes).
4. Cut at every critical vertex's x-coordinate, deduplicated and snapped at `1 mm` tolerance.
5. Return cells with the same hole-free, area-filtered invariants as today.

**Acceptance.**

- On `obstacle_off_center_map`, BCD returns at least 2 cells (one clean rectangle below the notch, plus the notched region) and the bottom cell produces approximately 16 long stripes with no internal fragmentation.
- `metrics.json.cells.count` ≥ 2 for any map whose mainland polygon has at least one concave critical vertex on the outer boundary.
- No regression on the maps where the current BCD already produces correct cells (rectangle, l_shape, narrow_pivot, obstacle, two_obstacles).

**Why this slot.** Second-biggest visual improvement after P10. Reduces stripe count and turn count on every map with a notched outer boundary. Independent of P10 (different code paths), so safe to land in a separate pass.

**Outcome (91c4f92).** Implementation in `lab_geometry.outer_reflex_xs`: walk the CCW outer ring, aggregate signed turn angles into contiguous reflex runs (a single rolled-disk arc from Minkowski erosion becomes one run with cumulative turn ≈ −π/2), emit one candidate cut per run at the |turn|-weighted centroid x. A second filter pass keeps only candidates that *actually fragment* a horizontal swath — the polygon's intersection with a horizontal line at the reflex y must have more than one connected component. This drops benign concavities like the L-shape's inner corner (cross-section stays one segment) and keeps notch corners (cross-section is two segments).

Results vs the post-P0/P1 baseline (no regressions; large wins where the symptom existed):

| map | warnings | cells | chunk jumps > 5 m |
|---|---:|---:|---:|
| rectangle_map | 0 → 0 | 1 → 1 | 0 → 0 |
| l_shape_map | 0 → 0 | 1 → 1 | 1 → 1 |
| narrow_pivot_map | 1 → 1 | 1 → 1 | 1 → 1 |
| obstacle_map | 13 → 13 | 4 → 4 | 6 → 6 |
| `obstacle_off_center_map` | **18 → 8** | 1 → 5 | **10 → 1** |
| two_obstacles_map | 30 → 30 | 7 → 7 | 9 → 9 |
| 57-Whitburn-Cres-simplified | 64 → 63 | 7 → 10 | **14 → 4** |

The dominant visible improvement is the collapse of large chunk-to-chunk jumps on the two maps that have outer-boundary notches. Remaining short-and-medium gaps are owned by P10 (always-connected base-link path).

---

## P12 — Robust dominant-direction stripe angle

**Problem.** Fields2Cover's `best_swath_length` picks the swath angle that maximises a single swath's length across the polygon. On synthetic rectangles it works well. On every other map — `obstacle_map`, `obstacle_off_center_map`, `two_obstacles_map`, the L-shape, narrow_pivot — it picks an angle a few degrees off the visually-dominant axis because the polygon's longest individual chord is on a slight diagonal even when the boundary is overwhelmingly axis-aligned. On real-world recorded outlines (Whitburn Cres) the problem is worse: the boundary is sampled at near-uniform spacing so there are no "longest edges" at all, and F2C's chosen angle is determined by whichever short chord happens to be slightly longer than the others. The user noted that stripes look "slightly skewed" on every example except `rectangle_map`.

**Solution.** Replace the swath-angle probe with a dominant-direction estimator on the eroded outer boundary:

1. Walk the eroded outer ring. For each edge, record its angle modulo π (since stripes are bidirectional) weighted by edge length.
2. Build a 1° histogram of weighted edge angles. Apply a small Gaussian smoothing kernel (~3°) so noisy single-segment edges don't create local spikes.
3. The peak bin of the smoothed histogram is the dominant direction; use that as the global stripe angle.
4. Fall back to F2C's `best_swath_length` only if the histogram is flat (no clear peak within 1.5× of the median bin).

This is robust to high-vertex-count recorded outlines (the dominant direction emerges from the cumulative edge-length weighting) and to small geometric perturbations (the histogram bin is wide enough to smooth them out).

**Acceptance.** On `obstacle_map`, `obstacle_off_center_map`, `two_obstacles_map`, `l_shape_map`, and `narrow_pivot_map`, the chosen stripe angle is within 0.5° of horizontal (or whichever true cardinal axis the boundary uses). On Whitburn the chosen angle visually matches the dominant property axis when the outline is overlaid on the SVG.

---

## P13 — Per-cell stripe angle for tight cells

**Problem.** With a single global stripe angle, BCD sub-cells whose short axis is parallel to the global stripe angle can have a width smaller than a single tool footprint — so stripes don't fit at all — or smaller than the U-turn radius — so the wheel-anchor / forward-U-turn / trim retry all fail with `negative_reverse_distance`. On `obstacle_off_center_map` this produces the 4-line slivers around the obstacle (paths 10–17 in the current run) where the geometry forbids any maneuver. The Whitburn cells reproduce the same pathology at scale: 9 of the 9 remaining splits after trim retry are in narrow sub-cells whose long axis runs perpendicular to the chosen global stripe.

**Solution.** When a cell's short-axis extent along the global stripe angle is less than `cell_local_stripe_angle_threshold_m` (default ≈ 2× tool_width, i.e. ~0.8 m), locally rotate stripes inside that cell to align with the cell's long axis:

1. After BCD, compute each cell's minimum-area bounding rectangle (or fit a line via PCA on the cell ring).
2. If the cell's short axis along the global stripe direction is below threshold, override that cell's swath angle to match its long axis.
3. Within the cell, run F2C swath generation with the local angle.
4. Inter-cell transitions are tagged so the visit-order optimiser still works on rotated-stripe cells. The wheel-anchor planner already handles arbitrary start/end yaws.

This breaks visual stripe-direction continuity across the rotated cell, but the alternative — a fragmented patchwork of unmowed slivers — is worse. P2 (stripe aesthetics) can later add a "redirect" maneuver that smooths the angle transition at the sliver boundary.

**Acceptance.** On `obstacle_off_center_map`, the slivers around the obstacle (current paths 10–17) collapse into one or two cells with continuous stripes aligned to the cell's long axis, and no split happens. On Whitburn, the within-cell split count drops from 9 (current) to under 3.

---

## P2 — Stripe aesthetics

**Problem.** Coverage% and path length are optimized, but visible stripe quality is not modeled. There is no global stripe direction (each F2C sub-region can pick its own best-length angle), no stripe-end alignment, no explicit blade on/off scheduling, no rotation-of-direction memory across sessions, and no closed-loop perimeter cut.

**Solution sketch.** Force a single global stripe angle per lawn; project stripe ends onto a true perpendicular boundary line; promote `blade_state` events from `simulation_preview.json` into the path contract; persist last-used stripe angle per lawn so successive mows rotate; close the perimeter loop with `tool_width/2` overlap at the join.

**Depends on.** P4 partially, because blade scheduling needs the maneuver-aware contract.

---

## P3 — Stripe-to-stripe turn diversity

**Problem.** The only viable maneuver is the wheel-anchor turn. Forward U-turn fallback rarely succeeds because `2 * min_turning_radius` typically exceeds stripe spacing. When wheel-anchor fails, no fallback works and the path is split. Real robot mowers visibly use omega turns and three-point Y-turns; neither is implemented.

**Solution sketch.** Maneuver library: wheel-anchor (existing), three-point Y-turn, omega/keyhole turn, in-place pivot at stripe end, skip-stripe ordering combined with omega. Each maneuver returns `(cost, min_clearance, time, reverse_distance, cutter_overlap)`; selector picks the lowest-cost footprint-safe option, with reverse heavily penalized. Skip-stripe + omega becomes the lawnmower-aesthetic default.

---

## P4 — FTC-aware execution contract

**Problem.** The mower-facing `PlanPath` shape is a flat `nav_msgs/Path`. `FTCPlanner` is a follow-the-carrot controller that skips duplicate-position poses, has no reverse, no blade modulation, no notion of "this segment is transit, not coverage." Wheel-anchor turns produced by the lab cannot execute on the real mower.

**Solution sketch.** Define a maneuver-aware planner message: `Path { segments[] { kind, blade_state, direction, poses[], maneuver_metadata } }` with `kind ∈ {STRIPE, HEADLAND, TRANSIT, PIVOT, REVERSE}`. v1 ships option C from the analysis: never emit pivot/reverse segments in the live planner; the lab plans them; the live mower only consumes FTC-trackable maneuvers (omega, three-point Y-turn, all-forward). v1.1 evaluates extending FTCPlanner or replacing it with MPC.

---

## P5 — Coverage closes to ≥95% on synthetic maps

**Problem.** Rectangle 81%, L-shape 77%, narrow_pivot 71%. The uncovered area is the boundary band between headland centerline and lawn edge, plus the stripe-end region.

**Solution sketch.** Make `outline_count` a function of perimeter-to-area ratio. Allow stripes to overrun into the headland by up to `tool_width / 2`, so the perimeter loop overlaps stripe ends and cleans the edge. Add an uncovered-area overlay to the SVG so the missing area is visible, not just summarized.

---

## P6 — Multi-lawn navigation and dock integration

**Problem.** Each lawn is planned in isolation. `nav` polygons are parsed but unused. Docking stations are in the map but ignored.

**Solution sketch.** Plan inter-lawn transit through `nav` polygons (visibility graph or A* on a navmesh). Lawn ordering as TSP weighted by transit cost. Dock entry/exit as fixed maneuvers from `docking_stations[]`. Output one executable path with explicit `kind: TRANSIT` segments.

---

## P7 — Slope and soft-zone awareness

**Problem.** No slope awareness in the coverage planner. Slope reliability work exists on a separate branch but is not integrated.

**Solution sketch.** Per-area slope vector annotation in map JSON. Prefer stripe direction perpendicular to steepest descent. Constrain reverse direction relative to slope. Treat `obstacle:soft` typed polygons as no-go. Merge with slope-reliability branch when it lands.

---

## P8 — Stripe-quality regression suite

**Problem.** Manual visual checks only. Any planner change risks silent regressions.

**Solution sketch.** Add `coverage_lab regress`: run all `examples/*.json` and tracked Google Earth maps, compare `metrics.json` against `tools/coverage_lab/regression_baselines.json`, fail on `unsafe_footprint_samples` increase, coverage drop > 0.5 pp, or new warnings. Update the baseline only with reviewer sign-off in the commit message.

---

## P9 — Path smoothing and FTC-truthful preview

**Problem.** Path is a polyline; sharp yaw jumps cause visible swerves. The HTML preview animates exact poses, not the controller's actual trajectory, so the lab cannot see tracking errors.

**Solution sketch.** Clothoid smoothing at stripe-headland transitions. Add an FTC-truthful preview that simulates the carrot chase against the path and overlays the actual `base_link` trajectory.

---

## Natural-lawn edge cases

These are the cases the planner must handle eventually. They drive the regression suite (P8) and the acceptance criteria of every priority above. Ordered by frequency in real backyards.

| # | Case | Why naive planners break |
|---|---|---|
| 1 | Concave corner of internal angle 60–120° | Footprint sweeps outside headland inset (root cause of Whitburn Cres unsafe samples) |
| 2 | Acute corner < 60° | Inset self-intersects or vanishes; needs an explicit corner pivot |
| 3 | Convex bulge | F2C swath assignment can leave a triangular uncovered patch |
| 4 | Obstacle inside lawn (single tree) | Splits swaths; needs sub-region decomposition |
| 5 | Obstacle adjacent to boundary | Inset between obstacle and boundary collapses; "neck" too narrow for footprint |
| 6 | Narrow neck or strip < 2 × tool_width | Single-stripe path; turns will not fit; needs explicit drive-through mode |
| 7 | Two disjoint mow areas | Needs inter-lawn transit through nav region |
| 8 | Annular lawn around a building | Topological hole; same shape as obstacle but at larger scale |
| 9 | Long thin strip alongside driveway | Stripe-along-strip is one stripe wide; stripe-across creates many tiny stripes |
| 10 | Curved boundary, no straight edges | F2C `best_swath_length` returns an irrelevant angle; headland is constantly curving |
| 11 | Boundary with many tiny segments (Google Earth unsimplified) | Each segment is a swath-end discontinuity; needs `--simplify-tolerance-m` |
| 12 | Lawn slope | Stripes across slope mandatory; reverse downhill unsafe (P7) |
| 13 | Damp or soft patch | Treat as obstacle of `kind: soft` (P7) |
| 14 | Sprinkler head or low fixture | Point obstacle; needs small-circle obstacle support |
| 15 | Lawn adjacent to hard surface (driveway, walkway) | Zero overcut allowed on that edge; allow underflow into lawn |
| 16 | Lawn adjacent to garden bed | Slight underflow OK, overcut forbidden; soft-edge marking |
| 17 | Variable growth (sunny vs shady) | Future: variable cut density |
| 18 | Dock corridor inside the lawn | Reserved approach corridor at a specific angle; not coverage area |
| 19 | Boundary self-near-self (narrow loop-back) | Inset self-intersects; needs topological cleanup |
| 20 | Previous-session stripe-direction memory | Aesthetic and grass-health requirement; needs persistent state (P2) |

Cases 1–6 will break v1 in the field. They should each have a tracked example map under `tools/coverage_lab/examples/` and a row in the regression baseline (P8).

## How to update this roadmap

When you finish a priority:

1. Update the status row with the merge SHA or PR link.
2. Add a short "Outcome" subsection at the end of the relevant priority section: what changed, what acceptance criterion was met, what to watch for next.
3. If new edge cases were discovered during implementation, add them to the table.
4. If a follow-up priority is needed, add it as `P10`, `P11`, etc. Do not renumber.
5. Do not delete entries. Done work stays visible so the next agent understands the lineage.

When you start a priority:

1. Mark the status row "in progress — <branch or agent>".
2. Create or append to its plan doc if the existing plan needs adjustment, and explain why in the doc.
3. Add or update tracked example maps under `tools/coverage_lab/examples/` for the edge cases the priority targets.
