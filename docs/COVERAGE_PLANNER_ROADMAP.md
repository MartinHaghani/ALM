# Coverage Planner Roadmap

Purpose: preserve the prioritization, design rationale, and implementation evidence needed to turn the laptop-only Fields2Cover lab into a mower-quality coverage planner. GitHub Issues are authoritative for current ownership and work status; this roadmap owns domain sequencing and historical outcomes.

This roadmap is shared between agents and humans. **Update the status table when work lands, do not rewrite the priorities silently.** When a priority is finished, leave the entry, mark it done, and link the merge commit so future agents can see what was implemented and why.

Related context:

- [COVERAGE_PLANNER_LAB.md](COVERAGE_PLANNER_LAB.md): current lab design, defaults, runtime semantics, and known gaps.
- [COVERAGE_PLANNER_V2_DESIGN.md](COVERAGE_PLANNER_V2_DESIGN.md): fresh mower-first planner design that keeps this roadmap's planner as a baseline while V2 develops.
- [tools/coverage_lab/README.md](../tools/coverage_lab/README.md): lab CLI usage.
- [tools/coverage_lab/AGENTS.md](../tools/coverage_lab/AGENTS.md): lab guardrails for agents.
- [src/lib/slic3r_coverage_planner/](../src/lib/slic3r_coverage_planner/): the slicer planner still wired into `mower_logic` at runtime. The lab does not yet replace it.

## Background

The mower currently ships the `slic3r_coverage_planner` from `src/lib/`. It covers the field but produces cutting paths that do not match the visual stripe quality expected from a lawnmower. The lab in `tools/coverage_lab/` was built to evaluate Fields2Cover (F2C) as a replacement. F2C is an agricultural CPP library; it is not a lawnmower planner. The lab wraps F2C with mower-specific logic (wheel-anchored stripe turns, footprint safety checks, KML conversion) and surfaces unsafe output instead of hiding it.

Historical pre-P0/P1 lab runs showed:

- Synthetic example coverage: 71–81%. Below lawnmower expectations.
- The synthetic `obstacle_map` example produces ~21 stripe-to-stripe turn failures because the swaths on opposite sides of the obstacle are 10+ m apart and the wheel-anchor planner cannot bridge them.
- Real Google Earth backyard maps (Whitburn Cres) reach ~95% coverage but with **30–49% of footprint samples flagged unsafe**, every one of them in the headland section. The headland centerline is generated as a constant inset of the boundary, so the rear-center-mounted footprint sweeps outside the lawn at every concave corner.

These tests motivated the priorities below. The later
[safety-clean baseline](#p1--obstacle-aware-swath-bridging) supersedes these values;
do not present this historical snapshot as current results.

## Status table

Listed in **execution order** (top = do next). IDs are stable per the "do not renumber" rule, so the ID column may jump around when an entry is reordered.

| Exec | ID | Topic | Roadmap record | Issue | Plan doc | Landed in |
|---|---|---|---|---|---|---|
| – | P0 | Footprint-aware headland | landed | – | [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md) | 5602b8e |
| – | P1 | Obstacle-aware swath bridging | landed | – | [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md) | 5602b8e |
| – | P11 | BCD critical-vertex decomposition (split at concave outer-boundary vertices, not just hole x-extents) | landed | – | – | 91c4f92 |
| – | P3 | Stripe-to-stripe turn diversity (omega, Y-turn, in-place pivot; skip-stripe ordering remains) | landed except skip-stripe follow-up | [tracking](https://github.com/MartinHaghani/open_mower_ros/issues/4) | – | 315cf02 |
| – | P12 | Robust dominant-direction stripe angle (replace F2C's `best_swath_length` with a weighted edge-angle histogram so noisy real-world outlines pick the visually-dominant axis, not the longest single segment) | landed | – | – | 9ca521f |
| – | P13 | Per-cell stripe angle for tight cells (cells whose short axis < ~2× tool_width along the global angle should rotate stripes to align with the cell's long axis, eliminating impossible U-turns in narrow slivers like obstacle_off_center paths 10–17) | landed | – | – | 9ca521f |
| – | P10 | Always-connected base-link path (eliminate teleports between paths) | landed | – | – | 310f63e |
| 1 | P5 | Coverage closes to ≥95% on synthetic maps (multi-headland, stripe overrun, gap-map overlay) | issue-owned | [#3](https://github.com/MartinHaghani/open_mower_ros/issues/3) | – | – |
| 2 | P2 | Stripe aesthetics (single angle, end discipline, blade scheduling, rotation memory, perimeter loop) | issue-owned | [#4](https://github.com/MartinHaghani/open_mower_ros/issues/4) | – | – |
| 3 | P4 | FTC-aware execution contract | issue-owned | [#6](https://github.com/MartinHaghani/open_mower_ros/issues/6) | – | – |
| 4 | P9 | Path smoothing and FTC-truthful preview | issue-owned | [#7](https://github.com/MartinHaghani/open_mower_ros/issues/7) | – | – |
| 5 | P8 | Stripe-quality regression suite | issue-owned | [#5](https://github.com/MartinHaghani/open_mower_ros/issues/5) | – | – |
| – | P6 | Multi-lawn navigation and dock integration | **dropped** (multi-lawn maps are now planned as separate maps; docking is being removed from runtime) | – | – | – |
| – | P7 | Slope and soft-zone awareness | **deferred** (re-evaluate after the coverage planner is stable; slope adds a variable that is not yet worth tracking) | – | – | – |

Roadmap states describe implementation history and sequencing. Current assignment,
priority, and completion state belong in the linked issue. When marking `landed`,
include the commit SHA or PR link in the last column and close the issue through the
merged PR.

Do not copy live assignment or in-progress state into this table; update the linked
issue and the applicable active ExecPlan.

### Items dropped or deferred

**P6 — Multi-lawn navigation and dock integration: dropped.** The decision is to handle multi-lawn properties as **separate maps** — each mow area is its own JSON map and planning run, with no cross-lawn connector emitted by the planner. The `cross_lawn_connectors_enabled` config flag defaults to `false`; the cross-lawn pass in `plan_profile_results` only runs when explicitly opted in. Docking is also being removed from the mower runtime, so the "dock integration" half of P6 is no longer a planner concern. If multi-lawn-as-one-map ever becomes a need again, P6 can be revived from this same entry.

**P7 — Slope and soft-zone awareness: deferred.** Re-evaluate after the planner is stable on flat geometry. Slope adds a variable that is not yet worth tracking — the current Whitburn-unsimplified run still has 4 visible gaps where no safe connector exists and coverage is 59.8 %; both numbers need to improve substantially before slope handling is the limiting factor.

### What each remaining priority means

**P5 — Coverage closes to ≥95 %.** Today every lab map sits between 71 % and 83 % coverage on the synthetic batch and around 60 % on Whitburn-unsimplified. The gap is the band between the eroded headland centerline and the lawn boundary (P0 gives up that band to keep the footprint safe at every yaw) plus stripe-placement misalignment between adjacent BCD bands (each band runs F2C independently and anchors stripe spacing to its own y-extent, so a ~tool_width/2 gap appears at every band boundary). Concrete subtasks: (1) make `outline_count` a function of perimeter-to-area ratio so wider lawns get a 2nd headland pass; (2) allow stripes to overrun their cell's y-extent by up to `tool_width/2` into the adjacent headland, eliminating the boundary gaps; (3) add an uncovered-area overlay to the SVG so the missing area is visible per-map.

**P2 — Stripe aesthetics.** Polish after the structural work. Single global stripe angle is already enforced (P12), but four cosmetic issues remain: stripe-end alignment (each stripe should terminate exactly at the lawn boundary projection along the stripe direction, not at a possibly-curved headland intersection); blade on/off scheduling promoted from `simulation_preview.json` into the path contract so the controller honours it; rotation-of-direction memory across sessions (alternate stripe angle every mow); perimeter cut closed as a loop with `tool_width/2` overlap at the join.

**P4 — FTC-aware execution contract.** Mower-side. The current `PlanPath` shape is a flat `nav_msgs/Path`. `FTCPlanner` skips duplicate-position poses (so in-place pivots silently drop), has no reverse semantics, and has no blade-state modulation. Replace with a maneuver-aware shape: `Path { segments[] { kind, blade_state, direction, poses[], maneuver_metadata } }` with `kind ∈ {STRIPE, HEADLAND, TRANSIT, PIVOT, REVERSE}`. v1 keeps `FTCPlanner` and never emits pivot/reverse segments to it; the lab continues to compute them for the preview only. v1.1 evaluates extending FTC or moving to MPC.

**P9 — Path smoothing and FTC-truthful preview.** Two things together. (1) Clothoid smoothing at stripe-to-headland transitions so yaw is C¹-continuous everywhere the blade is on. (2) Add an FTC-truthful preview mode that simulates `FTCPlanner`'s carrot-chase against the path and overlays the actual `base_link` trajectory; the current preview animates the exact pose path which can look clean while the controller's tracking has 30 cm of error.

**P8 — Stripe-quality regression suite.** Add `coverage_lab regress`. Runs all `examples/*.json` and the tracked Google Earth maps. Compares each `metrics.json` against `tools/coverage_lab/regression_baselines.json` (committed). Fails on: any increase in `safety.unsafe_footprint_samples`, any increase in `path_splits.within_cell_path_splits`, any drop in `coverage.coverage_percent` by more than 0.5 pp, any drop in `swath_length.mean_m` by more than 5 %, any new entry in `warnings`. Update the baseline only when the diff is reviewed and explicitly accepted in the commit message. Land this after P5 so the baselines are recorded at a higher quality.

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

**Outcome (310f63e).** `connect_paths_with_transits` post-processes the assembled `paths[]` list. For every consecutive pair whose endpoints differ by more than `2 × path_sample_step_m`, it tries: (1) a direct wheel-anchor / U-turn via `try_direct_inter_cell_connector`; (2) walking the eroded headland ring via `lab_geometry.walk_ring_between`, bracketed with the exact endpoint poses so the residual gap really closes; (3) a straight-line connector tagged `unsafe_transit=True` as a last resort. Cross-lawn jumps are handled in `plan_profile_results`: any consecutive paths whose `area_index` differs get an explicit `cross_lawn_transit=True` connector with a per-segment warning. `metrics.json` gains `fill_bridges.{inserted_count, direct_turn_count, unsafe_transit_count}` and `cross_lawn_transit_count`. On Whitburn the **within-lawn chunk-to-chunk jumps dropped from 29 (jumps > 0.5 m) and 5 (jumps > 5 m) to zero on both**; the two cross-property transitions between the three lawns now appear as explicit `cross-lawn 0 → 1` and `cross-lawn 1 → 2` records.

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

## Combined post-P3/P10/P12/P13 baseline

Four priorities landed together via parallel subagents and a single cherry-pick pass into `codex/remove-lowlevel-board`. The combined run on the synthetic batch + Whitburn shows:

| map | within-cell splits | jumps > 0.5 m | jumps > 5 m | cov% | per-cell angle overrides | turn planner counts |
|---|---:|---:|---:|---:|---:|---|
| rectangle_map | 0 → 0 | 0 → 0 | 0 → 0 | 80.94 | 0/1 | `wheel_anchor:12` |
| l_shape_map | 0 → 0 | **1 → 0** | **1 → 0** | 76.85 → **78.18** | 0/1 | `wheel_anchor:20` |
| narrow_pivot_map | 0 → 0 | **1 → 0** | **1 → 0** | 71.16 → **71.88** | 0/1 | `wheel_anchor:5` |
| obstacle_map | 0 → 1 | **2 → 0** | **1 → 0** | 80.27 → **82.79** | 0/4 | `wheel_anchor:28, forward_u_turn:1` |
| obstacle_off_center_map | 3 → **2** | **11 → 0** | **2 → 0** | 82.08 → 80.67 | 1/8 | `wheel_anchor:19, three_point_y:2, forward_u_turn:1` |
| two_obstacles_map | 0 → 0 | **4 → 0** | **2 → 0** | 81.57 → **82.33** | 2/7 | `wheel_anchor:40, forward_u_turn:5` |
| 57-Whitburn-Cres | 9 → 8 | **29 → 0** | **5 → 0** | 62.64 → 62.75 | **5/13** | `wheel_anchor:44, forward_u_turn:15, three_point_y:1` |

Headline: **within-lawn teleports are gone** on every map (all "jumps > 0.5 m" within a single lawn dropped to 0). The remaining "chunks" on each map are all explicit connectors — either a direct wheel-anchor/U-turn (P3 fallback), a headland-walk bridge (P10), or an explicit `cross_lawn_transit` segment with a per-segment warning (P10).

Whitburn now mowed as 3 fully-connected per-lawn paths joined by 2 explicit cross-lawn transits. `fill_bridges.inserted_count = 25`, of which 6 are direct inter-stripe turns and the rest are headland-ring walks. The path total grew from 484 m to 680 m — that 196 m is the cost of replacing every previous teleport with a real connector, which is exactly the trade asked for.

P3 turn diversity is paying off on `obstacle_off_center` (three_point_y picked up 2 turns the wheel-anchor + U-turn fallback would have failed on) and on Whitburn (1 three_point_y plus 15 forward_u_turn fallbacks). P13 per-cell angle is overriding 5 of Whitburn's 13 cells where the global angle would have produced unsweepable slivers.

The single-split regression on `obstacle_map` (0 → 1) is the new P12 dominant-direction angle picking a marginally different angle that triggered one wheel-anchor failure in a band that previously succeeded. Net win across the batch is overwhelming.

**Safety regression follow-up (491009b, d23691d).** The post-merge run also exposed a previously-hidden bug in cell reversal: `_reverse_cell_paths` rotates each pose's yaw by π, but the Mowrator footprint is asymmetric (front extends 0.82 m from base_link, rear is at base_link), so the asymmetric rotation moved the footprint front to where the back was and turned previously-safe poses into collisions. Counts at the merge point:

  obstacle_map         0 → 142 unsafe_footprint_samples
  obstacle_off_center  0 → 32
  two_obstacles_map    0 → 403
  Whitburn-simplified  0 → 920

Fixed by `_path_list_is_footprint_safe` + a `reverse_safe` gate in `optimize_cell_visit_order`: any cell whose reversed pose path puts the safety footprint outside the lawn or inside an obstacle is forced to forward direction regardless of transit distance.

Also fixed in the same pass: the P10 cross-lawn straight-line connector now defaults off (`cross_lawn_connectors_enabled: false`) so multi-lawn maps cleanly behave as separate plans; the within-area P10 straight-line last-resort fallback now footprint-validates before emitting and leaves a visible gap when no safe transit exists. The Whitburn run was also switched to the full-resolution `57-Whitburn-Cres-unsimplified.json` (the simplified version was a placeholder).

Final safety-clean baseline:

| map | unsafe samples | splits | chunks | cov% |
|---|---:|---:|---:|---:|
| rectangle_map | 0 | 0 | 2 | 80.94 |
| l_shape_map | 0 | 0 | 2 | 78.18 |
| narrow_pivot_map | 0 | 0 | 2 | 71.88 |
| obstacle_map | 0 | 1 | 15 | 82.79 |
| obstacle_off_center_map | 0 | 2 | 31 | 80.67 |
| two_obstacles_map | 0 | 0 | 27 | 82.34 |
| 57-Whitburn-Cres-unsimplified | 0 | 7 | 90 | 59.81 |

---

## P3 — Stripe-to-stripe turn diversity

**Outcome (315cf02).** Three new primitives in `coverage_lab.py`: `sample_omega_turn_base_poses` (forward overshoot + 180° side arc + return leg), `sample_three_point_y_turn_base_poses` (forward leg + in-place pivot + forward into end), and `sample_in_place_pivot_base_poses` (lab-only yaw-only pivot, gated behind `in_place_pivot_enabled` config because FTC cannot execute same-position pivots on the current mower). `plan_swath_turn_base_poses` now dispatches over `turn_fallback_planners` after the wheel-anchor primary attempt; the fallback list is now `[forward_u_turn, omega, three_point_y]` by default. Failure reasons are accumulated per-fallback in the warning string so an unplannable turn carries the diagnostic for all attempted primitives. Config knobs added: `omega_radius_m`, `omega_overshoot_m`, `y_turn_forward_m`, `y_turn_pivot_max_degrees`, `y_turn_pivot_step_degrees`, `in_place_pivot_enabled`.

On `obstacle_off_center_map` the split count dropped 3 → 2; the rescued turn used three_point_y. On Whitburn one Y-turn fired in the post-merge run. The remaining splits are the geometric impossibility at stripe_spacing 0.40 m < wheel_track 0.58 m — they need an additional skip-stripe ordering strategy that the agent left out of scope for this pass.

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

**Outcome (9ca521f).** `lab_geometry.dominant_direction_angle` builds a 1°-bin length-weighted edge-angle histogram (mod π so stripes are bidirectional), smooths with a Gaussian of `dominant_direction_smoothing_deg` (default 3°), and returns the peak bin's centre. Returns None when the peak is < 4× the mean bin so callers can fall back to F2C's `best_swath_length`. On the synthetic batch the chosen angle is 0° within tolerance for every axis-aligned map and matches the rectangle's rotation otherwise. The l_shape and narrow_pivot maps both saw small coverage improvements (+0.5–1.3 pp) from the cleaner angle pick.

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

**Outcome (9ca521f).** `lab_geometry.cell_long_axis_angle` (PCA on outer-ring vertices) and `lab_geometry.cell_short_axis_extent_along` let the per-cell F2C loop detect cells whose extent perpendicular to the global stripe angle is < `tool_width × tight_cell_stripe_threshold_factor` (default 2.0). Tight cells get their stripe angle overridden to the cell's long axis. `metrics.json.per_cell_angle` now reports `total_cells`, `tight_cell_count`, `angle_overrides`. On Whitburn 5 of 13 cells were flagged tight and rotated; on `obstacle_off_center_map` the sliver behind the obstacle (extent ≈ 0.13 m) was rotated. Splits in Whitburn dropped 9 → 8, short of the < 3 target — the remaining 8 are still geometric-impossibility cases that P3's missing skip-stripe ordering would resolve.

---

## P2 — Stripe aesthetics

**Problem.** Coverage% and path length are optimized, but visible stripe quality is not modeled. There is no global stripe direction (each F2C sub-region can pick its own best-length angle), no stripe-end alignment, no explicit blade on/off scheduling, no rotation-of-direction memory across sessions, and no closed-loop perimeter cut.

**Solution sketch.** Force a single global stripe angle per lawn; project stripe ends onto a true perpendicular boundary line; promote `blade_state` events from `simulation_preview.json` into the path contract; persist last-used stripe angle per lawn so successive mows rotate; close the perimeter loop with `tool_width/2` overlap at the join.

**Depends on.** P4 partially, because blade scheduling needs the maneuver-aware contract.

---

## P3 — Stripe-to-stripe turn diversity

**Status.** Mostly landed in `315cf02`: wheel-anchor remains primary, with forward U-turn, omega/keyhole, and three-point-Y fallbacks. The remaining unimplemented part is skip-stripe ordering, which should reduce split stripes that no local adjacent-stripe turn can solve.

**Remaining solution sketch.** Add skip-stripe ordering combined with omega/keyhole turns. Each maneuver should expose `(cost, min_clearance, time, reverse_distance, cutter_overlap)` so the selector can pick the lowest-cost footprint-safe option, with reverse heavily penalized. Skip-stripe + omega is still the likely lawnmower-aesthetic default.

---

## P4 — FTC-aware execution contract

**Problem.** The mower-facing `PlanPath` shape is a flat `nav_msgs/Path`. `FTCPlanner` is a follow-the-carrot controller that skips duplicate-position poses, has no reverse, no blade modulation, no notion of "this segment is transit, not coverage." Wheel-anchor turns produced by the lab cannot execute on the real mower.

**V2 bridge note (2026-06-19).** `classify-v2` now writes a lossy `planpath_compat.json` from V2 task paths. The bridge defaults to forward, blade-on `base_link` segments only and reports skipped reverse, rotate, or blade-off segments in `v2_planpath_compat_summary.json` plus `v2_metrics.json.planpath_compat`. This lets V2 become the source planner for simple-map tests while P4 remains the real maneuver-aware mower contract.

**Outline export note (2026-06-21).** V2 outline paths now smooth the sampled outline tangent/yaw before converting cutter-center samples to Mowrator `base_link` poses. The old per-chord yaw export could make an otherwise smooth inset outline appear as 0.3-0.5 m rear-center spikes in `planpath_compat.json`.

**Solution sketch.** Define a maneuver-aware planner message: `Path { segments[] { kind, blade_state, direction, poses[], maneuver_metadata } }` with `kind ∈ {STRIPE, HEADLAND, TRANSIT, PIVOT, REVERSE}`. v1 ships option C from the analysis: never emit pivot/reverse segments in the live planner; the lab plans them; the live mower only consumes FTC-trackable maneuvers (omega, three-point Y-turn, all-forward). v1.1 evaluates extending FTCPlanner or replacing it with MPC.

---

## P5 — Coverage closes to ≥95% on synthetic maps

**Problem.** Rectangle 81%, L-shape 77%, narrow_pivot 71%. The uncovered area is the boundary band between headland centerline and lawn edge, plus the stripe-end region.

**Solution sketch.** Make `outline_count` a function of perimeter-to-area ratio. Allow stripes to overrun into the headland by up to `tool_width / 2`, so the perimeter loop overlaps stripe ends and cleans the edge. Add an uncovered-area overlay to the SVG so the missing area is visible, not just summarized.

---

## P6 — Multi-lawn navigation and dock integration

**Status.** Dropped. Multi-lawn properties are now treated as separate maps/plans, and `cross_lawn_connectors_enabled` defaults to `false`. Dock integration is also no longer a planner priority because docking is being removed from the runtime.

**Revival note.** If a future workflow needs one plan spanning multiple lawns, revive this priority with nav-polygon routing and explicit transit semantics instead of re-enabling straight cross-lawn connectors.

---

## P7 — Slope and soft-zone awareness

**Status.** Deferred until the flat-geometry planner is stable and coverage quality is no longer the limiting factor.

**Future sketch.** Per-area slope vector annotation in map JSON; prefer stripe direction perpendicular to steepest descent; constrain reverse direction relative to slope; treat `obstacle:soft` typed polygons as no-go.

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
