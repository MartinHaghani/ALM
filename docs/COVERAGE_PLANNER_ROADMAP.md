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

| Priority | Topic | Status | Plan doc | Landed in |
|---|---|---|---|---|
| P0 | Footprint-aware headland | not started | [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md) | – |
| P1 | Obstacle-aware swath bridging | not started | [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md) | – |
| P2 | Stripe aesthetics (single angle, end discipline, blade scheduling, rotation memory, perimeter loop) | not started | – | – |
| P3 | Stripe-to-stripe turn diversity (omega, Y-turn, in-place pivot, skip-stripe ordering) | not started | – | – |
| P4 | FTC-aware execution contract | not started | – | – |
| P5 | Coverage closes to ≥95% on synthetic maps (multi-headland, stripe overrun, gap-map overlay) | not started | – | – |
| P6 | Multi-lawn navigation and dock integration | not started | – | – |
| P7 | Slope and soft-zone awareness | not started | – | – |
| P8 | Stripe-quality regression suite | not started | – | – |
| P9 | Path smoothing and FTC-truthful preview | not started | – | – |

Statuses: `not started`, `in progress`, `blocked`, `landed`. When marking `landed`, include the commit SHA or PR link in the last column.

When a priority is in progress, the assigned agent must update its row with a one-line note: "in progress — <agent or branch>".

---

## P0 — Footprint-aware headland

**Problem.** The lab generates the headland as a constant inward offset of the lawn boundary by `clearance + outline_count * tool_width`. The `outline_clearance_m: auto` setting derives the inset from `max(footprint extent) + safety_margin_m`. This is safe on straight segments. On every concave corner of a realistic boundary, the front-outboard footprint corner sweeps wider than the inset polyline because `base_link` sits at the rear-center of an 0.82 m × 0.68 m footprint. The result is 30–49% of headland footprint samples leaving the lawn on Whitburn Cres maps.

**Solution.** Plan the headland centerline as the boundary of `lawn ⊖ footprint_disk`, a Minkowski erosion using the footprint's circumscribed disk, then resample with corner-arc smoothing at the minimum turning radius. Document option evaluation and stepwise implementation in [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md).

**Acceptance.** On the tracked example maps and the Whitburn Cres real-world map, `metrics.json.safety.unsafe_footprint_samples == 0` for all headland-section samples. Coverage drop from the more conservative inset is allowed up to a configurable percent and is compensated by P5.

---

## P1 — Obstacle-aware swath bridging

**Problem.** F2C generates a swath per straight line crossing the polygon. When an obstacle is inside the lawn, swaths get clipped into two collinear segments on opposite sides of the obstacle. The wheel-anchor turn planner is asked to bridge anchor poses that may be many metres apart across the obstacle and fails. On the `obstacle_map` example this produces 21 unplanned turn gaps and a heavily fragmented path.

**Solution.** Decompose the mow polygon (lawn minus obstacles) into sub-regions with no interior holes before swath generation, then run F2C per sub-region and stitch sub-regions together with explicit transit segments. Recommended algorithm: Boustrophedon Cellular Decomposition (BCD) aligned to the chosen stripe direction. Document option evaluation and stepwise implementation in [COVERAGE_PLANNER_P0_P1_PLAN.md](COVERAGE_PLANNER_P0_P1_PLAN.md).

**Acceptance.** `obstacle_map` runs with 0 unplanned turn gaps and a single connected base-link path. Whitburn Cres runs show `metrics.json.connector_path.connector_runs` reduced by at least half versus the pre-P1 baseline.

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
