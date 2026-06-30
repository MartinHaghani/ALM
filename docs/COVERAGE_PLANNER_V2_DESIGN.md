# Coverage Planner V2 Design

Purpose: working design document for a fresh mower-first coverage planner. Keep the existing slic3r runtime planner and the current `tools/coverage_lab` Fields2Cover lab intact while this design matures.

This document is intentionally a living plan. It should capture what the planner needs to do, what path shape it should produce, how the mower should consume it, and which parts must stay lab-only until the runtime contract is explicit and tested.

## Why Start Fresh

The current lab is valuable because it exposes safety and coverage problems, but the recorded mower map shows the architectural limit clearly: a single obstacle-free 71.88 m2 lawn was decomposed into 13 cells, produced 122.8 m of connector travel, and covered only 61.1% of the mowable area. That is a planner strategy problem, not just a turn-shape problem.

The old planner should remain available as:

- a runtime fallback until V2 is proven;
- a visual and metrics baseline in the lab;
- a source of reusable mower geometry and preview code.

V2 should not be a pile of local fixes on top of the current cell and turn logic. It should plan at mower scale from the beginning.

## Near-Term Decision

V2 should assume from the beginning that the mower will eventually need:

- reverse maneuver phases;
- pivot-wheel turns;
- blade-off turn and transit phases;
- explicit alignment segments;
- per-segment controller/motion mode.

Those semantics should be present in the V2 internal plan and JSON artifacts early, even before the live mower can execute every segment type. They should not be implemented later as hidden meanings inside `nav_msgs/Path`.

The implementation order is different from the design contract:

1. Start with **map conditioning and macro-zone classification**.
2. Represent all future route output as maneuver-aware `CoverageSegment` records, even if the first prototype emits only geometry overlays and simple forward segments.
3. Add cutter-swept scoring and endpoint routing in the lab.
4. Add reverse, pivot, blade-off, and wheel-anchor maneuver candidates in the lab.
5. Only then redesign live mower execution so `MowingBehavior` or a new coverage executor can honor those segment semantics.

This keeps the first coding step focused on the current root problem, while preventing the fresh planner from inheriting the old path-only contract.

## Core Goals

V2 must produce a plan that is:

- **Safe by construction:** every accepted physical footprint sample stays inside the mowable lawn and outside obstacles with configured clearance.
- **Coverage-first:** optimize actual cutter swept area, not just path length or stripe count.
- **Mower-feasible:** turns, reverse moves, pivots, and transits must be represented as executable maneuvers, not hidden inside a plain centerline.
- **Readable:** produce lawn-quality stripes where geometry allows, with explicit warnings where geometry does not.
- **Inspectable:** every loss of coverage, failed maneuver, long transit, or safety rejection should be visible in HTML and metrics.
- **Runtime-honest:** lab output may be richer than live mower output, but the live adapter must never silently drop direction, blade, pivot, or safety semantics.

## Where To Start

Start with the recorded mower `map.json` under `tools/coverage_lab/data/maps/current/map.json`, plus the existing synthetic examples. The first V2 work should be a lab-only geometry and strategy prototype, not runtime integration.

The first milestone should not generate mower-ready paths. It should generate a **coverage problem model**:

1. Load the OpenMower map.
2. Build a mower-scale cleaned lawn polygon.
3. Build a footprint-safe drivable region.
4. Classify the shape into macro-zones, corridors, pockets, and unreachable notches.
5. Render the classified geometry in HTML.
6. Report how much area was kept, simplified, marked unreachable, or reserved for headland coverage.

This is the right starting point because the current failure begins before turn planning: the raw recorded boundary has too many small wrinkles, and the current decomposition treats many of them as planning-significant.

The research ledger lives in [COVERAGE_PLANNER_RESEARCH.md](COVERAGE_PLANNER_RESEARCH.md). The full V2 algorithm plan lives in [COVERAGE_PLANNER_V2_ALGORITHM_PLAN.md](COVERAGE_PLANNER_V2_ALGORITHM_PLAN.md). The original step-by-step implementation plan for the first geometry milestone lives in [COVERAGE_PLANNER_V2_PROTOTYPE_PLAN.md](COVERAGE_PLANNER_V2_PROTOTYPE_PLAN.md).

## Coordinate And Frame Conventions

The planner should preserve the current mower convention:

- Map frame: `map`.
- Executable body frame pose: `base_link`.
- On the current Mowrator model, `base_link` is the rear-center of the physical footprint.
- Default footprint: `[[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]]`.
- Cutter/tool center: `[0.41, 0.0]` relative to `base_link`.
- Wheel contact points are derived from `base_link`, wheel track, and wheel contact x offset.

The authoritative executable geometry should be a **base_link pose timeline**. Wheel paths and cutter paths should be derived from that timeline for validation, scoring, preview, and maneuver metadata.

For the perimeter outline, this means the planner should constrain the mower footprint directly instead of drawing a cutter-center or centerline inset first. The default V2 outline mode is `footprint_side_fit`: orient the boundary so it lies on the mower's right, place the executable `base_link` pose so the right footprint side has the requested clearance on straight sections, solve each corner from the rotated footprint's support against the adjacent boundary half-planes, and accept only poses whose full footprint remains inside the configured safety region. This keeps the right side close to the boundary without pretending the mower is a point or a circular disk while turning.

Do not make separate left/right wheel paths the primary planner output. The mower controller consumes body motion commands, and a differential-drive body pose plus direction is the natural execution contract. Wheel paths are still essential, but as constraints and diagnostics:

- they validate pivot anchors and reverse geometry;
- they show whether a turn is physically plausible;
- they provide preview layers;
- they can be used by a future low-level controller, but should not replace the body pose plan.

## Current Runtime Consumption

Today, `mower_logic` calls:

```text
slic3r_coverage_planner/plan_path -> slic3r_coverage_planner/PlanPath
```

The response shape is:

```text
Path[] paths

Path:
  uint8 is_outline
  nav_msgs/Path path
```

`MowingBehavior` then executes each path segment like this:

1. Send the first pose to Move Base Flex as `MoveBaseGoal` with `controller = "FTCPlanner"`.
2. After reaching that first pose, enable mowing.
3. Send the remaining poses as MBF `ExePathGoal` with `controller = "FTCPlanner"`.
4. Track progress through `/move_base_flex/FTCPlanner/planner_get_progress`.
5. Disable mowing only when the segment ends, pauses, aborts, or errors.

Current implications:

- The first-pose approach is not part of the coverage path.
- Blade state is effectively per executed path, not per maneuver.
- Direction is not encoded in `PlanPath`.
- Reverse is blocked in normal mowing because `FTCPlanner.forward_only` is `true`.
- Same-position yaw-only pivots are not reliable because `FTCPlanner` skips duplicate-position points while following.
- `FTCPlanner` is a follow-the-carrot controller, not a geometric turn executor.

This means `nav_msgs/Path` is not enough for V2 if V2 includes reverse, blade-off turns, pivot-wheel turns, or distinct transit segments.

## V2 Path Contract

Internally, V2 should produce a maneuver-aware plan. A sketch:

```text
CoveragePlan
  frame_id: map
  mower_model
  source_map_digest
  zones[]
  segments[]
  warnings[]
  metrics

CoverageSegment
  id
  zone_id
  kind: HEADLAND | SWATH | TURN | TRANSIT | ALIGN | PIVOT | RECOVERY
  purpose: CUT | MOVE | POSITION | VALIDATE_ONLY
  blade_state: ON | OFF
  direction: FORWARD | BACKWARD | ROTATE | MIXED
  controller_mode: FOLLOW_PATH | POINT_TURN | WHEEL_ANCHOR | OPEN_LOOP_TEST | UNSUPPORTED
  base_link_poses[]
  target_speed_mps
  max_angular_speed_radps
  clearance_m
  coverage_expected_m2
  maneuver_metadata
```

`base_link_poses[]` should remain the primary geometry. The segment metadata should carry anything that cannot be inferred safely from poses:

- whether the blade is enabled;
- whether motion is forward, backward, rotate-only, or mixed;
- whether a wheel is intended to remain fixed during a pivot;
- left/right wheel anchor points;
- final target wheel error;
- minimum clearance;
- whether the segment is allowed on the live mower.

For compatibility, V2 can also export a flattened `planpath_compat.json`, but that file must be treated as lossy. It should be used only for old-tool visualization or forward-only live experiments where unsupported maneuver segments have already been removed or replaced.

## Best Architecture

Use layered modules with explicit artifacts between them.

### 1. Map Intake

Inputs:

- OpenMower `map.json`;
- mower model config;
- optional no-go zones, soft zones, and operator preferences later.

Outputs:

- raw lawn polygons;
- raw obstacle polygons;
- source-map snapshot and digest;
- validation warnings.

Responsibilities:

- validate rings and repair only when explicitly requested;
- preserve source geometry for debugging;
- reject impossible map shapes early.

### 2. Geometry Conditioning

Inputs:

- raw polygons;
- mower footprint;
- cutter geometry;
- safety margin.

Outputs:

- cleaned coverage lawn;
- footprint-safe drivable region;
- simplified display boundary;
- unreachable or too-small pockets;
- boundary features classified by mower scale.

Responsibilities:

- remove or smooth boundary wrinkles below mower scale;
- preserve real narrow corridors;
- identify places where the footprint cannot physically fit;
- compute headland-safe offsets using footprint geometry, not just cutter width.

Recommended tools:

- Shapely in the Python lab;
- Clipper2 or GEOS-backed C++ only when moving toward runtime.

### 3. Macro-Zone Strategy

Inputs:

- conditioned lawn and drivable region.

Outputs:

- macro-zones: main bodies, corridors, appendages, pockets;
- zone adjacency graph;
- candidate stripe axes per zone;
- zone-level coverage priorities.

Responsibilities:

- avoid creating a new cell for every small boundary wiggle;
- detect long narrow corridors and handle them differently from wide open lawn;
- decide whether a pocket should be striped, perimeter-cut, or reported as low-value/unreachable.

This layer replaces the current over-eager decomposition. The recorded map should probably become a small number of meaningful zones, not 13 tiny cells.

### 4. Coverage Candidate Generation

Inputs:

- macro-zones;
- cutter width;
- overlap settings;
- operator stripe preference.

Outputs:

- candidate swath sets per zone;
- headland candidates;
- local coverage score for each candidate.

Responsibilities:

- generate stripes from a small set of candidate angles;
- support corridor-specific centerline or lengthwise passes;
- generate headland passes that intentionally cover the boundary band;
- estimate cutter swept area before routing.

Fields2Cover can remain a candidate generator, but should not own the full route strategy. A simple custom stripe generator is also worth building because the mower problem is polygonal and known-map, not full agricultural implement planning.

### 5. Motion Primitive Library

Inputs:

- start/end `base_link` poses;
- local drivable polygon;
- mower wheel and footprint model.

Outputs:

- feasible maneuver candidates with metadata;
- rejection reasons when no candidate exists.

Primitive families:

- adjacent-stripe wheel-anchor turn;
- forward U-turn;
- omega/keyhole turn;
- three-point turn;
- in-place pivot, lab-only until runtime supports it;
- reverse settle/align;
- footprint-safe transit connector.

The motion primitive library should be independent of the stripe generator. It answers: "Can the mower move from pose A to pose B safely, and what would that cost?"

### 6. Endpoint Graph And Routing

Inputs:

- swath candidates;
- headland candidates;
- zone graph;
- motion primitive costs.

Outputs:

- selected zone order;
- selected swath directions;
- selected turn/transit maneuvers;
- complete segment list.

This is where V2 should become smarter than the current planner. Instead of generating stripes and then trying to patch turns afterward, build a graph:

- each swath has two possible endpoint directions;
- adjacent swaths have turn edges;
- non-adjacent swaths or zones have transit edges;
- edges carry cost and feasibility.

Optimize the route using a practical solver. OR-Tools is a strong candidate for ordering, with custom edge costs from the motion primitive library.

Primary costs:

- reject unsafe footprints first;
- maximize cutter coverage;
- minimize unsupported maneuvers;
- minimize total distance;
- minimize non-cutting transit;
- minimize reverse distance;
- minimize visible stripe disorder;
- prefer clean headland-to-swath transitions.

### 7. Safety And Coverage Simulation

Inputs:

- full maneuver-aware plan.

Outputs:

- accepted/rejected samples;
- swept cutter coverage;
- swept footprint envelope;
- wheel tracks;
- warnings and metrics.

Responsibilities:

- resample all segments at a fixed spatial/yaw resolution;
- compute physical footprint and safety-margin footprint;
- compute actual cutter swept area;
- flag but do not hide unsafe, unsupported, or unplanned gaps;
- generate regression metrics.

This layer should be used during planning, not just after planning. The route optimizer should be able to ask for the real cost of a candidate.

### 8. Output Adapters

V2 should have multiple outputs:

- `coverage_plan_v2.json`: full maneuver-aware plan.
- `simulation_preview.json`: flattened preview timeline with metadata.
- `plan.html`: interactive report.
- `lawn_preview.png`: static grass-finish preview.
- `planpath_compat.json`: lossy compatibility output for old tooling only.
- Future ROS service/action messages for live execution.

The adapters should be thin. The planner core should not silently change behavior depending on output format.

## Live Mower Redesign Direction

The live mower should eventually consume a maneuver-aware contract, not a plain `nav_msgs/Path`.

Recommended migration path:

1. **Lab only:** V2 produces `coverage_plan_v2.json` and HTML. No mower runtime changes.
2. **Forward-only live adapter:** export only `HEADLAND`, `SWATH`, and safe forward `TRANSIT` segments to the old `PlanPath` shape. Drop or split unsupported reverse/pivot segments with visible warnings.
3. **Maneuver-aware service:** add a new planner service or action, for example `mower_msgs/PlanCoverage` or a new planner package message, with `segments[]`.
4. **Maneuver-aware executor:** update `MowingBehavior` or add a dedicated coverage executor that:
   - drives to segment starts with blade off;
   - toggles blade per segment;
   - selects the appropriate controller mode;
   - enforces direction;
   - pauses/retries per segment kind;
   - records progress by segment id and pose index.
5. **Controller support:** extend FTC or add a new controller for reverse and pivot-aware maneuvers. Do not rely on duplicate-position points or hidden yaw changes.

The safest first live contract is conservative:

- forward-only;
- blade on only for `SWATH` and approved `HEADLAND`;
- blade off for `TURN`, `TRANSIT`, `ALIGN`, `PIVOT`, and `RECOVERY`;
- unsupported segments block live export instead of being flattened.

Reverse and pivot support should come later through explicit segment types and low-speed dry-run validation.

## What The Planner Should Optimize

Hard constraints:

- physical footprint inside lawn;
- safety-margin footprint inside allowed region;
- no obstacle intersections;
- no unsupported live maneuvers in live export;
- segment endpoints connected unless the plan explicitly reports a gap.

Primary objectives:

- maximize cutter swept coverage;
- minimize missed boundary band;
- minimize non-cutting travel;
- minimize total route length;
- minimize reverse distance and pivot count;
- minimize stripe fragmentation;
- keep stripe direction visually consistent where possible.

Secondary objectives:

- prefer simple turns over complex turns;
- prefer fewer zones when coverage and safety are equal;
- prefer predictable repeated mowing patterns;
- support alternating stripe angle across sessions later.

## Questions To Keep Open

- What minimum coverage target is acceptable for awkward recorded maps before runtime testing? 95% is a useful target, but some tiny notches may be better reported as unreachable.
- How much boundary simplification is acceptable before the operator feels the mower is ignoring the recorded outline?
- Should V2 support operator-marked "do not care" pockets for rough edges under shrubs or fences?
- Should the first live executor remain MBF-based, or should coverage execution become its own action server that directly owns segment state and blade control?
- Should reverse be supported by extending `FTCPlanner`, adding a second controller, or using a low-speed trajectory follower for maneuver segments?
- How should progress checkpoints survive a plan re-run if the new planner has stable segment ids but different sampled poses?

## Initial Implementation Plan

Keep this first implementation lab-only.

1. Create a V2 prototype module under `tools/coverage_lab`, separate from the current planner path.
2. Add a `plan-v2` or config-gated V2 command that reads the same maps and writes a separate run directory.
3. Implement geometry conditioning and macro-zone classification first.
4. Render V2 geometry overlays before generating any route.
5. Add a simple stripe generator for one macro-zone.
6. Add cutter-swept coverage scoring.
7. Add motion primitive feasibility checks.
8. Add endpoint graph routing for one zone.
9. Expand to multiple zones.
10. Only after the lab results are clearly better than the old planner, design ROS messages and live execution changes.

The first acceptance target should be modest and objective:

- recorded current map produces fewer than 6 macro-zones unless the geometry truly requires more;
- no unsafe footprint samples in conditioned drivable region checks;
- uncovered/unreachable area is visible and quantified;
- initial route covers more area than the current 61.1% baseline without adding hidden unsafe connectors;
- every unsupported live maneuver is marked as unsupported rather than flattened.
