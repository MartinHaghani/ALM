# Coverage Planner V2 Algorithm Plan

## Current implementation status

Verified from commit `a0b5896` and the current `tools/coverage_lab/` sources on
2026-07-15:

- M0 research and the initial algorithm/design documents are present.
- M1 task-evidence reporting is implemented in the lab.
- Substantial M2.x task-classification, local task-path, annotation-QA, portal,
  turnaround, and axis-scoring prototypes are implemented, but the M2 acceptance
  criteria have not been formally closed as a single reviewed milestone.
- M3 and later globally scored candidate generation, route optimization, and live
  maneuver-aware execution remain outstanding.

[Issue #8](https://github.com/MartinHaghani/open_mower_ros/issues/8) is the
authoritative current work item for reconciling M2 and advancing candidate routing.
The milestone descriptions below remain the durable design plan, not a live status
tracker.

Purpose: concrete implementation plan for the fresh V2 coverage planner. This plan turns the current zoning frustration into a planner architecture that can produce a safe, efficient, good-looking mower path from irregular recorded lawns.

This document is intentionally more specific than [COVERAGE_PLANNER_V2_DESIGN.md](COVERAGE_PLANNER_V2_DESIGN.md). The design document describes the broad architecture and live mower contract. This document describes the actual algorithm we should build and the evidence each step must produce.

Research sources are tracked in [COVERAGE_PLANNER_RESEARCH.md](COVERAGE_PLANNER_RESEARCH.md). The source IDs in this plan refer to that ledger.

## Core Decision

Do not make "zoning" the product.

The V2 planner should produce a route made of coverage tasks and executable maneuvers. Zones are only an internal explanation layer. A visually plausible colored map is not enough, and a visually wrong colored map is not automatically fatal if the final task route covers the lawn safely and efficiently.

The planner should instead build and score many possible ways to cover the lawn:

1. task candidates: main-body stripes, corridor passes, dead-end corridor passes, notch service passes, headland passes, obstacle-edge passes;
2. route candidates: possible orders, entry sides, exit sides, stripe directions, and turn maneuvers;
3. verified plan: a full `base_link` pose timeline with cutter coverage, footprint safety, wheel tracks, blade state, direction, and warnings.

This is the main answer to the current question: if the operator does not know the perfect zones or the perfect route, the planner must not ask for them. It must generate alternatives, simulate them, and choose the lowest-cost verified plan.

## What "Perfect" Means

No algorithm can guarantee a perfect-looking path for every arbitrary boundary, mower model, controller, weather condition, and operator preference. V2 should define perfection in a way that is testable:

- every accepted footprint sample is inside the allowed lawn and outside obstacles;
- every accepted live-export segment uses a maneuver the live mower can actually execute;
- cutter-swept coverage is maximized against an explicit target grid;
- unreachable, unsafe, or low-value patches are shown instead of hidden;
- total non-cutting travel, reverse travel, and turn complexity are minimized after safety and coverage;
- path aesthetics are scored, not guessed: long straight stripes are preferred in body areas, lengthwise passes are preferred in corridors, and notches are serviced with short in/out patterns.

The planner is done only when it can either produce a verified route or explain exactly why some part of the lawn cannot be covered with the configured mower.

## Why The Current Zoning Feels Wrong

The current V2 geometry report is useful, but it is solving the wrong intermediate problem too directly. It tries to color areas by local width slices. That can expose width variation, but it cannot decide the best mowing behavior.

Examples from the marked-up recorded map:

- The red and blue marked areas behave like corridors. They should be covered along their long axis, and they should connect to the main body through explicit portals.
- The yellow and purple marked areas behave like notches or alcoves. They are too small to deserve full stripe fields in both directions. They should usually be serviced by a short pop-in path, probably with blade off during reverse or turn phases depending on the live contract.
- The main green area is not a rectangle, but it should still be treated as one dominant body task unless a cut meaningfully improves coverage and route cost.

The important distinction is behavior, not color. A notch can be part of the same polygon as the body, but the route should service it differently.

## Research-Backed Direction

The useful sources point to a combined approach:

- Exact cellular decomposition such as Boustrophedon/Morse decomposition gives completeness and explains where sweep topology changes (`choset_1997_bcd`, `choset_2000_known_spaces`, `acar_2002_morse`).
- Skeleton and Voronoi-style representations are good at recognizing narrow structures, corridors, dead ends, and central paths (`acar_2002_sensor_based`, `skimage_medial_axis_docs`).
- Lawnmower-specific research treats decomposition angle, section merging, path preview, non-mowing distance, turns, and coverage percentage as first-class metrics (`shah_2025_lawn_cpp`).
- Edge mowing should be its own task because concave boundaries and obstacles leave missed boundary patches when treated as ordinary interior stripes (`tian_2022_edge_mowing`).
- Modular frameworks separate headlands, swaths, route planning, and path planning. V2 should keep that separation but add mower-scale maneuver semantics (`fields2cover_2022`, `fields2cover_route_docs`).
- Turn costs and partial coverage need explicit optimization, not after-the-fact patching (`krupke_2024_turn_costs`).

The result should be an ensemble planner: use several geometry methods to propose tasks, then let coverage simulation and route optimization decide.

## Planner Data Model

### `CoverageProblem`

Built from the recorded map and mower configuration.

Fields:

- `frame_id`
- `source_map_digest`
- `raw_lawn_polygon`
- `raw_obstacles`
- `conditioned_lawn_polygon`
- `drivable_region`
- `cutter_coverage_region`
- `headland_band`
- `mower_model`
- `operator_preferences`
- `coverage_grid`
- `warnings`

### `CoverageTask`

A thing that needs to be covered, not just a colored polygon.

Fields:

- `id`
- `kind`: `BODY`, `CORRIDOR`, `DEAD_END_CORRIDOR`, `NOTCH`, `HEADLAND`, `OBSTACLE_EDGE`, `TRANSIT_ONLY`, `UNREACHABLE`, `ARTIFACT`
- `geometry`
- `priority`
- `portals[]`
- `preferred_axes[]`
- `coverage_demand_cells[]`
- `classification_evidence`
- `candidate_generators[]`

### `Portal`

A connection point between tasks.

Fields:

- `id`
- `from_task_id`
- `to_task_id`
- `mouth_line`
- `center_pose_candidates[]`
- `width_m`
- `clearance_m`
- `supports_turnaround`
- `supports_reverse_entry`
- `evidence`

### `CoverageCandidate`

One possible way to cover one task.

Fields:

- `id`
- `task_id`
- `generator`: `BODY_STRIPES`, `CORRIDOR_LENGTHWISE`, `DEAD_END_IN_OUT`, `NOTCH_POP_IN`, `HEADLAND_LOOP`, `OBSTACLE_EDGE_LOOP`
- `entry_states[]`
- `exit_states[]`
- `segments[]`
- `expected_coverage_m2`
- `expected_overlap_m2`
- `unsupported_live_segments[]`
- `local_cost`
- `rejection_reasons[]`

### `MotionState`

An endpoint used by the route graph.

Fields:

- `base_link_pose`
- `yaw`
- `task_id`
- `candidate_id`
- `segment_id`
- `side`: `START`, `END`, `PORTAL`, `ANCHOR`
- `direction_capability`
- `blade_state_requirement`

### `MotionEdge`

A feasible or rejected movement between two states.

Fields:

- `from_state_id`
- `to_state_id`
- `segments[]`
- `feasible`
- `controller_mode`
- `cost`
- `min_clearance_m`
- `distance_m`
- `reverse_distance_m`
- `non_cutting_distance_m`
- `turn_count`
- `rejection_reasons[]`

### `CoveragePlan`

The selected result.

Fields:

- `problem_digest`
- `selected_tasks[]`
- `selected_candidates[]`
- `segments[]`
- `coverage_summary`
- `route_cost`
- `unsupported_for_live[]`
- `warnings[]`

## End-To-End Algorithm

### Stage 0: Mower Model And Objective Config

Before interpreting any map, load the mower model and scoring policy.

Required mower parameters:

- physical footprint polygon in `base_link`;
- cutter footprint or cutter swept disk/rectangle in `base_link`;
- wheel track and wheel contact points;
- minimum safe boundary clearance;
- live-supported maneuver modes;
- allowed reverse, pivot, and blade states for lab and live export;
- stripe spacing, overlap, and desired coverage target.

Required objective weights:

- missed required coverage;
- missed boundary/headland coverage;
- unsafe sample rejection;
- unsupported live maneuver rejection;
- non-cutting distance;
- total distance;
- reverse distance;
- pivot count;
- number of task entries/exits;
- stripe fragmentation;
- stripe angle preference;
- route disorder.

Hard constraints are enforced before weights. Unsafe is not a high-cost option; it is rejected.

### Stage 1: Map Intake And Validation

Inputs:

- recorded OpenMower `map.json`;
- optional no-go zones and operator annotations later.

Outputs:

- raw source polygons;
- validation warnings;
- digest for repeatability.

Steps:

1. Parse map rings into Shapely polygons.
2. Normalize ring orientation.
3. Detect self-intersections, repeated points, tiny spikes, and very short edges.
4. Keep the raw polygon unchanged for display.
5. Build a repaired candidate polygon only when repair is explicitly allowed.
6. Record every repair as a warning with before/after area.

Acceptance:

- invalid geometry is never silently accepted;
- raw and conditioned boundaries are both visible in the report;
- area changes from repair/simplification are quantified.

### Stage 2: Configuration-Space Geometry

The planner needs multiple regions, not one generic inset.

Regions:

- `conditioned_lawn`: cleaned map boundary at human/display scale;
- `base_link_drivable_all_yaw`: conservative region where the full footprint fits at any yaw;
- `base_link_drivable_by_yaw`: optional yaw-dependent feasibility for candidate simulation;
- `cutter_coverage_region`: where the cutter can sweep while the body stays safe;
- `headland_band`: area near the boundary that interior stripes may miss;
- `unreachable_or_unsupported`: geometries that cannot be covered under current safety and motion rules.

Important rule:

Do not conflate drivable area with mowable area. The body may need to stay farther from the boundary than the cutter, and yaw-dependent turns can need more space than straight swaths.

Acceptance:

- report area of every region;
- show missed coverage due to footprint clearance separately from missed coverage due to route choice;
- keep the configured 10 cm mowing boundary behavior visible and adjustable.

### Stage 3: Coverage Demand Grid

Build a grid of small cells over the conditioned lawn. This is the truth target for scoring.

Each cell stores:

- point or small polygon geometry;
- required/optional/forbidden coverage class;
- boundary-band flag;
- obstacle-edge flag;
- current coverage count;
- best covering segment id after simulation.

Default cell size should be smaller than cutter width, for example 5 cm to 10 cm in the lab. The route can be generated from polygons, but scoring should use the demand grid so small notches and boundary gaps are visible.

Acceptance:

- every uncovered patch has an area and a reason;
- coverage percentage is based on cutter-swept area, not centerline length;
- optional low-value patches can be reported without pretending they were cut.

### Stage 4: Geometry Evidence Layers

Generate several evidence layers before committing to tasks.

#### 4.1 Clearance Field

Sample the drivable region and compute clearance to the drivable boundary.

Use:

- local width estimates;
- narrowness detection;
- corridor/notch evidence;
- candidate skeleton weighting.

Do not use this alone for final zoning.

#### 4.2 Medial Axis / Skeleton Graph

Compute a medial-axis-like skeleton in the lab from the clearance field.

Convert it into a graph:

- nodes: branch points, endpoints, portal candidates;
- edges: centerline runs;
- edge attributes: length, mean width, min width, width profile, nearest boundary sides.

This is the main evidence for corridors and notches.

Interpretation:

- long skeleton edge with two meaningful portals: corridor;
- long skeleton edge with one portal and one dead end: dead-end corridor;
- short one-portal branch with small area/depth: notch or alcove;
- dense branch region with high width: body area.

#### 4.3 Exact Decomposition Candidates

Run Boustrophedon/Morse-style decompositions for multiple sweep axes.

Use them to find:

- topology-changing cuts;
- obstacle splits;
- candidate body stripe directions;
- decomposition angle alternatives.

Do not blindly accept every cell. Merge cells aggressively when the split does not improve route cost or coverage.

#### 4.4 Rectangle / Sector Cover Candidates

As an alternative to strict cellular decomposition, fit a small set of overlapping body sectors or approximate rectangles that support clean stripe paths.

This is useful for irregular but mostly open main bodies. It avoids over-splitting the lawn just because the boundary wiggles.

Acceptance:

- evidence layers are separately visible in HTML;
- every proposed task lists which evidence supported it;
- every rejected cut has a reason, not just absence from the final map.

### Stage 5: Task Extraction And Classification

Use evidence layers to create coverage tasks.

The planner should prefer behavioral classifications:

#### Body Task

A 2D area where ordinary stripe coverage makes sense.

Evidence:

- area above threshold;
- skeleton region has high clearance and branch density;
- decomposition cells merge without creating long unsupported connectors;
- at least two stripe orientations cover it efficiently.

Generator:

- multi-angle stripe candidates;
- optional headland interaction;
- route optimizer chooses stripe order and direction.

#### Corridor Task

A long narrow region that should be mowed along its long axis.

Evidence:

- skeleton edge length is several times local width;
- width is within one to a few mower/tool widths;
- two portals or a clear through-connection;
- stripe-across would create many short stripes and many turns.

Generator:

- one or more lengthwise passes;
- optional out-and-back if width needs two passes;
- entry/exit states at portals;
- turnarounds only where the geometry supports them.

#### Dead-End Corridor Task

A corridor with one portal and a dead end.

Evidence:

- skeleton branch has one portal and one terminal endpoint;
- depth is longer than a notch threshold;
- width supports lengthwise travel but not easy cross-striping.

Generator:

- drive in, cover lengthwise, reverse or turn only if supported;
- possible blade-on forward, blade-off reverse depending on policy;
- terminal turnaround only if verified safe.

#### Notch / Alcove Task

A small side feature that should not become its own full stripe field.

Evidence:

- one portal to a body/corridor;
- small area relative to mower and stripe spacing;
- short depth or both dimensions too small for useful internal striping;
- skeleton branch terminates quickly;
- no feasible local turnaround.

Generator:

- pop-in service path;
- one or two cutter-centered passes;
- reverse-out or back-out maneuver in lab metadata;
- optional live export rejection if reverse is not supported.

#### Headland / Boundary Task

The boundary band that ordinary interior stripes miss.

Evidence:

- cells within the boundary band;
- concave corners;
- obstacle boundaries.

Generator:

- one or more perimeter passes;
- special concave-corner service passes;
- separate obstacle-edge loops where needed.

#### Artifact / Unreachable Task

A patch that should not drive the main route crazy.

Evidence:

- below area threshold;
- cannot fit footprint;
- no safe portal;
- would require unsupported live maneuvers;
- has low coverage value compared with added risk/cost.

Generator:

- none by default;
- visible report with area, reason, and possible operator action.

Acceptance:

- the red/blue-style side structures on the recorded map should classify as corridor or dead-end corridor tasks, not arbitrary body slivers;
- the yellow/purple-style small alcoves should classify as notch tasks if they lack enough room for a real internal stripe pattern;
- a useless sliver split from the main body should be rejected or merged unless it changes the selected route cost meaningfully.

### Stage 6: Task Boundary And Portal Selection

This is the replacement for hand-drawn zone boundaries.

For each candidate corridor or notch branch:

1. locate its mouth where clearance changes from body-like to branch-like;
2. propose several mouth cuts perpendicular to the skeleton edge;
3. evaluate each cut for:
   - width;
   - stability along nearby samples;
   - area on both sides;
   - whether it isolates a meaningful task;
   - whether it creates feasible entry and exit states;
   - whether it reduces route cost.
4. choose the cut only if it improves the final plan score or produces a necessary task type.

Key rule:

A cut is not good because it looks clean. A cut is good if it creates a better coverage task and a better route.

Acceptance:

- HTML shows mouth candidates, accepted portals, and rejected portal reasons;
- final task boundaries can differ from preliminary visual zones;
- every task has at least one portal unless it is a separate component or unreachable.

### Stage 7: Coverage Candidate Generation

For every task, generate several ways to cover it.

#### Body Candidate Generator

Inputs:

- task polygon;
- preferred axes from decomposition, minimum rotated rectangle, and operator preference;
- cutter width and overlap.

Generate:

- stripe sets at candidate angles;
- stripe sets anchored to reduce boundary slivers;
- stripe sets that align with adjacent corridor portals where useful;
- route variants with reversed stripe order.

Reject:

- stripe sets with excessive tiny fragments;
- stripe sets whose endpoints cannot be connected by feasible turns/transits;
- stripe sets that leave large uncovered patches in high-priority demand cells.

#### Corridor Candidate Generator

Inputs:

- skeleton edge;
- width profile;
- portals.

Generate:

- centerline pass;
- offset passes if width needs more than one cut;
- through-corridor variants from either portal;
- dead-end in/out variants if only one portal exists.

Reject:

- across-corridor striping when it creates excessive turns;
- terminal turnarounds that fail footprint simulation.

#### Notch Candidate Generator

Inputs:

- notch task geometry;
- portal pose candidates;
- local centerline/depth direction.

Generate:

- pop-in forward and reverse-out;
- forward-only partial service if reverse is disabled;
- cutter-offset pass that covers the widest part of the notch;
- optional "do not service live" candidate with reported missed area.

Reject:

- full stripe field if both dimensions are too small;
- local turnarounds that do not fit;
- unsupported reverse in live export.

#### Headland Candidate Generator

Inputs:

- conditioned boundary;
- obstacle boundaries;
- headland band demand cells.

Generate:

- perimeter loops;
- inner/outer headland pass variants;
- concave corner service maneuvers;
- obstacle-edge loops.

Reject:

- passes whose footprint or safety margin crosses forbidden boundary;
- loops with unreachable gaps unless explicitly split and reported.

Acceptance:

- every candidate carries expected coverage and endpoint states;
- the report can compare at least the top few candidates per task;
- candidate generation does not commit to route order.

### Stage 8: Motion Primitive Library

The motion primitive library answers one question: can the mower safely move from state A to state B?

Primitive families:

- straight cut segment;
- forward stripe-to-stripe turn;
- wheel-anchor turn;
- three-point turn;
- omega/keyhole turn;
- reverse align;
- in-place pivot, lab-only until runtime support exists;
- headland-following transit;
- safe non-cutting transit between task portals;
- recovery or split-gap marker when nothing safe exists.

Each primitive returns:

- sampled `base_link` poses;
- direction per sample or segment;
- blade state;
- controller mode;
- wheel tracks;
- swept footprint;
- swept cutter;
- min clearance;
- distance and reverse distance;
- rejection reasons.

Important rule:

Turn planning should not happen after the stripe route is fixed. The route optimizer must know turn feasibility and cost while choosing stripe order, skipped stripes, and task order.

Acceptance:

- unsafe primitives are rejected before scoring;
- unsupported live primitives block live export;
- lab HTML shows the same path color/style for route geometry, with metadata shown in details layers.

### Stage 9: Endpoint Graph And Route Optimization

Build a graph whose nodes are candidate entry/exit states and whose edges are motion primitives.

For each `CoverageCandidate`:

- represent possible entry and exit states;
- include internal segment order variants;
- include candidate coverage value and local cost.

For each pair of compatible states:

- ask the motion primitive library for feasible edges;
- store the best few edges, not just the shortest one;
- include headland transits as possible connectors.

Solve in layers:

1. Select required tasks.
2. Pick one candidate per required task.
3. Decide task order.
4. Decide entry and exit state for each task.
5. Insert feasible motion edges.
6. Optionally include or drop low-priority tasks through prize-collecting scoring.

Prototype solver path:

- start with exhaustive search or dynamic programming for small task counts;
- move to beam search when candidates multiply;
- use NetworkX for graph sanity checks;
- consider OR-Tools once the cost model and dropped-task behavior are stable.

Cost model:

Hard rejections:

- unsafe footprint;
- obstacle intersection;
- disconnected route;
- unsupported maneuver in live export;
- missed required task when no allowed fallback exists.

Primary score:

- uncovered required area;
- uncovered boundary/headland area;
- unsupported/lab-only segment penalty;
- non-cutting distance;
- total distance;
- reverse distance;
- turn count and pivot count;
- number of task boundary crossings;
- stripe fragmentation.

Secondary score:

- visual stripe continuity;
- fewer direction changes in the main body;
- stable repeated pattern across sessions;
- preference for one main stripe angle unless route cost says otherwise.

Acceptance:

- if the main body uses horizontal stripes and corridors use vertical/lengthwise passes, the optimizer decides where to enter each corridor by endpoint cost;
- it should not need to "navigate around" blindly because every portal-to-portal move is costed before selection;
- if a notch is expensive and low value, the plan either services it with a clear pop-in pattern or reports it as skipped with measured missed area.

### Stage 10: Plan Simulation And Verification

Simulate the selected plan with the same geometric checks used during planning.

Outputs:

- swept cutter coverage grid;
- swept footprint samples;
- wheel tracks;
- uncovered patches;
- overlap patches;
- blade-on/off timeline;
- direction timeline;
- unsupported live segments;
- route metrics.

Metrics:

- required coverage percent;
- optional coverage percent;
- boundary-band coverage percent;
- non-cutting distance;
- total distance;
- reverse distance;
- turn count by primitive;
- task count by kind;
- dropped task count and area;
- unsafe sample count;
- unsupported-live segment count.

Acceptance:

- simulation is the source of truth for reported coverage;
- the HTML report should let us inspect why any patch was missed;
- candidate score and final metrics should be close enough that optimizer choices are explainable.

### Stage 11: Output Artifacts

Required lab outputs:

- `coverage_problem_v2.json`: conditioned geometry, demand grid summary, evidence layers;
- `coverage_tasks_v2.json`: tasks, portals, classifications, candidate summaries;
- `coverage_plan_v2.json`: full selected maneuver-aware plan;
- `simulation_preview.json`: sampled timeline for HTML/simulation;
- `v2_plan.html`: visual report with task, route, wheel, coverage, and warnings layers;
- `v2_metrics.json`: numerical metrics and comparison to old planner;
- `planpath_compat.json`: only if explicitly requested, lossy and forward-only compatible.

HTML layers:

- raw boundary;
- conditioned boundary;
- drivable region;
- cutter coverage region;
- demand grid coverage/missed patches;
- skeleton graph;
- exact decomposition cuts;
- rectangle/sector candidates;
- tasks by kind;
- portals and mouth candidates;
- selected route;
- rejected route gaps;
- wheel tracks;
- swept footprint;
- swept cutter;
- blade state and direction details.

Acceptance:

- the operator can start visually, then inspect details only after seeing the map;
- visual layers explain classification, route order, missed patches, and maneuver feasibility;
- old mower-compatible output never hides missing reverse/pivot/blade semantics.

## Handling The Marked Map

The recorded map should be used as the first serious regression case.

Expected task behavior:

- large central green area: one main `BODY` task unless evidence proves a route benefit from splitting;
- left tall red side branch: likely `CORRIDOR` or `DEAD_END_CORRIDOR`, covered lengthwise;
- upper-right blue side branch: likely `DEAD_END_CORRIDOR`, covered lengthwise with explicit entry/exit behavior;
- yellow small top notch: likely `NOTCH`, serviced by a pop-in path or reported if live reverse is unsupported;
- lower-left purple notch: likely `NOTCH` or `DEAD_END_CORRIDOR` depending on measured depth and width;
- tiny boundary wrinkles: `ARTIFACT` or merged into adjacent body/headland, not separate zones.

The desired final route is not manually specified. The route optimizer should determine:

- whether to do headland first or last;
- which side of the main body to start from;
- whether main-body stripes should be horizontal, slightly rotated, or split into two stripe sets;
- when to enter each corridor;
- whether a corridor should be covered before or after nearby body stripes;
- whether a notch is worth servicing live or should be a lab-only reverse task;
- how to minimize portal-to-portal transit.

## Implementation Milestones

### M0: Research And Algorithm Plan

Status: this document plus [COVERAGE_PLANNER_RESEARCH.md](COVERAGE_PLANNER_RESEARCH.md).

Deliverables:

- durable source ledger;
- V2 algorithm plan;
- doc links from the documentation index and V2 design.

### M1: Evidence Report Upgrade

Goal: replace "preliminary zones" as the main diagnostic with task evidence.

Implement:

- skeleton graph extraction from the clearance field;
- branch metrics: length, width profile, endpoint kind, portal candidates;
- exact decomposition candidates for several sweep axes;
- mouth/portal candidate detection;
- task proposal table with evidence and rejection reasons.

Do not:

- generate route yet;
- remove existing preliminary zones until the task report is better.

Acceptance:

- marked red/blue/yellow/purple regions are visible as branch/task proposals even if final classification is still preliminary;
- useless slivers are marked as low-value cuts, not promoted to meaningful tasks.

### M2: Task Classifier

Goal: classify coverage tasks by behavior.

Implement:

- `BODY`, `CORRIDOR`, `DEAD_END_CORRIDOR`, `NOTCH`, `HEADLAND`, `OBSTACLE_EDGE`, `UNREACHABLE`, `ARTIFACT`;
- portal selection with accepted/rejected mouth cuts;
- task metrics and confidence scores;
- operator-facing HTML task layer.

Acceptance:

- the recorded map reports the expected corridor/notch candidates without requiring manual drawing;
- every task classification has an explanation tied to width, skeleton, portal, area, and route implications.

### M3: Coverage Candidate Generators

Goal: generate possible ways to mow each task.

Implement:

- body stripe candidates at multiple axes;
- corridor lengthwise candidates;
- dead-end corridor in/out candidates;
- notch pop-in candidates;
- headland and obstacle-edge candidates;
- expected coverage scoring against the demand grid.

Acceptance:

- each task has multiple candidates or an explicit reason it cannot;
- notch candidates do not become tiny decorative stripe fields;
- corridor candidates run lengthwise by default.

### M4: Motion Primitive Costing

Goal: make route choices aware of physical mower motion.

Implement:

- primitive library interface;
- primitive sampling and safety checks;
- costed edges between candidate endpoints;
- lab-only reverse/pivot metadata;
- live-supported filtering.

Acceptance:

- impossible turns are rejected before route selection;
- route preview shows blade, direction, wheel tracks, and footprint envelopes.

### M5: Route Optimizer

Goal: choose the full plan.

Implement:

- endpoint graph builder;
- small exact/DP solver;
- beam search fallback for larger candidate sets;
- optional dropped-task/prize scoring for low-value notches;
- old-planner baseline comparison.

Acceptance:

- selected plan has no hidden connector gaps;
- task order is explainable by endpoint edge costs;
- recorded map improves over the current planner in coverage and non-cutting distance, or explicitly shows which unsupported maneuvers block improvement.

### M6: Simulation And Regression Suite

Goal: make changes safe to iterate on.

Implement:

- final plan simulation;
- missed/overlap coverage overlays;
- regression baselines for recorded map and synthetic maps;
- edge-case fixture maps for natural lawn cases.

Acceptance:

- no change can silently reduce coverage or add unsafe samples;
- every known natural-lawn edge case has a fixture or a deferred-case marker with a reason.

### M7: Live Mower Contract

Goal: move only proven pieces toward runtime.

Implement:

- forward-only live adapter first;
- maneuver-aware ROS message or action design;
- blade-state segment control;
- reverse and pivot only after low-speed dry-run validation.

Acceptance:

- live export never flattens reverse/pivot/blade-off semantics into a plain path;
- unsupported lab maneuvers block live export with clear warnings.

## Edge Case Checklist

Each case should eventually have a fixture map and expected planner behavior.

| Case | Expected V2 behavior |
|---|---|
| Tiny boundary wrinkle | Smooth or mark artifact; do not create a task. |
| Small notch | `NOTCH` task; pop-in service or reported skip. |
| Long side strip | `CORRIDOR` or `DEAD_END_CORRIDOR`; mow lengthwise. |
| Wide irregular main body | `BODY`; stripe candidates at several angles; merge minor slivers. |
| Narrow neck between two bodies | Portal/corridor task; avoid unsafe cross-body connectors. |
| Obstacle in body | Exact decomposition around obstacle plus obstacle-edge task. |
| Obstacle near boundary | Report collapsed/drivable gap; do not hide unsafe pass. |
| Concave corner | Headland/corner service task; footprint-swept verification. |
| Acute corner | Possible unreachable/unsupported task unless a pivot service is safe. |
| Annular lawn | Body plus obstacle-edge/headland loops; route through portals. |
| Multiple components | Separate tasks/components; no cross-lawn connector unless configured. |
| Curved boundary | Headland follows boundary; body stripes use dominant axis only where useful. |
| Low-value unreachable patch | Show missed area and reason; optional task only if operator opts in. |
| Live reverse disabled | Lab may plan reverse; live export rejects or substitutes supported maneuver. |

## Immediate next step

Resume from [issue #8](https://github.com/MartinHaghani/open_mower_ros/issues/8),
not the original M1 bootstrap list. First reconcile the implemented M2.x behavior
against the M2 acceptance criteria; then generate multiple behavior-specific M3
coverage candidates with explicit safety, missed-area, and unsupported-maneuver
reasons. Do not promote the lossy compatibility export to live execution while it
cannot preserve reverse, pivot, or blade-state semantics.
