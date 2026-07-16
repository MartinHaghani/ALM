# Current Rectangle Dry-Run Plan

Status: preparation implemented; the physical validation is still outstanding.
The continuous exporter now exists as `v2_continuous_dry_run.py`, and the static
PlanPath bridge and trace tooling are documented in this directory. The
authoritative remaining work item is
[issue #9](https://github.com/MartinHaghani/open_mower_ros/issues/9).

## Goal

Use the recorded `57 Whitburn Cres Back Yard Rectangle` map to get a simple V2-generated route driving cleanly on the mower before reintroducing complex lawn geometry, boundary-adjacent outlines, reverse maneuvers, or global route optimization.

## Immediate Test Route

- Keep the blade disabled.
- Use the V2 planner output, not the old slic3r planner.
- Export a mower-compatible `planpath_compat.json`.
- For this test, export one continuous path instead of many disconnected `paths[]` entries.
- Keep the route inside a conservative inset region.
- Generate sampled forward-only connectors between stripes so the old mower runtime does not invent unsafe first-point moves between segments.
- Do not rely on reverse, K-turns, wheel-anchor pivots, or blade phase metadata yet.

## Connector Strategy For The First Test

The eventual optimal connector is likely maneuver-aware:

- deep enough headland turns where possible;
- K-turn / 3-point turn when reverse is available;
- blade-off turns;
- explicit direction and turn phase metadata;
- route optimization that chooses stripe order to minimize travel.

For the immediate old-runtime test, use conservative forward-only connectors:

- stay inside an inset safe region;
- connect stripe endpoints with sampled transit geometry;
- prefer simple headland-shuttle movement through an interior lane over tight end-of-stripe turns;
- keep connector curvature gentle enough for FTC to follow;
- accept extra travel distance in exchange for avoiding boundary exits.

This is a diagnostic bridge, not the final mower planner.

## Why This Is Needed

The old mower runtime treats every `paths[]` entry as a separate MBF goal. Between entries it stops, rotates, and drives to the next first point using runtime navigation rather than V2-planned footprint-safe connectors. That caused the mower to leave the recorded boundary even when the V2 route samples were footprint-safe.

The next test should prove whether FTC can follow a single continuous V2 route before we add boundary outline passes and smarter turns.

## Next Milestones

1. Completed: build the continuous dry-run compatibility exporter.
2. Reproduce and review the exact inner-safe rectangle route and configuration for
   the next test.
3. Verify the temporary static PlanPath service and trace recorder without motion.
4. With explicit user approval, test blade-disabled from a physically safe starting
   pose under the controls in issue #9.
5. Attach the trace and decision to issue #9, then select connector, staging, FTC,
   or maneuver-contract follow-up work from evidence.
