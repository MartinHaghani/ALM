# FTC Executor And Controller Limitations

Purpose: document why manual teach-path recordings must preserve richer evidence than the current `PlanPath`/FTC executor can consume.

## Current Execution Contract

The current mowing executor is built around these observed pieces:

- `slic3r_coverage_planner/PlanPath` produces paths that are consumed by `MowingBehavior`.
- `MowingBehavior` sends `mbf_msgs::MoveBaseGoal` and `mbf_msgs::ExePathGoal` to MBF with controller `FTCPlanner`.
- The executable path handed to MBF is a `nav_msgs/Path`, which is only a list of stamped poses.
- `src/open_mower/params/ftc_local_planner.yaml` sets `forward_only: true`, `max_lateral_follow_error: 0.25`, `max_longitudinal_follow_error: 1.0`, `max_follow_distance: 0.5`, and a maximum control-point lead of `0.35`.
- `MowingBehavior` already treats FTC completion with suspicion: it verifies first-point success against the live pose and pauses for inspection when mow-path progress is not actually near the end.

This is a workable legacy path-following interface. It is not a full maneuver or teach-repeat interface.

## What FTC Cannot Express

`PlanPath` and `nav_msgs/Path` cannot represent:

- reverse motion;
- deliberate yaw-only pivots;
- blade-off turns;
- pauses and dwell times;
- operator marker events;
- maneuver kinds such as lane mowing, turn, reposition, reverse, pivot, overlap correction, or recovery;
- wheel-anchor semantics, such as "hold one wheel nearly fixed while sweeping the deck";
- source provenance, such as GPS pose, aligned LIDAR pose, fused pose, or which source was trusted;
- quality gates, such as RTK state, LIDAR alignment state, fused readiness, pose age, or GPS/LIDAR separation;
- command evidence, such as requested twist versus measured twist.

Any conversion from a manual teach path into old `PlanPath` format therefore loses information. That loss is acceptable only for a conservative diagnostic compatibility export.

## Human Steering Choices FTC Does Not Have

A human operator can make small corrective choices that are visible in a recording but missing from the old executor contract:

- feather speed before a tight heading correction;
- slow down before entering a low-confidence or narrow area;
- pause briefly while the mower settles or the localization estimate stabilizes;
- steer slightly wide to maintain deck overlap;
- bias a turn around the rear axle or one wheel;
- choose a shallow S-curve instead of a point-to-point correction;
- keep mowing through tiny GPS wobble when visual context says the mower is still on track;
- stop the blade during repositioning and resume it on the next clean pass.

These are not all "better controls." Some are human context and judgement. The recorder should preserve them as diagnostic truth, while any live repeat system should make explicit decisions about which ones can be reproduced safely.

## Known Failure Modes

- **Virtual-carrot following is not exact path replay.** FTC follows a moving control target derived from the path. It does not promise to place the mower body or cutter center exactly on each taught sample.
- **Forward-only behavior collapses maneuvers.** Reverse corrections, pivots, and backing out of corners are unsupported by the default controller configuration.
- **Duplicate-position yaw changes are fragile.** A yaw-only pivot encoded as repeated XY poses may be skipped or treated as already reached because the old contract is position-path oriented.
- **Blade semantics are external.** The executor turns the mower on for a mow segment, but a `nav_msgs/Path` pose cannot say "blade off for this turn" or "pause here with blade off."
- **Source-quality loss hides root cause.** Once a path is flattened to poses, a later failure cannot tell whether the human pass relied on GPS, LIDAR, fusion, stale data, or a source disagreement.
- **Cross-track and along-track limits can stop execution.** FTC is configured to fail when follow error exceeds the configured lateral, longitudinal, or follow-distance limits. Human micro-corrections may recover from situations that old live replay will simply reject.
- **Progress reporting has known ambiguity.** `MowingBehavior` contains logic to distrust a successful FTC state unless reported progress is near the end of the pose list.

## Design Consequence

Manual path recording should not be shaped around what FTC can currently replay. It should record the full manual-mowing timeline first, then derive:

- `teacher_path.json` as the durable teach/debug contract;
- `reports/path_recording_summary.html` and `reports/source_comparison.json` as quality evidence;
- `exports/planpath_compat.json` as a narrow compatibility view that includes only clean forward, blade-on, base-pose segments.

Live repeat should graduate to a maneuver-aware executor before it tries to reproduce pivots, reverse moves, blade-off turns, or wheel-anchor behaviors.
