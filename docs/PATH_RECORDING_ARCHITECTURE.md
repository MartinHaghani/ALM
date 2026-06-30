# Manual Path Recording Architecture

Purpose: design and document the manual-mowing "teach path" recorder that captures GPS, LIDAR, and fused mower poses for debugging and later path-following experiments.

This document covers the first implemented milestone and the intended next steps. It does not imply that the live mower should replay a recorded path without a controlled dry run, mower-disabled validation, and an explicit operator action.

## Why This Exists

The current coverage-planner work is trying to generate safe, high-quality coverage from a recorded map. A manual path recording adds a different source of truth: what a human actually drove while mowing the lawn.

That recording has two separate uses:

- **Debugging:** compare GPS, LIDAR, fused pose, commands, mower state, and sensor quality over time.
- **Teach-and-repeat experiments:** convert the manually driven path into a cleaned `base_link` pose timeline that the mower can try to follow later.

Those uses should share one recording session, but not one artifact. Debugging needs raw evidence. Replay needs a filtered, versioned, quality-scored path contract.

## Existing Repo Context

Relevant current runtime pieces:

- `src/open_mower/launch/open_mower.launch` starts `_record.launch`, `_localization.launch`, optional passive SLAM, localization confidence, localization fusion, manual input routing, `mower_logic`, and the planner.
- `_localization.launch` starts `xbot_positioning`, which consumes `/hw/position/gps`, `/hw/imu/data_raw`, and `/hw/diff_drive/measured_twist`, and publishes `/xbot_positioning/xb_pose`.
- `_passive_slam.launch` starts the C1 passive SLAM stack when `OM_USE_PASSIVE_SLAM=True`. It provides `slam_odom -> slam_base_link`, `slam_map`, and an alignment helper that publishes `map -> slam_map` plus `/slam_toolbox_alignment/status`.
- `_localization_fusion.launch` starts `localization_fusion.py`, a shadow publisher for `/localization_fusion/pose`, `/localization_fusion/odom`, `/localization_fusion/status`, and `map -> fused_base_link`. It can feed area-recording geometry, but does not replace operational `map -> base_link` or feed navigation, planning, costmaps, or control.
- `AreaRecordingBehavior` already uses trusted pose gating for map recording: pose freshness, RTK-fixed raw GPS, accuracy limits, and segment breaks across GPS-quality gaps.
- `tools/coverage_lab/v2_live_trace_recorder.py` already records passive JSONL traces for V2 dry-run debugging, but it is plan-comparison oriented rather than a durable teach-path contract.

Important frame convention:

- `map` is the map frame.
- `base_link` is the executable body pose frame.
- On the current Mowrator model, `base_link` is the rear-center of the mower footprint.
- The cutter/tool center is separate, currently `[0.41, 0.0]` from `base_link`.

## Research Notes

Design choices here are grounded in ROS conventions and teach-and-repeat literature:

- ROS `rosbag` is the right raw replay/debug artifact because it records ROS messages and can play/read them back later: <https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/mainpage.dox>.
- `nav_msgs/Path` is only a header plus `geometry_msgs/PoseStamped[]`; it is useful as a compatibility view, but it cannot carry sensor provenance, blade state, direction, quality gates, or replay safety: <https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Path.msg>.
- `geometry_msgs/PoseStamped` is the standard stamped pose carrier, `nav_msgs/Odometry` carries pose plus twist in named frames, and `sensor_msgs/NavSatFix` preserves WGS84 GNSS data and covariance: <https://github.com/ros/common_msgs/blob/noetic-devel/geometry_msgs/msg/PoseStamped.msg>, <https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Odometry.msg>, <https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/NavSatFix.msg>.
- Teach-and-repeat systems separate the human-driven teach pass from the autonomous repeat pass. Furgale and Barfoot's VT&R work stores route/map evidence during a piloted teach phase, then localizes against it during repeat traversal: <https://asrl.utias.utoronto.ca/~ptf/JFR_VTnR/>.

Inference for this repo: store the raw teach evidence, then derive a replay path from it. Do not treat a raw bag or a plain `nav_msgs/Path` as the live execution contract.

See [FTC_EXECUTOR_LIMITATIONS.md](FTC_EXECUTOR_LIMITATIONS.md) for the current `PlanPath`/FTC executor concerns that make this separation necessary.

## Pose Streams

Record these pose streams separately:

- `gps_raw`: `/hw/position/gps`, type `xbot_msgs/AbsolutePose`. Receiver-origin absolute pose and RTK flags.
- `gps_fix`: `/hw/position/gps/fix`, type `sensor_msgs/NavSatFix`. WGS84 lat/lon evidence for satellite-map debugging.
- `gps_base`: `/xbot_positioning/xb_pose`, type `xbot_msgs/AbsolutePose`. Current operational GPS/IMU/twist-derived `base_link` pose.
- `lidar_local`: TF or odom for `slam_map -> slam_base_link`. Useful for diagnosing SLAM drift before global alignment.
- `lidar_map`: globally aligned LIDAR pose in `map`. Prefer TF lookup `map -> slam_base_link`; also record `/slam_toolbox_alignment/status` because it carries `lidar_pose`, `slam_pose`, residuals, source, and alignment state.
- `fused`: `/localization_fusion/pose`, type `xbot_msgs/AbsolutePose`. Shadow fusion of GPS and aligned passive LIDAR, used for `/next/` display and default area-recording geometry.

Also record the supporting quality topics:

- `/hw/position/gps/quality`
- `/localization_confidence/status`
- `/localization_fusion/status`
- `/slam_toolbox_alignment/status`
- `/tf` and `/tf_static`

## Session Artifacts

Each session writes a directory under `~/.ros/path_recordings` by default. Set `OM_MANUAL_PATH_RECORDINGS_PATH` to redirect it.

```text
$(RECORDINGS_PATH)/path_recordings/<session_id>/
  session.json
  samples.jsonl
  raw.bag                         # optional, only when selected by the operator
  teacher_path.json
  exports/
    planpath_compat.json
  reports/
    path_recording_summary.html
    source_comparison.json
```

The laptop lab should copy these into an ignored private path such as:

```text
tools/coverage_lab/data/path_recordings/<session_id>/
```

Implementation should add that directory to `.gitignore` before any real recording is copied into the repo.

## Raw Bag

`raw.bag` is the forensic artifact. It should include enough data to replay and re-derive every normalized artifact:

- `/tf`, `/tf_static`
- `/hw/position/gps`, `/hw/position/gps/fix`, `/hw/position/gps/quality`
- `/xbot_positioning/xb_pose`
- `/localization_fusion/pose`, `/localization_fusion/odom`, `/localization_fusion/status`
- `/slam_toolbox_alignment/status`
- `/slam_toolbox/local_odom`, `/slam_toolbox/map`, `/slam_toolbox/scan`
- `/hw/lidar`
- `/hw/imu/data_raw`
- `/hw/diff_drive/measured_twist`
- `/joy_vel`, `/web_joy_vel`, `/direct_joy_vel`, `/hw/cmd_vel`
- `/mower_logic/current_state`
- `/hw/status`, `/hw/power`, `/hw/emergency`
- `/mower_input/status`
- `/xbot/action`
- `/area_recorder/boundary_samples`

This bag is operator-selectable from `/next/` and only runs between Start Path Capture and Stop Path Capture. It is intentionally not automatic, because long raw bags can consume substantial disk and may include private yard traces.

## Normalized Samples

`samples.jsonl` is the analysis-friendly timeline. It should preserve raw-like per-topic records and also write synchronized sample records at a fixed rate, for example 10 Hz.

Example synchronized sample:

```json
{
  "type": "sync_sample",
  "seq": 1234,
  "stamp": 1782493201.42,
  "frame_id": "map",
  "body_frame": "base_link",
  "gps_raw": {"x": 10.1, "y": 4.2, "yaw": 1.57, "rtk": "fixed", "accuracy_m": 0.018},
  "gps_base": {"x": 10.0, "y": 4.1, "yaw": 1.55, "accuracy_m": 0.03},
  "lidar_map": {"x": 10.04, "y": 4.08, "yaw": 1.56, "state": "aligned", "residual_m": 0.09},
  "fused": {"x": 10.02, "y": 4.09, "yaw": 1.56, "accuracy_m": 0.04, "state": "ready"},
  "selected_pose_source": "fused",
  "source_quality": {"gps_lidar_separation_m": 0.05, "active_sources": ["gps", "lidar"]},
  "command": {"source_topic": "/web_joy_vel", "linear_x": 0.25, "angular_z": 0.02},
  "measured_twist": {"linear_x": 0.24, "angular_z": 0.02},
  "blade": {"mow_enabled": true},
  "accepted_for_replay": true,
  "reject_reasons": []
}
```

Rejected samples must still be written. Their `accepted_for_replay` should be false with concrete reasons such as `gps_not_rtk_fixed`, `fused_stale`, `lidar_unaligned`, `source_conflict`, `pose_jump`, or `emergency_active`. Blade-off samples are preserved as movement/transit evidence and are skipped only by the old PlanPath compatibility exporter.

## Teacher Path Contract

`teacher_path.json` is the future replay/debug path contract. It should not be a bag dump. It should be a compact, stable, versioned summary derived from `samples.jsonl`.

Top-level shape:

```json
{
  "schema": "open_mower.manual_teacher_path.v0",
  "session_id": "2026-06-26T18-30-10Z_back_yard_manual_mow",
  "frame_id": "map",
  "body_frame": "base_link",
  "mower_model": {
    "footprint": [[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]],
    "tool_center_offset": [0.41, 0.0],
    "tool_width_m": 0.4
  },
  "map": {
    "selected_map_id": "back_yard",
    "selected_map_hash": "sha256:...",
    "datum": {"lat": 0.0, "lon": 0.0}
  },
  "source_policy": {
    "replay_source": "fused",
    "sample_step_m": 0.1,
    "yaw_step_rad": 0.0873,
    "max_pose_age_s": 0.5,
    "max_gps_lidar_separation_m": 0.45
  },
  "segments": [],
  "events": [],
  "quality_summary": {}
}
```

Segment shape:

```json
{
  "id": "seg-0004",
  "kind": "MANUAL_MOW",
  "purpose": "CUT",
  "blade_state": "ON",
  "direction": "FORWARD",
  "controller_mode": "FOLLOW_PATH_DIAGNOSTIC_ONLY",
  "allowed_for_live_replay": false,
  "start_stamp": 1782493201.42,
  "end_stamp": 1782493262.80,
  "source_sample_range": [1234, 1881],
  "poses": [
    {
      "x": 10.02,
      "y": 4.09,
      "yaw": 1.56,
      "stamp": 1782493201.42,
      "distance_m": 0.0,
      "source": "fused",
      "position_accuracy_m": 0.04,
      "gps_lidar_separation_m": 0.05
    }
  ],
  "quality": {
    "rtk_fixed_ratio": 1.0,
    "lidar_aligned_ratio": 0.98,
    "fused_ready_ratio": 0.99,
    "max_gps_lidar_separation_m": 0.19,
    "mean_speed_mps": 0.23
  },
  "gap_before": null
}
```

`allowed_for_live_replay` should stay false until a separate exporter verifies that the segment is forward-only, sufficiently smooth, inside the selected map, and compatible with the current mower executor.

## Segmenting Rules

Create a new segment whenever any of these occur:

- recording starts, stops, or resumes;
- blade state changes;
- accepted pose source changes;
- selected pose is stale or rejected longer than `max_gap_s`;
- distance jump exceeds a configured threshold;
- yaw jump exceeds a configured threshold;
- GPS/LIDAR separation exceeds the replay block threshold;
- `mower_logic/current_state` leaves the expected manual recording state;
- operator inserts a marker event.

Keep rejected spans as `events[]` and in `samples.jsonl`; do not silently bridge them.

## Quality Gates

For debugging, record everything.

For replay export, require:

- selected replay source is fresh;
- selected pose is finite in `map`;
- raw GPS is RTK fixed when GPS participates in the selected source;
- fused pose is `ready` or `gps_only`/`lidar_only` with explicit policy approval;
- LIDAR pose is globally aligned when LIDAR participates;
- GPS/LIDAR separation is below the configured replay limit;
- command/measured twist does not show a large unmodeled skid or pause;
- footprint samples remain inside the selected mowing area when a selected map is available.

The default replay source should be `fused` because it is the closest match to the future GPS/LIDAR stack. Export `gps_base` and `lidar_map` paths too so debugging can show whether a replay failure is a controller issue or a localization-source issue.

## Node Architecture

The first milestone implements a passive recorder node under `src/open_mower/scripts/`:

```text
manual_path_recorder.py
```

Responsibilities:

- subscribe to the pose, TF, status, command, mower-state, and quality topics;
- expose services:
  - `/manual_path_recorder/start`, `std_srvs/SetBool`, with `data=true` requesting a raw bag;
  - `/manual_path_recorder/stop`, `std_srvs/Trigger`;
  - `/manual_path_recorder/mark_event`, `std_srvs/Trigger`;
  - `/manual_path_recorder/export`, `std_srvs/Trigger`;
- publish `/manual_path_recorder/status` as JSON;
- write `session.json`, `samples.jsonl`, optional `raw.bag`, and `teacher_path.json`;
- never publish drive commands;
- never enable the blade;
- never change mower state.

Launch integration is optional:

```text
src/open_mower/launch/include/_manual_path_recording.launch
```

It is included from `open_mower.launch` only when `OM_ENABLE_MANUAL_PATH_RECORDER=True`.

## UI Integration

The `/next/` UI adds a small path-recording control in the existing area-recording/manual-drive workflow:

- Start Path Capture
- Stop Path Capture
- Mark Event
- Export
- Raw Bag checkbox

Do not combine this with map-area "Start Recording" semantics. A map recording creates a mowing-area polygon. A path recording captures the route driven through that area. The operator may do both at once, but the artifacts should remain separate.

## Coverage Lab Integration

Add lab-side tools after the recorder exists:

- `teacher_path_summary`: read `teacher_path.json` and render GPS/LIDAR/fused overlays.
- `teacher_path_compare`: compare a planned route against the manually driven fused path.
- `teacher_path_to_planpath`: export a conservative old-shape `planpath_compat.json` only for forward, blade-on, FTC-trackable segments.

The existing `v2_trace_compare.py` can be reused for planned-vs-actual metrics. The new comparison direction is manual-teacher-vs-generated-plan.

## Replay Plan

Do not attempt live replay directly from a raw recording.

Safe progression:

1. Record a manual mow with blade enabled and full raw evidence.
2. Generate `teacher_path.json`, `reports/path_recording_summary.html`, and source-comparison metrics.
3. Export `planpath_compat.json` from only clean forward fused segments.
4. Use the existing static PlanPath server with `OM_ENABLE_MOWER=False`.
5. Run a dry drive and record a `v2_live_trace_recorder.py` trace.
6. Compare planned teacher path vs actual mower path.
7. Only after dry-run error is acceptable, test with blade enabled in a small controlled area.

This should feed the broader P4 planner work: live execution eventually needs maneuver-aware segment semantics, not hidden meanings inside `nav_msgs/Path`.

## Implementation Phases

1. **Passive local recorder:** implemented as `manual_path_recorder.py`, writing `samples.jsonl` and `teacher_path.json`.
2. **Raw bag option:** implemented through a bounded `rosbag record` subprocess that starts and stops with capture.
3. **Report generator:** implemented as `reports/path_recording_summary.html` and `reports/source_comparison.json`.
4. **`/next/` controls:** implemented with explicit start/stop/mark/export controls while keeping motion authority in `mower_input_router`.
5. **Replay export:** implemented as conservative `planpath_compat.json` that blocks unsafe or unsupported segments.
6. **Executor work:** if replay proves useful, graduate from old `PlanPath` compatibility into the maneuver-aware contract already described in the coverage-planner V2 docs.

## Open Questions

- Should `manual_path_recorder` live permanently in `open_mower` or become a new `mower_path_recording` package once services/messages stabilize?
- Should the selected replay source default to `fused` even when it is `lidar_only`, or require GPS participation for first live tests?
- Should blade-on/off be inferred from `/hw/status.mow_enabled`, `mower_logic` actions, or both?
- Should repeated manual passes be stored as separate sessions or as multiple named passes inside one session?
- Should the planner use the teacher path as a hard path to repeat, or as a scoring baseline for generated coverage plans?
