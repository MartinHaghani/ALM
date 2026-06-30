#!/usr/bin/env python3
"""Record live mower telemetry to JSONL for V2 coverage dry-run debugging.

The recorder is intentionally passive: it subscribes to ROS topics and writes
timestamped samples. It does not publish commands or call mower services.

The companion ``v2_trace_compare.py`` turns the JSONL plus a
``planpath_compat.json`` into an HTML path-vs-actual report.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import time
from typing import Any


DEFAULT_TOPICS = [
    "/xbot_positioning/xb_pose",
    "/hw/position/gps",
    "/localization_fusion/pose",
    "/hw/position/gps/quality",
    "/mower_logic/current_state",
    "/hw/diff_drive/measured_twist",
    "/hw/cmd_vel",
    "/hw/imu/data_raw",
    "/rosout_agg",
    "/mower_logic/route_plan_json",
    "/move_base_flex/GlobalPlanner/plan",
    "/move_base_flex/FTCPlanner/costmap_marker",
    "/move_base_flex/FTCPlanner/debug_pid",
    "/move_base_flex/FTCPlanner/global_point",
    "/move_base_flex/FTCPlanner/global_plan",
    "/move_base_flex/local_costmap/footprint",
    "/move_base_flex/global_costmap/footprint",
    "/move_base_flex/move_base/result",
    "/move_base_flex/exe_path/status",
    "/move_base_flex/exe_path/result",
    "/move_base_flex/move_base/status",
]

DEFAULT_HZ_LIMITS = {
    "/xbot_positioning/xb_pose": 20.0,
    "/localization_fusion/pose": 20.0,
    "/hw/position/gps": 10.0,
    "/hw/position/gps/quality": 2.0,
    "/mower_logic/current_state": 5.0,
    "/hw/diff_drive/measured_twist": 20.0,
    "/hw/cmd_vel": 20.0,
    "/hw/imu/data_raw": 20.0,
    "/rosout_agg": 20.0,
    "/mower_logic/route_plan_json": 5.0,
    "/move_base_flex/GlobalPlanner/plan": 2.0,
    "/move_base_flex/FTCPlanner/costmap_marker": 5.0,
    "/move_base_flex/FTCPlanner/debug_pid": 20.0,
    "/move_base_flex/FTCPlanner/global_point": 20.0,
    "/move_base_flex/FTCPlanner/global_plan": 2.0,
    "/move_base_flex/local_costmap/footprint": 5.0,
    "/move_base_flex/global_costmap/footprint": 2.0,
    "/move_base_flex/move_base/result": 0.0,
    "/move_base_flex/exe_path/status": 5.0,
    "/move_base_flex/exe_path/result": 0.0,
    "/move_base_flex/move_base/status": 5.0,
}


def _yaw_from_quaternion(rotation: Any) -> float:
    x = float(getattr(rotation, "x", 0.0))
    y = float(getattr(rotation, "y", 0.0))
    z = float(getattr(rotation, "z", 0.0))
    w = float(getattr(rotation, "w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _finite_float(value: Any) -> float | None:
    try:
        output = float(value)
    except (TypeError, ValueError):
        return None
    return output if math.isfinite(output) else None


def _stamp_to_sec(stamp: Any) -> float | None:
    if stamp is None:
        return None
    if hasattr(stamp, "to_sec"):
        try:
            return float(stamp.to_sec())
        except Exception:
            return None
    secs = getattr(stamp, "secs", None)
    nsecs = getattr(stamp, "nsecs", None)
    if secs is None or nsecs is None:
        return None
    try:
        return float(secs) + float(nsecs) * 1e-9
    except (TypeError, ValueError):
        return None


def _message_stamp(msg: Any) -> float | None:
    header = getattr(msg, "header", None)
    return _stamp_to_sec(getattr(header, "stamp", None))


def _primitive(value: Any) -> Any:
    if isinstance(value, (str, bool, int)) or value is None:
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if hasattr(value, "to_sec"):
        return _stamp_to_sec(value)
    if isinstance(value, (list, tuple)):
        return [_primitive(item) for item in value]
    if hasattr(value, "__slots__"):
        return {slot: _primitive(getattr(value, slot)) for slot in value.__slots__}
    return str(value)


def _absolute_pose_derived(msg: Any) -> dict[str, Any]:
    pose = getattr(getattr(msg, "pose", None), "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    motion = getattr(msg, "motion_vector", None)
    return {
        "x": _finite_float(getattr(position, "x", None)),
        "y": _finite_float(getattr(position, "y", None)),
        "yaw": _yaw_from_quaternion(orientation) if orientation is not None else None,
        "position_accuracy": _finite_float(getattr(msg, "position_accuracy", None)),
        "orientation_accuracy": _finite_float(getattr(msg, "orientation_accuracy", None)),
        "orientation_valid": bool(getattr(msg, "orientation_valid", False)),
        "motion_vector_valid": bool(getattr(msg, "motion_vector_valid", False)),
        "motion_vector_x": _finite_float(getattr(motion, "x", None)),
        "motion_vector_y": _finite_float(getattr(motion, "y", None)),
        "vehicle_heading": _finite_float(getattr(msg, "vehicle_heading", None)),
        "motion_heading": _finite_float(getattr(msg, "motion_heading", None)),
        "flags": int(getattr(msg, "flags", 0) or 0),
    }


def _twist_derived(msg: Any) -> dict[str, Any]:
    twist = getattr(msg, "twist", msg)
    linear = getattr(twist, "linear", None)
    angular = getattr(twist, "angular", None)
    return {
        "linear_x": _finite_float(getattr(linear, "x", None)),
        "linear_y": _finite_float(getattr(linear, "y", None)),
        "angular_z": _finite_float(getattr(angular, "z", None)),
    }


def _pose_stamped_derived(msg: Any) -> dict[str, Any]:
    pose = getattr(msg, "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    return {
        "x": _finite_float(getattr(position, "x", None)),
        "y": _finite_float(getattr(position, "y", None)),
        "yaw": _yaw_from_quaternion(orientation) if orientation is not None else None,
    }


def _path_derived(msg: Any) -> dict[str, Any]:
    poses = list(getattr(msg, "poses", []) or [])
    points: list[tuple[float, float, float | None]] = []
    for pose_stamped in poses:
        pose = getattr(pose_stamped, "pose", None)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        x = _finite_float(getattr(position, "x", None))
        y = _finite_float(getattr(position, "y", None))
        if x is None or y is None:
            continue
        points.append((x, y, _yaw_from_quaternion(orientation) if orientation is not None else None))
    length = 0.0
    for index in range(1, len(points)):
        length += math.hypot(points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1])
    output: dict[str, Any] = {
        "frame_id": getattr(getattr(msg, "header", None), "frame_id", ""),
        "pose_count": len(poses),
        "valid_point_count": len(points),
        "length_m": length,
    }
    if points:
        output.update(
            {
                "first": {"x": points[0][0], "y": points[0][1], "yaw": points[0][2]},
                "last": {"x": points[-1][0], "y": points[-1][1], "yaw": points[-1][2]},
                "bounds": {
                    "min_x": min(point[0] for point in points),
                    "min_y": min(point[1] for point in points),
                    "max_x": max(point[0] for point in points),
                    "max_y": max(point[1] for point in points),
                },
            }
        )
    return output


def _point_bounds(points: list[tuple[float, float]]) -> dict[str, float] | None:
    if not points:
        return None
    return {
        "min_x": min(point[0] for point in points),
        "min_y": min(point[1] for point in points),
        "max_x": max(point[0] for point in points),
        "max_y": max(point[1] for point in points),
    }


def _polygon_derived(msg: Any) -> dict[str, Any]:
    polygon = getattr(msg, "polygon", None)
    points = []
    for point in list(getattr(polygon, "points", []) or []):
        x = _finite_float(getattr(point, "x", None))
        y = _finite_float(getattr(point, "y", None))
        if x is None or y is None:
            continue
        points.append((x, y))
    return {
        "frame_id": getattr(getattr(msg, "header", None), "frame_id", ""),
        "point_count": len(points),
        "bounds": _point_bounds(points),
    }


def _marker_derived(msg: Any) -> dict[str, Any]:
    points = []
    for point in list(getattr(msg, "points", []) or []):
        x = _finite_float(getattr(point, "x", None))
        y = _finite_float(getattr(point, "y", None))
        if x is None or y is None:
            continue
        points.append((x, y))
    pose = getattr(msg, "pose", None)
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    return {
        "frame_id": getattr(getattr(msg, "header", None), "frame_id", ""),
        "ns": str(getattr(msg, "ns", "")),
        "id": int(getattr(msg, "id", 0) or 0),
        "type": int(getattr(msg, "type", 0) or 0),
        "action": int(getattr(msg, "action", 0) or 0),
        "pose_x": _finite_float(getattr(position, "x", None)),
        "pose_y": _finite_float(getattr(position, "y", None)),
        "pose_yaw": _yaw_from_quaternion(orientation) if orientation is not None else None,
        "point_count": len(points),
        "bounds": _point_bounds(points),
    }


def _imu_derived(msg: Any) -> dict[str, Any]:
    angular = getattr(msg, "angular_velocity", None)
    accel = getattr(msg, "linear_acceleration", None)
    return {
        "angular_z": _finite_float(getattr(angular, "z", None)),
        "linear_accel_x": _finite_float(getattr(accel, "x", None)),
        "linear_accel_y": _finite_float(getattr(accel, "y", None)),
        "linear_accel_z": _finite_float(getattr(accel, "z", None)),
    }


def _quality_derived(msg: Any) -> dict[str, Any]:
    payload = getattr(msg, "data", "")
    if not isinstance(payload, str):
        return {}
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return {"raw": payload}
    keys = [
        "fix_type",
        "rtk_type",
        "carrier_phase",
        "num_sv",
        "pdop",
        "h_acc_m",
        "v_acc_m",
        "speed_acc_mps",
        "head_acc_rad",
        "vehicle_heading_valid",
        "motion_heading_valid",
        "rtcm_age_s",
        "rtcm_bytes_per_sec",
        "parser_skipped_bytes",
        "parser_invalid_checksums",
        "parser_fix_warnings",
        "parser_timing_warnings",
    ]
    return {key: data.get(key) for key in keys if key in data}


def _state_derived(msg: Any) -> dict[str, Any]:
    return {
        "state": int(getattr(msg, "state", 0) or 0),
        "state_name": str(getattr(msg, "state_name", "")),
        "sub_state_name": str(getattr(msg, "sub_state_name", "")),
        "current_area": int(getattr(msg, "current_area", -1) or -1),
        "current_path": int(getattr(msg, "current_path", -1) or -1),
        "current_path_index": int(getattr(msg, "current_path_index", -1) or -1),
        "gps_quality_percent": _finite_float(getattr(msg, "gps_quality_percent", None)),
        "battery_percent": _finite_float(getattr(msg, "battery_percent", None)),
        "emergency": bool(getattr(msg, "emergency", False)),
    }


def _ftc_debug_derived(msg: Any) -> dict[str, Any]:
    return {
        "lon_err": _finite_float(getattr(msg, "lon_err", None)),
        "lat_err": _finite_float(getattr(msg, "lat_err", None)),
        "ang_err": _finite_float(getattr(msg, "ang_err", None)),
        "lin_speed": _finite_float(getattr(msg, "lin_speed", None)),
        "ang_speed": _finite_float(getattr(msg, "ang_speed", None)),
        "planner_state": int(getattr(msg, "planner_state", 0) or 0),
        "planner_state_name": str(getattr(msg, "planner_state_name", "")),
        "current_index": int(getattr(msg, "current_index", 0) or 0),
        "current_progress": _finite_float(getattr(msg, "current_progress", None)),
        "movement_speed": _finite_float(getattr(msg, "movement_speed", None)),
        "is_crashed": bool(getattr(msg, "is_crashed", False)),
        "control_distance": _finite_float(getattr(msg, "control_distance", None)),
        "control_yaw_error": _finite_float(getattr(msg, "control_yaw_error", None)),
        "failure_reason": str(getattr(msg, "failure_reason", "")),
    }


def _route_plan_derived(msg: Any) -> dict[str, Any]:
    payload = getattr(msg, "data", "")
    if not isinstance(payload, str):
        return {}
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return {"raw": payload[:500]}
    paths = list(data.get("paths", []) or [])
    pose_counts = [len(path.get("poses", []) or []) for path in paths]
    return {
        "schema": data.get("schema"),
        "source": data.get("source"),
        "plan_id": data.get("plan_id"),
        "active": data.get("active"),
        "current_area_index": data.get("current_area_index"),
        "current_path_index": data.get("current_path_index"),
        "current_pose_index": data.get("current_pose_index"),
        "last_event": data.get("last_event"),
        "last_error": data.get("last_error"),
        "last_mbf_state": data.get("last_mbf_state"),
        "path_count": len(paths),
        "pose_counts": pose_counts,
        "outline_flags": [bool(path.get("is_outline")) for path in paths],
        "map_id": data.get("map_id"),
        "map_hash": data.get("map_hash"),
    }


def _action_status_derived(msg: Any) -> dict[str, Any]:
    statuses = []
    for status in list(getattr(msg, "status_list", []) or [])[-5:]:
        statuses.append(
            {
                "goal_id": str(getattr(getattr(status, "goal_id", None), "id", "")),
                "status": int(getattr(status, "status", 0) or 0),
                "text": str(getattr(status, "text", "")),
            }
        )
    return {"statuses": statuses}


def _action_result_derived(msg: Any) -> dict[str, Any]:
    status = getattr(msg, "status", None)
    result = getattr(msg, "result", None)
    return {
        "status": int(getattr(status, "status", 0) or 0),
        "status_text": str(getattr(status, "text", "")),
        "outcome": int(getattr(result, "outcome", 0) or 0),
        "message": str(getattr(result, "message", "")),
    }


def _rosout_derived(msg: Any) -> dict[str, Any]:
    return {
        "level": int(getattr(msg, "level", 0) or 0),
        "name": str(getattr(msg, "name", "")),
        "msg": str(getattr(msg, "msg", "")),
        "file": str(getattr(msg, "file", "")),
        "function": str(getattr(msg, "function", "")),
        "line": int(getattr(msg, "line", 0) or 0),
    }


def _derive(topic: str, msg: Any) -> dict[str, Any]:
    if topic in {"/xbot_positioning/xb_pose", "/hw/position/gps", "/localization_fusion/pose"}:
        return _absolute_pose_derived(msg)
    if topic == "/hw/position/gps/quality":
        return _quality_derived(msg)
    if topic == "/mower_logic/current_state":
        return _state_derived(msg)
    if topic in {"/hw/diff_drive/measured_twist", "/hw/cmd_vel"}:
        return _twist_derived(msg)
    if topic == "/hw/imu/data_raw":
        return _imu_derived(msg)
    if topic == "/rosout_agg":
        return _rosout_derived(msg)
    if topic == "/mower_logic/route_plan_json":
        return _route_plan_derived(msg)
    if topic == "/move_base_flex/FTCPlanner/debug_pid":
        return _ftc_debug_derived(msg)
    if topic == "/move_base_flex/FTCPlanner/global_point":
        return _pose_stamped_derived(msg)
    if topic in {"/move_base_flex/GlobalPlanner/plan", "/move_base_flex/FTCPlanner/global_plan"}:
        return _path_derived(msg)
    if topic in {"/move_base_flex/local_costmap/footprint", "/move_base_flex/global_costmap/footprint"}:
        return _polygon_derived(msg)
    if topic == "/move_base_flex/FTCPlanner/costmap_marker":
        return _marker_derived(msg)
    if topic in {"/move_base_flex/exe_path/status", "/move_base_flex/move_base/status"}:
        return _action_status_derived(msg)
    if topic in {"/move_base_flex/exe_path/result", "/move_base_flex/move_base/result"}:
        return _action_result_derived(msg)
    return {}


def _plan_summary(path: pathlib.Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    try:
        data = path.read_bytes()
        plan = json.loads(data.decode("utf-8"))
    except (OSError, json.JSONDecodeError, UnicodeDecodeError):
        return {"path": str(path), "read_error": True}
    paths = list(plan.get("paths", []) or [])
    pose_count = sum(len(p.get("path", {}).get("poses", []) or []) for p in paths)
    return {
        "path": str(path),
        "sha256": hashlib.sha256(data).hexdigest(),
        "profile": plan.get("profile"),
        "path_count": len(paths),
        "pose_count": pose_count,
        "outline_count": sum(1 for p in paths if bool(p.get("is_outline", False))),
    }


class JsonlRecorder:
    def __init__(self, output: pathlib.Path, plan: pathlib.Path | None, hz_limits: dict[str, float]) -> None:
        self.output = output
        self.plan = plan
        self.hz_limits = hz_limits
        self.last_written: dict[str, float] = {}
        self.counts: dict[str, int] = {}
        self.handle = None

    def __enter__(self) -> "JsonlRecorder":
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.output.open("w", encoding="utf-8", buffering=1)
        return self

    def __exit__(self, *_exc: Any) -> None:
        if self.handle is not None:
            self.write(
                {
                    "type": "summary",
                    "wall_time": time.time(),
                    "counts": dict(sorted(self.counts.items())),
                }
            )
            self.handle.close()

    def write(self, record: dict[str, Any]) -> None:
        assert self.handle is not None
        self.handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")

    def metadata(self, topics: list[str], subscribed: list[str], unavailable: list[str]) -> None:
        self.write(
            {
                "type": "metadata",
                "schema": "open_mower.coverage_lab.live_trace.v0",
                "wall_time": time.time(),
                "topics_requested": topics,
                "topics_subscribed": subscribed,
                "topics_unavailable": unavailable,
                "hz_limits": self.hz_limits,
                "plan": _plan_summary(self.plan),
            }
        )

    def callback(self, topic: str, msg: Any) -> None:
        now = time.time()
        limit = self.hz_limits.get(topic, 0.0)
        if limit > 0:
            min_interval = 1.0 / limit
            if now - self.last_written.get(topic, -1e9) < min_interval:
                return
        self.last_written[topic] = now
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.write(
            {
                "type": "sample",
                "topic": topic,
                "wall_time": now,
                "stamp": _message_stamp(msg),
                "derived": _derive(topic, msg),
                "data": _primitive(msg),
            }
        )


def _parse_topic_limits(values: list[str]) -> dict[str, float]:
    limits = dict(DEFAULT_HZ_LIMITS)
    for value in values:
        if "=" not in value:
            raise SystemExit(f"topic limit must be TOPIC=HZ, got {value!r}")
        topic, raw_hz = value.split("=", 1)
        limits[topic.strip()] = float(raw_hz)
    return limits


def cmd_record(args: argparse.Namespace) -> None:
    import rospy
    import rostopic

    topics = args.topic or list(DEFAULT_TOPICS)
    hz_limits = _parse_topic_limits(args.limit_hz or [])
    plan = pathlib.Path(args.plan).resolve() if args.plan else None
    subscribers = []
    subscribed: list[str] = []
    unavailable: list[str] = []

    rospy.init_node(args.node_name, anonymous=True)
    deadline = time.time() + max(0.0, float(args.wait_topic_sec))
    topic_classes: dict[str, Any] = {}
    pending = set(topics)
    while pending and time.time() < deadline and not rospy.is_shutdown():
        for topic in list(pending):
            msg_class, _real_topic, _eval_fn = rostopic.get_topic_class(topic, blocking=False)
            if msg_class is not None:
                topic_classes[topic] = msg_class
                pending.remove(topic)
        if pending:
            time.sleep(0.2)
    unavailable = sorted(pending)

    with JsonlRecorder(pathlib.Path(args.output).resolve(), plan, hz_limits) as recorder:
        for topic in topics:
            msg_class = topic_classes.get(topic)
            if msg_class is None:
                continue
            subscribers.append(
                rospy.Subscriber(
                    topic,
                    msg_class,
                    lambda msg, topic=topic: recorder.callback(topic, msg),
                    queue_size=20,
                )
            )
            subscribed.append(topic)
        recorder.metadata(topics, subscribed, unavailable)
        rospy.logwarn("V2 live trace recorder writing %s", pathlib.Path(args.output).resolve())
        rospy.logwarn("Subscribed topics: %s", ", ".join(subscribed) if subscribed else "(none)")
        if unavailable:
            rospy.logwarn("Unavailable topics: %s", ", ".join(unavailable))
        start = time.time()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if args.duration_sec and time.time() - start >= float(args.duration_sec):
                break
            rate.sleep()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record passive ROS telemetry for coverage dry-run debugging.")
    parser.add_argument("--output", required=True, help="JSONL trace output path")
    parser.add_argument("--plan", help="planpath_compat JSON being tested, stored as metadata")
    parser.add_argument("--topic", action="append", help="topic to record; repeat to override defaults")
    parser.add_argument("--limit-hz", action="append", help="per-topic throttle as TOPIC=HZ; repeat as needed")
    parser.add_argument("--duration-sec", type=float, default=0.0, help="recording duration; 0 means until Ctrl-C")
    parser.add_argument("--wait-topic-sec", type=float, default=8.0, help="seconds to wait for topics at startup")
    parser.add_argument("--node-name", default="v2_live_trace_recorder", help="ROS node name")
    parser.set_defaults(func=cmd_record)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
