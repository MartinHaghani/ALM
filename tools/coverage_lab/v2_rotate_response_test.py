#!/usr/bin/env python3
"""Run a bounded rotate-in-place response diagnostic on the live mower.

The test publishes a short sequence of angular velocity commands and records
the command, measured twist, IMU yaw rate, pose yaw, GPS quality, mower state,
and ESC status signals needed to decide whether a rotate stall is command-path,
hardware-response, or heading-estimation related.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import time
from dataclasses import dataclass
from typing import Any

from v2_live_trace_recorder import (
    _absolute_pose_derived,
    _finite_float,
    _imu_derived,
    _primitive,
    _quality_derived,
    _state_derived,
    _twist_derived,
    _yaw_from_quaternion,
)


DEFAULT_PULSES = ["0.4:2.0", "0.0:1.0", "-0.4:2.0", "0.0:1.0", "0.8:2.0", "0.0:2.0"]


@dataclass
class Pulse:
    angular_z: float
    duration_s: float
    start_t: float | None = None
    end_t: float | None = None


def _parse_pulse(value: str) -> Pulse:
    try:
        angular_s, duration_s = value.split(":", 1)
        angular_z = float(angular_s)
        duration = float(duration_s)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("pulse must be ANGULAR_Z:DURATION_SECONDS") from exc
    if not math.isfinite(angular_z) or not math.isfinite(duration) or duration <= 0.0:
        raise argparse.ArgumentTypeError("pulse values must be finite and duration must be > 0")
    return Pulse(angular_z=angular_z, duration_s=duration)


def _now() -> float:
    return time.time()


def _median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def _max_abs(values: list[float]) -> float | None:
    return max((abs(value) for value in values), default=None)


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def _sign_matches(command: float, response: float | None) -> bool | None:
    if response is None or abs(command) < 1e-6 or abs(response) < 1e-6:
        return None
    return (command > 0.0) == (response > 0.0)


def _tf_message_derived(msg: Any) -> dict[str, Any]:
    transforms = []
    for transform in getattr(msg, "transforms", []):
        parent = getattr(getattr(transform, "header", None), "frame_id", "")
        child = getattr(transform, "child_frame_id", "")
        rotation = getattr(getattr(transform, "transform", None), "rotation", None)
        translation = getattr(getattr(transform, "transform", None), "translation", None)
        item = {
            "parent": parent,
            "child": child,
            "x": _finite_float(getattr(translation, "x", None)),
            "y": _finite_float(getattr(translation, "y", None)),
            "yaw": _yaw_from_quaternion(rotation) if rotation is not None else None,
        }
        transforms.append(item)
    direct = next((item for item in transforms if item["parent"] == "map" and item["child"] == "base_link"), None)
    return {
        "transforms": transforms,
        "map_base_yaw": direct["yaw"] if direct else None,
        "map_base_x": direct["x"] if direct else None,
        "map_base_y": direct["y"] if direct else None,
    }


class RotateResponseRecorder:
    def __init__(self, output: pathlib.Path):
        self.output = output
        self.file = None
        self.samples: list[dict[str, Any]] = []
        self.latest: dict[str, dict[str, Any]] = {}

    def open(self) -> None:
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output.open("w", encoding="utf-8")

    def close(self) -> None:
        if self.file is not None:
            self.file.close()
            self.file = None

    def record_event(self, event: str, payload: dict[str, Any] | None = None) -> None:
        self._write({"t": _now(), "event": event, "payload": payload or {}})

    def record_message(self, topic: str, msg: Any) -> None:
        derived = self._derive(topic, msg)
        row = {
            "t": _now(),
            "topic": topic,
            "type": getattr(msg, "_type", type(msg).__name__),
            "stamp": self._stamp(msg),
            "derived": derived,
            "msg": _primitive(msg),
        }
        self.latest[topic] = row
        self.samples.append(row)
        self._write(row)

    def _write(self, row: dict[str, Any]) -> None:
        if self.file is None:
            raise RuntimeError("recorder not open")
        self.file.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
        self.file.flush()

    def _stamp(self, msg: Any) -> float | None:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is None or not hasattr(stamp, "to_sec"):
            return None
        try:
            return float(stamp.to_sec())
        except Exception:
            return None

    def _derive(self, topic: str, msg: Any) -> dict[str, Any]:
        if topic in {"/hw/cmd_vel", "/override_vel", "/hw/diff_drive/measured_twist"}:
            return _twist_derived(msg)
        if topic == "/hw/imu/data_raw":
            return _imu_derived(msg)
        if topic in {"/xbot_positioning/xb_pose", "/localization_fusion/pose"}:
            return _absolute_pose_derived(msg)
        if topic == "/hw/position/gps/quality":
            return _quality_derived(msg)
        if topic == "/mower_logic/current_state":
            return _state_derived(msg)
        if topic in {"/hw/diff_drive/left_esc_status", "/hw/diff_drive/right_esc_status"}:
            return {
                "status": int(getattr(msg, "status", 0) or 0),
                "current": _finite_float(getattr(msg, "current", None)),
                "tacho": int(getattr(msg, "tacho", 0) or 0),
                "rpm": int(getattr(msg, "rpm", 0) or 0),
                "temperature_motor": _finite_float(getattr(msg, "temperature_motor", None)),
                "temperature_pcb": _finite_float(getattr(msg, "temperature_pcb", None)),
            }
        if topic == "/hw/emergency":
            return {
                "active_emergency": bool(getattr(msg, "active_emergency", False)),
                "latched_emergency": bool(getattr(msg, "latched_emergency", False)),
                "reason": str(getattr(msg, "reason", "")),
            }
        if topic == "/tf":
            return _tf_message_derived(msg)
        return {}

    def topic_values(self, topic: str, key: str, start_t: float, end_t: float) -> list[float]:
        values = []
        for row in self.samples:
            if row.get("topic") != topic:
                continue
            if not (start_t <= float(row["t"]) <= end_t):
                continue
            value = _finite_float(row.get("derived", {}).get(key))
            if value is not None:
                values.append(value)
        return values

    def yaw_delta(self, topic: str, key: str, start_t: float, end_t: float) -> float | None:
        yaws = []
        for row in self.samples:
            if row.get("topic") != topic:
                continue
            if not (start_t <= float(row["t"]) <= end_t):
                continue
            value = _finite_float(row.get("derived", {}).get(key))
            if value is not None:
                yaws.append(value)
        if len(yaws) < 2:
            return None
        return _angle_delta(yaws[0], yaws[-1])


def _wait_for_initial_state(recorder: RotateResponseRecorder, timeout_s: float) -> None:
    import rospy

    deadline = rospy.Time.now() + rospy.Duration(timeout_s)
    required = ["/mower_logic/current_state", "/hw/emergency"]
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if all(topic in recorder.latest for topic in required):
            return
        rospy.sleep(0.05)


def _assert_safe_to_execute(recorder: RotateResponseRecorder, allow_non_idle: bool) -> None:
    state = recorder.latest.get("/mower_logic/current_state")
    emergency = recorder.latest.get("/hw/emergency")
    if state is None:
        raise RuntimeError("No /mower_logic/current_state sample received")
    if emergency is None:
        raise RuntimeError("No /hw/emergency sample received")

    state_derived = state.get("derived", {})
    state_name = str(state_derived.get("state_name", ""))
    state_id = int(state_derived.get("state", 0) or 0) & 0b11111
    if not allow_non_idle and state_name != "IDLE" and state_id != 1:
        raise RuntimeError(f"Mower is not IDLE: state_name={state_name!r}, state={state_id}")
    if bool(state_derived.get("emergency", False)):
        raise RuntimeError("Mower high-level state reports emergency")

    emergency_derived = emergency.get("derived", {})
    if bool(emergency_derived.get("active_emergency", False)) or bool(emergency_derived.get("latched_emergency", False)):
        raise RuntimeError(f"Hardware emergency is active/latched: {emergency_derived.get('reason', '')}")


def _publish_for(pub: Any, recorder: RotateResponseRecorder, angular_z: float, duration_s: float, hz: float) -> None:
    import rospy
    from geometry_msgs.msg import Twist

    msg = Twist()
    msg.angular.z = angular_z
    period = 1.0 / hz
    end = time.monotonic() + duration_s
    while not rospy.is_shutdown() and time.monotonic() < end:
        pub.publish(msg)
        recorder.record_event("command_publish", {"linear_x": 0.0, "angular_z": angular_z})
        rospy.sleep(period)


def _subscribe(recorder: RotateResponseRecorder) -> list[Any]:
    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from mower_msgs.msg import ESCStatus, Emergency, HighLevelStatus
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from xbot_msgs.msg import AbsolutePose

    topics = [
        ("/mower_logic/current_state", HighLevelStatus),
        ("/hw/emergency", Emergency),
        ("/hw/cmd_vel", Twist),
        ("/override_vel", Twist),
        ("/hw/diff_drive/measured_twist", TwistStamped),
        ("/hw/imu/data_raw", Imu),
        ("/xbot_positioning/xb_pose", AbsolutePose),
        ("/localization_fusion/pose", AbsolutePose),
        ("/hw/position/gps/quality", String),
        ("/hw/diff_drive/left_esc_status", ESCStatus),
        ("/hw/diff_drive/right_esc_status", ESCStatus),
        ("/tf", TFMessage),
    ]
    subscribers = []
    for topic, msg_type in topics:
        subscribers.append(rospy.Subscriber(topic, msg_type, lambda msg, name=topic: recorder.record_message(name, msg), queue_size=100))
    return subscribers


def _summarize_pulses(recorder: RotateResponseRecorder, pulses: list[Pulse], min_response_wz: float) -> dict[str, Any]:
    summaries = []
    nonzero = [pulse for pulse in pulses if abs(pulse.angular_z) > 1e-6 and pulse.start_t is not None and pulse.end_t is not None]
    for pulse in nonzero:
        assert pulse.start_t is not None
        assert pulse.end_t is not None
        cmd = recorder.topic_values("/hw/cmd_vel", "angular_z", pulse.start_t, pulse.end_t)
        twist = recorder.topic_values("/hw/diff_drive/measured_twist", "angular_z", pulse.start_t, pulse.end_t)
        imu = recorder.topic_values("/hw/imu/data_raw", "angular_z", pulse.start_t, pulse.end_t)
        pose_delta = recorder.yaw_delta("/xbot_positioning/xb_pose", "yaw", pulse.start_t, pulse.end_t)
        fused_delta = recorder.yaw_delta("/localization_fusion/pose", "yaw", pulse.start_t, pulse.end_t)
        tf_delta = recorder.yaw_delta("/tf", "map_base_yaw", pulse.start_t, pulse.end_t)
        cmd_max_abs = _max_abs(cmd)
        twist_median = _median(twist)
        imu_median = _median(imu)
        response_seen = (_max_abs(twist) or 0.0) >= min_response_wz or (_max_abs(imu) or 0.0) >= min_response_wz
        pose_tracking = pose_delta is not None and abs(pose_delta) >= math.radians(2.0)
        summaries.append(
            {
                "commanded_angular_z": pulse.angular_z,
                "duration_s": round(pulse.duration_s, 3),
                "cmd_samples": len(cmd),
                "cmd_median_angular_z": _median(cmd),
                "cmd_max_abs_angular_z": cmd_max_abs,
                "cmd_nonzero_samples": sum(1 for value in cmd if abs(value) >= min_response_wz),
                "cmd_sign_matches": _sign_matches(pulse.angular_z, _median(cmd)),
                "measured_twist_samples": len(twist),
                "measured_twist_median_angular_z": twist_median,
                "measured_twist_max_abs_angular_z": _max_abs(twist),
                "measured_twist_sign_matches": _sign_matches(pulse.angular_z, twist_median),
                "imu_samples": len(imu),
                "imu_median_angular_z": imu_median,
                "imu_max_abs_angular_z": _max_abs(imu),
                "imu_sign_matches": _sign_matches(pulse.angular_z, imu_median),
                "xbot_yaw_delta_deg": math.degrees(pose_delta) if pose_delta is not None else None,
                "fusion_yaw_delta_deg": math.degrees(fused_delta) if fused_delta is not None else None,
                "tf_map_base_yaw_delta_deg": math.degrees(tf_delta) if tf_delta is not None else None,
                "response_seen": response_seen,
                "pose_yaw_tracking": pose_tracking,
            }
        )

    command_seen = any((item["cmd_samples"] or 0) > 0 and (item.get("cmd_max_abs_angular_z") or 0.0) > 0.05 for item in summaries)
    response_seen = any(bool(item["response_seen"]) for item in summaries)
    heading_seen = any(bool(item["pose_yaw_tracking"]) for item in summaries)
    if not summaries:
        verdict = "no_nonzero_pulses"
    elif not command_seen:
        verdict = "command_not_seen_on_hw_cmd_vel"
    elif not response_seen:
        verdict = "drive_response_missing_or_below_threshold"
    elif not heading_seen:
        verdict = "motion_seen_but_xbot_heading_not_tracking"
    else:
        verdict = "drive_and_xbot_heading_response_observed"

    return {
        "schema": "open_mower.coverage_lab.rotate_response.v0",
        "verdict": verdict,
        "min_response_wz": min_response_wz,
        "pulses": summaries,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=pathlib.Path, help="JSONL output path")
    parser.add_argument("--summary-output", type=pathlib.Path, help="summary JSON output path")
    parser.add_argument("--pulse", action="append", type=_parse_pulse, help="ANGULAR_Z:DURATION_SECONDS; may be repeated")
    parser.add_argument("--hz", type=float, default=20.0, help="command publish rate")
    parser.add_argument("--settle-seconds", type=float, default=1.0, help="zero-command settle time before the first pulse")
    parser.add_argument("--wait-timeout", type=float, default=5.0, help="seconds to wait for state/emergency topics")
    parser.add_argument("--max-abs-angular-z", type=float, default=0.8, help="largest permitted absolute angular command")
    parser.add_argument("--min-response-wz", type=float, default=0.05, help="minimum measured yaw rate treated as motion")
    parser.add_argument("--topic", default="/override_vel", help="command topic")
    parser.add_argument("--allow-non-idle", action="store_true", help="allow execution when mower state is not IDLE")
    parser.add_argument("--execute", action="store_true", help="actually publish motion commands")
    parser.add_argument("--confirm-clearance", action="store_true", help="confirm physical area is clear for rotate motion")
    parser.add_argument("--node-name", default="v2_rotate_response_test", help="ROS node name")
    args = parser.parse_args()

    pulses = args.pulse or [_parse_pulse(value) for value in DEFAULT_PULSES]
    if args.hz <= 0.0:
        raise SystemExit("--hz must be > 0")
    for pulse in pulses:
        if abs(pulse.angular_z) > args.max_abs_angular_z:
            raise SystemExit(
                f"pulse angular_z {pulse.angular_z} exceeds --max-abs-angular-z {args.max_abs_angular_z}"
            )

    stamp = time.strftime("%Y%m%d-%H%M%S")
    output = args.output or pathlib.Path("tools/coverage_lab/runs/live-traces") / f"rotate-response-{stamp}.jsonl"
    summary_output = args.summary_output or output.with_suffix(".summary.json")

    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node(args.node_name, anonymous=False)
    recorder = RotateResponseRecorder(output)
    recorder.open()
    pub = rospy.Publisher(args.topic, Twist, queue_size=1)
    subscribers = _subscribe(recorder)
    recorder.record_event(
        "start",
        {
            "execute": args.execute,
            "confirm_clearance": args.confirm_clearance,
            "topic": args.topic,
            "pulses": [{"angular_z": pulse.angular_z, "duration_s": pulse.duration_s} for pulse in pulses],
        },
    )

    summary: dict[str, Any]
    try:
        _wait_for_initial_state(recorder, args.wait_timeout)
        _assert_safe_to_execute(recorder, args.allow_non_idle)
        if not args.execute:
            summary = {
                "schema": "open_mower.coverage_lab.rotate_response.v0",
                "verdict": "dry_run_no_motion",
                "output": str(output),
                "state": recorder.latest.get("/mower_logic/current_state", {}).get("derived", {}),
                "emergency": recorder.latest.get("/hw/emergency", {}).get("derived", {}),
            }
        elif not args.confirm_clearance:
            raise RuntimeError("--execute requires --confirm-clearance")
        else:
            rospy.sleep(0.2)
            _publish_for(pub, recorder, 0.0, args.settle_seconds, args.hz)
            for index, pulse in enumerate(pulses):
                pulse.start_t = _now()
                recorder.record_event(
                    "pulse_start",
                    {"index": index, "angular_z": pulse.angular_z, "duration_s": pulse.duration_s},
                )
                _publish_for(pub, recorder, pulse.angular_z, pulse.duration_s, args.hz)
                pulse.end_t = _now()
                recorder.record_event("pulse_end", {"index": index})
            _publish_for(pub, recorder, 0.0, 1.0, args.hz)
            summary = _summarize_pulses(recorder, pulses, args.min_response_wz)
            summary["output"] = str(output)
    finally:
        try:
            _publish_for(pub, recorder, 0.0, 0.5, args.hz)
        except Exception:
            pass
        for subscriber in subscribers:
            subscriber.unregister()
        recorder.close()

    summary_output.parent.mkdir(parents=True, exist_ok=True)
    summary_output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Wrote {output}")
    print(f"Wrote {summary_output}")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
