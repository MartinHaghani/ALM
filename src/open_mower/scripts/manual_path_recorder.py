#!/usr/bin/env python3
"""Passive manual mowing path recorder.

This node records what happened while an operator manually mowed an area.  It
never publishes drive, blade, action, or hardware command topics.
"""

import html
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import Counter
from pathlib import Path

try:
    import rospy
    import rostopic
    import tf
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
except ImportError:  # pragma: no cover - lets pure export helpers be imported off-ROS.
    rospy = None
    rostopic = None
    tf = None
    String = None
    SetBool = None
    SetBoolResponse = None
    Trigger = None
    TriggerResponse = None


SCHEMA_SESSION = "open_mower.manual_path_recording.session.v0"
SCHEMA_SAMPLE = "open_mower.manual_path_recording.sample.v0"
SCHEMA_TEACHER_PATH = "open_mower.manual_teacher_path.v0"
SCHEMA_PLANPATH_COMPAT = "open_mower.planpath_compat.v0"

FLAG_GPS_RTK_FIXED = 2
FLAG_GPS_RTK_FLOAT = 4
FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE = 1

DEFAULT_TOPICS = [
    "/hw/position/gps",
    "/hw/position/gps/fix",
    "/xbot_positioning/xb_pose",
    "/localization_fusion/pose",
    "/localization_fusion/status",
    "/slam_toolbox_alignment/status",
    "/hw/diff_drive/measured_twist",
    "/hw/cmd_vel",
    "/joy_vel",
    "/web_joy_vel",
    "/direct_joy_vel",
    "/mower_logic/current_state",
    "/hw/status",
    "/hw/power",
    "/hw/emergency",
    "/mower_input/status",
    "/area_recorder/boundary_samples",
    "/tf",
    "/tf_static",
]

DEFAULT_RAW_BAG_TOPICS = [
    "/hw/position/gps",
    "/hw/position/gps/fix",
    "/xbot_positioning/xb_pose",
    "/localization_fusion/pose",
    "/localization_fusion/odom",
    "/localization_fusion/status",
    "/slam_toolbox_alignment/status",
    "/slam_toolbox/local_odom",
    "/slam_toolbox/map",
    "/slam_toolbox/scan",
    "/hw/lidar",
    "/hw/diff_drive/measured_twist",
    "/hw/cmd_vel",
    "/joy_vel",
    "/web_joy_vel",
    "/direct_joy_vel",
    "/mower_logic/current_state",
    "/hw/status",
    "/hw/power",
    "/hw/emergency",
    "/mower_input/status",
    "/area_recorder/boundary_samples",
    "/tf",
    "/tf_static",
]

DEFAULT_TOPIC_HZ_LIMITS = {
    "/tf": 2.0,
    "/tf_static": 0.2,
    "/hw/lidar": 1.0,
    "/slam_toolbox/scan": 1.0,
    "/slam_toolbox/map": 0.2,
}


def clean_topic_list(value, fallback):
    if isinstance(value, str):
        items = [entry.strip() for entry in value.split(",")]
    else:
        items = [str(entry).strip() for entry in value or []]
    return [entry for entry in items if entry] or list(fallback)


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def finite_pose(pose):
    return bool(pose) and finite(pose.get("x")) and finite(pose.get("y")) and finite(pose.get("yaw"))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_delta(a, b):
    return abs(normalize_angle(float(a) - float(b)))


def distance(a, b):
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def yaw_from_quaternion(rotation):
    if tf is None:
        return 0.0
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    return tf.transformations.euler_from_quaternion(quat)[2]


def stamp_to_sec(stamp):
    if stamp is None:
        return None
    try:
        if stamp.secs == 0 and stamp.nsecs == 0:
            return None
        return float(stamp.secs) + float(stamp.nsecs) / 1e9
    except AttributeError:
        try:
            value = float(stamp)
            return value if value > 0.0 else None
        except (TypeError, ValueError):
            return None


def message_stamp(msg):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    value = stamp_to_sec(stamp)
    if value is not None:
        return value

    stamp = getattr(msg, "stamp", None)
    return stamp_to_sec(stamp)


def now_wall():
    return time.time()


def safe_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def primitive(value, depth=0):
    if depth > 5:
        return str(type(value).__name__)
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [primitive(entry, depth + 1) for entry in value[:500]]
    if isinstance(value, dict):
        return {str(key): primitive(entry, depth + 1) for key, entry in value.items()}
    if hasattr(value, "secs") and hasattr(value, "nsecs"):
        return {"secs": int(value.secs), "nsecs": int(value.nsecs)}
    if hasattr(value, "__slots__"):
        return {slot: primitive(getattr(value, slot), depth + 1) for slot in value.__slots__}
    return str(value)


def pose_from_pose_msg(pose):
    return {
        "x": safe_float(pose.position.x),
        "y": safe_float(pose.position.y),
        "yaw": safe_float(yaw_from_quaternion(pose.orientation)),
    }


def pose_from_absolute_pose(msg):
    pose = pose_from_pose_msg(msg.pose.pose)
    pose.update(
        {
            "frame_id": getattr(getattr(msg, "header", None), "frame_id", ""),
            "flags": int(getattr(msg, "flags", 0)),
            "orientation_accuracy": safe_float(getattr(msg, "orientation_accuracy", None)),
            "orientation_valid": bool(getattr(msg, "orientation_valid", False)),
            "position_accuracy": safe_float(getattr(msg, "position_accuracy", None)),
            "source": int(getattr(msg, "source", 0)),
        }
    )
    sensor_stamp = safe_float(getattr(msg, "sensor_stamp", None))
    received_stamp = safe_float(getattr(msg, "received_stamp", None))
    if sensor_stamp is not None:
        pose["sensor_stamp"] = sensor_stamp
    if received_stamp is not None:
        pose["received_stamp"] = received_stamp
    return pose


def pose_from_pose_stamped(msg):
    pose = pose_from_pose_msg(msg.pose)
    pose["frame_id"] = getattr(getattr(msg, "header", None), "frame_id", "")
    return pose


def twist_derived(msg):
    twist = getattr(msg, "twist", msg)
    if hasattr(twist, "twist"):
        twist = twist.twist
    return {
        "angular_z": safe_float(getattr(twist.angular, "z", None)),
        "linear_x": safe_float(getattr(twist.linear, "x", None)),
        "linear_y": safe_float(getattr(twist.linear, "y", None)),
    }


def parse_json_string(msg):
    try:
        payload = json.loads(msg.data)
        return payload if isinstance(payload, dict) else {"payload": payload}
    except (AttributeError, TypeError, ValueError):
        return {"raw": getattr(msg, "data", "")}


def navsat_derived(msg):
    covariance = list(getattr(msg, "position_covariance", []) or [])
    horizontal_sigma = None
    if len(covariance) >= 5 and covariance[0] >= 0.0 and covariance[4] >= 0.0:
        horizontal_sigma = math.sqrt(max(0.0, covariance[0]) + max(0.0, covariance[4]))
    return {
        "altitude": safe_float(getattr(msg, "altitude", None)),
        "horizontal_sigma_m": horizontal_sigma,
        "latitude": safe_float(getattr(msg, "latitude", None)),
        "longitude": safe_float(getattr(msg, "longitude", None)),
        "status": int(getattr(getattr(msg, "status", None), "status", -999)),
    }


def state_derived(msg):
    return {
        "battery_percent": safe_float(getattr(msg, "battery_percent", None)),
        "current_area": int(getattr(msg, "current_area", -1)),
        "current_path": int(getattr(msg, "current_path", -1)),
        "current_path_index": int(getattr(msg, "current_path_index", -1)),
        "emergency": bool(getattr(msg, "emergency", False)),
        "gps_quality_percent": safe_float(getattr(msg, "gps_quality_percent", None)),
        "is_charging": bool(getattr(msg, "is_charging", False)),
        "state": int(getattr(msg, "state", 0)),
        "state_name": getattr(msg, "state_name", ""),
        "sub_state_name": getattr(msg, "sub_state_name", ""),
    }


def hw_status_derived(msg):
    return {
        "esc_power": bool(getattr(msg, "esc_power", False)),
        "hardware_status": int(getattr(msg, "hardware_status", 0)),
        "mow_enabled": bool(getattr(msg, "mow_enabled", False)),
        "mower_motor_rpm": safe_float(getattr(msg, "mower_motor_rpm", None)),
        "warning": getattr(msg, "warning", ""),
    }


def hw_power_derived(msg):
    return {
        "esc_power": bool(getattr(msg, "esc_power", False)),
        "is_charging": bool(getattr(msg, "is_charging", False)),
        "mow_enabled": bool(getattr(msg, "mow_enabled", False)),
        "raspberry_pi_power": bool(getattr(msg, "raspberry_pi_power", False)),
        "rain_detected": bool(getattr(msg, "rain_detected", False)),
    }


def emergency_derived(msg):
    return {
        "active_emergency": bool(getattr(msg, "active_emergency", False)),
        "latched_emergency": bool(getattr(msg, "latched_emergency", False)),
        "reason": getattr(msg, "reason", ""),
    }


def tf_derived(msg):
    transforms = getattr(msg, "transforms", []) or []
    return {
        "transforms": [
            {
                "child_frame_id": getattr(transform, "child_frame_id", ""),
                "frame_id": getattr(getattr(transform, "header", None), "frame_id", ""),
                "stamp": message_stamp(transform),
            }
            for transform in transforms[:50]
        ],
        "transform_count": len(transforms),
    }


def derived_for_topic(topic, msg):
    if topic in ("/hw/position/gps", "/xbot_positioning/xb_pose", "/localization_fusion/pose"):
        return pose_from_absolute_pose(msg)
    if topic == "/hw/position/gps/fix":
        return navsat_derived(msg)
    if topic == "/localization_fusion/status":
        return parse_json_string(msg)
    if topic == "/slam_toolbox_alignment/status":
        return parse_json_string(msg)
    if topic in ("/hw/diff_drive/measured_twist", "/hw/cmd_vel", "/joy_vel", "/web_joy_vel", "/direct_joy_vel"):
        return twist_derived(msg)
    if topic == "/mower_logic/current_state":
        return state_derived(msg)
    if topic == "/hw/status":
        return hw_status_derived(msg)
    if topic == "/hw/power":
        return hw_power_derived(msg)
    if topic == "/hw/emergency":
        return emergency_derived(msg)
    if topic == "/mower_input/status":
        return parse_json_string(msg)
    if topic in ("/tf", "/tf_static"):
        return tf_derived(msg)
    return {}


def gps_flags_are_fixed(pose):
    return bool(pose and int(pose.get("flags", 0)) & FLAG_GPS_RTK_FIXED)


def gps_flags_state(pose):
    if not pose:
        return "missing"
    flags = int(pose.get("flags", 0))
    if flags & FLAG_GPS_RTK_FIXED:
        return "fixed"
    if flags & FLAG_GPS_RTK_FLOAT:
        return "float"
    return "not_fixed"


def fusion_active_sources(fusion_status):
    if not fusion_status:
        return []
    direct = fusion_status.get("active_sources")
    if isinstance(direct, list):
        return [str(entry) for entry in direct]
    sources = fusion_status.get("sources") or {}
    active = []
    if isinstance(sources, dict):
        for name, status in sources.items():
            if isinstance(status, dict) and status.get("accepted"):
                active.append(str(name))
    weights = fusion_status.get("source_weights") or fusion_status.get("accepted_source_weights") or {}
    if isinstance(weights, dict):
        for name, weight in weights.items():
            if safe_float(weight) and safe_float(weight) > 0.0 and str(name) not in active:
                active.append(str(name))
    return active


def source_health_from_snapshot(snapshot, now_s, max_pose_age_s):
    health = {}
    for name, topic in (
        ("gps_raw", "/hw/position/gps"),
        ("gps_base", "/xbot_positioning/xb_pose"),
        ("fused", "/localization_fusion/pose"),
        ("gps_fix", "/hw/position/gps/fix"),
        ("slam_alignment", "/slam_toolbox_alignment/status"),
        ("fusion_status", "/localization_fusion/status"),
    ):
        entry = snapshot.get(topic) or {}
        age = None if entry.get("wall_time") is None else max(0.0, now_s - entry["wall_time"])
        health[name] = {
            "age_s": age,
            "fresh": age is not None and age <= max_pose_age_s,
            "seen": bool(entry),
            "stamp": entry.get("stamp"),
        }
    return health


def selected_command(snapshot):
    for topic in ("/web_joy_vel", "/direct_joy_vel", "/joy_vel", "/hw/cmd_vel"):
        entry = snapshot.get(topic)
        if entry and entry.get("derived"):
            return {"source_topic": topic, **entry["derived"]}
    return None


def current_blade_state(snapshot):
    hw_status = (snapshot.get("/hw/status") or {}).get("derived") or {}
    hw_power = (snapshot.get("/hw/power") or {}).get("derived") or {}
    return {
        "mow_enabled": bool(hw_status.get("mow_enabled") or hw_power.get("mow_enabled")),
        "mower_motor_rpm": hw_status.get("mower_motor_rpm"),
    }


def source_comparison(gps_base, lidar_map, fused, fusion_status, slam_status):
    comparison = {
        "gps_lidar_separation_m": None,
        "slam_alignment_separation_m": None,
        "fusion_state": None,
        "fusion_ready": None,
        "active_sources": [],
    }
    if gps_base and lidar_map and finite_pose(gps_base) and finite_pose(lidar_map):
        comparison["gps_lidar_separation_m"] = distance(gps_base, lidar_map)
    if isinstance(fusion_status, dict):
        comparison["fusion_state"] = fusion_status.get("state")
        comparison["fusion_ready"] = fusion_status.get("ready_for_navigation")
        comparison["active_sources"] = fusion_active_sources(fusion_status)
        if fusion_status.get("gps_lidar_separation_m") is not None:
            comparison["gps_lidar_separation_m"] = safe_float(fusion_status.get("gps_lidar_separation_m"))
    if isinstance(slam_status, dict):
        comparison["slam_alignment_separation_m"] = safe_float(slam_status.get("separation_m"))
    return comparison


def replay_reject_reasons(sample, max_pose_age_s, max_source_separation_m):
    reasons = []
    fused = sample.get("fused")
    gps_raw = sample.get("gps_raw")
    fusion_status = sample.get("fusion_status") or {}
    slam_status = sample.get("slam_alignment") or {}
    source_quality = sample.get("source_quality") or {}
    source_health = sample.get("source_health") or {}
    mower_state = sample.get("mower_state") or {}

    fused_health = source_health.get("fused") or {}
    if not finite_pose(fused):
        reasons.append("fused_missing")
    elif not fused_health.get("fresh"):
        reasons.append("fused_stale")
    elif fused_health.get("age_s") is not None and fused_health.get("age_s") > max_pose_age_s:
        reasons.append("fused_stale")

    state_name = str(mower_state.get("state_name") or "").upper()
    if state_name and state_name != "AREA_RECORDING":
        reasons.append("not_area_recording")
    elif not state_name:
        reasons.append("mower_state_missing")

    active_sources = source_quality.get("active_sources") or fusion_active_sources(fusion_status)
    if not active_sources:
        active_sources = ["gps", "lidar"]

    fusion_ready = fusion_status.get("ready_for_navigation")
    if fusion_ready is False:
        reasons.append("fused_not_ready")

    if "gps" in active_sources and not gps_flags_are_fixed(gps_raw):
        reasons.append("gps_not_rtk_fixed")

    if "lidar" in active_sources:
        slam_state = str(slam_status.get("state") or "")
        aligned = bool(slam_status.get("aligned"))
        if not slam_status:
            reasons.append("lidar_status_missing")
        elif not aligned and slam_state not in ("aligned", "aligned_rtk_hold"):
            reasons.append("lidar_unaligned")

    separation = source_quality.get("gps_lidar_separation_m")
    if separation is not None and finite(separation) and separation > max_source_separation_m:
        reasons.append("source_conflict")

    emergency = sample.get("emergency") or {}
    if emergency.get("active_emergency") or emergency.get("latched_emergency") or mower_state.get("emergency"):
        reasons.append("emergency_active")

    return sorted(set(reasons))


def sync_sample_from_snapshot(snapshot, now_s, config):
    gps_raw = ((snapshot.get("/hw/position/gps") or {}).get("derived") or None)
    gps_fix = ((snapshot.get("/hw/position/gps/fix") or {}).get("derived") or None)
    gps_base = ((snapshot.get("/xbot_positioning/xb_pose") or {}).get("derived") or None)
    fused = ((snapshot.get("/localization_fusion/pose") or {}).get("derived") or None)
    fusion_status = ((snapshot.get("/localization_fusion/status") or {}).get("derived") or None)
    slam_status = ((snapshot.get("/slam_toolbox_alignment/status") or {}).get("derived") or None)
    measured_twist = ((snapshot.get("/hw/diff_drive/measured_twist") or {}).get("derived") or None)
    mower_state = ((snapshot.get("/mower_logic/current_state") or {}).get("derived") or None)
    emergency = ((snapshot.get("/hw/emergency") or {}).get("derived") or None)
    blade = current_blade_state(snapshot)
    lidar_map = None
    if isinstance(slam_status, dict):
        lidar_map = slam_status.get("lidar_pose")

    source_quality = source_comparison(gps_base, lidar_map, fused, fusion_status, slam_status)
    health = source_health_from_snapshot(snapshot, now_s, config["max_pose_age_s"])
    sample = {
        "accepted_for_replay": False,
        "blade": blade,
        "command": selected_command(snapshot),
        "emergency": emergency or {},
        "fused": fused,
        "fusion_status": fusion_status or {},
        "gps_base": gps_base,
        "gps_fix": gps_fix,
        "gps_raw": gps_raw,
        "lidar_map": lidar_map,
        "measured_twist": measured_twist,
        "mower_state": mower_state or {},
        "reject_reasons": [],
        "selected_pose": fused,
        "selected_pose_source": "fused",
        "slam_alignment": slam_status or {},
        "source_health": health,
        "source_quality": source_quality,
        "stamp": now_s,
        "type": "sync_sample",
    }
    reasons = replay_reject_reasons(sample, config["max_pose_age_s"], config["max_source_separation_m"])
    sample["reject_reasons"] = reasons
    sample["accepted_for_replay"] = not reasons
    return sample


def downsample_segment_samples(samples, min_step_m, min_yaw_step_rad):
    if not samples:
        return []

    poses = []
    last_pose = None
    for index, sample in enumerate(samples):
        pose = sample.get("selected_pose") or {}
        if not finite_pose(pose):
            continue
        keep = False
        if last_pose is None:
            keep = True
        elif index == len(samples) - 1:
            keep = True
        elif distance(pose, last_pose) >= min_step_m:
            keep = True
        elif angle_delta(pose.get("yaw", 0.0), last_pose.get("yaw", 0.0)) >= min_yaw_step_rad:
            keep = True
        if not keep:
            continue
        poses.append(
            {
                "accepted_for_replay": bool(sample.get("accepted_for_replay")),
                "blade_state": "ON" if sample.get("blade", {}).get("mow_enabled") else "OFF",
                "gps_lidar_separation_m": sample.get("source_quality", {}).get("gps_lidar_separation_m"),
                "position_accuracy_m": pose.get("position_accuracy"),
                "source": sample.get("selected_pose_source", "fused"),
                "stamp": sample.get("stamp"),
                "x": pose.get("x"),
                "y": pose.get("y"),
                "yaw": pose.get("yaw"),
            }
        )
        last_pose = pose
    return poses


def segment_length(poses):
    total = 0.0
    previous = None
    for pose in poses:
        if previous is not None:
            total += distance(previous, pose)
        previous = pose
    return total


def has_operator_event_between(events, previous_stamp, current_stamp):
    for event in events:
        stamp = event.get("stamp")
        if stamp is None:
            continue
        if previous_stamp is None:
            if stamp <= current_stamp:
                return True
        elif previous_stamp < stamp <= current_stamp:
            return True
    return False


def close_segment(segments, current, break_reason, config):
    if not current:
        return []
    poses = downsample_segment_samples(current, config["min_pose_step_m"], config["min_yaw_step_rad"])
    if not poses:
        return []
    first = current[0]
    last = current[-1]
    blade_on = bool(first.get("blade", {}).get("mow_enabled"))
    segment = {
        "allowed_for_live_replay": False,
        "blade_state": "ON" if blade_on else "OFF",
        "break_after_reason": break_reason,
        "controller_mode": "FOLLOW_PATH_DIAGNOSTIC_ONLY",
        "direction": "FORWARD",
        "duration_s": max(0.0, float(last.get("stamp", 0.0)) - float(first.get("stamp", 0.0))),
        "end_stamp": last.get("stamp"),
        "frame_id": config["frame_id"],
        "length_m": segment_length(poses),
        "maneuver_kind": "MANUAL_MOW" if blade_on else "MANUAL_TRANSIT",
        "path_index": len(segments),
        "pose_count": len(poses),
        "poses": poses,
        "purpose": "CUT" if blade_on else "MOVE",
        "source": "manual_path_recorder",
        "source_pose": "fused",
        "start_stamp": first.get("stamp"),
    }
    segments.append(segment)
    return []


def build_teacher_path(session_meta, sync_samples, events, config):
    accepted = [sample for sample in sync_samples if sample.get("accepted_for_replay") and finite_pose(sample.get("selected_pose"))]
    segments = []
    current = []
    previous = None
    event_marks = [event for event in events if event.get("event") == "mark"]
    split_counts = Counter()

    for sample in sync_samples:
        if not sample.get("accepted_for_replay") or not finite_pose(sample.get("selected_pose")):
            if current:
                reason = "rejected_span"
                split_counts[reason] += 1
                current = close_segment(segments, current, reason, config)
            previous = None
            continue

        reason = None
        if previous is not None:
            prev_pose = previous.get("selected_pose") or {}
            pose = sample.get("selected_pose") or {}
            gap_s = float(sample.get("stamp", 0.0)) - float(previous.get("stamp", 0.0))
            jump_m = distance(prev_pose, pose)
            yaw_jump = angle_delta(prev_pose.get("yaw", 0.0), pose.get("yaw", 0.0))
            prev_blade = bool(previous.get("blade", {}).get("mow_enabled"))
            blade = bool(sample.get("blade", {}).get("mow_enabled"))
            prev_source = previous.get("selected_pose_source")
            source = sample.get("selected_pose_source")
            if has_operator_event_between(event_marks, previous.get("stamp"), sample.get("stamp")):
                reason = "operator_event"
            elif gap_s > config["max_gap_s"]:
                reason = "stale_pose_gap"
            elif jump_m > config["max_pose_jump_m"]:
                reason = "pose_jump"
            elif yaw_jump > config["max_yaw_jump_rad"]:
                reason = "yaw_jump"
            elif prev_blade != blade:
                reason = "blade_state_change"
            elif prev_source != source:
                reason = "source_change"

        if reason:
            split_counts[reason] += 1
            current = close_segment(segments, current, reason, config)

        current.append(sample)
        previous = sample

    current = close_segment(segments, current, "end_of_capture", config)

    total_duration = 0.0
    if sync_samples:
        total_duration = max(0.0, float(sync_samples[-1].get("stamp", 0.0)) - float(sync_samples[0].get("stamp", 0.0)))

    reject_counts = Counter()
    separation_values = []
    for sample in sync_samples:
        reject_counts.update(sample.get("reject_reasons") or [])
        separation = sample.get("source_quality", {}).get("gps_lidar_separation_m")
        if finite(separation):
            separation_values.append(float(separation))

    teacher_path = {
        "created_at": session_meta.get("ended_at") or session_meta.get("started_at"),
        "events": events,
        "frame_id": config["frame_id"],
        "quality": {
            "accepted_sample_count": len(accepted),
            "max_gps_lidar_separation_m": max(separation_values) if separation_values else None,
            "mean_gps_lidar_separation_m": (sum(separation_values) / len(separation_values)) if separation_values else None,
            "reject_counts": dict(reject_counts),
            "sample_count": len(sync_samples),
            "segment_split_counts": dict(split_counts),
        },
        "schema": SCHEMA_TEACHER_PATH,
        "segments": segments,
        "session_id": session_meta.get("session_id"),
        "source": "manual_path_recorder",
        "total_duration_s": total_duration,
        "version": 1,
    }
    return teacher_path


def build_planpath_compat(teacher_path, min_poses=3):
    paths = []
    skipped = []
    for segment in teacher_path.get("segments", []):
        reasons = []
        if segment.get("blade_state") != "ON":
            reasons.append("blade_off")
        if segment.get("direction") != "FORWARD":
            reasons.append("unsupported_direction")
        if len(segment.get("poses") or []) < min_poses:
            reasons.append("too_few_poses")
        if segment.get("break_after_reason") in ("pose_jump", "yaw_jump", "stale_pose_gap", "rejected_span"):
            reasons.append("unsafe_break")
        if reasons:
            skipped.append(
                {
                    "path_index": segment.get("path_index"),
                    "reasons": sorted(set(reasons)),
                    "source_pose_count": len(segment.get("poses") or []),
                }
            )
            continue
        poses = [
            {
                "pose_index": index,
                "x": pose["x"],
                "y": pose["y"],
                "yaw": pose["yaw"],
            }
            for index, pose in enumerate(segment.get("poses") or [])
        ]
        paths.append(
            {
                "is_outline": False,
                "label": "manual segment %s" % segment.get("path_index", len(paths)),
                "path": {
                    "frame_id": segment.get("frame_id", teacher_path.get("frame_id", "map")),
                    "poses": poses,
                },
            }
        )
    return {
        "active": False,
        "current_path_index": 0,
        "current_pose_index": 0,
        "frame_id": teacher_path.get("frame_id", "map"),
        "paths": paths,
        "profile": "manual_teacher_fused_forward",
        "schema": SCHEMA_PLANPATH_COMPAT,
        "session_id": teacher_path.get("session_id"),
        "skipped_segments": skipped,
        "source": "manual_path_recorder",
        "warning": "Compatibility export contains only clean forward blade-on base_link poses and is diagnostic-only.",
    }


def build_source_comparison_report(sync_samples, teacher_path):
    reject_counts = Counter()
    source_states = Counter()
    separation_values = []
    accepted = 0
    for sample in sync_samples:
        if sample.get("accepted_for_replay"):
            accepted += 1
        reject_counts.update(sample.get("reject_reasons") or [])
        state = sample.get("source_quality", {}).get("fusion_state")
        if state:
            source_states[str(state)] += 1
        separation = sample.get("source_quality", {}).get("gps_lidar_separation_m")
        if finite(separation):
            separation_values.append(float(separation))
    return {
        "accepted_sample_count": accepted,
        "fusion_state_counts": dict(source_states),
        "max_gps_lidar_separation_m": max(separation_values) if separation_values else None,
        "mean_gps_lidar_separation_m": (sum(separation_values) / len(separation_values)) if separation_values else None,
        "reject_counts": dict(reject_counts),
        "sample_count": len(sync_samples),
        "segment_count": len(teacher_path.get("segments", [])),
    }


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    tmp_path.replace(path)


def format_float(value, digits=2):
    return "--" if value is None or not finite(value) else ("%%.%df" % digits) % float(value)


def write_html_report(path, session_meta, teacher_path, source_report, planpath_compat):
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    rows.append(("Session", session_meta.get("session_id")))
    rows.append(("Started", session_meta.get("started_at")))
    rows.append(("Ended", session_meta.get("ended_at")))
    rows.append(("Duration", "%s s" % format_float(session_meta.get("duration_s"), 1)))
    rows.append(("Samples", source_report.get("sample_count")))
    rows.append(("Accepted", source_report.get("accepted_sample_count")))
    rows.append(("Segments", source_report.get("segment_count")))
    rows.append(("PlanPath paths", len(planpath_compat.get("paths", []))))
    rows.append(("Skipped PlanPath segments", len(planpath_compat.get("skipped_segments", []))))
    rows.append(("Mean GPS/LIDAR separation", "%s m" % format_float(source_report.get("mean_gps_lidar_separation_m"), 3)))

    def table_rows(entries):
        return "\n".join(
            "<tr><th>%s</th><td>%s</td></tr>"
            % (html.escape(str(key)), html.escape(str(value)))
            for key, value in entries
        )

    segment_rows = []
    for segment in teacher_path.get("segments", []):
        segment_rows.append(
            "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>"
            % (
                html.escape(str(segment.get("path_index"))),
                html.escape(str(segment.get("purpose"))),
                html.escape(str(segment.get("blade_state"))),
                html.escape(str(segment.get("pose_count"))),
                html.escape(format_float(segment.get("length_m"), 2)),
                html.escape(str(segment.get("break_after_reason"))),
            )
        )

    reject_rows = table_rows(sorted((source_report.get("reject_counts") or {}).items()))
    skip_rows = []
    for skipped in planpath_compat.get("skipped_segments", []):
        skip_rows.append(
            "<tr><td>%s</td><td>%s</td><td>%s</td></tr>"
            % (
                html.escape(str(skipped.get("path_index"))),
                html.escape(", ".join(skipped.get("reasons") or [])),
                html.escape(str(skipped.get("source_pose_count"))),
            )
        )

    document = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Manual Path Recording Summary</title>
  <style>
    body { margin: 24px; color: #17202a; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    h1, h2 { margin: 0 0 12px; }
    section { margin: 0 0 24px; }
    table { width: 100%%; border-collapse: collapse; }
    th, td { padding: 8px 10px; border: 1px solid #d9e1ea; text-align: left; vertical-align: top; }
    th { width: 220px; background: #f5f7fa; }
    .warning { padding: 10px 12px; border: 1px solid #f1c40f; background: #fff8df; }
  </style>
</head>
<body>
  <h1>Manual Path Recording Summary</h1>
  <section class="warning">This report is diagnostic. The compatibility export excludes unsupported reverse, pivot, stale, high-disagreement, blade-off, and too-short spans.</section>
  <section>
    <h2>Session</h2>
    <table>%s</table>
  </section>
  <section>
    <h2>Teacher Path Segments</h2>
    <table>
      <tr><th>Index</th><th>Purpose</th><th>Blade</th><th>Poses</th><th>Length m</th><th>Break</th></tr>
      %s
    </table>
  </section>
  <section>
    <h2>Reject Reasons</h2>
    <table>%s</table>
  </section>
  <section>
    <h2>Skipped PlanPath Compatibility Spans</h2>
    <table>
      <tr><th>Segment</th><th>Reasons</th><th>Source Poses</th></tr>
      %s
    </table>
  </section>
</body>
</html>
""" % (
        table_rows(rows),
        "\n".join(segment_rows) or "<tr><td colspan=\"6\">No accepted teacher-path segments.</td></tr>",
        reject_rows or "<tr><th>None</th><td>0</td></tr>",
        "\n".join(skip_rows) or "<tr><td colspan=\"3\">No skipped compatibility spans.</td></tr>",
    )
    path.write_text(document, encoding="utf-8")


class ManualPathRecorder:
    def __init__(self):
        if rospy is None or rostopic is None:
            raise RuntimeError("manual_path_recorder requires ROS Python packages")

        rospy.init_node("manual_path_recorder")
        self.recordings_path = Path(rospy.get_param("~recordings_path", os.path.expanduser("~/.ros/path_recordings"))).expanduser()
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.sync_rate_hz = float(rospy.get_param("~sync_rate_hz", 10.0))
        self.max_pose_age_s = float(rospy.get_param("~max_pose_age_s", 0.75))
        self.max_gap_s = float(rospy.get_param("~max_gap_s", 1.0))
        self.max_pose_jump_m = float(rospy.get_param("~max_pose_jump_m", 0.75))
        self.max_yaw_jump_rad = float(rospy.get_param("~max_yaw_jump_rad", math.radians(65.0)))
        self.max_source_separation_m = float(rospy.get_param("~max_source_separation_m", 0.45))
        self.min_pose_step_m = float(rospy.get_param("~min_pose_step_m", 0.10))
        self.min_yaw_step_rad = float(rospy.get_param("~min_yaw_step_rad", math.radians(5.0)))
        self.topics = clean_topic_list(rospy.get_param("~topics", DEFAULT_TOPICS), DEFAULT_TOPICS)
        self.raw_bag_topics = clean_topic_list(rospy.get_param("~raw_bag_topics", DEFAULT_RAW_BAG_TOPICS), DEFAULT_RAW_BAG_TOPICS)
        self.topic_hz_limits = dict(DEFAULT_TOPIC_HZ_LIMITS)
        self.topic_hz_limits.update(rospy.get_param("~topic_hz_limits", {}) or {})

        self.lock = threading.RLock()
        self.write_lock = threading.RLock()
        self.latest = {}
        self.last_record_wall_by_topic = {}
        self.subscribers = {}
        self.active = False
        self.raw_bag_active = False
        self.raw_bag_process = None
        self.session_id = None
        self.session_dir = None
        self.session_meta = None
        self.samples_handle = None
        self.sync_samples = []
        self.events = []
        self.sample_counts = Counter()
        self.reject_counts = Counter()
        self.last_error = None
        self.last_artifacts = {}

        self.status_pub = rospy.Publisher("/manual_path_recorder/status", String, queue_size=1, latch=True)
        self.start_service = rospy.Service("/manual_path_recorder/start", SetBool, self.start_capture)
        self.stop_service = rospy.Service("/manual_path_recorder/stop", Trigger, self.stop_capture)
        self.mark_service = rospy.Service("/manual_path_recorder/mark_event", Trigger, self.mark_event)
        self.export_service = rospy.Service("/manual_path_recorder/export", Trigger, self.export_capture)

        self.discovery_timer = rospy.Timer(rospy.Duration(2.0), self.discover_topics)
        self.sync_timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.sync_rate_hz)), self.write_sync_sample)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        rospy.on_shutdown(self.on_shutdown)
        self.discover_topics(None)
        self.publish_status(None)
        rospy.loginfo("Manual path recorder ready. Recordings path: %s", self.recordings_path)

    def config(self):
        return {
            "frame_id": self.frame_id,
            "max_gap_s": self.max_gap_s,
            "max_pose_age_s": self.max_pose_age_s,
            "max_pose_jump_m": self.max_pose_jump_m,
            "max_source_separation_m": self.max_source_separation_m,
            "max_yaw_jump_rad": self.max_yaw_jump_rad,
            "min_pose_step_m": self.min_pose_step_m,
            "min_yaw_step_rad": self.min_yaw_step_rad,
        }

    def discover_topics(self, _event):
        for topic in self.topics:
            if topic in self.subscribers:
                continue
            try:
                message_class, _, _ = rostopic.get_topic_class(topic, blocking=False)
            except Exception as exc:
                self.last_error = "topic discovery failed for %s: %s" % (topic, exc)
                continue
            if message_class is None:
                continue
            self.subscribers[topic] = rospy.Subscriber(
                topic,
                message_class,
                lambda msg, topic=topic: self.on_topic(topic, msg),
                queue_size=20,
            )
            rospy.loginfo("manual_path_recorder subscribed to %s", topic)

    def should_record_topic(self, topic, wall_time):
        limit = safe_float(self.topic_hz_limits.get(topic))
        if limit is None or limit <= 0.0:
            return True
        previous = self.last_record_wall_by_topic.get(topic)
        if previous is not None and wall_time - previous < 1.0 / limit:
            return False
        self.last_record_wall_by_topic[topic] = wall_time
        return True

    def on_topic(self, topic, msg):
        wall_time = now_wall()
        try:
            derived = derived_for_topic(topic, msg)
        except Exception as exc:
            derived = {"derive_error": str(exc)}
            self.last_error = "derive failed for %s: %s" % (topic, exc)
        record = {
            "derived": derived,
            "stamp": message_stamp(msg),
            "topic": topic,
            "wall_time": wall_time,
        }
        with self.lock:
            self.latest[topic] = record
            active = self.active

        if not active or not self.should_record_topic(topic, wall_time):
            return

        sample = {
            "data": primitive(msg),
            "derived": derived,
            "schema": SCHEMA_SAMPLE,
            "stamp": record["stamp"],
            "topic": topic,
            "type": "topic_sample",
            "wall_time": wall_time,
        }
        self.write_record(sample)
        with self.lock:
            self.sample_counts[topic] += 1

    def start_capture(self, request):
        with self.lock:
            if self.active:
                return SetBoolResponse(success=False, message="manual path capture already active")
            self.recordings_path.mkdir(parents=True, exist_ok=True)
            self.session_id = time.strftime("manual_path_%Y%m%d_%H%M%S")
            self.session_dir = self.recordings_path / self.session_id
            self.session_dir.mkdir(parents=True, exist_ok=False)
            (self.session_dir / "reports").mkdir(parents=True, exist_ok=True)
            (self.session_dir / "exports").mkdir(parents=True, exist_ok=True)
            self.samples_handle = (self.session_dir / "samples.jsonl").open("a", encoding="utf-8", buffering=1)
            self.sync_samples = []
            self.events = []
            self.sample_counts = Counter()
            self.reject_counts = Counter()
            self.last_error = None
            self.last_artifacts = {
                "samples_jsonl": str(self.session_dir / "samples.jsonl"),
                "session_json": str(self.session_dir / "session.json"),
            }
            started_wall = now_wall()
            self.session_meta = {
                "active": True,
                "artifacts": dict(self.last_artifacts),
                "config": self.config(),
                "ended_at": None,
                "raw_bag_requested": bool(request.data),
                "schema": SCHEMA_SESSION,
                "session_dir": str(self.session_dir),
                "session_id": self.session_id,
                "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(started_wall)),
                "started_wall_time": started_wall,
                "topics": self.topics,
            }
            self.active = True
            self.raw_bag_active = False
            self.write_session_file()

        self.write_record({"schema": SCHEMA_SAMPLE, "session": self.session_meta, "type": "session_start"})
        message = "manual path capture started"
        if request.data:
            try:
                self.start_raw_bag()
                message = "manual path capture started with raw bag"
            except Exception as exc:
                self.last_error = "raw bag failed to start: %s" % exc
                rospy.logwarn(self.last_error)
                message = "manual path capture started without raw bag: %s" % exc
        self.publish_status(None)
        return SetBoolResponse(success=True, message=message)

    def stop_capture(self, _request):
        with self.lock:
            if not self.active:
                return TriggerResponse(success=False, message="manual path capture is not active")
            self.active = False
            stop_wall = now_wall()
            if self.session_meta:
                self.session_meta["active"] = False
                self.session_meta["ended_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(stop_wall))
                self.session_meta["ended_wall_time"] = stop_wall
                self.session_meta["duration_s"] = max(0.0, stop_wall - self.session_meta.get("started_wall_time", stop_wall))
                self.session_meta["sample_counts"] = dict(self.sample_counts)
                self.session_meta["reject_counts"] = dict(self.reject_counts)
            session_id = self.session_id

        self.write_record({"schema": SCHEMA_SAMPLE, "stamp": stop_wall, "type": "session_stop"})
        self.stop_raw_bag()
        export_message = self.export_current_session()
        self.close_samples_file()
        self.write_session_file()
        self.publish_status(None)
        return TriggerResponse(success=True, message="manual path capture stopped: %s (%s)" % (session_id, export_message))

    def mark_event(self, _request):
        with self.lock:
            if not self.active:
                return TriggerResponse(success=False, message="manual path capture is not active")
            event = {
                "event": "mark",
                "index": len(self.events),
                "stamp": now_wall(),
            }
            self.events.append(event)
        self.write_record({"schema": SCHEMA_SAMPLE, "event": event, "type": "event"})
        self.publish_status(None)
        return TriggerResponse(success=True, message="event marker %d recorded" % event["index"])

    def export_capture(self, _request):
        with self.lock:
            if self.active:
                return TriggerResponse(success=False, message="stop capture before exporting")
            if not self.session_dir or not self.session_meta:
                return TriggerResponse(success=False, message="no stopped manual path capture session is available")
        try:
            message = self.export_current_session()
            self.write_session_file()
            self.publish_status(None)
            return TriggerResponse(success=True, message=message)
        except Exception as exc:
            self.last_error = str(exc)
            self.publish_status(None)
            return TriggerResponse(success=False, message=str(exc))

    def start_raw_bag(self):
        with self.lock:
            if not self.session_dir:
                raise RuntimeError("session directory is not ready")
            bag_path = self.session_dir / "raw.bag"
        argv = ["rosbag", "record", "-O", str(bag_path)] + self.raw_bag_topics
        self.raw_bag_process = subprocess.Popen(argv)
        with self.lock:
            self.raw_bag_active = True
            self.last_artifacts["raw_bag"] = str(bag_path)
            if self.session_meta:
                self.session_meta["artifacts"] = dict(self.last_artifacts)

    def stop_raw_bag(self):
        process = self.raw_bag_process
        self.raw_bag_process = None
        with self.lock:
            self.raw_bag_active = False
        if process is None:
            return
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        except Exception as exc:
            self.last_error = "raw bag stop failed: %s" % exc

    def write_sync_sample(self, _event):
        with self.lock:
            if not self.active:
                return
            snapshot = {topic: dict(record) for topic, record in self.latest.items()}
            session_id = self.session_id
        sample = sync_sample_from_snapshot(snapshot, now_wall(), self.config())
        sample["schema"] = SCHEMA_SAMPLE
        sample["session_id"] = session_id
        self.write_record(sample)
        with self.lock:
            self.sync_samples.append(sample)
            self.sample_counts["sync_sample"] += 1
            self.reject_counts.update(sample.get("reject_reasons") or [])

    def write_record(self, record):
        with self.write_lock:
            if not self.samples_handle:
                return
            self.samples_handle.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")

    def close_samples_file(self):
        with self.write_lock:
            if self.samples_handle:
                self.samples_handle.flush()
                self.samples_handle.close()
                self.samples_handle = None

    def write_session_file(self):
        with self.lock:
            if not self.session_dir or not self.session_meta:
                return
            meta = dict(self.session_meta)
            meta["artifacts"] = dict(self.last_artifacts)
        write_json(self.session_dir / "session.json", meta)

    def export_current_session(self):
        with self.lock:
            if not self.session_dir or not self.session_meta:
                raise RuntimeError("no manual path recording session is available")
            session_dir = self.session_dir
            session_meta = dict(self.session_meta)
            sync_samples = list(self.sync_samples)
            events = list(self.events)

        teacher_path = build_teacher_path(session_meta, sync_samples, events, self.config())
        planpath_compat = build_planpath_compat(teacher_path)
        source_report = build_source_comparison_report(sync_samples, teacher_path)

        teacher_path_path = session_dir / "teacher_path.json"
        source_report_path = session_dir / "reports" / "source_comparison.json"
        planpath_path = session_dir / "exports" / "planpath_compat.json"
        report_path = session_dir / "reports" / "path_recording_summary.html"

        write_json(teacher_path_path, teacher_path)
        write_json(source_report_path, source_report)
        write_json(planpath_path, planpath_compat)
        write_html_report(report_path, session_meta, teacher_path, source_report, planpath_compat)

        with self.lock:
            self.last_artifacts.update(
                {
                    "planpath_compat_json": str(planpath_path),
                    "source_comparison_json": str(source_report_path),
                    "summary_html": str(report_path),
                    "teacher_path_json": str(teacher_path_path),
                }
            )
            if self.session_meta:
                self.session_meta["artifacts"] = dict(self.last_artifacts)
                self.session_meta["exported_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now_wall()))
                self.session_meta["teacher_segment_count"] = len(teacher_path.get("segments", []))
                self.session_meta["planpath_compat_path_count"] = len(planpath_compat.get("paths", []))
        return "exported teacher_path.json, source_comparison.json, planpath_compat.json, and HTML report"

    def status_payload(self):
        with self.lock:
            now_s = now_wall()
            duration_s = None
            if self.active and self.session_meta:
                duration_s = now_s - self.session_meta.get("started_wall_time", now_s)
            elif self.session_meta:
                duration_s = self.session_meta.get("duration_s")
            source_health = source_health_from_snapshot(self.latest, now_s, self.max_pose_age_s)
            return {
                "active": self.active,
                "artifact_paths": dict(self.last_artifacts),
                "duration_s": duration_s,
                "last_error": self.last_error,
                "raw_bag_active": self.raw_bag_active,
                "reject_counts": dict(self.reject_counts),
                "sample_counts": dict(self.sample_counts),
                "session_dir": str(self.session_dir) if self.session_dir else None,
                "session_id": self.session_id,
                "source_health": source_health,
                "subscribed_topics": sorted(self.subscribers.keys()),
                "sync_rate_hz": self.sync_rate_hz,
            }

    def publish_status(self, _event):
        self.status_pub.publish(String(data=json.dumps(self.status_payload(), sort_keys=True)))

    def on_shutdown(self):
        with self.lock:
            was_active = self.active
            self.active = False
        if was_active:
            self.write_record({"schema": SCHEMA_SAMPLE, "stamp": now_wall(), "type": "shutdown_stop"})
        self.stop_raw_bag()
        if was_active:
            try:
                self.export_current_session()
            except Exception as exc:
                self.last_error = "shutdown export failed: %s" % exc
        self.close_samples_file()
        self.write_session_file()


def main():
    recorder = ManualPathRecorder()
    rospy.spin()
    return recorder


if __name__ == "__main__":
    main()
