#!/usr/bin/env python3

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import manual_path_recorder as recorder  # noqa: E402


CONFIG = {
    "frame_id": "map",
    "max_gap_s": 1.0,
    "max_pose_age_s": 0.75,
    "max_pose_jump_m": 0.75,
    "max_source_separation_m": 0.45,
    "max_yaw_jump_rad": 1.0,
    "min_pose_step_m": 0.01,
    "min_yaw_step_rad": 0.01,
}


def sample(stamp, x, y, yaw=0.0, blade=True, accepted=True):
    return {
        "accepted_for_replay": accepted,
        "blade": {"mow_enabled": blade},
        "emergency": {},
        "fused": {"x": x, "y": y, "yaw": yaw, "position_accuracy": 0.03},
        "fusion_status": {"active_sources": ["gps", "lidar"], "ready_for_navigation": True, "state": "ready"},
        "gps_raw": {"flags": recorder.FLAG_GPS_RTK_FIXED},
        "mower_state": {"state_name": "AREA_RECORDING"},
        "reject_reasons": [] if accepted else ["gps_not_rtk_fixed"],
        "selected_pose": {"x": x, "y": y, "yaw": yaw, "position_accuracy": 0.03},
        "selected_pose_source": "fused",
        "slam_alignment": {"aligned": True, "state": "aligned"},
        "source_health": {"fused": {"age_s": 0.1, "fresh": True, "seen": True, "stamp": stamp}},
        "source_quality": {"active_sources": ["gps", "lidar"], "gps_lidar_separation_m": 0.05},
        "stamp": stamp,
    }


def test_reject_reasons():
    bad = sample(1.0, 0.0, 0.0)
    bad["gps_raw"]["flags"] = 0
    reasons = recorder.replay_reject_reasons(bad, max_pose_age_s=0.75, max_source_separation_m=0.45)
    assert "gps_not_rtk_fixed" in reasons

    bad = sample(1.0, 0.0, 0.0)
    bad["source_quality"]["gps_lidar_separation_m"] = 0.9
    reasons = recorder.replay_reject_reasons(bad, max_pose_age_s=0.75, max_source_separation_m=0.45)
    assert "source_conflict" in reasons


def test_teacher_path_segment_splitting():
    sync_samples = [
        sample(1.0, 0.0, 0.0, blade=True),
        sample(1.2, 0.2, 0.0, blade=True),
        sample(1.4, 0.4, 0.0, blade=True),
        sample(1.6, 0.6, 0.0, blade=False),
        sample(1.8, 0.8, 0.0, blade=False),
    ]
    teacher = recorder.build_teacher_path(
        {"session_id": "test", "started_at": "start", "ended_at": "end"},
        sync_samples,
        [{"event": "mark", "stamp": 1.3}],
        CONFIG,
    )
    reasons = [segment["break_after_reason"] for segment in teacher["segments"]]
    assert reasons == ["operator_event", "blade_state_change", "end_of_capture"]
    assert [segment["blade_state"] for segment in teacher["segments"]] == ["ON", "ON", "OFF"]


def test_planpath_compat_skips_unsupported_spans():
    teacher = recorder.build_teacher_path(
        {"session_id": "test", "started_at": "start", "ended_at": "end"},
        [
            sample(1.0, 0.0, 0.0, blade=True),
            sample(1.2, 0.2, 0.0, blade=True),
            sample(1.4, 0.4, 0.0, blade=True),
            sample(1.6, 0.6, 0.0, blade=False),
            sample(1.8, 0.8, 0.0, blade=False),
            sample(2.0, 1.0, 0.0, blade=False),
        ],
        [],
        CONFIG,
    )
    compat = recorder.build_planpath_compat(teacher)
    assert len(compat["paths"]) == 1
    assert len(compat["skipped_segments"]) == 1
    assert "blade_off" in compat["skipped_segments"][0]["reasons"]


if __name__ == "__main__":
    test_reject_reasons()
    test_teacher_path_segment_splitting()
    test_planpath_compat_skips_unsupported_spans()
    print("manual path recorder tests ok")
