"""Lossy V2 task-path export for the current PlanPath-like JSON shape.

The V2 planner carries richer segment metadata than the mower-facing
``planpath_compat.json`` contract can express. This adapter deliberately emits
only old-shape ``base_link`` pose paths and returns a separate summary of what
was skipped or flattened.

>>> _clean_pose({"x": 1, "y": "2.5", "yaw": None})
{'x': 1.0, 'y': 2.5, 'yaw': 0.0}
"""
from __future__ import annotations

from typing import Any


DEFAULT_PROFILE = "v2_task_paths_forward_cutting_compat"


def _bool_config(config: dict[str, Any], key: str, default: bool) -> bool:
    value = config.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _int_config(config: dict[str, Any], key: str, default: int) -> int:
    try:
        return int(config.get(key, default))
    except (TypeError, ValueError):
        return default


def options_from_config(config: dict[str, Any]) -> dict[str, Any]:
    mode = str(config.get("v2_planpath_compat_mode", "forward_cutting_only")).strip().lower()
    mode_aliases = {
        "single_task_path": "single_infill_path",
        "continuous_infill": "single_infill_path",
        "continuous_infill_path": "single_infill_path",
    }
    mode = mode_aliases.get(mode, mode)
    if mode not in {"forward_cutting_only", "all_segments", "single_infill_path"}:
        mode = "forward_cutting_only"
    return {
        "enabled": _bool_config(config, "v2_planpath_compat_enabled", True),
        "mode": mode,
        "profile": str(config.get("v2_planpath_compat_profile", DEFAULT_PROFILE)),
        "min_pose_count": max(2, _int_config(config, "v2_planpath_compat_min_pose_count", 2)),
    }


def _clean_pose(pose: dict[str, Any]) -> dict[str, float]:
    yaw = pose.get("yaw", 0.0)
    if yaw is None:
        yaw = 0.0
    return {
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "yaw": float(yaw),
    }


def _skip_reason(segment: dict[str, Any], options: dict[str, Any]) -> str | None:
    poses = segment.get("base_poses") or []
    if len(poses) < int(options["min_pose_count"]):
        return "too_few_base_poses"
    if options["mode"] in {"all_segments", "single_infill_path"}:
        return None
    direction = str(segment.get("direction", "forward"))
    if direction != "forward":
        return f"unsupported_direction:{direction}"
    if not bool(segment.get("cutting_enabled", False)):
        return "blade_off_segment"
    return None


def _add_count(counts: dict[str, int], key: str) -> None:
    counts[key] = counts.get(key, 0) + 1


def _same_pose(a: dict[str, float], b: dict[str, float]) -> bool:
    return (
        abs(float(a.get("x", 0.0)) - float(b.get("x", 0.0))) < 1e-4
        and abs(float(a.get("y", 0.0)) - float(b.get("y", 0.0))) < 1e-4
        and abs(float(a.get("yaw", 0.0)) - float(b.get("yaw", 0.0))) < 1e-5
    )


def _append_clean_poses(path_poses: list[dict[str, float]], segment: dict[str, Any]) -> int:
    poses = [_clean_pose(pose) for pose in segment.get("base_poses", [])]
    if path_poses and poses and _same_pose(path_poses[-1], poses[0]):
        poses = poses[1:]
    path_poses.extend(poses)
    return len(poses)


def _export_segments(
    *,
    compat: dict[str, Any],
    summary: dict[str, Any],
    area_index: int,
    area_id: str,
    area_name: str,
    frame_id: str,
    segments: list[dict[str, Any]],
    label_parts: list[str],
    is_outline: bool,
    options: dict[str, Any],
) -> None:
    for segment_index, segment in enumerate(segments):
        reason = _skip_reason(segment, options)
        if reason is not None:
            summary["skipped_segment_count"] += 1
            _add_count(summary["skipped_by_reason"], reason)
            continue
        poses = [_clean_pose(pose) for pose in segment.get("base_poses", [])]
        compat["paths"].append(
            {
                "is_outline": bool(is_outline),
                "area_index": area_index,
                "area_id": area_id,
                "label": " ".join(label_parts + [str(segment.get("phase") or f"segment {segment_index}")]),
                "path": {
                    "frame_id": frame_id,
                    "poses": poses,
                },
            }
        )
        summary["exported_segment_count"] += 1
        summary["pose_count"] += len(poses)
        summary["exported_unsafe_sample_count"] += len(segment.get("unsafe_samples", []))
        if is_outline:
            summary["exported_outline_path_count"] += 1
            summary["exported_outline_segment_count"] += 1
            summary["outline_pose_count"] += len(poses)
        else:
            summary["exported_fill_path_count"] += 1
            summary["exported_fill_segment_count"] += 1


def _export_task_path_as_single_infill(
    *,
    compat: dict[str, Any],
    summary: dict[str, Any],
    area_index: int,
    area_id: str,
    area_name: str,
    frame_id: str,
    task_path: dict[str, Any],
    label_parts: list[str],
    options: dict[str, Any],
) -> None:
    merged_poses: list[dict[str, float]] = []
    included_segments: list[dict[str, Any]] = []
    for segment in task_path.get("segments", []):
        reason = _skip_reason(segment, options)
        if reason is not None:
            summary["skipped_segment_count"] += 1
            _add_count(summary["skipped_by_reason"], reason)
            continue
        before = len(merged_poses)
        added = _append_clean_poses(merged_poses, segment)
        if added <= 0 and len(merged_poses) == before:
            summary["skipped_segment_count"] += 1
            _add_count(summary["skipped_by_reason"], "too_few_base_poses")
            continue
        included_segments.append(segment)

    if len(merged_poses) < int(options["min_pose_count"]):
        if included_segments:
            summary["skipped_segment_count"] += len(included_segments)
            _add_count(summary["skipped_by_reason"], "single_infill_too_few_merged_poses")
        return

    compat["paths"].append(
        {
            "is_outline": False,
            "area_index": area_index,
            "area_id": area_id,
            "label": " ".join(label_parts + ["continuous infill"]),
            "segment_count": len(included_segments),
            "segment_ids": [str(segment.get("segment_id", "")) for segment in included_segments],
            "connector_count": sum(
                1
                for segment in included_segments
                if str(segment.get("phase", "")).endswith("_connector") or bool(segment.get("connector_kind"))
            ),
            "path": {
                "frame_id": frame_id,
                "poses": merged_poses,
            },
        }
    )
    summary["exported_fill_path_count"] += 1
    summary["exported_segment_count"] += len(included_segments)
    summary["exported_fill_segment_count"] += len(included_segments)
    summary["pose_count"] += len(merged_poses)
    summary["exported_unsafe_sample_count"] += sum(
        len(segment.get("unsafe_samples", [])) for segment in included_segments
    )


def build_planpath_compat(
    *,
    result: dict[str, Any],
    task_path_result: dict[str, Any],
    config: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return ``(compat, summary)`` for V2 task paths.

    ``compat`` intentionally keeps the old mower-facing shape:
    ``schema/profile/frame_id/source_map/paths[]`` where each path owns
    ``path.poses[{x,y,yaw}]`` in ``base_link`` coordinates.
    """
    options = options_from_config(config)
    frame_id = str(task_path_result.get("frame_id") or result.get("frame_id") or config.get("frame_id", "map"))
    compat = {
        "schema": "open_mower.planpath_compat.v0",
        "profile": options["profile"],
        "frame_id": frame_id,
        "source_map": str(task_path_result.get("source_map") or result.get("source_map") or ""),
        "paths": [],
    }
    summary: dict[str, Any] = {
        "schema": "open_mower.coverage_lab.v2_planpath_compat_summary.v0",
        "enabled": bool(options["enabled"]),
        "profile": options["profile"],
        "mode": options["mode"],
        "path_count": 0,
        "pose_count": 0,
        "exported_segment_count": 0,
        "exported_outline_path_count": 0,
        "exported_fill_path_count": 0,
        "exported_outline_segment_count": 0,
        "exported_fill_segment_count": 0,
        "outline_pose_count": 0,
        "exported_unsafe_sample_count": 0,
        "skipped_segment_count": 0,
        "skipped_by_reason": {},
        "warnings": [],
    }
    if not options["enabled"]:
        summary["warnings"].append("V2 planpath compatibility export disabled by config")
        return compat, summary

    for area in task_path_result.get("areas", []):
        area_index = int(area.get("area_index", 0))
        area_id = str(area.get("source_id", ""))
        area_name = str(area.get("source_name") or area_id or f"area-{area_index}")
        for outline_path in area.get("outline_paths", []):
            _export_segments(
                compat=compat,
                summary=summary,
                area_index=area_index,
                area_id=area_id,
                area_name=area_name,
                frame_id=frame_id,
                segments=list(outline_path.get("segments", [])),
                label_parts=[
                    area_name,
                    "V2",
                    "outline",
                    f"layer {outline_path.get('layer_index', 0)}",
                    str(outline_path.get("ring_type", "ring")),
                ],
                is_outline=True,
                options=options,
            )
        for task_path in area.get("task_paths", []):
            task_id = str(task_path.get("task_id") or task_path.get("task_path_id") or "task")
            task_type = str(task_path.get("task_type", "unknown"))
            if options["mode"] == "single_infill_path":
                _export_task_path_as_single_infill(
                    compat=compat,
                    summary=summary,
                    area_index=area_index,
                    area_id=area_id,
                    area_name=area_name,
                    frame_id=frame_id,
                    task_path=task_path,
                    label_parts=[
                        area_name,
                        "V2",
                        task_type,
                        task_id,
                    ],
                    options=options,
                )
                continue
            for segment_index, segment in enumerate(task_path.get("segments", [])):
                label_parts = [
                    area_name,
                    "V2",
                    task_type,
                    f"lane {segment.get('lane_index')}"
                    if segment.get("lane_index") is not None
                    else f"segment {segment_index}",
                ]
                _export_segments(
                    compat=compat,
                    summary=summary,
                    area_index=area_index,
                    area_id=area_id,
                    area_name=area_name,
                    frame_id=frame_id,
                    segments=[segment],
                    label_parts=label_parts,
                    is_outline=False,
                    options=options,
                )

    summary["path_count"] = len(compat["paths"])
    if summary["exported_unsafe_sample_count"]:
        summary["warnings"].append(
            f"exported {summary['exported_unsafe_sample_count']} unsafe V2 footprint samples; do not treat this as live-ready"
        )
    if summary["skipped_segment_count"]:
        summary["warnings"].append(
            f"skipped {summary['skipped_segment_count']} V2 segments that the current PlanPath shape cannot express"
        )
    if not compat["paths"]:
        summary["warnings"].append("no V2 segments were eligible for planpath compatibility export")
    return compat, summary
