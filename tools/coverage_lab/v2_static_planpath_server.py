#!/usr/bin/env python3
"""Serve a V2 ``planpath_compat.json`` through the current PlanPath service.

This is a temporary dry-run bridge for lab validation. It deliberately keeps
the old mower-facing service shape and does not add reverse, blade-state, or
maneuver semantics.

>>> _yaw_to_quaternion(0.0)
(0.0, 0.0, 0.0, 1.0)
>>> _clean_bool(1)
True
>>> _clean_bool("false")
False
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
from dataclasses import dataclass
from typing import Any


DEFAULT_SERVICE_NAME = "slic3r_coverage_planner/plan_path"


@dataclass(frozen=True)
class PlanSummary:
    path_count: int
    outline_count: int
    fill_count: int
    pose_count: int
    gap_count: int
    max_gap_m: float


def _clean_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _read_json(path: pathlib.Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"expected JSON object in {path}")
    return data


def _write_json(path: pathlib.Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _path_pose_count(path: dict[str, Any]) -> int:
    return len(path.get("path", {}).get("poses", []) or [])


def _pose_xy(path: dict[str, Any], index: int) -> tuple[float, float] | None:
    poses = path.get("path", {}).get("poses", []) or []
    if not poses:
        return None
    pose = poses[index]
    try:
        return (float(pose["x"]), float(pose["y"]))
    except (KeyError, TypeError, ValueError):
        return None


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def summarize_plan(plan: dict[str, Any]) -> PlanSummary:
    paths = list(plan.get("paths", []) or [])
    outline_count = sum(1 for path in paths if _clean_bool(path.get("is_outline", False)))
    fill_count = len(paths) - outline_count
    pose_count = sum(_path_pose_count(path) for path in paths)
    gaps: list[float] = []
    previous_end: tuple[float, float] | None = None
    for path in paths:
        start = _pose_xy(path, 0)
        end = _pose_xy(path, -1)
        if previous_end is not None and start is not None:
            gap = _distance(previous_end, start)
            if gap > 0.05:
                gaps.append(gap)
        if end is not None:
            previous_end = end
    return PlanSummary(
        path_count=len(paths),
        outline_count=outline_count,
        fill_count=fill_count,
        pose_count=pose_count,
        gap_count=len(gaps),
        max_gap_m=max(gaps) if gaps else 0.0,
    )


def filter_plan(
    plan: dict[str, Any],
    *,
    skip_outline: bool,
    max_fill_paths: int | None,
    max_total_paths: int | None,
) -> dict[str, Any]:
    kept: list[dict[str, Any]] = []
    fill_seen = 0
    for path in plan.get("paths", []) or []:
        is_outline = _clean_bool(path.get("is_outline", False))
        if skip_outline and is_outline:
            continue
        if not is_outline:
            fill_seen += 1
            if max_fill_paths is not None and fill_seen > max_fill_paths:
                continue
        kept.append(path)
        if max_total_paths is not None and len(kept) >= max_total_paths:
            break
    output = {
        "schema": plan.get("schema", "open_mower.planpath_compat.v0"),
        "profile": f"{plan.get('profile', 'unknown')}:static_service_filter",
        "frame_id": plan.get("frame_id", "map"),
        "source_map": plan.get("source_map", ""),
        "paths": kept,
    }
    summary = summarize_plan(output)
    output["static_service_filter"] = {
        "skip_outline": bool(skip_outline),
        "max_fill_paths": max_fill_paths,
        "max_total_paths": max_total_paths,
        "path_count": summary.path_count,
        "outline_count": summary.outline_count,
        "fill_count": summary.fill_count,
        "pose_count": summary.pose_count,
        "gap_count": summary.gap_count,
        "max_gap_m": round(summary.max_gap_m, 4),
    }
    return output


def _print_summary(plan: dict[str, Any], label: str) -> None:
    summary = summarize_plan(plan)
    print(
        f"{label}: paths={summary.path_count}, outline={summary.outline_count}, "
        f"fill={summary.fill_count}, poses={summary.pose_count}, "
        f"gaps={summary.gap_count}, max_gap_m={summary.max_gap_m:.3f}"
    )


def cmd_filter(args: argparse.Namespace) -> None:
    plan = _read_json(pathlib.Path(args.plan).resolve())
    filtered = filter_plan(
        plan,
        skip_outline=args.skip_outline,
        max_fill_paths=args.max_fill_paths,
        max_total_paths=args.max_total_paths,
    )
    _write_json(pathlib.Path(args.output).resolve(), filtered)
    _print_summary(plan, "input")
    _print_summary(filtered, "filtered")
    print(f"Wrote filtered static PlanPath JSON: {pathlib.Path(args.output).resolve()}")


def _build_ros_paths(plan: dict[str, Any]) -> list[Any]:
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath
    from slic3r_coverage_planner.msg import Path as PlannerPath
    import rospy

    frame_id = str(plan.get("frame_id") or "map")
    stamp = rospy.Time.now()
    ros_paths = []
    for path_index, path in enumerate(plan.get("paths", []) or []):
        nav_path = NavPath()
        nav_path.header.frame_id = str(path.get("path", {}).get("frame_id") or frame_id)
        nav_path.header.stamp = stamp
        for pose_index, pose in enumerate(path.get("path", {}).get("poses", []) or []):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = nav_path.header.frame_id
            pose_stamped.header.stamp = stamp
            pose_stamped.pose.position.x = float(pose["x"])
            pose_stamped.pose.position.y = float(pose["y"])
            pose_stamped.pose.position.z = float(pose.get("z", 0.0))
            qx, qy, qz, qw = _yaw_to_quaternion(float(pose.get("yaw", 0.0)))
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            nav_path.poses.append(pose_stamped)
        if len(nav_path.poses) < 2:
            raise ValueError(f"path {path_index} has fewer than 2 poses")
        planner_path = PlannerPath()
        planner_path.is_outline = 1 if _clean_bool(path.get("is_outline", False)) else 0
        planner_path.path = nav_path
        ros_paths.append(planner_path)
    return ros_paths


def cmd_serve(args: argparse.Namespace) -> None:
    import rospy
    from slic3r_coverage_planner.srv import PlanPath, PlanPathResponse

    plan = _read_json(pathlib.Path(args.plan).resolve())
    filtered = filter_plan(
        plan,
        skip_outline=args.skip_outline,
        max_fill_paths=args.max_fill_paths,
        max_total_paths=args.max_total_paths,
    )
    rospy.init_node(args.node_name, anonymous=False)
    ros_paths = _build_ros_paths(filtered)
    _print_summary(filtered, "serving")

    def handle_plan_path(_request: Any) -> Any:
        rospy.logwarn(
            "Serving static V2 planpath_compat response with %d paths. "
            "This is for dry-run validation only.",
            len(ros_paths),
        )
        return PlanPathResponse(paths=ros_paths)

    rospy.Service(args.service_name, PlanPath, handle_plan_path)
    rospy.logwarn("Static V2 PlanPath service ready on %s", args.service_name)
    rospy.spin()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Temporary V2 planpath_compat ROS service bridge.")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_filter_args(command: argparse.ArgumentParser) -> None:
        command.add_argument("--plan", required=True, help="path to V2 planpath_compat.json")
        command.add_argument("--skip-outline", action="store_true", help="omit outline paths from the served plan")
        command.add_argument("--max-fill-paths", type=int, help="limit number of non-outline paths")
        command.add_argument("--max-total-paths", type=int, help="limit total number of served paths")

    serve = sub.add_parser("serve", help="advertise slic3r_coverage_planner/plan_path from static JSON")
    add_filter_args(serve)
    serve.add_argument("--service-name", default=DEFAULT_SERVICE_NAME, help="ROS service name to advertise")
    serve.add_argument("--node-name", default="v2_static_planpath_server", help="ROS node name")
    serve.set_defaults(func=cmd_serve)

    filter_cmd = sub.add_parser("filter", help="write a reduced static PlanPath JSON for inspection/dry runs")
    add_filter_args(filter_cmd)
    filter_cmd.add_argument("--output", required=True, help="filtered JSON output path")
    filter_cmd.set_defaults(func=cmd_filter)

    summary = sub.add_parser("summary", help="print a PlanPath JSON summary")
    summary.add_argument("--plan", required=True, help="path to V2 planpath_compat.json")
    summary.set_defaults(func=lambda args: _print_summary(_read_json(pathlib.Path(args.plan).resolve()), "plan"))

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
