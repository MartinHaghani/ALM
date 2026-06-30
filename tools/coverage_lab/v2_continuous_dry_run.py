#!/usr/bin/env python3
"""Build a one-path, inner-safe V2 dry-run PlanPath JSON.

This is a temporary controller diagnostic. It intentionally generates stripes
inside the all-yaw-safe region so forward-only connectors can be simple and the
old mower runtime does not stop between separate ``paths[]`` entries.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
from dataclasses import dataclass
from typing import Any, Iterable

import yaml
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union


DEFAULT_FRAME_ID = "map"
DEFAULT_SAMPLE_STEP_M = 0.10
DEFAULT_SPACING_M = 0.55
DEFAULT_MIN_STRIPE_LENGTH_M = 0.75
DEFAULT_EXTRA_INSET_M = 0.10


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    section: str = "coverage"

    def as_json(self) -> dict[str, float | str]:
        return {
            "x": round(self.x, 6),
            "y": round(self.y, 6),
            "yaw": round(_wrap_angle(self.yaw), 6),
            "section": self.section,
        }


@dataclass(frozen=True)
class Stripe:
    start: tuple[float, float]
    end: tuple[float, float]
    offset_m: float
    length_m: float


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


def _read_config(path: pathlib.Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"expected YAML object in {path}")
    return data


def _wrap_angle(yaw: float) -> float:
    while yaw <= -math.pi:
        yaw += 2.0 * math.pi
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    return yaw


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _point_along(a: tuple[float, float], b: tuple[float, float], t: float) -> tuple[float, float]:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def _line_yaw(a: tuple[float, float], b: tuple[float, float], fallback: float = 0.0) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return fallback
    return math.atan2(dy, dx)


def _sample_line(
    a: tuple[float, float],
    b: tuple[float, float],
    *,
    step_m: float,
    section: str,
    include_start: bool,
) -> list[Pose]:
    length = _distance(a, b)
    if length < 1e-6:
        return []
    yaw = _line_yaw(a, b)
    count = max(2, int(math.ceil(length / max(step_m, 0.01))) + 1)
    start_i = 0 if include_start else 1
    return [
        Pose(*_point_along(a, b, i / (count - 1)), yaw=yaw, section=section)
        for i in range(start_i, count)
    ]


def _polygon_from_area(area: dict[str, Any]) -> Polygon:
    outline = area.get("outline") or []
    points = [(float(point["x"]), float(point["y"])) for point in outline]
    if len(points) < 3:
        raise ValueError("mow area outline has fewer than three points")
    polygon = Polygon(points).buffer(0)
    if polygon.is_empty or not polygon.is_valid:
        raise ValueError("mow area outline did not produce a valid polygon")
    return polygon


def _select_mow_area(map_data: dict[str, Any], area_index: int | None) -> tuple[int, dict[str, Any], Polygon]:
    mow_areas = [
        (index, area)
        for index, area in enumerate(map_data.get("areas", []) or [])
        if (area.get("properties") or {}).get("type") == "mow" and not bool((area.get("properties") or {}).get("hidden"))
    ]
    if not mow_areas:
        raise ValueError("map has no active mow areas")
    if area_index is None:
        index, area = mow_areas[0]
    else:
        matches = [(index, area) for index, area in mow_areas if index == area_index]
        if not matches:
            raise ValueError(f"map has no active mow area at index {area_index}")
        index, area = matches[0]
    return index, area, _polygon_from_area(area)


def _float_config(config: dict[str, Any], key: str, default: float) -> float:
    try:
        return float(config.get(key, default))
    except (TypeError, ValueError):
        return default


def _footprint(config: dict[str, Any]) -> list[tuple[float, float]]:
    raw = config.get("footprint") or [[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]]
    points: list[tuple[float, float]] = []
    for point in raw:
        if len(point) < 2:
            continue
        points.append((float(point[0]), float(point[1])))
    if len(points) < 3:
        raise ValueError("configured footprint has fewer than three points")
    return points


def _all_yaw_radius(config: dict[str, Any], extra_inset_m: float) -> float:
    margin = _float_config(config, "safety_margin_m", 0.05)
    return max(math.hypot(x, y) for x, y in _footprint(config)) + margin + max(0.0, extra_inset_m)


def _footprint_polygon(pose: Pose, config: dict[str, Any]) -> Polygon:
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    points = [
        (pose.x + c * px - s * py, pose.y + s * px + c * py)
        for px, py in _footprint(config)
    ]
    return Polygon(points).buffer(0)


def _dominant_yaw_from_plan(plan_path: pathlib.Path | None) -> float | None:
    if plan_path is None or not plan_path.exists():
        return None
    plan = _read_json(plan_path)
    lengths_by_yaw: dict[float, float] = {}
    for path in plan.get("paths", []) or []:
        if bool(path.get("is_outline", False)):
            continue
        poses = path.get("path", {}).get("poses", []) or []
        for a, b in zip(poses, poses[1:]):
            yaw = float(a.get("yaw", _line_yaw((a["x"], a["y"]), (b["x"], b["y"]))))
            bucket = round(_wrap_angle(yaw), 3)
            lengths_by_yaw[bucket] = lengths_by_yaw.get(bucket, 0.0) + _distance((a["x"], a["y"]), (b["x"], b["y"]))
    if not lengths_by_yaw:
        return None
    yaw = max(lengths_by_yaw.items(), key=lambda item: item[1])[0]
    # Normalize the stripe axis so either travel direction represents the same line.
    if yaw < -math.pi / 2.0:
        yaw += math.pi
    if yaw > math.pi / 2.0:
        yaw -= math.pi
    return _wrap_angle(yaw)


def _dominant_yaw_from_polygon(polygon: Polygon | MultiPolygon) -> float:
    bounds = polygon.minimum_rotated_rectangle.exterior.coords[:-1]
    edges = []
    for a, b in zip(bounds, list(bounds[1:]) + [bounds[0]]):
        length = _distance(a, b)
        edges.append((length, _line_yaw(a, b)))
    yaw = max(edges, key=lambda item: item[0])[1]
    if yaw < -math.pi / 2.0:
        yaw += math.pi
    if yaw > math.pi / 2.0:
        yaw -= math.pi
    return _wrap_angle(yaw)


def _to_local(point: tuple[float, float], origin: tuple[float, float], yaw: float) -> tuple[float, float]:
    dx = point[0] - origin[0]
    dy = point[1] - origin[1]
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def _to_world(local: tuple[float, float], origin: tuple[float, float], yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (origin[0] + c * local[0] - s * local[1], origin[1] + s * local[0] + c * local[1])


def _iter_lines(geometry: Any) -> Iterable[LineString]:
    if geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return [geometry]
    if isinstance(geometry, MultiLineString):
        return list(geometry.geoms)
    if hasattr(geometry, "geoms"):
        lines: list[LineString] = []
        for geom in geometry.geoms:
            lines.extend(_iter_lines(geom))
        return lines
    return []


def _generate_stripes(
    polygon: Polygon | MultiPolygon,
    *,
    yaw: float,
    spacing_m: float,
    min_length_m: float,
) -> list[Stripe]:
    origin = tuple(polygon.representative_point().coords[0])
    local_points: list[tuple[float, float]] = []
    for geom in ([polygon] if isinstance(polygon, Polygon) else list(polygon.geoms)):
        local_points.extend(_to_local(point, origin, yaw) for point in geom.exterior.coords)
    min_s = min(point[0] for point in local_points)
    max_s = max(point[0] for point in local_points)
    min_t = min(point[1] for point in local_points)
    max_t = max(point[1] for point in local_points)
    pad = 2.0
    count = max(1, int(math.floor((max_t - min_t) / spacing_m)) + 1)
    first_t = (min_t + max_t) * 0.5 - (count - 1) * spacing_m * 0.5

    stripes: list[Stripe] = []
    for stripe_index in range(count):
        t = first_t + stripe_index * spacing_m
        line = LineString([
            _to_world((min_s - pad, t), origin, yaw),
            _to_world((max_s + pad, t), origin, yaw),
        ])
        clipped = line.intersection(polygon)
        for segment in _iter_lines(clipped):
            if segment.length < min_length_m:
                continue
            coords = list(segment.coords)
            start = coords[0]
            end = coords[-1]
            if _to_local(start, origin, yaw)[0] > _to_local(end, origin, yaw)[0]:
                start, end = end, start
            stripes.append(
                Stripe(
                    start=(float(start[0]), float(start[1])),
                    end=(float(end[0]), float(end[1])),
                    offset_m=t,
                    length_m=float(segment.length),
                )
            )
    stripes.sort(key=lambda stripe: stripe.offset_m)
    return stripes


def _build_serpentine_route(stripes: list[Stripe], *, sample_step_m: float) -> tuple[list[Pose], list[dict[str, Any]]]:
    route: list[Pose] = []
    connectors: list[dict[str, Any]] = []
    previous_end: tuple[float, float] | None = None
    previous_yaw = 0.0
    for index, stripe in enumerate(stripes):
        start, end = (stripe.start, stripe.end) if index % 2 == 0 else (stripe.end, stripe.start)
        if previous_end is not None:
            connector_poses = _sample_line(
                previous_end,
                start,
                step_m=sample_step_m,
                section="connector",
                include_start=False,
            )
            route.extend(connector_poses)
            connectors.append(
                {
                    "from_stripe_index": index - 1,
                    "to_stripe_index": index,
                    "length_m": round(_distance(previous_end, start), 4),
                    "pose_count": len(connector_poses),
                }
            )
            previous_yaw = connector_poses[-1].yaw if connector_poses else previous_yaw
        stripe_poses = _sample_line(start, end, step_m=sample_step_m, section="coverage", include_start=not route)
        if not stripe_poses:
            continue
        route.extend(stripe_poses)
        previous_end = end
        previous_yaw = stripe_poses[-1].yaw
    if route:
        route[-1] = Pose(route[-1].x, route[-1].y, previous_yaw, route[-1].section)
    return route, connectors


def _unsafe_samples(
    poses: list[Pose],
    *,
    boundary: Polygon,
    config: dict[str, Any],
) -> list[dict[str, Any]]:
    unsafe: list[dict[str, Any]] = []
    for index, pose in enumerate(poses):
        footprint = _footprint_polygon(pose, config)
        if not boundary.covers(footprint):
            unsafe.append(
                {
                    "pose_index": index,
                    "x": round(pose.x, 6),
                    "y": round(pose.y, 6),
                    "yaw": round(_wrap_angle(pose.yaw), 6),
                    "outside_area_m2": round(float(footprint.difference(boundary).area), 6),
                }
            )
    return unsafe


def build_continuous_dry_run(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any]]:
    config = _read_config(pathlib.Path(args.config).resolve() if args.config else None)
    map_path = pathlib.Path(args.map).resolve()
    map_data = _read_json(map_path)
    area_index, area, boundary = _select_mow_area(map_data, args.area_index)
    extra_inset = float(args.extra_inset_m)
    radius = _all_yaw_radius(config, extra_inset)
    safe_region = boundary.buffer(-radius, join_style="round", quad_segs=16).buffer(0)
    if safe_region.is_empty:
        raise ValueError(
            f"all-yaw-safe inset is empty at radius {radius:.3f} m; reduce --extra-inset-m or use a larger map"
        )
    if isinstance(safe_region, MultiPolygon):
        safe_region = max(safe_region.geoms, key=lambda geom: geom.area)

    plan_yaw = _dominant_yaw_from_plan(pathlib.Path(args.source_plan).resolve() if args.source_plan else None)
    yaw = plan_yaw if plan_yaw is not None else _dominant_yaw_from_polygon(safe_region)
    stripes = _generate_stripes(
        safe_region,
        yaw=yaw,
        spacing_m=float(args.spacing_m),
        min_length_m=float(args.min_stripe_length_m),
    )
    if not stripes:
        raise ValueError("no dry-run stripes survived inside the all-yaw-safe region")
    route, connectors = _build_serpentine_route(stripes, sample_step_m=float(args.sample_step_m))
    unsafe = _unsafe_samples(route, boundary=boundary, config=config)

    area_id = str(area.get("id", ""))
    label = f"continuous inner dry run area {area_index}"
    plan = {
        "schema": "open_mower.planpath_compat.v0",
        "profile": "v2_continuous_inner_dry_run",
        "frame_id": DEFAULT_FRAME_ID,
        "source_map": str(map_path),
        "paths": [
            {
                "is_outline": False,
                "area_index": area_index,
                "area_id": area_id,
                "label": label,
                "path": {
                    "frame_id": DEFAULT_FRAME_ID,
                    "poses": [pose.as_json() for pose in route],
                },
            }
        ],
        "continuous_dry_run": {
            "schema": "open_mower.coverage_lab.v2_continuous_dry_run.v0",
            "strategy": "all_yaw_safe_serpentine",
            "purpose": "controller dry run only; not a final coverage route",
            "all_yaw_inset_radius_m": round(radius, 4),
            "extra_inset_m": round(extra_inset, 4),
            "stripe_yaw_rad": round(yaw, 6),
            "stripe_spacing_m": round(float(args.spacing_m), 4),
            "sample_step_m": round(float(args.sample_step_m), 4),
            "stripe_count": len(stripes),
            "connector_count": len(connectors),
            "unsafe_sample_count": len(unsafe),
            "safe_region_area_m2": round(float(safe_region.area), 4),
            "boundary_area_m2": round(float(boundary.area), 4),
            "connectors": connectors,
            "unsafe_samples": unsafe[:200],
        },
    }
    summary = {
        "path_count": 1,
        "pose_count": len(route),
        "stripe_count": len(stripes),
        "connector_count": len(connectors),
        "unsafe_sample_count": len(unsafe),
        "path_length_m": round(
            sum(_distance((a.x, a.y), (b.x, b.y)) for a, b in zip(route, route[1:])),
            4,
        ),
        "coverage_length_m": round(
            sum(stripe.length_m for stripe in stripes),
            4,
        ),
        "connector_length_m": round(
            sum(float(connector["length_m"]) for connector in connectors),
            4,
        ),
        "all_yaw_inset_radius_m": round(radius, 4),
        "safe_region_area_m2": round(float(safe_region.area), 4),
        "stripe_yaw_rad": round(yaw, 6),
    }
    return plan, summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate one continuous inner-safe V2 dry-run PlanPath JSON.")
    parser.add_argument("--map", required=True, help="OpenMower map.json")
    parser.add_argument("--config", default="tools/coverage_lab/configs/default.yaml", help="coverage lab config YAML")
    parser.add_argument("--source-plan", help="existing V2 planpath_compat.json to copy stripe direction from")
    parser.add_argument("--output", required=True, help="output planpath_compat JSON")
    parser.add_argument("--summary-output", help="optional summary JSON")
    parser.add_argument("--area-index", type=int, help="mow area index to use")
    parser.add_argument("--spacing-m", type=float, default=DEFAULT_SPACING_M, help="stripe spacing inside safe region")
    parser.add_argument("--sample-step-m", type=float, default=DEFAULT_SAMPLE_STEP_M, help="pose sample spacing")
    parser.add_argument(
        "--min-stripe-length-m",
        type=float,
        default=DEFAULT_MIN_STRIPE_LENGTH_M,
        help="minimum clipped stripe length",
    )
    parser.add_argument(
        "--extra-inset-m",
        type=float,
        default=DEFAULT_EXTRA_INSET_M,
        help="extra all-yaw inset beyond configured footprint+safety margin",
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    plan, summary = build_continuous_dry_run(args)
    output = pathlib.Path(args.output).resolve()
    _write_json(output, plan)
    if args.summary_output:
        _write_json(pathlib.Path(args.summary_output).resolve(), summary)
    print(
        f"Wrote continuous dry-run plan: {output}\n"
        f"paths={summary['path_count']} poses={summary['pose_count']} "
        f"stripes={summary['stripe_count']} connectors={summary['connector_count']} "
        f"unsafe={summary['unsafe_sample_count']} length_m={summary['path_length_m']:.2f}"
    )


if __name__ == "__main__":
    main()
