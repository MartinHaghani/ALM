#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import datetime as dt
import heapq
import html
import json
import math
import pathlib
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any, Iterable

import lab_geometry


LAB_DIR = pathlib.Path(__file__).resolve().parent
REPO_DIR = LAB_DIR.parents[1]
DEFAULT_CONFIG = LAB_DIR / "configs" / "default.yaml"
DEFAULT_RUNS_DIR = LAB_DIR / "runs"
DEFAULT_GOOGLE_EARTH_MAP_DIR = LAB_DIR / "data" / "maps" / "google_earth"
EPSILON = 1e-6
EARTH_RADIUS_M = 6378137.0
PRIMARY_PROFILE = "mowrator_zero_turn"
F2C_TINY_RADIUS_PROFILE = "f2c_tiny_radius"


@dataclass
class MapArea:
    id: str
    name: str
    type: str
    active: bool
    outline: list[tuple[float, float]]


@dataclass
class Lawn:
    area: MapArea
    holes: list[MapArea]


@dataclass
class OpenMowerMap:
    path: pathlib.Path
    raw: dict[str, Any]
    lawns: list[Lawn]
    obstacles: list[MapArea]
    docking_stations: list[dict[str, Any]]


def die(message: str) -> None:
    raise SystemExit(f"coverage_lab: error: {message}")


def read_json(path: pathlib.Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        die(f"file does not exist: {path}")
    except json.JSONDecodeError as exc:
        die(f"invalid JSON in {path}: {exc}")


def write_json(path: pathlib.Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")


def slugify(value: str, fallback: str = "item") -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", value.strip()).strip("-._")
    return slug or fallback


def unique_id(base: str, used: set[str]) -> str:
    candidate = base
    suffix = 2
    while candidate in used:
        candidate = f"{base}-{suffix}"
        suffix += 1
    used.add(candidate)
    return candidate


def parse_point(raw: Any, where: str) -> tuple[float, float]:
    if isinstance(raw, dict):
        if "x" not in raw or "y" not in raw:
            die(f"{where} point is missing x or y")
        return (float(raw["x"]), float(raw["y"]))
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return (float(raw[0]), float(raw[1]))
    die(f"{where} point must be an object with x/y or a two-value array")


def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def signed_area(ring: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        ring[i][0] * ring[i + 1][1] - ring[i + 1][0] * ring[i][1]
        for i in range(len(ring) - 1)
    )


def normalize_ring(raw_points: list[Any], where: str, repair_rings: bool) -> list[tuple[float, float]]:
    points = [parse_point(p, f"{where}[{i}]") for i, p in enumerate(raw_points)]
    if len(points) < 4:
        die(f"{where} must contain at least four points including the closing point")
    if dist(points[0], points[-1]) > EPSILON:
        if not repair_rings:
            die(f"{where} is not closed; first and last points differ")
        points.append(points[0])
    if len(points) < 4 or abs(signed_area(points)) < EPSILON:
        die(f"{where} has too little area")
    return points


def point_in_ring(point: tuple[float, float], ring: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    for i in range(len(ring) - 1):
        x1, y1 = ring[i]
        x2, y2 = ring[i + 1]
        if ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / ((y2 - y1) or EPSILON) + x1
        ):
            inside = not inside
    return inside


def point_in_lawn(point: tuple[float, float], lawn: Lawn) -> bool:
    if not point_in_ring(point, lawn.area.outline):
        return False
    return not any(point_in_ring(point, hole.outline) for hole in lawn.holes)


def ring_centroid(ring: list[tuple[float, float]]) -> tuple[float, float]:
    body = ring[:-1] if dist(ring[0], ring[-1]) <= EPSILON else ring
    return (
        sum(p[0] for p in body) / max(1, len(body)),
        sum(p[1] for p in body) / max(1, len(body)),
    )


def parse_map(path: pathlib.Path, repair_rings: bool = False) -> OpenMowerMap:
    raw = read_json(path)
    if not isinstance(raw, dict):
        die("map root must be an object")
    areas_raw = raw.get("areas")
    if not isinstance(areas_raw, list) or not areas_raw:
        die("map must contain a non-empty areas array")

    areas: list[MapArea] = []
    for idx, area in enumerate(areas_raw):
        if not isinstance(area, dict):
            die(f"areas[{idx}] must be an object")
        properties = area.get("properties") or {}
        if not isinstance(properties, dict):
            die(f"areas[{idx}].properties must be an object")
        outline_raw = area.get("outline")
        if not isinstance(outline_raw, list):
            die(f"areas[{idx}].outline must be a point array")
        area_id = str(area.get("id") or f"area-{idx}")
        area_type = str(properties.get("type", "draft"))
        areas.append(
            MapArea(
                id=area_id,
                name=str(properties.get("name") or area_id),
                type=area_type,
                active=bool(properties.get("active", True)),
                outline=normalize_ring(outline_raw, f"areas[{idx}].outline", repair_rings),
            )
        )

    mow_areas = [a for a in areas if a.active and a.type == "mow"]
    obstacles = [a for a in areas if a.active and a.type == "obstacle"]
    if not mow_areas:
        die("map contains no active mow areas")

    lawns: list[Lawn] = []
    for area in mow_areas:
        holes = [obs for obs in obstacles if point_in_ring(ring_centroid(obs.outline), area.outline)]
        lawns.append(Lawn(area=area, holes=holes))

    docking = raw.get("docking_stations") or []
    if not isinstance(docking, list):
        die("docking_stations must be an array when present")

    return OpenMowerMap(path=path, raw=raw, lawns=lawns, obstacles=obstacles, docking_stations=docking)


def polygon_area(lawn: Lawn) -> float:
    return abs(signed_area(lawn.area.outline)) - sum(abs(signed_area(h.outline)) for h in lawn.holes)


def bbox(points: Iterable[tuple[float, float]]) -> tuple[float, float, float, float]:
    pts = list(points)
    if not pts:
        return (0.0, 0.0, 1.0, 1.0)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def xml_local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def direct_child_text(node: ET.Element, child_name: str) -> str:
    for child in list(node):
        if xml_local_name(child.tag) == child_name:
            return (child.text or "").strip()
    return ""


def parse_kml_coordinates(text: str, where: str) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for token in text.split():
        fields = [part.strip() for part in token.split(",")]
        if len(fields) < 2 or not fields[0] or not fields[1]:
            die(f"{where} contains an invalid KML coordinate token: {token!r}")
        try:
            lon = float(fields[0])
            lat = float(fields[1])
        except ValueError as exc:
            die(f"{where} contains a non-numeric KML coordinate token {token!r}: {exc}")
        points.append((lon, lat))
    if len(points) < 3:
        die(f"{where} must contain at least three coordinate points")
    if points[0] != points[-1]:
        points.append(points[0])
    if len(points) < 4:
        die(f"{where} must contain at least four points after closing")
    return points


def polygon_outer_coordinates(polygon: ET.Element, where: str) -> list[tuple[float, float]]:
    for outer in polygon.iter():
        if xml_local_name(outer.tag) != "outerBoundaryIs":
            continue
        for ring in outer.iter():
            if xml_local_name(ring.tag) != "LinearRing":
                continue
            for child in ring.iter():
                if xml_local_name(child.tag) == "coordinates":
                    return parse_kml_coordinates(child.text or "", where)
    die(f"{where} does not contain an outerBoundaryIs/LinearRing/coordinates polygon")


def kml_polygon_type_and_name(raw_name: str, default_type: str | None, where: str) -> tuple[str, str]:
    name = raw_name.strip() or where
    for area_type in ("mow", "obstacle", "nav"):
        prefix = f"{area_type}:"
        if name.lower().startswith(prefix):
            display_name = name[len(prefix):].strip() or area_type.title()
            return area_type, display_name
    if default_type:
        return default_type, name
    die(f"{where} name must start with mow:, obstacle:, or nav:; got {name!r}")


def project_lonlat(
    lon: float,
    lat: float,
    origin_lon: float,
    origin_lat: float,
) -> tuple[float, float]:
    x = EARTH_RADIUS_M * math.radians(lon - origin_lon) * math.cos(math.radians(origin_lat))
    y = EARTH_RADIUS_M * math.radians(lat - origin_lat)
    return (x, y)


def tuple_point_segment_distance(
    point: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= EPSILON:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def simplify_closed_ring(
    points: list[tuple[float, float]],
    tolerance_m: float,
) -> list[tuple[float, float]]:
    if tolerance_m <= EPSILON or len(points) <= 5:
        return points
    body = points[:-1] if points[0] == points[-1] else list(points)
    changed = True
    while changed and len(body) > 3:
        changed = False
        for i, point in enumerate(list(body)):
            if len(body) <= 3:
                break
            prev_pt = body[i - 1]
            next_pt = body[(i + 1) % len(body)]
            if tuple_point_segment_distance(point, prev_pt, next_pt) <= tolerance_m:
                del body[i]
                changed = True
                break
    return body + [body[0]]


def output_path_for_kml(kml_path: pathlib.Path, output_dir: pathlib.Path) -> pathlib.Path:
    return output_dir / f"{slugify(kml_path.stem, 'google-earth-map')}.json"


def printable_repo_path(path: pathlib.Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_DIR))
    except ValueError:
        return str(path)


def ensure_plan_output_visible_to_docker(paths: list[pathlib.Path]) -> None:
    for path in paths:
        try:
            path.resolve().relative_to(REPO_DIR)
        except ValueError:
            die("--plan requires converted JSON outputs to be under this repository so Docker can read them")


def convert_kml_file(
    kml_path: pathlib.Path,
    output_path: pathlib.Path,
    default_type: str | None,
    simplify_tolerance_m: float = 0.0,
) -> pathlib.Path:
    if kml_path.suffix.lower() != ".kml":
        die(f"only .kml files are supported: {kml_path}")
    if not kml_path.is_file():
        die(f"KML file does not exist: {kml_path}")

    try:
        root = ET.parse(kml_path).getroot()
    except ET.ParseError as exc:
        die(f"invalid KML XML in {kml_path}: {exc}")

    raw_areas: list[dict[str, Any]] = []
    placemark_count = 0
    for placemark in root.iter():
        if xml_local_name(placemark.tag) != "Placemark":
            continue
        placemark_count += 1
        polygons = [node for node in placemark.iter() if xml_local_name(node.tag) == "Polygon"]
        if not polygons:
            continue
        placemark_name = direct_child_text(placemark, "name") or f"Placemark {placemark_count}"
        area_type, display_name = kml_polygon_type_and_name(placemark_name, default_type, f"Placemark {placemark_count}")
        for polygon_i, polygon in enumerate(polygons):
            coords = polygon_outer_coordinates(polygon, f"{placemark_name} polygon {polygon_i + 1}")
            raw_areas.append(
                {
                    "type": area_type,
                    "name": display_name if len(polygons) == 1 else f"{display_name} {polygon_i + 1}",
                    "lonlat": coords,
                }
            )

    if not raw_areas:
        die(f"KML contains no Placemark Polygon geometry: {kml_path}")

    mow_areas = [area for area in raw_areas if area["type"] == "mow"]
    if not mow_areas:
        die("KML conversion requires at least one mow: polygon")
    origin_lon, origin_lat = mow_areas[0]["lonlat"][0]

    used_ids: set[str] = set()
    areas = []
    for index, area in enumerate(raw_areas):
        area_id = unique_id(slugify(area["name"].lower(), f"area-{index + 1}"), used_ids)
        projected = []
        for lon, lat in area["lonlat"]:
            x, y = project_lonlat(lon, lat, origin_lon, origin_lat)
            projected.append((x, y))
        projected = simplify_closed_ring(projected, simplify_tolerance_m)
        outline = [{"x": round(x, 3), "y": round(y, 3)} for x, y in projected]
        areas.append(
            {
                "id": area_id,
                "properties": {
                    "active": True,
                    "name": area["name"],
                    "type": area["type"],
                },
                "outline": outline,
            }
        )

    converted = {
        "areas": areas,
        "docking_stations": [],
        "metadata": {
            "schema": "open_mower.coverage_lab.kml_conversion.v0",
            "source_format": "kml",
            "source_kml": str(kml_path),
            "source_kml_name": kml_path.name,
            "origin": {
                "latitude": origin_lat,
                "longitude": origin_lon,
            },
            "projection": "Local tangent equirectangular meters; x=east, y=north; origin is first point of first mow polygon.",
            "naming": "KML Placemark names use mow:, obstacle:, or nav: prefixes.",
            "simplify_tolerance_m": simplify_tolerance_m,
        },
    }
    write_json(output_path, converted)
    parse_map(output_path, repair_rings=False)
    return output_path


def load_config(path: pathlib.Path) -> dict[str, Any]:
    try:
        import yaml
    except ImportError:
        die("PyYAML is required for planning; use tools/coverage_lab/bin/coverage_lab")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        die(f"config must be a YAML object: {path}")
    return data


def heading_between(a: dict[str, float], b: dict[str, float]) -> float:
    return math.atan2(b["y"] - a["y"], b["x"] - a["x"])


def fill_missing_yaws(poses: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not poses:
        return poses
    for i, pose in enumerate(poses):
        yaw = pose.get("yaw")
        if isinstance(yaw, (int, float)) and math.isfinite(yaw):
            continue
        if i + 1 < len(poses):
            pose["yaw"] = heading_between(pose, poses[i + 1])
        elif i > 0:
            pose["yaw"] = heading_between(poses[i - 1], pose)
        else:
            pose["yaw"] = 0.0
    return poses


def normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def angle_delta(start: float, end: float) -> float:
    return normalize_angle(end - start)


def parse_xy_pair(raw: Any, where: str, default: tuple[float, float]) -> tuple[float, float]:
    if raw is None:
        return default
    if not isinstance(raw, (list, tuple)) or len(raw) < 2:
        die(f"{where} must be a two-value array")
    return (float(raw[0]), float(raw[1]))


def tool_center_offset(config: dict[str, Any]) -> tuple[float, float]:
    return parse_xy_pair(config.get("tool_center_offset"), "tool_center_offset", (0.0, 0.0))


def outline_clearance(config: dict[str, Any]) -> float:
    raw = config.get("outline_clearance_m", "auto")
    tool_width = float(config["tool_width"])
    if raw is not None and str(raw).lower() != "auto":
        return max(tool_width / 2.0, float(raw))

    offset = tool_center_offset(config)
    footprint = safety_footprint(config)
    if len(footprint) < 3:
        return tool_width / 2.0
    return max(
        tool_width / 2.0,
        max(math.hypot(float(point[0]) - offset[0], float(point[1]) - offset[1]) for point in footprint),
    )


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def base_pose_from_tool_pose(tool_pose: dict[str, Any], offset: tuple[float, float]) -> dict[str, Any]:
    yaw = float(tool_pose.get("yaw", 0.0))
    dx, dy = rotate_xy(offset[0], offset[1], yaw)
    pose = dict(tool_pose)
    pose["x"] = float(tool_pose["x"]) - dx
    pose["y"] = float(tool_pose["y"]) - dy
    pose["yaw"] = yaw
    return pose


def tool_pose_from_base_pose(base_pose: dict[str, Any], offset: tuple[float, float]) -> dict[str, Any]:
    yaw = float(base_pose.get("yaw", 0.0))
    dx, dy = rotate_xy(offset[0], offset[1], yaw)
    pose = dict(base_pose)
    pose["x"] = float(base_pose["x"]) + dx
    pose["y"] = float(base_pose["y"]) + dy
    pose["yaw"] = yaw
    return pose


def local_to_world_xy(pose: dict[str, Any], local_point: tuple[float, float]) -> tuple[float, float]:
    dx, dy = rotate_xy(local_point[0], local_point[1], float(pose.get("yaw", 0.0)))
    return (float(pose["x"]) + dx, float(pose["y"]) + dy)


def world_to_local_xy(reference_pose: dict[str, Any], point: tuple[float, float]) -> tuple[float, float]:
    yaw = float(reference_pose.get("yaw", 0.0))
    dx = point[0] - float(reference_pose["x"])
    dy = point[1] - float(reference_pose["y"])
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def wheel_track_m(config: dict[str, Any]) -> float:
    return max(EPSILON, float(config.get("wheel_track_m", 0.58)))


def wheel_contact_x_m(config: dict[str, Any]) -> float:
    return float(config.get("wheel_contact_x_m", 0.0))


def wheel_local_point(config: dict[str, Any], side: str) -> tuple[float, float]:
    half_track = wheel_track_m(config) / 2.0
    return (wheel_contact_x_m(config), half_track if side == "left" else -half_track)


def wheel_points_for_pose(pose: dict[str, Any], config: dict[str, Any]) -> dict[str, dict[str, float]]:
    left = local_to_world_xy(pose, wheel_local_point(config, "left"))
    right = local_to_world_xy(pose, wheel_local_point(config, "right"))
    return {
        "left": {"x": left[0], "y": left[1]},
        "right": {"x": right[0], "y": right[1]},
    }


def pose_with_wheel_at(
    anchor_point: tuple[float, float],
    local_wheel_point: tuple[float, float],
    yaw: float,
) -> dict[str, float]:
    dx, dy = rotate_xy(local_wheel_point[0], local_wheel_point[1], yaw)
    return {
        "x": anchor_point[0] - dx,
        "y": anchor_point[1] - dy,
        "yaw": normalize_angle(yaw),
    }


def sample_polyline_tool_poses(
    points: list[tuple[float, float]],
    section: str,
    sample_step: float,
    direction: str = "forward",
    cutting_enabled: bool | None = None,
) -> list[dict[str, Any]]:
    if cutting_enabled is None:
        cutting_enabled = section in {"headland", "swath"}
    if not points:
        return []
    if len(points) == 1:
        return [
            {
                "x": points[0][0],
                "y": points[0][1],
                "yaw": 0.0,
                "section": section,
                "direction": direction,
                "cutting_enabled": cutting_enabled,
            }
        ]

    poses: list[dict[str, Any]] = []
    step = sample_step if sample_step > EPSILON else float("inf")
    for seg_i in range(len(points) - 1):
        a, b = points[seg_i], points[seg_i + 1]
        length = dist(a, b)
        if length <= EPSILON:
            continue
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        count = max(1, int(math.ceil(length / step))) if math.isfinite(step) else 1
        start_i = 0 if not poses else 1
        for i in range(start_i, count + 1):
            t = i / count
            poses.append(
                {
                    "x": a[0] + (b[0] - a[0]) * t,
                    "y": a[1] + (b[1] - a[1]) * t,
                    "yaw": yaw,
                    "section": section,
                    "direction": direction,
                    "cutting_enabled": cutting_enabled,
                }
            )
    if not poses:
        poses.append(
            {
                "x": points[0][0],
                "y": points[0][1],
                "yaw": 0.0,
                "section": section,
                "direction": direction,
                "cutting_enabled": cutting_enabled,
            }
        )
    return fill_missing_yaws(poses)


def poses_to_base_link(tool_poses: list[dict[str, Any]], offset: tuple[float, float]) -> list[dict[str, Any]]:
    return [base_pose_from_tool_pose(pose, offset) for pose in tool_poses]


def poses_to_tool_center(base_poses: list[dict[str, Any]], offset: tuple[float, float]) -> list[dict[str, Any]]:
    return [tool_pose_from_base_pose(pose, offset) for pose in base_poses]


def same_pose(a: dict[str, Any], b: dict[str, Any]) -> bool:
    return (
        abs(float(a["x"]) - float(b["x"])) <= EPSILON
        and abs(float(a["y"]) - float(b["y"])) <= EPSILON
        and abs(angle_delta(float(a.get("yaw", 0.0)), float(b.get("yaw", 0.0)))) <= EPSILON
    )


def points_to_path(points: list[tuple[float, float]], section: str) -> list[dict[str, Any]]:
    poses = [{"x": x, "y": y, "yaw": None, "section": section} for x, y in points]
    return fill_missing_yaws(poses)


def f2c_point(point: Any) -> tuple[float, float]:
    return (float(point.getX()), float(point.getY()))


def f2c_ring_points(ring: Any) -> list[tuple[float, float]]:
    return [(float(ring.getX(i)), float(ring.getY(i))) for i in range(ring.size())]


def f2c_cells_rings(cells: Any) -> list[list[tuple[float, float]]]:
    rings: list[list[tuple[float, float]]] = []
    for cell_i in range(cells.size()):
        cell = cells.getGeometry(cell_i)
        for ring_i in range(cell.size()):
            ring = cell.getGeometry(ring_i)
            pts = f2c_ring_points(ring)
            if len(pts) >= 2:
                rings.append(pts)
    return rings


def f2c_swaths_to_json(swaths: Any) -> list[dict[str, Any]]:
    flat_swaths = []

    def collect(obj: Any) -> None:
        if hasattr(obj, "getPath"):
            flat_swaths.append(obj)
            return
        if hasattr(obj, "size") and hasattr(obj, "at"):
            for child_i in range(obj.size()):
                collect(obj.at(child_i))
            return
        die(f"unsupported Fields2Cover swath object: {type(obj).__name__}")

    collect(swaths)

    result = []
    for i, swath in enumerate(flat_swaths):
        line = swath.getPath()
        points = [(float(line.getX(j)), float(line.getY(j))) for j in range(line.size())]
        result.append(
            {
                "index": i,
                "width": float(swath.getWidth()),
                "length_m": float(swath.length()),
                "points": [{"x": x, "y": y} for x, y in points],
            }
        )
    return result


def f2c_path_to_poses(path: Any, f2c: Any) -> list[dict[str, Any]]:
    poses: list[dict[str, Any]] = []
    for i in range(path.size()):
        state = path.getState(i)
        point = state.point
        section = "swath" if state.type == f2c.PathSectionType_SWATH else "turn"
        direction = "backward" if state.dir == f2c.PathDirection_BACKWARD else "forward"
        poses.append(
            {
                "x": float(point.getX()),
                "y": float(point.getY()),
                "yaw": float(state.angle) if math.isfinite(float(state.angle)) else None,
                "section": section,
                "direction": direction,
                "cutting_enabled": section == "swath",
                "len": float(state.len),
            }
        )
    return fill_missing_yaws(poses)


def make_f2c_ring(points: list[tuple[float, float]], f2c: Any) -> Any:
    return f2c.LinearRing(f2c.VectorPoint([f2c.Point(x, y) for x, y in points]))


def make_f2c_cells(lawn: Lawn, f2c: Any) -> Any:
    cell = f2c.Cell(make_f2c_ring(lawn.area.outline, f2c))
    for hole in lawn.holes:
        cell.addRing(make_f2c_ring(hole.outline, f2c))
    return f2c.Cells(cell)


def make_f2c_cells_from_polygon(polygon: Any, f2c: Any) -> Any:
    """Build an F2C ``Cells`` from a shapely Polygon.

    Used by P0/P1: footprint-disk erosion and BCD cells produce shapely
    polygons that must be handed back to F2C for swath generation.
    """
    outer_ring = lab_geometry.polygon_outer_ring(polygon)
    cell = f2c.Cell(make_f2c_ring(outer_ring, f2c))
    for hole_ring in lab_geometry.polygon_holes(polygon):
        cell.addRing(make_f2c_ring(hole_ring, f2c))
    return f2c.Cells(cell)


def swath_angle_from_swaths_json(swaths_json: list[dict[str, Any]]) -> float | None:
    """Best-effort: extract the swath direction (radians) from F2C swath JSON.

    Used so BCD can be aligned to F2C's automatically-chosen swath angle in
    ``best_swath_length`` mode, before re-running F2C per cell with a fixed
    angle.
    """
    for swath in swaths_json:
        points = swath.get("points") or []
        if len(points) < 2:
            continue
        a = points[0]
        b = points[-1]
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        if math.hypot(dx, dy) > EPSILON:
            return math.atan2(dy, dx)
    return None


def configured_swath_angle(config: dict[str, Any]) -> float | None:
    """If the swath angle is fixed in config, return it (radians); else None."""
    angle_cfg = config.get("fields2cover", {}).get("swath_angle", {}) or {}
    mode = str(angle_cfg.get("mode", "best_swath_length")).lower()
    if mode == "fixed":
        return math.radians(float(angle_cfg.get("degrees", 0.0)))
    return None


def planner_turn(config: dict[str, Any], f2c: Any) -> Any:
    name = str(config["fields2cover"].get("turn_planner", "dubins")).lower()
    if name == "dubins":
        return f2c.PP_DubinsCurves()
    if name == "dubins_cc":
        return f2c.PP_DubinsCurvesCC()
    if name == "reeds_shepp":
        return f2c.PP_ReedsSheppCurves()
    if name == "reeds_shepp_hc":
        return f2c.PP_ReedsSheppCurvesHC()
    die(f"unsupported turn planner: {name}")


def sort_swaths(swaths: Any, config: dict[str, Any], f2c: Any) -> Any:
    name = str(config["fields2cover"].get("route_order", "boustrophedon")).lower()
    variant = int(config["fields2cover"].get("route_variant", 0))
    if name == "boustrophedon":
        planner = f2c.RP_Boustrophedon()
        try:
            return planner.genSortedSwaths(swaths, variant)
        except TypeError:
            if variant == 0:
                return planner.genSortedSwaths(swaths)
            raise
    if name == "snake":
        planner = f2c.RP_Snake()
        try:
            return planner.genSortedSwaths(swaths, variant)
        except TypeError:
            if variant == 0:
                return planner.genSortedSwaths(swaths)
            raise
    die(f"unsupported route_order: {name}")


def generate_swaths(mainland: Any, config: dict[str, Any], f2c: Any) -> Any:
    swath_gen = f2c.SG_BruteForce()
    tool_width = float(config["tool_width"])
    angle_cfg = config["fields2cover"].get("swath_angle", {})
    mode = str(angle_cfg.get("mode", "best_swath_length")).lower()
    target = mainland.getGeometry(0) if mainland.size() == 1 else mainland
    if mode == "fixed":
        return swath_gen.generateSwaths(math.radians(float(angle_cfg.get("degrees", 0.0))), tool_width, target)
    if mode == "best_n_swath":
        return swath_gen.generateBestSwaths(f2c.OBJ_NSwath(), tool_width, target)
    if mode == "best_swath_length":
        return swath_gen.generateBestSwaths(f2c.OBJ_SwathLength(), tool_width, target)
    die(f"unsupported swath_angle.mode: {mode}")


def profile_config(config: dict[str, Any], profile_name: str) -> dict[str, Any]:
    merged = copy.deepcopy(config)
    profile_overrides = ((config.get("profiles") or {}).get(profile_name) or {})
    fields = copy.deepcopy(config.get("fields2cover") or {})
    for key in ("turn_planner", "min_turning_radius", "max_diff_curv"):
        if key in profile_overrides:
            fields[key] = profile_overrides[key]
    merged["fields2cover"] = fields
    return merged


def primary_profile_name(config: dict[str, Any]) -> str:
    profiles = config.get("profiles") or {}
    return str(profiles.get("primary") or PRIMARY_PROFILE)


def comparison_profile_names(config: dict[str, Any]) -> list[str]:
    profiles = config.get("profiles") or {}
    raw = profiles["comparisons"] if "comparisons" in profiles else [F2C_TINY_RADIUS_PROFILE]
    if not isinstance(raw, list):
        die("profiles.comparisons must be a list")
    return [str(name) for name in raw]


def make_path_record(
    *,
    is_outline: bool,
    area_index: int,
    lawn: Lawn,
    label: str,
    frame_id: str,
    base_poses: list[dict[str, Any]],
    tool_poses: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "is_outline": is_outline,
        "area_index": area_index,
        "area_id": lawn.area.id,
        "label": label,
        "path": {"frame_id": frame_id, "poses": base_poses},
        "tool_path": {"frame_id": frame_id, "poses": tool_poses},
    }


def append_pose_pair(
    base_poses: list[dict[str, Any]],
    tool_poses: list[dict[str, Any]],
    base_pose: dict[str, Any],
    tool_pose: dict[str, Any],
) -> None:
    if base_poses and same_pose(base_poses[-1], base_pose) and base_poses[-1].get("section") == base_pose.get("section"):
        return
    base_poses.append(base_pose)
    tool_poses.append(tool_pose)


def pose_list_length(poses: list[dict[str, Any]]) -> float:
    return sum(dist(pose_xy(a), pose_xy(b)) for a, b in zip(poses, poses[1:]))


def pose_cutting_enabled(path: dict[str, Any], pose: dict[str, Any]) -> bool:
    if "cutting_enabled" in pose:
        return bool(pose["cutting_enabled"])
    return bool(path.get("is_outline")) or pose.get("section") in {"headland", "swath"}


def turn_cutting_enabled(config: dict[str, Any], direction: str) -> bool:
    raw_mode = config.get("turn_cutting_mode", "off")
    if isinstance(raw_mode, bool):
        mode = "all" if raw_mode else "off"
    else:
        mode = str(raw_mode).lower()
    if mode == "off":
        return False
    if mode == "forward_only":
        return direction == "forward"
    if mode == "all":
        return True
    die(f"unsupported turn_cutting_mode: {mode}")


def footprint_is_safe_for_lawn(lawn: Lawn, corners: list[tuple[float, float]]) -> bool:
    return all(point_in_lawn(corner, lawn) for corner in corners)


def footprint_unsafe_sample_count_for_lawn(
    lawn: Lawn,
    poses: list[dict[str, Any]],
    footprint: list[list[float]],
) -> int:
    return sum(1 for pose in poses if not footprint_is_safe_for_lawn(lawn, transform_footprint(pose, footprint)))


def turn_unsafe_sample_count(lawn: Lawn, poses: list[dict[str, Any]], config: dict[str, Any]) -> int:
    footprint = safety_footprint(config)
    return footprint_unsafe_sample_count_for_lawn(lawn, poses, footprint)


def ring_distance(point: tuple[float, float], ring: list[tuple[float, float]]) -> float:
    if len(ring) < 2:
        return float("inf")
    return min(point_segment_distance(point, ring[i], ring[i + 1]) for i in range(len(ring) - 1))


def footprint_clearance_m(lawn: Lawn, pose: dict[str, Any], footprint: list[list[float]]) -> float:
    corners = transform_footprint(pose, footprint)
    if not footprint_is_safe_for_lawn(lawn, corners):
        return -1.0
    clearances: list[float] = []
    for corner in corners:
        clearances.append(ring_distance(corner, lawn.area.outline))
        clearances.extend(ring_distance(corner, hole.outline) for hole in lawn.holes)
    return min(clearances) if clearances else 0.0


def turn_min_clearance_m(lawn: Lawn, poses: list[dict[str, Any]], config: dict[str, Any]) -> float:
    footprint = safety_footprint(config)
    if not poses:
        return 0.0
    return min(footprint_clearance_m(lawn, pose, footprint) for pose in poses)


def turn_reverse_length(start_pose: dict[str, Any], poses: list[dict[str, Any]]) -> float:
    total = 0.0
    all_poses = [start_pose] + poses
    for a, b in zip(all_poses, all_poses[1:]):
        if a.get("direction") == "backward" or b.get("direction") == "backward":
            total += dist(pose_xy(a), pose_xy(b))
    return total


def yaw_motion(poses: list[dict[str, Any]]) -> float:
    return sum(
        abs(angle_delta(float(a.get("yaw", 0.0)), float(b.get("yaw", 0.0))))
        for a, b in zip(poses, poses[1:])
    )


def pose_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def turn_motion_cost(start_pose: dict[str, Any], poses: list[dict[str, Any]], config: dict[str, Any]) -> float:
    all_poses = [start_pose] + poses
    pivot_cost = float(config.get("turn_lattice_pivot_cost_m_per_rad", 0.18))
    reverse_penalty = float(config.get("turn_lattice_reverse_penalty", 0.10))
    switch_penalty = float(config.get("turn_lattice_switch_penalty_m", 0.04))
    total = 0.0
    previous_direction = start_pose.get("direction", "forward")
    for a, b in zip(all_poses, all_poses[1:]):
        length = pose_distance(a, b)
        yaw = abs(angle_delta(float(a.get("yaw", 0.0)), float(b.get("yaw", 0.0))))
        total += length
        if length <= EPSILON:
            total += yaw * pivot_cost
        if b.get("direction") == "backward":
            total += length * reverse_penalty
        if previous_direction != b.get("direction") and b.get("direction") in {"forward", "backward"}:
            total += switch_penalty
        previous_direction = b.get("direction", previous_direction)
    return total


def mark_turn_pose(
    pose: dict[str, Any],
    *,
    direction: str,
    turn_planner: str,
    primitive: str,
    cutting_enabled: bool,
) -> dict[str, Any]:
    marked = dict(pose)
    marked["section"] = "turn"
    marked["direction"] = direction
    marked["turn_planner"] = turn_planner
    marked["turn_primitive"] = primitive
    marked["cutting_enabled"] = cutting_enabled
    return marked


def map_point_payload(point: tuple[float, float] | None) -> dict[str, float] | None:
    if point is None:
        return None
    return {"x": float(point[0]), "y": float(point[1])}


def wheel_anchor_pose_metadata(
    pose: dict[str, Any],
    config: dict[str, Any],
    *,
    direction: str,
    phase: str,
    maneuver_id: str,
    turn_leg: int,
    pivot_wheel: str | None,
    anchor_point: tuple[float, float] | None,
    target_anchor_point: tuple[float, float] | None,
    target_wheel_error_m: float,
    min_clearance_m: float,
    reverse_distance_m: float,
    pivot_angle_deg: float,
) -> dict[str, Any]:
    marked = mark_turn_pose(
        pose,
        direction=direction,
        turn_planner="wheel_anchor",
        primitive=phase,
        cutting_enabled=turn_cutting_enabled(config, direction),
    )
    marked["maneuver_id"] = maneuver_id
    marked["maneuver_type"] = "wheel_anchor_turn"
    marked["phase"] = phase
    marked["turn_leg"] = turn_leg
    marked["pivot_wheel"] = pivot_wheel
    marked["anchor_point"] = map_point_payload(anchor_point)
    marked["target_anchor_point"] = map_point_payload(target_anchor_point)
    marked["left_wheel"] = wheel_points_for_pose(marked, config)["left"]
    marked["right_wheel"] = wheel_points_for_pose(marked, config)["right"]
    marked["blade_enabled"] = marked["cutting_enabled"]
    marked["target_wheel_error_m"] = target_wheel_error_m
    marked["min_clearance_m"] = min_clearance_m
    marked["reverse_distance_m"] = reverse_distance_m
    marked["pivot_angle_deg"] = pivot_angle_deg
    return marked


def annotate_turn_legs(poses: list[dict[str, Any]]) -> list[dict[str, Any]]:
    leg = 0
    previous: tuple[str, str] | None = None
    for pose in poses:
        current = (str(pose.get("direction", "")), str(pose.get("turn_primitive", "")))
        if current != previous:
            leg += 1
            previous = current
        pose["turn_leg"] = leg
    return poses


def sample_pivot_base_poses(base_pose: dict[str, Any], target_yaw: float, config: dict[str, Any]) -> list[dict[str, Any]]:
    start_yaw = float(base_pose.get("yaw", 0.0))
    delta = angle_delta(start_yaw, target_yaw)
    if abs(delta) <= EPSILON:
        return []
    step_rad = math.radians(max(1.0, float(config.get("pivot_yaw_step_degrees", 10.0))))
    count = max(1, int(math.ceil(abs(delta) / step_rad)))
    poses = []
    for i in range(1, count + 1):
        yaw = normalize_angle(start_yaw + delta * (i / count))
        pose = dict(base_pose)
        pose["yaw"] = yaw
        pose["section"] = "pivot"
        pose["direction"] = "rotate"
        pose["cutting_enabled"] = False
        poses.append(pose)
    return poses


def sample_line_base_poses(
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    yaw: float,
    sample_step: float,
    section: str,
) -> list[dict[str, Any]]:
    length = math.hypot(float(end_pose["x"]) - float(start_pose["x"]), float(end_pose["y"]) - float(start_pose["y"]))
    if length <= EPSILON:
        pose = dict(end_pose)
        pose["yaw"] = yaw
        pose["section"] = section
        pose["direction"] = "forward"
        return [pose]
    step = sample_step if sample_step > EPSILON else length
    count = max(1, int(math.ceil(length / step)))
    poses = []
    for i in range(1, count + 1):
        t = i / count
        poses.append(
            {
                "x": float(start_pose["x"]) + (float(end_pose["x"]) - float(start_pose["x"])) * t,
                "y": float(start_pose["y"]) + (float(end_pose["y"]) - float(start_pose["y"])) * t,
                "yaw": yaw,
                "section": section,
                "direction": "forward",
            }
        )
    return poses


def sample_pivot_turn_poses(
    start_pose: dict[str, Any],
    target_yaw: float,
    sample_step: float,
    config: dict[str, Any],
    turn_planner: str = "lattice",
) -> list[dict[str, Any]]:
    start_yaw = float(start_pose.get("yaw", 0.0))
    delta = angle_delta(start_yaw, target_yaw)
    if abs(delta) <= math.radians(1.0):
        return []
    yaw_step = math.radians(max(1.0, float(config.get("turn_lattice_pivot_step_degrees", 12.0))))
    count = max(1, int(math.ceil(abs(delta) / yaw_step)))
    poses: list[dict[str, Any]] = []
    for i in range(1, count + 1):
        yaw = normalize_angle(start_yaw + delta * i / count)
        pose = dict(start_pose)
        pose["yaw"] = yaw
        poses.append(
            mark_turn_pose(
                pose,
                direction="rotate",
                turn_planner=turn_planner,
                primitive="pivot",
                cutting_enabled=False,
            )
        )
    return poses


def sample_straight_turn_poses(
    start_pose: dict[str, Any],
    end_xy: tuple[float, float],
    yaw: float,
    direction: str,
    sample_step: float,
    config: dict[str, Any],
    turn_planner: str = "lattice",
) -> list[dict[str, Any]]:
    start_xy = pose_xy(start_pose)
    length = dist(start_xy, end_xy)
    if length <= EPSILON:
        return []
    step = sample_step if sample_step > EPSILON else length
    count = max(1, int(math.ceil(length / step)))
    cutting_enabled = turn_cutting_enabled(config, direction)
    poses: list[dict[str, Any]] = []
    for i in range(1, count + 1):
        t = i / count
        poses.append(
            mark_turn_pose(
                {
                    "x": start_xy[0] + (end_xy[0] - start_xy[0]) * t,
                    "y": start_xy[1] + (end_xy[1] - start_xy[1]) * t,
                    "yaw": yaw,
                },
                direction=direction,
                turn_planner=turn_planner,
                primitive="straight",
                cutting_enabled=cutting_enabled,
            )
        )
    return poses


def sample_arc_turn_poses(
    start_pose: dict[str, Any],
    direction: str,
    curvature: float,
    length: float,
    sample_step: float,
    config: dict[str, Any],
    turn_planner: str = "lattice",
) -> list[dict[str, Any]]:
    if length <= EPSILON:
        return []
    step = sample_step if sample_step > EPSILON else length
    count = max(1, int(math.ceil(length / step)))
    ds = length / count
    sign = 1.0 if direction == "forward" else -1.0
    x, y = pose_xy(start_pose)
    yaw = float(start_pose.get("yaw", 0.0))
    cutting_enabled = turn_cutting_enabled(config, direction)
    primitive = "straight" if abs(curvature) <= EPSILON else "arc"
    poses: list[dict[str, Any]] = []
    for _ in range(count):
        signed_ds = sign * ds
        dtheta = curvature * signed_ds
        mid_yaw = yaw + dtheta / 2.0
        x += math.cos(mid_yaw) * signed_ds
        y += math.sin(mid_yaw) * signed_ds
        yaw = normalize_angle(yaw + dtheta)
        poses.append(
            mark_turn_pose(
                {"x": x, "y": y, "yaw": yaw},
                direction=direction,
                turn_planner=turn_planner,
                primitive=primitive,
                cutting_enabled=cutting_enabled,
            )
        )
    return poses


def sample_forward_u_turn_base_poses(
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    sample_step: float,
    config: dict[str, Any],
) -> list[dict[str, Any]]:
    start_yaw = float(start_pose.get("yaw", 0.0))
    end_yaw = float(end_pose.get("yaw", start_yaw + math.pi))
    p0 = (float(start_pose["x"]), float(start_pose["y"]))
    p3 = (float(end_pose["x"]), float(end_pose["y"]))
    delta = (p3[0] - p0[0], p3[1] - p0[1])
    distance = math.hypot(delta[0], delta[1])
    if distance <= EPSILON:
        return sample_pivot_base_poses(start_pose, end_yaw, config)

    start_heading = (math.cos(start_yaw), math.sin(start_yaw))
    start_left = (-start_heading[1], start_heading[0])
    longitudinal = start_heading[0] * delta[0] + start_heading[1] * delta[1]
    lateral = start_left[0] * delta[0] + start_left[1] * delta[1]
    stripe_spacing = abs(lateral)
    spacing_basis = stripe_spacing if stripe_spacing > EPSILON else distance
    extent_factor = max(0.0, float(config.get("turn_forward_extent_spacing_factor", 1.0)))
    forward_extent = max(sample_step, spacing_basis * extent_factor)
    approx_length = abs(lateral) * math.pi / 2.0 + abs(longitudinal) + 2.0 * forward_extent
    step = sample_step if sample_step > EPSILON else approx_length
    count = max(2, int(math.ceil(approx_length / step)))

    poses: list[dict[str, Any]] = []
    for i in range(1, count + 1):
        t = i / count
        theta = math.pi * t
        smooth = 0.5 - 0.5 * math.cos(theta)
        smooth_derivative = 0.5 * math.pi * math.sin(theta)
        local_x = longitudinal * smooth + forward_extent * math.sin(theta)
        local_y = lateral * smooth
        local_dx = longitudinal * smooth_derivative + forward_extent * math.pi * math.cos(theta)
        local_dy = lateral * smooth_derivative
        x = p0[0] + start_heading[0] * local_x + start_left[0] * local_y
        y = p0[1] + start_heading[1] * local_x + start_left[1] * local_y
        dx = start_heading[0] * local_dx + start_left[0] * local_dy
        dy = start_heading[1] * local_dx + start_left[1] * local_dy
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > EPSILON else start_yaw + angle_delta(start_yaw, end_yaw) * t
        if i == count:
            x, y, yaw = p3[0], p3[1], end_yaw
        poses.append(
            {
                "x": x,
                "y": y,
                "yaw": normalize_angle(yaw),
                "section": "turn",
                "direction": "forward",
                "turn_leg": 1,
                "turn_planner": "forward_u_turn",
                "cutting_enabled": turn_cutting_enabled(config, "forward"),
                "stripe_spacing_m": spacing_basis,
                "turn_forward_extent_m": forward_extent,
            }
        )
    return poses


def wheel_anchor_maneuver_id(
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    context: dict[str, Any] | None,
) -> str:
    if context:
        area = context.get("area_index", "area")
        start_swath = context.get("from_swath_index", "from")
        end_swath = context.get("to_swath_index", "to")
        return f"area-{area}-swath-{start_swath}-to-{end_swath}"
    return (
        "wheel-anchor-"
        f"{float(start_pose['x']):.2f}-{float(start_pose['y']):.2f}-"
        f"{float(end_pose['x']):.2f}-{float(end_pose['y']):.2f}"
    )


def turn_fallback_planners(config: dict[str, Any]) -> list[str]:
    raw = config.get("turn_fallback_planners", ["forward_u_turn"])
    if isinstance(raw, str):
        return [raw.lower()]
    if not isinstance(raw, list):
        die("turn_fallback_planners must be a list or string")
    return [str(item).lower() for item in raw]


def turn_is_footprint_safe(lawn: Lawn, poses: list[dict[str, Any]], config: dict[str, Any]) -> bool:
    physical = [[float(p[0]), float(p[1])] for p in (config.get("footprint") or [])]
    return (
        footprint_unsafe_sample_count_for_lawn(lawn, poses, physical) == 0
        and turn_unsafe_sample_count(lawn, poses, config) == 0
    )


def outward_footprint_extent(
    start_pose: dict[str, Any],
    poses: list[dict[str, Any]],
    footprint: list[list[float]],
    outward_sign: float,
) -> float:
    extent = -float("inf")
    for pose in poses:
        for corner in transform_footprint(pose, footprint):
            local_corner = world_to_local_xy(start_pose, corner)
            extent = max(extent, outward_sign * local_corner[1])
    return extent


def sample_wheel_anchor_pivot_poses(
    *,
    anchor_point: tuple[float, float],
    local_wheel_point: tuple[float, float],
    start_yaw: float,
    end_yaw: float,
    config: dict[str, Any],
    direction: str,
    phase: str,
    maneuver_id: str,
    turn_leg: int,
    pivot_wheel: str | None,
    target_anchor_point: tuple[float, float] | None,
    target_wheel_error_m: float,
    min_clearance_m: float,
    reverse_distance_m: float,
    pivot_angle_deg: float,
    forced_final_pose: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    delta = angle_delta(start_yaw, end_yaw)
    if abs(delta) <= math.radians(0.1):
        return []
    yaw_step = math.radians(
        max(1.0, float(config.get("turn_anchor_angle_sample_degrees", config.get("turn_anchor_angle_step_degrees", 2.5))))
    )
    count = max(1, int(math.ceil(abs(delta) / yaw_step)))
    poses: list[dict[str, Any]] = []
    for i in range(1, count + 1):
        yaw = normalize_angle(start_yaw + delta * i / count)
        pose = pose_with_wheel_at(anchor_point, local_wheel_point, yaw)
        if forced_final_pose is not None and i == count:
            pose = {
                "x": float(forced_final_pose["x"]),
                "y": float(forced_final_pose["y"]),
                "yaw": normalize_angle(float(forced_final_pose.get("yaw", yaw))),
            }
        poses.append(
            wheel_anchor_pose_metadata(
                pose,
                config,
                direction=direction,
                phase=phase,
                maneuver_id=maneuver_id,
                turn_leg=turn_leg,
                pivot_wheel=pivot_wheel,
                anchor_point=anchor_point,
                target_anchor_point=target_anchor_point,
                target_wheel_error_m=target_wheel_error_m,
                min_clearance_m=min_clearance_m,
                reverse_distance_m=reverse_distance_m,
                pivot_angle_deg=pivot_angle_deg,
            )
        )
    return poses


def sample_wheel_anchor_reverse_poses(
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    sample_step: float,
    config: dict[str, Any],
    *,
    maneuver_id: str,
    turn_leg: int,
    target_anchor_point: tuple[float, float],
    target_wheel_error_m: float,
    min_clearance_m: float,
    reverse_distance_m: float,
    pivot_angle_deg: float,
) -> list[dict[str, Any]]:
    length = pose_distance(start_pose, end_pose)
    if length <= EPSILON:
        return []
    step = sample_step if sample_step > EPSILON else length
    count = max(1, int(math.ceil(length / step)))
    poses: list[dict[str, Any]] = []
    yaw = normalize_angle(float(start_pose.get("yaw", 0.0)))
    for i in range(1, count + 1):
        t = i / count
        pose = {
            "x": float(start_pose["x"]) + (float(end_pose["x"]) - float(start_pose["x"])) * t,
            "y": float(start_pose["y"]) + (float(end_pose["y"]) - float(start_pose["y"])) * t,
            "yaw": yaw,
        }
        if i == count:
            pose["x"] = float(end_pose["x"])
            pose["y"] = float(end_pose["y"])
        poses.append(
            wheel_anchor_pose_metadata(
                pose,
                config,
                direction="backward",
                phase="reverse_anchor",
                maneuver_id=maneuver_id,
                turn_leg=turn_leg,
                pivot_wheel=None,
                anchor_point=None,
                target_anchor_point=target_anchor_point,
                target_wheel_error_m=target_wheel_error_m,
                min_clearance_m=min_clearance_m,
                reverse_distance_m=reverse_distance_m,
                pivot_angle_deg=pivot_angle_deg,
            )
        )
    return poses


def wheel_anchor_candidate_geometry(
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    config: dict[str, Any],
    *,
    inside_side: str,
    outside_side: str,
    relative_yaw: float,
) -> dict[str, Any]:
    start_wheels = wheel_points_for_pose(start_pose, config)
    end_wheels = wheel_points_for_pose(end_pose, config)
    inside_anchor = (start_wheels[inside_side]["x"], start_wheels[inside_side]["y"])
    outside_target = (end_wheels[outside_side]["x"], end_wheels[outside_side]["y"])
    inside_local = wheel_local_point(config, inside_side)
    outside_local = wheel_local_point(config, outside_side)
    mid_yaw = normalize_angle(float(start_pose.get("yaw", 0.0)) + relative_yaw)
    after_pivot = pose_with_wheel_at(inside_anchor, inside_local, mid_yaw)
    outside_after_pivot = local_to_world_xy(after_pivot, outside_local)
    heading = (math.cos(mid_yaw), math.sin(mid_yaw))
    target_delta = (
        outside_target[0] - outside_after_pivot[0],
        outside_target[1] - outside_after_pivot[1],
    )
    reverse_distance = -(target_delta[0] * heading[0] + target_delta[1] * heading[1])
    closest = (
        outside_after_pivot[0] - heading[0] * reverse_distance,
        outside_after_pivot[1] - heading[1] * reverse_distance,
    )
    target_error = dist(closest, outside_target)
    reverse_end = {
        "x": float(after_pivot["x"]) - heading[0] * reverse_distance,
        "y": float(after_pivot["y"]) - heading[1] * reverse_distance,
        "yaw": mid_yaw,
    }
    snapped_reverse_end = pose_with_wheel_at(outside_target, outside_local, mid_yaw)
    return {
        "inside_anchor": inside_anchor,
        "outside_target": outside_target,
        "inside_local": inside_local,
        "outside_local": outside_local,
        "mid_yaw": mid_yaw,
        "relative_yaw": relative_yaw,
        "after_pivot": after_pivot,
        "reverse_end": reverse_end,
        "snapped_reverse_end": snapped_reverse_end,
        "reverse_distance": reverse_distance,
        "target_error": target_error,
    }


def refine_wheel_anchor_relative_yaw(
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    config: dict[str, Any],
    *,
    inside_side: str,
    outside_side: str,
    relative_yaw: float,
    step_rad: float,
    min_rad: float,
    max_rad: float,
) -> float:
    sign = 1.0 if relative_yaw >= 0.0 else -1.0
    lo = max(min_rad, abs(relative_yaw) - step_rad)
    hi = min(max_rad, abs(relative_yaw) + step_rad)
    if hi <= lo + EPSILON:
        return relative_yaw

    def error_for(abs_value: float) -> float:
        geometry = wheel_anchor_candidate_geometry(
            start_pose,
            end_pose,
            config,
            inside_side=inside_side,
            outside_side=outside_side,
            relative_yaw=sign * abs_value,
        )
        return float(geometry["target_error"])

    for _ in range(18):
        left = lo + (hi - lo) / 3.0
        right = hi - (hi - lo) / 3.0
        if error_for(left) <= error_for(right):
            hi = right
        else:
            lo = left
    return sign * ((lo + hi) / 2.0)


def best_safe_wheel_anchor_turn(
    lawn: Lawn,
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    sample_step: float,
    config: dict[str, Any],
    context: dict[str, Any] | None = None,
) -> tuple[list[dict[str, Any]], str]:
    start_yaw = normalize_angle(float(start_pose.get("yaw", 0.0)))
    end_yaw = normalize_angle(float(end_pose.get("yaw", start_yaw + math.pi)))
    start_heading = (math.cos(start_yaw), math.sin(start_yaw))
    start_left = (-start_heading[1], start_heading[0])
    delta_xy = (float(end_pose["x"]) - float(start_pose["x"]), float(end_pose["y"]) - float(start_pose["y"]))
    lateral = start_left[0] * delta_xy[0] + start_left[1] * delta_xy[1]
    spacing = abs(lateral) if abs(lateral) > EPSILON else max(sample_step, math.hypot(delta_xy[0], delta_xy[1]))
    inside_side = "left" if lateral >= 0.0 else "right"
    outside_side = "right" if inside_side == "left" else "left"
    min_rad = math.radians(max(0.0, float(config.get("turn_anchor_pivot_min_degrees", 90.0))))
    max_rad = math.radians(max(math.degrees(min_rad), float(config.get("turn_anchor_pivot_max_degrees", 180.0))))
    step_rad = math.radians(max(0.25, float(config.get("turn_anchor_angle_step_degrees", 2.5))))
    target_tolerance = max(0.0, float(config.get("turn_anchor_target_tolerance_m", 0.03)))
    refine_tolerance = max(0.0, float(config.get("turn_anchor_refine_tolerance_m", 0.01)))
    max_reverse = spacing * max(0.0, float(config.get("turn_anchor_reverse_max_spacing_factor", 4.0)))
    envelope_tol = max(0.0, float(config.get("turn_anchor_envelope_tolerance_m", 0.03)))
    maneuver_id = wheel_anchor_maneuver_id(start_pose, end_pose, context)
    physical_footprint = [[float(p[0]), float(p[1])] for p in (config.get("footprint") or [])]
    outward_sign = 1.0 if lateral >= 0.0 else -1.0
    candidates: list[tuple[tuple[float, float, float, float, float], list[dict[str, Any]]]] = []
    rejections: dict[str, int] = {}

    def reject(reason: str) -> None:
        rejections[reason] = rejections.get(reason, 0) + 1

    angle_values: list[float] = []
    count = max(1, int(math.ceil((max_rad - min_rad) / step_rad)))
    for i in range(count + 1):
        angle_values.append(min(max_rad, min_rad + i * step_rad))

    tested: set[int] = set()
    for sign in (-1.0, 1.0):
        for angle in angle_values:
            refined = refine_wheel_anchor_relative_yaw(
                start_pose,
                end_pose,
                config,
                inside_side=inside_side,
                outside_side=outside_side,
                relative_yaw=sign * angle,
                step_rad=step_rad,
                min_rad=min_rad,
                max_rad=max_rad,
            )
            key = int(round(refined / math.radians(0.05)))
            if key in tested:
                continue
            tested.add(key)
            geometry = wheel_anchor_candidate_geometry(
                start_pose,
                end_pose,
                config,
                inside_side=inside_side,
                outside_side=outside_side,
                relative_yaw=refined,
            )
            reverse_distance = float(geometry["reverse_distance"])
            target_error = float(geometry["target_error"])
            if reverse_distance <= EPSILON:
                reject("negative_reverse_distance")
                continue
            if reverse_distance > max_reverse + EPSILON:
                reject("reverse_distance_limit")
                continue
            if target_error > target_tolerance + EPSILON:
                reject("target_error")
                continue

            pivot_angle_deg = abs(math.degrees(refined))
            min_clearance_placeholder = 0.0
            phase1 = sample_wheel_anchor_pivot_poses(
                anchor_point=geometry["inside_anchor"],
                local_wheel_point=geometry["inside_local"],
                start_yaw=start_yaw,
                end_yaw=float(geometry["mid_yaw"]),
                config=config,
                direction="rotate",
                phase="initial_pivot",
                maneuver_id=maneuver_id,
                turn_leg=1,
                pivot_wheel=inside_side,
                target_anchor_point=geometry["outside_target"],
                target_wheel_error_m=target_error,
                min_clearance_m=min_clearance_placeholder,
                reverse_distance_m=reverse_distance,
                pivot_angle_deg=pivot_angle_deg,
            )
            if not phase1:
                reject("empty_initial_pivot")
                continue

            reverse_start = phase1[-1]
            reverse_end = geometry["snapped_reverse_end"]
            phase2 = sample_wheel_anchor_reverse_poses(
                reverse_start,
                reverse_end,
                sample_step,
                config,
                maneuver_id=maneuver_id,
                turn_leg=2,
                target_anchor_point=geometry["outside_target"],
                target_wheel_error_m=target_error,
                min_clearance_m=min_clearance_placeholder,
                reverse_distance_m=reverse_distance,
                pivot_angle_deg=pivot_angle_deg,
            )
            if not phase2:
                reject("empty_reverse")
                continue

            phase3 = sample_wheel_anchor_pivot_poses(
                anchor_point=geometry["outside_target"],
                local_wheel_point=geometry["outside_local"],
                start_yaw=float(geometry["mid_yaw"]),
                end_yaw=end_yaw,
                config=config,
                direction="rotate",
                phase="final_straighten",
                maneuver_id=maneuver_id,
                turn_leg=3,
                pivot_wheel=outside_side,
                target_anchor_point=geometry["outside_target"],
                target_wheel_error_m=target_error,
                min_clearance_m=min_clearance_placeholder,
                reverse_distance_m=reverse_distance,
                pivot_angle_deg=pivot_angle_deg,
                forced_final_pose=end_pose,
            )
            if not phase3:
                phase3 = [
                    wheel_anchor_pose_metadata(
                        {
                            "x": float(end_pose["x"]),
                            "y": float(end_pose["y"]),
                            "yaw": end_yaw,
                        },
                        config,
                        direction="rotate",
                        phase="final_straighten",
                        maneuver_id=maneuver_id,
                        turn_leg=3,
                        pivot_wheel=outside_side,
                        anchor_point=geometry["outside_target"],
                        target_anchor_point=geometry["outside_target"],
                        target_wheel_error_m=target_error,
                        min_clearance_m=min_clearance_placeholder,
                        reverse_distance_m=reverse_distance,
                        pivot_angle_deg=pivot_angle_deg,
                    )
                ]

            poses = phase1 + phase2 + phase3
            if not same_pose(poses[-1], end_pose):
                reject("final_pose_mismatch")
                continue
            if not turn_is_footprint_safe(lawn, poses, config):
                reject("unsafe_footprint")
                continue
            initial_extent = outward_footprint_extent(start_pose, [start_pose] + phase1, physical_footprint, outward_sign)
            later_extent = outward_footprint_extent(start_pose, phase2 + phase3, physical_footprint, outward_sign)
            if later_extent > initial_extent + envelope_tol:
                reject("outward_envelope")
                continue

            min_clearance = turn_min_clearance_m(lawn, poses, config)
            for pose in poses:
                pose["min_clearance_m"] = min_clearance
                pose["target_wheel_error_m"] = target_error
                pose["reverse_distance_m"] = reverse_distance
                pose["pivot_angle_deg"] = pivot_angle_deg
            score = (
                -min_clearance,
                pose_list_length([start_pose] + poses),
                reverse_distance,
                max(0.0, pivot_angle_deg - math.degrees(min_rad)),
                target_error,
            )
            candidates.append((score, poses))

    if not candidates:
        reason = ", ".join(f"{key}={value}" for key, value in sorted(rejections.items())) or "no candidate samples"
        return [], (
            f"no footprint-safe wheel-anchor turn from {inside_side} pivot to {outside_side} target "
            f"(spacing {spacing:.2f} m, wheel track {wheel_track_m(config):.2f} m; {reason})"
        )

    candidates.sort(key=lambda item: item[0])
    return candidates[0][1], ""


def lattice_local_xy(start_pose: dict[str, Any], pose: dict[str, Any]) -> tuple[float, float]:
    start_yaw = float(start_pose.get("yaw", 0.0))
    dx = float(pose["x"]) - float(start_pose["x"])
    dy = float(pose["y"]) - float(start_pose["y"])
    c, s = math.cos(start_yaw), math.sin(start_yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def lattice_key(start_pose: dict[str, Any], pose: dict[str, Any], xy_res: float, yaw_res: float) -> tuple[int, int, int]:
    lx, ly = lattice_local_xy(start_pose, pose)
    yaw = angle_delta(float(start_pose.get("yaw", 0.0)), float(pose.get("yaw", 0.0)))
    return (
        int(round(lx / xy_res)),
        int(round(ly / xy_res)),
        int(round(yaw / yaw_res)),
    )


def lattice_in_bounds(
    start_pose: dict[str, Any],
    pose: dict[str, Any],
    end_pose: dict[str, Any],
    margin: float,
) -> bool:
    x, y = lattice_local_xy(start_pose, pose)
    gx, gy = lattice_local_xy(start_pose, end_pose)
    return (
        min(0.0, gx) - margin <= x <= max(0.0, gx) + margin
        and min(0.0, gy) - margin <= y <= max(0.0, gy) + margin
    )


def terminal_lattice_connection(
    lawn: Lawn,
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    sample_step: float,
    config: dict[str, Any],
) -> list[dict[str, Any]] | None:
    start_xy = pose_xy(start_pose)
    end_xy = pose_xy(end_pose)
    dx, dy = end_xy[0] - start_xy[0], end_xy[1] - start_xy[1]
    distance = math.hypot(dx, dy)
    directions = ["forward"]
    if bool(config.get("turn_reverse_enabled", True)):
        directions.append("backward")
    candidates: list[tuple[float, list[dict[str, Any]]]] = []

    if distance <= max(EPSILON, sample_step * 0.25):
        poses = sample_pivot_turn_poses(start_pose, float(end_pose.get("yaw", 0.0)), sample_step, config)
        if poses and turn_unsafe_sample_count(lawn, poses, config) == 0:
            candidates.append((turn_motion_cost(start_pose, poses, config), poses))
    else:
        direct_heading = math.atan2(dy, dx)
        max_terminal_distance = max(
            sample_step,
            float(config.get("turn_lattice_terminal_max_distance_m", 1.8)),
        )
        if distance <= max_terminal_distance:
            for direction in directions:
                travel_yaw = direct_heading if direction == "forward" else normalize_angle(direct_heading + math.pi)
                poses: list[dict[str, Any]] = []
                current = start_pose
                poses.extend(sample_pivot_turn_poses(current, travel_yaw, sample_step, config))
                if poses:
                    current = poses[-1]
                poses.extend(sample_straight_turn_poses(current, end_xy, travel_yaw, direction, sample_step, config))
                if poses:
                    current = poses[-1]
                poses.extend(sample_pivot_turn_poses(current, float(end_pose.get("yaw", travel_yaw)), sample_step, config))
                if poses and turn_unsafe_sample_count(lawn, poses, config) == 0:
                    candidates.append((turn_motion_cost(start_pose, poses, config), poses))

    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0])
    return annotate_turn_legs(candidates[0][1])


def best_safe_lattice_turn(
    lawn: Lawn,
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    sample_step: float,
    config: dict[str, Any],
) -> tuple[list[dict[str, Any]], int]:
    xy_res = max(sample_step, float(config.get("turn_lattice_xy_resolution_m", sample_step)))
    yaw_res = math.radians(max(5.0, float(config.get("turn_lattice_yaw_step_degrees", 15.0))))
    primitive_step = max(sample_step, float(config.get("turn_lattice_step_m", max(0.16, sample_step))))
    max_nodes = max(1, int(config.get("turn_lattice_max_nodes", 6000)))
    search_margin = max(0.75, float(config.get("turn_lattice_search_margin_m", 1.2)))
    pivot_step = math.radians(max(5.0, float(config.get("turn_lattice_pivot_step_degrees", 15.0))))
    reverse_enabled = bool(config.get("turn_reverse_enabled", True))
    min_radius = max(0.12, float(config.get("turn_lattice_min_radius_m", 0.28)))
    radii = [min_radius, min_radius * 1.75, min_radius * 2.75]
    curvatures = [0.0]
    for radius in radii:
        curvatures.extend([1.0 / radius, -1.0 / radius])
    directions = ["forward"] + (["backward"] if reverse_enabled else [])
    pivot_enabled = bool(config.get("turn_lattice_allow_pivots", True))
    pivot_cost = float(config.get("turn_lattice_pivot_cost_m_per_rad", 0.18))

    best_path = terminal_lattice_connection(lawn, start_pose, end_pose, sample_step, config)
    best_cost = turn_motion_cost(start_pose, best_path, config) if best_path else float("inf")

    def heuristic(pose: dict[str, Any]) -> float:
        return (
            pose_distance(pose, end_pose)
            + abs(angle_delta(float(pose.get("yaw", 0.0)), float(end_pose.get("yaw", 0.0)))) * pivot_cost
        )

    queue: list[tuple[float, int, float, dict[str, Any], list[dict[str, Any]]]] = []
    counter = 0
    start_key = lattice_key(start_pose, start_pose, xy_res, yaw_res)
    best_cost_by_key: dict[tuple[int, int, int], float] = {start_key: 0.0}
    heapq.heappush(queue, (heuristic(start_pose), counter, 0.0, dict(start_pose), []))
    nodes_expanded = 0

    while queue and nodes_expanded < max_nodes:
        priority, _, cost_so_far, pose, path_so_far = heapq.heappop(queue)
        key = lattice_key(start_pose, pose, xy_res, yaw_res)
        if cost_so_far > best_cost_by_key.get(key, float("inf")) + EPSILON:
            continue
        if priority > best_cost + EPSILON:
            break
        nodes_expanded += 1

        terminal = terminal_lattice_connection(lawn, pose, end_pose, sample_step, config)
        if terminal:
            candidate = path_so_far + terminal
            candidate_cost = cost_so_far + turn_motion_cost(pose, terminal, config)
            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_path = candidate

        primitives: list[list[dict[str, Any]]] = []
        if pivot_enabled:
            primitives.append(sample_pivot_turn_poses(pose, normalize_angle(float(pose.get("yaw", 0.0)) + pivot_step), sample_step, config))
            primitives.append(sample_pivot_turn_poses(pose, normalize_angle(float(pose.get("yaw", 0.0)) - pivot_step), sample_step, config))
        for direction in directions:
            for curvature in curvatures:
                primitives.append(sample_arc_turn_poses(pose, direction, curvature, primitive_step, sample_step, config))

        for primitive in primitives:
            if not primitive:
                continue
            next_pose = primitive[-1]
            if not lattice_in_bounds(start_pose, next_pose, end_pose, search_margin):
                continue
            if turn_unsafe_sample_count(lawn, primitive, config):
                continue
            next_key = lattice_key(start_pose, next_pose, xy_res, yaw_res)
            primitive_cost = turn_motion_cost(pose, primitive, config)
            next_cost = cost_so_far + primitive_cost
            if next_cost + EPSILON >= best_cost_by_key.get(next_key, float("inf")):
                continue
            best_cost_by_key[next_key] = next_cost
            counter += 1
            heapq.heappush(
                queue,
                (next_cost + heuristic(next_pose), counter, next_cost, next_pose, path_so_far + primitive),
            )

    if not best_path:
        return [], nodes_expanded
    annotated = annotate_turn_legs(best_path)
    for pose in annotated:
        pose["turn_lattice_nodes_expanded"] = nodes_expanded
    return annotated, nodes_expanded


def plan_swath_turn_base_poses(
    lawn: Lawn,
    start_pose: dict[str, Any],
    end_pose: dict[str, Any],
    sample_step: float,
    config: dict[str, Any],
    context: dict[str, Any] | None = None,
) -> tuple[list[dict[str, Any]], str | None]:
    planner = str(config.get("turn_planner", "wheel_anchor")).lower()
    if planner not in {"wheel_anchor", "lattice", "three_point", "forward_u_turn", "u_turn", "sampled_u_turn"}:
        die(f"unsupported turn_planner: {planner}")

    notes: list[str] = []
    if planner == "wheel_anchor":
        poses, note = best_safe_wheel_anchor_turn(lawn, start_pose, end_pose, sample_step, config, context)
        if poses:
            return poses, None
        notes.append(note)
    elif planner in {"lattice", "three_point"}:
        poses, attempted = best_safe_lattice_turn(lawn, start_pose, end_pose, sample_step, config)
        if poses:
            return poses, None
        notes.append(f"no footprint-safe lattice maneuver after expanding {attempted} nodes")
    else:
        notes.append(f"{planner} turn planner selected")

    if planner in {"forward_u_turn", "u_turn", "sampled_u_turn"} or "forward_u_turn" in turn_fallback_planners(config):
        fallback = sample_forward_u_turn_base_poses(start_pose, end_pose, sample_step, config)
        if fallback and same_pose(fallback[-1], end_pose) and turn_is_footprint_safe(lawn, fallback, config):
            for pose in fallback:
                pose["turn_fallback_reason"] = "; ".join(note for note in notes if note)
            return fallback, f"{'; '.join(note for note in notes if note)}; using footprint-safe forward U-turn fallback"
        notes.append("no footprint-safe forward U-turn fallback")

    return [], "; ".join(note for note in notes if note) or "no footprint-safe turn"


def build_headland_paths(
    lawn: Lawn,
    area_index: int,
    headland_specs: list[dict[str, Any]],
    config: dict[str, Any],
) -> list[dict[str, Any]]:
    offset = tool_center_offset(config)
    sample_step = float(config["evaluation"].get("path_sample_step_m", 0.1))
    paths = []
    for spec in headland_specs:
        ring = [(p["x"], p["y"]) for p in spec["points"]]
        tool_poses = sample_polyline_tool_poses(ring, "headland", sample_step)
        base_poses = poses_to_base_link(tool_poses, offset)
        paths.append(
            make_path_record(
                is_outline=True,
                area_index=area_index,
                lawn=lawn,
                label=f"{lawn.area.name} headland {spec['layer'] + 1}.{spec['ring'] + 1}",
                frame_id=config.get("frame_id", "map"),
                base_poses=base_poses,
                tool_poses=tool_poses,
            )
        )
    return paths


def _trim_pair_meters_from_tail(
    base_poses: list[dict[str, Any]],
    tool_poses: list[dict[str, Any]],
    trim_m: float,
    floor_idx: int,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Drop synchronised trailing poses from base/tool until ~trim_m of arc
    length has been removed, but never below ``floor_idx``."""
    if trim_m <= 0 or len(base_poses) <= floor_idx + 1:
        return list(base_poses), list(tool_poses)
    nb = list(base_poses)
    nt = list(tool_poses)
    removed = 0.0
    while len(nb) > floor_idx + 1 and removed < trim_m:
        last = nb[-1]
        prev = nb[-2]
        d = math.hypot(float(last["x"]) - float(prev["x"]),
                       float(last["y"]) - float(prev["y"]))
        if d <= 0:
            nb.pop()
            if nt:
                nt.pop()
            continue
        if removed + d > trim_m + 1e-9:
            break
        removed += d
        nb.pop()
        if nt:
            nt.pop()
    return nb, nt


def _trim_pair_meters_from_head(
    base_poses: list[dict[str, Any]],
    tool_poses: list[dict[str, Any]],
    trim_m: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if trim_m <= 0 or len(base_poses) < 2:
        return list(base_poses), list(tool_poses)
    nb = list(base_poses)
    nt = list(tool_poses)
    removed = 0.0
    while len(nb) > 1 and removed < trim_m:
        first = nb[0]
        nxt = nb[1]
        d = math.hypot(float(first["x"]) - float(nxt["x"]),
                       float(first["y"]) - float(nxt["y"]))
        if d <= 0:
            nb.pop(0)
            if nt:
                nt.pop(0)
            continue
        if removed + d > trim_m + 1e-9:
            break
        removed += d
        nb.pop(0)
        if nt:
            nt.pop(0)
    return nb, nt


def build_zero_turn_fill_paths(
    lawn: Lawn,
    area_index: int,
    swaths_json: list[dict[str, Any]],
    config: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[str], dict[str, Any]]:
    offset = tool_center_offset(config)
    sample_step = float(config["evaluation"].get("path_sample_step_m", 0.1))
    trim_max_m = float(config.get("turn_trim_max_m", 1.5))
    trim_step_m = float(config.get("turn_trim_step_m", 0.1))
    paths: list[dict[str, Any]] = []
    warnings: list[str] = []
    base_poses: list[dict[str, Any]] = []
    tool_poses: list[dict[str, Any]] = []
    chunk_index = 1
    last_stripe_start_idx = 0
    stats: dict[str, Any] = {
        "trim_attempts": 0,
        "trim_successes": 0,
        "splits": 0,
        "trim_total_distance_m": 0.0,
    }

    def finish_current() -> None:
        nonlocal base_poses, tool_poses, chunk_index, last_stripe_start_idx
        if not base_poses:
            return
        suffix = "" if chunk_index == 1 and not paths else f" {chunk_index}"
        paths.append(
            make_path_record(
                is_outline=False,
                area_index=area_index,
                lawn=lawn,
                label=f"{lawn.area.name} fill{suffix}",
                frame_id=config.get("frame_id", "map"),
                base_poses=base_poses,
                tool_poses=tool_poses,
            )
        )
        base_poses = []
        tool_poses = []
        last_stripe_start_idx = 0
        chunk_index += 1

    previous_swath_index: int | None = None
    for swath in swaths_json:
        swath_index = int(swath.get("index", len(paths)))
        swath_points = [(p["x"], p["y"]) for p in swath.get("points", [])]
        swath_tool_poses = sample_polyline_tool_poses(swath_points, "swath", sample_step)
        if not swath_tool_poses:
            continue
        swath_base_poses = poses_to_base_link(swath_tool_poses, offset)

        if not base_poses:
            last_stripe_start_idx = 0
            for base_pose, tool_pose in zip(swath_base_poses, swath_tool_poses):
                append_pose_pair(base_poses, tool_poses, base_pose, tool_pose)
            previous_swath_index = swath_index
            continue

        turn_context = {
            "area_index": area_index,
            "from_swath_index": previous_swath_index,
            "to_swath_index": swath_index,
        }
        turn_poses, turn_warning = plan_swath_turn_base_poses(
            lawn,
            base_poses[-1],
            swath_base_poses[0],
            sample_step,
            config,
            turn_context,
        )

        # Stripe-end trimming retry. If the turn failed and the failure is the
        # kind a shorter stripe could resolve (boundary-hit footprint, target
        # outside the cell), retry with progressively trimmed stripe ends.
        # Trim symmetrically: current stripe's tail and next stripe's head by
        # the same distance, so the U-turn happens further from the boundary.
        # Never trim past the current stripe's start within base_poses.
        next_swath_base = swath_base_poses
        next_swath_tool = swath_tool_poses
        trimmed_base_poses: list[dict[str, Any]] | None = None
        trimmed_tool_poses: list[dict[str, Any]] | None = None
        trim_applied = 0.0
        if not turn_poses and trim_max_m > 0:
            trim = trim_step_m
            while trim <= trim_max_m + 1e-9:
                stats["trim_attempts"] += 1
                cand_base, cand_tool = _trim_pair_meters_from_tail(
                    base_poses, tool_poses, trim, floor_idx=last_stripe_start_idx
                )
                cand_next_base, cand_next_tool = _trim_pair_meters_from_head(
                    swath_base_poses, swath_tool_poses, trim
                )
                if (
                    len(cand_base) <= last_stripe_start_idx + 1
                    or len(cand_next_base) < 2
                ):
                    break
                retry_turn, _retry_warn = plan_swath_turn_base_poses(
                    lawn,
                    cand_base[-1],
                    cand_next_base[0],
                    sample_step,
                    config,
                    turn_context,
                )
                if retry_turn:
                    trimmed_base_poses = cand_base
                    trimmed_tool_poses = cand_tool
                    next_swath_base = cand_next_base
                    next_swath_tool = cand_next_tool
                    turn_poses = retry_turn
                    trim_applied = trim
                    stats["trim_successes"] += 1
                    stats["trim_total_distance_m"] += trim
                    break
                trim += trim_step_m

        if trim_applied > 0 and trimmed_base_poses is not None:
            base_poses = trimmed_base_poses
            tool_poses = trimmed_tool_poses if trimmed_tool_poses is not None else tool_poses
            warnings.append(
                f"swath {previous_swath_index} to {swath_index}: "
                f"trimmed {trim_applied:.2f} m from stripe ends to fit U-turn"
            )
        elif turn_warning and not turn_poses:
            warnings.append(
                f"swath {previous_swath_index} to {swath_index}: {turn_warning}"
            )

        if not turn_poses:
            stats["splits"] += 1
            finish_current()
            for base_pose, tool_pose in zip(swath_base_poses, swath_tool_poses):
                append_pose_pair(base_poses, tool_poses, base_pose, tool_pose)
            previous_swath_index = swath_index
            continue

        for base_pose in turn_poses:
            append_pose_pair(base_poses, tool_poses, base_pose, tool_pose_from_base_pose(base_pose, offset))

        last_stripe_start_idx = len(base_poses)
        for base_pose, tool_pose in zip(next_swath_base, next_swath_tool):
            append_pose_pair(base_poses, tool_poses, base_pose, tool_pose)
        previous_swath_index = swath_index

    finish_current()
    return paths, warnings, stats


def robot_for_config(config: dict[str, Any], f2c: Any) -> Any:
    tool_width = float(config["tool_width"])
    footprint = config.get("footprint") or []
    robot_width = max((float(p[1]) for p in footprint), default=tool_width / 2.0) - min(
        (float(p[1]) for p in footprint), default=-tool_width / 2.0
    )
    robot = f2c.Robot(max(robot_width, tool_width), tool_width)
    robot.setMinTurningRadius(float(config["fields2cover"].get("min_turning_radius", 0.1)))
    robot.setMaxDiffCurv(float(config["fields2cover"].get("max_diff_curv", 0.1)))
    return robot


def build_f2c_fill_path(
    lawn: Lawn,
    area_index: int,
    swaths: Any,
    config: dict[str, Any],
    f2c: Any,
) -> dict[str, Any] | None:
    robot = robot_for_config(config, f2c)
    path = f2c.PP_PathPlanning().planPath(robot, swaths, planner_turn(config, f2c))
    sample_step = float(config["evaluation"].get("path_sample_step_m", 0.1))
    if sample_step > 0:
        path.discretize(sample_step)
    tool_poses = f2c_path_to_poses(path, f2c)
    if not tool_poses:
        return None
    base_poses = poses_to_base_link(tool_poses, tool_center_offset(config))
    return make_path_record(
        is_outline=False,
        area_index=area_index,
        lawn=lawn,
        label=f"{lawn.area.name} fill",
        frame_id=config.get("frame_id", "map"),
        base_poses=base_poses,
        tool_poses=tool_poses,
    )


def build_transit_path(
    *,
    lawn: Lawn,
    area_index: int,
    label: str,
    headland_outer_ring: list[tuple[float, float]],
    from_pose: dict[str, Any],
    to_pose: dict[str, Any],
    config: dict[str, Any],
) -> dict[str, Any] | None:
    """Build a path that walks along the headland centreline from ``from_pose`` to ``to_pose``.

    Used by P1: between two BCD cells, connect them by walking the eroded-lawn
    boundary instead of attempting a direct turn. Poses are tagged
    ``section='connector'`` and ``cutting_enabled=False`` so they are excluded
    from coverage metrics.
    """
    sample_step = float(config["evaluation"].get("path_sample_step_m", 0.1))
    offset = tool_center_offset(config)
    start_xy = (float(from_pose["x"]), float(from_pose["y"]))
    end_xy = (float(to_pose["x"]), float(to_pose["y"]))
    walk = lab_geometry.walk_ring_between(headland_outer_ring, start_xy, end_xy)
    if len(walk) < 2:
        return None
    # Bracket the walk with the exact endpoint poses so wheel-anchor turn metrics
    # remain consistent.
    base_poses = sample_polyline_tool_poses(
        walk, "connector", sample_step, cutting_enabled=False
    )
    base_poses = poses_to_base_link(base_poses, offset)
    # Re-tag and force connector semantics
    for pose in base_poses:
        pose["section"] = "connector"
        pose["cutting_enabled"] = False
        pose["transit"] = True
    tool_poses = [tool_pose_from_base_pose(p, offset) for p in base_poses]
    return make_path_record(
        is_outline=False,
        area_index=area_index,
        lawn=lawn,
        label=label,
        frame_id=config.get("frame_id", "map"),
        base_poses=base_poses,
        tool_poses=tool_poses,
    )


def _reverse_cell_paths(paths: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Return a deep-copy of ``paths`` with chunk order reversed, each chunk's
    poses reversed, AND each pose's yaw rotated 180° so the mower faces the
    direction it actually moves. Used by the visit-order optimiser when
    entering a cell from its "end" side gives a cleaner inter-cell connection
    than entering from its F2C-default "start" side. Wheel-anchor maneuver
    metadata (pivot_wheel, anchor_point, phase) is left attached to its
    original sample even though execution direction has flipped — that is a
    known cosmetic limitation flagged for P3.
    """
    out: list[dict[str, Any]] = []
    for original in reversed(paths):
        clone = copy.deepcopy(original)
        for key in ("path", "tool_path"):
            poses = clone.get(key, {}).get("poses") if isinstance(clone.get(key), dict) else None
            if not poses:
                continue
            new_poses = []
            for pose in reversed(poses):
                p2 = dict(pose)
                if "yaw" in p2 and p2["yaw"] is not None:
                    try:
                        p2["yaw"] = normalize_angle(float(p2["yaw"]) + math.pi)
                    except (TypeError, ValueError):
                        pass
                new_poses.append(p2)
            clone[key]["poses"] = new_poses
        out.append(clone)
    return out


def _cell_endpoint_poses(paths: list[dict[str, Any]]) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    first_pose = None
    last_pose = None
    for chunk in paths:
        poses = chunk.get("path", {}).get("poses") or []
        if poses:
            if first_pose is None:
                first_pose = poses[0]
            last_pose = poses[-1]
    return first_pose, last_pose


def _pose_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def _greedy_visit_order_from(
    cells: list[dict[str, Any]],
    first_cell: dict[str, Any],
    first_direction: str,
) -> tuple[list[tuple[int, str, list[dict[str, Any]]]], float]:
    """Run a greedy nearest-neighbour visit starting from ``first_cell`` with
    the given ``first_direction``. Returns the visit order and the total inter-
    cell transit distance accumulated.
    """
    visited: list[tuple[int, str, list[dict[str, Any]]]] = []
    if first_direction == "reverse":
        first_paths = _reverse_cell_paths(first_cell["paths"])
        current_end = first_cell["start"]
    else:
        first_paths = first_cell["paths"]
        current_end = first_cell["end"]
    visited.append((first_cell["idx"], first_direction, first_paths))
    remaining = [c for c in cells if c["idx"] != first_cell["idx"]]
    total_cost = 0.0

    while remaining:
        best_cell = None
        best_dir = "forward"
        best_dist = math.inf
        for candidate in remaining:
            d_fwd = _pose_distance(current_end, candidate["start"])
            d_rev = _pose_distance(current_end, candidate["end"])
            if d_fwd < best_dist:
                best_dist = d_fwd
                best_cell = candidate
                best_dir = "forward"
            if d_rev < best_dist:
                best_dist = d_rev
                best_cell = candidate
                best_dir = "reverse"
        assert best_cell is not None
        total_cost += best_dist
        if best_dir == "reverse":
            paths = _reverse_cell_paths(best_cell["paths"])
            current_end = best_cell["start"]
        else:
            paths = best_cell["paths"]
            current_end = best_cell["end"]
        visited.append((best_cell["idx"], best_dir, paths))
        remaining = [c for c in remaining if c["idx"] != best_cell["idx"]]

    return visited, total_cost


def optimize_cell_visit_order(
    cell_fill_paths: list[list[dict[str, Any]]],
) -> list[tuple[int, str, list[dict[str, Any]]]]:
    """Pick a cell visit order that minimises inter-cell transit distance.

    Each cell can be visited in F2C "forward" order or pose-reversed "reverse"
    order. The strategy is multi-start greedy nearest-neighbour: try every
    (starting cell, starting direction) pair, run a greedy walk from each,
    and keep the one with the smallest total inter-cell transit distance.
    This is O(N^3) with a small constant (≤ ~14 starts × ~N^2 greedy = a few
    thousand ops for N ≤ 16), which is fast enough that we always do it.

    Greedy with a fixed start can lock the planner into a corner — picking the
    bottom-left endpoint forward dumps the exit at the opposite corner on
    asymmetric maps. Trying all (start, direction) pairs costs almost nothing
    and consistently chooses a starting cell whose F2C-chosen end lands near
    the other cells.

    Returns a list of (original_cell_index, direction, paths) in execution
    order, with ``paths`` already reversed when ``direction == 'reverse'``.
    Empty cells are dropped.
    """
    cells: list[dict[str, Any]] = []
    for idx, paths in enumerate(cell_fill_paths):
        if not paths:
            continue
        start, end = _cell_endpoint_poses(paths)
        if start is None or end is None:
            continue
        cells.append({"idx": idx, "paths": paths, "start": start, "end": end})

    if not cells:
        return []
    if len(cells) == 1:
        return [(cells[0]["idx"], "forward", cells[0]["paths"])]

    best_visited: list[tuple[int, str, list[dict[str, Any]]]] | None = None
    best_total_cost = math.inf
    for first_cell in cells:
        for first_direction in ("forward", "reverse"):
            visited, cost = _greedy_visit_order_from(cells, first_cell, first_direction)
            if cost < best_total_cost:
                best_total_cost = cost
                best_visited = visited

    assert best_visited is not None
    return best_visited


def try_direct_inter_cell_connector(
    *,
    lawn: Lawn,
    area_index: int,
    label: str,
    from_pose: dict[str, Any],
    to_pose: dict[str, Any],
    config: dict[str, Any],
    context: dict[str, Any],
) -> dict[str, Any] | None:
    """Try to bridge two consecutive cells with the existing wheel-anchor / U-turn
    planner before falling back to a transit via the headland.

    Returns a path record carrying the turn poses, or ``None`` if no direct
    turn was found (the caller should then build a transit).
    """
    sample_step = float(config["evaluation"].get("path_sample_step_m", 0.1))
    offset = tool_center_offset(config)
    turn_poses, _warning = plan_swath_turn_base_poses(
        lawn,
        from_pose,
        to_pose,
        sample_step,
        config,
        context,
    )
    if not turn_poses:
        return None
    base_poses = list(turn_poses)
    tool_poses = [tool_pose_from_base_pose(p, offset) for p in base_poses]
    return make_path_record(
        is_outline=False,
        area_index=area_index,
        lawn=lawn,
        label=label,
        frame_id=config.get("frame_id", "map"),
        base_poses=base_poses,
        tool_poses=tool_poses,
    )


def plan_one_lawn_profiles(lawn: Lawn, area_index: int, config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    try:
        import fields2cover as f2c
    except ImportError:
        die("Fields2Cover is not installed; run through tools/coverage_lab/bin/coverage_lab")

    strategy = str(config.get("headland_strategy", "footprint_disk")).lower()
    if strategy == "f2c":
        return _plan_one_lawn_profiles_f2c(lawn, area_index, config, f2c)
    return _plan_one_lawn_profiles_footprint_disk(lawn, area_index, config, f2c)


def _plan_one_lawn_profiles_footprint_disk(
    lawn: Lawn, area_index: int, config: dict[str, Any], f2c: Any
) -> dict[str, dict[str, Any]]:
    """P0+P1 implementation: footprint-disk eroded headland and BCD cell decomposition."""
    warnings: list[str] = []
    tool_width = float(config["tool_width"])
    outline_count = int(config.get("outline_count", 0))
    outline_offset = float(config.get("outline_offset", 0.0))
    r_disk = lab_geometry.footprint_disk_radius(config)
    cell_decomposition = bool(config.get("cell_decomposition", True))

    base_debug: dict[str, Any] = {
        "area_index": area_index,
        "area_id": lawn.area.id,
        "area_name": lawn.area.name,
        "input_boundary": [{"x": x, "y": y} for x, y in lawn.area.outline],
        "input_obstacles": [
            {"id": h.id, "name": h.name, "points": [{"x": x, "y": y} for x, y in h.outline]}
            for h in lawn.holes
        ],
        "headland_strategy": "footprint_disk",
        "footprint_disk_radius_m": r_disk,
        "outline_clearance_m": r_disk,
        "headland_rings": [],
        "mainland_rings": [],
        "eroded_polygon_area_m2": 0.0,
        "mainland_polygon_area_m2": 0.0,
        "cells": [],
        "transit_count": 0,
        "transit_total_length_m": 0.0,
        "swaths": [],
        "tool_center_offset": list(tool_center_offset(config)),
    }

    hole_rings = [hole.outline for hole in lawn.holes]
    eroded_polygons = lab_geometry.erode_lawn(lawn.area.outline, hole_rings, r_disk)
    if not eroded_polygons:
        warnings.append(
            f"footprint-disk erosion (r={r_disk:.3f} m) left no drivable area in {lawn.area.name}; "
            "lawn is too narrow for the configured footprint+safety margin"
        )
        result_profiles: dict[str, dict[str, Any]] = {}
        for profile_name in [primary_profile_name(config)] + comparison_profile_names(config):
            debug = copy.deepcopy(base_debug)
            debug["profile"] = profile_name
            result_profiles[profile_name] = {
                "paths": [],
                "debug_area": debug,
                "warnings": list(warnings),
            }
        return result_profiles

    base_debug["eroded_polygon_area_m2"] = sum(p.area for p in eroded_polygons)

    # Build per-layer headland centrelines and record them in the same shape as the
    # legacy F2C path, so build_headland_paths and rendering stay unchanged.
    layer_centrelines: list[list] = []  # list[list[shapely Polygon]] per layer
    layer_centrelines.append(eroded_polygons)
    for layer_i in range(1, outline_count):
        inner = []
        for parent in eroded_polygons:
            extra = lab_geometry.erode_lawn(
                lab_geometry.polygon_outer_ring(parent),
                lab_geometry.polygon_holes(parent),
                layer_i * tool_width,
            )
            inner.extend(extra)
        layer_centrelines.append(inner)

    ring_index_counter = 0
    if outline_count > 0:
        for layer_i, polygons in enumerate(layer_centrelines):
            centerline_offset = r_disk + layer_i * tool_width
            for poly in polygons:
                for spec in lab_geometry.polygon_to_ring_dicts(poly, layer_i, ring_index_counter, centerline_offset):
                    base_debug["headland_rings"].append(spec)
                    ring_index_counter += 1

    # Mainland = drivable area after the headland passes.
    mainland_inset = max(0.0, outline_count * tool_width - tool_width / 2.0 + outline_offset)
    mainland_polygons: list = []
    if mainland_inset <= EPSILON:
        mainland_polygons = list(eroded_polygons)
    else:
        for parent in eroded_polygons:
            mainland_polygons.extend(
                lab_geometry.erode_lawn(
                    lab_geometry.polygon_outer_ring(parent),
                    lab_geometry.polygon_holes(parent),
                    mainland_inset,
                )
            )
    base_debug["mainland_polygon_area_m2"] = sum(p.area for p in mainland_polygons)
    base_debug["mainland_rings"] = [
        [{"x": x, "y": y} for x, y in lab_geometry.polygon_outer_ring(p)]
        for p in mainland_polygons
    ]

    result_profiles = {}
    profile_names = [primary_profile_name(config)] + comparison_profile_names(config)
    for profile_name in profile_names:
        debug = copy.deepcopy(base_debug)
        debug["profile"] = profile_name
        result_profiles[profile_name] = {
            "paths": build_headland_paths(lawn, area_index, debug["headland_rings"], config),
            "debug_area": debug,
            "warnings": list(warnings),
        }

    if not mainland_polygons:
        for profile_name in profile_names:
            result_profiles[profile_name]["warnings"].append(
                "footprint-disk erosion produced no mainland; lawn is too narrow for headland passes"
            )
        return result_profiles

    # Decide swath angle. If configured fixed, use it directly. Otherwise run F2C
    # swath generation once on the union mainland to pick the best angle, then
    # use that fixed angle for per-cell planning.
    fixed_angle = configured_swath_angle(config)
    if fixed_angle is None:
        try:
            probe_cells = make_f2c_cells_from_polygon(mainland_polygons[0], f2c)
            probe_swaths = generate_swaths(probe_cells, config, f2c)
            fixed_angle = swath_angle_from_swaths_json(f2c_swaths_to_json(probe_swaths))
        except Exception as exc:  # pragma: no cover - best-effort probe
            warnings.append(f"Fields2Cover swath angle probe failed: {exc}; using 0 rad")
            fixed_angle = 0.0
        if fixed_angle is None:
            fixed_angle = 0.0

    # Decompose each mainland piece into hole-free BCD cells aligned to the swath angle.
    sliver_area = max(tool_width * tool_width, 0.05)
    all_cells: list = []
    dropped_cell_area_m2 = 0.0
    dropped_cell_count = 0
    for mainland_polygon in mainland_polygons:
        if cell_decomposition:
            cells_here = lab_geometry.bcd_decompose(mainland_polygon, fixed_angle)
        else:
            cells_here = [mainland_polygon]
        for cell in cells_here:
            if cell.area < sliver_area:
                dropped_cell_area_m2 += cell.area
                dropped_cell_count += 1
                continue
            all_cells.append(cell)
    for profile_name in profile_names:
        result_profiles[profile_name]["debug_area"]["dropped_sliver_cell_count"] = dropped_cell_count
        result_profiles[profile_name]["debug_area"]["dropped_sliver_cell_area_m2"] = dropped_cell_area_m2
    cells_summary = [
        {"index": i, "area_m2": float(c.area)} for i, c in enumerate(all_cells)
    ]
    for profile_name in profile_names:
        result_profiles[profile_name]["debug_area"]["cells"] = copy.deepcopy(cells_summary)

    # Run F2C swath gen + sort + zero-turn fill per cell, with a fixed swath angle
    # so stripes are visually continuous across cells.
    fixed_config = copy.deepcopy(config)
    fixed_config.setdefault("fields2cover", {})
    fixed_config["fields2cover"]["swath_angle"] = {
        "mode": "fixed",
        "degrees": math.degrees(fixed_angle),
    }

    primary = primary_profile_name(config)
    cell_fill_paths: list[list[dict[str, Any]]] = []
    cell_swaths_json_all: list[dict[str, Any]] = []
    cell_fill_warnings: list[str] = []
    cell_trim_stats: list[dict[str, Any]] = []
    swath_index_offset = 0
    for cell_i, cell_polygon in enumerate(all_cells):
        try:
            cell_f2c = make_f2c_cells_from_polygon(cell_polygon, f2c)
            cell_swaths = generate_swaths(cell_f2c, fixed_config, f2c)
            try:
                cell_swaths = sort_swaths(cell_swaths, fixed_config, f2c)
            except TypeError as exc:
                warnings.append(
                    f"Fields2Cover swath ordering failed in cell {cell_i}; using generated order: {exc}"
                )
            cell_swaths_json = f2c_swaths_to_json(cell_swaths)
            # Re-number swath indices so per-cell warnings carry global numbering.
            for k, sw in enumerate(cell_swaths_json):
                sw["index"] = swath_index_offset + k
                sw["cell_index"] = cell_i
            swath_index_offset += len(cell_swaths_json)
            cell_swaths_json_all.extend(cell_swaths_json)
        except Exception as exc:
            warnings.append(f"Fields2Cover swath generation failed in cell {cell_i}: {exc}")
            cell_fill_paths.append([])
            continue

        if not cell_swaths_json:
            cell_fill_paths.append([])
            continue
        cell_paths, cell_warns, cell_stats = build_zero_turn_fill_paths(lawn, area_index, cell_swaths_json, config)
        cell_fill_warnings.extend(
            f"cell {cell_i}: {w}" for w in cell_warns
        )
        cell_fill_paths.append(cell_paths)
        cell_trim_stats.append(cell_stats)

    for profile_name in profile_names:
        result_profiles[profile_name]["debug_area"]["swaths"] = copy.deepcopy(cell_swaths_json_all)

    if not cell_swaths_json_all:
        for profile_name in profile_names:
            result_profiles[profile_name]["warnings"].append(
                "footprint-disk planner produced no swaths across any cell"
            )

    # Pick a single headland outer ring for transit stitching: largest eroded polygon.
    transit_ring: list[tuple[float, float]] = []
    if eroded_polygons:
        largest = max(eroded_polygons, key=lambda p: p.area)
        transit_ring = lab_geometry.polygon_outer_ring(largest)

    # Optimise cell visit order: greedy nearest-neighbour with per-cell
    # forward/reverse choice so that consecutive cells' endpoints sit close
    # together. This eliminates the long "across the property" transits the
    # previous arbitrary order produced (e.g. obstacle_map Cell 2 → Cell 3
    # used to walk the entire perimeter to reach the right-of-obstacle band).
    visit_order = optimize_cell_visit_order(cell_fill_paths)

    # Count within-cell path splits — places where build_zero_turn_fill_paths
    # called finish_current() because both wheel-anchor and the forward U-turn
    # fallback failed even after retry-with-trim. A cell with k chunks
    # contributes k-1 splits.
    split_count = sum(max(0, len(paths) - 1) for paths in cell_fill_paths)
    # Aggregate stripe-end trim retry stats across cells.
    trim_attempts = sum(int(s.get("trim_attempts", 0)) for s in cell_trim_stats)
    trim_successes = sum(int(s.get("trim_successes", 0)) for s in cell_trim_stats)
    trim_total_distance_m = sum(float(s.get("trim_total_distance_m", 0.0)) for s in cell_trim_stats)
    inter_cell_direct_turns = 0
    transit_count = 0
    transit_length = 0.0

    # Assemble primary path list: headland(s) + ordered cell fills + connector
    # (direct turn if feasible, else transit via headland) between cells.
    primary_paths = list(result_profiles[primary]["paths"])
    last_terminal: dict[str, Any] | None = None
    for slot_i, (cell_i, direction, fill_paths) in enumerate(visit_order):
        if not fill_paths:
            continue
        first_pose = fill_paths[0]["path"]["poses"][0] if fill_paths[0]["path"]["poses"] else None
        if last_terminal is not None and first_pose is not None:
            connector_label_base = f"{lawn.area.name} inter-cell {slot_i}"
            connector = try_direct_inter_cell_connector(
                lawn=lawn,
                area_index=area_index,
                label=f"{connector_label_base} turn",
                from_pose=last_terminal,
                to_pose=first_pose,
                config=config,
                context={
                    "area_index": area_index,
                    "from_cell_index": visit_order[slot_i - 1][0],
                    "to_cell_index": cell_i,
                },
            )
            if connector is not None:
                primary_paths.append(connector)
                inter_cell_direct_turns += 1
            elif transit_ring:
                transit = build_transit_path(
                    lawn=lawn,
                    area_index=area_index,
                    label=f"{lawn.area.name} transit {transit_count + 1}",
                    headland_outer_ring=transit_ring,
                    from_pose=last_terminal,
                    to_pose=first_pose,
                    config=config,
                )
                if transit is not None:
                    primary_paths.append(transit)
                    transit_count += 1
                    transit_length += pose_list_length(transit["path"]["poses"])
        primary_paths.extend(fill_paths)
        last_terminal = fill_paths[-1]["path"]["poses"][-1] if fill_paths[-1]["path"]["poses"] else last_terminal

    result_profiles[primary]["paths"] = primary_paths
    result_profiles[primary]["warnings"].extend(cell_fill_warnings)
    for profile_name in profile_names:
        result_profiles[profile_name]["debug_area"]["transit_count"] = transit_count
        result_profiles[profile_name]["debug_area"]["transit_total_length_m"] = transit_length
        result_profiles[profile_name]["debug_area"]["inter_cell_direct_turns"] = inter_cell_direct_turns
        result_profiles[profile_name]["debug_area"]["within_cell_path_splits"] = split_count
        result_profiles[profile_name]["debug_area"]["stripe_trim_attempts"] = trim_attempts
        result_profiles[profile_name]["debug_area"]["stripe_trim_successes"] = trim_successes
        result_profiles[profile_name]["debug_area"]["stripe_trim_total_distance_m"] = trim_total_distance_m
        result_profiles[profile_name]["debug_area"]["visit_order"] = [
            {"slot": i, "cell_index": ci, "direction": d}
            for i, (ci, d, _) in enumerate(visit_order)
        ]

    # Comparison profile: keep using global F2C path planning on a fresh swath set
    # built from the union mainland (so the f2c_tiny_radius comparison still shows
    # F2C's built-in connector behaviour).
    for profile_name in comparison_profile_names(config):
        try:
            union_cells = make_f2c_cells_from_polygon(mainland_polygons[0], f2c)
            union_swaths = generate_swaths(union_cells, fixed_config, f2c)
            try:
                union_swaths = sort_swaths(union_swaths, fixed_config, f2c)
            except TypeError:
                pass
            if union_swaths.size() == 0:
                result_profiles[profile_name]["warnings"].append(
                    "Fields2Cover produced no fill swaths for comparison profile"
                )
                continue
            fill_path = build_f2c_fill_path(
                lawn, area_index, union_swaths, profile_config(fixed_config, profile_name), f2c
            )
            if fill_path:
                result_profiles[profile_name]["paths"].append(fill_path)
            else:
                result_profiles[profile_name]["warnings"].append(
                    "Fields2Cover path planner produced no fill path"
                )
        except Exception as exc:
            result_profiles[profile_name]["warnings"].append(f"Fields2Cover path planner failed: {exc}")

    return result_profiles


def _plan_one_lawn_profiles_f2c(
    lawn: Lawn, area_index: int, config: dict[str, Any], f2c: Any
) -> dict[str, dict[str, Any]]:
    """Legacy F2C-headland strategy, kept for headland_strategy: f2c comparison."""
    warnings: list[str] = []
    cells = make_f2c_cells(lawn, f2c)
    const_hl = f2c.HG_Const_gen()
    tool_width = float(config["tool_width"])
    outline_count = int(config.get("outline_count", 0))
    outline_offset = float(config.get("outline_offset", 0.0))
    clearance = outline_clearance(config)
    headland_width = (
        max(0.0, clearance + (outline_count - 0.5) * tool_width + outline_offset)
        if outline_count > 0
        else max(0.0, outline_offset)
    )

    base_debug: dict[str, Any] = {
        "area_index": area_index,
        "area_id": lawn.area.id,
        "area_name": lawn.area.name,
        "input_boundary": [{"x": x, "y": y} for x, y in lawn.area.outline],
        "input_obstacles": [
            {"id": h.id, "name": h.name, "points": [{"x": x, "y": y} for x, y in h.outline]}
            for h in lawn.holes
        ],
        "headland_strategy": "f2c",
        "headland_width_m": headland_width,
        "outline_clearance_m": clearance,
        "headland_rings": [],
        "mainland_rings": [],
        "swaths": [],
        "tool_center_offset": list(tool_center_offset(config)),
    }

    if outline_count > 0:
        for layer_i in range(outline_count):
            centerline_offset = clearance + layer_i * tool_width
            try:
                headland_cells = const_hl.generateHeadlands(cells, centerline_offset)
                layer_rings = f2c_cells_rings(headland_cells)
                if not layer_rings:
                    warnings.append(
                        f"Fields2Cover produced no headland ring at {centerline_offset:.3f} m inset"
                    )
                for ring_i, ring in enumerate(layer_rings):
                    base_debug["headland_rings"].append(
                        {
                            "layer": layer_i,
                            "ring": ring_i,
                            "centerline_offset_m": centerline_offset,
                            "points": [{"x": x, "y": y} for x, y in ring],
                        }
                    )
            except Exception as exc:
                warnings.append(
                    f"Fields2Cover headland ring generation failed at {centerline_offset:.3f} m inset: {exc}"
                )

    try:
        mainland = const_hl.generateHeadlands(cells, headland_width) if headland_width > 0.0 else cells
    except Exception as exc:
        warnings.append(f"Fields2Cover headland area generation failed; using full cell for fill: {exc}")
        mainland = cells

    base_debug["mainland_rings"] = [
        [{"x": x, "y": y} for x, y in ring]
        for ring in f2c_cells_rings(mainland)
    ]

    result_profiles: dict[str, dict[str, Any]] = {}
    profile_names = [primary_profile_name(config)] + comparison_profile_names(config)
    for profile_name in profile_names:
        debug = copy.deepcopy(base_debug)
        debug["profile"] = profile_name
        result_profiles[profile_name] = {
            "paths": build_headland_paths(lawn, area_index, debug["headland_rings"], config),
            "debug_area": debug,
            "warnings": list(warnings),
        }

    if mainland.size() == 0:
        for profile_name in profile_names:
            result_profiles[profile_name]["warnings"].append("Fields2Cover produced no mainland after headland offset")
        return result_profiles

    swaths = generate_swaths(mainland, config, f2c)
    swath_ordering_failed = False
    try:
        swaths = sort_swaths(swaths, config, f2c)
    except TypeError as exc:
        swath_ordering_failed = True
        warnings.append(f"Fields2Cover swath ordering failed; using generated swath order: {exc}")
    swaths_json = f2c_swaths_to_json(swaths)
    for profile_name in profile_names:
        result_profiles[profile_name]["debug_area"]["swaths"] = copy.deepcopy(swaths_json)

    if swaths.size() == 0:
        for profile_name in profile_names:
            result_profiles[profile_name]["warnings"].append("Fields2Cover produced no fill swaths")
        return result_profiles

    primary = primary_profile_name(config)
    fill_paths, fill_warnings, _fill_stats = build_zero_turn_fill_paths(lawn, area_index, swaths_json, config)
    result_profiles[primary]["warnings"].extend(fill_warnings)
    if fill_paths:
        result_profiles[primary]["paths"].extend(fill_paths)
    else:
        result_profiles[primary]["warnings"].append("zero-turn planner produced no fill path")

    for profile_name in comparison_profile_names(config):
        if swath_ordering_failed:
            result_profiles[profile_name]["warnings"].append(
                "Fields2Cover comparison path skipped because swath ordering failed for nested generated swaths"
            )
            continue
        try:
            fill_path = build_f2c_fill_path(lawn, area_index, swaths, profile_config(config, profile_name), f2c)
            if fill_path:
                result_profiles[profile_name]["paths"].append(fill_path)
            else:
                result_profiles[profile_name]["warnings"].append("Fields2Cover path planner produced no fill path")
        except Exception as exc:
            result_profiles[profile_name]["warnings"].append(f"Fields2Cover path planner failed: {exc}")

    return result_profiles


def path_poses(path: dict[str, Any], source: str = "base") -> list[dict[str, Any]]:
    if source == "tool":
        return path.get("tool_path", {}).get("poses") or path.get("path", {}).get("poses", [])
    return path.get("path", {}).get("poses", [])


def is_coverage_section(path: dict[str, Any], a: dict[str, Any], b: dict[str, Any]) -> bool:
    return pose_cutting_enabled(path, a) and pose_cutting_enabled(path, b)


def iter_pose_segments(
    paths: list[dict[str, Any]],
    coverage_only: bool = False,
    source: str = "base",
) -> Iterable[tuple[dict[str, Any], dict[str, Any], dict[str, Any]]]:
    for path in paths:
        poses = path_poses(path, source)
        for i in range(len(poses) - 1):
            a, b = poses[i], poses[i + 1]
            if coverage_only and not is_coverage_section(path, a, b):
                continue
            yield path, a, b


def point_segment_distance(
    point: tuple[float, float], a: tuple[float, float], b: tuple[float, float]
) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom <= EPSILON:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def path_length(paths: list[dict[str, Any]], predicate: Any | None = None, source: str = "base") -> float:
    total = 0.0
    for path in paths:
        poses = path_poses(path, source)
        for i in range(len(poses) - 1):
            a, b = poses[i], poses[i + 1]
            if predicate and not predicate(path, a, b):
                continue
            total += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    return total


def transform_footprint(pose: dict[str, Any], footprint: list[list[float]]) -> list[tuple[float, float]]:
    yaw = float(pose.get("yaw", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    result = []
    for x, y in footprint:
        result.append((pose["x"] + c * x - s * y, pose["y"] + s * x + c * y))
    return result


def safety_footprint(config: dict[str, Any]) -> list[list[float]]:
    footprint = config.get("footprint") or []
    margin = float(config.get("safety_margin_m", 0.0))
    if margin <= EPSILON or len(footprint) < 3:
        return footprint
    xs = [float(p[0]) for p in footprint]
    ys = [float(p[1]) for p in footprint]
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin
    return [[min_x, max_y], [max_x, max_y], [max_x, min_y], [min_x, min_y]]


def footprint_is_safe(
    model: OpenMowerMap,
    corners: list[tuple[float, float]],
    lawns: list[Lawn] | None = None,
    obstacles: list[MapArea] | None = None,
) -> bool:
    check_lawns = lawns if lawns is not None else model.lawns
    check_obstacles = obstacles if obstacles is not None else model.obstacles
    for corner in corners:
        if not any(point_in_lawn(corner, lawn) for lawn in check_lawns):
            return False
        if any(point_in_ring(corner, obs.outline) for obs in check_obstacles):
            return False
    return True


def selected_metric_lawns(
    model: OpenMowerMap,
    compat: dict[str, Any],
    debug: dict[str, Any],
) -> tuple[list[int], list[Lawn], list[MapArea]]:
    indices: set[int] = set()
    for area in debug.get("areas", []):
        if isinstance(area, dict) and "area_index" in area:
            indices.add(int(area["area_index"]))
    for path in compat.get("paths", []):
        if isinstance(path, dict) and "area_index" in path:
            indices.add(int(path["area_index"]))
    valid_indices = sorted(i for i in indices if 0 <= i < len(model.lawns))
    if not valid_indices:
        valid_indices = list(range(len(model.lawns)))

    lawns = [model.lawns[i] for i in valid_indices]
    obstacle_ids: set[str] = set()
    obstacles: list[MapArea] = []
    for lawn in lawns:
        for obstacle in lawn.holes:
            if obstacle.id in obstacle_ids:
                continue
            obstacle_ids.add(obstacle.id)
            obstacles.append(obstacle)
    return valid_indices, lawns, obstacles


def segment_bins(
    segments: list[tuple[tuple[float, float], tuple[float, float]]],
    min_x: float,
    min_y: float,
    x_steps: int,
    y_steps: int,
    sample_res: float,
    radius: float,
) -> dict[tuple[int, int], list[tuple[tuple[float, float], tuple[float, float]]]]:
    bins: dict[tuple[int, int], list[tuple[tuple[float, float], tuple[float, float]]]] = {}
    pad = radius + sample_res
    for a, b in segments:
        ix0 = max(0, int(math.floor((min(a[0], b[0]) - pad - min_x) / sample_res)))
        ix1 = min(x_steps - 1, int(math.ceil((max(a[0], b[0]) + pad - min_x) / sample_res)))
        iy0 = max(0, int(math.floor((min(a[1], b[1]) - pad - min_y) / sample_res)))
        iy1 = min(y_steps - 1, int(math.ceil((max(a[1], b[1]) + pad - min_y) / sample_res)))
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                bins.setdefault((ix, iy), []).append((a, b))
    return bins


def count_section_runs(paths: list[dict[str, Any]], section: str) -> int:
    count = 0
    active = False
    for path in paths:
        for pose in path_poses(path):
            if pose.get("section") == section:
                if not active:
                    count += 1
                    active = True
            else:
                active = False
        active = False
    return count


def section_yaw_motion(paths: list[dict[str, Any]], section: str) -> float:
    total = 0.0
    for path in paths:
        poses = path_poses(path)
        for i in range(len(poses) - 1):
            if poses[i].get("section") == section and poses[i + 1].get("section") == section:
                total += abs(angle_delta(float(poses[i].get("yaw", 0.0)), float(poses[i + 1].get("yaw", 0.0))))
    return total


def count_turn_planners(paths: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for path in paths:
        active: str | None = None
        for pose in path_poses(path):
            if pose.get("section") != "turn":
                active = None
                continue
            planner = str(pose.get("turn_planner", "unknown"))
            if planner == active:
                continue
            counts[planner] = counts.get(planner, 0) + 1
            active = planner
    return counts


def compute_metrics(model: OpenMowerMap, compat: dict[str, Any], debug: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    paths = compat["paths"]
    selected_indices, metric_lawns, metric_obstacles = selected_metric_lawns(model, compat, debug)
    sample_res = float(config["evaluation"].get("coverage_sample_resolution_m", 0.1))
    all_points: list[tuple[float, float]] = []
    for lawn in metric_lawns:
        all_points.extend(lawn.area.outline)
        for hole in lawn.holes:
            all_points.extend(hole.outline)
    for path in paths:
        all_points.extend((p["x"], p["y"]) for p in path.get("path", {}).get("poses", []))
        all_points.extend((p["x"], p["y"]) for p in path.get("tool_path", {}).get("poses", []))
    min_x, min_y, max_x, max_y = bbox(all_points)
    pad = max(float(config["tool_width"]), 0.5)
    min_x, min_y, max_x, max_y = min_x - pad, min_y - pad, max_x + pad, max_y + pad

    coverage_segments = [
        ((a["x"], a["y"]), (b["x"], b["y"]))
        for _, a, b in iter_pose_segments(paths, coverage_only=True, source="tool")
    ]
    radius = float(config["tool_width"]) / 2.0
    mowable = covered = overcut = 0
    x_steps = max(1, int(math.ceil((max_x - min_x) / sample_res)))
    y_steps = max(1, int(math.ceil((max_y - min_y) / sample_res)))
    bins = segment_bins(coverage_segments, min_x, min_y, x_steps, y_steps, sample_res, radius)
    for ix in range(x_steps):
        x = min_x + (ix + 0.5) * sample_res
        for iy in range(y_steps):
            y = min_y + (iy + 0.5) * sample_res
            point = (x, y)
            inside = any(point_in_lawn(point, lawn) for lawn in metric_lawns)
            hit = any(point_segment_distance(point, a, b) <= radius for a, b in bins.get((ix, iy), []))
            if inside:
                mowable += 1
                if hit:
                    covered += 1
            elif hit:
                overcut += 1

    footprint = safety_footprint(config)
    stride = max(1, int(config["evaluation"].get("footprint_sample_stride", 1)))
    marker_limit = max(0, int(config["evaluation"].get("unsafe_marker_limit", 500)))
    footprint_samples = unsafe_footprint_samples = obstacle_pose_samples = 0
    unsafe_straight_connector_samples = unsafe_pivot_samples = unsafe_turn_samples = 0
    unsafe_samples: list[dict[str, Any]] = []
    for path in paths:
        for i, pose in enumerate(path_poses(path)):
            if i % stride:
                continue
            footprint_samples += 1
            corners = transform_footprint(pose, footprint)
            safe = footprint_is_safe(model, corners, metric_lawns, metric_obstacles)
            if not safe:
                unsafe_footprint_samples += 1
                if pose.get("section") == "connector":
                    unsafe_straight_connector_samples += 1
                if pose.get("section") == "pivot":
                    unsafe_pivot_samples += 1
                if pose.get("section") == "turn":
                    unsafe_turn_samples += 1
                if len(unsafe_samples) < marker_limit:
                    unsafe_samples.append(
                        {
                            "x": pose["x"],
                            "y": pose["y"],
                            "yaw": pose.get("yaw", 0.0),
                            "section": pose.get("section", "unknown"),
                            "direction": pose.get("direction", "unknown"),
                            "cutting_enabled": bool(pose_cutting_enabled(path, pose)),
                            "path_label": path.get("label", ""),
                        }
                    )
            if any(point_in_ring((pose["x"], pose["y"]), obs.outline) for obs in metric_obstacles):
                obstacle_pose_samples += 1

    area_unit = sample_res * sample_res
    all_swaths = [sw for area in debug.get("areas", []) for sw in area.get("swaths", [])]
    swath_count = len(all_swaths)
    swath_length_total = sum(float(sw.get("length_m", 0.0)) for sw in all_swaths)
    mean_swath_length = (swath_length_total / swath_count) if swath_count else 0.0
    if all_swaths:
        sorted_lengths = sorted(float(sw.get("length_m", 0.0)) for sw in all_swaths)
        median_swath_length = sorted_lengths[len(sorted_lengths) // 2]
        min_swath_length = sorted_lengths[0]
        max_swath_length = sorted_lengths[-1]
        short_swath_threshold = max(2.0 * float(config.get("tool_width", 0.4)), 1.0)
        short_swath_count = sum(1 for ell in sorted_lengths if ell < short_swath_threshold)
    else:
        median_swath_length = 0.0
        min_swath_length = 0.0
        max_swath_length = 0.0
        short_swath_count = 0
        short_swath_threshold = 0.0
    headland_count = sum(1 for p in paths if p.get("is_outline"))
    connector_length = path_length(
        paths,
        lambda path, a, b: (not path.get("is_outline")) and a.get("section") == "connector" and b.get("section") == "connector",
        source="base",
    )
    straight_connector_length = path_length(
        paths,
        lambda path, a, b: (not path.get("is_outline")) and a.get("section") == "connector" and b.get("section") == "connector",
        source="base",
    )
    total_length = path_length(paths, source="base")
    tool_length = path_length(paths, source="tool")
    turn_length = path_length(
        paths,
        lambda path, a, b: (not path.get("is_outline")) and a.get("section") == "turn" and b.get("section") == "turn",
        source="base",
    )
    turn_reverse_length_m = path_length(
        paths,
        lambda path, a, b: (
            (not path.get("is_outline"))
            and a.get("section") == "turn"
            and b.get("section") == "turn"
            and (a.get("direction") == "backward" or b.get("direction") == "backward")
        ),
        source="base",
    )
    turn_forward_length_m = path_length(
        paths,
        lambda path, a, b: (
            (not path.get("is_outline"))
            and a.get("section") == "turn"
            and b.get("section") == "turn"
            and a.get("direction") != "backward"
            and b.get("direction") != "backward"
        ),
        source="base",
    )
    turn_cutting_length_m = path_length(
        paths,
        lambda path, a, b: (
            (not path.get("is_outline"))
            and a.get("section") == "turn"
            and b.get("section") == "turn"
            and is_coverage_section(path, a, b)
        ),
        source="tool",
    )
    maneuver_ids = {
        str(pose.get("maneuver_id"))
        for path in paths
        for pose in path_poses(path)
        if pose.get("maneuver_id")
    }
    wheel_anchor_errors = [
        float(pose["target_wheel_error_m"])
        for path in paths
        for pose in path_poses(path)
        if pose.get("turn_planner") == "wheel_anchor" and isinstance(pose.get("target_wheel_error_m"), (int, float))
    ]
    wheel_anchor_reverse_distances = {
        str(pose.get("maneuver_id")): float(pose.get("reverse_distance_m", 0.0))
        for path in paths
        for pose in path_poses(path)
        if pose.get("turn_planner") == "wheel_anchor" and pose.get("maneuver_id")
    }
    coverage = {
        "sample_resolution_m": sample_res,
        "mowable_area_m2": mowable * area_unit,
        "covered_area_m2": covered * area_unit,
        "uncovered_area_m2": max(0, mowable - covered) * area_unit,
        "overcut_area_m2": overcut * area_unit,
        "coverage_percent": (100.0 * covered / mowable) if mowable else 0.0,
        "shoelace_mowable_area_m2": sum(polygon_area(lawn) for lawn in metric_lawns),
    }
    # Aggregate per-area P0/P1 debug into top-level metrics
    headland_metrics: dict[str, Any] = {
        "strategy": "unknown",
        "eroded_polygon_area_m2": 0.0,
        "mainland_polygon_area_m2": 0.0,
        "unsafe_footprint_samples": 0,
    }
    cells_metrics: dict[str, Any] = {"count": 0, "areas_m2": []}
    transit_metrics: dict[str, Any] = {"count": 0, "total_length_m": 0.0}
    inter_cell_metrics: dict[str, Any] = {"direct_turn_count": 0}
    split_metrics: dict[str, Any] = {"within_cell_path_splits": 0}
    trim_metrics: dict[str, Any] = {"attempts": 0, "successes": 0, "total_distance_m": 0.0}
    seen_strategy: set[str] = set()
    for area in debug.get("areas", []):
        if not isinstance(area, dict):
            continue
        seen_strategy.add(str(area.get("headland_strategy", "unknown")))
        headland_metrics["eroded_polygon_area_m2"] += float(area.get("eroded_polygon_area_m2", 0.0) or 0.0)
        headland_metrics["mainland_polygon_area_m2"] += float(area.get("mainland_polygon_area_m2", 0.0) or 0.0)
        for cell in area.get("cells", []) or []:
            cells_metrics["count"] += 1
            cells_metrics["areas_m2"].append(float(cell.get("area_m2", 0.0)))
        transit_metrics["count"] += int(area.get("transit_count", 0) or 0)
        transit_metrics["total_length_m"] += float(area.get("transit_total_length_m", 0.0) or 0.0)
        inter_cell_metrics["direct_turn_count"] += int(area.get("inter_cell_direct_turns", 0) or 0)
        split_metrics["within_cell_path_splits"] += int(area.get("within_cell_path_splits", 0) or 0)
        trim_metrics["attempts"] += int(area.get("stripe_trim_attempts", 0) or 0)
        trim_metrics["successes"] += int(area.get("stripe_trim_successes", 0) or 0)
        trim_metrics["total_distance_m"] += float(area.get("stripe_trim_total_distance_m", 0.0) or 0.0)
    if len(seen_strategy) == 1:
        headland_metrics["strategy"] = next(iter(seen_strategy))
    elif seen_strategy:
        headland_metrics["strategy"] = "mixed"
    # Headland-section unsafe sample count (independent from connector/turn/pivot)
    unsafe_headland_samples = sum(
        1 for s in unsafe_samples if s.get("section") == "headland"
    )
    headland_metrics["unsafe_footprint_samples"] = unsafe_headland_samples
    return {
        "schema": "open_mower.coverage_lab.metrics.v0",
        "profile": compat.get("profile", primary_profile_name(config)),
        "map": str(model.path),
        "selected_area_indices": selected_indices,
        "area_count": len(metric_lawns),
        "map_area_count": len(model.lawns),
        "obstacle_count": len(metric_obstacles),
        "map_obstacle_count": len(model.obstacles),
        "swath_count": swath_count,
        "swath_length": {
            "total_m": swath_length_total,
            "mean_m": mean_swath_length,
            "median_m": median_swath_length,
            "min_m": min_swath_length,
            "max_m": max_swath_length,
            "short_threshold_m": short_swath_threshold,
            "short_count": short_swath_count,
        },
        "headland_path_count": headland_count,
        "path": {
            "total_length_m": total_length,
            "connector_length_m": connector_length,
        },
        "coverage": coverage,
        "tool_coverage": coverage,
        "base_link_path": {
            "total_length_m": total_length,
            "connector_length_m": connector_length,
            "straight_connector_length_m": straight_connector_length,
        },
        "tool_path": {
            "total_length_m": tool_length,
            "coverage_length_m": path_length(paths, lambda path, a, b: is_coverage_section(path, a, b), source="tool"),
        },
        "connector_path": {
            "straight_connector_length_m": straight_connector_length,
            "unsafe_straight_connector_samples": unsafe_straight_connector_samples,
            "connector_runs": count_section_runs(paths, "connector"),
        },
        "pivot_sweeps": {
            "count": count_section_runs(paths, "pivot"),
            "total_yaw_rad": section_yaw_motion(paths, "pivot"),
            "unsafe_pivot_samples": unsafe_pivot_samples,
        },
        "turn_path": {
            "count": count_section_runs(paths, "turn"),
            "length_m": turn_length,
            "forward_length_m": turn_forward_length_m,
            "reverse_length_m": turn_reverse_length_m,
            "cutting_length_m": turn_cutting_length_m,
            "total_yaw_rad": section_yaw_motion(paths, "turn"),
            "unsafe_turn_samples": unsafe_turn_samples,
            "planner_counts": count_turn_planners(paths),
            "maneuver_count": len(maneuver_ids),
            "wheel_anchor_target_error_max_m": max(wheel_anchor_errors, default=0.0),
            "wheel_anchor_reverse_distance_m": sum(wheel_anchor_reverse_distances.values()),
            "reverse_pose_count": sum(
                1
                for path in paths
                for pose in path_poses(path)
                if pose.get("section") == "turn" and pose.get("direction") == "backward"
            ),
        },
        "safety": {
            "footprint_samples": footprint_samples,
            "unsafe_footprint_samples": unsafe_footprint_samples,
            "obstacle_pose_samples": obstacle_pose_samples,
            "unsafe_straight_connector_samples": unsafe_straight_connector_samples,
            "unsafe_pivot_samples": unsafe_pivot_samples,
            "unsafe_turn_samples": unsafe_turn_samples,
            "unsafe_samples": unsafe_samples,
        },
        "headland": headland_metrics,
        "cells": cells_metrics,
        "transit": transit_metrics,
        "inter_cell": inter_cell_metrics,
        "path_splits": split_metrics,
        "stripe_trim": trim_metrics,
    }


def make_run_dir(map_path: pathlib.Path, output: pathlib.Path | None) -> pathlib.Path:
    if output:
        return output
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    return DEFAULT_RUNS_DIR / f"{stamp}-{map_path.stem}"


def empty_profile_result(profile_name: str, map_path: pathlib.Path, config: dict[str, Any]) -> dict[str, Any]:
    return {
        "compat": {
            "schema": "open_mower.planpath_compat.v0",
            "profile": profile_name,
            "frame_id": config.get("frame_id", "map"),
            "source_map": str(map_path),
            "paths": [],
        },
        "debug": {
            "schema": "open_mower.coverage_lab.debug_geometry.v0",
            "profile": profile_name,
            "areas": [],
            "warnings": [],
        },
    }


def plan_profile_results(
    model: OpenMowerMap,
    map_path: pathlib.Path,
    config: dict[str, Any],
    selected: Iterable[int],
) -> dict[str, dict[str, Any]]:
    profile_names = [primary_profile_name(config)] + comparison_profile_names(config)
    results = {name: empty_profile_result(name, map_path, config) for name in profile_names}
    for area_index in selected:
        if area_index < 0 or area_index >= len(model.lawns):
            die(f"area index out of range: {area_index}")
        area_results = plan_one_lawn_profiles(model.lawns[area_index], area_index, config)
        for profile_name in profile_names:
            area_result = area_results[profile_name]
            results[profile_name]["compat"]["paths"].extend(area_result["paths"])
            results[profile_name]["debug"]["areas"].append(area_result["debug_area"])
            results[profile_name]["debug"]["warnings"].extend(
                [f"area {area_index}: {w}" for w in area_result["warnings"]]
            )
    return results


def sanitize_planpath_compat(compat: dict[str, Any]) -> dict[str, Any]:
    clean = {
        "schema": compat.get("schema", "open_mower.planpath_compat.v0"),
        "profile": compat.get("profile", "unknown"),
        "frame_id": compat.get("frame_id", "map"),
        "source_map": compat.get("source_map", ""),
        "paths": [],
    }
    for path in compat.get("paths", []):
        clean_path = {
            "is_outline": bool(path.get("is_outline", False)),
            "area_index": path.get("area_index"),
            "area_id": path.get("area_id", ""),
            "label": path.get("label", ""),
            "path": {
                "frame_id": path.get("path", {}).get("frame_id", compat.get("frame_id", "map")),
                "poses": [
                    {
                        "x": float(pose["x"]),
                        "y": float(pose["y"]),
                        "yaw": float(pose.get("yaw", 0.0)),
                    }
                    for pose in path.get("path", {}).get("poses", [])
                ],
            },
        }
        clean["paths"].append(clean_path)
    return clean


def write_profile_artifacts(
    run_dir: pathlib.Path,
    map_path: pathlib.Path,
    model: OpenMowerMap,
    config: dict[str, Any],
    profile_result: dict[str, Any],
) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    compat = profile_result["compat"]
    debug = profile_result["debug"]
    metrics = compute_metrics(model, compat, debug, config)
    metrics["warnings"] = debug["warnings"]
    debug_with_preview = copy.deepcopy(debug)
    debug_with_preview["preview_paths"] = compat.get("paths", [])
    shutil.copyfile(map_path, run_dir / "source_map_snapshot.json")
    write_json(run_dir / "planpath_compat.json", sanitize_planpath_compat(compat))
    write_json(run_dir / "planning_debug.json", debug_with_preview)
    write_json(run_dir / "metrics.json", metrics)
    write_json(run_dir / "config_snapshot.json", config)


def render_all_reports(run_dir: pathlib.Path) -> None:
    profiles_dir = run_dir / "profiles"
    if profiles_dir.is_dir():
        for profile_dir in sorted(p for p in profiles_dir.iterdir() if p.is_dir()):
            if (profile_dir / "planpath_compat.json").exists():
                render_run(profile_dir)
    render_run(run_dir)


def plan_map(args: argparse.Namespace) -> pathlib.Path:
    map_path = pathlib.Path(args.map).resolve()
    config = load_config(pathlib.Path(args.config).resolve())
    if getattr(args, "skip_comparisons", False):
        config = copy.deepcopy(config)
        config.setdefault("profiles", {})["comparisons"] = []
    if getattr(args, "swath_angle_mode", None):
        config = copy.deepcopy(config)
        fields = config.setdefault("fields2cover", {})
        angle_cfg = fields.setdefault("swath_angle", {})
        angle_cfg["mode"] = args.swath_angle_mode
        if args.swath_angle_degrees is not None:
            angle_cfg["degrees"] = args.swath_angle_degrees
    if getattr(args, "outline_count", None) is not None:
        config = copy.deepcopy(config)
        config["outline_count"] = args.outline_count
    if getattr(args, "outline_clearance_m", None) is not None:
        config = copy.deepcopy(config)
        config["outline_clearance_m"] = args.outline_clearance_m
    if getattr(args, "turn_forward_extent_spacing_factor", None) is not None:
        config = copy.deepcopy(config)
        config["turn_forward_extent_spacing_factor"] = args.turn_forward_extent_spacing_factor
    if getattr(args, "turn_planner", None) is not None:
        config = copy.deepcopy(config)
        config["turn_planner"] = args.turn_planner
    if getattr(args, "turn_cutting_mode", None) is not None:
        config = copy.deepcopy(config)
        config["turn_cutting_mode"] = args.turn_cutting_mode
    if getattr(args, "turn_reverse_enabled", None) is not None:
        config = copy.deepcopy(config)
        config["turn_reverse_enabled"] = args.turn_reverse_enabled
    model = parse_map(map_path, repair_rings=args.repair_rings)
    run_dir = make_run_dir(map_path, pathlib.Path(args.output).resolve() if args.output else None)
    run_dir.mkdir(parents=True, exist_ok=True)

    selected = range(len(model.lawns)) if args.area_index is None else [args.area_index]
    profile_results = plan_profile_results(model, map_path, config, selected)
    primary = primary_profile_name(config)

    write_profile_artifacts(run_dir, map_path, model, config, profile_results[primary])
    for profile_name in comparison_profile_names(config):
        write_profile_artifacts(run_dir / "profiles" / profile_name, map_path, model, config, profile_results[profile_name])

    render_all_reports(run_dir)
    print(f"Wrote coverage lab run: {run_dir}")
    if not getattr(args, "suppress_display", False):
        print(f"Display report: {run_dir / 'plan.html'}")
    return run_dir


def svg_polyline(points: list[tuple[float, float]], tx: Any, **attrs: str) -> str:
    attr_text = " ".join(f'{key.replace("_", "-")}="{html.escape(str(value))}"' for key, value in attrs.items())
    coords = " ".join(f"{tx(x, y)[0]:.2f},{tx(x, y)[1]:.2f}" for x, y in points)
    return f"<polyline points=\"{coords}\" {attr_text} />"


def svg_polygon(points: list[tuple[float, float]], tx: Any, **attrs: str) -> str:
    attr_text = " ".join(f'{key.replace("_", "-")}="{html.escape(str(value))}"' for key, value in attrs.items())
    coords = " ".join(f"{tx(x, y)[0]:.2f},{tx(x, y)[1]:.2f}" for x, y in points)
    return f"<polygon points=\"{coords}\" {attr_text} />"


def metric_number(metrics: dict[str, Any], section: str, key: str, default: float = 0.0) -> float:
    value = metrics.get(section, {}).get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def comparison_rows(run_dir: pathlib.Path, current_metrics: dict[str, Any]) -> str:
    rows = []

    def add_row(profile_name: str, href: str, metrics: dict[str, Any]) -> None:
        coverage = metric_number(metrics, "tool_coverage", "coverage_percent")
        length = metric_number(metrics, "base_link_path", "total_length_m")
        connector = metric_number(metrics, "base_link_path", "connector_length_m")
        safety = metrics.get("safety", {})
        warnings = len(metrics.get("warnings", []))
        rows.append(
            "<tr>"
            f"<td>{html.escape(profile_name)}</td>"
            f"<td><a href=\"{html.escape(href)}\">report</a></td>"
            f"<td>{coverage:.2f}</td>"
            f"<td>{length:.2f}</td>"
            f"<td>{connector:.2f}</td>"
            f"<td>{int(safety.get('unsafe_footprint_samples', 0))}</td>"
            f"<td>{int(safety.get('unsafe_straight_connector_samples', 0))}</td>"
            f"<td>{int(safety.get('unsafe_pivot_samples', 0))}</td>"
            f"<td>{warnings}</td>"
            "</tr>"
        )

    add_row(str(current_metrics.get("profile", run_dir.name)), "plan.html", current_metrics)
    profiles_dir = run_dir / "profiles"
    if profiles_dir.is_dir():
        for profile_dir in sorted(p for p in profiles_dir.iterdir() if p.is_dir()):
            metrics_path = profile_dir / "metrics.json"
            if not metrics_path.exists():
                continue
            profile_metrics = read_json(metrics_path)
            add_row(str(profile_metrics.get("profile", profile_dir.name)), f"profiles/{profile_dir.name}/plan.html", profile_metrics)

    if len(rows) <= 1:
        return ""
    return f"""
  <h2>Profile Comparison</h2>
  <table>
    <thead>
      <tr>
        <th>Profile</th>
        <th>Report</th>
        <th>Coverage %</th>
        <th>Base Length m</th>
        <th>Connector m</th>
        <th>Unsafe Footprint</th>
        <th>Unsafe Connectors</th>
        <th>Unsafe Pivots</th>
        <th>Warnings</th>
      </tr>
    </thead>
    <tbody>{"".join(rows)}</tbody>
  </table>
"""


def json_script(data: Any) -> str:
    return json.dumps(data, separators=(",", ":"), sort_keys=True).replace("<", "\\u003c").replace("&", "\\u0026")


def map_point_dict(point: tuple[float, float]) -> dict[str, float]:
    return {"x": float(point[0]), "y": float(point[1])}


def pose_xy(pose: dict[str, Any]) -> tuple[float, float]:
    return (float(pose["x"]), float(pose["y"]))


def transform_status(model: OpenMowerMap, area_index: int, pose: dict[str, Any], footprint: list[list[float]]) -> bool:
    if 0 <= area_index < len(model.lawns):
        lawn = model.lawns[area_index]
        return footprint_is_safe(model, transform_footprint(pose, footprint), [lawn], lawn.holes)
    return footprint_is_safe(model, transform_footprint(pose, footprint))


def build_simulation_preview(
    model: OpenMowerMap,
    compat: dict[str, Any],
    debug: dict[str, Any],
    metrics: dict[str, Any],
    config: dict[str, Any],
    view: dict[str, float],
) -> dict[str, Any]:
    footprint = [[float(p[0]), float(p[1])] for p in (config.get("footprint") or [])]
    safety_fp = [[float(p[0]), float(p[1])] for p in safety_footprint(config)]
    offset = tool_center_offset(config)
    front_x = max((p[0] for p in footprint), default=0.0)
    min_y = min((p[1] for p in footprint), default=0.0)
    max_y = max((p[1] for p in footprint), default=0.0)
    front_center = [front_x, (min_y + max_y) / 2.0]

    timeline: list[dict[str, Any]] = []
    segment_gaps: list[dict[str, Any]] = []
    cumulative_m = 0.0
    previous_path_end: dict[str, Any] | None = None
    previous_path_index: int | None = None

    paths = compat.get("paths", [])
    for path_index, path in enumerate(paths):
        base_poses = path.get("path", {}).get("poses", [])
        tool_poses = path.get("tool_path", {}).get("poses", [])
        if not base_poses:
            continue
        first_pose = base_poses[0]
        pending_gap: dict[str, Any] | None = None
        if previous_path_end is not None and previous_path_index is not None:
            gap_distance = dist(pose_xy(previous_path_end), pose_xy(first_pose))
            yaw_gap = abs(angle_delta(float(previous_path_end.get("yaw", 0.0)), float(first_pose.get("yaw", 0.0))))
            if gap_distance > 0.05 or yaw_gap > math.radians(5.0):
                pending_gap = {
                    "from_path_index": previous_path_index,
                    "to_path_index": path_index,
                    "distance_m": gap_distance,
                    "from": {
                        "x": float(previous_path_end["x"]),
                        "y": float(previous_path_end["y"]),
                        "yaw": float(previous_path_end.get("yaw", 0.0)),
                    },
                    "to": {
                        "x": float(first_pose["x"]),
                        "y": float(first_pose["y"]),
                        "yaw": float(first_pose.get("yaw", 0.0)),
                    },
                    "label": "unplanned first-point move between path segments",
                }
                segment_gaps.append(pending_gap)

        for pose_index, pose in enumerate(base_poses):
            if pose_index > 0:
                cumulative_m += dist(pose_xy(base_poses[pose_index - 1]), pose_xy(pose))
            area_index = int(path.get("area_index", -1))
            physical_safe = transform_status(model, area_index, pose, footprint)
            safety_safe = transform_status(model, area_index, pose, safety_fp)
            if not physical_safe:
                safety_status = "collision"
            elif not safety_safe:
                safety_status = "margin"
            else:
                safety_status = "safe"
            tool_pose = tool_poses[pose_index] if pose_index < len(tool_poses) else None
            wheel_points = wheel_points_for_pose(pose, config)
            record = {
                "global_index": len(timeline),
                "path_index": path_index,
                "pose_index": pose_index,
                "area_index": area_index,
                "area_id": path.get("area_id", ""),
                "label": path.get("label", f"path {path_index}"),
                "is_outline": bool(path.get("is_outline", False)),
                "section": str(pose.get("section", "unknown")),
                "direction": str(pose.get("direction", "unknown")),
                "cutting_enabled": bool(pose_cutting_enabled(path, pose)),
                "turn_leg": pose.get("turn_leg"),
                "turn_planner": pose.get("turn_planner"),
                "turn_candidate": pose.get("turn_candidate"),
                "maneuver_id": pose.get("maneuver_id"),
                "maneuver_type": pose.get("maneuver_type"),
                "phase": pose.get("phase"),
                "pivot_wheel": pose.get("pivot_wheel"),
                "anchor_point": pose.get("anchor_point"),
                "target_anchor_point": pose.get("target_anchor_point"),
                "left_wheel": pose.get("left_wheel") or wheel_points["left"],
                "right_wheel": pose.get("right_wheel") or wheel_points["right"],
                "blade_enabled": bool(pose.get("blade_enabled", pose_cutting_enabled(path, pose))),
                "target_wheel_error_m": pose.get("target_wheel_error_m"),
                "min_clearance_m": pose.get("min_clearance_m"),
                "reverse_distance_m": pose.get("reverse_distance_m"),
                "pivot_angle_deg": pose.get("pivot_angle_deg"),
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose.get("yaw", 0.0)),
                "tool_pose": (
                    {
                        "x": float(tool_pose["x"]),
                        "y": float(tool_pose["y"]),
                        "yaw": float(tool_pose.get("yaw", pose.get("yaw", 0.0))),
                    }
                    if tool_pose
                    else None
                ),
                "cumulative_distance_m": cumulative_m,
                "physical_safe": physical_safe,
                "safety_margin_safe": safety_safe,
                "safety_status": safety_status,
            }
            if pose_index == 0 and pending_gap is not None:
                record["segment_gap"] = pending_gap
            timeline.append(record)

        previous_path_end = base_poses[-1]
        previous_path_index = path_index

    maneuvers_by_id: dict[str, dict[str, Any]] = {}
    for record in timeline:
        maneuver_id = record.get("maneuver_id")
        if not maneuver_id:
            continue
        maneuver = maneuvers_by_id.setdefault(
            str(maneuver_id),
            {
                "maneuver_id": str(maneuver_id),
                "maneuver_type": record.get("maneuver_type"),
                "path_index": record.get("path_index"),
                "start_global_index": record.get("global_index"),
                "end_global_index": record.get("global_index"),
                "phases": [],
                "pivot_wheel": record.get("pivot_wheel"),
                "anchor_point": record.get("anchor_point"),
                "target_anchor_point": record.get("target_anchor_point"),
                "target_wheel_error_m": record.get("target_wheel_error_m"),
                "min_clearance_m": record.get("min_clearance_m"),
                "reverse_distance_m": record.get("reverse_distance_m"),
                "pivot_angle_deg": record.get("pivot_angle_deg"),
            },
        )
        maneuver["end_global_index"] = record.get("global_index")
        maneuver["target_wheel_error_m"] = record.get("target_wheel_error_m", maneuver.get("target_wheel_error_m"))
        maneuver["min_clearance_m"] = record.get("min_clearance_m", maneuver.get("min_clearance_m"))
        maneuver["reverse_distance_m"] = record.get("reverse_distance_m", maneuver.get("reverse_distance_m"))
        maneuver["pivot_angle_deg"] = record.get("pivot_angle_deg", maneuver.get("pivot_angle_deg"))
        if not maneuver.get("target_anchor_point") and record.get("target_anchor_point"):
            maneuver["target_anchor_point"] = record.get("target_anchor_point")
        phase = record.get("phase")
        if phase:
            phases = maneuver["phases"]
            if not phases or phases[-1]["phase"] != phase:
                phases.append(
                    {
                        "phase": phase,
                        "direction": record.get("direction"),
                        "pivot_wheel": record.get("pivot_wheel"),
                        "start_global_index": record.get("global_index"),
                        "end_global_index": record.get("global_index"),
                    }
                )
            else:
                phases[-1]["end_global_index"] = record.get("global_index")

    map_payload = {
        "lawns": [
            {
                "area_index": i,
                "id": lawn.area.id,
                "name": lawn.area.name,
                "outline": [map_point_dict(p) for p in lawn.area.outline],
                "holes": [hole.id for hole in lawn.holes],
            }
            for i, lawn in enumerate(model.lawns)
        ],
        "obstacles": [
            {
                "id": obstacle.id,
                "name": obstacle.name,
                "outline": [map_point_dict(p) for p in obstacle.outline],
            }
            for obstacle in model.obstacles
        ],
    }
    return {
        "schema": "open_mower.coverage_lab.simulation_preview.v0",
        "profile": compat.get("profile", metrics.get("profile", "unknown")),
        "frame_id": compat.get("frame_id", "map"),
        "view": view,
        "map": map_payload,
        "debug_summary": {
            "area_count": len(debug.get("areas", [])),
            "warning_count": len(debug.get("warnings", [])),
        },
        "mower": {
            "base_link": [0.0, 0.0],
            "front_center": front_center,
            "tool_center_offset": [float(offset[0]), float(offset[1])],
            "wheel_track_m": wheel_track_m(config),
            "wheel_contact_x_m": wheel_contact_x_m(config),
            "left_wheel": list(wheel_local_point(config, "left")),
            "right_wheel": list(wheel_local_point(config, "right")),
            "footprint": footprint,
            "safety_footprint": safety_fp,
            "safety_margin_m": float(config.get("safety_margin_m", 0.0)),
        },
        "counts": {
            "timeline": len(timeline),
            "unsafe_physical": sum(1 for record in timeline if not record["physical_safe"]),
            "unsafe_margin_only": sum(
                1 for record in timeline if record["physical_safe"] and not record["safety_margin_safe"]
            ),
            "segment_gaps": len(segment_gaps),
            "reverse_turn_poses": sum(
                1 for record in timeline if record["section"] == "turn" and record["direction"] == "backward"
            ),
            "cutting_disabled_poses": sum(1 for record in timeline if not record["cutting_enabled"]),
            "maneuvers": len(maneuvers_by_id),
        },
        "maneuvers": list(maneuvers_by_id.values()),
        "segment_gaps": segment_gaps,
        "timeline": timeline,
    }


def svg_group(group_id: str, children: list[str], **attrs: str) -> str:
    attr_text = " ".join(f'{key.replace("_", "-")}="{html.escape(str(value))}"' for key, value in attrs.items())
    body = "\n".join(children)
    return f'<g id="{html.escape(group_id)}" {attr_text}>\n{body}\n</g>'


def svg_circle(x: float, y: float, r: float, **attrs: str) -> str:
    attr_text = " ".join(f'{key.replace("_", "-")}="{html.escape(str(value))}"' for key, value in attrs.items())
    return f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" {attr_text} />'


def split_pose_runs(poses: list[dict[str, Any]], predicate: Any) -> list[list[tuple[float, float]]]:
    runs: list[list[tuple[float, float]]] = []
    active: list[tuple[float, float]] = []
    for i in range(len(poses) - 1):
        a, b = poses[i], poses[i + 1]
        if predicate(a, b):
            if not active:
                active.append((float(a["x"]), float(a["y"])))
            active.append((float(b["x"]), float(b["y"])))
        elif active:
            runs.append(active)
            active = []
    if active:
        runs.append(active)
    return runs


def render_run(run_dir: pathlib.Path) -> None:
    source = read_json(run_dir / "source_map_snapshot.json")
    compat = read_json(run_dir / "planpath_compat.json")
    debug = read_json(run_dir / "planning_debug.json")
    preview_compat = copy.deepcopy(compat)
    if isinstance(debug.get("preview_paths"), list):
        preview_compat["paths"] = debug["preview_paths"]
    metrics = read_json(run_dir / "metrics.json")
    config_path = run_dir / "config_snapshot.json"
    config = read_json(config_path) if config_path.exists() else load_config(DEFAULT_CONFIG)
    temp_map = run_dir / "source_map_snapshot.json"
    model = parse_map(temp_map, repair_rings=True)

    all_points: list[tuple[float, float]] = []
    for lawn in model.lawns:
        all_points.extend(lawn.area.outline)
        for hole in lawn.holes:
            all_points.extend(hole.outline)
    for path in preview_compat.get("paths", []):
        all_points.extend((p["x"], p["y"]) for p in path.get("path", {}).get("poses", []))
        all_points.extend((p["x"], p["y"]) for p in path.get("tool_path", {}).get("poses", []))
        for pose in path.get("path", {}).get("poses", []):
            all_points.extend(transform_footprint(pose, config.get("footprint") or []))
            all_points.extend(transform_footprint(pose, safety_footprint(config)))
    min_x, min_y, max_x, max_y = bbox(all_points)
    width_m = max(max_x - min_x, 1.0)
    height_m = max(max_y - min_y, 1.0)
    scale = min(90.0, max(20.0, 1200.0 / max(width_m, height_m)))
    pad = 30.0
    svg_w = width_m * scale + 2 * pad
    svg_h = height_m * scale + 2 * pad
    view = {
        "min_x": min_x,
        "min_y": min_y,
        "max_x": max_x,
        "max_y": max_y,
        "scale": scale,
        "pad": pad,
        "svg_width": svg_w,
        "svg_height": svg_h,
    }
    simulation_preview = build_simulation_preview(model, preview_compat, debug, metrics, config, view)
    write_json(run_dir / "simulation_preview.json", simulation_preview)

    def tx(x: float, y: float) -> tuple[float, float]:
        return (pad + (x - min_x) * scale, pad + (max_y - y) * scale)

    svg_defs = """
<defs>
  <marker id="mower-front-arrow" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto" markerUnits="strokeWidth">
    <path d="M0,0 L0,6 L7,3 z" fill="#111827" />
  </marker>
</defs>""".strip()
    lawn_elements: list[str] = []
    debug_elements: list[str] = []
    tool_elements: list[str] = []
    connector_elements: list[str] = []
    pivot_elements: list[str] = []
    base_elements: list[str] = []
    unsafe_elements: list[str] = []
    gap_elements: list[str] = []
    marker_elements: list[str] = []
    left_wheel_elements: list[str] = []
    right_wheel_elements: list[str] = []
    turn_anchor_elements: list[str] = []
    swept_elements: list[str] = []

    for lawn in model.lawns:
        lawn_elements.append(svg_polygon(lawn.area.outline, tx, fill="#d7efd2", stroke="#1d7a32", stroke_width="2"))
        for hole in lawn.holes:
            lawn_elements.append(svg_polygon(hole.outline, tx, fill="#ffd5d5", stroke="#b42323", stroke_width="2"))

    for area in debug.get("areas", []):
        for ring in area.get("mainland_rings", []):
            pts = [(p["x"], p["y"]) for p in ring]
            debug_elements.append(
                svg_polyline(pts, tx, fill="none", stroke="#f59f00", stroke_width="1.5", stroke_dasharray="5 5")
            )
        for swath in area.get("swaths", []):
            pts = [(p["x"], p["y"]) for p in swath.get("points", [])]
            debug_elements.append(svg_polyline(pts, tx, fill="none", stroke="#7b8794", stroke_width="1", opacity="0.55"))

    for gap in simulation_preview.get("segment_gaps", []):
        pts = [(gap["from"]["x"], gap["from"]["y"]), (gap["to"]["x"], gap["to"]["y"])]
        gap_elements.append(
            svg_polyline(pts, tx, fill="none", stroke="#be185d", stroke_width="1.8", stroke_dasharray="3 6", opacity="0.85")
        )

    swath_colors = ["#2563eb", "#00897b"]
    for i, path in enumerate(preview_compat.get("paths", [])):
        base_poses = path.get("path", {}).get("poses", [])
        tool_poses = path.get("tool_path", {}).get("poses", [])
        base_pts = [(p["x"], p["y"]) for p in base_poses]
        if len(base_pts) >= 2:
            base_elements.append(
                svg_polyline(base_pts, tx, fill="none", stroke="#111827", stroke_width="1.2", stroke_dasharray="4 4", opacity="0.65")
            )

        coverage_color = "#234f1e" if path.get("is_outline") else swath_colors[i % len(swath_colors)]
        coverage_width = "2.1" if path.get("is_outline") else "2.8"
        for run in split_pose_runs(
            tool_poses,
            lambda a, b: path.get("is_outline")
            or (a.get("section") in {"headland", "swath", "turn"} and b.get("section") in {"headland", "swath", "turn"}),
        ):
            if len(run) >= 2:
                tool_elements.append(
                    svg_polyline(run, tx, fill="none", stroke=coverage_color, stroke_width=coverage_width, opacity="0.95")
                )
        for run in split_pose_runs(
            tool_poses,
            lambda a, b: a.get("section") == "connector" or b.get("section") == "connector",
        ):
            if len(run) >= 2:
                connector_elements.append(
                    svg_polyline(run, tx, fill="none", stroke="#f97316", stroke_width="2.0", stroke_dasharray="6 5", opacity="0.9")
                )
        for pose in tool_poses:
            if pose.get("section") == "pivot":
                sx, sy = tx(float(pose["x"]), float(pose["y"]))
                pivot_elements.append(svg_circle(sx, sy, 1.6, fill="#7c3aed", opacity="0.65"))

        pts = [(p["x"], p["y"]) for p in (tool_poses or base_poses)]
        if len(pts) >= 2:
            sx, sy = tx(*pts[0])
            ex, ey = tx(*pts[-1])
            marker_elements.append(svg_circle(sx, sy, 4.0, fill=coverage_color))
            marker_elements.append(svg_circle(ex, ey, 4.0, fill="#ffffff", stroke=coverage_color, stroke_width="2"))

    left_run: list[tuple[float, float]] = []
    right_run: list[tuple[float, float]] = []
    anchor_keys: set[tuple[str, int, int]] = set()
    wheel_turn_records = [
        record
        for record in simulation_preview.get("timeline", [])
        if record.get("section") == "turn" and record.get("turn_planner") == "wheel_anchor"
    ]

    def flush_wheel_runs() -> None:
        nonlocal left_run, right_run
        if len(left_run) >= 2:
            left_wheel_elements.append(
                svg_polyline(left_run, tx, fill="none", stroke="#0f766e", stroke_width="1.5", stroke_dasharray="2 4", opacity="0.85")
            )
        if len(right_run) >= 2:
            right_wheel_elements.append(
                svg_polyline(right_run, tx, fill="none", stroke="#9333ea", stroke_width="1.5", stroke_dasharray="2 4", opacity="0.85")
            )
        left_run = []
        right_run = []

    previous_maneuver_id: str | None = None
    for record in wheel_turn_records:
        maneuver_id = str(record.get("maneuver_id") or "")
        if previous_maneuver_id is not None and maneuver_id != previous_maneuver_id:
            flush_wheel_runs()
        previous_maneuver_id = maneuver_id
        left = record.get("left_wheel") or {}
        right = record.get("right_wheel") or {}
        if "x" in left and "y" in left:
            left_run.append((float(left["x"]), float(left["y"])))
        if "x" in right and "y" in right:
            right_run.append((float(right["x"]), float(right["y"])))
    flush_wheel_runs()

    for record in wheel_turn_records:
        for key_name, color, radius, label in (
            ("anchor_point", "#ea580c", 3.2, "pivot"),
            ("target_anchor_point", "#0891b2", 2.8, "target"),
        ):
            point = record.get(key_name)
            if not point:
                continue
            key = (key_name, int(round(float(point["x"]) * 1000)), int(round(float(point["y"]) * 1000)))
            if key in anchor_keys:
                continue
            anchor_keys.add(key)
            sx, sy = tx(float(point["x"]), float(point["y"]))
            turn_anchor_elements.append(
                svg_circle(
                    sx,
                    sy,
                    radius,
                    fill=color,
                    opacity="0.90",
                    **{"data-anchor": label},
                )
            )

    footprint = [[float(p[0]), float(p[1])] for p in (config.get("footprint") or [])]
    if footprint:
        stride = max(1, int(math.ceil(len(wheel_turn_records) / 180))) if wheel_turn_records else 1
        for i, record in enumerate(wheel_turn_records):
            if i % stride:
                continue
            swept_elements.append(
                svg_polygon(
                    transform_footprint(record, footprint),
                    tx,
                    fill="#0284c7",
                    fill_opacity="0.055",
                    stroke="#0284c7",
                    stroke_width="0.7",
                    stroke_opacity="0.22",
                )
            )

    for sample in metrics.get("safety", {}).get("unsafe_samples", []):
        sx, sy = tx(float(sample["x"]), float(sample["y"]))
        unsafe_elements.append(svg_circle(sx, sy, 3.5, fill="#dc2626", opacity="0.85"))

    mower_overlay = """
<g id="layer-mower-safety">
  <polygon id="mower-safety-footprint" points="" fill="#f59e0b" fill-opacity="0.10" stroke="#d97706" stroke-width="1.2" stroke-dasharray="5 4" />
</g>
<g id="layer-mower">
  <polygon id="mower-footprint" points="" fill="#16a34a" fill-opacity="0.20" stroke="#166534" stroke-width="2.2" />
  <line id="mower-heading" x1="0" y1="0" x2="0" y2="0" stroke="#111827" stroke-width="2" marker-end="url(#mower-front-arrow)" />
  <circle id="mower-base-link" cx="0" cy="0" r="4" fill="#111827" />
  <circle id="mower-tool-center" cx="0" cy="0" r="3.5" fill="#ffffff" stroke="#2563eb" stroke-width="2" />
</g>""".strip()

    elements = [
        f'<svg id="plan-map" xmlns="http://www.w3.org/2000/svg" width="{svg_w:.0f}" height="{svg_h:.0f}" viewBox="0 0 {svg_w:.0f} {svg_h:.0f}">',
        '<rect width="100%" height="100%" fill="#f7f7f2" />',
        svg_defs,
        '<g id="map-viewport">',
        svg_group("layer-lawns", lawn_elements),
        svg_group("layer-debug", debug_elements),
        svg_group("layer-tool", tool_elements),
        svg_group("layer-connectors", connector_elements),
        svg_group("layer-pivots", pivot_elements),
        svg_group("layer-left-wheel", left_wheel_elements),
        svg_group("layer-right-wheel", right_wheel_elements),
        svg_group("layer-turn-anchors", turn_anchor_elements),
        svg_group("layer-swept-footprint", swept_elements),
        svg_group("layer-base", base_elements),
        svg_group("layer-gaps", gap_elements),
        svg_group("layer-unsafe", unsafe_elements),
        svg_group("layer-markers", marker_elements),
        mower_overlay,
        "</g>",
    ]
    elements.append("</svg>")
    svg = "\n".join(elements)
    (run_dir / "plan.svg").write_text(svg, encoding="utf-8")

    title = html.escape(run_dir.name)
    metric_rows = "\n".join(
        f"<tr><th>{html.escape(k)}</th><td>{html.escape(json.dumps(v, sort_keys=True))}</td></tr>"
        for k, v in metrics.items()
        if k not in {"schema"}
    )
    preview_json = json_script(simulation_preview)
    preview_counts = simulation_preview.get("counts", {})
    viewer_script = r"""
<script>
(function () {
  const dataNode = document.getElementById("simulation-data");
  const data = JSON.parse(dataNode.textContent);
  const timeline = data.timeline || [];
  const view = data.view || {};
  const mower = data.mower || {};
  const svg = document.getElementById("plan-map");
  const mapViewport = document.getElementById("map-viewport");
  const slider = document.getElementById("pose-slider");
  const playButton = document.getElementById("preview-play");
  const stepBack = document.getElementById("preview-step-back");
  const stepForward = document.getElementById("preview-step-forward");
  const speed = document.getElementById("preview-speed");
  const zoomIn = document.getElementById("preview-zoom-in");
  const zoomOut = document.getElementById("preview-zoom-out");
  const zoomFit = document.getElementById("preview-zoom-fit");
  const zoomLevel = document.getElementById("preview-zoom-level");
  const rotateLeft = document.getElementById("preview-rotate-left");
  const rotateRight = document.getElementById("preview-rotate-right");
  const rotateReset = document.getElementById("preview-rotate-reset");
  const rotateSlider = document.getElementById("preview-rotate-slider");
  const rotateLevel = document.getElementById("preview-rotate-level");
  const prevUnsafe = document.getElementById("preview-prev-unsafe");
  const nextUnsafe = document.getElementById("preview-next-unsafe");
  const safetyToggle = document.getElementById("show-safety-footprint");
  const footprint = document.getElementById("mower-footprint");
  const safetyFootprint = document.getElementById("mower-safety-footprint");
  const baseDot = document.getElementById("mower-base-link");
  const toolDot = document.getElementById("mower-tool-center");
  const heading = document.getElementById("mower-heading");
  const status = document.getElementById("preview-status");
  const poseMeta = document.getElementById("preview-pose-meta");
  const gapMeta = document.getElementById("preview-gap-meta");
  const initialViewBox = {
    x: 0,
    y: 0,
    w: Number(view.svg_width || svg.getAttribute("width") || 1),
    h: Number(view.svg_height || svg.getAttribute("height") || 1)
  };
  let currentViewBox = Object.assign({}, initialViewBox);
  let panStart = null;
  let rotationDeg = 0;
  let timer = null;

  function toSvg(x, y) {
    return {
      x: view.pad + (x - view.min_x) * view.scale,
      y: view.pad + (view.max_y - y) * view.scale
    };
  }

  function rotate(x, y, yaw) {
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return {x: c * x - s * y, y: s * x + c * y};
  }

  function localToMap(pose, point) {
    const rotated = rotate(point[0], point[1], pose.yaw || 0);
    return {x: pose.x + rotated.x, y: pose.y + rotated.y};
  }

  function polygonPoints(pose, localPoints) {
    return (localPoints || []).map(function (point) {
      const mapPoint = localToMap(pose, point);
      const svgPoint = toSvg(mapPoint.x, mapPoint.y);
      return svgPoint.x.toFixed(2) + "," + svgPoint.y.toFixed(2);
    }).join(" ");
  }

  function setCircle(circle, point) {
    circle.setAttribute("cx", point.x.toFixed(2));
    circle.setAttribute("cy", point.y.toFixed(2));
  }

  function setLine(line, a, b) {
    line.setAttribute("x1", a.x.toFixed(2));
    line.setAttribute("y1", a.y.toFixed(2));
    line.setAttribute("x2", b.x.toFixed(2));
    line.setAttribute("y2", b.y.toFixed(2));
  }

  function screenToSvgPoint(clientX, clientY, inverseMatrix) {
    const matrix = inverseMatrix || (svg.getScreenCTM() && svg.getScreenCTM().inverse());
    if (!matrix) {
      const rect = svg.getBoundingClientRect();
      return {
        x: currentViewBox.x + ((clientX - rect.left) / Math.max(rect.width, 1)) * currentViewBox.w,
        y: currentViewBox.y + ((clientY - rect.top) / Math.max(rect.height, 1)) * currentViewBox.h
      };
    }
    const point = svg.createSVGPoint();
    point.x = clientX;
    point.y = clientY;
    const transformed = point.matrixTransform(matrix);
    return {x: transformed.x, y: transformed.y};
  }

  function applyViewBox(nextViewBox) {
    const minWidth = initialViewBox.w / 40;
    const minHeight = initialViewBox.h / 40;
    const maxWidth = initialViewBox.w * 3;
    const maxHeight = initialViewBox.h * 3;
    currentViewBox = {
      x: nextViewBox.x,
      y: nextViewBox.y,
      w: Math.max(minWidth, Math.min(maxWidth, nextViewBox.w)),
      h: Math.max(minHeight, Math.min(maxHeight, nextViewBox.h))
    };
    svg.setAttribute(
      "viewBox",
      currentViewBox.x.toFixed(2) + " " +
      currentViewBox.y.toFixed(2) + " " +
      currentViewBox.w.toFixed(2) + " " +
      currentViewBox.h.toFixed(2)
    );
    zoomLevel.textContent = Math.round(initialViewBox.w / currentViewBox.w * 100) + "%";
  }

  function zoomAt(factor, clientX, clientY) {
    const anchor = screenToSvgPoint(clientX, clientY);
    const px = currentViewBox.w ? (anchor.x - currentViewBox.x) / currentViewBox.w : 0.5;
    const py = currentViewBox.h ? (anchor.y - currentViewBox.y) / currentViewBox.h : 0.5;
    const nextWidth = currentViewBox.w * factor;
    const nextHeight = currentViewBox.h * factor;
    applyViewBox({
      x: anchor.x - px * nextWidth,
      y: anchor.y - py * nextHeight,
      w: nextWidth,
      h: nextHeight
    });
  }

  function zoomFromCenter(factor) {
    const rect = svg.getBoundingClientRect();
    zoomAt(factor, rect.left + rect.width / 2, rect.top + rect.height / 2);
  }

  function normalizeDegrees(degrees) {
    let value = Number(degrees) || 0;
    while (value > 180) value -= 360;
    while (value < -180) value += 360;
    return value;
  }

  function applyRotation(degrees) {
    rotationDeg = normalizeDegrees(degrees);
    const cx = initialViewBox.w / 2;
    const cy = initialViewBox.h / 2;
    mapViewport.setAttribute(
      "transform",
      "rotate(" + rotationDeg.toFixed(2) + " " + cx.toFixed(2) + " " + cy.toFixed(2) + ")"
    );
    rotateSlider.value = String(Math.round(rotationDeg));
    rotateLevel.textContent = Math.round(rotationDeg) + " deg";
  }

  function safetyLabel(record) {
    if (!record) return "no poses";
    if (record.safety_status === "collision") return "collision";
    if (record.safety_status === "margin") return "inside boundary, safety margin violated";
    return "safe";
  }

  function styleFootprint(record) {
    const styles = {
      safe: ["#16a34a", "#166534"],
      margin: ["#f59e0b", "#b45309"],
      collision: ["#dc2626", "#991b1b"]
    };
    const pair = styles[record.safety_status] || styles.safe;
    footprint.setAttribute("fill", pair[0]);
    footprint.setAttribute("stroke", pair[1]);
    status.className = "status-pill status-" + (record.safety_status || "safe");
  }

  function update(index) {
    if (!timeline.length) {
      status.textContent = "No executable poses";
      return;
    }
    const clamped = Math.max(0, Math.min(timeline.length - 1, index));
    const record = timeline[clamped];
    slider.value = String(clamped);
    footprint.setAttribute("points", polygonPoints(record, mower.footprint));
    safetyFootprint.setAttribute("points", polygonPoints(record, mower.safety_footprint));
    safetyFootprint.style.display = safetyToggle.checked ? "" : "none";

    const base = toSvg(record.x, record.y);
    const toolMap = record.tool_pose || localToMap(record, mower.tool_center_offset || [0, 0]);
    const tool = toSvg(toolMap.x, toolMap.y);
    const frontMap = localToMap(record, mower.front_center || [0, 0]);
    const front = toSvg(frontMap.x, frontMap.y);
    setCircle(baseDot, base);
    setCircle(toolDot, tool);
    setLine(heading, base, front);
    styleFootprint(record);

    status.textContent = safetyLabel(record);
    const poseParts = [
      "pose " + (clamped + 1) + " / " + timeline.length,
      "path " + record.path_index,
      "index " + record.pose_index,
      "area " + record.area_index,
      record.section,
      record.direction,
      record.is_outline ? "outline" : "fill",
      record.cutting_enabled ? "cutting on" : "cutting off",
      record.cumulative_distance_m.toFixed(2) + " m"
    ];
    if (record.turn_leg) poseParts.push("leg " + record.turn_leg);
    if (record.turn_planner) poseParts.push(record.turn_planner);
    if (record.phase) poseParts.push(record.phase);
    if (record.pivot_wheel) poseParts.push("pivot " + record.pivot_wheel);
    if (Number.isFinite(record.target_wheel_error_m)) {
      poseParts.push("target err " + record.target_wheel_error_m.toFixed(3) + " m");
    }
    if (Number.isFinite(record.reverse_distance_m) && record.reverse_distance_m > 0) {
      poseParts.push("reverse " + record.reverse_distance_m.toFixed(2) + " m");
    }
    if (Number.isFinite(record.pivot_angle_deg)) {
      poseParts.push("pivot " + record.pivot_angle_deg.toFixed(1) + " deg");
    }
    poseMeta.textContent = poseParts.join(" | ");
    if (record.segment_gap) {
      gapMeta.textContent = "Unplanned first-point move from path " +
        record.segment_gap.from_path_index + " to " + record.segment_gap.to_path_index +
        ": " + record.segment_gap.distance_m.toFixed(2) + " m";
      gapMeta.hidden = false;
    } else {
      gapMeta.hidden = true;
    }
  }

  function setPlaying(playing) {
    if (timer) {
      clearInterval(timer);
      timer = null;
    }
    playButton.textContent = playing ? "Pause" : "Play";
    if (playing && timeline.length > 1) {
      const interval = Math.max(16, 1000 / Number(speed.value || 8));
      timer = setInterval(function () {
        const next = Number(slider.value) + 1;
        if (next >= timeline.length) {
          setPlaying(false);
          return;
        }
        update(next);
      }, interval);
    }
  }

  function seekUnsafe(direction) {
    if (!timeline.length) return;
    const current = Number(slider.value);
    let i = current + direction;
    while (i >= 0 && i < timeline.length) {
      if (timeline[i].safety_status !== "safe") {
        update(i);
        return;
      }
      i += direction;
    }
  }

  document.querySelectorAll("[data-layer-toggle]").forEach(function (input) {
    input.addEventListener("change", function () {
      const layer = document.getElementById(input.getAttribute("data-layer-toggle"));
      if (layer) layer.style.display = input.checked ? "" : "none";
    });
  });
  slider.max = String(Math.max(0, timeline.length - 1));
  slider.addEventListener("input", function () { update(Number(slider.value)); });
  playButton.addEventListener("click", function () { setPlaying(!timer); });
  speed.addEventListener("change", function () { if (timer) setPlaying(true); });
  stepBack.addEventListener("click", function () { update(Number(slider.value) - 1); });
  stepForward.addEventListener("click", function () { update(Number(slider.value) + 1); });
  zoomIn.addEventListener("click", function () { zoomFromCenter(0.8); });
  zoomOut.addEventListener("click", function () { zoomFromCenter(1.25); });
  zoomFit.addEventListener("click", function () { applyViewBox(initialViewBox); });
  rotateLeft.addEventListener("click", function () { applyRotation(rotationDeg - 15); });
  rotateRight.addEventListener("click", function () { applyRotation(rotationDeg + 15); });
  rotateReset.addEventListener("click", function () { applyRotation(0); });
  rotateSlider.addEventListener("input", function () { applyRotation(Number(rotateSlider.value)); });
  svg.addEventListener("wheel", function (event) {
    event.preventDefault();
    zoomAt(Math.exp(event.deltaY * 0.001), event.clientX, event.clientY);
  }, {passive: false});
  svg.addEventListener("pointerdown", function (event) {
    if (event.button !== 0) return;
    const screenCtm = svg.getScreenCTM();
    panStart = {
      clientX: event.clientX,
      clientY: event.clientY,
      viewBox: Object.assign({}, currentViewBox),
      inverseMatrix: screenCtm ? screenCtm.inverse() : null
    };
    svg.classList.add("is-panning");
    svg.setPointerCapture(event.pointerId);
  });
  svg.addEventListener("pointermove", function (event) {
    if (!panStart) return;
    const start = screenToSvgPoint(panStart.clientX, panStart.clientY, panStart.inverseMatrix);
    const current = screenToSvgPoint(event.clientX, event.clientY, panStart.inverseMatrix);
    const dx = current.x - start.x;
    const dy = current.y - start.y;
    applyViewBox({
      x: panStart.viewBox.x - dx,
      y: panStart.viewBox.y - dy,
      w: panStart.viewBox.w,
      h: panStart.viewBox.h
    });
  });
  function stopPan(event) {
    if (!panStart) return;
    panStart = null;
    svg.classList.remove("is-panning");
    if (event && svg.hasPointerCapture(event.pointerId)) {
      svg.releasePointerCapture(event.pointerId);
    }
  }
  svg.addEventListener("pointerup", stopPan);
  svg.addEventListener("pointercancel", stopPan);
  svg.addEventListener("pointerleave", stopPan);
  prevUnsafe.addEventListener("click", function () { seekUnsafe(-1); });
  nextUnsafe.addEventListener("click", function () { seekUnsafe(1); });
  safetyToggle.addEventListener("change", function () { update(Number(slider.value)); });
  applyViewBox(initialViewBox);
  applyRotation(0);
  update(0);
})();
</script>
""".strip()
    report = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{title}</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 16px; color: #1f2933; background: #fbfbf8; }}
    h1 {{ font-size: 22px; margin-bottom: 6px; }}
    h2 {{ font-size: 18px; margin: 0; }}
    button, select, input[type="range"] {{ font: inherit; }}
    button {{ border: 1px solid #c7ccd1; background: #ffffff; color: #111827; border-radius: 6px; padding: 6px 10px; cursor: pointer; }}
    button:hover {{ background: #f3f4f6; }}
    .report-meta {{ margin: 4px 0; }}
    .svg-wrap {{ border: 1px solid #d7d7d0; overflow: hidden; background: #f7f7f2; flex: 1 1 auto; min-height: 280px; }}
    #plan-map {{ width: 100%; height: 100%; display: block; touch-action: none; cursor: grab; user-select: none; }}
    #plan-map.is-panning {{ cursor: grabbing; }}
    .preview-panel {{ border: 1px solid #d7d7d0; background: #ffffff; border-radius: 8px; padding: 12px; margin: 14px 0; height: calc(100vh - 32px); min-height: 560px; box-sizing: border-box; display: flex; flex-direction: column; gap: 10px; }}
    .preview-title-row {{ display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center; gap: 10px; }}
    .preview-controls {{ display: grid; grid-template-columns: auto auto auto minmax(280px, 1fr) auto auto; gap: 8px; align-items: center; }}
    .preview-controls input[type="range"] {{ width: 100%; }}
    .map-toolbar {{ display: flex; flex-wrap: wrap; align-items: center; gap: 8px; }}
    .map-toolbar button {{ min-width: 36px; padding: 5px 8px; }}
    .toolbar-label {{ color: #4b5563; font-size: 13px; font-weight: 700; }}
    .rotation-slider {{ width: 120px; }}
    .zoom-readout, .rotate-readout {{ min-width: 58px; text-align: right; font-variant-numeric: tabular-nums; color: #4b5563; }}
    .preview-meta {{ display: flex; flex-wrap: wrap; align-items: center; gap: 10px; margin-top: 10px; font-size: 13px; }}
    .status-pill {{ display: inline-flex; align-items: center; min-width: 96px; justify-content: center; padding: 3px 8px; border-radius: 999px; font-weight: 700; }}
    .status-safe {{ color: #166534; background: #dcfce7; }}
    .status-margin {{ color: #92400e; background: #fef3c7; }}
    .status-collision {{ color: #991b1b; background: #fee2e2; }}
    .gap-note {{ color: #9d174d; font-weight: 700; }}
    .toggles, .legend {{ display: flex; flex-wrap: wrap; gap: 10px 14px; margin-top: 12px; font-size: 13px; }}
    .toggles label {{ white-space: nowrap; }}
    .legend span {{ display: inline-flex; align-items: center; gap: 5px; white-space: nowrap; }}
    .key {{ width: 22px; height: 0; border-top: 3px solid #111827; display: inline-block; }}
    .key-dashed {{ border-top-style: dashed; }}
    .key-dot {{ width: 9px; height: 9px; border: 0; border-radius: 50%; background: #dc2626; }}
    .key-fill {{ width: 16px; height: 10px; border: 2px solid #1d7a32; background: #d7efd2; }}
    .preview-counts {{ margin: 0; font-size: 13px; }}
    table {{ border-collapse: collapse; margin-top: 20px; max-width: 1100px; }}
    th, td {{ border: 1px solid #ddd; padding: 6px 8px; text-align: left; vertical-align: top; }}
    th {{ width: 220px; background: #f3f4f6; }}
    code {{ background: #f3f4f6; padding: 1px 4px; }}
  </style>
</head>
<body>
  <h1>{title}</h1>
  <p class="report-meta">Profile: <code>{html.escape(str(metrics.get("profile", preview_compat.get("profile", "unknown"))))}</code></p>
  <p class="report-meta">Source map: <code>{html.escape(str(source.get("id", "source_map_snapshot.json")))}</code></p>
  {comparison_rows(run_dir, metrics)}
  <section class="preview-panel">
    <div class="preview-title-row">
      <h2>Mower Path Preview</h2>
      <div class="map-toolbar">
        <span class="toolbar-label">Zoom</span>
        <button id="preview-zoom-out" type="button" title="Zoom out">-</button>
        <button id="preview-zoom-in" type="button" title="Zoom in">+</button>
        <button id="preview-zoom-fit" type="button" title="Fit map">Fit</button>
        <span id="preview-zoom-level" class="zoom-readout">100%</span>
        <span class="toolbar-label">Rotate</span>
        <button id="preview-rotate-left" type="button" title="Rotate left">-15</button>
        <input id="preview-rotate-slider" class="rotation-slider" type="range" min="-180" max="180" step="1" value="0" aria-label="Map rotation">
        <button id="preview-rotate-right" type="button" title="Rotate right">+15</button>
        <button id="preview-rotate-reset" type="button" title="Reset rotation">0</button>
        <span id="preview-rotate-level" class="rotate-readout">0 deg</span>
      </div>
    </div>
    <div class="preview-controls">
      <button id="preview-play" type="button">Play</button>
      <button id="preview-step-back" type="button">Back</button>
      <button id="preview-step-forward" type="button">Forward</button>
      <input id="pose-slider" type="range" min="0" max="0" value="0">
      <select id="preview-speed" aria-label="Preview speed">
        <option value="2">2 poses/s</option>
        <option value="8" selected>8 poses/s</option>
        <option value="20">20 poses/s</option>
        <option value="60">60 poses/s</option>
      </select>
      <span id="preview-status" class="status-pill status-safe">safe</span>
    </div>
    <div class="preview-meta">
      <span id="preview-pose-meta">pose 0 / 0</span>
      <span id="preview-gap-meta" class="gap-note" hidden></span>
      <button id="preview-prev-unsafe" type="button">Previous unsafe</button>
      <button id="preview-next-unsafe" type="button">Next unsafe</button>
    </div>
    <div class="toggles">
      <label><input type="checkbox" data-layer-toggle="layer-base" checked> base_link path</label>
      <label><input type="checkbox" data-layer-toggle="layer-tool" checked> cutter coverage</label>
      <label><input type="checkbox" data-layer-toggle="layer-connectors" checked> connectors</label>
      <label><input type="checkbox" data-layer-toggle="layer-pivots" checked> pivots</label>
      <label><input type="checkbox" data-layer-toggle="layer-left-wheel" checked> left wheel track</label>
      <label><input type="checkbox" data-layer-toggle="layer-right-wheel" checked> right wheel track</label>
      <label><input type="checkbox" data-layer-toggle="layer-turn-anchors" checked> turn anchors</label>
      <label><input type="checkbox" data-layer-toggle="layer-swept-footprint" checked> swept footprint</label>
      <label><input type="checkbox" data-layer-toggle="layer-debug" checked> F2C debug</label>
      <label><input type="checkbox" data-layer-toggle="layer-unsafe" checked> unsafe markers</label>
      <label><input id="show-safety-footprint" type="checkbox" checked> safety footprint</label>
    </div>
    <div class="legend">
      <span><i class="key key-fill"></i> mow area</span>
      <span><i class="key" style="border-color:#2563eb"></i> cutter coverage</span>
      <span><i class="key key-dashed" style="border-color:#111827"></i> base_link</span>
      <span><i class="key key-dashed" style="border-color:#f97316"></i> connector</span>
      <span><i class="key-dot" style="background:#7c3aed"></i> pivot sample</span>
      <span><i class="key key-dashed" style="border-color:#0f766e"></i> left wheel</span>
      <span><i class="key key-dashed" style="border-color:#9333ea"></i> right wheel</span>
      <span><i class="key-dot" style="background:#ea580c"></i> pivot anchor</span>
      <span><i class="key-dot" style="background:#0891b2"></i> target anchor</span>
      <span><i class="key-dot"></i> unsafe sample</span>
      <span><i class="key key-dashed" style="border-color:#be185d"></i> segment gap</span>
    </div>
	    <p class="preview-counts">Timeline poses: <code>{int(preview_counts.get("timeline", 0))}</code>.
	    Physical collisions: <code>{int(preview_counts.get("unsafe_physical", 0))}</code>.
	    Safety-margin only: <code>{int(preview_counts.get("unsafe_margin_only", 0))}</code>.
	    Segment gaps: <code>{int(preview_counts.get("segment_gaps", 0))}</code>.
	    Maneuvers: <code>{int(preview_counts.get("maneuvers", 0))}</code>.
	    Reverse turn poses: <code>{int(preview_counts.get("reverse_turn_poses", 0))}</code>.
	    Cutting-disabled poses: <code>{int(preview_counts.get("cutting_disabled_poses", 0))}</code>.</p>
    <div class="svg-wrap">{svg}</div>
  </section>
  <table>{metric_rows}</table>
  <script id="simulation-data" type="application/json">{preview_json}</script>
  {viewer_script}
</body>
</html>
"""
    (run_dir / "plan.html").write_text(report, encoding="utf-8")


def cmd_validate_map(args: argparse.Namespace) -> None:
    model = parse_map(pathlib.Path(args.map).resolve(), repair_rings=args.repair_rings)
    summary = {
        "map": str(model.path),
        "active_mow_areas": len(model.lawns),
        "active_obstacles": len(model.obstacles),
        "docking_stations": len(model.docking_stations),
        "lawns": [
            {
                "id": lawn.area.id,
                "name": lawn.area.name,
                "area_m2": polygon_area(lawn),
                "holes": [hole.id for hole in lawn.holes],
            }
            for lawn in model.lawns
        ],
    }
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(f"Map OK: {summary['map']}")
        print(f"  mow areas: {summary['active_mow_areas']}")
        print(f"  obstacles: {summary['active_obstacles']}")
        for lawn in summary["lawns"]:
            print(f"  - {lawn['name']}: {lawn['area_m2']:.2f} m^2, holes={len(lawn['holes'])}")


def cmd_render(args: argparse.Namespace) -> None:
    run_dir = pathlib.Path(args.run_dir).resolve()
    render_all_reports(run_dir)
    print(f"Rendered {run_dir / 'plan.svg'} and {run_dir / 'plan.html'}")
    print(f"Display report: {run_dir / 'plan.html'}")


def write_batch_report(output_root: pathlib.Path, results: list[dict[str, Any]]) -> pathlib.Path:
    rows = []
    for result in results:
        map_name = html.escape(pathlib.Path(result.get("map", "")).name)
        if not result.get("ok"):
            rows.append(
                "<tr>"
                f"<td>{map_name}</td>"
                "<td>failed</td>"
                f"<td colspan=\"5\">{html.escape(result.get('error', 'unknown error'))}</td>"
                "</tr>"
            )
            continue

        run_dir = pathlib.Path(result["run_dir"])
        metrics_path = run_dir / "metrics.json"
        try:
            metrics = read_json(metrics_path)
        except SystemExit:
            metrics = {}
        rel_report = html.escape(f"{run_dir.name}/plan.html")
        coverage = metrics.get("coverage", {})
        safety = metrics.get("safety", {})
        path = metrics.get("path", {})
        rows.append(
            "<tr>"
            f"<td>{map_name}</td>"
            "<td>ok</td>"
            f"<td><a href=\"{rel_report}\">open report</a></td>"
            f"<td>{metrics.get('swath_count', 0)}</td>"
            f"<td>{path.get('total_length_m', 0.0):.2f}</td>"
            f"<td>{coverage.get('coverage_percent', 0.0):.2f}</td>"
            f"<td>{safety.get('unsafe_footprint_samples', 0)}</td>"
            "</tr>"
        )

    report = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Coverage Lab Batch</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 24px; color: #1f2933; }}
    h1 {{ font-size: 22px; }}
    table {{ border-collapse: collapse; margin-top: 16px; }}
    th, td {{ border: 1px solid #ddd; padding: 7px 9px; text-align: left; }}
    th {{ background: #f3f4f6; }}
  </style>
</head>
<body>
  <h1>Coverage Lab Batch</h1>
  <table>
    <thead>
      <tr>
        <th>Map</th>
        <th>Status</th>
        <th>Report</th>
        <th>Swaths</th>
        <th>Length m</th>
        <th>Coverage %</th>
        <th>Unsafe Footprint Samples</th>
      </tr>
    </thead>
    <tbody>
      {"".join(rows)}
    </tbody>
  </table>
</body>
</html>
"""
    report_path = output_root / "batch.html"
    report_path.write_text(report, encoding="utf-8")
    return report_path


def cmd_batch(args: argparse.Namespace) -> None:
    maps_dir = pathlib.Path(args.maps).resolve()
    if not maps_dir.is_dir():
        die(f"--maps must be a directory: {maps_dir}")
    output_root = pathlib.Path(args.output).resolve() if args.output else DEFAULT_RUNS_DIR / dt.datetime.now().strftime("%Y%m%d-%H%M%S-batch")
    output_root.mkdir(parents=True, exist_ok=True)
    results = []
    for map_file in sorted(maps_dir.glob("*.json")):
        child_args = copy.copy(args)
        child_args.map = str(map_file)
        child_args.output = str(output_root / map_file.stem)
        child_args.area_index = None
        child_args.suppress_display = True
        try:
            run_dir = plan_map(child_args)
            results.append({"map": str(map_file), "run_dir": str(run_dir), "ok": True})
        except SystemExit as exc:
            results.append({"map": str(map_file), "ok": False, "error": str(exc)})
    write_json(output_root / "batch_summary.json", results)
    report_path = write_batch_report(output_root, results)
    print(f"Wrote batch summary: {output_root / 'batch_summary.json'}")
    print(f"Display report: {report_path}")


def cmd_convert_kml(args: argparse.Namespace) -> None:
    if args.output and args.output_dir:
        die("--output and --output-dir cannot be used together")
    if args.kml_dir and args.output:
        die("--output cannot be used with --kml-dir; use --output-dir")

    output_dir = pathlib.Path(args.output_dir).expanduser().resolve() if args.output_dir else DEFAULT_GOOGLE_EARTH_MAP_DIR
    output_paths: list[pathlib.Path] = []

    if args.kml:
        kml_path = pathlib.Path(args.kml).expanduser().resolve()
        output_path = pathlib.Path(args.output).expanduser().resolve() if args.output else output_path_for_kml(kml_path, output_dir)
        output_paths.append(convert_kml_file(kml_path, output_path, args.default_type, args.simplify_tolerance_m))

    if args.kml_dir:
        kml_dir = pathlib.Path(args.kml_dir).expanduser().resolve()
        if not kml_dir.is_dir():
            die(f"--kml-dir must be a directory: {kml_dir}")
        kml_files = sorted(kml_dir.glob("*.kml"))
        if not kml_files:
            die(f"--kml-dir contains no .kml files: {kml_dir}")
        for kml_path in kml_files:
            output_paths.append(
                convert_kml_file(kml_path, output_path_for_kml(kml_path, output_dir), args.default_type, args.simplify_tolerance_m)
            )

    if args.plan:
        ensure_plan_output_visible_to_docker(output_paths)

    for output_path in output_paths:
        print(f"Converted KML map: {output_path}")
        print(f"Next command: tools/coverage_lab/bin/coverage_lab plan --map {printable_repo_path(output_path)}")
        if args.plan:
            print(f"Plan map: {output_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline Fields2Cover lab for OpenMower map.json files.")
    sub = parser.add_subparsers(dest="command", required=True)

    validate = sub.add_parser("validate-map", help="validate an OpenMower map.json file")
    validate.add_argument("--map", required=True, help="path to OpenMower map.json")
    validate.add_argument("--repair-rings", action="store_true", help="append closing points before validation")
    validate.add_argument("--json", action="store_true", help="print JSON summary")
    validate.set_defaults(func=cmd_validate_map)

    plan = sub.add_parser("plan", help="run Fields2Cover and write evaluation artifacts")
    plan.add_argument("--map", required=True, help="path to OpenMower map.json")
    plan.add_argument("--config", default=str(DEFAULT_CONFIG), help="planner config YAML")
    plan.add_argument("--output", help="output run directory")
    plan.add_argument("--area-index", type=int, help="plan one active mow area by index")
    plan.add_argument("--repair-rings", action="store_true", help="append closing points before planning")
    plan.add_argument("--skip-comparisons", action="store_true", help="only generate the primary Mowrator profile")
    plan.add_argument(
        "--swath-angle-mode",
        choices=["fixed", "best_n_swath", "best_swath_length"],
        help="override fields2cover.swath_angle.mode from the config",
    )
    plan.add_argument("--swath-angle-degrees", type=float, help="fixed swath angle override in degrees")
    plan.add_argument("--outline-count", type=int, help="override outline/headland count from the config")
    plan.add_argument("--outline-clearance-m", type=float, help="override outline centerline clearance from boundaries")
    plan.add_argument(
        "--turn-forward-extent-spacing-factor",
        type=float,
        help="override forward U-turn fallback extent as a factor of stripe spacing",
    )
    plan.add_argument(
        "--turn-planner",
        choices=["wheel_anchor", "lattice", "three_point", "forward_u_turn"],
        help="override the primary stripe-to-stripe turn planner",
    )
    plan.add_argument(
        "--turn-cutting-mode",
        choices=["off", "forward_only", "all"],
        help="override whether turn poses count as cutting coverage",
    )
    plan.add_argument("--turn-reverse-enabled", dest="turn_reverse_enabled", action="store_true", default=None)
    plan.add_argument("--turn-reverse-disabled", dest="turn_reverse_enabled", action="store_false", default=None)
    plan.set_defaults(func=plan_map)

    render = sub.add_parser("render", help="re-render SVG/HTML for an existing run directory")
    render.add_argument("--run-dir", required=True, help="run directory containing planpath_compat.json")
    render.set_defaults(func=cmd_render)

    batch = sub.add_parser("batch", help="plan every .json map in a directory")
    batch.add_argument("--maps", required=True, help="directory containing map JSON files")
    batch.add_argument("--config", default=str(DEFAULT_CONFIG), help="planner config YAML")
    batch.add_argument("--output", help="batch output directory")
    batch.add_argument("--repair-rings", action="store_true", help="append closing points before planning")
    batch.add_argument("--skip-comparisons", action="store_true", help="only generate the primary Mowrator profile")
    batch.add_argument(
        "--swath-angle-mode",
        choices=["fixed", "best_n_swath", "best_swath_length"],
        help="override fields2cover.swath_angle.mode from the config",
    )
    batch.add_argument("--swath-angle-degrees", type=float, help="fixed swath angle override in degrees")
    batch.add_argument("--outline-count", type=int, help="override outline/headland count from the config")
    batch.add_argument("--outline-clearance-m", type=float, help="override outline centerline clearance from boundaries")
    batch.add_argument(
        "--turn-forward-extent-spacing-factor",
        type=float,
        help="override forward U-turn fallback extent as a factor of stripe spacing",
    )
    batch.add_argument(
        "--turn-planner",
        choices=["wheel_anchor", "lattice", "three_point", "forward_u_turn"],
        help="override the primary stripe-to-stripe turn planner",
    )
    batch.add_argument(
        "--turn-cutting-mode",
        choices=["off", "forward_only", "all"],
        help="override whether turn poses count as cutting coverage",
    )
    batch.add_argument("--turn-reverse-enabled", dest="turn_reverse_enabled", action="store_true", default=None)
    batch.add_argument("--turn-reverse-disabled", dest="turn_reverse_enabled", action="store_false", default=None)
    batch.set_defaults(func=cmd_batch)

    convert = sub.add_parser("convert-kml", help="convert Google Earth KML polygons into coverage-lab map JSON")
    kml_source = convert.add_mutually_exclusive_group(required=True)
    kml_source.add_argument("--kml", help="single Google Earth .kml file to convert")
    kml_source.add_argument("--kml-dir", help="directory of Google Earth .kml files to convert")
    convert.add_argument("--output", help="output JSON file for a single --kml input")
    convert.add_argument(
        "--output-dir",
        default=None,
        help="output directory for converted JSON maps",
    )
    convert.add_argument(
        "--default-type",
        choices=["mow", "obstacle", "nav"],
        help="area type to use when a KML Placemark name has no mow:/obstacle:/nav: prefix",
    )
    convert.add_argument(
        "--simplify-tolerance-m",
        type=float,
        default=0.0,
        help="remove near-collinear polygon points within this meter tolerance during conversion",
    )
    convert.add_argument("--plan", action="store_true", help="plan each converted map after conversion")
    convert.set_defaults(func=cmd_convert_kml)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
