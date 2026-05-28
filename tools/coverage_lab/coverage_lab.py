#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import datetime as dt
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


def sample_polyline_tool_poses(
    points: list[tuple[float, float]],
    section: str,
    sample_step: float,
    direction: str = "forward",
) -> list[dict[str, Any]]:
    if not points:
        return []
    if len(points) == 1:
        return [{"x": points[0][0], "y": points[0][1], "yaw": 0.0, "section": section, "direction": direction}]

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
                }
            )
    if not poses:
        poses.append({"x": points[0][0], "y": points[0][1], "yaw": 0.0, "section": section, "direction": direction})
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
    if base_poses and same_pose(base_poses[-1], base_pose):
        return
    base_poses.append(base_pose)
    tool_poses.append(tool_pose)


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


def build_zero_turn_fill_path(
    lawn: Lawn,
    area_index: int,
    swaths_json: list[dict[str, Any]],
    config: dict[str, Any],
) -> dict[str, Any] | None:
    offset = tool_center_offset(config)
    sample_step = float(config["evaluation"].get("path_sample_step_m", 0.1))
    base_poses: list[dict[str, Any]] = []
    tool_poses: list[dict[str, Any]] = []

    for swath in swaths_json:
        swath_points = [(p["x"], p["y"]) for p in swath.get("points", [])]
        swath_tool_poses = sample_polyline_tool_poses(swath_points, "swath", sample_step)
        if not swath_tool_poses:
            continue
        swath_base_poses = poses_to_base_link(swath_tool_poses, offset)

        if not base_poses:
            for base_pose, tool_pose in zip(swath_base_poses, swath_tool_poses):
                append_pose_pair(base_poses, tool_poses, base_pose, tool_pose)
            continue

        current = base_poses[-1]
        target = swath_base_poses[0]
        connector_len = math.hypot(float(target["x"]) - float(current["x"]), float(target["y"]) - float(current["y"]))
        connector_yaw = (
            math.atan2(float(target["y"]) - float(current["y"]), float(target["x"]) - float(current["x"]))
            if connector_len > EPSILON
            else float(target.get("yaw", current.get("yaw", 0.0)))
        )

        for base_pose in sample_pivot_base_poses(current, connector_yaw, config):
            append_pose_pair(base_poses, tool_poses, base_pose, tool_pose_from_base_pose(base_pose, offset))

        pivoted_current = dict(base_poses[-1])
        pivoted_current["yaw"] = connector_yaw
        for base_pose in sample_line_base_poses(pivoted_current, target, connector_yaw, sample_step, "connector"):
            append_pose_pair(base_poses, tool_poses, base_pose, tool_pose_from_base_pose(base_pose, offset))

        at_target = dict(base_poses[-1])
        at_target["x"] = target["x"]
        at_target["y"] = target["y"]
        at_target["yaw"] = connector_yaw
        for base_pose in sample_pivot_base_poses(at_target, float(target.get("yaw", connector_yaw)), config):
            append_pose_pair(base_poses, tool_poses, base_pose, tool_pose_from_base_pose(base_pose, offset))

        for base_pose, tool_pose in zip(swath_base_poses, swath_tool_poses):
            append_pose_pair(base_poses, tool_poses, base_pose, tool_pose)

    if not base_poses:
        return None

    return make_path_record(
        is_outline=False,
        area_index=area_index,
        lawn=lawn,
        label=f"{lawn.area.name} fill",
        frame_id=config.get("frame_id", "map"),
        base_poses=base_poses,
        tool_poses=tool_poses,
    )


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


def plan_one_lawn_profiles(lawn: Lawn, area_index: int, config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    try:
        import fields2cover as f2c
    except ImportError:
        die("Fields2Cover is not installed; run through tools/coverage_lab/bin/coverage_lab")

    warnings: list[str] = []
    cells = make_f2c_cells(lawn, f2c)
    const_hl = f2c.HG_Const_gen()
    tool_width = float(config["tool_width"])
    outline_count = int(config.get("outline_count", 0))
    outline_offset = float(config.get("outline_offset", 0.0))
    headland_width = max(0.0, outline_count * tool_width + outline_offset)

    base_debug: dict[str, Any] = {
        "area_index": area_index,
        "area_id": lawn.area.id,
        "area_name": lawn.area.name,
        "input_boundary": [{"x": x, "y": y} for x, y in lawn.area.outline],
        "input_obstacles": [
            {"id": h.id, "name": h.name, "points": [{"x": x, "y": y} for x, y in h.outline]}
            for h in lawn.holes
        ],
        "headland_width_m": headland_width,
        "headland_rings": [],
        "mainland_rings": [],
        "swaths": [],
        "tool_center_offset": list(tool_center_offset(config)),
    }

    if outline_count > 0:
        try:
            headland_layers = const_hl.generateHeadlandSwaths(cells, tool_width, outline_count, True)
            for layer_i in range(len(headland_layers)):
                layer_rings = f2c_cells_rings(headland_layers[layer_i])
                for ring_i, ring in enumerate(layer_rings):
                    base_debug["headland_rings"].append(
                        {
                            "layer": layer_i,
                            "ring": ring_i,
                            "points": [{"x": x, "y": y} for x, y in ring],
                        }
                    )
        except Exception as exc:
            warnings.append(f"Fields2Cover headland swath generation failed: {exc}")

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
    fill_path = build_zero_turn_fill_path(lawn, area_index, swaths_json, config)
    if fill_path:
        result_profiles[primary]["paths"].append(fill_path)
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
    coverage_sections = {"headland", "swath"}
    return path.get("is_outline") or (a.get("section") in coverage_sections and b.get("section") in coverage_sections)


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
    unsafe_straight_connector_samples = unsafe_pivot_samples = 0
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
                if len(unsafe_samples) < marker_limit:
                    unsafe_samples.append(
                        {
                            "x": pose["x"],
                            "y": pose["y"],
                            "yaw": pose.get("yaw", 0.0),
                            "section": pose.get("section", "unknown"),
                            "path_label": path.get("label", ""),
                        }
                    )
            if any(point_in_ring((pose["x"], pose["y"]), obs.outline) for obs in metric_obstacles):
                obstacle_pose_samples += 1

    area_unit = sample_res * sample_res
    swath_count = sum(len(area.get("swaths", [])) for area in debug.get("areas", []))
    headland_count = sum(1 for p in paths if p.get("is_outline"))
    connector_length = path_length(
        paths,
        lambda path, a, b: (not path.get("is_outline")) and (a.get("section") != "swath" or b.get("section") != "swath"),
        source="base",
    )
    straight_connector_length = path_length(
        paths,
        lambda path, a, b: (not path.get("is_outline")) and a.get("section") == "connector" and b.get("section") == "connector",
        source="base",
    )
    total_length = path_length(paths, source="base")
    tool_length = path_length(paths, source="tool")
    coverage = {
        "sample_resolution_m": sample_res,
        "mowable_area_m2": mowable * area_unit,
        "covered_area_m2": covered * area_unit,
        "uncovered_area_m2": max(0, mowable - covered) * area_unit,
        "overcut_area_m2": overcut * area_unit,
        "coverage_percent": (100.0 * covered / mowable) if mowable else 0.0,
        "shoelace_mowable_area_m2": sum(polygon_area(lawn) for lawn in metric_lawns),
    }
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
        "safety": {
            "footprint_samples": footprint_samples,
            "unsafe_footprint_samples": unsafe_footprint_samples,
            "obstacle_pose_samples": obstacle_pose_samples,
            "unsafe_straight_connector_samples": unsafe_straight_connector_samples,
            "unsafe_pivot_samples": unsafe_pivot_samples,
            "unsafe_samples": unsafe_samples,
        },
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
    shutil.copyfile(map_path, run_dir / "source_map_snapshot.json")
    write_json(run_dir / "planpath_compat.json", compat)
    write_json(run_dir / "planning_debug.json", debug)
    write_json(run_dir / "metrics.json", metrics)


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


def render_run(run_dir: pathlib.Path) -> None:
    source = read_json(run_dir / "source_map_snapshot.json")
    compat = read_json(run_dir / "planpath_compat.json")
    debug = read_json(run_dir / "planning_debug.json")
    metrics = read_json(run_dir / "metrics.json")
    temp_map = run_dir / "source_map_snapshot.json"
    model = parse_map(temp_map, repair_rings=True)

    all_points: list[tuple[float, float]] = []
    for lawn in model.lawns:
        all_points.extend(lawn.area.outline)
        for hole in lawn.holes:
            all_points.extend(hole.outline)
    for path in compat.get("paths", []):
        all_points.extend((p["x"], p["y"]) for p in path.get("path", {}).get("poses", []))
        all_points.extend((p["x"], p["y"]) for p in path.get("tool_path", {}).get("poses", []))
    min_x, min_y, max_x, max_y = bbox(all_points)
    width_m = max(max_x - min_x, 1.0)
    height_m = max(max_y - min_y, 1.0)
    scale = min(90.0, max(20.0, 1200.0 / max(width_m, height_m)))
    pad = 30.0
    svg_w = width_m * scale + 2 * pad
    svg_h = height_m * scale + 2 * pad

    def tx(x: float, y: float) -> tuple[float, float]:
        return (pad + (x - min_x) * scale, pad + (max_y - y) * scale)

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w:.0f}" height="{svg_h:.0f}" viewBox="0 0 {svg_w:.0f} {svg_h:.0f}">',
        '<rect width="100%" height="100%" fill="#f7f7f2" />',
    ]
    for lawn in model.lawns:
        elements.append(svg_polygon(lawn.area.outline, tx, fill="#d7efd2", stroke="#1d7a32", stroke_width="2"))
        for hole in lawn.holes:
            elements.append(svg_polygon(hole.outline, tx, fill="#ffd5d5", stroke="#b42323", stroke_width="2"))
    for area in debug.get("areas", []):
        for ring in area.get("mainland_rings", []):
            pts = [(p["x"], p["y"]) for p in ring]
            elements.append(svg_polyline(pts, tx, fill="none", stroke="#f59f00", stroke_width="1.5", stroke_dasharray="5 5"))
        for swath in area.get("swaths", []):
            pts = [(p["x"], p["y"]) for p in swath.get("points", [])]
            elements.append(svg_polyline(pts, tx, fill="none", stroke="#7b8794", stroke_width="1", opacity="0.75"))
    tool_colors = ["#2f6fdd", "#00897b", "#7c3aed", "#c2410c", "#0f766e"]
    for i, path in enumerate(compat.get("paths", [])):
        base_pts = [(p["x"], p["y"]) for p in path.get("path", {}).get("poses", [])]
        tool_pts = [(p["x"], p["y"]) for p in path.get("tool_path", {}).get("poses", [])]
        color = "#234f1e" if path.get("is_outline") else tool_colors[i % len(tool_colors)]
        if len(tool_pts) >= 2:
            width = "2.1" if path.get("is_outline") else "2.8"
            elements.append(svg_polyline(tool_pts, tx, fill="none", stroke=color, stroke_width=width, opacity="0.95"))
        if len(base_pts) >= 2:
            elements.append(
                svg_polyline(
                    base_pts,
                    tx,
                    fill="none",
                    stroke="#111827",
                    stroke_width="1.2",
                    stroke_dasharray="4 4",
                    opacity="0.65",
                )
            )
        pts = tool_pts or base_pts
        if len(pts) >= 2:
            sx, sy = tx(*pts[0])
            ex, ey = tx(*pts[-1])
            elements.append(f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="4" fill="{color}" />')
            elements.append(f'<circle cx="{ex:.2f}" cy="{ey:.2f}" r="4" fill="#ffffff" stroke="{color}" stroke-width="2" />')
    for sample in metrics.get("safety", {}).get("unsafe_samples", []):
        sx, sy = tx(float(sample["x"]), float(sample["y"]))
        elements.append(f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="3.5" fill="#dc2626" opacity="0.85" />')
    elements.append("</svg>")
    svg = "\n".join(elements)
    (run_dir / "plan.svg").write_text(svg, encoding="utf-8")

    title = html.escape(run_dir.name)
    metric_rows = "\n".join(
        f"<tr><th>{html.escape(k)}</th><td>{html.escape(json.dumps(v, sort_keys=True))}</td></tr>"
        for k, v in metrics.items()
        if k not in {"schema"}
    )
    report = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{title}</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 24px; color: #1f2933; }}
    h1 {{ font-size: 22px; }}
    h2 {{ font-size: 18px; margin-top: 24px; }}
    .svg-wrap {{ border: 1px solid #d7d7d0; overflow: auto; background: #f7f7f2; }}
    table {{ border-collapse: collapse; margin-top: 20px; max-width: 1100px; }}
    th, td {{ border: 1px solid #ddd; padding: 6px 8px; text-align: left; vertical-align: top; }}
    th {{ width: 220px; background: #f3f4f6; }}
    code {{ background: #f3f4f6; padding: 1px 4px; }}
  </style>
</head>
<body>
  <h1>{title}</h1>
  <p>Profile: <code>{html.escape(str(metrics.get("profile", compat.get("profile", "unknown"))))}</code></p>
  <p>Source map: <code>{html.escape(str(source.get("id", "source_map_snapshot.json")))}</code></p>
  {comparison_rows(run_dir, metrics)}
  <div class="svg-wrap">{svg}</div>
  <table>{metric_rows}</table>
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
