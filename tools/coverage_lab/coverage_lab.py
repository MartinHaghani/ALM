#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import datetime as dt
import html
import json
import math
import pathlib
import shutil
import sys
from dataclasses import dataclass
from typing import Any, Iterable


LAB_DIR = pathlib.Path(__file__).resolve().parent
REPO_DIR = LAB_DIR.parents[1]
DEFAULT_CONFIG = LAB_DIR / "configs" / "default.yaml"
DEFAULT_RUNS_DIR = LAB_DIR / "runs"
EPSILON = 1e-6


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
    result = []
    for i in range(swaths.size()):
        swath = swaths.at(i)
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
        return f2c.RP_Boustrophedon().genSortedSwaths(swaths, variant)
    if name == "snake":
        return f2c.RP_Snake().genSortedSwaths(swaths, variant)
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


def plan_one_lawn(lawn: Lawn, area_index: int, config: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any], list[str]]:
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

    compat_paths: list[dict[str, Any]] = []
    debug: dict[str, Any] = {
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
    }

    if outline_count > 0:
        try:
            headland_layers = const_hl.generateHeadlandSwaths(cells, tool_width, outline_count, True)
            for layer_i in range(len(headland_layers)):
                layer_rings = f2c_cells_rings(headland_layers[layer_i])
                for ring_i, ring in enumerate(layer_rings):
                    path_poses = points_to_path(ring, "headland")
                    debug["headland_rings"].append(
                        {
                            "layer": layer_i,
                            "ring": ring_i,
                            "points": [{"x": x, "y": y} for x, y in ring],
                        }
                    )
                    compat_paths.append(
                        {
                            "is_outline": True,
                            "area_index": area_index,
                            "area_id": lawn.area.id,
                            "label": f"{lawn.area.name} headland {layer_i + 1}.{ring_i + 1}",
                            "path": {"frame_id": config.get("frame_id", "map"), "poses": path_poses},
                        }
                    )
        except Exception as exc:
            warnings.append(f"Fields2Cover headland swath generation failed: {exc}")

    try:
        mainland = const_hl.generateHeadlands(cells, headland_width) if headland_width > 0.0 else cells
    except Exception as exc:
        warnings.append(f"Fields2Cover headland area generation failed; using full cell for fill: {exc}")
        mainland = cells

    debug["mainland_rings"] = [
        [{"x": x, "y": y} for x, y in ring]
        for ring in f2c_cells_rings(mainland)
    ]

    if mainland.size() == 0:
        warnings.append("Fields2Cover produced no mainland after headland offset")
        return compat_paths, debug, warnings

    swaths = generate_swaths(mainland, config, f2c)
    swaths = sort_swaths(swaths, config, f2c)
    debug["swaths"] = f2c_swaths_to_json(swaths)

    if swaths.size() == 0:
        warnings.append("Fields2Cover produced no fill swaths")
        return compat_paths, debug, warnings

    footprint = config.get("footprint") or []
    robot_width = max((float(p[1]) for p in footprint), default=tool_width / 2.0) - min(
        (float(p[1]) for p in footprint), default=-tool_width / 2.0
    )
    robot = f2c.Robot(max(robot_width, tool_width), tool_width)
    robot.setMinTurningRadius(float(config["fields2cover"].get("min_turning_radius", 1.0)))
    robot.setMaxDiffCurv(float(config["fields2cover"].get("max_diff_curv", 0.1)))

    path = f2c.PP_PathPlanning().planPath(robot, swaths, planner_turn(config, f2c))
    sample_step = float(config["evaluation"].get("path_sample_step_m", 0.1))
    if sample_step > 0:
        path.discretize(sample_step)
    poses = f2c_path_to_poses(path, f2c)
    compat_paths.append(
        {
            "is_outline": False,
            "area_index": area_index,
            "area_id": lawn.area.id,
            "label": f"{lawn.area.name} fill",
            "path": {"frame_id": config.get("frame_id", "map"), "poses": poses},
        }
    )
    return compat_paths, debug, warnings


def iter_pose_segments(paths: list[dict[str, Any]], coverage_only: bool = False) -> Iterable[tuple[dict[str, Any], dict[str, Any]]]:
    for path in paths:
        poses = path.get("path", {}).get("poses", [])
        for i in range(len(poses) - 1):
            a, b = poses[i], poses[i + 1]
            if coverage_only and not (
                path.get("is_outline") or a.get("section") == "swath" or b.get("section") == "swath"
            ):
                continue
            yield a, b


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


def path_length(paths: list[dict[str, Any]], predicate: Any | None = None) -> float:
    total = 0.0
    for path in paths:
        poses = path.get("path", {}).get("poses", [])
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


def compute_metrics(model: OpenMowerMap, compat: dict[str, Any], debug: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    paths = compat["paths"]
    sample_res = float(config["evaluation"].get("coverage_sample_resolution_m", 0.1))
    all_points: list[tuple[float, float]] = []
    for lawn in model.lawns:
        all_points.extend(lawn.area.outline)
        for hole in lawn.holes:
            all_points.extend(hole.outline)
    for path in paths:
        all_points.extend((p["x"], p["y"]) for p in path.get("path", {}).get("poses", []))
    min_x, min_y, max_x, max_y = bbox(all_points)
    pad = max(float(config["tool_width"]), 0.5)
    min_x, min_y, max_x, max_y = min_x - pad, min_y - pad, max_x + pad, max_y + pad

    coverage_segments = [
        ((a["x"], a["y"]), (b["x"], b["y"]))
        for a, b in iter_pose_segments(paths, coverage_only=True)
    ]
    radius = float(config["tool_width"]) / 2.0
    mowable = covered = overcut = 0
    x_steps = max(1, int(math.ceil((max_x - min_x) / sample_res)))
    y_steps = max(1, int(math.ceil((max_y - min_y) / sample_res)))
    for ix in range(x_steps):
        x = min_x + (ix + 0.5) * sample_res
        for iy in range(y_steps):
            y = min_y + (iy + 0.5) * sample_res
            point = (x, y)
            inside = any(point_in_lawn(point, lawn) for lawn in model.lawns)
            hit = any(point_segment_distance(point, a, b) <= radius for a, b in coverage_segments)
            if inside:
                mowable += 1
                if hit:
                    covered += 1
            elif hit:
                overcut += 1

    footprint = config.get("footprint") or []
    stride = max(1, int(config["evaluation"].get("footprint_sample_stride", 1)))
    footprint_samples = unsafe_footprint_samples = obstacle_pose_samples = 0
    for path in paths:
        for i, pose in enumerate(path.get("path", {}).get("poses", [])):
            if i % stride:
                continue
            footprint_samples += 1
            corners = transform_footprint(pose, footprint)
            safe = all(any(point_in_lawn(corner, lawn) for lawn in model.lawns) for corner in corners)
            if not safe:
                unsafe_footprint_samples += 1
            if any(point_in_ring((pose["x"], pose["y"]), obs.outline) for obs in model.obstacles):
                obstacle_pose_samples += 1

    area_unit = sample_res * sample_res
    swath_count = sum(len(area.get("swaths", [])) for area in debug.get("areas", []))
    headland_count = sum(1 for p in paths if p.get("is_outline"))
    connector_length = path_length(
        paths,
        lambda path, a, b: (not path.get("is_outline")) and (a.get("section") != "swath" or b.get("section") != "swath"),
    )
    total_length = path_length(paths)
    return {
        "schema": "open_mower.coverage_lab.metrics.v0",
        "map": str(model.path),
        "area_count": len(model.lawns),
        "obstacle_count": len(model.obstacles),
        "swath_count": swath_count,
        "headland_path_count": headland_count,
        "path": {
            "total_length_m": total_length,
            "connector_length_m": connector_length,
        },
        "coverage": {
            "sample_resolution_m": sample_res,
            "mowable_area_m2": mowable * area_unit,
            "covered_area_m2": covered * area_unit,
            "uncovered_area_m2": max(0, mowable - covered) * area_unit,
            "overcut_area_m2": overcut * area_unit,
            "coverage_percent": (100.0 * covered / mowable) if mowable else 0.0,
            "shoelace_mowable_area_m2": sum(polygon_area(lawn) for lawn in model.lawns),
        },
        "safety": {
            "footprint_samples": footprint_samples,
            "unsafe_footprint_samples": unsafe_footprint_samples,
            "obstacle_pose_samples": obstacle_pose_samples,
        },
    }


def make_run_dir(map_path: pathlib.Path, output: pathlib.Path | None) -> pathlib.Path:
    if output:
        return output
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    return DEFAULT_RUNS_DIR / f"{stamp}-{map_path.stem}"


def plan_map(args: argparse.Namespace) -> pathlib.Path:
    map_path = pathlib.Path(args.map).resolve()
    config = load_config(pathlib.Path(args.config).resolve())
    model = parse_map(map_path, repair_rings=args.repair_rings)
    run_dir = make_run_dir(map_path, pathlib.Path(args.output).resolve() if args.output else None)
    run_dir.mkdir(parents=True, exist_ok=True)

    compat = {
        "schema": "open_mower.planpath_compat.v0",
        "frame_id": config.get("frame_id", "map"),
        "source_map": str(map_path),
        "paths": [],
    }
    debug = {"schema": "open_mower.coverage_lab.debug_geometry.v0", "areas": [], "warnings": []}

    selected = range(len(model.lawns)) if args.area_index is None else [args.area_index]
    for area_index in selected:
        if area_index < 0 or area_index >= len(model.lawns):
            die(f"area index out of range: {area_index}")
        paths, area_debug, warnings = plan_one_lawn(model.lawns[area_index], area_index, config)
        compat["paths"].extend(paths)
        debug["areas"].append(area_debug)
        debug["warnings"].extend([f"area {area_index}: {w}" for w in warnings])

    metrics = compute_metrics(model, compat, debug, config)
    metrics["warnings"] = debug["warnings"]

    shutil.copyfile(map_path, run_dir / "source_map_snapshot.json")
    write_json(run_dir / "planpath_compat.json", compat)
    write_json(run_dir / "planning_debug.json", debug)
    write_json(run_dir / "metrics.json", metrics)
    render_run(run_dir)
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
    colors = ["#5b3cc4", "#00897b", "#e76f51", "#2f6fdd", "#7d4e24"]
    for i, path in enumerate(compat.get("paths", [])):
        pts = [(p["x"], p["y"]) for p in path.get("path", {}).get("poses", [])]
        if len(pts) < 2:
            continue
        color = "#234f1e" if path.get("is_outline") else colors[i % len(colors)]
        width = "2.2" if path.get("is_outline") else "2.8"
        elements.append(svg_polyline(pts, tx, fill="none", stroke=color, stroke_width=width, opacity="0.95"))
        sx, sy = tx(*pts[0])
        ex, ey = tx(*pts[-1])
        elements.append(f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="4" fill="{color}" />')
        elements.append(f'<circle cx="{ex:.2f}" cy="{ey:.2f}" r="4" fill="#ffffff" stroke="{color}" stroke-width="2" />')
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
    .svg-wrap {{ border: 1px solid #d7d7d0; overflow: auto; background: #f7f7f2; }}
    table {{ border-collapse: collapse; margin-top: 20px; max-width: 1100px; }}
    th, td {{ border: 1px solid #ddd; padding: 6px 8px; text-align: left; vertical-align: top; }}
    th {{ width: 220px; background: #f3f4f6; }}
    code {{ background: #f3f4f6; padding: 1px 4px; }}
  </style>
</head>
<body>
  <h1>{title}</h1>
  <p>Source map: <code>{html.escape(str(source.get("id", "source_map_snapshot.json")))}</code></p>
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
    render_run(run_dir)
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
    plan.set_defaults(func=plan_map)

    render = sub.add_parser("render", help="re-render SVG/HTML for an existing run directory")
    render.add_argument("--run-dir", required=True, help="run directory containing planpath_compat.json")
    render.set_defaults(func=cmd_render)

    batch = sub.add_parser("batch", help="plan every .json map in a directory")
    batch.add_argument("--maps", required=True, help="directory containing map JSON files")
    batch.add_argument("--config", default=str(DEFAULT_CONFIG), help="planner config YAML")
    batch.add_argument("--output", help="batch output directory")
    batch.add_argument("--repair-rings", action="store_true", help="append closing points before planning")
    batch.set_defaults(func=cmd_batch)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
