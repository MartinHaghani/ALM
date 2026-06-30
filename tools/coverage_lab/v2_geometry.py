"""V2 coverage-planner geometry conditioning and macro-zone classification.

This module is intentionally independent of Fields2Cover. It backs the first
V2 prototype from ``docs/COVERAGE_PLANNER_V2_PROTOTYPE_PLAN.md``: turn an
OpenMower lawn polygon into mower-scale classified geometry before any route
or maneuver planning starts.

The first version favours explainable geometry over cleverness:

* simplify the recorded boundary at a mower-scale tolerance;
* compute a yaw-independent footprint-safe drivable region;
* classify drivable slabs into main-body, corridor, and pocket zones;
* make boundary bands, removed wrinkles, and unreachable notches visible.

>>> opts = make_options({}, "normal", None, None, None)
>>> round(opts["simplify_tolerance_m"], 2)
0.1
>>> bounds = oriented_bounds(box_polygon(4.0, 1.0))
>>> round(bounds["length_m"], 1), round(bounds["width_m"], 1)
(4.0, 1.0)
"""
from __future__ import annotations

import html
import json
import math
import pathlib
import warnings
from typing import Any, Iterable

import lab_geometry
from shapely.affinity import rotate
from shapely.geometry import (
    GeometryCollection,
    LineString,
    MultiLineString,
    MultiPolygon,
    Point,
    Polygon,
    box,
)
from shapely.ops import nearest_points, unary_union


EPS = 1e-9

TASK_TYPE_ORDER = {
    "body_core": 0,
    "corridor": 1,
    "dead_end_corridor": 2,
    "notch": 3,
    "artifact": 4,
}

TASK_TYPE_LABELS = {
    "body_core": "BODY",
    "corridor": "CORRIDOR",
    "dead_end_corridor": "DEAD END",
    "notch": "NOTCH",
    "artifact": "ARTIFACT",
}

TASK_TYPE_COLORS = {
    "body_core": "#22c55e",
    "corridor": "#ef4444",
    "dead_end_corridor": "#06b6d4",
    "notch": "#facc15",
    "artifact": "#94a3b8",
}


def box_polygon(length: float, width: float) -> Polygon:
    """Small doctest helper."""
    return Polygon([(0.0, 0.0), (length, 0.0), (length, width), (0.0, width)])


def _float_config(config: dict[str, Any], key: str, default: float) -> float:
    try:
        return float(config.get(key, default))
    except (TypeError, ValueError):
        return default


def _bool_config(config: dict[str, Any], key: str, default: bool) -> bool:
    value = config.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _footprint_dimensions(config: dict[str, Any]) -> dict[str, float]:
    footprint = config.get("footprint") or []
    if len(footprint) < 3:
        tool_width = _float_config(config, "tool_width", 0.4)
        return {
            "length_m": tool_width,
            "width_m": tool_width,
            "safety_length_m": tool_width,
            "safety_width_m": tool_width,
        }
    xs = [float(p[0]) for p in footprint]
    ys = [float(p[1]) for p in footprint]
    margin = _float_config(config, "safety_margin_m", 0.0)
    return {
        "length_m": max(xs) - min(xs),
        "width_m": max(ys) - min(ys),
        "safety_length_m": max(xs) - min(xs) + 2.0 * margin,
        "safety_width_m": max(ys) - min(ys) + 2.0 * margin,
    }


def base_link_all_yaw_radius(config: dict[str, Any]) -> float:
    """Return circumscribed safety-footprint radius around ``base_link``.

    V2's executable pose convention is ``base_link``. If a base_link point is
    this far inside the boundary, the full safety footprint can rotate at any
    yaw without crossing the boundary.

    >>> round(base_link_all_yaw_radius({
    ...     "footprint": [[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]],
    ...     "safety_margin_m": 0.05,
    ... }), 3)
    0.953
    """
    margin = _float_config(config, "safety_margin_m", 0.0)
    footprint = config.get("footprint") or []
    if len(footprint) < 3:
        return max(_float_config(config, "tool_width", 0.4) / 2.0 + margin, margin)
    xs = [float(p[0]) for p in footprint]
    ys = [float(p[1]) for p in footprint]
    corners = (
        (min(xs) - margin, min(ys) - margin),
        (min(xs) - margin, max(ys) + margin),
        (max(xs) + margin, min(ys) - margin),
        (max(xs) + margin, max(ys) + margin),
    )
    return max(math.hypot(x, y) for x, y in corners)


def mower_model(config: dict[str, Any]) -> dict[str, Any]:
    dims = _footprint_dimensions(config)
    footprint = [[float(p[0]), float(p[1])] for p in (config.get("footprint") or [])]
    offset = config.get("tool_center_offset") or [0.0, 0.0]
    return {
        "footprint": footprint,
        "tool_width_m": _float_config(config, "tool_width", 0.4),
        "tool_center_offset": [float(offset[0]), float(offset[1])],
        "safety_margin_m": _float_config(config, "safety_margin_m", 0.0),
        "footprint_length_m": dims["length_m"],
        "footprint_width_m": dims["width_m"],
        "safety_footprint_length_m": dims["safety_length_m"],
        "safety_footprint_width_m": dims["safety_width_m"],
        "base_link_all_yaw_radius_m": base_link_all_yaw_radius(config),
        "tool_center_all_yaw_radius_m": lab_geometry.footprint_disk_radius(config),
    }


def make_options(
    config: dict[str, Any],
    conditioning_profile: str = "normal",
    simplify_tolerance_m: float | None = None,
    min_zone_area_m2: float | None = None,
    corridor_width_factor: float | None = None,
    drivable_boundary_clearance_m: float | None = None,
) -> dict[str, Any]:
    """Return effective V2 conditioning options.

    The default tolerance is tied to tool width so the prototype scales with
    the configured mower instead of a hard-coded map-cleanup magic number.
    """
    profile = str(conditioning_profile or "normal").lower()
    if profile not in {"conservative", "normal", "aggressive"}:
        raise ValueError(f"unsupported conditioning profile: {conditioning_profile}")

    model = mower_model(config)
    tool_width = float(model["tool_width_m"])
    safety_width = float(model["safety_footprint_width_m"])
    profile_scale = {
        "conservative": 0.12,
        "normal": 0.25,
        "aggressive": 0.50,
    }[profile]
    tolerance = simplify_tolerance_m
    if tolerance is None:
        tolerance = max(0.03, min(0.25, tool_width * profile_scale))
    min_area = min_zone_area_m2
    if min_area is None:
        min_area = max(0.5, model["footprint_length_m"] * model["footprint_width_m"] * 2.0)
    width_factor = corridor_width_factor if corridor_width_factor is not None else 2.5
    boundary_clearance = drivable_boundary_clearance_m
    if boundary_clearance is None:
        boundary_clearance = _float_config(
            config,
            "v2_drivable_boundary_clearance_m",
            max(0.10, float(model["safety_margin_m"])),
        )
    width_sample_step = max(0.05, _float_config(config, "v2_width_sample_step_m", 0.20))
    body_width_factor = max(1.0, _float_config(config, "v2_body_width_factor", 2.5))
    neck_widening_ratio = max(1.0, _float_config(config, "v2_neck_widening_ratio", 1.5))
    neck_min_area = max(0.0, _float_config(config, "v2_neck_min_area_m2", 1.0))
    task_min_branch_length = max(0.0, _float_config(config, "v2_task_min_branch_length_m", 0.80))
    task_corridor_aspect = max(1.0, _float_config(config, "v2_task_corridor_aspect_ratio", 2.5))
    task_notch_max_area = max(0.0, _float_config(config, "v2_task_notch_max_area_m2", 1.5))
    task_notch_max_depth = max(0.0, _float_config(config, "v2_task_notch_max_depth_m", 1.5))
    task_artifact_max_area = max(0.0, _float_config(config, "v2_task_artifact_max_area_m2", 0.25))
    task_portal_body_width_ratio = max(1.01, _float_config(config, "v2_task_portal_body_width_ratio", 1.35))
    task_dead_end_min_depth = max(0.0, _float_config(config, "v2_task_dead_end_min_depth_m", 1.8))
    task_dead_end_min_area = max(0.0, _float_config(config, "v2_task_dead_end_min_area_m2", 1.0))
    task_region_merge_overlap = min(1.0, max(0.0, _float_config(config, "v2_task_region_merge_overlap_ratio", 0.45)))
    task_portal_cut_buffer = max(0.0, _float_config(config, "v2_task_portal_cut_buffer_m", 0.03))
    task_portal_search_enabled = _bool_config(config, "v2_task_portal_search_enabled", True)
    task_portal_search = max(0.0, _float_config(config, "v2_task_portal_search_m", 0.80))
    task_portal_search_step = max(0.03, _float_config(config, "v2_task_portal_search_step_m", 0.10))
    task_pocket_min_visible_area = max(0.0, _float_config(config, "v2_task_pocket_min_visible_area_m2", 0.25))
    task_false_notch_suppression_enabled = _bool_config(
        config, "v2_task_false_notch_suppression_enabled", True
    )
    task_false_notch_max_area = max(0.0, _float_config(config, "v2_task_false_notch_max_area_m2", 0.60))
    task_false_notch_max_area_ratio = max(
        0.0, _float_config(config, "v2_task_false_notch_max_area_ratio", 0.03)
    )
    task_false_notch_max_depth = max(0.0, _float_config(config, "v2_task_false_notch_max_depth_m", 1.60))
    task_false_notch_portal_depth_ratio = max(
        0.0, _float_config(config, "v2_task_false_notch_min_portal_width_depth_ratio", 0.85)
    )
    task_path_enabled = _bool_config(config, "v2_task_path_enabled", True)
    task_path_spacing_factor = max(0.10, _float_config(config, "v2_task_path_spacing_factor", 0.90))
    task_path_min_pass_length = max(0.0, _float_config(config, "v2_task_path_min_pass_length_m", 0.35))
    task_path_body_min_pass_length = max(0.0, _float_config(config, "v2_task_path_body_min_pass_length_m", 2.0))
    task_path_sample_step = max(0.03, _float_config(config, "v2_task_path_sample_step_m", 0.10))
    task_path_stripe_boundary_clearance_factor = max(
        0.0, _float_config(config, "v2_task_path_stripe_boundary_clearance_factor", 1.0)
    )
    task_path_body_pocket_enabled = _bool_config(config, "v2_task_path_body_pocket_enabled", True)
    task_path_body_pocket_outline_band = max(
        0.0, _float_config(config, "v2_task_path_body_pocket_outline_band_m", 0.50)
    )
    task_path_body_pocket_min_area = max(
        0.0, _float_config(config, "v2_task_path_body_pocket_min_area_m2", 0.50)
    )
    task_path_body_pocket_min_width_factor = max(
        0.0, _float_config(config, "v2_task_path_body_pocket_min_width_factor", 0.60)
    )
    task_path_body_pocket_max_strokes = max(
        1, int(_float_config(config, "v2_task_path_body_pocket_max_strokes", 4))
    )
    task_path_reverse_out_enabled = _bool_config(config, "v2_task_path_reverse_out_enabled", True)
    task_path_reverse_cutting_enabled = _bool_config(config, "v2_task_path_reverse_cutting_enabled", False)
    task_path_terminal_angle_tolerance = max(
        0.0, _float_config(config, "v2_task_path_terminal_angle_tolerance_deg", 35.0)
    )
    task_path_side_lane_min_length_factor = max(
        0.0, _float_config(config, "v2_task_path_side_lane_min_length_factor", 0.55)
    )
    task_path_notch_max_strokes = max(1, int(_float_config(config, "v2_task_path_notch_max_strokes", 3)))
    task_path_service_axis_ray_pad = max(0.0, _float_config(config, "v2_task_path_service_axis_ray_pad_m", 0.50))
    task_path_axis_candidate_scoring_enabled = _bool_config(
        config, "v2_task_path_axis_candidate_scoring_enabled", True
    )
    task_path_axis_candidate_sweep = max(
        0.0, _float_config(config, "v2_task_path_axis_candidate_sweep_degrees", 25.0)
    )
    task_path_axis_candidate_step = max(
        1.0, _float_config(config, "v2_task_path_axis_candidate_step_degrees", 12.5)
    )
    task_path_axis_candidate_max_count = max(
        4, int(_float_config(config, "v2_task_path_axis_candidate_max_count", 24))
    )
    task_annotation_cut_buffer = max(0.0, _float_config(config, "v2_task_annotation_cut_buffer_m", 0.03))
    task_ownership_enabled = _bool_config(config, "v2_task_ownership_enabled", True)
    task_ownership_portal_band = max(0.0, _float_config(config, "v2_task_ownership_portal_band_m", 0.90))
    task_ownership_short_body_pass = max(0.0, _float_config(config, "v2_task_ownership_short_body_pass_m", 1.20))
    task_ownership_axis_alignment = max(0.0, _float_config(config, "v2_task_ownership_axis_alignment_deg", 35.0))
    task_path_dead_end_turnaround_enabled = _bool_config(config, "v2_task_path_dead_end_turnaround_enabled", True)
    task_path_turnaround_yaw_step = max(1.0, _float_config(config, "v2_task_path_turnaround_yaw_step_deg", 10.0))
    task_path_turnaround_clearance = max(0.0, _float_config(config, "v2_task_path_turnaround_clearance_m", 0.05))
    task_path_turnaround_terminal_band = max(0.0, _float_config(config, "v2_task_path_turnaround_terminal_band_m", 0.80))
    return {
        "conditioning_profile": profile,
        "simplify_tolerance_m": float(tolerance),
        "min_zone_area_m2": float(min_area),
        "corridor_width_factor": float(width_factor),
        "corridor_width_threshold_m": float(width_factor) * safety_width,
        "drivable_boundary_clearance_m": float(boundary_clearance),
        "width_sample_step_m": float(width_sample_step),
        "body_width_factor": float(body_width_factor),
        "body_width_threshold_m": float(body_width_factor) * safety_width,
        "neck_widening_ratio": float(neck_widening_ratio),
        "neck_min_area_m2": float(neck_min_area),
        "zone_slice_step_m": max(0.6, float(model["footprint_length_m"])),
        "tiny_feature_area_m2": max(0.02, tool_width * tool_width * 0.2),
        "task_min_branch_length_m": float(task_min_branch_length),
        "task_corridor_aspect_ratio": float(task_corridor_aspect),
        "task_notch_max_area_m2": float(task_notch_max_area),
        "task_notch_max_depth_m": float(task_notch_max_depth),
        "task_artifact_max_area_m2": float(task_artifact_max_area),
        "task_portal_body_width_ratio": float(task_portal_body_width_ratio),
        "task_dead_end_min_depth_m": float(task_dead_end_min_depth),
        "task_dead_end_min_area_m2": float(task_dead_end_min_area),
        "task_region_merge_overlap_ratio": float(task_region_merge_overlap),
        "task_portal_cut_buffer_m": float(task_portal_cut_buffer),
        "task_portal_search_enabled": bool(task_portal_search_enabled),
        "task_portal_search_m": float(task_portal_search),
        "task_portal_search_step_m": float(task_portal_search_step),
        "task_pocket_min_visible_area_m2": float(task_pocket_min_visible_area),
        "task_false_notch_suppression_enabled": bool(task_false_notch_suppression_enabled),
        "task_false_notch_max_area_m2": float(task_false_notch_max_area),
        "task_false_notch_max_area_ratio": float(task_false_notch_max_area_ratio),
        "task_false_notch_max_depth_m": float(task_false_notch_max_depth),
        "task_false_notch_min_portal_width_depth_ratio": float(task_false_notch_portal_depth_ratio),
        "task_path_enabled": bool(task_path_enabled),
        "task_path_spacing_factor": float(task_path_spacing_factor),
        "task_path_min_pass_length_m": float(task_path_min_pass_length),
        "task_path_body_min_pass_length_m": float(task_path_body_min_pass_length),
        "task_path_sample_step_m": float(task_path_sample_step),
        "task_path_stripe_boundary_clearance_factor": float(task_path_stripe_boundary_clearance_factor),
        "task_path_body_pocket_enabled": bool(task_path_body_pocket_enabled),
        "task_path_body_pocket_outline_band_m": float(task_path_body_pocket_outline_band),
        "task_path_body_pocket_min_area_m2": float(task_path_body_pocket_min_area),
        "task_path_body_pocket_min_width_factor": float(task_path_body_pocket_min_width_factor),
        "task_path_body_pocket_max_strokes": int(task_path_body_pocket_max_strokes),
        "task_path_reverse_out_enabled": bool(task_path_reverse_out_enabled),
        "task_path_reverse_cutting_enabled": bool(task_path_reverse_cutting_enabled),
        "task_path_terminal_angle_tolerance_deg": float(task_path_terminal_angle_tolerance),
        "task_path_side_lane_min_length_factor": float(task_path_side_lane_min_length_factor),
        "task_path_notch_max_strokes": int(task_path_notch_max_strokes),
        "task_path_service_axis_ray_pad_m": float(task_path_service_axis_ray_pad),
        "task_path_axis_candidate_scoring_enabled": bool(task_path_axis_candidate_scoring_enabled),
        "task_path_axis_candidate_sweep_degrees": float(task_path_axis_candidate_sweep),
        "task_path_axis_candidate_step_degrees": float(task_path_axis_candidate_step),
        "task_path_axis_candidate_max_count": int(task_path_axis_candidate_max_count),
        "task_annotation_cut_buffer_m": float(task_annotation_cut_buffer),
        "task_ownership_enabled": bool(task_ownership_enabled),
        "task_ownership_portal_band_m": float(task_ownership_portal_band),
        "task_ownership_short_body_pass_m": float(task_ownership_short_body_pass),
        "task_ownership_axis_alignment_deg": float(task_ownership_axis_alignment),
        "task_path_dead_end_turnaround_enabled": bool(task_path_dead_end_turnaround_enabled),
        "task_path_turnaround_yaw_step_deg": float(task_path_turnaround_yaw_step),
        "task_path_turnaround_clearance_m": float(task_path_turnaround_clearance),
        "task_path_turnaround_terminal_band_m": float(task_path_turnaround_terminal_band),
    }


def _closed_ring(points: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    ring = [(float(x), float(y)) for x, y in points]
    if ring and math.hypot(ring[0][0] - ring[-1][0], ring[0][1] - ring[-1][1]) > EPS:
        ring.append(ring[0])
    return ring


def _make_polygon(
    outline: Iterable[tuple[float, float]],
    holes: Iterable[Iterable[tuple[float, float]]] | None = None,
) -> Polygon | MultiPolygon:
    outer = _closed_ring(outline)
    interior_rings: list[list[tuple[float, float]]] = []
    for hole in holes or ():
        ring = _closed_ring(hole)
        if len(ring) >= 4:
            interior_rings.append(ring)
    polygon = Polygon(outer, interior_rings)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    return _polygonal_only(polygon)


def _polygonal_only(geom: Any) -> Polygon | MultiPolygon:
    if geom.is_empty:
        return MultiPolygon([])
    if isinstance(geom, (Polygon, MultiPolygon)):
        return geom
    if isinstance(geom, GeometryCollection):
        polys = [g for g in geom.geoms if isinstance(g, Polygon) and g.area > EPS]
        return unary_union(polys) if polys else MultiPolygon([])
    return MultiPolygon([])


def _iter_polygons(geom: Any) -> list[Polygon]:
    if geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom] if geom.area > EPS else []
    if isinstance(geom, MultiPolygon):
        return [g for g in geom.geoms if g.area > EPS]
    if isinstance(geom, GeometryCollection):
        return [g for g in geom.geoms if isinstance(g, Polygon) and g.area > EPS]
    return []


def _clean_polygonal(geom: Any) -> Polygon | MultiPolygon:
    if geom.is_empty:
        return MultiPolygon([])
    cleaned = geom.buffer(0)
    return _polygonal_only(cleaned)


def _condition_polygon(raw: Polygon | MultiPolygon, tolerance_m: float) -> Polygon | MultiPolygon:
    conditioned = raw
    if tolerance_m > EPS:
        conditioned = conditioned.simplify(tolerance_m, preserve_topology=True)
    return _clean_polygonal(conditioned)


def _round_point(x: float, y: float) -> list[float]:
    return [round(float(x), 4), round(float(y), 4)]


def geometry_to_json(geom: Any) -> dict[str, Any]:
    polygons = []
    for poly in _iter_polygons(geom):
        outer = [_round_point(x, y) for x, y in poly.exterior.coords]
        holes = [[_round_point(x, y) for x, y in interior.coords] for interior in poly.interiors]
        polygons.append({"outer": outer, "holes": holes})
    return {"type": "MultiPolygon", "polygons": polygons}


def area_m2(geom: Any) -> float:
    return float(getattr(geom, "area", 0.0) or 0.0)


def oriented_bounds(poly: Polygon) -> dict[str, float]:
    if poly.is_empty or poly.area <= EPS:
        return {"length_m": 0.0, "width_m": 0.0, "angle_deg": 0.0}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        rect = poly.minimum_rotated_rectangle
    coords = list(rect.exterior.coords)
    if len(coords) < 5:
        minx, miny, maxx, maxy = poly.bounds
        return {
            "length_m": float(max(maxx - minx, maxy - miny)),
            "width_m": float(min(maxx - minx, maxy - miny)),
            "angle_deg": 0.0,
        }
    edges: list[tuple[float, float, float]] = []
    for i in range(4):
        x1, y1 = coords[i]
        x2, y2 = coords[i + 1]
        length = math.hypot(x2 - x1, y2 - y1)
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        edges.append((length, angle, i))
    long_length, long_angle, long_idx = max(edges, key=lambda item: item[0])
    width = edges[(long_idx + 1) % 4][0]
    if width > long_length:
        long_length, width = width, long_length
        long_angle += 90.0
    while long_angle <= -90.0:
        long_angle += 180.0
    while long_angle > 90.0:
        long_angle -= 180.0
    return {
        "length_m": float(long_length),
        "width_m": float(width),
        "angle_deg": float(long_angle),
    }


def _zone_record(
    area_index: int,
    component_index: int,
    zone_index: int,
    kind: str,
    poly: Polygon,
    local_width_m: float | None,
    reason: str,
) -> dict[str, Any]:
    bounds = oriented_bounds(poly)
    label_point = poly.representative_point()
    return {
        "id": f"area-{area_index}-zone-{zone_index}",
        "component_index": component_index,
        "kind": kind,
        "preliminary": True,
        "display_index": zone_index + 1,
        "label": f"Z{zone_index + 1}",
        "label_point": _round_point(float(label_point.x), float(label_point.y)),
        "area_m2": area_m2(poly),
        "oriented_bounds": bounds,
        "local_width_m": float(local_width_m if local_width_m is not None else bounds["width_m"]),
        "reason": reason,
        "geometry": geometry_to_json(poly),
    }


def _classify_component_zones(
    component: Polygon,
    area_index: int,
    component_index: int,
    options: dict[str, Any],
    next_zone_index: int,
) -> tuple[list[dict[str, Any]], int]:
    if component.area <= EPS:
        return [], next_zone_index

    min_zone_area = float(options["min_zone_area_m2"])
    corridor_threshold = float(options["corridor_width_threshold_m"])
    slice_step = float(options["zone_slice_step_m"])
    bounds = oriented_bounds(component)

    if component.area < min_zone_area:
        return [
            _zone_record(
                area_index,
                component_index,
                next_zone_index,
                "pocket",
                component,
                bounds["width_m"],
                "drivable component below min_zone_area_m2",
            )
        ], next_zone_index + 1

    angle = bounds["angle_deg"]
    rotated = rotate(component, -angle, origin=(0.0, 0.0), use_radians=False)
    minx, miny, maxx, maxy = rotated.bounds
    if maxx - minx <= EPS:
        return [
            _zone_record(
                area_index,
                component_index,
                next_zone_index,
                "pocket",
                component,
                bounds["width_m"],
                "drivable component has near-zero long-axis extent",
            )
        ], next_zone_index + 1

    body_parts: list[Polygon] = []
    corridor_parts: list[Polygon] = []
    x = minx
    pad = max(slice_step, corridor_threshold, 1.0)
    while x < maxx - EPS:
        x2 = min(maxx, x + slice_step)
        slab = box(x, miny - pad, x2, maxy + pad)
        section = rotated.intersection(slab)
        for part in _iter_polygons(section):
            _, py0, _, py1 = part.bounds
            local_width = py1 - py0
            if local_width <= corridor_threshold:
                corridor_parts.append(part)
            else:
                body_parts.append(part)
        x = x2

    zones: list[dict[str, Any]] = []

    def add_zone_parts(kind: str, parts: list[Polygon], reason: str) -> None:
        nonlocal next_zone_index
        if not parts:
            return
        union = _clean_polygonal(unary_union(parts))
        for part in _iter_polygons(union):
            world_part = rotate(part, angle, origin=(0.0, 0.0), use_radians=False)
            world_part = _clean_polygonal(world_part.intersection(component))
            for world_poly in _iter_polygons(world_part):
                ob = oriented_bounds(world_poly)
                final_kind = kind
                final_reason = reason
                if world_poly.area < min_zone_area:
                    final_kind = "pocket"
                    final_reason = "zone fragment below min_zone_area_m2"
                elif ob["width_m"] <= corridor_threshold and ob["length_m"] >= max(1.5 * ob["width_m"], slice_step):
                    final_kind = "corridor"
                    final_reason = "final zone oriented width below corridor threshold"
                elif kind == "corridor" and ob["length_m"] < max(1.5 * ob["width_m"], slice_step):
                    final_kind = "pocket"
                    final_reason = "narrow zone too short to treat as corridor"
                zones.append(
                    _zone_record(
                        area_index,
                        component_index,
                        next_zone_index,
                        final_kind,
                        world_poly,
                        ob["width_m"],
                        final_reason,
                    )
                )
                next_zone_index += 1

    add_zone_parts("main_body", body_parts, "local cross-section wider than corridor threshold")
    add_zone_parts("corridor", corridor_parts, "local cross-section below corridor threshold")

    if not zones:
        kind = "corridor" if bounds["width_m"] <= corridor_threshold else "main_body"
        zones.append(
            _zone_record(
                area_index,
                component_index,
                next_zone_index,
                kind,
                component,
                bounds["width_m"],
                "whole component classified from oriented width fallback",
            )
        )
        next_zone_index += 1

    return zones, next_zone_index


def classify_zones(
    drivable: Polygon | MultiPolygon,
    area_index: int,
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    zones: list[dict[str, Any]] = []
    zone_index = 0
    for component_index, component in enumerate(sorted(_iter_polygons(drivable), key=lambda p: p.area, reverse=True)):
        component_zones, zone_index = _classify_component_zones(
            component,
            area_index,
            component_index,
            options,
            zone_index,
        )
        zones.extend(component_zones)
    return zones


def sample_width_field(
    drivable: Polygon | MultiPolygon,
    area_index: int,
    config: dict[str, Any],
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    """Sample local boundary clearance inside the drivable region.

    The estimate is intentionally simple and dependency-free: twice the
    distance to the drivable boundary. It is a diagnostic field, not a final
    decomposition algorithm.
    """
    model = mower_model(config)
    step = float(options["width_sample_step_m"])
    too_tight_threshold = float(model["safety_footprint_width_m"])
    body_threshold = float(options["body_width_threshold_m"])
    samples: list[dict[str, Any]] = []
    sample_index = 0

    for component_index, component in enumerate(sorted(_iter_polygons(drivable), key=lambda p: p.area, reverse=True)):
        if component.area <= EPS:
            continue
        minx, miny, maxx, maxy = component.bounds
        span_x = max(maxx - minx, step)
        span_y = max(maxy - miny, step)
        count_x = max(1, int(math.ceil(span_x / step)))
        count_y = max(1, int(math.ceil(span_y / step)))
        actual_step_x = span_x / count_x
        actual_step_y = span_y / count_y
        boundary = component.boundary
        for ix in range(count_x):
            x = minx + (ix + 0.5) * actual_step_x
            for iy in range(count_y):
                y = miny + (iy + 0.5) * actual_step_y
                point = Point(x, y)
                if not component.covers(point):
                    continue
                clearance = float(point.distance(boundary))
                local_width = 2.0 * clearance
                if local_width < too_tight_threshold:
                    sample_class = "too_tight"
                elif local_width < body_threshold:
                    sample_class = "corridor_width"
                else:
                    sample_class = "body_width"
                samples.append(
                    {
                        "id": f"area-{area_index}-width-sample-{sample_index}",
                        "area_index": area_index,
                        "component_index": component_index,
                        "grid_x": ix,
                        "grid_y": iy,
                        "x": round(float(x), 4),
                        "y": round(float(y), 4),
                        "clearance_m": round(clearance, 4),
                        "local_width_m": round(local_width, 4),
                        "class": sample_class,
                    }
                )
                sample_index += 1
    return samples


def _append_ridge_link(
    ridge_links: list[dict[str, Any]],
    seen_links: set[tuple[str, str]],
    ridge: dict[str, Any],
    other: dict[str, Any],
) -> None:
    pair = tuple(sorted((str(ridge["id"]), str(other["id"]))))
    if pair in seen_links:
        return
    distance = math.hypot(float(ridge["x"]) - float(other["x"]), float(ridge["y"]) - float(other["y"]))
    seen_links.add(pair)
    ridge_links.append(
        {
            "id": f"ridge-link-{len(ridge_links)}",
            "from": ridge["id"],
            "to": other["id"],
            "length_m": round(distance, 4),
            "line": [
                _round_point(float(ridge["x"]), float(ridge["y"])),
                _round_point(float(other["x"]), float(other["y"])),
            ],
        }
    )


def _append_capped_nearest_ridge_links(
    ridge_points: list[dict[str, Any]],
    ridge_links: list[dict[str, Any]],
    seen_links: set[tuple[str, str]],
    options: dict[str, Any],
) -> None:
    step = float(options["width_sample_step_m"])
    near_link_m = max(step * 3.5, 0.75)
    max_degree = 2
    by_component: dict[int, list[dict[str, Any]]] = {}
    for ridge in ridge_points:
        by_component.setdefault(int(ridge["component_index"]), []).append(ridge)
    degree_by_id: dict[str, int] = {str(ridge["id"]): 0 for ridge in ridge_points}
    for link in ridge_links:
        degree_by_id[str(link["from"])] = degree_by_id.get(str(link["from"]), 0) + 1
        degree_by_id[str(link["to"])] = degree_by_id.get(str(link["to"]), 0) + 1
    for component_points in by_component.values():
        candidate_pairs: list[tuple[float, dict[str, Any], dict[str, Any]]] = []
        for idx, ridge in enumerate(component_points):
            for other in component_points[idx + 1 :]:
                distance = math.hypot(float(ridge["x"]) - float(other["x"]), float(ridge["y"]) - float(other["y"]))
                if distance <= near_link_m:
                    candidate_pairs.append((distance, ridge, other))
        for _distance, ridge, other in sorted(candidate_pairs, key=lambda item: item[0]):
            if degree_by_id[str(ridge["id"])] >= max_degree or degree_by_id[str(other["id"])] >= max_degree:
                continue
            before_count = len(ridge_links)
            _append_ridge_link(ridge_links, seen_links, ridge, other)
            if len(ridge_links) != before_count:
                degree_by_id[str(ridge["id"])] += 1
                degree_by_id[str(other["id"])] += 1


def detect_ridge_diagnostics(
    width_samples: list[dict[str, Any]],
    options: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Find local-clearance maxima in the width sample grid."""
    by_key = {
        (int(sample["component_index"]), int(sample["grid_x"]), int(sample["grid_y"])): sample
        for sample in width_samples
    }
    ridge_points: list[dict[str, Any]] = []
    ridge_by_key: dict[tuple[int, int, int], dict[str, Any]] = {}
    step = float(options["width_sample_step_m"])
    prominence = max(0.01, step * 0.05)
    neighbor_offsets = [
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    ]

    for key, sample in by_key.items():
        if sample.get("class") == "too_tight":
            continue
        component_index, grid_x, grid_y = key
        neighbors = [
            by_key[(component_index, grid_x + dx, grid_y + dy)]
            for dx, dy in neighbor_offsets
            if (component_index, grid_x + dx, grid_y + dy) in by_key
        ]
        if not neighbors:
            continue
        clearance = float(sample["clearance_m"])
        max_neighbor = max(float(neighbor["clearance_m"]) for neighbor in neighbors)
        min_neighbor = min(float(neighbor["clearance_m"]) for neighbor in neighbors)
        if clearance + EPS < max_neighbor:
            continue
        if clearance - min_neighbor < prominence:
            continue
        ridge = {
            "id": f"ridge-{len(ridge_points)}",
            "sample_id": sample["id"],
            "area_index": sample["area_index"],
            "component_index": component_index,
            "grid_x": grid_x,
            "grid_y": grid_y,
            "x": sample["x"],
            "y": sample["y"],
            "clearance_m": sample["clearance_m"],
            "local_width_m": sample["local_width_m"],
            "class": sample["class"],
        }
        ridge_points.append(ridge)
        ridge_by_key[key] = ridge

    ridge_links: list[dict[str, Any]] = []
    seen_links: set[tuple[str, str]] = set()
    _append_capped_nearest_ridge_links(ridge_points, ridge_links, seen_links, options)

    return ridge_points, ridge_links


def _adjacency_from_links(
    links: list[dict[str, Any]],
) -> dict[str, list[tuple[str, dict[str, Any]]]]:
    adjacency: dict[str, list[tuple[str, dict[str, Any]]]] = {}
    for link in links:
        from_id = str(link.get("from", ""))
        to_id = str(link.get("to", ""))
        if not from_id or not to_id:
            continue
        adjacency.setdefault(from_id, []).append((to_id, link))
        adjacency.setdefault(to_id, []).append((from_id, link))
    return adjacency


def _edge_key(a: str, b: str) -> tuple[str, str]:
    return tuple(sorted((str(a), str(b))))


def _branch_line_segments(coords: list[tuple[float, float]]) -> list[list[list[float]]]:
    segments = []
    for start, end in zip(coords, coords[1:]):
        segments.append([_round_point(start[0], start[1]), _round_point(end[0], end[1])])
    return segments


def _branch_endpoint_evidence(
    point: dict[str, Any],
    degree: int,
    body_samples: list[dict[str, Any]],
    options: dict[str, Any],
) -> dict[str, Any]:
    step = float(options["width_sample_step_m"])
    body_threshold = float(options["body_width_threshold_m"])
    portal_threshold = body_threshold / float(options["task_portal_body_width_ratio"])
    x = float(point.get("x", 0.0))
    y = float(point.get("y", 0.0))
    width = float(point.get("local_width_m", 0.0))
    nearby_body_distance = None
    search_radius = max(step * 3.0, width * 0.75)
    for sample in body_samples:
        distance = math.hypot(x - float(sample.get("x", 0.0)), y - float(sample.get("y", 0.0)))
        if distance <= search_radius and (nearby_body_distance is None or distance < nearby_body_distance):
            nearby_body_distance = distance
    is_opening = degree >= 3 or width >= portal_threshold or nearby_body_distance is not None
    if degree >= 3:
        reason = "skeleton branch point attaches to multiple directions"
    elif width >= portal_threshold:
        reason = "endpoint width widens toward body-scale clearance"
    elif nearby_body_distance is not None:
        reason = "nearby body-width sample"
    else:
        reason = "terminal ridge endpoint"
    return {
        "point_id": point.get("id", ""),
        "degree": int(degree),
        "local_width_m": round(width, 4),
        "nearby_body_distance_m": round(float(nearby_body_distance), 4) if nearby_body_distance is not None else None,
        "is_opening": bool(is_opening),
        "reason": reason,
    }


def _confidence_for_branch(
    branch_type: str,
    length: float,
    mean_width: float,
    aspect_ratio: float,
    opening_count: int,
    options: dict[str, Any],
) -> float:
    min_length = float(options["task_min_branch_length_m"])
    corridor_aspect = float(options["task_corridor_aspect_ratio"])
    if branch_type == "artifact":
        return 0.35
    if branch_type == "body_core":
        return min(0.9, 0.55 + min(mean_width / max(float(options["body_width_threshold_m"]), EPS), 1.0) * 0.35)
    length_score = min(length / max(min_length, EPS), 1.5) / 1.5
    aspect_score = min(aspect_ratio / max(corridor_aspect, EPS), 1.5) / 1.5
    opening_score = 0.2 if opening_count <= 0 else (0.6 if opening_count == 1 else 0.9)
    if branch_type == "notch":
        return round(min(0.9, 0.45 + 0.25 * length_score + 0.20 * opening_score), 3)
    return round(min(0.95, 0.40 + 0.30 * length_score + 0.25 * aspect_score + 0.20 * opening_score), 3)


def _classify_branch(
    *,
    length: float,
    mean_width: float,
    branch_area: float,
    opening_count: int,
    aspect_ratio: float,
    class_counts: dict[str, int],
    options: dict[str, Any],
) -> tuple[str, str]:
    body_threshold = float(options["body_width_threshold_m"])
    min_branch_length = float(options["task_min_branch_length_m"])
    corridor_aspect = float(options["task_corridor_aspect_ratio"])
    notch_max_area = float(options["task_notch_max_area_m2"])
    notch_max_depth = float(options["task_notch_max_depth_m"])
    artifact_max_area = float(options["task_artifact_max_area_m2"])
    body_votes = int(class_counts.get("body_width", 0))
    sample_count = max(sum(class_counts.values()), 1)
    body_ratio = body_votes / sample_count

    if length <= min_branch_length and branch_area <= artifact_max_area:
        return "artifact", "branch below mower-scale length and area thresholds"
    if mean_width >= body_threshold or body_ratio >= 0.50:
        return "body_core", "wide/high-clearance ridge evidence"
    if aspect_ratio >= corridor_aspect:
        if opening_count >= 2:
            return "corridor", "long narrow branch with two opening endpoints"
        if opening_count == 1:
            if length <= notch_max_depth or branch_area <= notch_max_area:
                return "notch", "one-opening branch is shallow or small enough to service as a notch"
            return "dead_end_corridor", "long narrow branch with one opening and one terminal endpoint"
        return "dead_end_corridor", "long narrow branch with no confirmed body opening"
    if opening_count == 1 and (length <= notch_max_depth or branch_area <= notch_max_area):
        return "notch", "short one-opening branch"
    if mean_width < body_threshold and length >= min_branch_length:
        if length <= notch_max_depth or branch_area <= notch_max_area:
            return "notch", "narrow branch without enough depth for corridor behavior"
        return "dead_end_corridor", "narrow branch without confirmed two-opening corridor evidence"
    if branch_area <= artifact_max_area:
        return "artifact", "small branch area without strong corridor evidence"
    return "body_core", "branch does not have corridor/notch evidence"


def _portal_candidate_for_endpoint(
    *,
    area_index: int,
    branch: dict[str, Any],
    endpoint_index: int,
    endpoint: dict[str, Any],
    point_by_id: dict[str, dict[str, Any]],
    drivable: Polygon | MultiPolygon,
    options: dict[str, Any],
    portal_index: int,
) -> dict[str, Any]:
    point_ids = branch.get("point_ids", [])
    if len(point_ids) < 2:
        center_point = point_by_id[str(endpoint.get("point_id", ""))]
        direction_point = center_point
    elif endpoint_index == 0:
        center_point = point_by_id[str(point_ids[0])]
        direction_point = point_by_id[str(point_ids[1])]
    else:
        center_point = point_by_id[str(point_ids[-1])]
        direction_point = point_by_id[str(point_ids[-2])]
    cx = float(center_point.get("x", 0.0))
    cy = float(center_point.get("y", 0.0))
    dx = float(direction_point.get("x", cx)) - cx
    dy = float(direction_point.get("y", cy)) - cy
    norm = math.hypot(dx, dy)
    if norm <= EPS:
        dx, dy, norm = 1.0, 0.0, 1.0
    nx = -dy / norm
    ny = dx / norm
    width = max(float(endpoint.get("local_width_m", 0.0)), float(branch.get("mean_width_m", 0.0)))
    half_len = max(width * 0.75, float(options["width_sample_step_m"]) * 1.5)
    raw_line = LineString([(cx - nx * half_len, cy - ny * half_len), (cx + nx * half_len, cy + ny * half_len)])
    clipped = raw_line.intersection(drivable)
    accepted = bool(endpoint.get("is_opening")) and not clipped.is_empty
    return {
        "id": f"area-{area_index}-portal-{portal_index}",
        "area_index": area_index,
        "component_index": branch.get("component_index", 0),
        "branch_id": branch.get("id", ""),
        "endpoint_index": endpoint_index,
        "status": "accepted" if accepted else "rejected",
        "reason": endpoint.get("reason", "endpoint evidence") if accepted else "endpoint is not a confirmed opening",
        "width_m": round(width, 4),
        "clearance_m": round(width * 0.5, 4),
        "center": _round_point(cx, cy),
        "line_segments": _line_segments_to_json(clipped if not clipped.is_empty else raw_line),
    }


def _body_core_geometry(
    drivable: Polygon | MultiPolygon,
    width_samples: list[dict[str, Any]],
    options: dict[str, Any],
) -> Polygon | MultiPolygon:
    step = float(options["width_sample_step_m"])
    body_samples = [sample for sample in width_samples if sample.get("class") == "body_width"]
    if body_samples:
        core = unary_union([Point(float(sample["x"]), float(sample["y"])).buffer(step * 0.72) for sample in body_samples])
        core = _clean_polygonal(core.intersection(drivable))
        if area_m2(core) > EPS:
            return core
    body_threshold = float(options["body_width_threshold_m"])
    fallback_parts = [
        component
        for component in _iter_polygons(drivable)
        if oriented_bounds(component)["width_m"] >= body_threshold and component.area >= float(options["min_zone_area_m2"])
    ]
    return _clean_polygonal(unary_union(fallback_parts)) if fallback_parts else MultiPolygon([])


def _appendage_line_segments(poly: Polygon) -> list[list[list[float]]]:
    center = poly.representative_point()
    bounds = oriented_bounds(poly)
    angle = math.radians(bounds["angle_deg"])
    half = max(bounds["length_m"] * 0.5, EPS)
    dx = math.cos(angle) * half
    dy = math.sin(angle) * half
    line = LineString([(center.x - dx, center.y - dy), (center.x + dx, center.y + dy)]).intersection(poly)
    return _line_segments_to_json(line)


def _classify_appendage(
    poly: Polygon,
    options: dict[str, Any],
) -> tuple[str, str, float]:
    bounds = oriented_bounds(poly)
    length = float(bounds["length_m"])
    width = float(bounds["width_m"])
    area = float(poly.area)
    aspect = length / max(width, EPS)
    if area <= float(options["task_artifact_max_area_m2"]):
        return "artifact", "body-core subtraction fragment below artifact area threshold", aspect
    if area <= float(options["task_notch_max_area_m2"]) or length <= float(options["task_notch_max_depth_m"]):
        return "notch", "small body-attached appendage best treated as notch evidence", aspect
    if width <= float(options["body_width_threshold_m"]) and aspect >= float(options["task_corridor_aspect_ratio"]):
        return "dead_end_corridor", "long narrow body-attached appendage", aspect
    if width <= float(options["body_width_threshold_m"]):
        return "notch", "narrow appendage without enough aspect for corridor behavior", aspect
    return "body_core", "wide appendage remains body-like after body-core subtraction", aspect


def _appendage_portal_candidate(
    *,
    area_index: int,
    appendage_id: str,
    appendage: Polygon,
    body_core: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    options: dict[str, Any],
    portal_index: int,
) -> dict[str, Any]:
    if body_core.is_empty:
        center = appendage.representative_point()
        body_point = center
    else:
        body_point, center = nearest_points(body_core, appendage)
    cx = float(center.x)
    cy = float(center.y)
    dx = cx - float(body_point.x)
    dy = cy - float(body_point.y)
    norm = math.hypot(dx, dy)
    if norm <= EPS:
        angle = math.radians(oriented_bounds(appendage)["angle_deg"])
        dx = math.cos(angle)
        dy = math.sin(angle)
        norm = 1.0
    nx = -dy / norm
    ny = dx / norm
    bounds = oriented_bounds(appendage)
    width = max(float(bounds["width_m"]), float(options["width_sample_step_m"]) * 2.0)
    half_len = max(width * 0.70, float(options["width_sample_step_m"]) * 1.5)
    raw_line = LineString([(cx - nx * half_len, cy - ny * half_len), (cx + nx * half_len, cy + ny * half_len)])
    clipped = raw_line.intersection(drivable)
    accepted = not clipped.is_empty
    return {
        "id": f"area-{area_index}-portal-{portal_index}",
        "area_index": area_index,
        "component_index": 0,
        "branch_id": appendage_id,
        "endpoint_index": 0,
        "status": "accepted" if accepted else "rejected",
        "reason": "appendage mouth nearest to body core" if accepted else "appendage mouth cut did not intersect drivable area",
        "width_m": round(width, 4),
        "clearance_m": round(width * 0.5, 4),
        "center": _round_point(cx, cy),
        "line_segments": _line_segments_to_json(clipped if not clipped.is_empty else raw_line),
    }


def _geometry_from_json(geom_json: dict[str, Any]) -> Polygon | MultiPolygon:
    polygons = []
    for poly in geom_json.get("polygons", []):
        outer = poly.get("outer", [])
        if len(outer) < 4:
            continue
        holes = [hole for hole in poly.get("holes", []) if len(hole) >= 4]
        polygons.append(Polygon(outer, holes))
    return _clean_polygonal(unary_union(polygons)) if polygons else MultiPolygon([])


def _label_point_for_geometry_json(geom_json: dict[str, Any]) -> list[float]:
    geom = _geometry_from_json(geom_json)
    if geom.is_empty:
        return []
    point = geom.representative_point()
    return _round_point(float(point.x), float(point.y))


def _largest_polygon(geom: Any) -> Polygon | None:
    polygons = _iter_polygons(geom)
    if not polygons:
        return None
    return max(polygons, key=lambda poly: poly.area)


def _distance_points_for_geometry(geom: Any) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for poly in _iter_polygons(geom):
        points.extend((float(x), float(y)) for x, y in poly.exterior.coords)
        for interior in poly.interiors:
            points.extend((float(x), float(y)) for x, y in interior.coords)
    if not points and not geom.is_empty:
        point = geom.representative_point()
        points.append((float(point.x), float(point.y)))
    return points


def _nearest_body_mouth(
    seed_geom: Polygon | MultiPolygon,
    body_core: Polygon | MultiPolygon,
) -> tuple[Point, Point]:
    if body_core.is_empty or seed_geom.is_empty:
        point = seed_geom.representative_point() if not seed_geom.is_empty else Point(0.0, 0.0)
        return point, point
    body_point, seed_point = nearest_points(body_core, seed_geom)
    if body_point.distance(seed_point) > EPS:
        return body_point, seed_point
    body_center = body_core.representative_point()
    seed_center = seed_geom.representative_point()
    return body_center, seed_center


def _service_portal_candidate_for_seed(
    *,
    area_index: int,
    seed_id: str,
    seed_geom: Polygon | MultiPolygon,
    body_core: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    options: dict[str, Any],
    portal_index: int,
    reason: str,
    seed_source: str = "",
    type_hint: str = "",
) -> tuple[dict[str, Any], LineString]:
    body_point, seed_point = _nearest_body_mouth(seed_geom, body_core)
    cx = float(seed_point.x)
    cy = float(seed_point.y)
    dx = cx - float(body_point.x)
    dy = cy - float(body_point.y)
    norm = math.hypot(dx, dy)
    primary = _largest_polygon(seed_geom)
    bounds = oriented_bounds(primary) if primary is not None else {"width_m": 0.0, "angle_deg": 0.0}
    can_search_toward_body = norm > EPS
    if norm <= EPS:
        angle = math.radians(float(bounds["angle_deg"]))
        dx = math.cos(angle)
        dy = math.sin(angle)
        norm = 1.0
    ux = dx / norm
    uy = dy / norm
    nx = -dy / norm
    ny = dx / norm
    width = max(
        float(bounds["width_m"]),
        float(options["width_sample_step_m"]) * 2.0,
        float(options["body_width_threshold_m"]) * 0.70,
    )
    half_len = max(width * 0.95, float(options["width_sample_step_m"]) * 2.5)
    search_allowed = can_search_toward_body and bool(options.get("task_portal_search_enabled", True))
    if type_hint == "notch" and seed_source != "preliminary_zone_pocket":
        search_allowed = False
    shift_values = [0.0]
    if search_allowed:
        search_m = float(options.get("task_portal_search_m", 0.80))
        search_step = max(float(options.get("task_portal_search_step_m", 0.10)), EPS)
        count = int(math.floor(search_m / search_step))
        shift_values.extend(round((idx + 1) * search_step, 6) for idx in range(count))

    seed_area = area_m2(seed_geom)
    drivable_area = area_m2(drivable)
    candidates = []
    for shift in shift_values:
        shifted_x = cx - ux * shift
        shifted_y = cy - uy * shift
        raw_line = LineString(
            [
                (shifted_x - nx * half_len, shifted_y - ny * half_len),
                (shifted_x + nx * half_len, shifted_y + ny * half_len),
            ]
        )
        clipped = raw_line.intersection(drivable)
        if clipped.is_empty:
            continue
        service_region, extent_source, body_overlap = _grow_service_region_from_seed(
            seed_geom=seed_geom,
            drivable=drivable,
            body_core=body_core,
            portal_line=raw_line,
            options=options,
        )
        service_area = area_m2(service_region)
        seed_coverage = area_m2(service_region.intersection(seed_geom)) / max(seed_area, EPS)
        drivable_ratio = service_area / max(drivable_area, EPS)
        accepted_extent = extent_source == "portal_cut_component"
        # Prefer the largest cleanly isolated service-side region. The shift
        # term breaks ties toward the body-side mouth, which is what operator
        # annotations usually mean by "where the task starts."
        score = (
            1 if accepted_extent else 0,
            round(seed_coverage, 6),
            round(service_area, 6),
            round(shift, 6),
            -round(body_overlap, 6),
            -round(drivable_ratio, 6),
        )
        candidates.append(
            {
                "raw_line": raw_line,
                "clipped": clipped,
                "center": (shifted_x, shifted_y),
                "shift_m": shift,
                "extent_source": extent_source,
                "body_overlap_ratio": body_overlap,
                "service_area_m2": service_area,
                "seed_coverage_ratio": seed_coverage,
                "drivable_ratio": drivable_ratio,
                "accepted_extent": accepted_extent,
                "score": score,
            }
        )
    if candidates:
        accepted_candidates = [candidate for candidate in candidates if candidate["accepted_extent"]]
        best = max(accepted_candidates or candidates, key=lambda candidate: candidate["score"])
        raw_line = best["raw_line"]
        clipped = best["clipped"]
        selected_center = best["center"]
        selected_shift = float(best["shift_m"])
        selected_extent = str(best["extent_source"])
        selected_area = float(best["service_area_m2"])
        selected_seed_coverage = float(best["seed_coverage_ratio"])
        selected_body_overlap = float(best["body_overlap_ratio"])
    else:
        raw_line = LineString([(cx - nx * half_len, cy - ny * half_len), (cx + nx * half_len, cy + ny * half_len)])
        clipped = raw_line.intersection(drivable)
        selected_center = (cx, cy)
        selected_shift = 0.0
        selected_extent = "no_candidate"
        selected_area = 0.0
        selected_seed_coverage = 0.0
        selected_body_overlap = 0.0
    accepted = not clipped.is_empty and not body_core.is_empty
    display_reason = reason
    if selected_shift > EPS:
        display_reason = f"{reason}; selected body-side portal candidate"
    return (
        {
            "id": f"area-{area_index}-portal-{portal_index}",
            "area_index": area_index,
            "component_index": 0,
            "branch_id": seed_id,
            "endpoint_index": 0,
            "status": "accepted" if accepted else "rejected",
            "reason": display_reason if accepted else "service-region mouth cut did not isolate body attachment",
            "width_m": round(width, 4),
            "clearance_m": round(width * 0.5, 4),
            "center": _round_point(selected_center[0], selected_center[1]),
            "line_segments": _line_segments_to_json(clipped if not clipped.is_empty else raw_line),
            "portal_search": {
                "enabled": bool(search_allowed),
                "candidate_count": len(candidates),
                "selected_shift_toward_body_m": round(selected_shift, 4),
                "selected_extent_source": selected_extent,
                "selected_service_area_m2": round(selected_area, 4),
                "selected_seed_coverage_ratio": round(selected_seed_coverage, 4),
                "selected_body_overlap_ratio": round(selected_body_overlap, 4),
                "seed_source": str(seed_source),
                "type_hint": str(type_hint),
            },
        },
        raw_line,
    )


def _grow_service_region_from_seed(
    *,
    seed_geom: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    body_core: Polygon | MultiPolygon,
    portal_line: LineString | None,
    options: dict[str, Any],
) -> tuple[Polygon | MultiPolygon, str, float]:
    fallback = _clean_polygonal(seed_geom.intersection(drivable))
    if fallback.is_empty or portal_line is None or body_core.is_empty:
        body_overlap = area_m2(fallback.intersection(body_core)) / max(area_m2(fallback), EPS)
        return fallback, "seed_only_no_portal_cut", body_overlap

    cut_width = max(float(options["task_portal_cut_buffer_m"]), EPS)
    cut_band = portal_line.buffer(cut_width, cap_style=2, join_style=2)
    split_region = _clean_polygonal(drivable.difference(cut_band))
    components = _iter_polygons(split_region)
    if not components:
        body_overlap = area_m2(fallback.intersection(body_core)) / max(area_m2(fallback), EPS)
        return fallback, "seed_only_cut_removed_all_components", body_overlap

    seed_probe = fallback.buffer(max(float(options["width_sample_step_m"]) * 0.20, cut_width), join_style="round")
    selected = max(components, key=lambda component: area_m2(component.intersection(seed_probe)))
    selected_seed_overlap = area_m2(selected.intersection(seed_probe))
    if selected_seed_overlap <= EPS:
        body_overlap = area_m2(fallback.intersection(body_core)) / max(area_m2(fallback), EPS)
        return fallback, "seed_only_no_component_contains_seed", body_overlap

    grown = _clean_polygonal(selected.union(fallback).intersection(drivable))
    grown_area = area_m2(grown)
    body_overlap_ratio = area_m2(grown.intersection(body_core)) / max(grown_area, EPS)
    drivable_ratio = grown_area / max(area_m2(drivable), EPS)
    if body_overlap_ratio > 0.35 or drivable_ratio > 0.70:
        fallback_overlap = area_m2(fallback.intersection(body_core)) / max(area_m2(fallback), EPS)
        return fallback, "seed_only_cut_did_not_separate_from_body", fallback_overlap
    if grown_area < area_m2(fallback) * 0.35:
        fallback_overlap = area_m2(fallback.intersection(body_core)) / max(area_m2(fallback), EPS)
        return fallback, "seed_only_grown_region_lost_seed_area", fallback_overlap
    return grown, "portal_cut_component", body_overlap_ratio


def _service_measurements(
    region: Polygon | MultiPolygon,
    portal: dict[str, Any] | None,
) -> dict[str, Any]:
    primary = _largest_polygon(region)
    bounds = oriented_bounds(primary) if primary is not None else {"length_m": 0.0, "width_m": 0.0, "angle_deg": 0.0}
    center = portal.get("center", []) if portal else []
    if not portal and primary is not None:
        point = primary.representative_point()
        return {
            "service_depth_m": round(float(bounds["length_m"]), 4),
            "service_width_m": round(float(bounds["width_m"]), 4),
            "oriented_length_m": round(float(bounds["length_m"]), 4),
            "oriented_width_m": round(float(bounds["width_m"]), 4),
            "farthest_point": _round_point(float(point.x), float(point.y)),
            "portal_point": _round_point(float(point.x), float(point.y)),
        }
    if len(center) >= 2:
        px = float(center[0])
        py = float(center[1])
    elif not region.is_empty:
        point = region.representative_point()
        px = float(point.x)
        py = float(point.y)
    else:
        px = py = 0.0
    farthest = (px, py)
    depth = 0.0
    for x, y in _distance_points_for_geometry(region):
        distance = math.hypot(x - px, y - py)
        if distance > depth:
            depth = distance
            farthest = (x, y)
    area = area_m2(region)
    service_width = area / max(depth, EPS) if depth > EPS else float(bounds["width_m"])
    return {
        "service_depth_m": round(depth, 4),
        "service_width_m": round(float(service_width), 4),
        "oriented_length_m": round(float(bounds["length_m"]), 4),
        "oriented_width_m": round(float(bounds["width_m"]), 4),
        "farthest_point": _round_point(farthest[0], farthest[1]),
        "portal_point": _round_point(px, py),
    }


def _classify_service_region(
    *,
    type_hint: str,
    region: Polygon | MultiPolygon,
    service_depth: float,
    service_width: float,
    portal_count: int,
    body_overlap_ratio: float,
    options: dict[str, Any],
) -> tuple[str, str]:
    area = area_m2(region)
    artifact_area = float(options["task_artifact_max_area_m2"])
    dead_end_depth = float(options["task_dead_end_min_depth_m"])
    dead_end_area = float(options["task_dead_end_min_area_m2"])
    notch_area = float(options["task_notch_max_area_m2"])
    notch_depth = float(options["task_notch_max_depth_m"])
    body_width = float(options["body_width_threshold_m"])
    aspect = service_depth / max(service_width, EPS)

    if area <= artifact_area:
        return "artifact", "service region below artifact area threshold"
    if body_overlap_ratio > 0.50 and type_hint not in {"notch", "dead_end_corridor", "corridor"}:
        return "artifact", "candidate overlaps body core too much for a separate task"
    if portal_count >= 2:
        return "corridor", "service region has two meaningful openings"
    if portal_count == 1:
        if type_hint == "notch" and (service_depth <= notch_depth + float(options["width_sample_step_m"]) or area <= notch_area):
            return "notch", "single-entry service region remains shallow enough for notch behavior"
        if service_depth >= dead_end_depth or (
            area >= dead_end_area and service_depth >= max(notch_depth, dead_end_depth * 0.75)
        ):
            return "dead_end_corridor", "single-entry service region is deep or large enough to behave as a dead end"
        return "notch", "single-entry service region is shallow enough to service as a notch"
    if type_hint == "corridor" and aspect >= max(2.0, float(options["task_corridor_aspect_ratio"]) * 0.75):
        return "corridor", "long narrow component without a body-core mouth"
    if service_depth >= dead_end_depth and service_width <= body_width * 1.15:
        return "dead_end_corridor", "narrow service region has dead-end depth without two openings"
    if area <= notch_area or service_depth <= notch_depth:
        return "notch", "service region is small or shallow enough to service as a notch"
    if type_hint == "dead_end_corridor":
        return "dead_end_corridor", "seed evidence already indicates dead-end corridor behavior"
    return "artifact", "service region lacks clear body/corridor/notch evidence"


def _terminal_cap_for_region(
    *,
    task_type: str,
    region: Polygon | MultiPolygon,
    portal: dict[str, Any] | None,
    measurements: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any] | None:
    if task_type != "dead_end_corridor" or region.is_empty:
        return None
    portal_point = measurements.get("portal_point", [])
    farthest = measurements.get("farthest_point", [])
    if len(portal_point) < 2 or len(farthest) < 2:
        return None
    px, py = float(portal_point[0]), float(portal_point[1])
    fx, fy = float(farthest[0]), float(farthest[1])
    dx = fx - px
    dy = fy - py
    depth = math.hypot(dx, dy)
    if depth <= EPS:
        return None
    ux = dx / depth
    uy = dy / depth
    step = float(options["width_sample_step_m"])
    center_point = Point(fx, fy)
    for backoff in (step * 0.25, step * 0.5, step, step * 1.5, step * 2.0):
        candidate = Point(fx - ux * backoff, fy - uy * backoff)
        if region.buffer(EPS).covers(candidate):
            center_point = candidate
            break
    nx = -uy
    ny = ux
    half_len = max(float(measurements.get("service_width_m", 0.0)) * 0.70, step * 1.5)
    raw_line = LineString(
        [
            (center_point.x - nx * half_len, center_point.y - ny * half_len),
            (center_point.x + nx * half_len, center_point.y + ny * half_len),
        ]
    )
    clipped = raw_line.intersection(region)
    return {
        "center": _round_point(float(center_point.x), float(center_point.y)),
        "label_point": _round_point(float(center_point.x), float(center_point.y)),
        "line_segments": _line_segments_to_json(clipped if not clipped.is_empty else raw_line),
        "depth_m": round(depth, 4),
        "portal_id": str(portal.get("id", "")) if portal else "",
    }


def _confidence_for_service_region(
    task_type: str,
    service_depth: float,
    service_width: float,
    portal_count: int,
    options: dict[str, Any],
) -> float:
    aspect = service_depth / max(service_width, EPS)
    return _confidence_for_branch(task_type, service_depth, service_width, aspect, portal_count, options)


def _unique_list(values: Iterable[Any]) -> list[Any]:
    result = []
    seen = set()
    for value in values:
        key = str(value)
        if key in seen:
            continue
        seen.add(key)
        result.append(value)
    return result


def _finalize_service_task(
    *,
    proposal: dict[str, Any],
    portal_by_id: dict[str, dict[str, Any]],
    body_core: Polygon | MultiPolygon,
    options: dict[str, Any],
) -> dict[str, Any]:
    if proposal.get("task_type") == "body_core":
        proposal["classification_reason"] = str(
            proposal.get("classification_reason", "union of body-width samples clipped to drivable area")
        )
        proposal["service_depth_m"] = float(proposal.get("service_depth_m", 0.0))
        proposal["service_width_m"] = float(proposal.get("service_width_m", 0.0))
        evidence = dict(proposal.get("evidence", {}))
        evidence.setdefault("service_area_m2", area_m2(_geometry_from_json(proposal.get("geometry", {}))))
        evidence.setdefault("service_depth_m", 0.0)
        evidence.setdefault("service_width_m", 0.0)
        evidence.setdefault("portal_count", 0)
        evidence.setdefault("body_overlap_ratio", 1.0)
        proposal["evidence"] = evidence
        return proposal

    region = _geometry_from_json(proposal.get("geometry", {}))
    portal_ids = [str(portal_id) for portal_id in proposal.get("entry_portals", [])]
    portal = portal_by_id.get(portal_ids[0]) if portal_ids else None
    measurements = _service_measurements(region, portal)
    body_overlap = area_m2(region.intersection(body_core)) / max(area_m2(region), EPS)
    task_type, reason = _classify_service_region(
        type_hint=str(proposal.get("task_type", "artifact")),
        region=region,
        service_depth=float(measurements["service_depth_m"]),
        service_width=float(measurements["service_width_m"]),
        portal_count=len(portal_ids),
        body_overlap_ratio=body_overlap,
        options=options,
    )
    proposal["task_type"] = task_type
    proposal["confidence"] = _confidence_for_service_region(
        task_type,
        float(measurements["service_depth_m"]),
        float(measurements["service_width_m"]),
        len(portal_ids),
        options,
    )
    proposal["service_depth_m"] = measurements["service_depth_m"]
    proposal["service_width_m"] = measurements["service_width_m"]
    proposal["classification_reason"] = reason
    terminal_cap = _terminal_cap_for_region(
        task_type=task_type,
        region=region,
        portal=portal,
        measurements=measurements,
        options=options,
    )
    if terminal_cap:
        proposal["terminal_cap"] = terminal_cap
    else:
        proposal.pop("terminal_cap", None)

    evidence = dict(proposal.get("evidence", {}))
    evidence.update(
        {
            "service_depth_m": measurements["service_depth_m"],
            "service_width_m": measurements["service_width_m"],
            "service_area_m2": area_m2(region),
            "portal_count": len(portal_ids),
            "body_overlap_ratio": round(body_overlap, 4),
            "reason": reason,
        }
    )
    proposal["evidence"] = evidence
    return proposal


def _merge_service_task_proposals(
    proposals: list[dict[str, Any]],
    portal_candidates: list[dict[str, Any]],
    body_core: Polygon | MultiPolygon,
    area_index: int,
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    portal_by_id = {str(portal.get("id", "")): portal for portal in portal_candidates}
    threshold = float(options["task_region_merge_overlap_ratio"])
    merged: list[dict[str, Any]] = []

    for proposal in proposals:
        proposal_geom = _geometry_from_json(proposal.get("geometry", {}))
        if proposal_geom.is_empty:
            continue
        target: dict[str, Any] | None = None
        if proposal.get("task_type") != "body_core":
            proposal_portals = {str(portal_id) for portal_id in proposal.get("entry_portals", [])}
            for existing in merged:
                if existing.get("task_type") == "body_core":
                    continue
                existing_geom = _geometry_from_json(existing.get("geometry", {}))
                if existing_geom.is_empty:
                    continue
                overlap = area_m2(existing_geom.intersection(proposal_geom)) / max(
                    min(area_m2(existing_geom), area_m2(proposal_geom)), EPS
                )
                existing_portals = {str(portal_id) for portal_id in existing.get("entry_portals", [])}
                if overlap >= threshold or bool(proposal_portals.intersection(existing_portals)):
                    target = existing
                    break
        if target is None:
            merged.append(proposal)
            continue

        target_geom = _geometry_from_json(target.get("geometry", {}))
        target_seed = _geometry_from_json(target.get("seed_geometry", {}))
        proposal_seed = _geometry_from_json(proposal.get("seed_geometry", {}))
        target["geometry"] = geometry_to_json(_clean_polygonal(target_geom.union(proposal_geom)))
        target["seed_geometry"] = geometry_to_json(_clean_polygonal(target_seed.union(proposal_seed)))
        target["line_segments"] = target.get("line_segments", []) + proposal.get("line_segments", [])
        target["source_line_segments"] = _unique_list(
            target.get("source_line_segments", [])
            + target.get("line_segments", [])
            + proposal.get("source_line_segments", [])
            + proposal.get("line_segments", [])
        )
        target["entry_portals"] = _unique_list(target.get("entry_portals", []) + proposal.get("entry_portals", []))
        target["source_ids"] = _unique_list(target.get("source_ids", []) + proposal.get("source_ids", []))
        target["extent_source"] = "merged_service_region"
        target_evidence = dict(target.get("evidence", {}))
        sources = list(target_evidence.get("sources", []))
        sources.extend(proposal.get("evidence", {}).get("sources", []))
        target_evidence["sources"] = sources
        target_evidence["merged_from"] = _unique_list(
            target_evidence.get("merged_from", []) + [proposal.get("id", "")]
        )
        target["evidence"] = target_evidence

    finalized = [
        _finalize_service_task(
            proposal=proposal,
            portal_by_id=portal_by_id,
            body_core=body_core,
            options=options,
        )
        for proposal in merged
    ]
    for task_index, proposal in enumerate(finalized):
        proposal["id"] = f"area-{area_index}-task-{task_index}"
    return finalized


def _task_evidence_sources(proposal: dict[str, Any]) -> set[str]:
    evidence = proposal.get("evidence", {})
    sources: set[str] = set()
    if isinstance(evidence, dict):
        source = evidence.get("source")
        if source:
            sources.add(str(source))
        for source_record in evidence.get("sources", []):
            if not isinstance(source_record, dict):
                continue
            source = source_record.get("source")
            if source:
                sources.add(str(source))
    return sources


def _zones_describe_single_main_body(zones: list[dict[str, Any]], options: dict[str, Any]) -> bool:
    main_body_count = 0
    meaningful_non_body_count = 0
    pocket_min_area = float(options["task_pocket_min_visible_area_m2"])
    for zone in zones:
        kind = str(zone.get("kind", ""))
        zone_area = float(zone.get("area_m2", 0.0) or 0.0)
        if kind == "main_body":
            main_body_count += 1
        elif zone_area >= pocket_min_area:
            meaningful_non_body_count += 1
    return main_body_count == 1 and meaningful_non_body_count == 0


def _suppress_false_notches_on_simple_body(
    *,
    task_proposals: list[dict[str, Any]],
    portal_candidates: list[dict[str, Any]],
    zones: list[dict[str, Any]],
    drivable: Polygon | MultiPolygon,
    options: dict[str, Any],
) -> None:
    if not bool(options.get("task_false_notch_suppression_enabled", True)):
        return
    if not _zones_describe_single_main_body(zones, options):
        return

    portal_by_id = {str(portal.get("id", "")): portal for portal in portal_candidates}
    drivable_area = area_m2(drivable)
    max_area = float(options["task_false_notch_max_area_m2"])
    max_area_ratio = float(options["task_false_notch_max_area_ratio"])
    max_depth = float(options["task_false_notch_max_depth_m"])
    min_portal_depth_ratio = float(options["task_false_notch_min_portal_width_depth_ratio"])

    suppressed_portals: set[str] = set()
    for proposal in task_proposals:
        if proposal.get("task_type") != "notch":
            continue
        sources = _task_evidence_sources(proposal)
        if sources != {"body_core_subtraction"}:
            continue
        service_area = float(proposal.get("evidence", {}).get("service_area_m2", 0.0) or 0.0)
        if service_area <= EPS:
            service_area = area_m2(_geometry_from_json(proposal.get("geometry", {})))
        service_depth = float(proposal.get("service_depth_m", 0.0) or 0.0)
        if service_area > max_area:
            continue
        if drivable_area > EPS and service_area / drivable_area > max_area_ratio:
            continue
        if service_depth > max_depth:
            continue
        entry_portals = [str(portal_id) for portal_id in proposal.get("entry_portals", [])]
        portal_width = max(
            [float(portal_by_id[portal_id].get("width_m", 0.0) or 0.0) for portal_id in entry_portals if portal_id in portal_by_id]
            or [0.0]
        )
        if service_depth > EPS and portal_width < service_depth * min_portal_depth_ratio:
            continue

        reason = (
            "weak body-core subtraction notch suppressed because preliminary zoning found one main body"
        )
        proposal["task_type"] = "artifact"
        proposal["confidence"] = min(float(proposal.get("confidence", 0.0) or 0.0), 0.35)
        proposal["visible_by_default"] = False
        proposal["suppressed_reason"] = reason
        proposal["classification_reason"] = reason
        proposal["extent_source"] = str(proposal.get("extent_source", "")) or "suppressed_body_core_subtraction"
        proposal["entry_portals"] = []
        proposal.pop("terminal_cap", None)
        evidence = dict(proposal.get("evidence", {}))
        evidence["reason"] = reason
        evidence["suppression"] = {
            "rule": "simple_body_weak_body_core_subtraction_notch",
            "service_area_m2": round(service_area, 4),
            "service_area_ratio": round(service_area / max(drivable_area, EPS), 4),
            "service_depth_m": round(service_depth, 4),
            "portal_width_m": round(portal_width, 4),
        }
        proposal["evidence"] = evidence
        suppressed_portals.update(entry_portals)

    if not suppressed_portals:
        return
    active_portals = {
        str(portal_id)
        for proposal in task_proposals
        if proposal.get("task_type") != "artifact"
        for portal_id in proposal.get("entry_portals", [])
    }
    for portal_id in suppressed_portals.difference(active_portals):
        portal = portal_by_id.get(portal_id)
        if not portal:
            continue
        portal["status"] = "rejected"
        portal["reason"] = "portal hidden because weak false notch was suppressed"
        portal.pop("display_label", None)
        portal.pop("label_point", None)
        portal["suppressed_reason"] = (
            "weak body-core subtraction notch suppressed because preliminary zoning found one main body"
        )


def _task_sort_key(proposal: dict[str, Any]) -> tuple[int, int, float, str]:
    task_type = str(proposal.get("task_type", "artifact"))
    return (
        TASK_TYPE_ORDER.get(task_type, 99),
        0 if bool(proposal.get("visible_by_default", True)) else 1,
        -float(proposal.get("confidence", 0.0)),
        str(proposal.get("id", "")),
    )


def _task_display_label(task_type: str) -> str:
    return TASK_TYPE_LABELS.get(task_type, task_type.replace("_", " ").upper())


def _decorate_task_display_metadata(
    task_proposals: list[dict[str, Any]],
    portal_candidates: list[dict[str, Any]],
) -> None:
    visible_index = 1
    artifact_index = 1
    for proposal in sorted(task_proposals, key=_task_sort_key):
        task_type = str(proposal.get("task_type", "artifact"))
        proposal["display_label"] = _task_display_label(task_type)
        proposal["label_point"] = _label_point_for_geometry_json(proposal.get("geometry", {}))
        if task_type == "artifact":
            proposal["visible_by_default"] = False
            proposal["display_index"] = artifact_index
            proposal.setdefault("suppressed_reason", "artifact hidden from default task view")
            artifact_index += 1
        else:
            proposal["visible_by_default"] = True
            proposal["display_index"] = visible_index
            visible_index += 1

    portal_index = 1
    for portal in portal_candidates:
        if portal.get("status") != "accepted":
            continue
        portal["display_label"] = f"P{portal_index}"
        center = portal.get("center", [])
        portal["label_point"] = _round_point(center[0], center[1]) if len(center) >= 2 else []
        portal_index += 1


def build_task_evidence(
    drivable: Polygon | MultiPolygon,
    width_samples: list[dict[str, Any]],
    ridge_points: list[dict[str, Any]],
    ridge_links: list[dict[str, Any]],
    zones: list[dict[str, Any]],
    area_index: int,
    options: dict[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    """Convert ridge diagnostics into task-level evidence for M1."""
    point_by_id = {str(point["id"]): point for point in ridge_points}
    adjacency = _adjacency_from_links(ridge_links)
    body_samples = [sample for sample in width_samples if sample.get("class") == "body_width"]
    nodes = []
    for point in ridge_points:
        point_id = str(point["id"])
        degree = len(adjacency.get(point_id, []))
        if degree <= 1:
            node_type = "endpoint"
        elif degree >= 3:
            node_type = "branch_point"
        else:
            node_type = "chain"
        nodes.append(
            {
                "id": point_id,
                "area_index": point.get("area_index"),
                "component_index": point.get("component_index"),
                "node_type": node_type,
                "degree": degree,
                "x": point.get("x"),
                "y": point.get("y"),
                "local_width_m": point.get("local_width_m"),
                "clearance_m": point.get("clearance_m"),
            }
        )

    significant = {node["id"] for node in nodes if node["degree"] != 2}
    visited_edges: set[tuple[str, str]] = set()
    branches: list[dict[str, Any]] = []

    def append_branch(point_ids: list[str]) -> None:
        if len(point_ids) < 2:
            return
        coords = [(float(point_by_id[pid]["x"]), float(point_by_id[pid]["y"])) for pid in point_ids]
        line = LineString(coords)
        widths = [float(point_by_id[pid].get("local_width_m", 0.0)) for pid in point_ids]
        clearances = [float(point_by_id[pid].get("clearance_m", 0.0)) for pid in point_ids]
        class_counts: dict[str, int] = {}
        for pid in point_ids:
            sample_class = str(point_by_id[pid].get("class", "unknown"))
            class_counts[sample_class] = class_counts.get(sample_class, 0) + 1
        mean_width = sum(widths) / max(len(widths), 1)
        length = float(line.length)
        buffer_width = max(mean_width * 0.5, float(options["width_sample_step_m"]) * 0.55)
        branch_geom = _clean_polygonal(line.buffer(buffer_width, cap_style=2, join_style=2).intersection(drivable))
        endpoint_records = [
            _branch_endpoint_evidence(point_by_id[point_ids[0]], len(adjacency.get(point_ids[0], [])), body_samples, options),
            _branch_endpoint_evidence(point_by_id[point_ids[-1]], len(adjacency.get(point_ids[-1], [])), body_samples, options),
        ]
        opening_count = sum(1 for endpoint in endpoint_records if endpoint["is_opening"])
        aspect = length / max(mean_width, EPS)
        branch_type, reason = _classify_branch(
            length=length,
            mean_width=mean_width,
            branch_area=area_m2(branch_geom),
            opening_count=opening_count,
            aspect_ratio=aspect,
            class_counts=class_counts,
            options=options,
        )
        branches.append(
            {
                "id": f"area-{area_index}-branch-{len(branches)}",
                "area_index": area_index,
                "component_index": point_by_id[point_ids[0]].get("component_index", 0),
                "point_ids": point_ids,
                "node_start": point_ids[0],
                "node_end": point_ids[-1],
                "task_type": branch_type,
                "reason": reason,
                "length_m": round(length, 4),
                "mean_width_m": round(mean_width, 4),
                "min_width_m": round(min(widths), 4),
                "max_width_m": round(max(widths), 4),
                "mean_clearance_m": round(sum(clearances) / max(len(clearances), 1), 4),
                "aspect_ratio": round(aspect, 4),
                "area_m2": area_m2(branch_geom),
                "opening_count": opening_count,
                "class_counts": class_counts,
                "endpoints": endpoint_records,
                "line_segments": _branch_line_segments(coords),
                "geometry": geometry_to_json(branch_geom),
            }
        )

    for start_id in sorted(significant):
        for neighbor_id, _link in adjacency.get(start_id, []):
            key = _edge_key(start_id, neighbor_id)
            if key in visited_edges:
                continue
            visited_edges.add(key)
            point_ids = [start_id, neighbor_id]
            previous_id = start_id
            current_id = neighbor_id
            while current_id not in significant:
                next_edges = [
                    (candidate_id, candidate_link)
                    for candidate_id, candidate_link in adjacency.get(current_id, [])
                    if candidate_id != previous_id and _edge_key(current_id, candidate_id) not in visited_edges
                ]
                if not next_edges:
                    break
                next_id, _next_link = next_edges[0]
                visited_edges.add(_edge_key(current_id, next_id))
                point_ids.append(next_id)
                previous_id, current_id = current_id, next_id
            append_branch(point_ids)

    remaining_edges = {
        _edge_key(from_id, to_id)
        for from_id, neighbors in adjacency.items()
        for to_id, _link in neighbors
        if _edge_key(from_id, to_id) not in visited_edges
    }
    while remaining_edges:
        seed = next(iter(remaining_edges))
        stack = [seed[0], seed[1]]
        component_nodes: set[str] = set()
        component_edges: set[tuple[str, str]] = set()
        while stack:
            node_id = stack.pop()
            if node_id in component_nodes:
                continue
            component_nodes.add(node_id)
            for neighbor_id, _link in adjacency.get(node_id, []):
                key = _edge_key(node_id, neighbor_id)
                if key not in remaining_edges:
                    continue
                component_edges.add(key)
                stack.append(neighbor_id)
        visited_edges.update(component_edges)
        remaining_edges.difference_update(component_edges)
        if len(component_nodes) < 2:
            continue
        xs = [float(point_by_id[node_id]["x"]) for node_id in component_nodes]
        ys = [float(point_by_id[node_id]["y"]) for node_id in component_nodes]
        if max(xs) - min(xs) >= max(ys) - min(ys):
            ordered_ids = sorted(component_nodes, key=lambda node_id: (float(point_by_id[node_id]["x"]), float(point_by_id[node_id]["y"])))
        else:
            ordered_ids = sorted(component_nodes, key=lambda node_id: (float(point_by_id[node_id]["y"]), float(point_by_id[node_id]["x"])))
        append_branch(ordered_ids)

    graph_links = [
        {
            "id": link.get("id", f"skeleton-link-{idx}"),
            "from": link.get("from"),
            "to": link.get("to"),
            "length_m": link.get("length_m", 0.0),
            "line": link.get("line", []),
        }
        for idx, link in enumerate(ridge_links)
    ]
    skeleton_graph = {
        "nodes": nodes,
        "links": graph_links,
        "branches": branches,
    }

    portal_candidates: list[dict[str, Any]] = []
    task_proposals: list[dict[str, Any]] = []
    task_seeds: list[dict[str, Any]] = []
    body_core = _body_core_geometry(drivable, width_samples, options)
    body_area = area_m2(body_core)
    if body_area > EPS:
        body_samples_count = len(body_samples)
        confidence = min(0.95, 0.50 + min(body_area / max(area_m2(drivable), EPS), 1.0) * 0.35)
        task_proposals.append(
            {
                "id": f"area-{area_index}-task-0",
                "area_index": area_index,
                "task_type": "body_core",
                "confidence": round(confidence, 3),
                "geometry": geometry_to_json(body_core),
                "seed_geometry": geometry_to_json(body_core),
                "extent_source": "body_width_samples",
                "source_ids": ["body_core"],
                "service_depth_m": 0.0,
                "service_width_m": 0.0,
                "classification_reason": "union of body-width samples clipped to drivable area",
                "line_segments": [],
                "entry_portals": [],
                "evidence": {
                    "area_m2": area_m2(body_core),
                    "body_width_sample_count": body_samples_count,
                    "service_area_m2": area_m2(body_core),
                    "service_depth_m": 0.0,
                    "service_width_m": 0.0,
                    "portal_count": 0,
                    "body_overlap_ratio": 1.0,
                    "sources": [{"source": "body_width_samples", "id": "body_core"}],
                    "reason": "union of body-width samples clipped to drivable area",
                },
            }
        )

    for branch in branches:
        task_type = str(branch.get("task_type", "artifact"))
        if task_type == "body_core":
            continue
        branch_geom = _geometry_from_json(branch.get("geometry", {}))
        if branch_geom.is_empty:
            continue
        task_seeds.append(
            {
                "id": str(branch.get("id", f"area-{area_index}-branch-seed-{len(task_seeds)}")),
                "source": "skeleton_branch",
                "task_type": task_type,
                "geometry": branch_geom,
                "geometry_json": branch.get("geometry", geometry_to_json(branch_geom)),
                "line_segments": branch.get("line_segments", []),
                "evidence": {
                    "source": "skeleton_branch",
                    "branch_id": branch.get("id", ""),
                    "length_m": branch.get("length_m", 0.0),
                    "mean_width_m": branch.get("mean_width_m", 0.0),
                    "min_width_m": branch.get("min_width_m", 0.0),
                    "max_width_m": branch.get("max_width_m", 0.0),
                    "aspect_ratio": branch.get("aspect_ratio", 0.0),
                    "opening_count": branch.get("opening_count", 0),
                    "area_m2": branch.get("area_m2", 0.0),
                    "reason": branch.get("reason", ""),
                },
            }
        )

    if body_area > EPS:
        for component_index, component in enumerate(sorted(_iter_polygons(drivable), key=lambda poly: poly.area, reverse=True)):
            bounds = oriented_bounds(component)
            aspect = float(bounds["length_m"]) / max(float(bounds["width_m"]), EPS)
            if (
                component.area >= float(options["min_zone_area_m2"])
                and float(bounds["width_m"]) <= float(options["body_width_threshold_m"]) * 2.0
                and aspect >= max(2.0, float(options["task_corridor_aspect_ratio"]) * 0.75)
            ):
                confidence = min(0.82, 0.45 + min(aspect / max(float(options["task_corridor_aspect_ratio"]), EPS), 1.0) * 0.28)
                task_proposals.append(
                    {
                        "id": f"area-{area_index}-task-{len(task_proposals)}",
                        "area_index": area_index,
                        "task_type": "corridor",
                        "confidence": round(confidence, 3),
                        "geometry": geometry_to_json(component),
                        "seed_geometry": geometry_to_json(component),
                        "extent_source": "whole_component_shape",
                        "source_ids": [f"area-{area_index}-component-{component_index}"],
                        "service_depth_m": round(float(bounds["length_m"]), 4),
                        "service_width_m": round(float(bounds["width_m"]), 4),
                        "classification_reason": "whole component is long and narrow enough for corridor-style coverage",
                        "line_segments": _appendage_line_segments(component),
                        "source_line_segments": _appendage_line_segments(component),
                        "entry_portals": [],
                        "evidence": {
                            "source": "whole_component_shape",
                            "sources": [{"source": "whole_component_shape", "id": f"area-{area_index}-component-{component_index}"}],
                            "component_index": component_index,
                            "length_m": round(float(bounds["length_m"]), 4),
                            "mean_width_m": round(float(bounds["width_m"]), 4),
                            "min_width_m": round(float(bounds["width_m"]), 4),
                            "max_width_m": round(float(bounds["width_m"]), 4),
                            "aspect_ratio": round(aspect, 4),
                            "opening_count": 0,
                            "area_m2": area_m2(component),
                            "service_area_m2": area_m2(component),
                            "service_depth_m": round(float(bounds["length_m"]), 4),
                            "service_width_m": round(float(bounds["width_m"]), 4),
                            "portal_count": 0,
                            "body_overlap_ratio": 0.0,
                            "reason": "whole component is long and narrow enough for corridor-style coverage",
                        },
                    }
                )
        body_buffer_m = max(float(options["width_sample_step_m"]) * 4.0, float(options["body_width_threshold_m"]) * 0.50)
        appendage_area = _clean_polygonal(drivable.difference(body_core.buffer(body_buffer_m, join_style="round")))
        for appendage_index, appendage in enumerate(sorted(_iter_polygons(appendage_area), key=lambda poly: poly.area, reverse=True)):
            task_type, reason, aspect = _classify_appendage(appendage, options)
            if task_type == "body_core":
                continue
            bounds = oriented_bounds(appendage)
            appendage_id = f"area-{area_index}-appendage-{appendage_index}"
            task_seeds.append(
                {
                    "id": appendage_id,
                    "source": "body_core_subtraction",
                    "task_type": task_type,
                    "geometry": appendage,
                    "geometry_json": geometry_to_json(appendage),
                    "line_segments": _appendage_line_segments(appendage),
                    "evidence": {
                        "source": "body_core_subtraction",
                        "appendage_index": appendage_index,
                        "length_m": round(float(bounds["length_m"]), 4),
                        "mean_width_m": round(float(bounds["width_m"]), 4),
                        "min_width_m": round(float(bounds["width_m"]), 4),
                        "max_width_m": round(float(bounds["width_m"]), 4),
                        "aspect_ratio": round(float(aspect), 4),
                        "opening_count": 1 if task_type != "artifact" else 0,
                        "area_m2": area_m2(appendage),
                        "body_core_buffer_m": round(body_buffer_m, 4),
                        "reason": reason,
                    },
                }
            )

    pocket_min_area = float(options["task_pocket_min_visible_area_m2"])
    for zone in zones:
        if str(zone.get("kind", "")) == "main_body":
            continue
        zone_geom = _geometry_from_json(zone.get("geometry", {}))
        if zone_geom.is_empty or area_m2(zone_geom) < pocket_min_area:
            continue
        primary = _largest_polygon(zone_geom)
        if primary is None:
            continue
        bounds = oriented_bounds(primary)
        aspect = float(bounds["length_m"]) / max(float(bounds["width_m"]), EPS)
        task_seeds.append(
            {
                "id": str(zone.get("id", f"area-{area_index}-zone-seed-{len(task_seeds)}")),
                "source": "preliminary_zone_pocket",
                "task_type": "notch",
                "geometry": zone_geom,
                "geometry_json": zone.get("geometry", geometry_to_json(zone_geom)),
                "line_segments": _appendage_line_segments(primary),
                "evidence": {
                    "source": "preliminary_zone_pocket",
                    "zone_id": zone.get("id", ""),
                    "zone_kind": zone.get("kind", ""),
                    "length_m": round(float(bounds["length_m"]), 4),
                    "mean_width_m": round(float(bounds["width_m"]), 4),
                    "min_width_m": round(float(bounds["width_m"]), 4),
                    "max_width_m": round(float(bounds["width_m"]), 4),
                    "aspect_ratio": round(float(aspect), 4),
                    "opening_count": 1,
                    "area_m2": area_m2(zone_geom),
                    "reason": "preliminary non-main pocket promoted as service-region seed",
                },
            }
        )

    for seed in task_seeds:
        seed_geom = _clean_polygonal(seed["geometry"].intersection(drivable))
        if seed_geom.is_empty:
            continue
        entry_portals: list[str] = []
        portal: dict[str, Any] | None = None
        portal_line: LineString | None = None
        if seed.get("task_type") != "artifact":
            portal, portal_line = _service_portal_candidate_for_seed(
                area_index=area_index,
                seed_id=str(seed["id"]),
                seed_geom=seed_geom,
                body_core=body_core,
                drivable=drivable,
                options=options,
                portal_index=len(portal_candidates),
                reason="service-region mouth nearest to body core",
                seed_source=str(seed.get("source", "")),
                type_hint=str(seed.get("task_type", "")),
            )
            portal_candidates.append(portal)
            if portal["status"] == "accepted":
                entry_portals.append(str(portal["id"]))
        service_region, extent_source, body_overlap_ratio = _grow_service_region_from_seed(
            seed_geom=seed_geom,
            drivable=drivable,
            body_core=body_core,
            portal_line=portal_line if entry_portals else None,
            options=options,
        )
        measurements = _service_measurements(service_region, portal if entry_portals else None)
        task_type, reason = _classify_service_region(
            type_hint=str(seed.get("task_type", "artifact")),
            region=service_region,
            service_depth=float(measurements["service_depth_m"]),
            service_width=float(measurements["service_width_m"]),
            portal_count=len(entry_portals),
            body_overlap_ratio=body_overlap_ratio,
            options=options,
        )
        confidence = _confidence_for_service_region(
            task_type,
            float(measurements["service_depth_m"]),
            float(measurements["service_width_m"]),
            len(entry_portals),
            options,
        )
        source_id = str(seed["id"])
        seed_evidence = dict(seed.get("evidence", {}))
        seed_evidence["id"] = source_id
        seed_evidence["source"] = str(seed.get("source", "unknown"))
        task_proposals.append(
            {
                "id": f"area-{area_index}-task-{len(task_proposals)}",
                "area_index": area_index,
                "task_type": task_type,
                "confidence": confidence,
                "geometry": geometry_to_json(service_region),
                "seed_geometry": geometry_to_json(seed_geom),
                "extent_source": extent_source,
                "source_ids": [source_id],
                "service_depth_m": measurements["service_depth_m"],
                "service_width_m": measurements["service_width_m"],
                "classification_reason": reason,
                "line_segments": seed.get("line_segments", []),
                "source_line_segments": seed.get("line_segments", []),
                "entry_portals": entry_portals,
                "evidence": {
                    "source": seed.get("source", "unknown"),
                    "sources": [seed_evidence],
                    "length_m": measurements["service_depth_m"],
                    "mean_width_m": measurements["service_width_m"],
                    "min_width_m": seed_evidence.get("min_width_m", measurements["service_width_m"]),
                    "max_width_m": seed_evidence.get("max_width_m", measurements["service_width_m"]),
                    "aspect_ratio": round(
                        float(measurements["service_depth_m"]) / max(float(measurements["service_width_m"]), EPS),
                        4,
                    ),
                    "opening_count": len(entry_portals),
                    "area_m2": area_m2(seed_geom),
                    "service_area_m2": area_m2(service_region),
                    "service_depth_m": measurements["service_depth_m"],
                    "service_width_m": measurements["service_width_m"],
                    "portal_count": len(entry_portals),
                    "body_overlap_ratio": round(body_overlap_ratio, 4),
                    "extent_source": extent_source,
                    "reason": reason,
                },
            }
        )

    task_proposals = _merge_service_task_proposals(
        task_proposals,
        portal_candidates,
        body_core,
        area_index,
        options,
    )
    _suppress_false_notches_on_simple_body(
        task_proposals=task_proposals,
        portal_candidates=portal_candidates,
        zones=zones,
        drivable=drivable,
        options=options,
    )
    _decorate_task_display_metadata(task_proposals, portal_candidates)
    return skeleton_graph, portal_candidates, task_proposals


def _line_length(geom: Any) -> float:
    if geom.is_empty:
        return 0.0
    if isinstance(geom, (LineString, MultiLineString)):
        return float(geom.length)
    if isinstance(geom, GeometryCollection):
        return sum(_line_length(part) for part in geom.geoms)
    return 0.0


def _line_segments_to_json(geom: Any) -> list[list[list[float]]]:
    if geom.is_empty:
        return []
    if isinstance(geom, LineString):
        coords = list(geom.coords)
        if len(coords) < 2:
            return []
        return [[_round_point(coords[0][0], coords[0][1]), _round_point(coords[-1][0], coords[-1][1])]]
    if isinstance(geom, MultiLineString):
        segments: list[list[list[float]]] = []
        for line in geom.geoms:
            segments.extend(_line_segments_to_json(line))
        return segments
    if isinstance(geom, GeometryCollection):
        segments = []
        for part in geom.geoms:
            segments.extend(_line_segments_to_json(part))
        return segments
    return []


def detect_neck_candidates(
    drivable: Polygon | MultiPolygon,
    area_index: int,
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    """Sweep cross-sections and report evidence for possible neck cuts."""
    candidates: list[dict[str, Any]] = []
    sweep_step = max(float(options["width_sample_step_m"]) * 2.0, 0.4)
    body_threshold = float(options["body_width_threshold_m"])
    widening_target = float(options["neck_widening_ratio"])
    min_area = float(options["neck_min_area_m2"])
    min_visible_widening = 1.05

    for component_index, component in enumerate(sorted(_iter_polygons(drivable), key=lambda p: p.area, reverse=True)):
        if component.area <= min_area * 2.0:
            continue
        bounds = oriented_bounds(component)
        angle = bounds["angle_deg"]
        rotated = rotate(component, -angle, origin=(0.0, 0.0), use_radians=False)
        minx, miny, maxx, maxy = rotated.bounds
        if maxx - minx < sweep_step * 3.0 or maxy - miny <= EPS:
            continue
        pad = max(1.0, sweep_step, (maxy - miny) * 0.1)
        sections: list[dict[str, Any]] = []
        x = minx + sweep_step
        while x < maxx - sweep_step + EPS:
            line = LineString([(x, miny - pad), (x, maxy + pad)])
            intersection = rotated.intersection(line)
            width = _line_length(intersection)
            if width > EPS:
                sections.append({"x": x, "width_m": width, "intersection": intersection, "line": line})
            x += sweep_step
        if len(sections) < 3:
            continue

        window_radius = max(4, int(math.ceil(max(body_threshold, bounds["width_m"] * 0.35) / sweep_step)))
        for idx in range(1, len(sections) - 1):
            current = sections[idx]
            width = float(current["width_m"])
            if width <= EPS:
                continue
            prev_width = float(sections[idx - 1]["width_m"])
            next_width = float(sections[idx + 1]["width_m"])
            if width > prev_width + EPS or width > next_width + EPS:
                continue
            left_window = sections[max(0, idx - window_radius) : idx]
            right_window = sections[idx + 1 : min(len(sections), idx + window_radius + 1)]
            if not left_window or not right_window:
                continue
            left_max = max(float(section["width_m"]) for section in left_window)
            right_max = max(float(section["width_m"]) for section in right_window)
            widening_ratio = min(left_max, right_max) / max(width, EPS)
            if widening_ratio < min_visible_widening:
                continue
            station = float(current["x"] - minx)
            if any(
                candidate.get("area_index") == area_index
                and candidate.get("component_index") == component_index
                and abs(float(candidate.get("axis_station_m", 0.0)) - station) <= sweep_step * 2.0
                for candidate in candidates
            ):
                continue

            strip_half_width = max(0.025, sweep_step * 0.05)
            cut_strip = current["line"].buffer(strip_half_width, cap_style=2)
            split_geom = _clean_polygonal(rotated.difference(cut_strip))
            significant_regions = [poly for poly in _iter_polygons(split_geom) if poly.area >= min_area]
            status = "accepted"
            reason = "separates_meaningful_regions"
            if width > body_threshold:
                status = "rejected"
                reason = "width_above_body_threshold"
            elif widening_ratio < widening_target:
                status = "rejected"
                reason = "insufficient_widening"
            elif len(significant_regions) < 2:
                status = "rejected"
                reason = "does_not_separate_meaningful_regions"

            world_intersection = rotate(current["intersection"], angle, origin=(0.0, 0.0), use_radians=False)
            score = (widening_ratio * (body_threshold / max(width, EPS))) + max(0, len(significant_regions) - 1)
            candidates.append(
                {
                    "id": f"area-{area_index}-neck-{len(candidates)}",
                    "area_index": area_index,
                    "component_index": component_index,
                    "status": status,
                    "reason": reason,
                    "width_m": round(width, 4),
                    "left_width_m": round(left_max, 4),
                    "right_width_m": round(right_max, 4),
                    "widening_ratio": round(widening_ratio, 4),
                    "score": round(score, 4),
                    "axis_angle_deg": round(float(angle), 4),
                    "axis_station_m": round(station, 4),
                    "separated_region_count": len(significant_regions),
                    "line_segments": _line_segments_to_json(world_intersection),
                }
            )

    return candidates


def _feature_records(kind: str, geom: Any, min_area_m2: float, reason: str) -> list[dict[str, Any]]:
    features = []
    for idx, poly in enumerate(sorted(_iter_polygons(geom), key=lambda p: p.area, reverse=True)):
        if poly.area < min_area_m2:
            continue
        features.append(
            {
                "id": f"{kind}-{idx}",
                "kind": kind,
                "area_m2": area_m2(poly),
                "reason": reason,
                "geometry": geometry_to_json(poly),
            }
        )
    return features


def classify_lawn(
    *,
    area_index: int,
    source_id: str,
    source_name: str,
    outline: list[tuple[float, float]],
    holes: list[dict[str, Any]],
    config: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    raw = _make_polygon(outline, [hole["outline"] for hole in holes])
    tolerance = float(options["simplify_tolerance_m"])
    conditioned = _condition_polygon(raw, tolerance)

    radius = base_link_all_yaw_radius(config)
    boundary_clearance = float(options["drivable_boundary_clearance_m"])
    drivable = _clean_polygonal(conditioned.buffer(-boundary_clearance, join_style="round", quad_segs=16))
    all_yaw_safe = _clean_polygonal(conditioned.buffer(-radius, join_style="round", quad_segs=16))
    coverage_band = _clean_polygonal(conditioned.difference(drivable))
    maneuver_limited_area = _clean_polygonal(drivable.difference(all_yaw_safe))
    removed_noise = _clean_polygonal(raw.difference(conditioned))
    filled_smooth_area = _clean_polygonal(conditioned.difference(raw))
    if drivable.is_empty:
        unreachable = conditioned
    else:
        reachable_lawn = _clean_polygonal(
            drivable.buffer(boundary_clearance, join_style="round", quad_segs=16).intersection(conditioned)
        )
        unreachable = _clean_polygonal(conditioned.difference(reachable_lawn))

    zones = classify_zones(drivable, area_index, options)
    width_samples = sample_width_field(drivable, area_index, config, options)
    ridge_points, ridge_links = detect_ridge_diagnostics(width_samples, options)
    skeleton_graph, portal_candidates, task_proposals = build_task_evidence(
        drivable,
        width_samples,
        ridge_points,
        ridge_links,
        zones,
        area_index,
        options,
    )
    neck_candidates = detect_neck_candidates(drivable, area_index, options)
    tiny_area = float(options["tiny_feature_area_m2"])
    features: list[dict[str, Any]] = []
    features.extend(
        _feature_records(
            "unreachable_notch",
            unreachable,
            tiny_area,
            "conditioned lawn area outside yaw-independent footprint-safe reach",
        )
    )
    features.extend(
        _feature_records(
            "removed_wrinkle",
            removed_noise,
            tiny_area,
            "removed by mower-scale boundary conditioning",
        )
    )
    features.extend(
        _feature_records(
            "filled_smooth_notch",
            filled_smooth_area,
            tiny_area,
            "added by mower-scale boundary conditioning",
        )
    )

    counts: dict[str, int] = {}
    areas_by_kind: dict[str, float] = {}
    widths = []
    for zone in zones:
        kind = str(zone["kind"])
        counts[kind] = counts.get(kind, 0) + 1
        areas_by_kind[kind] = areas_by_kind.get(kind, 0.0) + float(zone["area_m2"])
        widths.append(float(zone.get("oriented_bounds", {}).get("width_m", 0.0)))

    width_values = [float(sample["local_width_m"]) for sample in width_samples]
    width_classes: dict[str, int] = {}
    for sample in width_samples:
        sample_class = str(sample.get("class", "unknown"))
        width_classes[sample_class] = width_classes.get(sample_class, 0) + 1

    neck_by_reason: dict[str, int] = {}
    neck_by_status: dict[str, int] = {}
    for candidate in neck_candidates:
        reason = str(candidate.get("reason", "unknown"))
        status = str(candidate.get("status", "unknown"))
        neck_by_reason[reason] = neck_by_reason.get(reason, 0) + 1
        neck_by_status[status] = neck_by_status.get(status, 0) + 1
    task_by_type: dict[str, int] = {}
    visible_task_count = 0
    hidden_artifact_count = 0
    terminal_cap_count = 0
    for proposal in task_proposals:
        task_type = str(proposal.get("task_type", "unknown"))
        task_by_type[task_type] = task_by_type.get(task_type, 0) + 1
        if proposal.get("visible_by_default", True):
            visible_task_count += 1
        elif task_type == "artifact":
            hidden_artifact_count += 1
        if proposal.get("terminal_cap"):
            terminal_cap_count += 1
    portal_by_status: dict[str, int] = {}
    portal_by_reason: dict[str, int] = {}
    accepted_visible_portals = 0
    for candidate in portal_candidates:
        status = str(candidate.get("status", "unknown"))
        reason = str(candidate.get("reason", "unknown"))
        portal_by_status[status] = portal_by_status.get(status, 0) + 1
        portal_by_reason[reason] = portal_by_reason.get(reason, 0) + 1
        if status == "accepted" and candidate.get("display_label"):
            accepted_visible_portals += 1

    warnings = []
    if not zones:
        warnings.append("no footprint-safe drivable zones survived conditioning")
    if area_m2(conditioned) < area_m2(raw) * 0.90:
        warnings.append("conditioning removed more than 10% of raw lawn area")

    return {
        "area_index": area_index,
        "source_id": source_id,
        "source_name": source_name,
        "raw_area_m2": area_m2(raw),
        "conditioned_area_m2": area_m2(conditioned),
        "drivable_area_m2": area_m2(drivable),
        "all_yaw_safe_area_m2": area_m2(all_yaw_safe),
        "coverage_band_area_m2": area_m2(coverage_band),
        "maneuver_limited_area_m2": area_m2(maneuver_limited_area),
        "unreachable_area_m2": area_m2(unreachable),
        "removed_wrinkle_area_m2": area_m2(removed_noise),
        "filled_smooth_area_m2": area_m2(filled_smooth_area),
        "zone_counts": counts,
        "zone_areas_m2": areas_by_kind,
        "zone_widths_m": {
            "min": min(widths) if widths else 0.0,
            "median": sorted(widths)[len(widths) // 2] if widths else 0.0,
            "max": max(widths) if widths else 0.0,
        },
        "local_width_m": {
            "sample_count": len(width_samples),
            "min": min(width_values) if width_values else 0.0,
            "median": sorted(width_values)[len(width_values) // 2] if width_values else 0.0,
            "max": max(width_values) if width_values else 0.0,
            "class_counts": width_classes,
        },
        "ridge": {
            "point_count": len(ridge_points),
            "link_count": len(ridge_links),
        },
        "skeleton_graph_summary": {
            "node_count": len(skeleton_graph.get("nodes", [])),
            "link_count": len(skeleton_graph.get("links", [])),
            "branch_count": len(skeleton_graph.get("branches", [])),
        },
        "task_proposals_summary": {
            "total": len(task_proposals),
            "by_type": task_by_type,
            "visible_by_default": visible_task_count,
            "hidden_artifacts": hidden_artifact_count,
            "terminal_caps": terminal_cap_count,
        },
        "portal_candidates_summary": {
            "total": len(portal_candidates),
            "accepted": portal_by_status.get("accepted", 0),
            "rejected": portal_by_status.get("rejected", 0),
            "accepted_visible": accepted_visible_portals,
            "by_status": portal_by_status,
            "by_reason": portal_by_reason,
        },
        "neck_candidates_summary": {
            "total": len(neck_candidates),
            "accepted": neck_by_status.get("accepted", 0),
            "rejected": neck_by_status.get("rejected", 0),
            "by_reason": neck_by_reason,
            "by_status": neck_by_status,
        },
        "geometries": {
            "raw_lawn": geometry_to_json(raw),
            "conditioned_lawn": geometry_to_json(conditioned),
            "drivable_region": geometry_to_json(drivable),
            "all_yaw_safe_region": geometry_to_json(all_yaw_safe),
            "coverage_band": geometry_to_json(coverage_band),
            "maneuver_limited_area": geometry_to_json(maneuver_limited_area),
            "unreachable": geometry_to_json(unreachable),
            "removed_noise": geometry_to_json(removed_noise),
            "filled_smooth_area": geometry_to_json(filled_smooth_area),
        },
        "zones": zones,
        "width_samples": width_samples,
        "ridge_points": ridge_points,
        "ridge_links": ridge_links,
        "skeleton_graph": skeleton_graph,
        "portal_candidates": portal_candidates,
        "task_proposals": task_proposals,
        "neck_candidates": neck_candidates,
        "features": features,
        "warnings": warnings,
    }


def build_result(
    *,
    map_path: pathlib.Path,
    frame_id: str,
    config: dict[str, Any],
    options: dict[str, Any],
    areas: list[dict[str, Any]],
) -> dict[str, Any]:
    warnings = [f"area {area['area_index']}: {warning}" for area in areas for warning in area.get("warnings", [])]
    return {
        "schema": "open_mower.coverage_lab.v2_geometry.v0",
        "source_map": str(map_path),
        "frame_id": frame_id,
        "mower_model": mower_model(config),
        "conditioning": options,
        "areas": areas,
        "warnings": warnings,
    }


def build_metrics(result: dict[str, Any], baseline_current_planner: dict[str, Any] | None = None) -> dict[str, Any]:
    areas = result.get("areas", [])
    zone_counts: dict[str, int] = {}
    zone_areas: dict[str, float] = {}
    widths: list[float] = []
    local_width_values: list[float] = []
    local_width_classes: dict[str, int] = {}
    ridge_point_count = 0
    ridge_link_count = 0
    skeleton_node_count = 0
    skeleton_branch_count = 0
    task_by_type: dict[str, int] = {}
    visible_task_count = 0
    hidden_artifact_count = 0
    terminal_cap_count = 0
    portal_by_reason: dict[str, int] = {}
    portal_by_status: dict[str, int] = {}
    portal_total = 0
    accepted_visible_portals = 0
    neck_by_reason: dict[str, int] = {}
    neck_by_status: dict[str, int] = {}
    neck_total = 0
    task_path_by_type: dict[str, int] = {}
    task_path_total = 0
    task_path_region_area = 0.0
    task_path_covered_area = 0.0
    task_path_forward_length = 0.0
    task_path_reverse_length = 0.0
    task_path_unsafe_samples = 0
    task_path_warning_count = 0
    task_path_exit_modes: dict[str, int] = {}
    task_path_axis_candidates_total = 0
    task_path_selected_axis_by_source: dict[str, int] = {}
    task_path_body_pocket_count = 0
    task_path_body_pocket_area = 0.0
    task_path_compact_connector_count = 0
    task_path_compact_connector_length = 0.0
    task_path_compact_connector_rejected = 0
    task_path_compact_connector_unsafe_rejected = 0
    task_path_compact_connector_too_far_rejected = 0
    outline_path_total = 0
    outline_path_length = 0.0
    outline_path_cutting_length = 0.0
    outline_path_unsafe_samples = 0
    outline_path_warning_count = 0
    outline_generation_modes: dict[str, int] = {}
    outline_right_edge_clearance_sources: dict[str, int] = {}
    outline_centerline_offsets: list[float] = []
    outline_right_edge_min_values: list[float] = []
    outline_right_edge_median_values: list[float] = []
    outline_right_edge_max_values: list[float] = []
    outline_right_footprint_min_values: list[float] = []
    outline_right_footprint_median_values: list[float] = []
    outline_right_footprint_max_values: list[float] = []
    outline_footprint_min_values: list[float] = []
    outline_corner_local_fit_extra_values: list[float] = []
    annotation_total = 0
    annotation_accepted = 0
    annotation_rejected = 0
    ownership_adjustment_count = 0
    ownership_adjustment_area = 0.0
    for area in areas:
        for kind, count in area.get("zone_counts", {}).items():
            zone_counts[kind] = zone_counts.get(kind, 0) + int(count)
        for kind, value in area.get("zone_areas_m2", {}).items():
            zone_areas[kind] = zone_areas.get(kind, 0.0) + float(value)
        for zone in area.get("zones", []):
            width = zone.get("oriented_bounds", {}).get("width_m")
            if isinstance(width, (int, float)):
                widths.append(float(width))
        for sample in area.get("width_samples", []):
            width = sample.get("local_width_m")
            if isinstance(width, (int, float)):
                local_width_values.append(float(width))
            sample_class = str(sample.get("class", "unknown"))
            local_width_classes[sample_class] = local_width_classes.get(sample_class, 0) + 1
        ridge_point_count += int(area.get("ridge", {}).get("point_count", 0))
        ridge_link_count += int(area.get("ridge", {}).get("link_count", 0))
        skeleton = area.get("skeleton_graph", {})
        skeleton_node_count += len(skeleton.get("nodes", []))
        skeleton_branch_count += len(skeleton.get("branches", []))
        for proposal in area.get("task_proposals", []):
            task_type = str(proposal.get("task_type", "unknown"))
            task_by_type[task_type] = task_by_type.get(task_type, 0) + 1
            if proposal.get("visible_by_default", True):
                visible_task_count += 1
            elif task_type == "artifact":
                hidden_artifact_count += 1
            if proposal.get("terminal_cap"):
                terminal_cap_count += 1
        for candidate in area.get("portal_candidates", []):
            portal_total += 1
            status = str(candidate.get("status", "unknown"))
            reason = str(candidate.get("reason", "unknown"))
            portal_by_status[status] = portal_by_status.get(status, 0) + 1
            portal_by_reason[reason] = portal_by_reason.get(reason, 0) + 1
            if status == "accepted" and candidate.get("display_label"):
                accepted_visible_portals += 1
        for candidate in area.get("neck_candidates", []):
            neck_total += 1
            status = str(candidate.get("status", "unknown"))
            reason = str(candidate.get("reason", "unknown"))
            neck_by_status[status] = neck_by_status.get(status, 0) + 1
            neck_by_reason[reason] = neck_by_reason.get(reason, 0) + 1
        for task_path in area.get("task_paths", []):
            task_path_total += 1
            task_type = str(task_path.get("task_type", "unknown"))
            task_path_by_type[task_type] = task_path_by_type.get(task_type, 0) + 1
            exit_mode = str(task_path.get("exit_mode", "unknown"))
            task_path_exit_modes[exit_mode] = task_path_exit_modes.get(exit_mode, 0) + 1
            task_path_axis_candidates_total += len(task_path.get("axis_candidates", []))
            axis_source = str((task_path.get("service_axis") or {}).get("source", "none"))
            task_path_selected_axis_by_source[axis_source] = task_path_selected_axis_by_source.get(axis_source, 0) + 1
            task_path_region_area += float(task_path.get("coverage_region_area_m2", 0.0))
            task_path_covered_area += float(task_path.get("estimated_covered_area_m2", 0.0))
            task_path_forward_length += float(task_path.get("forward_length_m", 0.0))
            task_path_reverse_length += float(task_path.get("reverse_length_m", 0.0))
            task_path_unsafe_samples += int(task_path.get("unsafe_sample_count", 0))
            task_path_warning_count += len(task_path.get("warnings", []))
            task_path_body_pocket_count += int(task_path.get("body_service_pocket_count", 0))
            task_path_body_pocket_area += float(task_path.get("body_service_pocket_area_m2", 0.0))
            connector_summary = task_path.get("compact_connector_summary") or {}
            task_path_compact_connector_count += int(
                connector_summary.get("inserted_count", task_path.get("compact_connector_count", 0))
            )
            task_path_compact_connector_length += float(
                connector_summary.get("length_m", task_path.get("compact_connector_length_m", 0.0))
            )
            task_path_compact_connector_rejected += int(connector_summary.get("rejected_count", 0))
            task_path_compact_connector_unsafe_rejected += int(connector_summary.get("unsafe_rejected_count", 0))
            task_path_compact_connector_too_far_rejected += int(
                connector_summary.get("too_far_rejected_count", 0)
            )
        for outline_path in area.get("outline_paths", []):
            outline_path_total += 1
            outline_path_length += float(outline_path.get("path_length_m", 0.0))
            outline_path_cutting_length += float(outline_path.get("cutting_length_m", 0.0))
            outline_path_unsafe_samples += int(outline_path.get("unsafe_sample_count", 0))
            outline_path_warning_count += len(outline_path.get("warnings", []))
            generation_mode = str(outline_path.get("outline_generation_mode", "unknown"))
            outline_generation_modes[generation_mode] = outline_generation_modes.get(generation_mode, 0) + 1
            clearance_source = str(
                outline_path.get(
                    "outline_right_edge_clearance_source",
                    outline_path.get("outline_right_footprint_clearance_source", "unknown"),
                )
            )
            outline_right_edge_clearance_sources[clearance_source] = (
                outline_right_edge_clearance_sources.get(clearance_source, 0) + 1
            )
            try:
                outline_centerline_offsets.append(float(outline_path.get("centerline_offset_m")))
            except (TypeError, ValueError):
                pass
            try:
                outline_right_edge_min_values.append(float(outline_path.get("right_cut_edge_min_clearance_m")))
            except (TypeError, ValueError):
                pass
            try:
                outline_right_edge_median_values.append(float(outline_path.get("right_cut_edge_median_clearance_m")))
            except (TypeError, ValueError):
                pass
            try:
                outline_right_edge_max_values.append(float(outline_path.get("right_cut_edge_max_clearance_m")))
            except (TypeError, ValueError):
                pass
            try:
                outline_right_footprint_min_values.append(
                    float(outline_path.get("right_footprint_side_min_clearance_m"))
                )
            except (TypeError, ValueError):
                pass
            try:
                outline_right_footprint_median_values.append(
                    float(outline_path.get("right_footprint_side_median_clearance_m"))
                )
            except (TypeError, ValueError):
                pass
            try:
                outline_right_footprint_max_values.append(
                    float(outline_path.get("right_footprint_side_max_clearance_m"))
                )
            except (TypeError, ValueError):
                pass
            try:
                outline_footprint_min_values.append(float(outline_path.get("footprint_min_clearance_m")))
            except (TypeError, ValueError):
                pass
            try:
                outline_corner_local_fit_extra_values.append(
                    float(outline_path.get("outline_corner_local_fit_max_extra_offset_m"))
                )
            except (TypeError, ValueError):
                pass
        for annotation in area.get("operator_annotations", []):
            annotation_total += 1
            if annotation.get("status") == "accepted":
                annotation_accepted += 1
            else:
                annotation_rejected += 1
        for adjustment in area.get("ownership_adjustments", []):
            ownership_adjustment_count += 1
            ownership_adjustment_area += float(adjustment.get("area_m2", 0.0))

    metrics = {
        "schema": "open_mower.coverage_lab.v2_metrics.v0",
        "source_map": result.get("source_map", ""),
        "area_count": len(areas),
        "raw_area_m2": sum(float(area.get("raw_area_m2", 0.0)) for area in areas),
        "conditioned_area_m2": sum(float(area.get("conditioned_area_m2", 0.0)) for area in areas),
        "drivable_area_m2": sum(float(area.get("drivable_area_m2", 0.0)) for area in areas),
        "all_yaw_safe_area_m2": sum(float(area.get("all_yaw_safe_area_m2", 0.0)) for area in areas),
        "coverage_band_area_m2": sum(float(area.get("coverage_band_area_m2", 0.0)) for area in areas),
        "maneuver_limited_area_m2": sum(float(area.get("maneuver_limited_area_m2", 0.0)) for area in areas),
        "unreachable_area_m2": sum(float(area.get("unreachable_area_m2", 0.0)) for area in areas),
        "removed_wrinkle_area_m2": sum(float(area.get("removed_wrinkle_area_m2", 0.0)) for area in areas),
        "filled_smooth_area_m2": sum(float(area.get("filled_smooth_area_m2", 0.0)) for area in areas),
        "zone_count": sum(len(area.get("zones", [])) for area in areas),
        "zone_counts": zone_counts,
        "zone_areas_m2": zone_areas,
        "zone_widths_m": {
            "min": min(widths) if widths else 0.0,
            "median": sorted(widths)[len(widths) // 2] if widths else 0.0,
            "max": max(widths) if widths else 0.0,
        },
        "local_width": {
            "sample_count": len(local_width_values),
            "min_m": min(local_width_values) if local_width_values else 0.0,
            "median_m": sorted(local_width_values)[len(local_width_values) // 2] if local_width_values else 0.0,
            "max_m": max(local_width_values) if local_width_values else 0.0,
            "class_counts": local_width_classes,
        },
        "ridge": {
            "point_count": ridge_point_count,
            "link_count": ridge_link_count,
        },
        "skeleton_graph": {
            "node_count": skeleton_node_count,
            "branch_count": skeleton_branch_count,
        },
        "task_proposals": {
            "total": sum(task_by_type.values()),
            "by_type": task_by_type,
            "visible_by_default": visible_task_count,
            "hidden_artifacts": hidden_artifact_count,
            "terminal_caps": terminal_cap_count,
        },
        "portal_candidates": {
            "total": portal_total,
            "accepted": portal_by_status.get("accepted", 0),
            "rejected": portal_by_status.get("rejected", 0),
            "accepted_visible": accepted_visible_portals,
            "by_reason": portal_by_reason,
            "by_status": portal_by_status,
        },
        "neck_candidates": {
            "total": neck_total,
            "accepted": neck_by_status.get("accepted", 0),
            "rejected": neck_by_status.get("rejected", 0),
            "by_reason": neck_by_reason,
            "by_status": neck_by_status,
        },
        "task_paths": {
            "total": task_path_total,
            "by_task_type": task_path_by_type,
            "by_exit_mode": task_path_exit_modes,
            "estimated_coverage_percent": (
                100.0 * task_path_covered_area / task_path_region_area if task_path_region_area > EPS else 0.0
            ),
            "coverage_region_area_m2": task_path_region_area,
            "estimated_covered_area_m2": task_path_covered_area,
            "forward_length_m": task_path_forward_length,
            "reverse_length_m": task_path_reverse_length,
            "unsafe_sample_count": task_path_unsafe_samples,
            "warning_count": task_path_warning_count,
            "axis_candidates_total": task_path_axis_candidates_total,
            "selected_axis_by_source": task_path_selected_axis_by_source,
            "body_service_pocket_count": task_path_body_pocket_count,
            "body_service_pocket_area_m2": task_path_body_pocket_area,
            "compact_connector_count": task_path_compact_connector_count,
            "compact_connector_length_m": task_path_compact_connector_length,
            "compact_connector_rejected_count": task_path_compact_connector_rejected,
            "compact_connector_unsafe_rejected_count": task_path_compact_connector_unsafe_rejected,
            "compact_connector_too_far_rejected_count": task_path_compact_connector_too_far_rejected,
        },
        "outline_paths": {
            "total": outline_path_total,
            "path_length_m": outline_path_length,
            "cutting_length_m": outline_path_cutting_length,
            "unsafe_sample_count": outline_path_unsafe_samples,
            "warning_count": outline_path_warning_count,
            "generation_modes": outline_generation_modes,
            "right_edge_clearance_sources": outline_right_edge_clearance_sources,
            "centerline_offset_min_m": min(outline_centerline_offsets) if outline_centerline_offsets else 0.0,
            "centerline_offset_median_m": (
                sorted(outline_centerline_offsets)[len(outline_centerline_offsets) // 2]
                if outline_centerline_offsets
                else 0.0
            ),
            "centerline_offset_max_m": max(outline_centerline_offsets) if outline_centerline_offsets else 0.0,
            "right_cut_edge_min_clearance_m": (
                min(outline_right_edge_min_values) if outline_right_edge_min_values else 0.0
            ),
            "right_cut_edge_median_clearance_m": (
                sorted(outline_right_edge_median_values)[len(outline_right_edge_median_values) // 2]
                if outline_right_edge_median_values
                else 0.0
            ),
            "right_cut_edge_max_clearance_m": (
                max(outline_right_edge_max_values) if outline_right_edge_max_values else 0.0
            ),
            "right_footprint_side_min_clearance_m": (
                min(outline_right_footprint_min_values) if outline_right_footprint_min_values else 0.0
            ),
            "right_footprint_side_median_clearance_m": (
                sorted(outline_right_footprint_median_values)[len(outline_right_footprint_median_values) // 2]
                if outline_right_footprint_median_values
                else 0.0
            ),
            "right_footprint_side_max_clearance_m": (
                max(outline_right_footprint_max_values) if outline_right_footprint_max_values else 0.0
            ),
            "footprint_min_clearance_m": min(outline_footprint_min_values) if outline_footprint_min_values else 0.0,
            "corner_local_fit_max_extra_offset_m": (
                max(outline_corner_local_fit_extra_values) if outline_corner_local_fit_extra_values else 0.0
            ),
        },
        "task_annotations": {
            "total": annotation_total,
            "accepted": annotation_accepted,
            "rejected": annotation_rejected,
        },
        "annotation_qa": result.get("annotation_qa", {}).get(
            "summary",
            {
                "operator_tasks": 0,
                "matched": 0,
                "missed_by_auto": 0,
                "type_mismatch": 0,
                "mouth_misaligned": 0,
                "auto_too_large": 0,
                "auto_too_small": 0,
                "partial_match": 0,
                "unmatched_auto": 0,
                "hidden_auto_artifacts": 0,
                "status_counts": {},
            },
        ),
        "ownership_adjustments": {
            "total": ownership_adjustment_count,
            "area_m2": ownership_adjustment_area,
        },
        "warnings": result.get("warnings", []),
    }
    if baseline_current_planner:
        metrics["baseline_current_planner"] = baseline_current_planner
    return metrics


def _collect_points_from_geom(geom_json: dict[str, Any]) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for poly in geom_json.get("polygons", []):
        for ring_key in ("outer",):
            for x, y in poly.get(ring_key, []):
                pts.append((float(x), float(y)))
        for hole in poly.get("holes", []):
            for x, y in hole:
                pts.append((float(x), float(y)))
    return pts


def _all_report_points(result: dict[str, Any]) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for area in result.get("areas", []):
        for geom in area.get("geometries", {}).values():
            pts.extend(_collect_points_from_geom(geom))
        for task_path in area.get("task_paths", []):
            for record in (task_path.get("compact_connector_summary") or {}).get("records", []):
                diagnostic = record.get("diagnostic_segment") or {}
                for point in diagnostic.get("base_poses", []) or diagnostic.get("tool_points", []):
                    pts.append((float(point.get("x", 0.0)), float(point.get("y", 0.0))))
                for line in record.get("diagnostic_line_segments", []):
                    for point in line:
                        if len(point) >= 2:
                            pts.append((float(point[0]), float(point[1])))
    return pts


def _svg_path_for_geometry(
    geom_json: dict[str, Any],
    project: Any,
) -> str:
    parts = []
    for poly in geom_json.get("polygons", []):
        outer = poly.get("outer", [])
        if outer:
            first = project(outer[0][0], outer[0][1])
            cmds = [f"M {first[0]:.2f} {first[1]:.2f}"]
            for x, y in outer[1:]:
                sx, sy = project(x, y)
                cmds.append(f"L {sx:.2f} {sy:.2f}")
            cmds.append("Z")
            parts.append(" ".join(cmds))
        for hole in poly.get("holes", []):
            if not hole:
                continue
            first = project(hole[0][0], hole[0][1])
            cmds = [f"M {first[0]:.2f} {first[1]:.2f}"]
            for x, y in hole[1:]:
                sx, sy = project(x, y)
                cmds.append(f"L {sx:.2f} {sy:.2f}")
            cmds.append("Z")
            parts.append(" ".join(cmds))
    return " ".join(parts)


def _svg_arrow_polygon(
    start: tuple[float, float],
    end: tuple[float, float],
    project: Any,
    size: float = 7.0,
) -> str:
    sx, sy = project(start[0], start[1])
    ex, ey = project(end[0], end[1])
    dx = ex - sx
    dy = ey - sy
    norm = math.hypot(dx, dy)
    if norm <= EPS:
        return ""
    ux = dx / norm
    uy = dy / norm
    bx = ex - ux * size
    by = ey - uy * size
    nx = -uy
    ny = ux
    p1 = (ex, ey)
    p2 = (bx + nx * size * 0.55, by + ny * size * 0.55)
    p3 = (bx - nx * size * 0.55, by - ny * size * 0.55)
    return f"{p1[0]:.2f},{p1[1]:.2f} {p2[0]:.2f},{p2[1]:.2f} {p3[0]:.2f},{p3[1]:.2f}"


def render_svg(result: dict[str, Any]) -> str:
    pts = _all_report_points(result)
    if not pts:
        pts = [(0.0, 0.0), (1.0, 1.0)]
    minx = min(p[0] for p in pts)
    maxx = max(p[0] for p in pts)
    miny = min(p[1] for p in pts)
    maxy = max(p[1] for p in pts)
    width_m = max(maxx - minx, 1.0)
    height_m = max(maxy - miny, 1.0)
    target_w = 1100.0
    target_h = 760.0
    pad = 36.0
    scale = min((target_w - 2 * pad) / width_m, (target_h - 2 * pad) / height_m)
    svg_w = width_m * scale + 2 * pad
    svg_h = height_m * scale + 2 * pad

    def project(x: float, y: float) -> tuple[float, float]:
        return (pad + (float(x) - minx) * scale, pad + (maxy - float(y)) * scale)

    layers: list[str] = []

    def add_layer(layer_id: str, paths: list[str]) -> None:
        layers.append(f'<g id="{html.escape(layer_id)}">\n' + "\n".join(paths) + "\n</g>")

    raw_paths = []
    conditioned_paths = []
    band_paths = []
    drivable_paths = []
    all_yaw_safe_paths = []
    maneuver_limited_paths = []
    unreachable_paths = []
    noise_paths = []
    zone_paths = []
    zone_label_paths = []
    width_sample_paths = []
    ridge_paths = []
    skeleton_node_paths = []
    task_proposal_paths = []
    task_branch_paths = []
    task_label_paths = []
    task_artifact_paths = []
    task_portal_paths = []
    task_terminal_paths = []
    task_coverage_region_paths = []
    task_infill_outline_paths = []
    task_stripe_region_paths = []
    task_body_pocket_paths = []
    operator_annotation_paths = []
    ownership_adjustment_paths = []
    task_service_axis_paths = []
    task_axis_candidate_paths = []
    outline_path_paths = []
    outline_start_paths = []
    outline_right_side_paths = []
    task_local_path_paths = []
    task_reverse_path_paths = []
    task_rejected_connector_paths = []
    task_direction_arrow_paths = []
    task_unsafe_paths = []
    task_turnaround_paths = []
    annotation_qa_paths = []
    annotation_qa_link_paths = []
    neck_paths = []

    zone_palette = [
        "#22c55e",
        "#f97316",
        "#8b5cf6",
        "#06b6d4",
        "#e11d48",
        "#84cc16",
        "#f59e0b",
        "#2563eb",
        "#ec4899",
        "#14b8a6",
    ]
    width_colors = {
        "too_tight": "#ef4444",
        "corridor_width": "#f59e0b",
        "body_width": "#22c55e",
    }
    qa_status_colors = {
        "matched": "#16a34a",
        "missed_by_auto": "#dc2626",
        "type_mismatch": "#8b5cf6",
        "mouth_misaligned": "#f97316",
        "auto_too_large": "#0ea5e9",
        "auto_too_small": "#eab308",
        "partial_match": "#f59e0b",
        "unmatched_auto_proposal": "#64748b",
    }
    sample_radius = max(1.1, min(3.2, float(result.get("conditioning", {}).get("width_sample_step_m", 0.20)) * scale * 0.12))

    for area in result.get("areas", []):
        geoms = area.get("geometries", {})
        portal_to_task: dict[str, str] = {}
        for proposal in area.get("task_proposals", []):
            for portal_id in proposal.get("entry_portals", []):
                portal_to_task.setdefault(str(portal_id), str(proposal.get("id", "")))
        for region in area.get("task_coverage_regions", []):
            task_type = str(region.get("task_type", "artifact"))
            color = TASK_TYPE_COLORS.get(task_type, "#64748b")
            task_id = html.escape(str(region.get("task_id", "")))
            title = (
                f"{region.get('coverage_region_id', '')} {task_type} coverage region, "
                f"area {float(region.get('area_m2', 0.0)):.2f} m2"
            )
            task_coverage_region_paths.append(
                f'<path class="task-hit task-coverage-region" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                f'd="{_svg_path_for_geometry(region.get("geometry", {}), project)}" fill="none" '
                f'stroke="{color}" stroke-width="1.8" stroke-opacity="0.90" stroke-dasharray="8 5" '
                'fill-rule="evenodd" vector-effect="non-scaling-stroke">'
                f'<title>{html.escape(title)}</title>'
                '</path>'
            )
        for adjustment in area.get("ownership_adjustments", []):
            task_type = str(adjustment.get("task_type", "artifact"))
            task_id = html.escape(str(adjustment.get("task_id", "")))
            color = TASK_TYPE_COLORS.get(task_type, "#f97316")
            title = (
                f"{adjustment.get('id', '')} ownership adjustment, "
                f"area {float(adjustment.get('area_m2', 0.0)):.2f} m2: "
                f"{adjustment.get('reason', '')}"
            )
            ownership_adjustment_paths.append(
                f'<path class="task-hit task-ownership-adjustment" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                f'd="{_svg_path_for_geometry(adjustment.get("geometry", {}), project)}" fill="{color}" '
                'fill-opacity="0.28" stroke="#f97316" stroke-width="1.7" stroke-dasharray="3 3" '
                'fill-rule="evenodd" vector-effect="non-scaling-stroke">'
                f'<title>{html.escape(title)}</title>'
                '</path>'
            )
        for annotation in area.get("operator_annotations", []):
            annotation_id = html.escape(str(annotation.get("id", "")))
            status = str(annotation.get("status", "unknown"))
            stroke = "#111827" if status == "accepted" else "#991b1b"
            title = f"{annotation.get('id', '')} {status}: {annotation.get('reason', '')}"
            cut_line = annotation.get("cut_line", [])
            if isinstance(cut_line, list) and len(cut_line) >= 2:
                start = cut_line[0]
                end = cut_line[1]
                if len(start) >= 2 and len(end) >= 2:
                    x1, y1 = project(float(start[0]), float(start[1]))
                    x2, y2 = project(float(end[0]), float(end[1]))
                    operator_annotation_paths.append(
                        f'<line class="operator-annotation" data-annotation-id="{annotation_id}" '
                        f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                        f'stroke="{stroke}" stroke-width="3.0" stroke-opacity="0.90" '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>{html.escape(title)}</title>'
                        '</line>'
                    )
            service_point = annotation.get("service_point", [])
            if isinstance(service_point, list) and len(service_point) >= 2:
                sx, sy = project(float(service_point[0]), float(service_point[1]))
                operator_annotation_paths.append(
                    f'<circle class="operator-annotation" data-annotation-id="{annotation_id}" '
                    f'cx="{sx:.2f}" cy="{sy:.2f}" r="5.5" fill="#ffffff" stroke="{stroke}" stroke-width="2.2">'
                    f'<title>{html.escape(title)}</title>'
                    '</circle>'
                )
        annotation_qa = area.get("annotation_qa", {})
        for match in annotation_qa.get("matches", []):
            status = str(match.get("status", "unknown"))
            color = qa_status_colors.get(status, "#64748b")
            task_id = html.escape(str(match.get("operator_task_id", "")))
            title = (
                f"{match.get('operator_label', '')} vs {match.get('matched_auto_label', 'none')}: "
                f"{status}, IoU {float(match.get('iou') or 0.0):.2f}, "
                f"{match.get('reason', '')}"
            )
            annotation_qa_paths.append(
                f'<path class="task-hit annotation-qa annotation-qa-operator" data-task-id="{task_id}" '
                f'd="{_svg_path_for_geometry(match.get("operator_geometry", {}), project)}" fill="none" '
                f'stroke="{color}" stroke-width="3.0" stroke-opacity="0.95" stroke-dasharray="2 4" '
                'fill-rule="evenodd" vector-effect="non-scaling-stroke">'
                f'<title>{html.escape(title)}</title>'
                '</path>'
            )
            matched_auto_geom = match.get("matched_auto_geometry", {})
            if matched_auto_geom:
                annotation_qa_paths.append(
                    f'<path class="task-hit annotation-qa annotation-qa-auto" data-task-id="{task_id}" '
                    f'd="{_svg_path_for_geometry(matched_auto_geom, project)}" fill="none" '
                    f'stroke="{color}" stroke-width="2.2" stroke-opacity="0.72" stroke-dasharray="9 5" '
                    'fill-rule="evenodd" vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(title)}</title>'
                    '</path>'
                )
            operator_label_point = match.get("operator_label_point", [])
            auto_label_point = match.get("matched_auto_label_point", [])
            if len(operator_label_point) >= 2 and len(auto_label_point) >= 2:
                x1, y1 = project(float(operator_label_point[0]), float(operator_label_point[1]))
                x2, y2 = project(float(auto_label_point[0]), float(auto_label_point[1]))
                annotation_qa_link_paths.append(
                    f'<line class="task-hit annotation-qa-link" data-task-id="{task_id}" '
                    f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                    f'stroke="{color}" stroke-width="1.9" stroke-opacity="0.82" stroke-dasharray="4 4" '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(title)}</title>'
                    '</line>'
                )
        for unmatched in annotation_qa.get("unmatched_auto_proposals", []):
            color = qa_status_colors["unmatched_auto_proposal"]
            task_id = html.escape(str(unmatched.get("auto_task_id", "")))
            title = (
                f"{unmatched.get('auto_label', '')}: unmatched automatic proposal, "
                f"area {float(unmatched.get('area_m2') or 0.0):.2f} m2"
            )
            annotation_qa_paths.append(
                f'<path class="task-hit annotation-qa annotation-qa-unmatched-auto" data-task-id="{task_id}" '
                f'd="{_svg_path_for_geometry(unmatched.get("geometry", {}), project)}" fill="none" '
                f'stroke="{color}" stroke-width="2.4" stroke-opacity="0.78" stroke-dasharray="6 4" '
                'fill-rule="evenodd" vector-effect="non-scaling-stroke">'
                f'<title>{html.escape(title)}</title>'
                '</path>'
            )
        raw_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("raw_lawn", {}), project)}" fill="none" '
            'stroke="#475569" stroke-width="2.2" stroke-dasharray="5 4"/>'
        )
        conditioned_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("conditioned_lawn", {}), project)}" fill="none" '
            'stroke="#0f172a" stroke-width="2.4"/>'
        )
        band_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("coverage_band", {}), project)}" fill="#bef264" '
            'fill-opacity="0.26" stroke="none" fill-rule="evenodd"/>'
        )
        drivable_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("drivable_region", {}), project)}" fill="#bbf7d0" '
            'fill-opacity="0.38" stroke="#16a34a" stroke-width="1.2" fill-rule="evenodd"/>'
        )
        maneuver_limited_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("maneuver_limited_area", {}), project)}" fill="#facc15" '
            'fill-opacity="0.24" stroke="none" fill-rule="evenodd"/>'
        )
        all_yaw_safe_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("all_yaw_safe_region", {}), project)}" fill="none" '
            'stroke="#2563eb" stroke-width="1.4" stroke-dasharray="7 5" fill-rule="evenodd"/>'
        )
        unreachable_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("unreachable", {}), project)}" fill="#ef4444" '
            'fill-opacity="0.42" stroke="#991b1b" stroke-width="1.0" fill-rule="evenodd"/>'
        )
        noise_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("removed_noise", {}), project)}" fill="#f97316" '
            'fill-opacity="0.42" stroke="#9a3412" stroke-width="1.0" fill-rule="evenodd"/>'
        )
        noise_paths.append(
            f'<path d="{_svg_path_for_geometry(geoms.get("filled_smooth_area", {}), project)}" fill="#a855f7" '
            'fill-opacity="0.32" stroke="#6b21a8" stroke-width="1.0" fill-rule="evenodd"/>'
        )
        for zone in area.get("zones", []):
            display_index = int(zone.get("display_index", len(zone_paths) + 1))
            color = zone_palette[(display_index - 1) % len(zone_palette)]
            label = str(zone.get("label", f"Z{display_index}"))
            title = (
                f"{zone.get('id', '')} {zone.get('kind', '')}, "
                f"area {float(zone.get('area_m2', 0.0)):.2f} m2, "
                f"{zone.get('reason', '')}"
            )
            zone_paths.append(
                f'<path d="{_svg_path_for_geometry(zone.get("geometry", {}), project)}" fill="{color}" '
                'fill-opacity="0.62" stroke="#111827" stroke-width="2.2" fill-rule="evenodd" '
                'vector-effect="non-scaling-stroke">'
                f'<title>{html.escape(title)}</title>'
                '</path>'
            )
            label_point = zone.get("label_point") or []
            if len(label_point) >= 2:
                sx, sy = project(float(label_point[0]), float(label_point[1]))
                marker_radius = 14.0
                zone_label_paths.append(
                    f'<g class="zone-label" data-zone-id="{html.escape(str(zone.get("id", "")))}">'
                    f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="{marker_radius:.2f}" fill="#ffffff" '
                    f'stroke="{color}" stroke-width="3.0"/>'
                    f'<text x="{sx:.2f}" y="{sy + 4.5:.2f}" text-anchor="middle" '
                    'font-family="system-ui, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif" '
                    'font-size="12" font-weight="800" fill="#111827">'
                    f'{html.escape(label)}</text>'
                    f'<title>{html.escape(title)}</title>'
                    '</g>'
                )
        for sample in area.get("width_samples", []):
            color = width_colors.get(str(sample.get("class", "")), "#64748b")
            sx, sy = project(float(sample.get("x", 0.0)), float(sample.get("y", 0.0)))
            width_sample_paths.append(
                f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="{sample_radius:.2f}" fill="{color}" fill-opacity="0.36">'
                f'<title>{html.escape(str(sample.get("class", "")))} width {float(sample.get("local_width_m", 0.0)):.2f} m, clearance {float(sample.get("clearance_m", 0.0)):.2f} m</title>'
                '</circle>'
            )
        for link in area.get("ridge_links", []):
            line = link.get("line", [])
            if len(line) < 2:
                continue
            x1, y1 = project(float(line[0][0]), float(line[0][1]))
            x2, y2 = project(float(line[1][0]), float(line[1][1]))
            ridge_paths.append(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                'stroke="#7c3aed" stroke-width="1.3" stroke-opacity="0.78"/>'
            )
        for point in area.get("ridge_points", []):
            sx, sy = project(float(point.get("x", 0.0)), float(point.get("y", 0.0)))
            ridge_paths.append(
                f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="{max(1.4, sample_radius * 0.8):.2f}" '
                'fill="#581c87" fill-opacity="0.82">'
                f'<title>ridge clearance {float(point.get("clearance_m", 0.0)):.2f} m</title>'
                '</circle>'
            )
        for node in area.get("skeleton_graph", {}).get("nodes", []):
            sx, sy = project(float(node.get("x", 0.0)), float(node.get("y", 0.0)))
            node_type = str(node.get("node_type", "chain"))
            if node_type == "branch_point":
                fill = "#db2777"
                radius = max(2.6, sample_radius * 1.2)
            elif node_type == "endpoint":
                fill = "#111827"
                radius = max(2.2, sample_radius)
            else:
                fill = "#7c3aed"
                radius = max(1.2, sample_radius * 0.6)
            skeleton_node_paths.append(
                f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="{radius:.2f}" fill="{fill}" fill-opacity="0.86">'
                f'<title>{html.escape(node_type)} degree {int(node.get("degree", 0))}, width {float(node.get("local_width_m", 0.0)):.2f} m</title>'
                '</circle>'
            )
        for proposal in area.get("task_proposals", []):
            task_type = str(proposal.get("task_type", "artifact"))
            color = TASK_TYPE_COLORS.get(task_type, "#64748b")
            task_id = html.escape(str(proposal.get("id", "")))
            visible_by_default = bool(proposal.get("visible_by_default", True))
            title = (
                f"{proposal.get('id', '')} {task_type}, confidence "
                f"{float(proposal.get('confidence', 0.0)):.2f}: "
                f"{proposal.get('evidence', {}).get('reason', '')}"
            )
            shape_markup = (
                f'<path class="task-hit task-shape" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                f'd="{_svg_path_for_geometry(proposal.get("geometry", {}), project)}" fill="{color}" '
                'fill-opacity="0.36" stroke="none" fill-rule="evenodd">'
                f'<title>{html.escape(title)}</title>'
                '</path>'
            )
            if visible_by_default:
                task_proposal_paths.append(shape_markup)
            else:
                task_artifact_paths.append(shape_markup)
            for segment in proposal.get("line_segments", []):
                if len(segment) < 2:
                    continue
                x1, y1 = project(float(segment[0][0]), float(segment[0][1]))
                x2, y2 = project(float(segment[1][0]), float(segment[1][1]))
                branch_markup = (
                    f'<line class="task-hit task-branch" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                    f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                    f'stroke="{color}" stroke-width="3.0" stroke-opacity="0.92" vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(title)}</title>'
                    '</line>'
                )
                if visible_by_default:
                    task_branch_paths.append(branch_markup)
                else:
                    task_artifact_paths.append(branch_markup)
            label_point = proposal.get("label_point", [])
            if visible_by_default and len(label_point) >= 2:
                sx, sy = project(float(label_point[0]), float(label_point[1]))
                label = html.escape(str(proposal.get("display_label", _task_display_label(task_type))))
                text_width = max(36.0, len(label) * 7.6 + 14.0)
                task_label_paths.append(
                    f'<g class="task-hit task-label" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}">'
                    f'<rect x="{sx - text_width / 2:.2f}" y="{sy - 12.5:.2f}" width="{text_width:.2f}" height="21.0" '
                    f'fill="#ffffff" fill-opacity="0.92" stroke="{color}" stroke-width="2.0" rx="4.0"/>'
                    f'<text x="{sx:.2f}" y="{sy + 3.8:.2f}" text-anchor="middle" '
                    'font-family="system-ui, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif" '
                    'font-size="10.5" font-weight="800" fill="#111827">'
                    f'{label}</text>'
                    f'<title>{html.escape(title)}</title>'
                    '</g>'
                )
            terminal_cap = proposal.get("terminal_cap") or {}
            if visible_by_default and terminal_cap:
                for segment in terminal_cap.get("line_segments", []):
                    if len(segment) < 2:
                        continue
                    x1, y1 = project(float(segment[0][0]), float(segment[0][1]))
                    x2, y2 = project(float(segment[1][0]), float(segment[1][1]))
                    task_terminal_paths.append(
                        f'<line class="task-hit task-terminal" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                        f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                        'stroke="#111827" stroke-width="3.2" stroke-opacity="0.95" '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>terminal end, depth {float(terminal_cap.get("depth_m", 0.0)):.2f} m</title>'
                        '</line>'
                    )
                end_label = terminal_cap.get("label_point", [])
                if len(end_label) >= 2:
                    ex, ey = project(float(end_label[0]), float(end_label[1]))
                    task_terminal_paths.append(
                        f'<g class="task-hit task-terminal-label" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}">'
                        f'<rect x="{ex - 16.0:.2f}" y="{ey - 11.5:.2f}" width="32.0" height="19.0" '
                        'fill="#ffffff" fill-opacity="0.95" stroke="#111827" stroke-width="2.0" rx="4.0"/>'
                        f'<text x="{ex:.2f}" y="{ey + 3.2:.2f}" text-anchor="middle" '
                        'font-family="system-ui, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif" '
                        'font-size="9.2" font-weight="850" fill="#111827">END</text>'
                        f'<title>terminal end, depth {float(terminal_cap.get("depth_m", 0.0)):.2f} m</title>'
                        '</g>'
                    )
        for outline_path in area.get("outline_paths", []):
            outline_id = html.escape(str(outline_path.get("outline_path_id", "")))
            title = (
                f"{outline_path.get('outline_path_id', '')} V2 outline, "
                f"mode {outline_path.get('outline_generation_mode', outline_path.get('strategy', 'unknown'))}, "
                f"offset {float(outline_path.get('centerline_offset_m', 0.0)):.2f} m, "
                f"length {float(outline_path.get('path_length_m', 0.0)):.2f} m"
            )
            if isinstance(outline_path.get("right_footprint_side_min_clearance_m"), (int, float)):
                title += (
                    f", right footprint side min/median/max "
                    f"{float(outline_path.get('right_footprint_side_min_clearance_m', 0.0)):.2f}/"
                    f"{float(outline_path.get('right_footprint_side_median_clearance_m', 0.0)):.2f}/"
                    f"{float(outline_path.get('right_footprint_side_max_clearance_m', 0.0)):.2f} m"
                )
            elif isinstance(outline_path.get("right_cut_edge_min_clearance_m"), (int, float)):
                title += (
                    f", right edge min/median/max "
                    f"{float(outline_path.get('right_cut_edge_min_clearance_m', 0.0)):.2f}/"
                    f"{float(outline_path.get('right_cut_edge_median_clearance_m', 0.0)):.2f}/"
                    f"{float(outline_path.get('right_cut_edge_max_clearance_m', 0.0)):.2f} m"
                )
            for segment in outline_path.get("segments", []):
                tool_points = segment.get("tool_points", [])
                if len(tool_points) >= 2:
                    projected_points = []
                    for point in tool_points:
                        sx, sy = project(float(point["x"]), float(point["y"]))
                        projected_points.append(f"{sx:.2f},{sy:.2f}")
                    outline_path_paths.append(
                        f'<polyline class="v2-outline-path" data-outline-id="{outline_id}" '
                        f'points="{" ".join(projected_points)}" fill="none" '
                        'stroke="#14532d" stroke-width="3.3" stroke-opacity="0.96" '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>{html.escape(title)}</title>'
                        '</polyline>'
                    )
                    start_point = tool_points[0]
                    sx, sy = project(float(start_point["x"]), float(start_point["y"]))
                    selector = str(outline_path.get("outline_start_selector", "outline start"))
                    run_length = float(outline_path.get("outline_start_straight_run_m", 0.0))
                    start_title = (
                        f"{outline_path.get('outline_path_id', '')} start, "
                        f"{selector.replace('_', ' ')}, straight run {run_length:.2f} m"
                    )
                    outline_start_paths.append(
                        f'<g class="v2-outline-start" data-outline-id="{outline_id}">'
                        f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="5.4" fill="#facc15" stroke="#713f12" stroke-width="2.2" '
                        'vector-effect="non-scaling-stroke"/>'
                        f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="2.0" fill="#713f12"/>'
                        f'<title>{html.escape(start_title)}</title>'
                        '</g>'
                    )
                right_side_points = segment.get("right_footprint_side_points", [])
                if len(right_side_points) >= 2:
                    projected_right_points = []
                    for point in right_side_points:
                        sx, sy = project(float(point["x"]), float(point["y"]))
                        projected_right_points.append(f"{sx:.2f},{sy:.2f}")
                    outline_right_side_paths.append(
                        f'<polyline class="v2-outline-right-side" data-outline-id="{outline_id}" '
                        f'points="{" ".join(projected_right_points)}" fill="none" '
                        'stroke="#ea580c" stroke-width="2.4" stroke-opacity="0.95" '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>{html.escape(title)}</title>'
                        '</polyline>'
                    )
                for side_segment in segment.get("right_footprint_side_sample_segments", []):
                    if len(side_segment) < 2:
                        continue
                    x1, y1 = project(float(side_segment[0][0]), float(side_segment[0][1]))
                    x2, y2 = project(float(side_segment[1][0]), float(side_segment[1][1]))
                    outline_right_side_paths.append(
                        f'<line class="v2-outline-right-side-sample" data-outline-id="{outline_id}" '
                        f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                        'stroke="#fb923c" stroke-width="1.3" stroke-opacity="0.65" '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>{html.escape(title)}</title>'
                        '</line>'
                    )
                for unsafe in segment.get("unsafe_samples", []):
                    sx, sy = project(float(unsafe.get("x", 0.0)), float(unsafe.get("y", 0.0)))
                    task_unsafe_paths.append(
                        f'<g class="task-unsafe-sample" data-outline-id="{outline_id}">'
                        f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="5.0" fill="#ffffff" stroke="#dc2626" stroke-width="2.2"/>'
                        f'<line x1="{sx - 3.3:.2f}" y1="{sy - 3.3:.2f}" x2="{sx + 3.3:.2f}" y2="{sy + 3.3:.2f}" stroke="#dc2626" stroke-width="1.8"/>'
                        f'<line x1="{sx + 3.3:.2f}" y1="{sy - 3.3:.2f}" x2="{sx - 3.3:.2f}" y2="{sy + 3.3:.2f}" stroke="#dc2626" stroke-width="1.8"/>'
                        f'<title>{html.escape(str(unsafe.get("reason", "")))}</title>'
                        '</g>'
                    )
        for task_path in area.get("task_paths", []):
            task_type = str(task_path.get("task_type", "artifact"))
            task_id = html.escape(str(task_path.get("task_id", "")))
            local_stroke = "#1d4ed8"
            reverse_stroke = "#dc2626"
            axis_stroke = "#f59e0b"
            stripe_region = task_path.get("stripe_region") or {}
            infill_region = stripe_region or task_path.get("coverage_region") or {}
            if infill_region:
                infill_source = (
                    task_path.get("stripe_region_source")
                    if stripe_region
                    else "task_coverage_region"
                )
                infill_title = (
                    f"{task_path.get('task_path_id', '')} infill area outline, "
                    f"source {infill_source}, "
                    f"area {float(task_path.get('stripe_region_area_m2', task_path.get('coverage_region_area_m2', 0.0))):.2f} m2. "
                    "This is the polygon used to generate or clip the visible fill strokes."
                )
                task_infill_outline_paths.append(
                    f'<path class="task-hit task-infill-outline" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                    f'd="{_svg_path_for_geometry(infill_region, project)}" fill="none" '
                    'stroke="#f97316" stroke-width="2.6" stroke-dasharray="10 5" fill-rule="evenodd" '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(infill_title)}</title>'
                    '</path>'
                )
            if stripe_region:
                stripe_title = (
                    f"{task_path.get('task_path_id', '')} stripe-safe area, "
                    f"target boundary clearance {float(task_path.get('stripe_boundary_clearance_m', 0.0)):.2f} m, "
                    f"applied drivable inset {float(task_path.get('stripe_drivable_inset_m', 0.0)):.2f} m"
                )
                task_stripe_region_paths.append(
                    f'<path class="task-hit task-stripe-region" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                    f'd="{_svg_path_for_geometry(stripe_region, project)}" fill="#a78bfa" fill-opacity="0.12" '
                    'stroke="#7c3aed" stroke-width="1.6" stroke-dasharray="5 4" fill-rule="evenodd" '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(stripe_title)}</title>'
                    '</path>'
                )
            for pocket in task_path.get("body_service_pockets", []):
                pocket_geom = pocket.get("geometry") or {}
                if not pocket_geom:
                    continue
                pocket_title = (
                    f"{pocket.get('id', '')} BODY pocket service, "
                    f"area {float(pocket.get('area_m2', 0.0)):.2f} m2, "
                    f"strokes {int(pocket.get('stroke_count', 0))}, "
                    f"axis {float(pocket.get('axis_angle_deg', 0.0)):.1f} deg "
                    f"from {str(pocket.get('axis_source', 'unknown')).replace('_', ' ')}, "
                    f"boundary {str(pocket.get('boundary_source', 'unknown')).replace('_', ' ')}: "
                    f"{pocket.get('reason', '')}"
                )
                task_body_pocket_paths.append(
                    f'<path class="task-hit task-body-pocket" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                    f'd="{_svg_path_for_geometry(pocket_geom, project)}" fill="#facc15" fill-opacity="0.24" '
                    'stroke="#ca8a04" stroke-width="1.8" stroke-dasharray="4 3" fill-rule="evenodd" '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(pocket_title)}</title>'
                    '</path>'
                )
            title = (
                f"{task_path.get('task_path_id', '')} {task_type}, "
                f"{task_path.get('strategy', '')}, coverage "
                f"{float(task_path.get('estimated_coverage_percent', 0.0)):.1f}%"
            )
            service_axis = task_path.get("service_axis") or {}
            axis_start = service_axis.get("start", [])
            axis_end = service_axis.get("end", [])
            if len(axis_start) >= 2 and len(axis_end) >= 2:
                x1, y1 = project(float(axis_start[0]), float(axis_start[1]))
                x2, y2 = project(float(axis_end[0]), float(axis_end[1]))
                axis_title = (
                    f"{task_path.get('task_path_id', '')} service axis from "
                    f"{service_axis.get('source', '')}, length "
                    f"{float(service_axis.get('length_m', 0.0)):.2f} m"
                )
                task_service_axis_paths.append(
                    f'<line class="task-hit task-service-axis" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                    f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                    f'stroke="{axis_stroke}" stroke-width="2.4" stroke-opacity="0.96" stroke-dasharray="4 3" '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(axis_title)}</title>'
                    '</line>'
                )
                arrow = _svg_arrow_polygon(
                    (float(axis_start[0]), float(axis_start[1])),
                    (float(axis_end[0]), float(axis_end[1])),
                    project,
                    size=8.0,
                )
                if arrow:
                    task_service_axis_paths.append(
                        f'<polygon class="task-hit task-service-axis-arrow" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                        f'points="{arrow}" fill="{axis_stroke}" fill-opacity="0.96">'
                        f'<title>{html.escape(axis_title)}</title>'
                        '</polygon>'
                    )
            for candidate in task_path.get("axis_candidates", []):
                if candidate.get("selected"):
                    continue
                candidate_start = candidate.get("start", [])
                candidate_end = candidate.get("end", [])
                if len(candidate_start) < 2 or len(candidate_end) < 2:
                    continue
                x1, y1 = project(float(candidate_start[0]), float(candidate_start[1]))
                x2, y2 = project(float(candidate_end[0]), float(candidate_end[1]))
                candidate_title = (
                    f"{task_path.get('task_path_id', '')} rejected axis "
                    f"{candidate.get('rank', '?')}: {candidate.get('source', '')}, "
                    f"score {float(candidate.get('score', 0.0)):.1f}, "
                    f"coverage {float(candidate.get('coverage_percent', 0.0)):.1f}%, "
                    f"lanes {int(candidate.get('lane_count', 0))}, "
                    f"unsafe {int(candidate.get('unsafe_sample_count', 0))}"
                )
                task_axis_candidate_paths.append(
                    f'<line class="task-hit task-axis-candidate" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                    f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                    'stroke="#64748b" stroke-width="1.5" stroke-opacity="0.56" stroke-dasharray="3 5" '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(candidate_title)}</title>'
                    '</line>'
                )
            turnaround = task_path.get("turnaround") or {}
            envelope = turnaround.get("envelope", {})
            if envelope:
                feasible = bool(turnaround.get("feasible"))
                fill = "#22c55e" if feasible else "#ef4444"
                task_turnaround_paths.append(
                    f'<path class="task-hit task-turnaround-envelope" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                    f'd="{_svg_path_for_geometry(envelope, project)}" fill="{fill}" fill-opacity="0.20" '
                    f'stroke="{fill}" stroke-width="1.8" stroke-dasharray="5 4" fill-rule="evenodd" '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(str(turnaround.get("reason", "")))}</title>'
                    '</path>'
                )
            for segment in task_path.get("segments", []):
                direction = str(segment.get("direction", "forward"))
                phase = str(segment.get("phase", ""))
                is_connector = phase.endswith("_connector") or bool(segment.get("connector_kind"))
                stroke = "#f97316" if is_connector else (reverse_stroke if direction == "backward" else local_stroke)
                target_paths = task_reverse_path_paths if direction == "backward" else task_local_path_paths
                dash = ' stroke-dasharray="6 4"' if is_connector else (' stroke-dasharray="7 5"' if direction == "backward" else "")
                display_points = segment.get("tool_points", []) or segment.get("base_poses", [])
                if len(display_points) >= 2:
                    projected_points = []
                    for point in display_points:
                        sx, sy = project(float(point["x"]), float(point["y"]))
                        projected_points.append(f"{sx:.2f},{sy:.2f}")
                    target_paths.append(
                        f'<polyline class="task-hit task-local-path" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                        f'points="{" ".join(projected_points)}" fill="none" '
                        f'stroke="{stroke}" stroke-width="2.9" stroke-opacity="0.95"{dash} '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>{html.escape(title)}; {html.escape(direction)} {html.escape(phase)}</title>'
                        '</polyline>'
                    )
                    start = (float(display_points[0]["x"]), float(display_points[0]["y"]))
                    end = (float(display_points[-1]["x"]), float(display_points[-1]["y"]))
                    arrow = _svg_arrow_polygon(start, end, project)
                    if arrow:
                        task_direction_arrow_paths.append(
                            f'<polygon class="task-hit task-direction-arrow" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                            f'points="{arrow}" fill="{stroke}" fill-opacity="0.95">'
                            f'<title>{html.escape(title)} direction {html.escape(direction)}</title>'
                            '</polygon>'
                        )
                for unsafe in segment.get("unsafe_samples", []):
                    sx, sy = project(float(unsafe.get("x", 0.0)), float(unsafe.get("y", 0.0)))
                    task_unsafe_paths.append(
                        f'<g class="task-hit task-unsafe-sample" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}">'
                        f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="5.0" fill="#ffffff" stroke="#dc2626" stroke-width="2.2"/>'
                        f'<line x1="{sx - 3.3:.2f}" y1="{sy - 3.3:.2f}" x2="{sx + 3.3:.2f}" y2="{sy + 3.3:.2f}" stroke="#dc2626" stroke-width="1.8"/>'
                        f'<line x1="{sx + 3.3:.2f}" y1="{sy - 3.3:.2f}" x2="{sx - 3.3:.2f}" y2="{sy + 3.3:.2f}" stroke="#dc2626" stroke-width="1.8"/>'
                        f'<title>{html.escape(str(unsafe.get("reason", "")))}</title>'
                        '</g>'
                    )
            for record in (task_path.get("compact_connector_summary") or {}).get("records", []):
                if str(record.get("status", "")).startswith("inserted"):
                    continue
                reason = str(record.get("reason", "rejected"))
                from_segment = str(record.get("from_segment_id", ""))
                to_segment = str(record.get("to_segment_id", ""))
                connector_style = str(record.get("connector_style", "connector")).replace("_", " ")
                rejected_title = (
                    f"{task_path.get('task_path_id', '')} rejected {connector_style} connector "
                    f"{from_segment} -> {to_segment}, "
                    f"gap {float(record.get('gap_distance_m', 0.0)):.2f} m, "
                    f"heading {float(record.get('heading_delta_deg', 0.0)):.0f} deg: {reason}"
                )
                diagnostic = record.get("diagnostic_segment") or {}
                diagnostic_points = diagnostic.get("base_poses", []) or diagnostic.get("tool_points", [])
                if len(diagnostic_points) >= 2:
                    projected_points = []
                    for point in diagnostic_points:
                        sx, sy = project(float(point["x"]), float(point["y"]))
                        projected_points.append(f"{sx:.2f},{sy:.2f}")
                    task_rejected_connector_paths.append(
                        f'<polyline class="task-hit task-rejected-connector" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                        f'points="{" ".join(projected_points)}" fill="none" '
                        'stroke="#b91c1c" stroke-width="3.2" stroke-opacity="0.92" stroke-dasharray="3 4" '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>{html.escape(rejected_title)}</title>'
                        '</polyline>'
                    )
                    for unsafe in diagnostic.get("unsafe_samples", []):
                        sx, sy = project(float(unsafe.get("x", 0.0)), float(unsafe.get("y", 0.0)))
                        task_rejected_connector_paths.append(
                            f'<g class="task-hit task-rejected-connector-unsafe" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}">'
                            f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="4.4" fill="#fee2e2" stroke="#991b1b" stroke-width="2.0"/>'
                            f'<line x1="{sx - 3.0:.2f}" y1="{sy - 3.0:.2f}" x2="{sx + 3.0:.2f}" y2="{sy + 3.0:.2f}" stroke="#991b1b" stroke-width="1.6"/>'
                            f'<line x1="{sx + 3.0:.2f}" y1="{sy - 3.0:.2f}" x2="{sx - 3.0:.2f}" y2="{sy + 3.0:.2f}" stroke="#991b1b" stroke-width="1.6"/>'
                            f'<title>{html.escape(rejected_title)}</title>'
                            '</g>'
                        )
                    continue
                for line in record.get("diagnostic_line_segments", []):
                    if len(line) < 2:
                        continue
                    x1, y1 = project(float(line[0][0]), float(line[0][1]))
                    x2, y2 = project(float(line[1][0]), float(line[1][1]))
                    task_rejected_connector_paths.append(
                        f'<line class="task-hit task-rejected-connector" data-task-id="{task_id}" data-task-type="{html.escape(task_type)}" '
                        f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                        'stroke="#7f1d1d" stroke-width="2.4" stroke-opacity="0.70" stroke-dasharray="2 5" '
                        'vector-effect="non-scaling-stroke">'
                        f'<title>{html.escape(rejected_title)}</title>'
                        '</line>'
                    )
        for portal in area.get("portal_candidates", []):
            accepted = portal.get("status") == "accepted"
            stroke = "#0f172a" if accepted else "#94a3b8"
            dash = "" if accepted else ' stroke-dasharray="4 4"'
            width = "2.6" if accepted else "1.4"
            opacity = "0.9" if accepted else "0.45"
            portal_id = html.escape(str(portal.get("id", "")))
            task_id = html.escape(portal_to_task.get(str(portal.get("id", "")), ""))
            portal_search = portal.get("portal_search") or {}
            search_note = ""
            if portal_search:
                search_note = (
                    f", search shift {float(portal_search.get('selected_shift_toward_body_m', 0.0)):.2f} m, "
                    f"service area {float(portal_search.get('selected_service_area_m2', 0.0)):.2f} m2"
                )
            for segment in portal.get("line_segments", []):
                if len(segment) < 2:
                    continue
                x1, y1 = project(float(segment[0][0]), float(segment[0][1]))
                x2, y2 = project(float(segment[1][0]), float(segment[1][1]))
                task_portal_paths.append(
                    f'<line class="portal-hit" data-portal-id="{portal_id}" data-task-id="{task_id}" '
                    f'x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                    f'stroke="{stroke}" stroke-width="{width}" stroke-opacity="{opacity}"{dash} '
                    'vector-effect="non-scaling-stroke">'
                    f'<title>{html.escape(str(portal.get("status", "")))} portal, width {float(portal.get("width_m", 0.0)):.2f} m{html.escape(search_note)}: {html.escape(str(portal.get("reason", "")))}</title>'
                    '</line>'
                )
            center = portal.get("center", [])
            if len(center) >= 2:
                sx, sy = project(float(center[0]), float(center[1]))
                task_portal_paths.append(
                    f'<circle class="portal-hit" data-portal-id="{portal_id}" data-task-id="{task_id}" '
                    f'cx="{sx:.2f}" cy="{sy:.2f}" r="{max(2.4, sample_radius):.2f}" '
                    f'fill="{stroke}" fill-opacity="{opacity}">'
                    f'<title>{html.escape(str(portal.get("id", "")))}</title>'
                    '</circle>'
                )
                label_point = portal.get("label_point", [])
                if accepted and len(label_point) >= 2:
                    lx, ly = project(float(label_point[0]), float(label_point[1]))
                    label = html.escape(str(portal.get("display_label", "")))
                    task_portal_paths.append(
                        f'<g class="portal-hit portal-label" data-portal-id="{portal_id}" data-task-id="{task_id}">'
                        f'<circle cx="{lx:.2f}" cy="{ly - 12.0:.2f}" r="10.0" fill="#ffffff" fill-opacity="0.95" '
                        f'stroke="{stroke}" stroke-width="2.0"/>'
                        f'<text x="{lx:.2f}" y="{ly - 8.5:.2f}" text-anchor="middle" '
                        'font-family="system-ui, -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif" '
                        'font-size="9.5" font-weight="800" fill="#111827">'
                        f'{label}</text>'
                        f'<title>{html.escape(str(portal.get("id", "")))}</title>'
                        '</g>'
                    )
        for candidate in area.get("neck_candidates", []):
            accepted = candidate.get("status") == "accepted"
            stroke = "#dc2626" if accepted else "#64748b"
            width = "2.4" if accepted else "1.3"
            dash = "" if accepted else ' stroke-dasharray="5 4"'
            opacity = "0.92" if accepted else "0.48"
            for segment in candidate.get("line_segments", []):
                if len(segment) < 2:
                    continue
                x1, y1 = project(float(segment[0][0]), float(segment[0][1]))
                x2, y2 = project(float(segment[1][0]), float(segment[1][1]))
                neck_paths.append(
                    f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                    f'stroke="{stroke}" stroke-width="{width}" stroke-opacity="{opacity}"{dash}>'
                    f'<title>{html.escape(str(candidate.get("status", "")))} neck, width {float(candidate.get("width_m", 0.0)):.2f} m, score {float(candidate.get("score", 0.0)):.2f}: {html.escape(str(candidate.get("reason", "")))}</title>'
                    '</line>'
                )

    add_layer("layer-coverage-band", band_paths)
    add_layer("layer-drivable", drivable_paths)
    add_layer("layer-width-heatmap", width_sample_paths)
    add_layer("layer-maneuver-limited", maneuver_limited_paths)
    add_layer("layer-all-yaw-safe", all_yaw_safe_paths)
    add_layer("layer-ridge", ridge_paths)
    add_layer("layer-skeleton-nodes", skeleton_node_paths)
    add_layer("layer-task-coverage-regions", task_coverage_region_paths)
    add_layer("layer-task-infill-outlines", task_infill_outline_paths)
    add_layer("layer-task-stripe-regions", task_stripe_region_paths)
    add_layer("layer-task-body-pockets", task_body_pocket_paths)
    add_layer("layer-operator-annotations", operator_annotation_paths)
    add_layer("layer-service-ownership", ownership_adjustment_paths)
    add_layer("layer-task-proposals", task_proposal_paths)
    add_layer("layer-task-branches", task_branch_paths)
    add_layer("layer-task-service-axes", task_service_axis_paths)
    add_layer("layer-task-axis-candidates", task_axis_candidate_paths)
    add_layer("layer-v2-outline-paths", outline_path_paths)
    add_layer("layer-v2-outline-right-side", outline_right_side_paths)
    add_layer("layer-v2-outline-starts", outline_start_paths)
    add_layer("layer-task-turnaround-envelopes", task_turnaround_paths)
    add_layer("layer-task-local-paths", task_local_path_paths)
    add_layer("layer-task-reverse-paths", task_reverse_path_paths)
    add_layer("layer-task-rejected-connectors", task_rejected_connector_paths)
    add_layer("layer-task-direction-arrows", task_direction_arrow_paths)
    add_layer("layer-task-unsafe-samples", task_unsafe_paths)
    add_layer("layer-task-labels", task_label_paths)
    add_layer("layer-task-artifacts", task_artifact_paths)
    add_layer("layer-task-portals", task_portal_paths)
    add_layer("layer-task-terminals", task_terminal_paths)
    add_layer("layer-annotation-qa", annotation_qa_paths)
    add_layer("layer-annotation-qa-links", annotation_qa_link_paths)
    add_layer("layer-neck-cuts", neck_paths)
    add_layer("layer-unreachable", unreachable_paths)
    add_layer("layer-noise", noise_paths)
    add_layer("layer-zones", zone_paths)
    add_layer("layer-conditioned", conditioned_paths)
    add_layer("layer-raw", raw_paths)
    add_layer("layer-zone-labels", zone_label_paths)

    model = result.get("mower_model", {})
    footprint_w = float(model.get("safety_footprint_width_m", 0.78))
    footprint_l = float(model.get("safety_footprint_length_m", 0.92))
    marker_x = pad
    marker_y = svg_h - pad - footprint_l * scale
    mower_marker = (
        f'<g id="layer-scale-marker"><rect x="{marker_x:.2f}" y="{marker_y:.2f}" '
        f'width="{footprint_w * scale:.2f}" height="{footprint_l * scale:.2f}" '
        'fill="none" stroke="#111827" stroke-width="1.5"/>'
        f'<text x="{marker_x:.2f}" y="{marker_y - 6:.2f}" font-size="12" fill="#111827">safety footprint scale</text></g>'
    )

    return f"""<svg id="v2-geometry-map" xmlns="http://www.w3.org/2000/svg" width="{svg_w:.0f}" height="{svg_h:.0f}" viewBox="0 0 {svg_w:.2f} {svg_h:.2f}" data-base-viewbox="0 0 {svg_w:.2f} {svg_h:.2f}" data-minx="{minx:.8f}" data-maxy="{maxy:.8f}" data-scale="{scale:.8f}" data-pad="{pad:.8f}">
  <rect width="100%" height="100%" fill="#f8fafc"/>
  {"".join(layers)}
  <g id="layer-annotation-draft"></g>
  {mower_marker}
</svg>
"""


def render_html(result: dict[str, Any], metrics: dict[str, Any]) -> str:
    svg = render_svg(result)
    area_rows = []
    for area in result.get("areas", []):
        area_rows.append(
            "<tr>"
            f"<td>{area.get('area_index')}</td>"
            f"<td>{html.escape(str(area.get('source_name', '')))}</td>"
            f"<td>{float(area.get('raw_area_m2', 0.0)):.2f}</td>"
            f"<td>{float(area.get('conditioned_area_m2', 0.0)):.2f}</td>"
            f"<td>{float(area.get('drivable_area_m2', 0.0)):.2f}</td>"
            f"<td>{float(area.get('coverage_band_area_m2', 0.0)):.2f}</td>"
            f"<td>{len(area.get('zones', []))}</td>"
            f"<td>{html.escape(str(area.get('zone_counts', {})))}</td>"
            "</tr>"
        )

    warning_items = "".join(f"<li>{html.escape(str(w))}</li>" for w in result.get("warnings", []))
    if not warning_items:
        warning_items = "<li>none</li>"

    all_neck_candidates = [
        candidate for area in result.get("areas", []) for candidate in area.get("neck_candidates", [])
    ]
    all_neck_candidates.sort(
        key=lambda candidate: (
            0 if candidate.get("status") == "accepted" else 1,
            -float(candidate.get("score", 0.0)),
        )
    )
    neck_rows = []
    for candidate in all_neck_candidates[:30]:
        neck_rows.append(
            "<tr>"
            f"<td>{html.escape(str(candidate.get('component_index', '')))}</td>"
            f"<td>{html.escape(str(candidate.get('status', '')))}</td>"
            f"<td>{float(candidate.get('width_m', 0.0)):.2f}</td>"
            f"<td>{float(candidate.get('widening_ratio', 0.0)):.2f}</td>"
            f"<td>{float(candidate.get('score', 0.0)):.2f}</td>"
            f"<td>{html.escape(str(candidate.get('reason', '')))}</td>"
            "</tr>"
        )
    if not neck_rows:
        neck_rows.append('<tr><td colspan="6">No candidate neck cuts found.</td></tr>')
    neck_note = ""
    if len(all_neck_candidates) > 30:
        neck_note = f"<p>Showing top 30 of {len(all_neck_candidates)} candidates.</p>"

    all_task_proposals = [
        proposal for area in result.get("areas", []) for proposal in area.get("task_proposals", [])
    ]
    all_task_proposals.sort(key=_task_sort_key)
    portal_label_by_id = {
        str(candidate.get("id", "")): str(candidate.get("display_label", ""))
        for area in result.get("areas", [])
        for candidate in area.get("portal_candidates", [])
        if candidate.get("display_label")
    }
    task_rows = []
    for proposal in all_task_proposals[:40]:
        evidence = proposal.get("evidence", {})
        task_type = str(proposal.get("task_type", ""))
        row_class = "task-row"
        if task_type == "artifact":
            row_class += " task-artifact-row"
        portal_labels = [
            portal_label_by_id.get(str(portal_id), str(portal_id)) for portal_id in proposal.get("entry_portals", [])
        ]
        terminal_label = "END" if proposal.get("terminal_cap") else "-"
        source_label = str(proposal.get("extent_source", evidence.get("source", ""))).replace("_", " ")
        task_rows.append(
            f'<tr class="{row_class}" data-task-id="{html.escape(str(proposal.get("id", "")))}" data-task-type="{html.escape(task_type)}">'
            f"<td>{html.escape(str(proposal.get('display_label', _task_display_label(task_type))))}</td>"
            f"<td>{float(proposal.get('confidence', 0.0)):.2f}</td>"
            f"<td>{float(proposal.get('service_depth_m', evidence.get('service_depth_m', 0.0))):.2f}</td>"
            f"<td>{float(proposal.get('service_width_m', evidence.get('service_width_m', 0.0))):.2f}</td>"
            f"<td>{html.escape(source_label)}</td>"
            f"<td>{html.escape(', '.join(portal_labels) if portal_labels else '-')}</td>"
            f"<td>{html.escape(terminal_label)}</td>"
            f"<td>{html.escape(str(evidence.get('reason', '')))}</td>"
            "</tr>"
        )
    if not task_rows:
        task_rows.append('<tr><td colspan="8">No task proposals found.</td></tr>')
    task_note = ""
    if len(all_task_proposals) > 40:
        task_note = f"<p>Showing top 40 of {len(all_task_proposals)} task proposals.</p>"

    all_task_paths = [path for area in result.get("areas", []) for path in area.get("task_paths", [])]
    all_task_paths.sort(
        key=lambda path: (TASK_TYPE_ORDER.get(str(path.get("task_type", "artifact")), 99), str(path.get("task_id", "")))
    )
    task_path_rows = []
    for task_path in all_task_paths[:40]:
        task_type = str(task_path.get("task_type", "artifact"))
        warnings_text = "; ".join(str(warning) for warning in task_path.get("warnings", [])) or "-"
        service_axis = task_path.get("service_axis") or {}
        axis_source = str(service_axis.get("source", "-")).replace("_", " ")
        exit_mode = str(task_path.get("exit_mode", "-")).replace("_", " ")
        turn_feasible = "yes" if task_path.get("turn_feasible") else "-"
        owner_source = str(task_path.get("owner_source", "-")).replace("_", " ")
        stripe_clearance = float(task_path.get("stripe_boundary_clearance_m", 0.0))
        min_pass = float(task_path.get("body_min_pass_length_m", task_path.get("stripe_min_pass_length_m", 0.0)))
        pocket_count = int(task_path.get("body_service_pocket_count", 0))
        compact_connector_count = int(task_path.get("compact_connector_count", 0))
        task_path_rows.append(
            f'<tr class="task-row" data-task-id="{html.escape(str(task_path.get("task_id", "")))}" data-task-type="{html.escape(task_type)}">'
            f"<td>{html.escape(_task_display_label(task_type))}</td>"
            f"<td>{html.escape(str(task_path.get('strategy', '')).replace('_', ' '))}</td>"
            f"<td>{html.escape(axis_source)}</td>"
            f"<td>{html.escape(exit_mode)}</td>"
            f"<td>{html.escape(turn_feasible)}</td>"
            f"<td>{html.escape(owner_source)}</td>"
            f"<td>{stripe_clearance:.2f}</td>"
            f"<td>{min_pass:.2f}</td>"
            f"<td>{int(task_path.get('lane_count', task_path.get('pass_count', 0)))}</td>"
            f"<td>{pocket_count}</td>"
            f"<td>{compact_connector_count}</td>"
            f"<td>{float(task_path.get('estimated_coverage_percent', 0.0)):.1f}</td>"
            f"<td>{float(task_path.get('forward_length_m', 0.0)):.2f}</td>"
            f"<td>{float(task_path.get('reverse_length_m', 0.0)):.2f}</td>"
            f"<td>{int(task_path.get('unsafe_sample_count', 0))}</td>"
            f"<td>{html.escape(warnings_text)}</td>"
            "</tr>"
        )
    if not task_path_rows:
        task_path_rows.append('<tr><td colspan="16">No task paths generated.</td></tr>')
    task_path_note = ""
    if len(all_task_paths) > 40:
        task_path_note = f"<p>Showing top 40 of {len(all_task_paths)} task paths.</p>"

    axis_candidate_rows = []
    axis_candidate_total = 0
    for task_path in all_task_paths:
        task_type = str(task_path.get("task_type", "artifact"))
        candidates = list(task_path.get("axis_candidates", []))
        if not candidates:
            continue
        candidates.sort(key=lambda item: int(item.get("rank", 999)))
        axis_candidate_total += len(candidates)
        for candidate in candidates[:6]:
            selected_text = "chosen" if candidate.get("selected") else "rejected"
            axis_candidate_rows.append(
                f'<tr class="task-row" data-task-id="{html.escape(str(task_path.get("task_id", "")))}" data-task-type="{html.escape(task_type)}">'
                f"<td>{html.escape(_task_display_label(task_type))}</td>"
                f"<td>{int(candidate.get('rank', 0))}</td>"
                f"<td>{html.escape(selected_text)}</td>"
                f"<td>{html.escape(str(candidate.get('source', '')).replace('_', ' '))}</td>"
                f"<td>{float(candidate.get('score', 0.0)):.1f}</td>"
                f"<td>{int(candidate.get('lane_count', 0))}</td>"
                f"<td>{float(candidate.get('coverage_percent', 0.0)):.1f}</td>"
                f"<td>{float(candidate.get('mean_lane_length_m', 0.0)):.2f}</td>"
                f"<td>{float(candidate.get('min_lane_length_m', 0.0)):.2f}</td>"
                f"<td>{int(candidate.get('short_lane_count', 0))}</td>"
                f"<td>{int(candidate.get('unsafe_sample_count', 0))}</td>"
                "</tr>"
            )
    if not axis_candidate_rows:
        axis_candidate_rows.append('<tr><td colspan="11">No axis candidates generated for side tasks.</td></tr>')
    axis_candidate_note = ""
    if axis_candidate_total > len(axis_candidate_rows):
        axis_candidate_note = f"<p>Showing top candidates per side task from {axis_candidate_total} total scored candidates.</p>"

    all_qa_matches = [
        match
        for area in result.get("areas", [])
        for match in area.get("annotation_qa", {}).get("matches", [])
    ]
    all_qa_unmatched = [
        unmatched
        for area in result.get("areas", [])
        for unmatched in area.get("annotation_qa", {}).get("unmatched_auto_proposals", [])
    ]
    annotation_qa_rows = []
    for match in all_qa_matches[:40]:
        status = str(match.get("status", "unknown"))
        auto_label = str(match.get("matched_auto_label", "")) or "-"
        auto_type = str(match.get("matched_auto_task_type", "")).replace("_", " ") or "-"
        portal_distance = match.get("portal_distance_m")
        portal_text = "-" if portal_distance is None else f"{float(portal_distance):.2f} m"
        annotation_qa_rows.append(
            f'<tr class="task-row annotation-qa-row" data-task-id="{html.escape(str(match.get("operator_task_id", "")))}">'
            f"<td>{html.escape(str(match.get('operator_label', '')))}</td>"
            f"<td>{html.escape(auto_label)}</td>"
            f"<td>{html.escape(status.replace('_', ' '))}</td>"
            f"<td>{float(match.get('iou') or 0.0):.2f}</td>"
            f"<td>{100.0 * float(match.get('operator_coverage_ratio') or 0.0):.0f}%</td>"
            f"<td>{100.0 * float(match.get('auto_coverage_ratio') or 0.0):.0f}%</td>"
            f"<td>{html.escape(portal_text)}</td>"
            f"<td>{html.escape(str(match.get('operator_task_type', '')).replace('_', ' '))} -> {html.escape(auto_type)}</td>"
            f"<td>{html.escape(str(match.get('reason', '')))}</td>"
            "</tr>"
        )
    remaining_slots = max(0, 40 - len(annotation_qa_rows))
    for unmatched in all_qa_unmatched[:remaining_slots]:
        annotation_qa_rows.append(
            f'<tr class="task-row annotation-qa-row" data-task-id="{html.escape(str(unmatched.get("auto_task_id", "")))}">'
            "<td>-</td>"
            f"<td>{html.escape(str(unmatched.get('auto_label', '')))}</td>"
            "<td>unmatched auto</td>"
            "<td>-</td><td>-</td><td>-</td><td>-</td>"
            f"<td>- -> {html.escape(str(unmatched.get('auto_task_type', '')).replace('_', ' '))}</td>"
            f"<td>{html.escape(str(unmatched.get('reason', '')))}</td>"
            "</tr>"
        )
    if not annotation_qa_rows:
        annotation_qa_rows.append('<tr><td colspan="9">No annotation QA available. Draw/export annotations and rerun with --task-annotations.</td></tr>')
    annotation_qa_note = ""
    if len(all_qa_matches) + len(all_qa_unmatched) > 40:
        annotation_qa_note = f"<p>Showing top 40 of {len(all_qa_matches) + len(all_qa_unmatched)} QA records.</p>"

    task_counts = metrics.get("task_proposals", {}).get("by_type", {})
    legend_parts = []
    for task_type, label in sorted(TASK_TYPE_LABELS.items(), key=lambda item: TASK_TYPE_ORDER.get(item[0], 99)):
        count = int(task_counts.get(task_type, 0))
        if count <= 0:
            continue
        color = TASK_TYPE_COLORS.get(task_type, "#64748b")
        legend_parts.append(
            '<span class="task-legend-item">'
            f'<span class="task-swatch" style="background:{color}"></span>'
            f'{html.escape(label)} <strong>{count}</strong>'
            '</span>'
        )
    legend_html = "".join(legend_parts) or "<span>No task proposals</span>"

    zone_palette = [
        "#22c55e",
        "#f97316",
        "#8b5cf6",
        "#06b6d4",
        "#e11d48",
        "#84cc16",
        "#f59e0b",
        "#2563eb",
        "#ec4899",
        "#14b8a6",
    ]
    all_zones = [zone for area in result.get("areas", []) for zone in area.get("zones", [])]
    zone_rows = []
    for zone in all_zones:
        display_index = int(zone.get("display_index", len(zone_rows) + 1))
        color = zone_palette[(display_index - 1) % len(zone_palette)]
        zone_rows.append(
            "<tr>"
            f'<td><span class="zone-swatch" style="background:{color}"></span>{html.escape(str(zone.get("label", f"Z{display_index}")))}</td>'
            f"<td>{html.escape(str(zone.get('kind', '')))}</td>"
            f"<td>{float(zone.get('area_m2', 0.0)):.2f}</td>"
            f"<td>{float(zone.get('oriented_bounds', {}).get('width_m', 0.0)):.2f}</td>"
            f"<td>{html.escape(str(zone.get('reason', '')))}</td>"
            "</tr>"
        )
    if not zone_rows:
        zone_rows.append('<tr><td colspan="5">No preliminary zones found.</td></tr>')

    layer_descriptions = {
        "layer-raw": "Original recorded map boundary before V2 conditioning.",
        "layer-conditioned": "Boundary after simplification/smoothing used by V2 diagnostics.",
        "layer-drivable": "Mowable/drivable region after the configured boundary clearance inset.",
        "layer-task-proposals": "Detected service regions: BODY, DEAD END, NOTCH, or CORRIDOR task areas.",
        "layer-task-coverage-regions": "Path-owned coverage areas after subtracting side tasks from BODY and applying ownership cleanup.",
        "layer-task-infill-outlines": "Exact polygon used to generate or clip each task's visible fill strokes. For BODY/corridor this is usually the boundary-inset stripe region.",
        "layer-task-stripe-regions": "Boundary-inset areas where BODY/corridor stripe centers are allowed; notches and dead ends can use shorter service strokes.",
        "layer-task-body-pockets": "Supplemental BODY residual pockets serviced after clean inset stripes cannot reach them.",
        "layer-task-local-paths": "Forward local cutter-center task paths. Dashed orange segments are blade-off adjacent connectors.",
        "layer-task-rejected-connectors": "Base_link diagnostics for adjacent connector candidates rejected by gap limits or footprint safety.",
        "layer-v2-outline-paths": "Generated V2 perimeter cutting pass exported as is_outline in planpath_compat.json.",
        "layer-v2-outline-right-side": "Orange trace of the mower footprint's right side while following the generated outline.",
        "layer-v2-outline-starts": "Deterministic start point selected for each generated V2 outline ring.",
        "layer-task-reverse-paths": "Reverse or blade-off exit strokes; these are metadata diagnostics, not current live mower output.",
        "layer-task-direction-arrows": "Direction arrows for local path segments.",
        "layer-task-service-axes": "Chosen stripe/service axes used for BODY and side-task path direction.",
        "layer-task-axis-candidates": "Rejected candidate service axes scored before choosing the local path direction.",
        "layer-task-portals": "Mouth cut lines where a side task attaches to the body. These are not zones.",
        "layer-task-labels": "Map labels for visible task proposals.",
        "layer-task-terminals": "Dead-end terminal caps used for exit/turnaround diagnostics.",
        "layer-task-unsafe-samples": "Base-link footprint samples that leave the drivable region.",
        "layer-operator-annotations": "Manually drawn mouth cuts and service-side points imported from annotation JSON.",
        "layer-annotation-qa": "Dashed outlines comparing automatic task regions against operator annotations.",
        "layer-annotation-qa-links": "Connector lines between an operator task and its matched automatic task.",
        "layer-scale-marker": "A scale rectangle for the configured safety footprint.",
        "layer-service-ownership": "Small BODY fragments reassigned to an adjacent side task near its mouth.",
        "layer-task-turnaround-envelopes": "Sampled dead-end terminal footprint sweeps for forward-turn feasibility.",
        "layer-task-branches": "Thin seed evidence lines from the skeleton/appendage detector; not planned paths.",
        "layer-task-artifacts": "Tiny/noisy task candidates hidden from the default view.",
        "layer-width-heatmap": "Sampled local width/clearance points.",
        "layer-ridge": "Prototype clearance ridge/centerline samples; not route geometry.",
        "layer-skeleton-nodes": "Endpoint/branch/chain nodes from the sampled ridge graph.",
        "layer-neck-cuts": "Candidate neck/mouth cuts from the width-profile diagnostic pass.",
        "layer-zones": "Old preliminary zone heuristic kept only for comparison.",
        "layer-zone-labels": "Labels for the old preliminary zone heuristic.",
        "layer-coverage-band": "Boundary band between conditioned lawn and the drivable inset.",
        "layer-maneuver-limited": "Drivable area that needs planned yaw instead of arbitrary rotation.",
        "layer-all-yaw-safe": "Stricter footprint-disk region where any yaw is safe.",
        "layer-unreachable": "Conditioned lawn area removed by drivable inset/geometry constraints.",
        "layer-noise": "Removed or filled conditioning artifacts.",
    }
    primary_controls = [
        ("layer-raw", "raw outline"),
        ("layer-conditioned", "conditioned outline"),
        ("layer-drivable", "mowable/drivable"),
        ("layer-task-proposals", "service regions"),
        ("layer-task-coverage-regions", "path coverage areas"),
        ("layer-task-infill-outlines", "infill area outline"),
        ("layer-task-body-pockets", "body pocket fills"),
        ("layer-v2-outline-paths", "outline path"),
        ("layer-v2-outline-right-side", "outline right side"),
        ("layer-v2-outline-starts", "outline start"),
        ("layer-task-local-paths", "task paths"),
        ("layer-task-rejected-connectors", "rejected connectors"),
        ("layer-task-reverse-paths", "reverse/exits"),
        ("layer-task-direction-arrows", "direction arrows"),
        ("layer-task-service-axes", "service axes"),
        ("layer-task-portals", "mouth cuts"),
        ("layer-task-labels", "task labels"),
        ("layer-task-terminals", "dead-end terminals"),
        ("layer-task-unsafe-samples", "unsafe samples"),
        ("layer-operator-annotations", "operator annotations"),
        ("layer-annotation-qa", "annotation QA"),
        ("layer-annotation-qa-links", "QA links"),
        ("layer-scale-marker", "footprint scale"),
    ]
    advanced_controls = [
        ("layer-service-ownership", "service ownership"),
        ("layer-task-turnaround-envelopes", "turnaround envelopes"),
        ("layer-task-stripe-regions", "stripe-safe areas"),
        ("layer-task-axis-candidates", "axis alternatives"),
        ("layer-task-branches", "seed evidence lines"),
        ("layer-task-artifacts", "task artifacts"),
        ("layer-width-heatmap", "local width"),
        ("layer-ridge", "ridge centerline"),
        ("layer-skeleton-nodes", "skeleton nodes"),
        ("layer-neck-cuts", "candidate neck cuts"),
        ("layer-zones", "old preliminary zones"),
        ("layer-zone-labels", "old zone labels"),
        ("layer-coverage-band", "coverage band"),
        ("layer-maneuver-limited", "needs planned yaw"),
        ("layer-all-yaw-safe", "all-yaw safe"),
        ("layer-unreachable", "unreachable"),
        ("layer-noise", "removed/filled noise"),
    ]

    def render_layer_controls(items: list[tuple[str, str]]) -> str:
        return "".join(
            '<label class="layer-control">'
            f'<input type="checkbox" data-layer="{layer}" checked> '
            f'<span>{html.escape(label)}</span>'
            f'<span class="layer-info" title="{html.escape(layer_descriptions.get(layer, ""), quote=True)}">i</span>'
            '</label>'
            for layer, label in items
        )

    controls = primary_controls + advanced_controls
    control_html = (
        f'<div class="layers">{render_layer_controls(primary_controls)}</div>'
        '<details class="advanced-layers"><summary>Advanced diagnostics</summary>'
        f'<div class="layers">{render_layer_controls(advanced_controls)}</div>'
        '</details>'
    )

    baseline = metrics.get("baseline_current_planner", {})
    baseline_html = ""
    if baseline:
        baseline_html = (
            "<section><h2>Current Planner Baseline</h2>"
            "<table><tbody>"
            f"<tr><th>coverage %</th><td>{float(baseline.get('coverage_percent', 0.0)):.2f}</td></tr>"
            f"<tr><th>cells</th><td>{int(baseline.get('cells', 0))}</td></tr>"
            f"<tr><th>connector length m</th><td>{float(baseline.get('connector_length_m', 0.0)):.2f}</td></tr>"
            "</tbody></table></section>"
        )
    local_width = metrics.get("local_width", {})
    ridge = metrics.get("ridge", {})
    skeleton = metrics.get("skeleton_graph", {})
    task_summary = metrics.get("task_proposals", {})
    task_path_summary = metrics.get("task_paths", {})
    outline_summary = metrics.get("outline_paths", {})
    annotation_summary = metrics.get("task_annotations", {})
    ownership_summary = metrics.get("ownership_adjustments", {})
    portal_summary = metrics.get("portal_candidates", {})
    neck_summary = metrics.get("neck_candidates", {})
    annotation_qa_summary = metrics.get("annotation_qa", {})
    annotation_source_map_arg = str(result.get("source_map", ""))
    if annotation_source_map_arg.startswith("/workspace/"):
        annotation_source_map_arg = annotation_source_map_arg[len("/workspace/") :]
    annotation_rerun_command = (
        "tools/coverage_lab/bin/coverage_lab classify-v2 --map "
        f"{annotation_source_map_arg} --task-annotations v2_task_annotations.json"
    )

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Coverage Planner V2 Geometry Report</title>
  <style>
    body {{ margin: 0; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; color: #172033; background: #f4f7fb; }}
    header {{ padding: 18px 24px; background: #122033; color: white; }}
    h1 {{ font-size: 22px; margin: 0 0 4px; }}
    h2 {{ font-size: 16px; margin: 0 0 10px; }}
    main {{ display: grid; grid-template-columns: minmax(0, 1fr) 360px; gap: 16px; padding: 16px; }}
    .map-panel {{ background: white; border: 1px solid #d7dee9; border-radius: 8px; overflow: auto; min-height: 520px; position: relative; }}
    .map-toolbar {{ position: sticky; top: 0; z-index: 5; display: flex; flex-wrap: wrap; gap: 6px; align-items: center; padding: 8px; background: rgba(248, 250, 252, 0.94); border-bottom: 1px solid #e2e8f0; }}
    .map-toolbar span {{ font-size: 12px; color: #475569; }}
    .map-stage {{ width: max-content; min-width: 100%; }}
    .side {{ display: grid; gap: 12px; align-content: start; }}
    section {{ background: white; border: 1px solid #d7dee9; border-radius: 8px; padding: 12px; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 13px; }}
    th, td {{ border-bottom: 1px solid #e5eaf2; padding: 6px 4px; text-align: left; vertical-align: top; }}
    th {{ color: #475569; font-weight: 650; }}
    button {{ appearance: none; border: 1px solid #cbd5e1; border-radius: 6px; background: #ffffff; color: #172033; font: inherit; font-size: 13px; padding: 6px 8px; cursor: pointer; }}
    button:hover {{ background: #f8fafc; border-color: #94a3b8; }}
    .quick-views {{ display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 10px; }}
    .artifact-toggle {{ display: inline-flex; align-items: center; gap: 5px; margin: 2px 0 10px; font-size: 13px; color: #334155; }}
    .layers {{ display: grid; grid-template-columns: 1fr 1fr; gap: 6px 10px; font-size: 13px; }}
    .layer-control {{ display: inline-flex; align-items: center; gap: 5px; min-width: 0; }}
    .layer-control span:first-of-type {{ overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }}
    .layer-info {{ display: inline-grid; place-items: center; width: 15px; height: 15px; border-radius: 50%; border: 1px solid #94a3b8; color: #475569; font-size: 10px; font-weight: 800; line-height: 1; cursor: help; flex: 0 0 auto; }}
    .advanced-layers {{ margin-top: 10px; border-top: 1px solid #e2e8f0; padding-top: 8px; }}
    .advanced-layers summary {{ cursor: pointer; color: #334155; font-size: 13px; font-weight: 650; margin-bottom: 8px; }}
    .zone-swatch {{ display: inline-block; width: 14px; height: 14px; border: 1px solid #111827; border-radius: 3px; margin-right: 6px; vertical-align: -2px; }}
    .task-legend {{ display: flex; flex-wrap: wrap; gap: 6px 10px; margin: 0 0 10px; font-size: 12px; color: #334155; }}
    .task-legend-item {{ display: inline-flex; align-items: center; gap: 5px; }}
    .task-swatch {{ display: inline-block; width: 13px; height: 13px; border: 1px solid #111827; border-radius: 3px; }}
    .task-row {{ cursor: pointer; }}
    .task-row:hover, .task-row.is-active {{ background: #e0f2fe; }}
    body:not(.show-artifacts) .task-artifact-row {{ display: none; }}
    body:not(.show-artifacts) #layer-task-artifacts {{ display: none; }}
    .task-hit {{ cursor: pointer; transition: opacity 120ms ease, stroke-width 120ms ease; }}
    .task-hit.is-active.task-shape {{ stroke: #111827 !important; stroke-width: 3.5 !important; fill-opacity: 0.64 !important; }}
    .task-hit.is-active.task-branch {{ stroke-width: 5.2 !important; stroke-opacity: 1 !important; }}
    .task-service-axis.is-active {{ stroke-width: 4.4 !important; stroke-opacity: 1 !important; }}
    .task-service-axis-arrow.is-active {{ stroke: #111827 !important; stroke-width: 1.4 !important; }}
    .task-axis-candidate.is-active {{ stroke-width: 3.2 !important; stroke-opacity: 0.92 !important; }}
    .task-turnaround-envelope.is-active {{ stroke-width: 3.6 !important; fill-opacity: 0.38 !important; }}
    .task-ownership-adjustment.is-active {{ stroke-width: 3.2 !important; fill-opacity: 0.42 !important; }}
    .task-infill-outline.is-active {{ stroke-width: 4.6 !important; stroke-opacity: 1 !important; }}
    .v2-outline-path {{ filter: drop-shadow(0 1px 0 rgba(255,255,255,0.75)); }}
    .task-local-path.is-active {{ stroke-width: 5.2 !important; stroke-opacity: 1 !important; }}
    .task-direction-arrow.is-active {{ stroke: #111827 !important; stroke-width: 1.5 !important; }}
    .task-coverage-region.is-active {{ stroke-width: 3.2 !important; stroke-opacity: 1 !important; }}
    .task-label.is-active rect {{ stroke-width: 3.2 !important; fill-opacity: 1 !important; }}
    .portal-hit.is-active {{ stroke: #111827 !important; stroke-width: 4.0 !important; opacity: 1 !important; }}
    .portal-label.is-active circle {{ stroke-width: 3.0 !important; }}
    .task-terminal.is-active {{ stroke-width: 5.0 !important; }}
    .task-terminal-label.is-active rect {{ stroke-width: 3.0 !important; }}
    .annotation-qa.is-active {{ stroke-width: 4.8 !important; stroke-opacity: 1 !important; }}
    .annotation-qa-link.is-active {{ stroke-width: 3.4 !important; stroke-opacity: 1 !important; }}
    ul {{ margin: 0; padding-left: 18px; font-size: 13px; }}
    button.primary {{ background: #0f766e; border-color: #0f766e; color: white; }}
    button.primary:hover {{ background: #115e59; border-color: #115e59; }}
    button.annotation-active {{ background: #111827; border-color: #111827; color: white; }}
    .annotation-tools {{ display: grid; gap: 10px; font-size: 13px; }}
    .annotation-help {{ color: #475569; line-height: 1.35; margin: 0; }}
    .annotation-steps {{ display: grid; gap: 5px; }}
    .annotation-step {{ display: grid; grid-template-columns: 24px 1fr; align-items: center; gap: 7px; color: #64748b; }}
    .annotation-step-number {{ width: 22px; height: 22px; border: 1px solid #cbd5e1; border-radius: 50%; display: inline-grid; place-items: center; font-size: 12px; font-weight: 700; background: #fff; }}
    .annotation-step.is-current {{ color: #0f766e; font-weight: 700; }}
    .annotation-step.is-current .annotation-step-number {{ background: #ccfbf1; border-color: #14b8a6; color: #0f766e; }}
    .annotation-step.is-done {{ color: #334155; }}
    .annotation-step.is-done .annotation-step-number {{ background: #dcfce7; border-color: #22c55e; color: #166534; }}
    .annotation-fields {{ display: grid; grid-template-columns: 1fr 88px; gap: 8px; }}
    .annotation-tools label {{ display: grid; gap: 3px; }}
    .annotation-tools input, .annotation-tools select {{ font: inherit; border: 1px solid #cbd5e1; border-radius: 6px; padding: 5px 6px; }}
    .annotation-actions {{ display: flex; flex-wrap: wrap; gap: 6px; }}
    .annotation-status {{ color: #334155; min-height: 18px; padding: 7px 8px; background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; }}
    .annotation-rerun {{ display: grid; gap: 5px; }}
    .annotation-rerun-row {{ display: grid; grid-template-columns: 1fr auto; gap: 6px; }}
    .annotation-rerun input {{ min-width: 0; color: #334155; background: #f8fafc; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 11px; }}
    .annotation-list-title {{ display: flex; align-items: center; justify-content: space-between; gap: 8px; color: #475569; font-weight: 650; }}
    #annotation-list {{ list-style: none; margin: 0; padding: 0; display: grid; gap: 6px; }}
    #annotation-list li {{ display: grid; grid-template-columns: 1fr auto; gap: 6px; align-items: center; padding: 6px 7px; border: 1px solid #e2e8f0; border-radius: 6px; background: #f8fafc; }}
    .annotation-list-main {{ display: grid; gap: 2px; }}
    .annotation-list-title-line {{ font-weight: 700; color: #172033; }}
    .annotation-list-detail {{ font-size: 12px; color: #64748b; }}
    #annotation-list button {{ padding: 4px 6px; font-size: 12px; }}
    .annotation-preview-line {{ stroke: #111827; stroke-width: 3; stroke-opacity: 0.9; vector-effect: non-scaling-stroke; }}
    .annotation-preview-service-line {{ stroke: #0f766e; stroke-width: 2.2; stroke-dasharray: 7 4; vector-effect: non-scaling-stroke; }}
    .annotation-preview-point {{ fill: #ffffff; stroke: #111827; stroke-width: 2.2; vector-effect: non-scaling-stroke; }}
    .annotation-preview-service-point {{ fill: #ccfbf1; stroke: #0f766e; stroke-width: 2.6; vector-effect: non-scaling-stroke; }}
    .annotation-preview-label {{ font-size: 12px; font-weight: 800; fill: #111827; paint-order: stroke; stroke: white; stroke-width: 3px; }}
    body.annotation-mode #v2-geometry-map {{ cursor: crosshair; }}
    #v2-geometry-map {{ display: block; width: 100%; height: auto; }}
    @media (max-width: 980px) {{ main {{ grid-template-columns: 1fr; }} .side {{ grid-template-columns: 1fr; }} }}
  </style>
</head>
<body>
  <header>
    <h1>Coverage Planner V2 Geometry Report</h1>
    <div>{html.escape(str(result.get("source_map", "")))}</div>
  </header>
  <main>
    <div class="map-panel">
      <div class="map-toolbar">
        <button type="button" data-map-zoom="in">Zoom in</button>
        <button type="button" data-map-zoom="out">Zoom out</button>
        <button type="button" data-map-zoom="reset">Reset</button>
        <button type="button" id="map-pan-button">Pan</button>
        <span>Wheel zooms. Pan mode lets you drag the map.</span>
      </div>
      <div class="map-stage">{svg}</div>
    </div>
    <div class="side">
      <section>
        <h2>Layers</h2>
        <div class="quick-views">
          <button type="button" data-preset="tasks">Tasks</button>
          <button type="button" data-preset="skeleton">Skeleton</button>
          <button type="button" data-preset="zones">Old Zones Only</button>
          <button type="button" data-preset="diagnostics">Diagnostics</button>
          <button type="button" data-preset="all">All Layers</button>
        </div>
        <label class="artifact-toggle"><input id="show-artifacts" type="checkbox"> Show artifacts</label>
        {control_html}
      </section>
      <section>
        <h2>Annotations</h2>
        <div class="annotation-tools">
          <p class="annotation-help">Draw where a task should start: click both ends of the mouth cut, then click inside the side that belongs to the notch, corridor, or dead end.</p>
          <div class="annotation-steps" id="annotation-steps">
            <div class="annotation-step" data-annotation-step="0"><span class="annotation-step-number">1</span><span>Click first end of the mouth cut</span></div>
            <div class="annotation-step" data-annotation-step="1"><span class="annotation-step-number">2</span><span>Click second end of the mouth cut</span></div>
            <div class="annotation-step" data-annotation-step="2"><span class="annotation-step-number">3</span><span>Click the service side to keep</span></div>
          </div>
          <div class="annotation-actions">
            <button type="button" class="primary" id="annotation-mode-button">Start drawing</button>
            <button type="button" id="annotation-undo-button">Undo point</button>
            <button type="button" id="annotation-clear-button">Clear draft</button>
            <button type="button" id="annotation-delete-last-button">Delete last</button>
            <button type="button" id="annotation-export-button">Export JSON</button>
          </div>
          <div class="annotation-fields">
            <label>Task type
              <select id="annotation-task-type">
                <option value="dead_end_corridor">Dead end</option>
                <option value="notch">Notch</option>
                <option value="corridor">Corridor</option>
              </select>
            </label>
            <label>Area
              <input id="annotation-area-index" type="number" min="0" step="1" value="0">
            </label>
          </div>
          <label>Label
            <input id="annotation-label" type="text" placeholder="optional, for example left dead end">
          </label>
          <label>Note
            <input id="annotation-note" type="text" placeholder="optional reason">
          </label>
          <div class="annotation-status" id="annotation-status">Click Start drawing, then use the three map clicks above.</div>
          <div class="annotation-rerun">
            <label>Rerun command after export</label>
            <div class="annotation-rerun-row">
              <input id="annotation-rerun-command" type="text" readonly value="{html.escape(annotation_rerun_command, quote=True)}">
              <button type="button" id="annotation-copy-command-button">Copy</button>
            </div>
          </div>
          <div class="annotation-list-title"><span>Draft annotations</span><span id="annotation-count">0</span></div>
          <ul id="annotation-list"></ul>
        </div>
      </section>
      <section>
        <h2>Task Paths</h2>
        {task_path_note}
        <table>
          <thead><tr><th>Task</th><th>Strategy</th><th>Axis</th><th>Exit</th><th>Turn</th><th>Owner</th><th>Clear</th><th>Min</th><th>Lanes</th><th>Pockets</th><th>Connectors</th><th>Cov. %</th><th>Fwd</th><th>Rev</th><th>Unsafe</th><th>Warnings</th></tr></thead>
          <tbody>{"".join(task_path_rows)}</tbody>
        </table>
      </section>
      <section>
        <h2>Axis Candidates</h2>
        {axis_candidate_note}
        <table>
          <thead><tr><th>Task</th><th>Rank</th><th>Status</th><th>Source</th><th>Score</th><th>Lanes</th><th>Cov. %</th><th>Mean</th><th>Min</th><th>Short</th><th>Unsafe</th></tr></thead>
          <tbody>{"".join(axis_candidate_rows)}</tbody>
        </table>
      </section>
      <section>
        <h2>Task Evidence</h2>
        {task_note}
        <div class="task-legend">{legend_html}</div>
        <table>
          <thead><tr><th>Task</th><th>Conf.</th><th>Depth</th><th>Width</th><th>Source</th><th>Portals</th><th>End</th><th>Why</th></tr></thead>
          <tbody>{"".join(task_rows)}</tbody>
        </table>
      </section>
      <section>
        <h2>Annotation QA</h2>
        {annotation_qa_note}
        <table>
          <thead><tr><th>Operator</th><th>Auto</th><th>Status</th><th>IoU</th><th>Op %</th><th>Auto %</th><th>Portal</th><th>Type</th><th>Why</th></tr></thead>
          <tbody>{"".join(annotation_qa_rows)}</tbody>
        </table>
      </section>
      <section>
        <h2>Old Preliminary Zones</h2>
        <p>These Z labels are the earlier comparison-only zone heuristic. Operator annotations do not rewrite this layer; use Task Evidence and Task Paths to judge the current annotated tasks.</p>
        <table>
          <thead><tr><th>Zone</th><th>Kind</th><th>Area</th><th>Width</th><th>Why</th></tr></thead>
          <tbody>{"".join(zone_rows)}</tbody>
        </table>
      </section>
      <section>
        <h2>Region Meaning</h2>
        <table><tbody>
          <tr><th>mowable/drivable</th><td>conditioned lawn inset by the configured boundary clearance, currently {float(result.get('conditioning', {}).get('drivable_boundary_clearance_m', 0.0)):.2f} m</td></tr>
          <tr><th>all-yaw safe</th><td>stricter base_link footprint-disk inset where the full mower footprint can rotate at any yaw without crossing the boundary</td></tr>
          <tr><th>needs planned yaw</th><td>drivable area that is usable for mowing, but turns/pivots there need planned orientation instead of arbitrary rotation</td></tr>
          <tr><th>local width</th><td>sampled diagnostic estimate, computed as twice the point clearance to the drivable boundary</td></tr>
          <tr><th>ridge centerline</th><td>prototype skeleton points where sampled clearance is locally maximal; not route geometry</td></tr>
          <tr><th>task proposals</th><td>diagnostic behavior proposals from skeleton branches and body-width evidence; not route geometry yet</td></tr>
          <tr><th>task service axes</th><td>Chosen direction arrows for BODY and single-entry tasks; they explain local stripe/path direction</td></tr>
          <tr><th>axis candidates</th><td>M2.6/M2.7 scored alternatives for BODY and side-task stripe direction; rejected axes are shown only in diagnostics so boundary wrinkles do not force the path direction</td></tr>
          <tr><th>operator annotations</th><td>exported/imported mouth cuts and service-side points that override automatic task starts</td></tr>
          <tr><th>service ownership</th><td>BODY fragments near task mouths reassigned to the adjacent task before local path generation</td></tr>
          <tr><th>turnaround envelopes</th><td>sampled 180-degree dead-end terminal footprint sweeps used to choose forward turnaround versus reverse out</td></tr>
          <tr><th>task local paths</th><td>M2 local service-pattern prototypes inside one task at a time; they are not globally ordered and are not mower-ready output</td></tr>
          <tr><th>annotation QA</th><td>M2.3 dashed operator-vs-automatic outlines showing where automatic task evidence matched, missed, or disagreed with imported task-mouth annotations</td></tr>
          <tr><th>seed evidence lines</th><td>thin diagnostic lines showing why a task was detected; they are not planned mower paths</td></tr>
          <tr><th>mouth cuts</th><td>portal lines where a corridor, dead end, or notch attaches to the body. They are not generated zones.</td></tr>
          <tr><th>portal search</th><td>M2.4 automatic candidates shift eligible mouth cuts toward the body core and keep the largest safe service-side component; used to avoid under-grown task regions</td></tr>
        </tbody></table>
      </section>
      <section>
        <h2>Summary</h2>
        <table><tbody>
          <tr><th>raw area m2</th><td>{float(metrics.get('raw_area_m2', 0.0)):.2f}</td></tr>
          <tr><th>conditioned area m2</th><td>{float(metrics.get('conditioned_area_m2', 0.0)):.2f}</td></tr>
          <tr><th>drivable area m2</th><td>{float(metrics.get('drivable_area_m2', 0.0)):.2f}</td></tr>
          <tr><th>all-yaw safe m2</th><td>{float(metrics.get('all_yaw_safe_area_m2', 0.0)):.2f}</td></tr>
          <tr><th>coverage band m2</th><td>{float(metrics.get('coverage_band_area_m2', 0.0)):.2f}</td></tr>
          <tr><th>needs planned yaw m2</th><td>{float(metrics.get('maneuver_limited_area_m2', 0.0)):.2f}</td></tr>
          <tr><th>unreachable m2</th><td>{float(metrics.get('unreachable_area_m2', 0.0)):.2f}</td></tr>
          <tr><th>task proposals</th><td>{int(task_summary.get('total', 0))}</td></tr>
          <tr><th>task types</th><td>{html.escape(str(task_summary.get('by_type', {})))}</td></tr>
          <tr><th>task paths</th><td>{int(task_path_summary.get('total', 0))}</td></tr>
          <tr><th>adjacent connectors</th><td>{int(task_path_summary.get('compact_connector_count', 0))} inserted, {int(task_path_summary.get('compact_connector_rejected_count', 0))} rejected</td></tr>
          <tr><th>outline paths</th><td>{int(outline_summary.get('total', 0))}</td></tr>
          <tr><th>outline length m</th><td>{float(outline_summary.get('path_length_m', 0.0)):.2f}</td></tr>
          <tr><th>outline modes</th><td>{html.escape(str(outline_summary.get('generation_modes', {})))}</td></tr>
          <tr><th>outline offset min / median / max m</th><td>{float(outline_summary.get('centerline_offset_min_m', 0.0)):.2f} / {float(outline_summary.get('centerline_offset_median_m', 0.0)):.2f} / {float(outline_summary.get('centerline_offset_max_m', 0.0)):.2f}</td></tr>
          <tr><th>right footprint side min / median / max m</th><td>{float(outline_summary.get('right_footprint_side_min_clearance_m', outline_summary.get('right_cut_edge_min_clearance_m', 0.0))):.2f} / {float(outline_summary.get('right_footprint_side_median_clearance_m', outline_summary.get('right_cut_edge_median_clearance_m', 0.0))):.2f} / {float(outline_summary.get('right_footprint_side_max_clearance_m', outline_summary.get('right_cut_edge_max_clearance_m', 0.0))):.2f}</td></tr>
          <tr><th>footprint min clearance m</th><td>{float(outline_summary.get('footprint_min_clearance_m', 0.0)):.2f}</td></tr>
          <tr><th>task path exit modes</th><td>{html.escape(str(task_path_summary.get('by_exit_mode', {})))}</td></tr>
          <tr><th>axis candidates</th><td>{int(task_path_summary.get('axis_candidates_total', 0))}</td></tr>
          <tr><th>selected axis sources</th><td>{html.escape(str(task_path_summary.get('selected_axis_by_source', {})))}</td></tr>
          <tr><th>task path coverage %</th><td>{float(task_path_summary.get('estimated_coverage_percent', 0.0)):.1f}</td></tr>
          <tr><th>operator annotations</th><td>{int(annotation_summary.get('accepted', 0))} accepted / {int(annotation_summary.get('total', 0))} total</td></tr>
          <tr><th>annotation QA</th><td>{int(annotation_qa_summary.get('matched', 0))} matched, {int(annotation_qa_summary.get('missed_by_auto', 0))} missed, {int(annotation_qa_summary.get('type_mismatch', 0))} type mismatch</td></tr>
          <tr><th>ownership adjustments</th><td>{int(ownership_summary.get('total', 0))} adjustments, {float(ownership_summary.get('area_m2', 0.0)):.2f} m2</td></tr>
          <tr><th>preliminary zones</th><td>{int(metrics.get('zone_count', 0))}</td></tr>
          <tr><th>zone counts</th><td>{html.escape(str(metrics.get('zone_counts', {})))}</td></tr>
        </tbody></table>
      </section>
      <section>
        <h2>Diagnostics</h2>
        <table><tbody>
          <tr><th>width samples</th><td>{int(local_width.get('sample_count', 0))}</td></tr>
          <tr><th>width min / median / max m</th><td>{float(local_width.get('min_m', 0.0)):.2f} / {float(local_width.get('median_m', 0.0)):.2f} / {float(local_width.get('max_m', 0.0)):.2f}</td></tr>
          <tr><th>width classes</th><td>{html.escape(str(local_width.get('class_counts', {})))}</td></tr>
          <tr><th>ridge points / links</th><td>{int(ridge.get('point_count', 0))} / {int(ridge.get('link_count', 0))}</td></tr>
          <tr><th>skeleton nodes / branches</th><td>{int(skeleton.get('node_count', 0))} / {int(skeleton.get('branch_count', 0))}</td></tr>
          <tr><th>task path fwd / rev m</th><td>{float(task_path_summary.get('forward_length_m', 0.0)):.2f} / {float(task_path_summary.get('reverse_length_m', 0.0)):.2f}</td></tr>
          <tr><th>adjacent connector length m</th><td>{float(task_path_summary.get('compact_connector_length_m', 0.0)):.2f}</td></tr>
          <tr><th>outline cutting length m</th><td>{float(outline_summary.get('cutting_length_m', 0.0)):.2f}</td></tr>
          <tr><th>outline corner local max extra m</th><td>{float(outline_summary.get('corner_local_fit_max_extra_offset_m', 0.0)):.2f}</td></tr>
          <tr><th>right edge clearance sources</th><td>{html.escape(str(outline_summary.get('right_edge_clearance_sources', {})))}</td></tr>
          <tr><th>task path unsafe samples</th><td>{int(task_path_summary.get('unsafe_sample_count', 0))}</td></tr>
          <tr><th>outline unsafe samples</th><td>{int(outline_summary.get('unsafe_sample_count', 0))}</td></tr>
          <tr><th>annotation accepted / rejected</th><td>{int(annotation_summary.get('accepted', 0))} / {int(annotation_summary.get('rejected', 0))}</td></tr>
          <tr><th>annotation QA statuses</th><td>{html.escape(str(annotation_qa_summary.get('status_counts', {})))}</td></tr>
          <tr><th>unmatched automatic tasks</th><td>{int(annotation_qa_summary.get('unmatched_auto', 0))}</td></tr>
          <tr><th>portals</th><td>{int(portal_summary.get('total', 0))} total, {int(portal_summary.get('accepted', 0))} accepted, {int(portal_summary.get('rejected', 0))} rejected</td></tr>
          <tr><th>portal reasons</th><td>{html.escape(str(portal_summary.get('by_reason', {})))}</td></tr>
          <tr><th>neck candidates</th><td>{int(neck_summary.get('total', 0))} total, {int(neck_summary.get('accepted', 0))} accepted, {int(neck_summary.get('rejected', 0))} rejected</td></tr>
          <tr><th>neck reasons</th><td>{html.escape(str(neck_summary.get('by_reason', {})))}</td></tr>
        </tbody></table>
      </section>
      <section>
        <h2>Why This Cut?</h2>
        {neck_note}
        <table>
          <thead><tr><th>Comp.</th><th>Status</th><th>Width</th><th>Widen</th><th>Score</th><th>Reason</th></tr></thead>
          <tbody>{"".join(neck_rows)}</tbody>
        </table>
      </section>
      {baseline_html}
      <section>
        <h2>Areas</h2>
        <table>
          <thead><tr><th>#</th><th>Name</th><th>Raw</th><th>Cond.</th><th>Drive</th><th>Band</th><th>Zones</th><th>Kinds</th></tr></thead>
          <tbody>{"".join(area_rows)}</tbody>
        </table>
      </section>
      <section>
        <h2>Warnings</h2>
        <ul>{warning_items}</ul>
      </section>
    </div>
  </main>
  <script>
    const layerInputs = Array.from(document.querySelectorAll("[data-layer]"));
    const artifactToggle = document.getElementById("show-artifacts");
    let pinnedTaskId = null;
    function setLayer(layerId, checked) {{
      const input = layerInputs.find((candidate) => candidate.dataset.layer === layerId);
      const layer = document.getElementById(layerId);
      if (input) input.checked = checked;
      if (layer) layer.style.display = checked ? "" : "none";
    }}
    function setArtifactVisibility(checked) {{
      document.body.classList.toggle("show-artifacts", checked);
      if (artifactToggle) artifactToggle.checked = checked;
      setLayer("layer-task-artifacts", checked);
    }}
    function applyPreset(name) {{
      const allLayers = layerInputs.map((input) => input.dataset.layer);
      if (name === "all") {{
        allLayers.forEach((layer) => setLayer(layer, true));
        setArtifactVisibility(true);
        return;
      }}
      let visible;
      if (name === "zones") {{
        visible = new Set(["layer-conditioned", "layer-raw", "layer-zones", "layer-zone-labels", "layer-scale-marker"]);
      }} else if (name === "skeleton") {{
        visible = new Set(["layer-conditioned", "layer-raw", "layer-drivable", "layer-ridge", "layer-skeleton-nodes", "layer-task-branches", "layer-task-portals", "layer-task-terminals", "layer-operator-annotations", "layer-scale-marker"]);
      }} else if (name === "tasks") {{
        visible = new Set(["layer-conditioned", "layer-raw", "layer-drivable", "layer-task-coverage-regions", "layer-task-infill-outlines", "layer-task-body-pockets", "layer-v2-outline-paths", "layer-v2-outline-right-side", "layer-v2-outline-starts", "layer-operator-annotations", "layer-task-proposals", "layer-task-service-axes", "layer-task-local-paths", "layer-task-rejected-connectors", "layer-task-direction-arrows", "layer-task-labels", "layer-task-portals", "layer-task-terminals", "layer-annotation-qa", "layer-annotation-qa-links", "layer-scale-marker"]);
      }} else {{
        visible = new Set(["layer-conditioned", "layer-raw", "layer-drivable", "layer-width-heatmap", "layer-ridge", "layer-skeleton-nodes", "layer-neck-cuts", "layer-task-coverage-regions", "layer-task-infill-outlines", "layer-task-stripe-regions", "layer-task-body-pockets", "layer-v2-outline-paths", "layer-v2-outline-right-side", "layer-v2-outline-starts", "layer-operator-annotations", "layer-service-ownership", "layer-task-proposals", "layer-task-branches", "layer-task-service-axes", "layer-task-axis-candidates", "layer-task-turnaround-envelopes", "layer-task-local-paths", "layer-task-rejected-connectors", "layer-task-reverse-paths", "layer-task-direction-arrows", "layer-task-unsafe-samples", "layer-task-labels", "layer-task-portals", "layer-task-terminals", "layer-task-artifacts", "layer-annotation-qa", "layer-annotation-qa-links", "layer-zones", "layer-zone-labels", "layer-scale-marker"]);
      }}
      allLayers.forEach((layer) => setLayer(layer, visible.has(layer)));
      setArtifactVisibility(name === "diagnostics");
    }}
    function elementsForTask(taskId) {{
      return Array.from(document.querySelectorAll("[data-task-id]")).filter((element) => element.dataset.taskId === taskId);
    }}
    function clearActiveTask() {{
      document.querySelectorAll(".is-active").forEach((element) => element.classList.remove("is-active"));
    }}
    function setActiveTask(taskId, pinned = false) {{
      if (!taskId) return;
      clearActiveTask();
      elementsForTask(taskId).forEach((element) => element.classList.add("is-active"));
      if (pinned) pinnedTaskId = taskId;
    }}
    function restorePinnedTask() {{
      clearActiveTask();
      if (pinnedTaskId) setActiveTask(pinnedTaskId, false);
    }}
    const annotationState = {{ active: false, draft: [], annotations: [] }};
    const mapPanButton = document.getElementById("map-pan-button");
    const annotationButton = document.getElementById("annotation-mode-button");
    const annotationUndoButton = document.getElementById("annotation-undo-button");
    const annotationClearButton = document.getElementById("annotation-clear-button");
    const annotationDeleteLastButton = document.getElementById("annotation-delete-last-button");
    const annotationExportButton = document.getElementById("annotation-export-button");
    const annotationCopyCommandButton = document.getElementById("annotation-copy-command-button");
    const annotationStatus = document.getElementById("annotation-status");
    const annotationList = document.getElementById("annotation-list");
    const annotationCount = document.getElementById("annotation-count");
    const annotationSvg = document.getElementById("v2-geometry-map");
    const annotationDraftLayer = document.getElementById("layer-annotation-draft");
    const annotationStepElements = Array.from(document.querySelectorAll("[data-annotation-step]"));
    const annotationStorageKey = "open_mower_v2_annotations:" + {json.dumps(str(result.get("source_map", "")))} + ":" + {json.dumps(str(result.get("frame_id", "map")))};
    const mapBaseViewBox = (annotationSvg?.dataset.baseViewbox || annotationSvg?.getAttribute("viewBox") || "0 0 1 1").split(/\\s+/).map(Number);
    const mapView = {{
      x: mapBaseViewBox[0] || 0,
      y: mapBaseViewBox[1] || 0,
      w: mapBaseViewBox[2] || 1,
      h: mapBaseViewBox[3] || 1,
      baseX: mapBaseViewBox[0] || 0,
      baseY: mapBaseViewBox[1] || 0,
      baseW: mapBaseViewBox[2] || 1,
      baseH: mapBaseViewBox[3] || 1,
      pan: false,
      dragging: false,
      lastX: 0,
      lastY: 0,
    }};
    function mapApplyViewBox() {{
      if (!annotationSvg) return;
      annotationSvg.setAttribute("viewBox", [mapView.x, mapView.y, mapView.w, mapView.h].map((value) => value.toFixed(2)).join(" "));
    }}
    function mapSvgPoint(clientX, clientY) {{
      const point = annotationSvg.createSVGPoint();
      point.x = clientX;
      point.y = clientY;
      return point.matrixTransform(annotationSvg.getScreenCTM().inverse());
    }}
    function mapZoom(factor, clientX = null, clientY = null) {{
      if (!annotationSvg) return;
      const rect = annotationSvg.getBoundingClientRect();
      const focus = clientX == null || clientY == null
        ? {{ x: mapView.x + mapView.w * 0.5, y: mapView.y + mapView.h * 0.5 }}
        : mapSvgPoint(clientX, clientY);
      const nextW = Math.max(mapView.baseW * 0.08, Math.min(mapView.baseW * 1.25, mapView.w * factor));
      const nextH = Math.max(mapView.baseH * 0.08, Math.min(mapView.baseH * 1.25, mapView.h * factor));
      const rx = (focus.x - mapView.x) / mapView.w;
      const ry = (focus.y - mapView.y) / mapView.h;
      mapView.x = focus.x - nextW * rx;
      mapView.y = focus.y - nextH * ry;
      mapView.w = nextW;
      mapView.h = nextH;
      mapApplyViewBox();
    }}
    function mapResetZoom() {{
      mapView.x = mapView.baseX;
      mapView.y = mapView.baseY;
      mapView.w = mapView.baseW;
      mapView.h = mapView.baseH;
      mapApplyViewBox();
    }}
    function mapSetPan(enabled) {{
      mapView.pan = enabled;
      if (mapPanButton) {{
        mapPanButton.classList.toggle("annotation-active", enabled);
        mapPanButton.textContent = enabled ? "Pan on" : "Pan";
      }}
      if (annotationSvg) annotationSvg.style.cursor = enabled ? "grab" : "";
    }}
    document.querySelectorAll("[data-map-zoom]").forEach((button) => {{
      button.addEventListener("click", () => {{
        const action = button.dataset.mapZoom;
        if (action === "in") mapZoom(0.78);
        else if (action === "out") mapZoom(1.28);
        else mapResetZoom();
      }});
    }});
    if (mapPanButton) mapPanButton.addEventListener("click", () => mapSetPan(!mapView.pan));
    if (annotationSvg) {{
      annotationSvg.addEventListener("wheel", (event) => {{
        event.preventDefault();
        mapZoom(event.deltaY > 0 ? 1.16 : 0.86, event.clientX, event.clientY);
      }}, {{ passive: false }});
      annotationSvg.addEventListener("pointerdown", (event) => {{
        if (!mapView.pan || annotationState.active) return;
        mapView.dragging = true;
        mapView.lastX = event.clientX;
        mapView.lastY = event.clientY;
        annotationSvg.setPointerCapture(event.pointerId);
        annotationSvg.style.cursor = "grabbing";
      }});
      annotationSvg.addEventListener("pointermove", (event) => {{
        if (!mapView.dragging) return;
        const rect = annotationSvg.getBoundingClientRect();
        const dx = event.clientX - mapView.lastX;
        const dy = event.clientY - mapView.lastY;
        mapView.x -= dx * mapView.w / Math.max(rect.width, 1);
        mapView.y -= dy * mapView.h / Math.max(rect.height, 1);
        mapView.lastX = event.clientX;
        mapView.lastY = event.clientY;
        mapApplyViewBox();
      }});
      annotationSvg.addEventListener("pointerup", (event) => {{
        if (!mapView.dragging) return;
        mapView.dragging = false;
        try {{ annotationSvg.releasePointerCapture(event.pointerId); }} catch (error) {{}}
        annotationSvg.style.cursor = mapView.pan ? "grab" : "";
      }});
    }}
    function annotationMapPointFromEvent(event) {{
      const point = annotationSvg.createSVGPoint();
      point.x = event.clientX;
      point.y = event.clientY;
      const svgPoint = point.matrixTransform(annotationSvg.getScreenCTM().inverse());
      const minx = Number(annotationSvg.dataset.minx || 0);
      const maxy = Number(annotationSvg.dataset.maxy || 0);
      const scale = Number(annotationSvg.dataset.scale || 1);
      const pad = Number(annotationSvg.dataset.pad || 0);
      return [
        Number((minx + (svgPoint.x - pad) / scale).toFixed(4)),
        Number((maxy - (svgPoint.y - pad) / scale).toFixed(4)),
      ];
    }}
    function annotationTaskLabel(taskType) {{
      if (taskType === "dead_end_corridor") return "Dead end";
      if (taskType === "corridor") return "Corridor";
      if (taskType === "notch") return "Notch";
      return taskType.replaceAll("_", " ");
    }}
    function annotationSvgPoint(mapPoint) {{
      const minx = Number(annotationSvg.dataset.minx || 0);
      const maxy = Number(annotationSvg.dataset.maxy || 0);
      const scale = Number(annotationSvg.dataset.scale || 1);
      const pad = Number(annotationSvg.dataset.pad || 0);
      return [
        pad + (mapPoint[0] - minx) * scale,
        pad + (maxy - mapPoint[1]) * scale,
      ];
    }}
    function annotationSetStatus(text) {{
      if (annotationStatus) annotationStatus.textContent = text;
    }}
    function annotationUpdateSteps() {{
      annotationStepElements.forEach((element) => {{
        const step = Number(element.dataset.annotationStep || 0);
        element.classList.toggle("is-done", annotationState.draft.length > step);
        element.classList.toggle("is-current", annotationState.active && annotationState.draft.length === step);
      }});
    }}
    function annotationSaveToBrowser() {{
      try {{
        window.localStorage.setItem(annotationStorageKey, JSON.stringify(annotationState.annotations));
      }} catch (error) {{
        // Local storage is only a convenience for the static report.
      }}
    }}
    function annotationLoadFromBrowser() {{
      try {{
        const raw = window.localStorage.getItem(annotationStorageKey);
        if (!raw) return;
        const parsed = JSON.parse(raw);
        if (Array.isArray(parsed)) annotationState.annotations = parsed.filter((item) => item && typeof item === "object");
      }} catch (error) {{
        annotationState.annotations = [];
      }}
    }}
    function annotationAppendLine(ns, start, end, className) {{
      const line = document.createElementNS(ns, "line");
      line.setAttribute("x1", start[0].toFixed(2));
      line.setAttribute("y1", start[1].toFixed(2));
      line.setAttribute("x2", end[0].toFixed(2));
      line.setAttribute("y2", end[1].toFixed(2));
      line.setAttribute("class", className);
      annotationDraftLayer.appendChild(line);
      return line;
    }}
    function annotationAppendPoint(ns, point, label, service = false) {{
      const circle = document.createElementNS(ns, "circle");
      circle.setAttribute("cx", point[0].toFixed(2));
      circle.setAttribute("cy", point[1].toFixed(2));
      circle.setAttribute("r", service ? "6.5" : "5.2");
      circle.setAttribute("class", service ? "annotation-preview-point annotation-preview-service-point" : "annotation-preview-point");
      annotationDraftLayer.appendChild(circle);
      const text = document.createElementNS(ns, "text");
      text.setAttribute("x", (point[0] + 7).toFixed(2));
      text.setAttribute("y", (point[1] - 7).toFixed(2));
      text.setAttribute("class", "annotation-preview-label");
      text.textContent = label;
      annotationDraftLayer.appendChild(text);
    }}
    function annotationRenderOne(annotation, ns) {{
      const cutLine = annotation.cut_line || [];
      const servicePoint = annotation.service_point;
      if (cutLine.length !== 2 || !servicePoint) return;
      const first = annotationSvgPoint(cutLine[0]);
      const second = annotationSvgPoint(cutLine[1]);
      const service = annotationSvgPoint(servicePoint);
      annotationAppendLine(ns, first, second, "annotation-preview-line");
      const mid = [(first[0] + second[0]) / 2, (first[1] + second[1]) / 2];
      annotationAppendLine(ns, mid, service, "annotation-preview-service-line");
      annotationAppendPoint(ns, first, "1");
      annotationAppendPoint(ns, second, "2");
      annotationAppendPoint(ns, service, "side", true);
    }}
    function annotationRenderDraft() {{
      if (!annotationDraftLayer) return;
      annotationDraftLayer.innerHTML = "";
      const ns = "http://www.w3.org/2000/svg";
      annotationState.annotations.forEach((annotation) => annotationRenderOne(annotation, ns));
      const points = annotationState.draft.map(annotationSvgPoint);
      if (points.length >= 2) {{
        annotationAppendLine(ns, points[0], points[1], "annotation-preview-line");
      }}
      points.forEach((point, index) => {{
        const label = index === 2 ? "side" : String(index + 1);
        annotationAppendPoint(ns, point, label, index === 2);
      }});
      annotationUpdateSteps();
    }}
    function annotationRenderList() {{
      if (!annotationList) return;
      annotationList.innerHTML = "";
      if (annotationCount) annotationCount.textContent = String(annotationState.annotations.length);
      if (annotationState.annotations.length === 0) {{
        const item = document.createElement("li");
        const main = document.createElement("span");
        main.className = "annotation-list-detail";
        main.textContent = "No annotations yet. Start drawing on the map.";
        item.appendChild(main);
        annotationList.appendChild(item);
        return;
      }}
      annotationState.annotations.forEach((annotation, index) => {{
        const item = document.createElement("li");
        const main = document.createElement("span");
        main.className = "annotation-list-main";
        const title = document.createElement("span");
        title.className = "annotation-list-title-line";
        title.textContent = annotation.id + " " + annotationTaskLabel(annotation.task_type);
        const detail = document.createElement("span");
        detail.className = "annotation-list-detail";
        const label = annotation.label ? " - " + annotation.label : "";
        detail.textContent = "Area " + annotation.area_index + label;
        main.appendChild(title);
        main.appendChild(detail);
        const remove = document.createElement("button");
        remove.type = "button";
        remove.textContent = "Remove";
        remove.addEventListener("click", () => {{
          annotationState.annotations.splice(index, 1);
          annotationSaveToBrowser();
          annotationRenderDraft();
          annotationRenderList();
          annotationSetStatus("Removed annotation. Export JSON again when you are ready.");
        }});
        item.appendChild(main);
        item.appendChild(remove);
        annotationList.appendChild(item);
      }});
    }}
    function annotationPromptForNextPoint() {{
      if (!annotationState.active) return "Click Start drawing, then use the three map clicks above.";
      if (annotationState.draft.length === 0) return "Step 1: click one end of the mouth cut.";
      if (annotationState.draft.length === 1) return "Step 2: click the other end of the mouth cut.";
      if (annotationState.draft.length === 2) return "Step 3: click inside the service side to keep.";
      return "Annotation ready.";
    }}
    function annotationResetDraft() {{
      annotationState.draft = [];
      annotationRenderDraft();
      annotationSetStatus(annotationPromptForNextPoint());
    }}
    function annotationUndoPoint() {{
      if (annotationState.draft.length === 0) {{
        annotationSetStatus("No draft point to undo.");
        return;
      }}
      annotationState.draft.pop();
      annotationRenderDraft();
      annotationSetStatus(annotationPromptForNextPoint());
    }}
    function annotationDeleteLast() {{
      if (annotationState.annotations.length === 0) {{
        annotationSetStatus("No saved annotation to delete.");
        return;
      }}
      const removed = annotationState.annotations.pop();
      annotationSaveToBrowser();
      annotationRenderDraft();
      annotationRenderList();
      annotationSetStatus("Deleted " + removed.id + ".");
    }}
    function annotationAddFromDraft() {{
      if (annotationState.draft.length < 3) return;
      const taskType = document.getElementById("annotation-task-type")?.value || "dead_end_corridor";
      const areaIndex = Number(document.getElementById("annotation-area-index")?.value || 0);
      const label = document.getElementById("annotation-label")?.value || "";
      const note = document.getElementById("annotation-note")?.value || "";
      const annotation = {{
        id: "annotation-" + String(annotationState.annotations.length + 1).padStart(2, "0"),
        area_index: areaIndex,
        task_type: taskType,
        cut_line: [annotationState.draft[0], annotationState.draft[1]],
        service_point: annotationState.draft[2],
      }};
      if (label.trim()) annotation.label = label.trim();
      if (note.trim()) annotation.note = note.trim();
      annotationState.annotations.push(annotation);
      annotationSaveToBrowser();
      annotationState.draft = [];
      annotationRenderDraft();
      annotationRenderList();
      annotationSetStatus("Added " + annotation.id + ". Draw another cut or export JSON.");
    }}
    function annotationExport() {{
      if (annotationState.annotations.length === 0) {{
        annotationSetStatus("No annotations to export yet.");
        return;
      }}
      const payload = {{
        schema: "open_mower.coverage_lab.v2_task_annotations.v0",
        source_map: {json.dumps(str(result.get("source_map", "")))},
        frame_id: {json.dumps(str(result.get("frame_id", "map")))},
        annotations: annotationState.annotations,
      }};
      const blob = new Blob([JSON.stringify(payload, null, 2)], {{ type: "application/json" }});
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = "v2_task_annotations.json";
      document.body.appendChild(link);
      link.click();
      link.remove();
      URL.revokeObjectURL(url);
      annotationSetStatus("Exported " + annotationState.annotations.length + " annotations. Rerun with the command below.");
    }}
    function annotationCopyCommand() {{
      const command = document.getElementById("annotation-rerun-command")?.value || "";
      if (!command) return;
      if (navigator.clipboard && navigator.clipboard.writeText) {{
        navigator.clipboard.writeText(command).then(
          () => annotationSetStatus("Copied rerun command."),
          () => annotationSetStatus("Could not copy automatically. Select the command field manually.")
        );
      }} else {{
        annotationSetStatus("Select the command field and copy it manually.");
      }}
    }}
    if (annotationButton) {{
      annotationButton.addEventListener("click", () => {{
        annotationState.active = !annotationState.active;
        document.body.classList.toggle("annotation-mode", annotationState.active);
        annotationButton.classList.toggle("annotation-active", annotationState.active);
        annotationButton.textContent = annotationState.active ? "Stop drawing" : "Start drawing";
        if (annotationState.active) applyPreset("tasks");
        annotationRenderDraft();
        annotationSetStatus(annotationState.active ? annotationPromptForNextPoint() : "Annotation mode paused. Saved draft annotations stay visible.");
      }});
    }}
    if (annotationUndoButton) annotationUndoButton.addEventListener("click", annotationUndoPoint);
    if (annotationClearButton) annotationClearButton.addEventListener("click", annotationResetDraft);
    if (annotationDeleteLastButton) annotationDeleteLastButton.addEventListener("click", annotationDeleteLast);
    if (annotationExportButton) annotationExportButton.addEventListener("click", annotationExport);
    if (annotationCopyCommandButton) annotationCopyCommandButton.addEventListener("click", annotationCopyCommand);
    if (annotationSvg) {{
      annotationSvg.addEventListener("click", (event) => {{
        if (!annotationState.active) return;
        event.preventDefault();
        event.stopPropagation();
        annotationState.draft.push(annotationMapPointFromEvent(event));
        annotationSetStatus(annotationPromptForNextPoint());
        if (annotationState.draft.length >= 3) annotationAddFromDraft();
        annotationRenderDraft();
      }}, true);
    }}
    annotationLoadFromBrowser();
    annotationRenderDraft();
    annotationRenderList();
    annotationSetStatus(annotationPromptForNextPoint());
    layerInputs.forEach((input) => {{
      input.addEventListener("change", () => {{
        if (input.dataset.layer === "layer-task-artifacts") {{
          setArtifactVisibility(input.checked);
          return;
        }}
        const layer = document.getElementById(input.dataset.layer);
        if (layer) layer.style.display = input.checked ? "" : "none";
      }});
    }});
    if (artifactToggle) {{
      artifactToggle.addEventListener("change", () => setArtifactVisibility(artifactToggle.checked));
    }}
    document.querySelectorAll("[data-task-id]").forEach((element) => {{
      element.addEventListener("mouseenter", () => setActiveTask(element.dataset.taskId));
      element.addEventListener("mouseleave", restorePinnedTask);
      element.addEventListener("click", () => {{
        pinnedTaskId = pinnedTaskId === element.dataset.taskId ? null : element.dataset.taskId;
        restorePinnedTask();
      }});
    }});
    document.querySelectorAll("[data-preset]").forEach((button) => {{
      button.addEventListener("click", () => applyPreset(button.dataset.preset));
    }});
    applyPreset("tasks");
  </script>
</body>
</html>
"""
