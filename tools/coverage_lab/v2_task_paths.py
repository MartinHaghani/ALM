"""Diagnostic task-level path prototypes for coverage-planner V2.

This module intentionally stays lab-only. It consumes M1.6 task service
regions and emits local path sketches with direction/blade metadata, but it
does not by itself create mower-facing ``PlanPath`` output. The separate
``v2_planpath_compat`` adapter can flatten eligible forward cutting segments
into the old JSON shape as a temporary bridge.

>>> round(_path_length([(0, 0), (3, 4)]), 1)
5.0
>>> round(_normalize_angle(4 * math.pi), 3)
0.0
"""
from __future__ import annotations

import bisect
import math
from typing import Any, Iterable

from shapely.affinity import rotate
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union

import v2_geometry


EPS = 1e-9


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


def _float_list_config(config: dict[str, Any], key: str, default: list[float]) -> list[float]:
    value = config.get(key, default)
    raw_values: list[Any]
    if isinstance(value, str):
        raw_values = [part.strip() for part in value.split(",") if part.strip()]
    elif isinstance(value, (list, tuple)):
        raw_values = list(value)
    else:
        raw_values = list(default)
    values: list[float] = []
    for item in raw_values:
        try:
            parsed = float(item)
        except (TypeError, ValueError):
            continue
        if parsed > EPS:
            values.append(parsed)
    return sorted(set(round(value, 4) for value in values)) or list(default)


def task_path_options(config: dict[str, Any]) -> dict[str, Any]:
    outline_count = int(_float_config(config, "v2_outline_count", _float_config(config, "outline_count", 0)))
    outline_enabled = _bool_config(config, "v2_outline_enabled", outline_count > 0)
    outline_sample_step_m = max(
        0.03,
        _float_config(
            config,
            "v2_outline_sample_step_m",
            _float_config(config, "v2_task_path_sample_step_m", 0.10),
        ),
    )
    connector_style = str(config.get("v2_task_path_adjacent_connector_style", "straight")).strip().lower()
    if connector_style not in {"straight", "keyhole"}:
        connector_style = "straight"
    connector_unsafe_policy = (
        str(config.get("v2_task_path_connector_unsafe_policy", "reject")).strip().lower()
    )
    if connector_unsafe_policy not in {"reject", "include"}:
        connector_unsafe_policy = "reject"
    outline_generation_mode = str(config.get("v2_outline_generation_mode", "footprint_disk")).strip().lower()
    if outline_generation_mode not in {
        "footprint_disk",
        "right_edge_fit",
        "footprint_side_fit",
        "footprint_elastic_band",
        "footprint_cspace_shell",
    }:
        outline_generation_mode = "footprint_disk"
    raw_right_edge_clearance = config.get("v2_outline_right_edge_clearance_m", "auto")
    if raw_right_edge_clearance is not None and str(raw_right_edge_clearance).strip().lower() != "auto":
        outline_right_edge_clearance_m = max(0.0, float(raw_right_edge_clearance))
        outline_right_edge_clearance_source = "configured"
    else:
        outline_right_edge_clearance_m = max(
            0.0, _float_config(config, "v2_drivable_boundary_clearance_m", 0.0)
        )
        outline_right_edge_clearance_source = "drivable_boundary_clearance"
    raw_right_footprint_clearance = config.get("v2_outline_right_footprint_clearance_m", "auto")
    if (
        raw_right_footprint_clearance is not None
        and str(raw_right_footprint_clearance).strip().lower() != "auto"
    ):
        outline_right_footprint_clearance_m = max(0.0, float(raw_right_footprint_clearance))
        outline_right_footprint_clearance_source = "configured"
    else:
        outline_right_footprint_clearance_m = max(
            0.0,
            _float_config(
                config,
                "v2_outline_footprint_clearance_m",
                _float_config(config, "safety_margin_m", 0.05),
            ),
        )
        outline_right_footprint_clearance_source = "outline_footprint_clearance"
    return {
        "enabled": _bool_config(config, "v2_task_path_enabled", True),
        "outline_enabled": outline_enabled,
        "outline_count": max(0, outline_count),
        "outline_offset_m": max(0.0, _float_config(config, "outline_offset", 0.0)),
        "outline_sample_step_m": outline_sample_step_m,
        "outline_yaw_window_m": _outline_yaw_window(config, outline_sample_step_m),
        "outline_cutting_enabled": _bool_config(config, "v2_outline_cutting_enabled", True),
        "outline_generation_mode": outline_generation_mode,
        "outline_right_edge_clearance_m": outline_right_edge_clearance_m,
        "outline_right_edge_clearance_source": outline_right_edge_clearance_source,
        "outline_right_footprint_clearance_m": outline_right_footprint_clearance_m,
        "outline_right_footprint_clearance_source": outline_right_footprint_clearance_source,
        "outline_corner_candidate_radii_m": _float_list_config(
            config,
            "v2_outline_corner_candidate_radii_m",
            [0.15, 0.25, 0.35, 0.50, 0.70, 0.90],
        ),
        "outline_corner_sample_step_m": max(
            0.02,
            _float_config(config, "v2_outline_corner_sample_step_m", outline_sample_step_m * 0.5),
        ),
        "outline_boundary_simplify_tolerance_m": max(
            0.0,
            _float_config(config, "v2_outline_boundary_simplify_tolerance_m", 0.40),
        ),
        "outline_fit_offset_step_m": max(
            0.01, _float_config(config, "v2_outline_fit_offset_step_m", 0.05)
        ),
        "outline_fit_max_extra_m": max(
            0.0, _float_config(config, "v2_outline_fit_max_extra_m", 0.60)
        ),
        "outline_corner_fit_max_extra_m": max(
            0.0,
            _float_config(
                config,
                "v2_outline_corner_fit_max_extra_m",
                _float_config(config, "v2_outline_fit_max_extra_m", 0.60),
            ),
        ),
        "outline_global_fit_max_extra_m": max(
            0.0, _float_config(config, "v2_outline_global_fit_max_extra_m", 0.0)
        ),
        "outline_footprint_clearance_m": max(
            0.0,
            _float_config(
                config,
                "v2_outline_footprint_clearance_m",
                _float_config(config, "safety_margin_m", 0.0),
            ),
        ),
        "outline_trajectory_yaw_window_m": max(
            outline_sample_step_m * 2.0,
            _float_config(
                config,
                "v2_outline_trajectory_yaw_window_m",
                max(0.80, _float_config(config, "v2_outline_yaw_window_m", 0.0)),
            ),
        ),
        "outline_trajectory_iterations": max(
            0, int(_float_config(config, "v2_outline_trajectory_iterations", 35.0))
        ),
        "outline_trajectory_anchor_weight": max(
            0.01, _float_config(config, "v2_outline_trajectory_anchor_weight", 1.0)
        ),
        "outline_trajectory_smooth_weight": max(
            0.0, _float_config(config, "v2_outline_trajectory_smooth_weight", 0.45)
        ),
        "outline_trajectory_projection_step_m": max(
            0.005, _float_config(config, "v2_outline_trajectory_projection_step_m", 0.025)
        ),
        "outline_trajectory_projection_max_extra_m": max(
            0.0,
            _float_config(
                config,
                "v2_outline_trajectory_projection_max_extra_m",
                _float_config(config, "v2_outline_fit_max_extra_m", 0.60),
            ),
        ),
        "outline_trajectory_max_yaw_step_deg": max(
            1.0, _float_config(config, "v2_outline_trajectory_max_yaw_step_deg", 6.0)
        ),
        "outline_cspace_yaw_sweep_deg": max(
            0.0, _float_config(config, "v2_outline_cspace_yaw_sweep_deg", 35.0)
        ),
        "outline_cspace_yaw_step_deg": max(
            1.0, _float_config(config, "v2_outline_cspace_yaw_step_deg", 5.0)
        ),
        "outline_cspace_offset_step_m": max(
            0.005, _float_config(config, "v2_outline_cspace_offset_step_m", 0.025)
        ),
        "outline_cspace_max_extra_m": max(
            0.0,
            _float_config(
                config,
                "v2_outline_cspace_max_extra_m",
                _float_config(config, "v2_outline_fit_max_extra_m", 0.60),
            ),
        ),
        "outline_start_selector_enabled": _bool_config(
            config, "v2_outline_start_selector_enabled", True
        ),
        "outline_start_straight_angle_tolerance_deg": max(
            0.0, _float_config(config, "v2_outline_start_straight_angle_tolerance_deg", 8.0)
        ),
        "spacing_factor": max(0.10, _float_config(config, "v2_task_path_spacing_factor", 0.90)),
        "min_pass_length_m": max(0.0, _float_config(config, "v2_task_path_min_pass_length_m", 0.35)),
        "body_min_pass_length_m": max(
            0.0, _float_config(config, "v2_task_path_body_min_pass_length_m", 2.0)
        ),
        "sample_step_m": max(0.03, _float_config(config, "v2_task_path_sample_step_m", 0.10)),
        "compact_connectors_enabled": _bool_config(
            config, "v2_task_path_compact_connectors_enabled", True
        ),
        "adjacent_connector_style": connector_style,
        "compact_connector_max_distance_m": max(
            0.0, _float_config(config, "v2_task_path_compact_connector_max_distance_m", 1.50)
        ),
        "straight_connector_max_distance_m": max(
            0.0, _float_config(config, "v2_task_path_straight_connector_max_distance_m", 5.00)
        ),
        "straight_connector_min_heading_delta_deg": max(
            0.0, _float_config(config, "v2_task_path_straight_connector_min_heading_delta_deg", 0.0)
        ),
        "connector_unsafe_policy": connector_unsafe_policy,
        "compact_connector_min_heading_delta_deg": max(
            0.0, _float_config(config, "v2_task_path_compact_connector_min_heading_delta_deg", 120.0)
        ),
        "compact_connector_extra_forward_m": max(
            0.0, _float_config(config, "v2_task_path_compact_connector_extra_forward_m", 0.35)
        ),
        "compact_connector_max_forward_extension_m": max(
            0.0, _float_config(config, "v2_task_path_compact_connector_max_forward_extension_m", 0.65)
        ),
        "compact_connector_turn_radius_m": max(
            0.05,
            _float_config(
                config,
                "v2_task_path_compact_connector_turn_radius_m",
                _float_config(config, "omega_radius_m", 0.60),
            ),
        ),
        "compact_connector_sample_step_m": max(
            0.03,
            _float_config(
                config,
                "v2_task_path_compact_connector_sample_step_m",
                _float_config(config, "v2_task_path_sample_step_m", 0.10),
            ),
        ),
        "stripe_boundary_clearance_factor": max(
            0.0, _float_config(config, "v2_task_path_stripe_boundary_clearance_factor", 1.0)
        ),
        "body_pocket_enabled": _bool_config(config, "v2_task_path_body_pocket_enabled", True),
        "body_pocket_outline_band_m": max(
            0.0, _float_config(config, "v2_task_path_body_pocket_outline_band_m", 0.50)
        ),
        "body_pocket_min_area_m2": max(
            0.0, _float_config(config, "v2_task_path_body_pocket_min_area_m2", 0.50)
        ),
        "body_pocket_min_width_factor": max(
            0.0, _float_config(config, "v2_task_path_body_pocket_min_width_factor", 0.60)
        ),
        "body_pocket_max_strokes": max(1, int(_float_config(config, "v2_task_path_body_pocket_max_strokes", 4))),
        "reverse_out_enabled": _bool_config(config, "v2_task_path_reverse_out_enabled", True),
        "reverse_cutting_enabled": _bool_config(config, "v2_task_path_reverse_cutting_enabled", False),
        "terminal_angle_tolerance_deg": max(
            0.0, _float_config(config, "v2_task_path_terminal_angle_tolerance_deg", 35.0)
        ),
        "side_lane_min_length_factor": max(
            0.0, _float_config(config, "v2_task_path_side_lane_min_length_factor", 0.55)
        ),
        "notch_max_strokes": max(1, int(_float_config(config, "v2_task_path_notch_max_strokes", 3))),
        "service_axis_ray_pad_m": max(0.0, _float_config(config, "v2_task_path_service_axis_ray_pad_m", 0.50)),
        "axis_candidate_scoring_enabled": _bool_config(
            config, "v2_task_path_axis_candidate_scoring_enabled", True
        ),
        "axis_candidate_sweep_deg": max(
            0.0, _float_config(config, "v2_task_path_axis_candidate_sweep_degrees", 25.0)
        ),
        "axis_candidate_step_deg": max(
            1.0, _float_config(config, "v2_task_path_axis_candidate_step_degrees", 12.5)
        ),
        "axis_candidate_max_count": max(4, int(_float_config(config, "v2_task_path_axis_candidate_max_count", 24))),
        "ownership_enabled": _bool_config(config, "v2_task_ownership_enabled", True),
        "ownership_portal_band_m": max(0.0, _float_config(config, "v2_task_ownership_portal_band_m", 0.90)),
        "ownership_short_body_pass_m": max(
            0.0, _float_config(config, "v2_task_ownership_short_body_pass_m", 1.20)
        ),
        "ownership_axis_alignment_deg": max(
            0.0, _float_config(config, "v2_task_ownership_axis_alignment_deg", 35.0)
        ),
        "dead_end_turnaround_enabled": _bool_config(config, "v2_task_path_dead_end_turnaround_enabled", True),
        "turnaround_yaw_step_deg": max(1.0, _float_config(config, "v2_task_path_turnaround_yaw_step_deg", 10.0)),
        "turnaround_clearance_m": max(0.0, _float_config(config, "v2_task_path_turnaround_clearance_m", 0.05)),
        "turnaround_terminal_band_m": max(
            0.0, _float_config(config, "v2_task_path_turnaround_terminal_band_m", 0.80)
        ),
    }


def _normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def _round_point(x: float, y: float) -> list[float]:
    return [round(float(x), 4), round(float(y), 4)]


def _path_length(points: Iterable[tuple[float, float]]) -> float:
    pts = list(points)
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def _tool_width(config: dict[str, Any]) -> float:
    return max(0.05, float(config.get("tool_width", 0.4)))


def _mower_width(config: dict[str, Any]) -> float:
    footprint = config.get("footprint") or []
    margin = max(0.0, _float_config(config, "safety_margin_m", 0.0))
    if len(footprint) >= 3:
        try:
            ys = [float(point[1]) for point in footprint]
            footprint_width = max(ys) - min(ys)
            return max(_tool_width(config), footprint_width + 2.0 * margin)
        except (TypeError, ValueError, IndexError):
            pass
    return _tool_width(config) + 2.0 * margin


def _outline_clearance(config: dict[str, Any]) -> float:
    raw = config.get("v2_outline_clearance_m", config.get("outline_clearance_m", "auto"))
    if raw is not None and str(raw).strip().lower() != "auto":
        return max(_tool_width(config) * 0.5, float(raw))
    drivable_clearance = max(0.0, _float_config(config, "v2_drivable_boundary_clearance_m", 0.0))
    return drivable_clearance + _mower_width(config) * 0.5


def _outline_clearance_source(config: dict[str, Any]) -> str:
    raw = config.get("v2_outline_clearance_m", config.get("outline_clearance_m", "auto"))
    if raw is not None and str(raw).strip().lower() != "auto":
        return "configured"
    return "drivable_boundary_plus_half_mower_width"


def _outline_yaw_window(config: dict[str, Any], sample_step_m: float) -> float:
    raw = config.get("v2_outline_yaw_window_m", "auto")
    if raw is not None and str(raw).strip().lower() != "auto":
        return max(0.0, float(raw))
    ox, oy = _tool_center_offset(config)
    return max(float(sample_step_m) * 4.0, 2.0 * math.hypot(ox, oy))


def _stripe_clearance_metadata(config: dict[str, Any], options: dict[str, Any]) -> dict[str, Any]:
    target_clearance = _mower_width(config) * float(options["stripe_boundary_clearance_factor"])
    existing_drivable_clearance = max(0.0, _float_config(config, "v2_drivable_boundary_clearance_m", 0.0))
    drivable_inset = max(0.0, target_clearance - existing_drivable_clearance)
    return {
        "stripe_boundary_clearance_m": round(float(target_clearance), 4),
        "stripe_drivable_inset_m": round(float(drivable_inset), 4),
        "stripe_clearance_source": "safety_footprint_width",
    }


def _stripe_generation_region(
    coverage_region: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[Polygon | MultiPolygon, dict[str, Any]]:
    metadata = _stripe_clearance_metadata(config, options)
    inset = float(metadata["stripe_drivable_inset_m"])
    if inset <= EPS:
        metadata.update(
            {
                "stripe_region_source": "coverage_region",
                "stripe_region_area_m2": round(v2_geometry.area_m2(coverage_region), 4),
            }
        )
        return coverage_region, metadata
    safe_center_region = v2_geometry._clean_polygonal(drivable.buffer(-inset, join_style=2))
    stripe_region = v2_geometry._clean_polygonal(coverage_region.intersection(safe_center_region))
    metadata.update(
        {
            "stripe_region_source": "coverage_region_intersect_drivable_inset",
            "stripe_region_area_m2": round(v2_geometry.area_m2(stripe_region), 4),
            "stripe_region": v2_geometry.geometry_to_json(stripe_region),
        }
    )
    return stripe_region, metadata


def _tool_center_offset(config: dict[str, Any]) -> tuple[float, float]:
    raw = config.get("tool_center_offset", [0.0, 0.0])
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return (float(raw[0]), float(raw[1]))
    return (0.0, 0.0)


def _iter_lines(geom: Any) -> list[LineString]:
    if geom.is_empty:
        return []
    if isinstance(geom, LineString):
        return [geom] if geom.length > EPS else []
    if isinstance(geom, MultiLineString):
        return [line for line in geom.geoms if line.length > EPS]
    if hasattr(geom, "geoms"):
        lines: list[LineString] = []
        for part in geom.geoms:
            lines.extend(_iter_lines(part))
        return lines
    return []


def _largest_polygon(geom: Any) -> Polygon | None:
    polygons = v2_geometry._iter_polygons(geom)
    if not polygons:
        return None
    return max(polygons, key=lambda poly: poly.area)


def _line_points(line: LineString, reverse_points: bool = False) -> list[tuple[float, float]]:
    coords = [(float(x), float(y)) for x, y in line.coords]
    if reverse_points:
        coords.reverse()
    return coords


def _point_from_record(raw: Any) -> tuple[float, float] | None:
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return (float(raw[0]), float(raw[1]))
    return None


def _vector_between(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    return (float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))


def _normalize_vector(vector: tuple[float, float]) -> tuple[float, float] | None:
    norm = math.hypot(vector[0], vector[1])
    if norm <= EPS:
        return None
    return (vector[0] / norm, vector[1] / norm)


def _dot(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _angle_error_deg(a: tuple[float, float] | None, b: tuple[float, float] | None) -> float | None:
    if a is None or b is None:
        return None
    value = max(-1.0, min(1.0, _dot(a, b)))
    return math.degrees(math.acos(value))


def _unit_from_angle_deg(angle_deg: float) -> tuple[float, float]:
    radians = math.radians(angle_deg)
    return (math.cos(radians), math.sin(radians))


def _rotate_unit(direction: tuple[float, float], angle_deg: float) -> tuple[float, float]:
    radians = math.radians(angle_deg)
    cos_a = math.cos(radians)
    sin_a = math.sin(radians)
    return (
        direction[0] * cos_a - direction[1] * sin_a,
        direction[0] * sin_a + direction[1] * cos_a,
    )


def _point_along(
    start: tuple[float, float],
    direction: tuple[float, float],
    distance: float,
) -> tuple[float, float]:
    return (start[0] + direction[0] * distance, start[1] + direction[1] * distance)


def _offset_point(
    point: tuple[float, float],
    lateral: tuple[float, float],
    offset: float,
) -> tuple[float, float]:
    return (point[0] + lateral[0] * offset, point[1] + lateral[1] * offset)


def _region_diagonal(region: Polygon | MultiPolygon) -> float:
    if region.is_empty:
        return 0.0
    minx, miny, maxx, maxy = region.bounds
    return math.hypot(maxx - minx, maxy - miny)


def _portal_record(
    task: dict[str, Any],
    portal_by_id: dict[str, dict[str, Any]],
) -> tuple[str | None, dict[str, Any] | None]:
    for portal_id in task.get("entry_portals", []):
        portal = portal_by_id.get(str(portal_id))
        if portal:
            return str(portal_id), portal
    return None, None


def _portal_center(
    task: dict[str, Any],
    portal_by_id: dict[str, dict[str, Any]],
    region: Polygon | MultiPolygon,
) -> tuple[float, float]:
    _portal_id, portal = _portal_record(task, portal_by_id)
    if portal:
        center = _point_from_record(portal.get("center"))
        if center is not None:
            return center
    return _portal_point(task, portal_by_id, region)


def _portal_cut_unit(portal: dict[str, Any] | None) -> tuple[float, float] | None:
    if not portal:
        return None
    for segment in portal.get("line_segments", []):
        if len(segment) < 2:
            continue
        start = _point_from_record(segment[0])
        end = _point_from_record(segment[1])
        if start is None or end is None:
            continue
        unit = _normalize_vector(_vector_between(start, end))
        if unit is not None:
            return unit
    return None


def _portal_inward_unit(
    portal: dict[str, Any] | None,
    region: Polygon | MultiPolygon,
    center: tuple[float, float],
) -> tuple[float, float] | None:
    cut = _portal_cut_unit(portal)
    if cut is None:
        rep = region.representative_point()
        return _normalize_vector((float(rep.x) - center[0], float(rep.y) - center[1]))
    candidates = [(-cut[1], cut[0]), (cut[1], -cut[0])]
    rep = region.representative_point()
    to_rep = _normalize_vector((float(rep.x) - center[0], float(rep.y) - center[1])) or candidates[0]
    buffered_region = region.buffer(0.02)
    probe_distance = max(0.05, min(0.25, float(portal.get("width_m", 0.5)) * 0.25 if portal else 0.10))

    def score(candidate: tuple[float, float]) -> float:
        inward_probe = Point(*_point_along(center, candidate, probe_distance))
        outward_probe = Point(*_point_along(center, (-candidate[0], -candidate[1]), probe_distance))
        inside_score = 0.0
        if buffered_region.covers(inward_probe):
            inside_score += 1.0
        if buffered_region.covers(outward_probe):
            inside_score -= 1.0
        ray_length = _path_length(_axis_line_points(region, center, candidate, probe_distance))
        return ray_length * 2.0 + _dot(candidate, to_rep) + inside_score

    return max(candidates, key=score)


def _axis_line_points(
    region: Polygon | MultiPolygon,
    origin: tuple[float, float],
    direction: tuple[float, float],
    pad_m: float,
) -> list[tuple[float, float]]:
    span = _region_diagonal(region) + max(0.5, pad_m)
    start = _point_along(origin, direction, -max(0.05, pad_m))
    end = _point_along(origin, direction, span + max(0.05, pad_m))
    clipped = LineString([start, end]).intersection(region)
    pieces = sorted(
        _iter_lines(clipped),
        key=lambda line: (line.distance(Point(*origin)), -float(line.length)),
    )
    if not pieces:
        return []
    return _orient_points_near_to_far(_line_points(pieces[0]), origin)


def _deepest_axis_point(
    region: Polygon | MultiPolygon,
    origin: tuple[float, float],
    direction: tuple[float, float],
    pad_m: float,
) -> tuple[float, float]:
    points = _axis_line_points(region, origin, direction, pad_m)
    if len(points) >= 2:
        return points[-1]
    return _point_along(origin, direction, max(0.0, _region_diagonal(region)))


def _service_axis(
    *,
    task_type: str,
    task: dict[str, Any],
    region: Polygon | MultiPolygon,
    portal_by_id: dict[str, dict[str, Any]],
    options: dict[str, Any],
) -> dict[str, Any]:
    portal_id, portal = _portal_record(task, portal_by_id)
    portal = portal or {}
    start = _portal_center(task, portal_by_id, region)
    inward = _portal_inward_unit(portal, region, start)
    terminal = _terminal_point(task, region, start)
    terminal_unit = _normalize_vector(_vector_between(start, terminal))
    angle_error = _angle_error_deg(terminal_unit, inward)
    pad_m = float(options["service_axis_ray_pad_m"])

    source = "portal_normal_deepest_ray"
    direction = inward or terminal_unit or (1.0, 0.0)
    end = _deepest_axis_point(region, start, direction, pad_m)

    if task_type == "dead_end_corridor" and terminal_unit is not None:
        terminal_length = math.hypot(terminal[0] - start[0], terminal[1] - start[1])
        normal_length = math.hypot(end[0] - start[0], end[1] - start[1])
        normal_is_too_shallow = normal_length < terminal_length * float(options["side_lane_min_length_factor"])
        source_axis = _source_segment_axis(task, region, start, inward, pad_m)
        if source_axis is not None and float(source_axis["length_m"]) >= max(
            float(options["min_pass_length_m"]),
            terminal_length * 0.45,
        ):
            source = "source_centerline"
            direction = source_axis["direction"]
            end = source_axis["end"]
            angle_error = source_axis.get("angle_error_deg")
        elif (
            inward is None
            or angle_error is None
            or angle_error <= float(options["terminal_angle_tolerance_deg"])
            or normal_is_too_shallow
        ):
            source = "terminal_cap" if task.get("terminal_cap") else "terminal_point"
            if normal_is_too_shallow and angle_error and angle_error > float(options["terminal_angle_tolerance_deg"]):
                source = f"{source}_depth_override"
            direction = terminal_unit
            end = terminal
        else:
            source = "portal_normal_longest_ray"
            direction = inward
            end = _deepest_axis_point(region, start, direction, pad_m)

    length = math.hypot(end[0] - start[0], end[1] - start[1])
    yaw = math.atan2(direction[1], direction[0])
    return {
        "source": source,
        "portal_id": portal_id,
        "start": _round_point(start[0], start[1]),
        "end": _round_point(end[0], end[1]),
        "yaw": round(float(yaw), 6),
        "length_m": round(float(length), 4),
        "terminal_angle_error_deg": (
            round(float(angle_error), 2) if angle_error is not None else None
        ),
        "_direction": direction,
        "_origin": start,
    }


def _lateral_bounds(
    region: Polygon | MultiPolygon,
    origin: tuple[float, float],
    lateral: tuple[float, float],
) -> tuple[float, float]:
    offsets: list[float] = []
    for poly in v2_geometry._iter_polygons(region):
        rings = [poly.exterior, *poly.interiors]
        for ring in rings:
            for x, y in ring.coords:
                offsets.append((float(x) - origin[0]) * lateral[0] + (float(y) - origin[1]) * lateral[1])
    if not offsets:
        return (0.0, 0.0)
    return (min(offsets), max(offsets))


def _center_first_offsets(
    min_offset: float,
    max_offset: float,
    spacing_m: float,
    max_lanes: int | None,
) -> list[float]:
    offsets = [0.0]
    max_abs = max(abs(min_offset), abs(max_offset))
    step = 1
    while step * spacing_m <= max_abs + spacing_m * 0.35:
        positive = step * spacing_m
        negative = -positive
        if positive <= max_offset + spacing_m * 0.25:
            offsets.append(positive)
        if negative >= min_offset - spacing_m * 0.25:
            offsets.append(negative)
        step += 1
    if max_lanes is not None:
        offsets = offsets[:max_lanes]
    return offsets


def _single_entry_pass_records(
    *,
    task_type: str,
    task: dict[str, Any],
    region: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    portal_by_id: dict[str, dict[str, Any]],
    config: dict[str, Any],
    spacing_m: float,
    min_length_m: float,
    options: dict[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    def records_for_axis(axis_record: dict[str, Any]) -> list[dict[str, Any]]:
        direction = axis_record["_direction"]
        origin = axis_record["_origin"]
        lateral = (-direction[1], direction[0])
        min_offset, max_offset = _lateral_bounds(region, origin, lateral)
        max_lanes = int(options["notch_max_strokes"]) if task_type == "notch" else None
        offsets = _center_first_offsets(min_offset, max_offset, spacing_m, max_lanes)
        side_length_factor = float(options["side_lane_min_length_factor"])
        pad_m = float(options["service_axis_ray_pad_m"])
        records: list[dict[str, Any]] = []
        center_length = 0.0

        for offset in offsets:
            lane_origin = _offset_point(origin, lateral, offset)
            points = _axis_line_points(region, lane_origin, direction, pad_m)
            if len(points) < 2:
                continue
            length = _path_length(points)
            if abs(offset) <= EPS:
                if length < min_length_m:
                    continue
                center_length = length
            else:
                required_length = min_length_m
                if center_length > EPS:
                    required_length = max(required_length, center_length * side_length_factor)
                elif task_type != "notch":
                    continue
                if length < required_length:
                    continue
            records.append(
                {
                    "points": points,
                    "lane_offset_m": round(float(offset), 4),
                    "entry_anchor": _round_point(points[0][0], points[0][1]),
                    "deep_anchor": _round_point(points[-1][0], points[-1][1]),
                }
            )

        for lane_index, record in enumerate(records):
            record["lane_index"] = lane_index
        return records

    scored: list[tuple[float, dict[str, Any], list[dict[str, Any]], dict[str, Any]]] = []
    candidates = _axis_candidates(
        task_type=task_type,
        task=task,
        region=region,
        portal_by_id=portal_by_id,
        options=options,
    )
    for candidate in candidates:
        axis_record = {
            "source": candidate.get("source", "unknown"),
            "portal_id": candidate.get("portal_id"),
            "start": _round_point(candidate["start"][0], candidate["start"][1]),
            "end": _round_point(candidate["end"][0], candidate["end"][1]),
            "yaw": round(float(_axis_candidate_key(candidate)), 6),
            "length_m": round(float(candidate.get("length_m", 0.0)), 4),
            "terminal_angle_error_deg": (
                round(float(candidate["terminal_angle_error_deg"]), 2)
                if candidate.get("terminal_angle_error_deg") is not None
                else None
            ),
            "_direction": candidate["direction"],
            "_origin": candidate["start"],
        }
        records = records_for_axis(axis_record)
        summary = _axis_candidate_summary(task_type, candidate, records, region, drivable, config, options)
        scored.append((float(summary["score"]), axis_record, records, summary))

    scored.sort(key=lambda item: item[0], reverse=True)
    axis_candidates: list[dict[str, Any]] = []
    axis: dict[str, Any]
    records: list[dict[str, Any]]
    if scored:
        _score, axis, records, _summary = scored[0]
        for rank, (_candidate_score, _candidate_axis, _candidate_records, summary) in enumerate(scored, start=1):
            summary["rank"] = rank
            summary["selected"] = rank == 1
            summary["status"] = "selected" if rank == 1 else "rejected"
            axis_candidates.append(summary)
        axis["selection_score"] = axis_candidates[0]["score"]
        axis["candidate_rank"] = 1
        axis["candidate_count"] = len(axis_candidates)
    else:
        axis = _service_axis(
            task_type=task_type,
            task=task,
            region=region,
            portal_by_id=portal_by_id,
            options=options,
        )
        records = records_for_axis(axis)

    if not records and task_type == "notch":
        origin = axis["_origin"]
        deepest = _farthest_region_point(region, origin)
        fallback_direction = _normalize_vector(_vector_between(origin, deepest))
        if fallback_direction is not None:
            yaw = math.atan2(fallback_direction[1], fallback_direction[0])
            axis = {
                "source": "deepest_region_point_fallback",
                "portal_id": axis.get("portal_id"),
                "start": _round_point(origin[0], origin[1]),
                "end": _round_point(deepest[0], deepest[1]),
                "yaw": round(float(yaw), 6),
                "length_m": round(float(math.hypot(deepest[0] - origin[0], deepest[1] - origin[1])), 4),
                "terminal_angle_error_deg": axis.get("terminal_angle_error_deg"),
                "_direction": fallback_direction,
                "_origin": origin,
            }
            records = records_for_axis(axis)
            if not axis_candidates:
                fallback_candidate = {
                    "source": axis["source"],
                    "portal_id": axis.get("portal_id"),
                    "start": origin,
                    "end": deepest,
                    "direction": fallback_direction,
                    "length_m": math.hypot(deepest[0] - origin[0], deepest[1] - origin[1]),
                    "terminal_angle_error_deg": axis.get("terminal_angle_error_deg"),
                }
                summary = _axis_candidate_summary(task_type, fallback_candidate, records, region, drivable, config, options)
                summary["rank"] = 1
                summary["selected"] = True
                summary["status"] = "selected"
                axis_candidates = [summary]
                axis["selection_score"] = summary["score"]
                axis["candidate_rank"] = 1
                axis["candidate_count"] = 1

    clean_axis = {key: value for key, value in axis.items() if not key.startswith("_")}
    return clean_axis, records, axis_candidates


def _sample_points(points: list[tuple[float, float]], sample_step: float) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points
    line = LineString(points)
    length = float(line.length)
    if length <= EPS:
        return [points[0]]
    step = sample_step if sample_step > EPS else length
    sampled: list[tuple[float, float]] = []
    distance = 0.0
    while distance < length:
        point = line.interpolate(distance)
        sampled.append((float(point.x), float(point.y)))
        distance += step
    end = line.interpolate(length)
    sampled.append((float(end.x), float(end.y)))
    return sampled


def _segment_yaw(points: list[tuple[float, float]], direction: str) -> float:
    if len(points) < 2:
        return 0.0
    start = points[0]
    end = points[-1]
    travel_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    if direction == "backward":
        return _normalize_angle(travel_yaw + math.pi)
    return travel_yaw


def _base_pose_from_tool_point(
    point: tuple[float, float],
    yaw: float,
    offset: tuple[float, float],
) -> dict[str, float]:
    ox, oy = offset
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return {
        "x": round(float(point[0] - ox * cos_y + oy * sin_y), 4),
        "y": round(float(point[1] - ox * sin_y - oy * cos_y), 4),
        "yaw": round(float(yaw), 6),
    }


def _tool_pose_from_base_pose(
    base_pose: dict[str, float],
    offset: tuple[float, float],
) -> dict[str, float]:
    yaw = float(base_pose.get("yaw", 0.0))
    ox, oy = offset
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return {
        "x": round(float(base_pose.get("x", 0.0)) + ox * cos_y - oy * sin_y, 4),
        "y": round(float(base_pose.get("y", 0.0)) + ox * sin_y + oy * cos_y, 4),
        "yaw": round(float(_normalize_angle(yaw)), 6),
    }


def _footprint_polygon(base_pose: dict[str, float], config: dict[str, Any]) -> Polygon:
    footprint = config.get("footprint") or []
    if len(footprint) < 3:
        return Point(float(base_pose["x"]), float(base_pose["y"])).buffer(_tool_width(config) * 0.5)
    yaw = float(base_pose.get("yaw", 0.0))
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    bx = float(base_pose.get("x", 0.0))
    by = float(base_pose.get("y", 0.0))
    points = []
    for px, py in footprint:
        px = float(px)
        py = float(py)
        points.append((bx + px * cos_y - py * sin_y, by + px * sin_y + py * cos_y))
    return Polygon(points)


def _covers_with_tolerance(region: Polygon | MultiPolygon, geometry: Any, tolerance_m: float = 1e-4) -> bool:
    if region.is_empty:
        return False
    if region.covers(geometry):
        return True
    return bool(tolerance_m > 0.0 and region.buffer(tolerance_m).covers(geometry))


def _footprint_extents(config: dict[str, Any]) -> dict[str, float]:
    footprint = config.get("footprint") or []
    if len(footprint) < 3:
        half = _tool_width(config) * 0.5
        return {
            "min_x": -half,
            "max_x": half,
            "min_y": -half,
            "max_y": half,
            "right_y": -half,
            "right_offset_m": half,
        }
    xs = [float(point[0]) for point in footprint]
    ys = [float(point[1]) for point in footprint]
    right_y = min(ys)
    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
        "right_y": right_y,
        "right_offset_m": abs(right_y),
    }


def _transform_local_point(
    base_pose: dict[str, float],
    local_point: tuple[float, float],
) -> tuple[float, float]:
    yaw = float(base_pose.get("yaw", 0.0))
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    bx = float(base_pose.get("x", 0.0))
    by = float(base_pose.get("y", 0.0))
    px, py = local_point
    return (bx + px * cos_y - py * sin_y, by + px * sin_y + py * cos_y)


def _footprint_right_side_line(base_pose: dict[str, float], config: dict[str, Any]) -> LineString:
    extents = _footprint_extents(config)
    right_y = float(extents["right_y"])
    rear = _transform_local_point(base_pose, (float(extents["min_x"]), right_y))
    front = _transform_local_point(base_pose, (float(extents["max_x"]), right_y))
    return LineString([rear, front])


def _footprint_right_side_midpoint(base_pose: dict[str, float], config: dict[str, Any]) -> tuple[float, float]:
    extents = _footprint_extents(config)
    right_y = float(extents["right_y"])
    midpoint_x = (float(extents["min_x"]) + float(extents["max_x"])) * 0.5
    return _transform_local_point(base_pose, (midpoint_x, right_y))


def _footprint_right_corner_points(
    base_pose: dict[str, float],
    config: dict[str, Any],
) -> tuple[tuple[float, float], tuple[float, float]]:
    extents = _footprint_extents(config)
    right_y = float(extents["right_y"])
    rear = _transform_local_point(base_pose, (float(extents["min_x"]), right_y))
    front = _transform_local_point(base_pose, (float(extents["max_x"]), right_y))
    return rear, front


def _tool_pose_records(points: list[tuple[float, float]], yaw: float) -> list[dict[str, float]]:
    return [{"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 6)} for x, y in points]


def _line_segments_for_points(points: list[tuple[float, float]]) -> list[list[list[float]]]:
    return [[_round_point(a[0], a[1]), _round_point(b[0], b[1])] for a, b in zip(points, points[1:])]


def _portal_point(task: dict[str, Any], portal_by_id: dict[str, dict[str, Any]], region: Polygon | MultiPolygon) -> tuple[float, float]:
    for portal_id in task.get("entry_portals", []):
        portal = portal_by_id.get(str(portal_id))
        center = portal.get("center", []) if portal else []
        if len(center) >= 2:
            return (float(center[0]), float(center[1]))
    if region.is_empty:
        return (0.0, 0.0)
    point = region.representative_point()
    return (float(point.x), float(point.y))


def _farthest_region_point(region: Polygon | MultiPolygon, origin: tuple[float, float]) -> tuple[float, float]:
    best = origin
    best_distance = -1.0
    for poly in v2_geometry._iter_polygons(region):
        rings = [poly.exterior, *poly.interiors]
        for ring in rings:
            for x, y in ring.coords:
                distance = math.hypot(float(x) - origin[0], float(y) - origin[1])
                if distance > best_distance:
                    best = (float(x), float(y))
                    best_distance = distance
    return best


def _terminal_point(task: dict[str, Any], region: Polygon | MultiPolygon, portal: tuple[float, float]) -> tuple[float, float]:
    terminal = task.get("terminal_cap") or {}
    center = terminal.get("center", [])
    if len(center) >= 2:
        return (float(center[0]), float(center[1]))
    return _farthest_region_point(region, portal)


def _source_segment_axis(
    task: dict[str, Any],
    region: Polygon | MultiPolygon,
    origin: tuple[float, float],
    inward: tuple[float, float] | None,
    pad_m: float,
) -> dict[str, Any] | None:
    raw_segments = task.get("source_line_segments") or []
    candidates: list[dict[str, Any]] = []
    for segment in raw_segments:
        if not isinstance(segment, list) or len(segment) < 2:
            continue
        start = _point_from_record(segment[0])
        end = _point_from_record(segment[-1])
        if start is None or end is None:
            continue
        unit = _normalize_vector(_vector_between(start, end))
        if unit is None:
            continue
        for direction in (unit, (-unit[0], -unit[1])):
            points = _axis_line_points(region, origin, direction, pad_m)
            if len(points) < 2:
                continue
            length = _path_length(points)
            angle_error = _angle_error_deg(direction, inward)
            score = length
            if angle_error is not None:
                score += max(0.0, 90.0 - angle_error) * 0.01
            candidates.append(
                {
                    "direction": direction,
                    "end": points[-1],
                    "length_m": length,
                    "angle_error_deg": angle_error,
                    "score": score,
                }
            )
    if not candidates:
        return None
    best = max(candidates, key=lambda item: item["score"])
    return best


def _axis_candidate_from_direction(
    *,
    source: str,
    region: Polygon | MultiPolygon,
    origin: tuple[float, float],
    direction: tuple[float, float] | None,
    inward: tuple[float, float] | None,
    terminal_unit: tuple[float, float] | None,
    pad_m: float,
    source_bias: float = 0.0,
) -> dict[str, Any] | None:
    unit = _normalize_vector(direction) if direction is not None else None
    if unit is None:
        return None
    choices: list[dict[str, Any]] = []
    for oriented in (unit, (-unit[0], -unit[1])):
        points = _axis_line_points(region, origin, oriented, pad_m)
        if len(points) < 2:
            continue
        length = _path_length(points)
        portal_score = max(0.0, _dot(oriented, inward)) if inward is not None else 0.0
        terminal_score = max(0.0, _dot(oriented, terminal_unit)) if terminal_unit is not None else 0.0
        choices.append(
            {
                "source": source,
                "direction": oriented,
                "start": origin,
                "end": points[-1],
                "length_m": length,
                "source_bias": float(source_bias),
                "terminal_angle_error_deg": _angle_error_deg(oriented, terminal_unit),
                "_orient_score": length + portal_score * 0.35 + terminal_score * 0.20,
            }
        )
    if not choices:
        return None
    return max(choices, key=lambda item: float(item["_orient_score"]))


def _axis_candidate_key(candidate: dict[str, Any]) -> float:
    direction = candidate.get("direction")
    if not direction:
        return 0.0
    return math.atan2(float(direction[1]), float(direction[0]))


def _dedupe_axis_candidates(candidates: list[dict[str, Any]], max_count: int) -> list[dict[str, Any]]:
    deduped: list[dict[str, Any]] = []
    for candidate in sorted(candidates, key=lambda item: float(item.get("source_bias", 0.0)), reverse=True):
        direction = candidate.get("direction")
        if direction is None:
            continue
        duplicate = False
        for existing in deduped:
            if (_angle_error_deg(direction, existing.get("direction")) or 180.0) <= 3.0:
                duplicate = True
                break
        if duplicate:
            continue
        deduped.append(candidate)
        if len(deduped) >= max_count:
            break
    return deduped


def _axis_candidates(
    *,
    task_type: str,
    task: dict[str, Any],
    region: Polygon | MultiPolygon,
    portal_by_id: dict[str, dict[str, Any]],
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    portal_id, portal = _portal_record(task, portal_by_id)
    portal = portal or {}
    origin = _portal_center(task, portal_by_id, region)
    inward = _portal_inward_unit(portal, region, origin)
    terminal = _terminal_point(task, region, origin)
    terminal_unit = _normalize_vector(_vector_between(origin, terminal))
    pad_m = float(options["service_axis_ray_pad_m"])

    if not bool(options.get("axis_candidate_scoring_enabled", True)):
        heuristic = _service_axis(
            task_type=task_type,
            task=task,
            region=region,
            portal_by_id=portal_by_id,
            options={**options, "axis_candidate_scoring_enabled": False},
        )
        candidate = _axis_candidate_from_direction(
            source=str(heuristic.get("source", "legacy_service_axis")),
            region=region,
            origin=origin,
            direction=heuristic.get("_direction"),
            inward=inward,
            terminal_unit=terminal_unit,
            pad_m=pad_m,
            source_bias=3.0,
        )
        if candidate is None:
            return []
        candidate["portal_id"] = portal_id
        return [candidate]

    seeds: list[tuple[str, tuple[float, float] | None, float]] = []
    if inward is not None:
        seeds.append(("portal_normal_longest_ray", inward, 1.5))
    if terminal_unit is not None:
        terminal_source = "terminal_cap" if task.get("terminal_cap") else "terminal_point"
        seeds.append((terminal_source, terminal_unit, 1.3))
    source_axis = _source_segment_axis(task, region, origin, inward, pad_m)
    if source_axis is not None:
        seeds.append(("source_centerline", source_axis.get("direction"), 2.0))
    dominant = _unit_from_angle_deg(_dominant_angle(region))
    seeds.append(("oriented_bounds", dominant, 1.0))
    farthest = _farthest_region_point(region, origin)
    seeds.append(("farthest_boundary_point", _normalize_vector(_vector_between(origin, farthest)), 0.2))

    sweep = float(options["axis_candidate_sweep_deg"])
    step = float(options["axis_candidate_step_deg"])
    offsets = [0.0]
    if sweep > EPS and step > EPS:
        value = step
        while value <= sweep + EPS:
            offsets.extend([value, -value])
            value += step

    candidates: list[dict[str, Any]] = []
    for source, seed_direction, source_bias in seeds:
        if seed_direction is None:
            continue
        for offset in offsets:
            direction = seed_direction if abs(offset) <= EPS else _rotate_unit(seed_direction, offset)
            label = source if abs(offset) <= EPS else f"{source}_offset_{offset:+.1f}deg"
            candidate = _axis_candidate_from_direction(
                source=label,
                region=region,
                origin=origin,
                direction=direction,
                inward=inward,
                terminal_unit=terminal_unit,
                pad_m=pad_m,
                source_bias=source_bias,
            )
            if candidate is not None:
                candidate["portal_id"] = portal_id
                candidates.append(candidate)
    return _dedupe_axis_candidates(candidates, int(options["axis_candidate_max_count"]))


def _candidate_coverage_percent(
    region: Polygon | MultiPolygon,
    records: list[dict[str, Any]],
    config: dict[str, Any],
) -> tuple[float, float, float]:
    lines = [LineString(record["points"]) for record in records if len(record.get("points", [])) >= 2]
    region_area = v2_geometry.area_m2(region)
    if not lines or region_area <= EPS:
        return 0.0, 0.0, region_area
    covered = unary_union([line.buffer(_tool_width(config) * 0.5, cap_style=1, join_style=1) for line in lines])
    covered_area = v2_geometry.area_m2(covered.intersection(region))
    return (100.0 * covered_area / region_area, covered_area, region_area)


def _candidate_unsafe_count(
    records: list[dict[str, Any]],
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> int:
    count = 0
    for index, record in enumerate(records):
        segment = _make_segment(
            segment_id=f"axis-candidate-{index}",
            phase="axis_candidate_score",
            direction="forward",
            cutting_enabled=True,
            points=record["points"],
            drivable=drivable,
            config=config,
            options=options,
            metadata=None,
        )
        count += len(segment.get("unsafe_samples", []))
    return count


def _axis_candidate_summary(
    task_type: str,
    candidate: dict[str, Any],
    records: list[dict[str, Any]],
    region: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    lengths = [_path_length(record.get("points", [])) for record in records]
    total_length = sum(lengths)
    mean_length = total_length / len(lengths) if lengths else 0.0
    min_length = min(lengths) if lengths else 0.0
    coverage_percent, _covered_area, _region_area = _candidate_coverage_percent(region, records, config)
    unsafe_count = _candidate_unsafe_count(records, drivable, config, options) if records else 0
    region_diag = max(0.1, _region_diagonal(region))
    short_lane_threshold = max(float(options["min_pass_length_m"]), mean_length * 0.45)
    short_lane_count = sum(1 for length in lengths if length < short_lane_threshold)
    source = str(candidate.get("source", "unknown"))
    bonus = 0.0
    if source == "source_centerline":
        bonus = 12.0 if task_type == "dead_end_corridor" else 2.0
    elif source.startswith("source_centerline_offset"):
        bonus = 4.0 if task_type == "dead_end_corridor" else 0.5
    elif source == "body_centerline":
        bonus = 24.0
    elif source.startswith("body_centerline_offset"):
        bonus = 3.0
    elif source.startswith("oriented_bounds"):
        bonus = 3.0
    elif source.startswith("portal_normal_longest_ray"):
        bonus = 5.0 if task_type == "notch" else 2.0
    elif source.startswith("terminal_cap"):
        bonus = 0.8
    elif source.startswith("terminal_point"):
        bonus = 0.6
    if task_type == "body_core":
        lane_term = -len(records) * 0.35
        short_lane_penalty = 0.0
        mean_length_weight = 46.0
        min_length_weight = 8.0
    else:
        lane_term = len(records) * 1.5
        short_lane_penalty = short_lane_count * 10.0
        mean_length_weight = 28.0
        min_length_weight = 12.0
    score = (
        coverage_percent * 1.8
        + (mean_length / region_diag) * mean_length_weight
        + (min_length / region_diag) * min_length_weight
        + lane_term
        + bonus
        - short_lane_penalty
        - unsafe_count * 0.20
    )
    return {
        "source": source,
        "portal_id": candidate.get("portal_id"),
        "start": _round_point(candidate["start"][0], candidate["start"][1]),
        "end": _round_point(candidate["end"][0], candidate["end"][1]),
        "yaw": round(float(_axis_candidate_key(candidate)), 6),
        "length_m": round(float(candidate.get("length_m", 0.0)), 4),
        "terminal_angle_error_deg": (
            round(float(candidate["terminal_angle_error_deg"]), 2)
            if candidate.get("terminal_angle_error_deg") is not None
            else None
        ),
        "score": round(float(score), 4),
        "lane_count": len(records),
        "coverage_percent": round(float(coverage_percent), 2),
        "mean_lane_length_m": round(float(mean_length), 4),
        "min_lane_length_m": round(float(min_length), 4),
        "total_lane_length_m": round(float(total_length), 4),
        "unsafe_sample_count": int(unsafe_count),
        "short_lane_count": int(short_lane_count),
        "status": "candidate",
    }


def _dominant_angle(region: Polygon | MultiPolygon, fallback: float = 0.0) -> float:
    poly = _largest_polygon(region)
    if poly is None:
        return fallback
    return float(v2_geometry.oriented_bounds(poly)["angle_deg"])


def _parallel_pass_lines(
    region: Polygon | MultiPolygon,
    *,
    angle_deg: float,
    spacing_m: float,
    min_length_m: float,
) -> list[LineString]:
    if region.is_empty:
        return []
    rotated = rotate(region, -angle_deg, origin=(0.0, 0.0), use_radians=False)
    minx, miny, maxx, maxy = rotated.bounds
    if maxx - minx <= EPS or maxy - miny <= EPS:
        return []
    span_y = maxy - miny
    if span_y <= spacing_m * 1.15:
        ys = [(miny + maxy) * 0.5]
    else:
        count = max(1, int(math.floor(span_y / spacing_m)) + 1)
        center_y = (miny + maxy) * 0.5
        start_y = center_y - (count - 1) * spacing_m * 0.5
        ys = [start_y + i * spacing_m for i in range(count)]

    pad = max(1.0, spacing_m)
    lines: list[LineString] = []
    for y in ys:
        raw = LineString([(minx - pad, y), (maxx + pad, y)])
        clipped = raw.intersection(rotated)
        pieces = sorted(_iter_lines(clipped), key=lambda line: (line.centroid.x, line.length))
        for piece in pieces:
            if piece.length < min_length_m:
                continue
            lines.append(rotate(piece, angle_deg, origin=(0.0, 0.0), use_radians=False))
    return lines


def _orient_points_near_to_far(
    points: list[tuple[float, float]],
    origin: tuple[float, float],
) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points
    first = math.hypot(points[0][0] - origin[0], points[0][1] - origin[1])
    last = math.hypot(points[-1][0] - origin[0], points[-1][1] - origin[1])
    return points if first <= last else list(reversed(points))


def _make_segment(
    *,
    segment_id: str,
    phase: str,
    direction: str,
    cutting_enabled: bool,
    points: list[tuple[float, float]],
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    sampled = _sample_points(points, float(options["sample_step_m"]))
    yaw = _segment_yaw(sampled, direction)
    offset = _tool_center_offset(config)
    base_poses = [_base_pose_from_tool_point(point, yaw, offset) for point in sampled]
    unsafe_samples = []
    for index, pose in enumerate(base_poses):
        footprint = _footprint_polygon(pose, config)
        if not _covers_with_tolerance(drivable, footprint):
            unsafe_samples.append(
                {
                    "index": index,
                    "x": pose["x"],
                    "y": pose["y"],
                    "yaw": pose["yaw"],
                    "reason": "footprint leaves drivable region or intersects an obstacle",
                }
            )
    length = _path_length(sampled)
    segment = {
        "segment_id": segment_id,
        "phase": phase,
        "direction": direction,
        "cutting_enabled": bool(cutting_enabled),
        "tool_points": _tool_pose_records(sampled, yaw),
        "base_poses": base_poses,
        "line_segments": _line_segments_for_points(sampled),
        "length_m": round(length, 4),
        "unsafe_samples": unsafe_samples,
    }
    if metadata:
        segment.update(metadata)
    return segment


def _make_base_pose_segment(
    *,
    segment_id: str,
    phase: str,
    direction: str,
    cutting_enabled: bool,
    base_poses: list[dict[str, float]],
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    rounded_base_poses = [
        {
            "x": round(float(pose.get("x", 0.0)), 4),
            "y": round(float(pose.get("y", 0.0)), 4),
            "yaw": round(float(_normalize_angle(float(pose.get("yaw", 0.0)))), 6),
        }
        for pose in base_poses
    ]
    offset = _tool_center_offset(config)
    tool_points = [_tool_pose_from_base_pose(pose, offset) for pose in rounded_base_poses]
    unsafe_samples = []
    for index, pose in enumerate(rounded_base_poses):
        footprint = _footprint_polygon(pose, config)
        if not _covers_with_tolerance(drivable, footprint):
            unsafe_samples.append(
                {
                    "index": index,
                    "x": pose["x"],
                    "y": pose["y"],
                    "yaw": pose["yaw"],
                    "reason": "connector footprint leaves drivable region or intersects an obstacle",
                }
            )
    base_points = [(float(pose["x"]), float(pose["y"])) for pose in rounded_base_poses]
    tool_path_points = [(float(point["x"]), float(point["y"])) for point in tool_points]
    right_side_midpoints = [_footprint_right_side_midpoint(pose, config) for pose in rounded_base_poses]
    sample_stride = max(1, len(rounded_base_poses) // 80)
    right_side_sample_segments = []
    for pose in rounded_base_poses[::sample_stride]:
        rear, front = _footprint_right_corner_points(pose, config)
        right_side_sample_segments.append([_round_point(rear[0], rear[1]), _round_point(front[0], front[1])])
    segment = {
        "segment_id": segment_id,
        "phase": phase,
        "direction": direction,
        "cutting_enabled": bool(cutting_enabled),
        "tool_points": tool_points,
        "base_poses": rounded_base_poses,
        "line_segments": _line_segments_for_points(tool_path_points),
        "base_line_segments": _line_segments_for_points(base_points),
        "right_footprint_side_points": [
            {"x": round(point[0], 4), "y": round(point[1], 4)}
            for point in right_side_midpoints
        ],
        "right_footprint_side_sample_segments": right_side_sample_segments,
        "length_m": round(_path_length(base_points), 4),
        "unsafe_samples": unsafe_samples,
    }
    if metadata:
        segment.update(metadata)
    return segment


def _cubic_bezier_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    u = 1.0 - t
    b0 = u * u * u
    b1 = 3.0 * u * u * t
    b2 = 3.0 * u * t * t
    b3 = t * t * t
    return (
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
    )


def _cubic_bezier_derivative(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    u = 1.0 - t
    return (
        3.0 * u * u * (p1[0] - p0[0])
        + 6.0 * u * t * (p2[0] - p1[0])
        + 3.0 * t * t * (p3[0] - p2[0]),
        3.0 * u * u * (p1[1] - p0[1])
        + 6.0 * u * t * (p2[1] - p1[1])
        + 3.0 * t * t * (p3[1] - p2[1]),
    )


def _compact_connector_gap_metrics(
    start_pose: dict[str, float],
    end_pose: dict[str, float],
) -> dict[str, float]:
    sx = float(start_pose.get("x", 0.0))
    sy = float(start_pose.get("y", 0.0))
    ex = float(end_pose.get("x", 0.0))
    ey = float(end_pose.get("y", 0.0))
    start_yaw = float(start_pose.get("yaw", 0.0))
    end_yaw = float(end_pose.get("yaw", 0.0))
    heading = (math.cos(start_yaw), math.sin(start_yaw))
    lateral = (-heading[1], heading[0])
    delta = (ex - sx, ey - sy)
    return {
        "distance_m": math.hypot(delta[0], delta[1]),
        "longitudinal_gap_m": _dot(delta, heading),
        "lateral_gap_m": _dot(delta, lateral),
        "heading_delta_deg": abs(math.degrees(_normalize_angle(end_yaw - start_yaw))),
    }


def _mod2pi(angle: float) -> float:
    return angle - 2.0 * math.pi * math.floor(angle / (2.0 * math.pi))


def _dubins_lsl(alpha: float, beta: float, distance: float) -> tuple[float, float, float] | None:
    tmp0 = distance + math.sin(alpha) - math.sin(beta)
    p_squared = (
        2.0
        + distance * distance
        - 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (math.sin(alpha) - math.sin(beta))
    )
    if p_squared < -EPS:
        return None
    tmp1 = math.atan2(math.cos(beta) - math.cos(alpha), tmp0)
    return (_mod2pi(-alpha + tmp1), math.sqrt(max(0.0, p_squared)), _mod2pi(beta - tmp1))


def _dubins_rsr(alpha: float, beta: float, distance: float) -> tuple[float, float, float] | None:
    tmp0 = distance - math.sin(alpha) + math.sin(beta)
    p_squared = (
        2.0
        + distance * distance
        - 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (-math.sin(alpha) + math.sin(beta))
    )
    if p_squared < -EPS:
        return None
    tmp1 = math.atan2(math.cos(alpha) - math.cos(beta), tmp0)
    return (_mod2pi(alpha - tmp1), math.sqrt(max(0.0, p_squared)), _mod2pi(-beta + tmp1))


def _dubins_lsr(alpha: float, beta: float, distance: float) -> tuple[float, float, float] | None:
    p_squared = (
        -2.0
        + distance * distance
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (math.sin(alpha) + math.sin(beta))
    )
    if p_squared < -EPS:
        return None
    p = math.sqrt(max(0.0, p_squared))
    tmp0 = math.atan2(-math.cos(alpha) - math.cos(beta), distance + math.sin(alpha) + math.sin(beta))
    tmp1 = math.atan2(-2.0, p)
    t = _mod2pi(-alpha + tmp0 - tmp1)
    q = _mod2pi(-_mod2pi(beta) + tmp0 - tmp1)
    return (t, p, q)


def _dubins_rsl(alpha: float, beta: float, distance: float) -> tuple[float, float, float] | None:
    p_squared = (
        distance * distance
        - 2.0
        + 2.0 * math.cos(alpha - beta)
        - 2.0 * distance * (math.sin(alpha) + math.sin(beta))
    )
    if p_squared < -EPS:
        return None
    p = math.sqrt(max(0.0, p_squared))
    tmp0 = math.atan2(math.cos(alpha) + math.cos(beta), distance - math.sin(alpha) - math.sin(beta))
    tmp1 = math.atan2(2.0, p)
    t = _mod2pi(alpha - tmp0 + tmp1)
    q = _mod2pi(beta - tmp0 + tmp1)
    return (t, p, q)


def _dubins_rlr(alpha: float, beta: float, distance: float) -> tuple[float, float, float] | None:
    tmp0 = (
        6.0
        - distance * distance
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (math.sin(alpha) - math.sin(beta))
    ) / 8.0
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp0))))
    t = _mod2pi(
        alpha
        - math.atan2(math.cos(alpha) - math.cos(beta), distance - math.sin(alpha) + math.sin(beta))
        + p * 0.5
    )
    q = _mod2pi(alpha - beta - t + p)
    return (t, p, q)


def _dubins_lrl(alpha: float, beta: float, distance: float) -> tuple[float, float, float] | None:
    tmp0 = (
        6.0
        - distance * distance
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * distance * (-math.sin(alpha) + math.sin(beta))
    ) / 8.0
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp0))))
    t = _mod2pi(
        -alpha
        - math.atan2(math.cos(alpha) - math.cos(beta), distance + math.sin(alpha) - math.sin(beta))
        + p * 0.5
    )
    q = _mod2pi(beta - alpha - t + p)
    return (t, p, q)


def _dubins_candidates(
    end_x: float,
    end_y: float,
    end_yaw: float,
    radius: float,
) -> list[dict[str, Any]]:
    distance = math.hypot(end_x, end_y) / radius
    theta = _mod2pi(math.atan2(end_y, end_x))
    alpha = _mod2pi(-theta)
    beta = _mod2pi(end_yaw - theta)
    planners = [
        ("LSL", ("L", "S", "L"), _dubins_lsl),
        ("RSR", ("R", "S", "R"), _dubins_rsr),
        ("LSR", ("L", "S", "R"), _dubins_lsr),
        ("RSL", ("R", "S", "L"), _dubins_rsl),
        ("RLR", ("R", "L", "R"), _dubins_rlr),
        ("LRL", ("L", "R", "L"), _dubins_lrl),
    ]
    candidates = []
    for name, modes, planner in planners:
        lengths = planner(alpha, beta, distance)
        if lengths is None:
            continue
        candidates.append(
            {
                "name": name,
                "modes": modes,
                "lengths": lengths,
                "length_m": radius * sum(lengths),
            }
        )
    return candidates


def _sample_dubins_candidate(
    candidate: dict[str, Any],
    radius: float,
    sample_step: float,
) -> list[tuple[float, float, float]]:
    poses: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]
    x = y = yaw = 0.0

    def append_pose(nx: float, ny: float, nyaw: float) -> None:
        if poses and math.hypot(nx - poses[-1][0], ny - poses[-1][1]) <= EPS:
            poses[-1] = (nx, ny, _normalize_angle(nyaw))
        else:
            poses.append((nx, ny, _normalize_angle(nyaw)))

    for mode, length in zip(candidate["modes"], candidate["lengths"]):
        if length <= EPS:
            continue
        segment_length_m = length * radius if mode != "S" else length * radius
        count = max(1, int(math.ceil(segment_length_m / sample_step)))
        for i in range(1, count + 1):
            step = length * i / count
            if mode == "S":
                nx = x + radius * step * math.cos(yaw)
                ny = y + radius * step * math.sin(yaw)
                nyaw = yaw
            elif mode == "L":
                nyaw = yaw + step
                cx = x - radius * math.sin(yaw)
                cy = y + radius * math.cos(yaw)
                nx = cx + radius * math.sin(nyaw)
                ny = cy - radius * math.cos(nyaw)
            else:
                nyaw = yaw - step
                cx = x + radius * math.sin(yaw)
                cy = y - radius * math.cos(yaw)
                nx = cx - radius * math.sin(nyaw)
                ny = cy + radius * math.cos(nyaw)
            append_pose(nx, ny, nyaw)
        x, y, yaw = poses[-1]
    return poses


def _straight_connector_base_poses(
    start_pose: dict[str, float],
    end_pose: dict[str, float],
    metrics: dict[str, float],
    options: dict[str, Any],
) -> list[dict[str, float]]:
    sample_step = float(options["compact_connector_sample_step_m"])
    distance = float(metrics["distance_m"])
    count = max(1, int(math.ceil(distance / sample_step))) if distance > EPS else 1
    start_yaw = float(start_pose.get("yaw", 0.0))
    end_yaw = float(end_pose.get("yaw", start_yaw))
    yaw_delta = _normalize_angle(end_yaw - start_yaw)
    poses = []
    for i in range(count + 1):
        t = i / count
        poses.append(
            {
                "x": float(start_pose.get("x", 0.0))
                + (float(end_pose.get("x", 0.0)) - float(start_pose.get("x", 0.0))) * t,
                "y": float(start_pose.get("y", 0.0))
                + (float(end_pose.get("y", 0.0)) - float(start_pose.get("y", 0.0))) * t,
                "yaw": _normalize_angle(start_yaw + yaw_delta * t),
            }
        )
    poses[0]["yaw"] = start_yaw
    poses[-1]["x"] = float(end_pose.get("x", poses[-1]["x"]))
    poses[-1]["y"] = float(end_pose.get("y", poses[-1]["y"]))
    poses[-1]["yaw"] = end_yaw
    return poses


def _compact_keyhole_base_poses(
    start_pose: dict[str, float],
    end_pose: dict[str, float],
    metrics: dict[str, float],
    options: dict[str, Any],
) -> list[dict[str, float]]:
    start_yaw = float(start_pose.get("yaw", 0.0))
    end_yaw = float(end_pose.get("yaw", 0.0))
    sample_step = float(options["compact_connector_sample_step_m"])
    longitudinal = float(metrics["longitudinal_gap_m"])
    lateral = float(metrics["lateral_gap_m"])
    end_delta_yaw = _normalize_angle(end_yaw - start_yaw)
    if abs(lateral) <= EPS:
        return [
            {
                "x": float(start_pose.get("x", 0.0)),
                "y": float(start_pose.get("y", 0.0)),
                "yaw": start_yaw,
            },
            {
                "x": float(end_pose.get("x", 0.0)),
                "y": float(end_pose.get("y", 0.0)),
                "yaw": end_yaw,
            },
        ]
    radius = float(options["compact_connector_turn_radius_m"])
    candidates = _dubins_candidates(longitudinal, lateral, end_delta_yaw, radius)
    sampled_candidates = []
    for candidate in candidates:
        local = _sample_dubins_candidate(candidate, radius, sample_step)
        if not local:
            continue
        max_forward = max(point[0] for point in local)
        min_forward = min(point[0] for point in local)
        max_lateral = max(point[1] for point in local)
        min_lateral = min(point[1] for point in local)
        is_bulb = candidate["name"] in {"RLR", "LRL"}
        sampled_candidates.append(
            {
                "candidate": candidate,
                "poses": local,
                "max_forward": max_forward,
                "min_forward": min_forward,
                "max_lateral": max_lateral,
                "min_lateral": min_lateral,
                "score": (
                    (0 if is_bulb else 1000.0)
                    + max(0.0, -min_forward) * 25.0
                    - max_forward * 0.05
                    + float(candidate["length_m"])
                ),
            }
        )
    if not sampled_candidates:
        return [
            {
                "x": float(start_pose.get("x", 0.0)),
                "y": float(start_pose.get("y", 0.0)),
                "yaw": start_yaw,
            },
            {
                "x": float(end_pose.get("x", 0.0)),
                "y": float(end_pose.get("y", 0.0)),
                "yaw": end_yaw,
            },
        ]
    best = min(sampled_candidates, key=lambda item: float(item["score"]))
    h0 = (math.cos(start_yaw), math.sin(start_yaw))
    left = (-h0[1], h0[0])

    def world_pose(local_x: float, local_y: float, local_yaw: float) -> dict[str, float]:
        sx = float(start_pose.get("x", 0.0))
        sy = float(start_pose.get("y", 0.0))
        return {
            "x": sx + h0[0] * local_x + left[0] * local_y,
            "y": sy + h0[1] * local_x + left[1] * local_y,
            "yaw": _normalize_angle(start_yaw + local_yaw),
        }

    poses: list[dict[str, float]] = []
    local_poses = list(best["poses"])
    for index, (x, y, yaw) in enumerate(local_poses):
        pose = world_pose(x, y, yaw)
        if index == 0:
            pose["yaw"] = start_yaw
        elif index == len(local_poses) - 1:
            pose["x"] = float(end_pose.get("x", pose["x"]))
            pose["y"] = float(end_pose.get("y", pose["y"]))
            pose["yaw"] = end_yaw
        poses.append(pose)
    return poses


def _try_compact_keyhole_connector(
    *,
    segment_id: str,
    from_segment: dict[str, Any],
    to_segment: dict[str, Any],
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    from_poses = from_segment.get("base_poses", [])
    to_poses = to_segment.get("base_poses", [])
    if not from_poses or not to_poses:
        return None, {"status": "not_candidate", "reason": "missing_endpoint_pose"}
    start_pose = from_poses[-1]
    end_pose = to_poses[0]
    metrics = _compact_connector_gap_metrics(start_pose, end_pose)
    record: dict[str, Any] = {
        "from_segment_id": from_segment.get("segment_id"),
        "to_segment_id": to_segment.get("segment_id"),
        "connector_style": str(options.get("adjacent_connector_style", "straight")),
        "start_pose": {
            "x": round(float(start_pose.get("x", 0.0)), 4),
            "y": round(float(start_pose.get("y", 0.0)), 4),
            "yaw": round(float(start_pose.get("yaw", 0.0)), 6),
        },
        "end_pose": {
            "x": round(float(end_pose.get("x", 0.0)), 4),
            "y": round(float(end_pose.get("y", 0.0)), 4),
            "yaw": round(float(end_pose.get("yaw", 0.0)), 6),
        },
        "diagnostic_line_segments": [
            [
                _round_point(float(start_pose.get("x", 0.0)), float(start_pose.get("y", 0.0))),
                _round_point(float(end_pose.get("x", 0.0)), float(end_pose.get("y", 0.0))),
            ]
        ],
        "gap_distance_m": round(float(metrics["distance_m"]), 4),
        "longitudinal_gap_m": round(float(metrics["longitudinal_gap_m"]), 4),
        "lateral_gap_m": round(float(metrics["lateral_gap_m"]), 4),
        "heading_delta_deg": round(float(metrics["heading_delta_deg"]), 2),
    }
    connector_style = str(options.get("adjacent_connector_style", "straight"))
    max_distance = (
        float(options["straight_connector_max_distance_m"])
        if connector_style == "straight"
        else float(options["compact_connector_max_distance_m"])
    )
    if metrics["distance_m"] > max_distance:
        record.update({"status": "rejected", "reason": f"gap_too_far_for_{connector_style}_connector"})
        return None, record
    if metrics["distance_m"] <= EPS:
        record.update({"status": "rejected", "reason": "zero_length_gap"})
        return None, record
    min_heading_delta = (
        float(options["straight_connector_min_heading_delta_deg"])
        if connector_style == "straight"
        else float(options["compact_connector_min_heading_delta_deg"])
    )
    if metrics["heading_delta_deg"] < min_heading_delta:
        record.update({"status": "rejected", "reason": "heading_change_too_small"})
        return None, record
    if connector_style == "straight":
        base_poses = _straight_connector_base_poses(start_pose, end_pose, metrics, options)
        phase = "straight_connector"
        connector_kind = "adjacent_straight_line"
        turn_style = "straight_line_yaw_blend"
        metadata_extra = {}
    else:
        base_poses = _compact_keyhole_base_poses(start_pose, end_pose, metrics, options)
        phase = "compact_keyhole_connector"
        connector_kind = "adjacent_compact_keyhole"
        turn_style = "compact_keyhole_dubins_bulb"
        metadata_extra = {"turn_radius_m": round(float(options["compact_connector_turn_radius_m"]), 4)}
    connector = _make_base_pose_segment(
        segment_id=segment_id,
        phase=phase,
        direction="forward",
        cutting_enabled=False,
        base_poses=base_poses,
        drivable=drivable,
        config=config,
        metadata={
            "connector_kind": connector_kind,
            "turn_style": turn_style,
            "connects_from_segment_id": from_segment.get("segment_id"),
            "connects_to_segment_id": to_segment.get("segment_id"),
            "gap_distance_m": record["gap_distance_m"],
            "longitudinal_gap_m": record["longitudinal_gap_m"],
            "lateral_gap_m": record["lateral_gap_m"],
            "heading_delta_deg": record["heading_delta_deg"],
            "blade_state": "off",
            "mower_consumption_hint": "follow_base_link_poses",
            **metadata_extra,
        },
    )
    if connector.get("unsafe_samples"):
        unsafe_count = len(connector.get("unsafe_samples", []))
        diagnostic_segment = {
            "segment_id": connector.get("segment_id"),
            "phase": connector.get("phase"),
            "direction": connector.get("direction"),
            "turn_style": connector.get("turn_style"),
            "tool_points": connector.get("tool_points", []),
            "base_poses": connector.get("base_poses", []),
            "line_segments": connector.get("line_segments", []),
            "base_line_segments": connector.get("base_line_segments", []),
            "length_m": connector.get("length_m", 0.0),
            "unsafe_samples": connector.get("unsafe_samples", []),
        }
        if str(options.get("connector_unsafe_policy", "reject")) != "include":
            record.update(
                {
                    "status": "rejected",
                    "reason": "connector_footprint_unsafe",
                    "unsafe_sample_count": unsafe_count,
                    "diagnostic_segment": diagnostic_segment,
                }
            )
            return None, record
        connector["unsafe_connector_policy"] = "included_for_temporary_consumption_test"
        record.update(
            {
                "status": "inserted_with_unsafe",
                "reason": "included_unsafe_for_temporary_consumption_test",
                "unsafe_sample_count": unsafe_count,
                "diagnostic_segment": diagnostic_segment,
                "segment_id": connector["segment_id"],
                "length_m": connector["length_m"],
                "pose_count": len(connector.get("base_poses", [])),
            }
        )
        return connector, record
    record.update(
        {
            "status": "inserted",
            "reason": "ok",
            "segment_id": connector["segment_id"],
            "length_m": connector["length_m"],
            "pose_count": len(connector.get("base_poses", [])),
        }
    )
    return connector, record


def _insert_compact_keyhole_connectors(
    *,
    task_id: str,
    segments: list[dict[str, Any]],
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    summary: dict[str, Any] = {
        "enabled": bool(options.get("compact_connectors_enabled", True)),
        "connector_style": str(options.get("adjacent_connector_style", "straight")),
        "candidate_count": 0,
        "inserted_count": 0,
        "rejected_count": 0,
        "unsafe_rejected_count": 0,
        "too_far_rejected_count": 0,
        "unsafe_inserted_count": 0,
        "unsafe_inserted_sample_count": 0,
        "length_m": 0.0,
        "records": [],
    }
    if not summary["enabled"] or len(segments) < 2:
        return segments, summary

    def eligible(segment: dict[str, Any]) -> bool:
        return (
            not (str(segment.get("phase", "")).endswith("_connector") or bool(segment.get("connector_kind")))
            and segment.get("direction") == "forward"
            and bool(segment.get("cutting_enabled"))
            and len(segment.get("base_poses", []) or []) >= 2
        )

    out: list[dict[str, Any]] = []
    for index, segment in enumerate(segments):
        out.append(segment)
        if index >= len(segments) - 1:
            continue
        next_segment = segments[index + 1]
        if not eligible(segment) or not eligible(next_segment):
            continue
        summary["candidate_count"] += 1
        connector, record = _try_compact_keyhole_connector(
            segment_id=f"{task_id}-connector-{summary['candidate_count'] - 1}",
            from_segment=segment,
            to_segment=next_segment,
            drivable=drivable,
            config=config,
            options=options,
        )
        summary["records"].append(record)
        if connector is None:
            summary["rejected_count"] += 1
            if record.get("reason") == "connector_footprint_unsafe":
                summary["unsafe_rejected_count"] += 1
            elif str(record.get("reason", "")).startswith("gap_too_far_for_"):
                summary["too_far_rejected_count"] += 1
            continue
        out.append(connector)
        summary["inserted_count"] += 1
        summary["length_m"] += float(connector.get("length_m", 0.0))
        if str(record.get("status", "")) == "inserted_with_unsafe":
            summary["unsafe_inserted_count"] += 1
            summary["unsafe_inserted_sample_count"] += int(record.get("unsafe_sample_count", 0))

    summary["length_m"] = round(float(summary["length_m"]), 4)
    return out, summary


def _sample_polyline_tool_poses(
    points: list[tuple[float, float]],
    sample_step: float,
    direction: str,
    yaw_window_m: float = 0.0,
) -> list[dict[str, float]]:
    if not points:
        return []
    if len(points) == 1:
        return [{"x": round(points[0][0], 4), "y": round(points[0][1], 4), "yaw": 0.0}]
    poses: list[dict[str, float]] = []
    vertex_distances = [0.0]
    for start, end in zip(points, points[1:]):
        vertex_distances.append(vertex_distances[-1] + math.hypot(end[0] - start[0], end[1] - start[1]))
    total_length = vertex_distances[-1]
    closed = len(points) > 2 and math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1]) <= EPS
    step = sample_step if sample_step > EPS else float("inf")
    for segment_index, (start, end) in enumerate(zip(points, points[1:])):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        if length <= EPS:
            continue
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        if direction == "backward":
            yaw = _normalize_angle(yaw + math.pi)
        count = max(1, int(math.ceil(length / step))) if math.isfinite(step) else 1
        start_i = 0 if segment_index == 0 or not poses else 1
        for i in range(start_i, count + 1):
            t = i / count
            poses.append(
                {
                    "x": round(float(start[0] + (end[0] - start[0]) * t), 4),
                    "y": round(float(start[1] + (end[1] - start[1]) * t), 4),
                    "s": float(vertex_distances[segment_index] + length * t),
                    "yaw": round(float(yaw), 6),
                }
            )
    if not poses:
        poses.append({"x": round(points[0][0], 4), "y": round(points[0][1], 4), "s": 0.0, "yaw": 0.0})

    yaw_window = min(max(0.0, float(yaw_window_m)), total_length * 0.5 if total_length > EPS else 0.0)
    if yaw_window > EPS and total_length > EPS:

        def point_at(distance_m: float) -> tuple[float, float]:
            if closed:
                distance_m = math.fmod(distance_m, total_length)
                if distance_m < 0.0:
                    distance_m += total_length
            else:
                distance_m = min(max(distance_m, 0.0), total_length)
            if distance_m <= 0.0:
                return points[0]
            if distance_m >= total_length:
                return points[-1]
            point_index = bisect.bisect_right(vertex_distances, distance_m) - 1
            point_index = min(max(point_index, 0), len(points) - 2)
            segment_length = vertex_distances[point_index + 1] - vertex_distances[point_index]
            if segment_length <= EPS:
                return points[point_index]
            t = (distance_m - vertex_distances[point_index]) / segment_length
            start = points[point_index]
            end = points[point_index + 1]
            return (float(start[0] + (end[0] - start[0]) * t), float(start[1] + (end[1] - start[1]) * t))

        for pose in poses:
            sample_s = float(pose["s"])
            before = point_at(sample_s - yaw_window * 0.5)
            after = point_at(sample_s + yaw_window * 0.5)
            if math.hypot(after[0] - before[0], after[1] - before[1]) > EPS:
                yaw = math.atan2(after[1] - before[1], after[0] - before[0])
                if direction == "backward":
                    yaw = _normalize_angle(yaw + math.pi)
                pose["yaw"] = round(float(yaw), 6)

    return [{"x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose["yaw"])} for pose in poses]


def _make_polyline_segment(
    *,
    segment_id: str,
    phase: str,
    direction: str,
    cutting_enabled: bool,
    points: list[tuple[float, float]],
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
    sample_step_m: float | None = None,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    sample_step = float(sample_step_m if sample_step_m is not None else options["sample_step_m"])
    tool_points = _sample_polyline_tool_poses(
        points,
        sample_step,
        direction,
        yaw_window_m=float(options.get("outline_yaw_window_m", 0.0)),
    )
    offset = _tool_center_offset(config)
    base_poses = [
        _base_pose_from_tool_point((float(point["x"]), float(point["y"])), float(point["yaw"]), offset)
        for point in tool_points
    ]
    unsafe_samples = []
    for index, pose in enumerate(base_poses):
        footprint = _footprint_polygon(pose, config)
        if not _covers_with_tolerance(safety_region, footprint):
            unsafe_samples.append(
                {
                    "index": index,
                    "x": pose["x"],
                    "y": pose["y"],
                    "yaw": pose["yaw"],
                    "reason": "outline footprint leaves conditioned lawn or intersects an obstacle",
                }
            )
    sampled_points = [(float(point["x"]), float(point["y"])) for point in tool_points]
    length = _path_length(sampled_points)
    segment = {
        "segment_id": segment_id,
        "phase": phase,
        "direction": direction,
        "cutting_enabled": bool(cutting_enabled),
        "tool_points": tool_points,
        "base_poses": base_poses,
        "line_segments": _line_segments_for_points(sampled_points),
        "length_m": round(length, 4),
        "unsafe_samples": unsafe_samples,
    }
    if metadata:
        segment.update(metadata)
    return segment


def _make_turnaround_segment(
    *,
    segment_id: str,
    point: tuple[float, float],
    start_yaw: float,
    coverage_region: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    step = math.radians(float(options["turnaround_yaw_step_deg"]))
    yaw_values = []
    yaw_delta = 0.0
    while yaw_delta < math.pi:
        yaw_values.append(_normalize_angle(start_yaw + yaw_delta))
        yaw_delta += step
    yaw_values.append(_normalize_angle(start_yaw + math.pi))
    offset = _tool_center_offset(config)
    tool_points = [{"x": round(point[0], 4), "y": round(point[1], 4), "yaw": round(yaw, 6)} for yaw in yaw_values]
    base_poses = [_base_pose_from_tool_point(point, yaw, offset) for yaw in yaw_values]
    unsafe_samples = []
    safe_region = coverage_region
    clearance = float(options["turnaround_clearance_m"])
    if clearance > EPS:
        safe_region = coverage_region.buffer(-clearance, join_style=2)
    for index, pose in enumerate(base_poses):
        footprint = _footprint_polygon(pose, config)
        if not _covers_with_tolerance(safe_region, footprint) or not _covers_with_tolerance(drivable, footprint):
            unsafe_samples.append(
                {
                    "index": index,
                    "x": pose["x"],
                    "y": pose["y"],
                    "yaw": pose["yaw"],
                    "reason": "turnaround footprint leaves service region or drivable region",
                }
            )
    segment = {
        "segment_id": segment_id,
        "phase": "terminal_turnaround",
        "direction": "rotate",
        "cutting_enabled": False,
        "tool_points": tool_points,
        "base_poses": base_poses,
        "line_segments": [],
        "length_m": 0.0,
        "unsafe_samples": unsafe_samples,
    }
    if metadata:
        segment.update(metadata)
    return segment


def _turnaround_diagnostic(
    *,
    task_type: str,
    pass_records: list[dict[str, Any]],
    coverage_region: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    if task_type != "dead_end_corridor" or not bool(options["dead_end_turnaround_enabled"]) or not pass_records:
        return {"feasible": False, "reason": "turnaround not requested for this task type"}
    center_record = max(pass_records, key=lambda record: float(_path_length(record.get("points", []))))
    points = center_record.get("points", [])
    if len(points) < 2:
        return {"feasible": False, "reason": "no center lane available for terminal turnaround"}
    terminal = points[-1]
    start = points[0]
    yaw = math.atan2(terminal[1] - start[1], terminal[0] - start[0])
    terminal_band = Point(*terminal).buffer(float(options["turnaround_terminal_band_m"]))
    if coverage_region.intersection(terminal_band).is_empty:
        return {"feasible": False, "reason": "terminal band does not overlap service region"}
    trial = _make_turnaround_segment(
        segment_id="turnaround-trial",
        point=terminal,
        start_yaw=yaw,
        coverage_region=coverage_region,
        drivable=drivable,
        config=config,
        options=options,
        metadata=None,
    )
    footprints = [_footprint_polygon(pose, config) for pose in trial.get("base_poses", [])]
    envelope = v2_geometry._clean_polygonal(unary_union(footprints)) if footprints else MultiPolygon([])
    unsafe_count = len(trial.get("unsafe_samples", []))
    return {
        "feasible": unsafe_count == 0,
        "reason": "terminal 180-degree footprint sweep is service-region safe"
        if unsafe_count == 0
        else "terminal 180-degree footprint sweep is not service-region safe",
        "terminal_point": _round_point(terminal[0], terminal[1]),
        "yaw": round(float(yaw), 6),
        "sample_count": len(trial.get("base_poses", [])),
        "unsafe_sample_count": unsafe_count,
        "envelope": v2_geometry.geometry_to_json(envelope),
    }


def _coverage_percent(
    coverage_region: Polygon | MultiPolygon,
    segments: list[dict[str, Any]],
    config: dict[str, Any],
) -> tuple[float, float, float]:
    region_area = v2_geometry.area_m2(coverage_region)
    covered = _cutting_coverage_geometry(coverage_region, segments, config)
    covered_area = v2_geometry.area_m2(covered)
    if region_area <= EPS:
        return 0.0, 0.0, region_area
    return (100.0 * covered_area / region_area, covered_area, region_area)


def _cutting_coverage_geometry(
    coverage_region: Polygon | MultiPolygon,
    segments: list[dict[str, Any]],
    config: dict[str, Any],
) -> Polygon | MultiPolygon:
    lines = []
    for segment in segments:
        if not segment.get("cutting_enabled"):
            continue
        points = [(float(p["x"]), float(p["y"])) for p in segment.get("tool_points", [])]
        if len(points) >= 2:
            lines.append(LineString(points))
    if not lines or coverage_region.is_empty:
        return MultiPolygon([])
    covered = unary_union([line.buffer(_tool_width(config) * 0.5, cap_style=1, join_style=1) for line in lines])
    return v2_geometry._clean_polygonal(covered.intersection(coverage_region))


def _path_summary(segments: list[dict[str, Any]]) -> dict[str, Any]:
    forward_length = sum(float(s.get("length_m", 0.0)) for s in segments if s.get("direction") == "forward")
    reverse_length = sum(float(s.get("length_m", 0.0)) for s in segments if s.get("direction") == "backward")
    cutting_length = sum(float(s.get("length_m", 0.0)) for s in segments if s.get("cutting_enabled"))
    connector_length = sum(
        float(s.get("length_m", 0.0))
        for s in segments
        if str(s.get("phase", "")).endswith("_connector") or bool(s.get("connector_kind"))
    )
    connector_count = sum(
        1 for s in segments if str(s.get("phase", "")).endswith("_connector") or bool(s.get("connector_kind"))
    )
    return {
        "path_length_m": round(forward_length + reverse_length, 4),
        "forward_length_m": round(forward_length, 4),
        "reverse_length_m": round(reverse_length, 4),
        "cutting_length_m": round(cutting_length, 4),
        "connector_count": int(connector_count),
        "connector_length_m": round(connector_length, 4),
        "unsafe_sample_count": sum(len(s.get("unsafe_samples", [])) for s in segments),
    }


def _direct_portal_stroke(
    region: Polygon | MultiPolygon,
    portal: tuple[float, float],
    terminal: tuple[float, float],
    min_length_m: float,
) -> list[list[tuple[float, float]]]:
    line = LineString([portal, terminal]).intersection(region)
    pieces = sorted(_iter_lines(line), key=lambda item: item.length, reverse=True)
    if not pieces or pieces[0].length < min_length_m:
        return []
    return [_orient_points_near_to_far(_line_points(pieces[0]), portal)]


def _line_pass_points(
    lines: list[LineString],
    *,
    task_type: str,
    portal: tuple[float, float] | None = None,
) -> list[list[tuple[float, float]]]:
    passes = []
    for index, line in enumerate(lines):
        points = _line_points(line)
        if task_type in {"dead_end_corridor", "notch"} and portal is not None:
            points = _orient_points_near_to_far(points, portal)
        elif index % 2 == 1:
            points = list(reversed(points))
        passes.append(points)
    return passes


def _undirected_angle_error_deg(
    a: tuple[float, float] | None,
    b: tuple[float, float] | None,
) -> float | None:
    error = _angle_error_deg(a, b)
    if error is None:
        return None
    return min(error, abs(180.0 - error))


def _body_centerline_direction(
    area: dict[str, Any],
    region: Polygon | MultiPolygon,
) -> tuple[float, float] | None:
    raw_segments: list[Any] = []
    for branch in area.get("skeleton_graph", {}).get("branches", []):
        raw_segments.extend(branch.get("line_segments", []))
    if not raw_segments:
        for link in area.get("ridge_links", []):
            raw_segments.append(link.get("line", []))

    probe_region = region.buffer(0.08, join_style=2)
    sum_cos = 0.0
    sum_sin = 0.0
    total_weight = 0.0
    for segment in raw_segments:
        if not isinstance(segment, list) or len(segment) < 2:
            continue
        start = _point_from_record(segment[0])
        end = _point_from_record(segment[-1])
        if start is None or end is None:
            continue
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= EPS:
            continue
        line = LineString([start, end])
        overlap = line.intersection(probe_region)
        overlap_length = sum(piece.length for piece in _iter_lines(overlap))
        if overlap_length < max(0.05, length * 0.35):
            continue
        theta = math.atan2(dy, dx)
        weight = float(overlap_length)
        sum_cos += math.cos(2.0 * theta) * weight
        sum_sin += math.sin(2.0 * theta) * weight
        total_weight += weight
    if total_weight <= EPS or math.hypot(sum_cos, sum_sin) <= EPS:
        return None
    angle = 0.5 * math.atan2(sum_sin, sum_cos)
    return (math.cos(angle), math.sin(angle))


def _axis_candidate_for_body(
    source: str,
    region: Polygon | MultiPolygon,
    direction: tuple[float, float],
    pad_m: float,
    source_bias: float,
) -> dict[str, Any] | None:
    origin_point = region.representative_point()
    origin = (float(origin_point.x), float(origin_point.y))
    unit = _normalize_vector(direction)
    if unit is None:
        return None
    points = _axis_line_points(region, origin, unit, pad_m)
    if len(points) >= 2:
        start = points[0]
        end = points[-1]
        length = _path_length(points)
    else:
        span = max(0.0, _region_diagonal(region))
        start = _point_along(origin, unit, -span * 0.5)
        end = _point_along(origin, unit, span * 0.5)
        length = span
    return {
        "source": source,
        "direction": unit,
        "start": start,
        "end": end,
        "length_m": length,
        "source_bias": float(source_bias),
        "terminal_angle_error_deg": None,
        "portal_id": None,
    }


def _body_axis_candidates(
    area: dict[str, Any],
    region: Polygon | MultiPolygon,
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    seeds: list[tuple[str, tuple[float, float] | None, float]] = [
        ("oriented_bounds", _unit_from_angle_deg(_dominant_angle(region)), 1.0),
        ("body_centerline", _body_centerline_direction(area, region), 3.0),
    ]
    sweep = float(options["axis_candidate_sweep_deg"])
    step = float(options["axis_candidate_step_deg"])
    offsets = [0.0]
    if sweep > EPS and step > EPS:
        value = step
        while value <= sweep + EPS:
            offsets.extend([value, -value])
            value += step

    candidates: list[dict[str, Any]] = []
    pad_m = float(options["service_axis_ray_pad_m"])
    for source, seed_direction, source_bias in seeds:
        if seed_direction is None:
            continue
        for offset in offsets:
            direction = seed_direction if abs(offset) <= EPS else _rotate_unit(seed_direction, offset)
            label = source if abs(offset) <= EPS else f"{source}_offset_{offset:+.1f}deg"
            candidate = _axis_candidate_for_body(label, region, direction, pad_m, source_bias)
            if candidate is not None:
                candidates.append(candidate)

    deduped: list[dict[str, Any]] = []
    for candidate in sorted(candidates, key=lambda item: float(item.get("source_bias", 0.0)), reverse=True):
        direction = candidate.get("direction")
        if direction is None:
            continue
        if any((_undirected_angle_error_deg(direction, existing.get("direction")) or 180.0) <= 3.0 for existing in deduped):
            continue
        deduped.append(candidate)
        if len(deduped) >= int(options["axis_candidate_max_count"]):
            break
    return deduped


def _body_pass_records_for_axis(
    region: Polygon | MultiPolygon,
    candidate: dict[str, Any],
    spacing_m: float,
    min_length_m: float,
) -> list[dict[str, Any]]:
    direction = candidate.get("direction") or (1.0, 0.0)
    angle_deg = math.degrees(math.atan2(float(direction[1]), float(direction[0])))
    lines = _parallel_pass_lines(
        region,
        angle_deg=angle_deg,
        spacing_m=spacing_m,
        min_length_m=min_length_m,
    )
    return [
        {"points": points, "lane_index": index, "lane_offset_m": None}
        for index, points in enumerate(_line_pass_points(lines, task_type="body_core"))
    ]


def _body_pass_records(
    *,
    area: dict[str, Any],
    region: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    spacing_m: float,
    min_length_m: float,
    options: dict[str, Any],
) -> tuple[dict[str, Any] | None, list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    stripe_region, stripe_metadata = _stripe_generation_region(region, drivable, config, options)
    stripe_metadata["body_min_pass_length_m"] = round(
        float(max(min_length_m, float(options["body_min_pass_length_m"]))), 4
    )
    if stripe_region.is_empty:
        return None, [], [], stripe_metadata

    path_min_length = max(min_length_m, float(options["body_min_pass_length_m"]))
    candidates = _body_axis_candidates(area, region, options)
    scored: list[tuple[float, dict[str, Any], list[dict[str, Any]], dict[str, Any]]] = []
    for candidate in candidates:
        records = _body_pass_records_for_axis(stripe_region, candidate, spacing_m, path_min_length)
        summary = _axis_candidate_summary("body_core", candidate, records, stripe_region, drivable, config, options)
        scored.append((float(summary["score"]), candidate, records, summary))
    scored.sort(key=lambda item: item[0], reverse=True)
    if not scored:
        angle = _dominant_angle(stripe_region)
        direction = _unit_from_angle_deg(angle)
        fallback = _axis_candidate_for_body("oriented_bounds_fallback", stripe_region, direction, 0.0, 0.0)
        records = []
        if fallback is not None:
            records = _body_pass_records_for_axis(stripe_region, fallback, spacing_m, path_min_length)
            if records:
                axis = {
                    "source": fallback.get("source", "oriented_bounds_fallback"),
                    "portal_id": None,
                    "start": _round_point(fallback["start"][0], fallback["start"][1]),
                    "end": _round_point(fallback["end"][0], fallback["end"][1]),
                    "yaw": round(float(math.atan2(direction[1], direction[0])), 6),
                    "length_m": round(float(fallback.get("length_m", 0.0)), 4),
                    "terminal_angle_error_deg": None,
                    "selection_score": None,
                    "candidate_rank": None,
                    "candidate_count": 0,
                }
                return axis, records, [], stripe_metadata
        return None, [], [], stripe_metadata

    _score, selected, records, _summary = scored[0]
    centerline_choices = [
        item for item in scored if str(item[1].get("source", "")) == "body_centerline" and item[2]
    ]
    if centerline_choices and float(centerline_choices[0][0]) >= float(scored[0][0]) - 8.0:
        _score, selected, records, _summary = centerline_choices[0]
    axis_candidates: list[dict[str, Any]] = []
    for rank, (_candidate_score, _candidate, _candidate_records, summary) in enumerate(scored, start=1):
        summary["rank"] = rank
        summary["selected"] = rank == 1
        summary["status"] = "selected" if rank == 1 else "rejected"
        axis_candidates.append(summary)

    direction = selected["direction"]
    axis = {
        "source": selected.get("source", "unknown"),
        "portal_id": None,
        "start": _round_point(selected["start"][0], selected["start"][1]),
        "end": _round_point(selected["end"][0], selected["end"][1]),
        "yaw": round(float(math.atan2(direction[1], direction[0])), 6),
        "length_m": round(float(selected.get("length_m", 0.0)), 4),
        "terminal_angle_error_deg": None,
        "selection_score": axis_candidates[0]["score"],
        "candidate_rank": 1,
        "candidate_count": len(axis_candidates),
    }
    return axis, records, axis_candidates, stripe_metadata


def _body_pocket_service(
    *,
    coverage_region: Polygon | MultiPolygon,
    infill_region: Polygon | MultiPolygon,
    initial_segments: list[dict[str, Any]],
    body_axis_yaw: float | None,
    body_axis_source: str,
    drivable: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
    spacing_m: float,
    min_length_m: float,
    task_id: str,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if not bool(options.get("body_pocket_enabled", True)) or not initial_segments:
        return [], []
    if infill_region.is_empty:
        return [], []
    covered = _cutting_coverage_geometry(infill_region, initial_segments, config)
    residual = v2_geometry._clean_polygonal(infill_region.difference(covered))
    if residual.is_empty:
        return [], []
    outline_band = float(options["body_pocket_outline_band_m"])
    pocket_cores = residual
    if pocket_cores.is_empty:
        return [], []

    min_area = float(options["body_pocket_min_area_m2"])
    min_width = _tool_width(config) * float(options["body_pocket_min_width_factor"])
    pocket_min_length = max(min_length_m, _tool_width(config) * 2.0)
    max_strokes = int(options["body_pocket_max_strokes"])
    service_segments: list[dict[str, Any]] = []
    pockets: list[dict[str, Any]] = []
    for pocket_index, core in enumerate(
        sorted(v2_geometry._iter_polygons(pocket_cores), key=lambda poly: poly.area, reverse=True),
        start=1,
    ):
        if core.area < min_area:
            continue
        service_region = v2_geometry._clean_polygonal(
            core.buffer(_tool_width(config) * 0.25, join_style=2).intersection(residual).intersection(infill_region)
        )
        if service_region.is_empty:
            service_region = v2_geometry._clean_polygonal(core.intersection(infill_region))
        if service_region.is_empty:
            continue
        bounds = v2_geometry.oriented_bounds(service_region)
        if float(bounds.get("width_m", 0.0)) < min_width and v2_geometry.area_m2(service_region) < min_area * 1.75:
            continue
        if body_axis_yaw is None:
            angle = float(bounds.get("angle_deg", _dominant_angle(service_region)))
            axis_source = "pocket_oriented_bounds_fallback"
        else:
            angle = math.degrees(float(body_axis_yaw))
            axis_source = body_axis_source or "body_service_axis"
        lines = _parallel_pass_lines(
            service_region,
            angle_deg=angle,
            spacing_m=spacing_m,
            min_length_m=pocket_min_length,
        )
        if not lines:
            rep = service_region.representative_point()
            direction = _unit_from_angle_deg(angle)
            points = _axis_line_points(service_region, (float(rep.x), float(rep.y)), direction, spacing_m)
            if len(points) >= 2 and _path_length(points) >= pocket_min_length:
                lines = [LineString(points)]
        records = _line_pass_points(lines[:max_strokes], task_type="body_core")
        if not records:
            continue
        pocket_id = f"{task_id}-body-pocket-{len(pockets) + 1}"
        segment_ids = []
        for record_index, points in enumerate(records):
            segment = _make_segment(
                segment_id=f"{task_id}-seg-pocket-{len(service_segments)}",
                phase="body_pocket_service",
                direction="forward",
                cutting_enabled=True,
                points=points,
                drivable=drivable,
                config=config,
                options=options,
                metadata={
                    "body_pocket_id": pocket_id,
                    "lane_index": record_index,
                    "lane_offset_m": None,
                    "entry_anchor": _round_point(points[0][0], points[0][1]),
                    "deep_anchor": _round_point(points[-1][0], points[-1][1]),
                },
            )
            segment_ids.append(segment["segment_id"])
            service_segments.append(segment)
        pockets.append(
            {
                "id": pocket_id,
                "area_m2": round(v2_geometry.area_m2(service_region), 4),
                "core_area_m2": round(v2_geometry.area_m2(core), 4),
                "outline_band_m": round(float(outline_band), 4),
                "boundary_source": "stripe_safe_infill_region",
                "axis_angle_deg": round(float(angle), 2),
                "axis_source": axis_source,
                "stroke_count": len(records),
                "segment_ids": segment_ids,
                "geometry": v2_geometry.geometry_to_json(service_region),
                "core_geometry": v2_geometry.geometry_to_json(core),
                "reason": "BODY stripe field left a meaningful residual beyond the outline-owned boundary band",
            }
        )
    return service_segments, pockets


def _build_task_path(
    *,
    area_index: int,
    area: dict[str, Any],
    task: dict[str, Any],
    coverage_region: Polygon | MultiPolygon,
    drivable: Polygon | MultiPolygon,
    portal_by_id: dict[str, dict[str, Any]],
    config: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    task_id = str(task.get("id", "task"))
    task_type = str(task.get("task_type", "artifact"))
    warnings: list[str] = []
    spacing = _tool_width(config) * float(options["spacing_factor"])
    min_length = float(options["min_pass_length_m"])
    portal = _portal_point(task, portal_by_id, coverage_region)
    terminal = _terminal_point(task, coverage_region, portal)

    service_axis = None
    axis_candidates: list[dict[str, Any]] = []
    stripe_metadata: dict[str, Any] = {}
    if task_type == "body_core":
        strategy = "residual_body_axis_scored_parallel_stripes"
        service_axis, pass_records, axis_candidates, stripe_metadata = _body_pass_records(
            area=area,
            region=coverage_region,
            drivable=drivable,
            config=config,
            spacing_m=spacing,
            min_length_m=min_length,
            options=options,
        )
        if not pass_records:
            warnings.append("no BODY stripe fits the mower-width boundary clearance and minimum length")
    elif task_type == "corridor":
        stripe_region, stripe_metadata = _stripe_generation_region(coverage_region, drivable, config, options)
        angle = _dominant_angle(coverage_region)
        strategy = "corridor_lengthwise_passes"
        if stripe_region.is_empty:
            pass_records = []
            warnings.append("no corridor stripe fits the mower-width boundary clearance")
        else:
            corridor_min_length = max(min_length, float(options["body_min_pass_length_m"]))
            stripe_metadata["body_min_pass_length_m"] = round(float(corridor_min_length), 4)
            lines = _parallel_pass_lines(
                stripe_region,
                angle_deg=angle,
                spacing_m=spacing,
                min_length_m=corridor_min_length,
            )
            pass_records = [
                {"points": points, "lane_index": index, "lane_offset_m": None}
                for index, points in enumerate(_line_pass_points(lines, task_type=task_type, portal=portal))
            ]
    else:
        stripe_metadata = _stripe_clearance_metadata(config, options)
        stripe_metadata.update(
            {
                "stripe_region_source": "not_applied_single_entry_service_task",
                "stripe_region_area_m2": round(v2_geometry.area_m2(coverage_region), 4),
                "body_min_pass_length_m": round(float(min_length), 4),
            }
        )
        strategy = "dead_end_portal_passes" if task_type == "dead_end_corridor" else "notch_pop_in_reverse_out"
        service_axis, pass_records, axis_candidates = _single_entry_pass_records(
            task_type=task_type,
            task=task,
            region=coverage_region,
            drivable=drivable,
            portal_by_id=portal_by_id,
            config=config,
            spacing_m=spacing,
            min_length_m=min_length,
            options=options,
        )
        if not pass_records:
            fallback_points = _direct_portal_stroke(coverage_region, portal, terminal, min_length)
            pass_records = [
                {
                    "points": points,
                    "lane_index": index,
                    "lane_offset_m": 0.0,
                    "entry_anchor": _round_point(points[0][0], points[0][1]),
                    "deep_anchor": _round_point(points[-1][0], points[-1][1]),
                }
                for index, points in enumerate(fallback_points)
            ]
            if pass_records:
                warnings.append("used direct portal-to-terminal fallback stroke")
    if not pass_records:
        warnings.append("no local pass longer than minimum pass length")

    turnaround = _turnaround_diagnostic(
        task_type=task_type,
        pass_records=pass_records,
        coverage_region=coverage_region,
        drivable=drivable,
        config=config,
        options=options,
    )
    if task_type == "dead_end_corridor":
        exit_mode = "forward_turnaround" if bool(turnaround.get("feasible")) else "reverse_out"
    elif task_type == "notch":
        exit_mode = "reverse_out"
    elif task_type == "corridor":
        exit_mode = "through"
    else:
        exit_mode = "not_applicable"

    segments: list[dict[str, Any]] = []
    body_service_pockets: list[dict[str, Any]] = []
    compact_connector_summary: dict[str, Any] = {
        "enabled": bool(options.get("compact_connectors_enabled", True)),
        "connector_style": str(options.get("adjacent_connector_style", "straight")),
        "candidate_count": 0,
        "inserted_count": 0,
        "rejected_count": 0,
        "unsafe_rejected_count": 0,
        "too_far_rejected_count": 0,
        "unsafe_inserted_count": 0,
        "unsafe_inserted_sample_count": 0,
        "length_m": 0.0,
        "records": [],
    }
    for pass_index, pass_record in enumerate(pass_records):
        points = pass_record["points"]
        phase = "coverage_pass"
        if task_type == "dead_end_corridor":
            phase = "dead_end_entry_pass"
        elif task_type == "notch":
            phase = "notch_pop_in"
        lane_metadata = {
            "lane_index": int(pass_record.get("lane_index", pass_index)),
            "lane_offset_m": pass_record.get("lane_offset_m"),
            "entry_anchor": pass_record.get("entry_anchor"),
            "deep_anchor": pass_record.get("deep_anchor"),
        }
        segments.append(
            _make_segment(
                segment_id=f"{task_id}-seg-{len(segments)}",
                phase=phase,
                direction="forward",
                cutting_enabled=True,
                points=points,
                drivable=drivable,
                config=config,
                options=options,
                metadata=lane_metadata,
            )
        )
        if task_type == "dead_end_corridor" and exit_mode == "forward_turnaround":
            lane_yaw = math.atan2(points[-1][1] - points[0][1], points[-1][0] - points[0][0])
            segments.append(
                _make_turnaround_segment(
                    segment_id=f"{task_id}-seg-{len(segments)}",
                    point=points[-1],
                    start_yaw=lane_yaw,
                    coverage_region=coverage_region,
                    drivable=drivable,
                    config=config,
                    options=options,
                    metadata=lane_metadata,
                )
            )
            segments.append(
                _make_segment(
                    segment_id=f"{task_id}-seg-{len(segments)}",
                    phase="dead_end_forward_exit",
                    direction="forward",
                    cutting_enabled=False,
                    points=list(reversed(points)),
                    drivable=drivable,
                    config=config,
                    options=options,
                    metadata=lane_metadata,
                )
            )
        elif task_type in {"dead_end_corridor", "notch"} and bool(options["reverse_out_enabled"]):
            segments.append(
                _make_segment(
                    segment_id=f"{task_id}-seg-{len(segments)}",
                    phase="reverse_out",
                    direction="backward",
                    cutting_enabled=bool(options["reverse_cutting_enabled"]),
                    points=list(reversed(points)),
                    drivable=drivable,
                    config=config,
                    options=options,
                    metadata=lane_metadata,
                )
            )

    if task_type == "body_core" and segments:
        infill_region = coverage_region
        if stripe_metadata.get("stripe_region"):
            parsed_infill = v2_geometry._geometry_from_json(stripe_metadata.get("stripe_region", {}))
            if not parsed_infill.is_empty:
                infill_region = parsed_infill
        pocket_segments, body_service_pockets = _body_pocket_service(
            coverage_region=coverage_region,
            infill_region=infill_region,
            initial_segments=segments,
            body_axis_yaw=float(service_axis["yaw"]) if service_axis and service_axis.get("yaw") is not None else None,
            body_axis_source=str((service_axis or {}).get("source", "body_service_axis")),
            drivable=drivable,
            config=config,
            options=options,
            spacing_m=spacing,
            min_length_m=min_length,
            task_id=task_id,
        )
        if pocket_segments:
            segments.extend(pocket_segments)
            strategy = f"{strategy}_plus_body_pocket_service"
            warnings.append(
                f"added {len(pocket_segments)} BODY pocket service strokes for narrow-entry residuals"
            )

    if task_type == "body_core" and segments:
        segments, compact_connector_summary = _insert_compact_keyhole_connectors(
            task_id=task_id,
            segments=segments,
            drivable=drivable,
            config=config,
            options=options,
        )
        inserted = int(compact_connector_summary.get("inserted_count", 0))
        unsafe_rejected = int(compact_connector_summary.get("unsafe_rejected_count", 0))
        unsafe_inserted = int(compact_connector_summary.get("unsafe_inserted_count", 0))
        connector_style = str(compact_connector_summary.get("connector_style", options.get("adjacent_connector_style", "straight")))
        if inserted:
            strategy = f"{strategy}_plus_{connector_style}_connectors"
            warnings.append(f"added {inserted} {connector_style} adjacent connector segments")
        if unsafe_inserted:
            warnings.append(
                f"included {unsafe_inserted} unsafe {connector_style} adjacent connectors for temporary consumption testing"
            )
        if unsafe_rejected:
            warnings.append(f"rejected {unsafe_rejected} {connector_style} adjacent connectors for footprint safety")

    coverage_percent, covered_area, region_area = _coverage_percent(coverage_region, segments, config)
    summary = _path_summary(segments)
    unsafe_count = int(summary["unsafe_sample_count"])
    if unsafe_count:
        warnings.append(f"{unsafe_count} base-link footprint samples are outside the drivable region")
    status = "ok" if segments else "no_path"
    if status == "ok" and unsafe_count:
        status = "warning"
    return {
        "task_path_id": f"{task_id}-path",
        "area_index": area_index,
        "task_id": task_id,
        "task_type": task_type,
        "strategy": strategy,
        "status": status,
        "warnings": warnings,
        "pass_count": len(pass_records),
        "lane_count": len(pass_records),
        "service_axis": service_axis,
        "axis_candidates": axis_candidates,
        "exit_mode": exit_mode,
        "turn_feasible": bool(turnaround.get("feasible")),
        "turnaround": turnaround,
        "body_service_pockets": body_service_pockets,
        "body_service_pocket_count": len(body_service_pockets),
        "body_service_pocket_area_m2": round(
            sum(float(pocket.get("area_m2", 0.0)) for pocket in body_service_pockets), 4
        ),
        "compact_connector_summary": compact_connector_summary,
        "compact_connector_count": int(compact_connector_summary.get("inserted_count", 0)),
        "compact_connector_length_m": round(float(compact_connector_summary.get("length_m", 0.0)), 4),
        "owner_source": task.get("owner_source", task.get("extent_source", "")),
        "coverage_region": v2_geometry.geometry_to_json(coverage_region),
        "coverage_region_area_m2": round(region_area, 4),
        **stripe_metadata,
        "estimated_covered_area_m2": round(covered_area, 4),
        "estimated_coverage_percent": round(coverage_percent, 2),
        "segments": segments,
        **summary,
    }


def _coverage_regions_for_area(
    area: dict[str, Any],
    config: dict[str, Any],
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    drivable = v2_geometry._geometry_from_json(area.get("geometries", {}).get("drivable_region", {}))
    tasks = [
        task
        for task in area.get("task_proposals", [])
        if task.get("visible_by_default", True) and str(task.get("task_type", "")) != "artifact"
    ]
    non_body_regions = [
        v2_geometry._geometry_from_json(task.get("geometry", {}))
        for task in tasks
        if str(task.get("task_type", "")) != "body_core"
    ]
    non_body_union = v2_geometry._clean_polygonal(unary_union(non_body_regions)) if non_body_regions else MultiPolygon([])
    regions = []
    for task in tasks:
        task_type = str(task.get("task_type", "artifact"))
        if task_type == "body_core":
            region = v2_geometry._clean_polygonal(drivable.difference(non_body_union))
            source = "residual_drivable_minus_non_body_tasks"
        else:
            region = v2_geometry._clean_polygonal(v2_geometry._geometry_from_json(task.get("geometry", {})).intersection(drivable))
            source = "m1_6_service_region"
        if region.is_empty or v2_geometry.area_m2(region) <= EPS:
            continue
        regions.append(
            {
                "coverage_region_id": f"{task.get('id', 'task')}-coverage",
                "area_index": area.get("area_index", 0),
                "task_id": task.get("id", ""),
                "task_type": task_type,
                "source": source,
                "owner_source": task.get("extent_source", source),
                "area_m2": round(v2_geometry.area_m2(region), 4),
                "geometry": v2_geometry.geometry_to_json(region),
            }
        )
    if bool(options.get("ownership_enabled", True)):
        regions = _apply_service_ownership(area, regions, tasks, config, options)
    return regions


def _apply_service_ownership(
    area: dict[str, Any],
    regions: list[dict[str, Any]],
    tasks: list[dict[str, Any]],
    config: dict[str, Any],
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    body_records = [record for record in regions if str(record.get("task_type")) == "body_core"]
    if not body_records:
        area["ownership_adjustments"] = []
        return regions
    body_record = body_records[0]
    body_region = v2_geometry._geometry_from_json(body_record.get("geometry", {}))
    if body_region.is_empty:
        area["ownership_adjustments"] = []
        return regions

    portal_by_id = {str(portal.get("id", "")): portal for portal in area.get("portal_candidates", [])}
    task_by_id = {str(task.get("id", "")): task for task in tasks}
    body_axis = _unit_from_angle_deg(_dominant_angle(body_region))
    portal_band = float(options["ownership_portal_band_m"])
    short_pass = float(options["ownership_short_body_pass_m"])
    alignment_tolerance = float(options["ownership_axis_alignment_deg"])
    adjustments: list[dict[str, Any]] = []

    for record in regions:
        task_type = str(record.get("task_type", ""))
        if task_type in {"body_core", "artifact"}:
            continue
        task = task_by_id.get(str(record.get("task_id", "")))
        if not task:
            continue
        task_region = v2_geometry._geometry_from_json(record.get("geometry", {}))
        if task_region.is_empty:
            continue
        portal = None
        for portal_id in task.get("entry_portals", []):
            portal = portal_by_id.get(str(portal_id))
            if portal:
                break
        portal_center = _portal_center(task, portal_by_id, task_region)
        service_axis = _service_axis(
            task_type=task_type,
            task=task,
            region=task_region,
            portal_by_id=portal_by_id,
            options=options,
        )
        task_axis = service_axis.get("_direction") or _unit_from_angle_deg(_dominant_angle(task_region))
        mouth_band = Point(*portal_center).buffer(portal_band)
        candidate = v2_geometry._clean_polygonal(body_region.intersection(task_region.buffer(portal_band)).intersection(mouth_band))
        transfer_parts = []
        reasons = []
        for component in v2_geometry._iter_polygons(candidate):
            bounds = v2_geometry.oriented_bounds(component)
            comp_axis = _unit_from_angle_deg(float(bounds.get("angle_deg", 0.0)))
            task_error = _angle_error_deg(comp_axis, task_axis) or 180.0
            body_error = _angle_error_deg(comp_axis, body_axis) or 180.0
            short_fragment = float(bounds.get("length_m", 0.0)) <= short_pass
            task_aligned = task_error + alignment_tolerance < body_error
            if short_fragment or task_aligned:
                transfer_parts.append(component)
                reasons.append("short mouth-side body fragment" if short_fragment else "axis aligns better with task")
        if not transfer_parts:
            continue
        transfer = v2_geometry._clean_polygonal(unary_union(transfer_parts))
        if transfer.is_empty:
            continue
        body_region = v2_geometry._clean_polygonal(body_region.difference(transfer))
        new_task_region = v2_geometry._clean_polygonal(task_region.union(transfer))
        record["geometry"] = v2_geometry.geometry_to_json(new_task_region)
        record["area_m2"] = round(v2_geometry.area_m2(new_task_region), 4)
        record["source"] = f"{record.get('source', 'task_region')}_plus_service_ownership"
        record["owner_source"] = "service_ownership_adjusted"
        task["owner_source"] = "service_ownership_adjusted"
        adjustments.append(
            {
                "id": f"{record.get('task_id', 'task')}-ownership-{len(adjustments) + 1}",
                "task_id": record.get("task_id", ""),
                "task_type": task_type,
                "area_m2": round(v2_geometry.area_m2(transfer), 4),
                "reason": "; ".join(v2_geometry._unique_list(reasons)),
                "geometry": v2_geometry.geometry_to_json(transfer),
            }
        )

    body_record["geometry"] = v2_geometry.geometry_to_json(body_region)
    body_record["area_m2"] = round(v2_geometry.area_m2(body_region), 4)
    if adjustments:
        body_record["source"] = "residual_drivable_minus_non_body_tasks_service_ownership_adjusted"
        body_record["owner_source"] = "service_ownership_adjusted"
    area["ownership_adjustments"] = adjustments
    return regions


def _ring_coords(ring: Any) -> list[tuple[float, float]]:
    coords = [(float(x), float(y)) for x, y in ring.coords]
    if len(coords) >= 2 and coords[0] != coords[-1]:
        coords.append(coords[0])
    return coords


def _closed_ring_area(points: list[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    closed = list(points)
    if closed[0] != closed[-1]:
        closed.append(closed[0])
    area = 0.0
    for start, end in zip(closed, closed[1:]):
        area += float(start[0]) * float(end[1]) - float(end[0]) * float(start[1])
    return area * 0.5


def _reverse_closed_ring(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if len(points) < 3:
        return points
    vertices = list(points)
    if vertices[0] == vertices[-1]:
        vertices = vertices[:-1]
    vertices.reverse()
    if vertices:
        vertices.append(vertices[0])
    return vertices


def _orient_outline_ring_for_right_boundary(
    points: list[tuple[float, float]],
    ring_type: str,
) -> tuple[list[tuple[float, float]], dict[str, Any]]:
    signed_area = _closed_ring_area(points)
    wants_ccw = ring_type != "hole"
    should_reverse = (wants_ccw and signed_area < 0.0) or ((not wants_ccw) and signed_area > 0.0)
    oriented = _reverse_closed_ring(points) if should_reverse else points
    final_area = _closed_ring_area(oriented)
    return oriented, {
        "outline_boundary_side": "right",
        "outline_ring_orientation": "ccw" if final_area >= 0.0 else "cw",
        "outline_ring_was_reversed": bool(should_reverse),
    }


def _angle_delta_deg(a: float, b: float) -> float:
    return abs(math.degrees(_normalize_angle(a - b)))


def _outline_ring_segments(points: list[tuple[float, float]]) -> tuple[list[dict[str, Any]], float]:
    if len(points) < 2:
        return [], 0.0
    closed = list(points)
    if closed[0] != closed[-1]:
        closed.append(closed[0])
    segments: list[dict[str, Any]] = []
    distance_m = 0.0
    for index, (start, end) in enumerate(zip(closed, closed[1:])):
        length_m = math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
        if length_m > EPS:
            segments.append(
                {
                    "index": index,
                    "start": start,
                    "end": end,
                    "start_s": distance_m,
                    "end_s": distance_m + length_m,
                    "length_m": length_m,
                    "yaw": math.atan2(float(end[1]) - float(start[1]), float(end[0]) - float(start[0])),
                }
            )
        distance_m += length_m
    return segments, distance_m


def _outline_point_at_distance(
    segments: list[dict[str, Any]],
    total_length_m: float,
    distance_m: float,
) -> tuple[tuple[float, float], float, int]:
    if not segments or total_length_m <= EPS:
        return ((0.0, 0.0), 0.0, 0)
    sample_s = math.fmod(float(distance_m), total_length_m)
    if sample_s < 0.0:
        sample_s += total_length_m
    for segment in segments:
        if sample_s <= float(segment["end_s"]) + EPS:
            length_m = float(segment["length_m"])
            t = 0.0 if length_m <= EPS else (sample_s - float(segment["start_s"])) / length_m
            t = min(max(t, 0.0), 1.0)
            start = segment["start"]
            end = segment["end"]
            point = (
                float(start[0]) + (float(end[0]) - float(start[0])) * t,
                float(start[1]) + (float(end[1]) - float(start[1])) * t,
            )
            return point, float(segment["yaw"]), int(segment["index"])
    segment = segments[-1]
    return segment["end"], float(segment["yaw"]), int(segment["index"])


def _rotate_closed_ring_to_distance(
    points: list[tuple[float, float]],
    distance_m: float,
) -> list[tuple[float, float]]:
    if len(points) < 3:
        return points
    vertices = list(points)
    if vertices[0] == vertices[-1]:
        vertices = vertices[:-1]
    if len(vertices) < 3:
        return points
    closed = vertices + [vertices[0]]
    segments, total_length_m = _outline_ring_segments(closed)
    if not segments or total_length_m <= EPS:
        return points
    start_point, _yaw, segment_index = _outline_point_at_distance(segments, total_length_m, distance_m)
    containing = None
    for segment in segments:
        if int(segment["index"]) == segment_index:
            containing = segment
            break
    if containing is None:
        return points

    def same_point(a: tuple[float, float], b: tuple[float, float]) -> bool:
        return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])) <= EPS

    rotated: list[tuple[float, float]] = [start_point]

    def append_unique(point: tuple[float, float]) -> None:
        if not rotated or not same_point(rotated[-1], point):
            rotated.append(point)

    n = len(vertices)
    start_index = int(containing["index"]) % n
    append_unique(vertices[(start_index + 1) % n])
    index = (start_index + 2) % n
    while index != start_index:
        append_unique(vertices[index])
        index = (index + 1) % n
    append_unique(vertices[start_index])
    append_unique(start_point)
    return rotated


def _select_outline_start(
    points: list[tuple[float, float]],
    options: dict[str, Any],
) -> tuple[list[tuple[float, float]], dict[str, Any]]:
    metadata: dict[str, Any] = {
        "outline_start_selector": "disabled",
        "outline_start_reason": "selector disabled",
    }
    if not bool(options.get("outline_start_selector_enabled", True)) or len(points) < 4:
        return points, metadata
    segments, total_length_m = _outline_ring_segments(points)
    if not segments or total_length_m <= EPS:
        metadata["outline_start_reason"] = "ring too short"
        return points, metadata

    tolerance_deg = float(options.get("outline_start_straight_angle_tolerance_deg", 8.0))
    candidates: list[dict[str, Any]] = []
    segment_count = len(segments)
    for index, segment in enumerate(segments):
        yaw = float(segment["yaw"])
        run_indices = [index]
        cursor = (index - 1) % segment_count
        while cursor != index and _angle_delta_deg(float(segments[cursor]["yaw"]), yaw) <= tolerance_deg:
            run_indices.insert(0, cursor)
            cursor = (cursor - 1) % segment_count
        cursor = (index + 1) % segment_count
        while cursor != index and cursor not in run_indices and _angle_delta_deg(float(segments[cursor]["yaw"]), yaw) <= tolerance_deg:
            run_indices.append(cursor)
            cursor = (cursor + 1) % segment_count
        run_length_m = sum(float(segments[item]["length_m"]) for item in run_indices)
        run_start_s = float(segments[run_indices[0]]["start_s"])
        start_distance_m = math.fmod(run_start_s + run_length_m * 0.5, total_length_m)
        if start_distance_m < 0.0:
            start_distance_m += total_length_m
        start_point, start_yaw, _segment_index = _outline_point_at_distance(
            segments, total_length_m, start_distance_m
        )
        candidates.append(
            {
                "score": run_length_m,
                "run_length_m": run_length_m,
                "run_segment_count": len(run_indices),
                "start_distance_m": start_distance_m,
                "x": start_point[0],
                "y": start_point[1],
                "yaw": start_yaw,
                "source_segment_index": int(segment["index"]),
            }
        )
    if not candidates:
        metadata["outline_start_reason"] = "no nonzero ring segments"
        return points, metadata

    best = min(
        candidates,
        key=lambda item: (
            -float(item["score"]),
            round(float(item["y"]), 3),
            round(float(item["x"]), 3),
            round(float(item["start_distance_m"]), 3),
        ),
    )
    rotated = _rotate_closed_ring_to_distance(points, float(best["start_distance_m"]))
    metadata = {
        "outline_start_selector": "straight_run_midpoint",
        "outline_start_reason": "selected midpoint of longest deterministic straight run",
        "outline_start_distance_m": round(float(best["start_distance_m"]), 4),
        "outline_start_straight_run_m": round(float(best["run_length_m"]), 4),
        "outline_start_run_segment_count": int(best["run_segment_count"]),
        "outline_start_point": _round_point(float(best["x"]), float(best["y"])),
        "outline_start_yaw": round(float(best["yaw"]), 6),
        "outline_start_angle_tolerance_deg": round(tolerance_deg, 4),
        "outline_start_candidate_count": len(candidates),
    }
    return rotated, metadata


def _outline_safety_region(
    conditioned: Polygon | MultiPolygon,
    options: dict[str, Any],
) -> Polygon | MultiPolygon:
    clearance = float(options.get("outline_footprint_clearance_m", 0.0))
    if clearance <= EPS:
        return conditioned
    safety_region = v2_geometry._clean_polygonal(conditioned.buffer(-clearance, join_style=2))
    return safety_region if not safety_region.is_empty else conditioned


def _outline_right_edge_clearance_stats(
    segment: dict[str, Any],
    boundary: Any,
    config: dict[str, Any],
) -> dict[str, Any]:
    if boundary.is_empty:
        return {}
    offset = _tool_center_offset(config)
    tool_half_width = _tool_width(config) * 0.5
    distances: list[float] = []
    for pose in segment.get("base_poses", []):
        yaw = float(pose.get("yaw", 0.0))
        ox = float(offset[0])
        oy = float(offset[1]) - tool_half_width
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        right_edge = Point(
            float(pose.get("x", 0.0)) + ox * cos_y - oy * sin_y,
            float(pose.get("y", 0.0)) + ox * sin_y + oy * cos_y,
        )
        distances.append(float(right_edge.distance(boundary)))
    if not distances:
        return {}
    distances.sort()
    return {
        "right_cut_edge_min_clearance_m": round(distances[0], 4),
        "right_cut_edge_median_clearance_m": round(distances[len(distances) // 2], 4),
        "right_cut_edge_max_clearance_m": round(distances[-1], 4),
    }


def _outline_footprint_side_clearance_stats(
    segment: dict[str, Any],
    boundary: Any,
    config: dict[str, Any],
) -> dict[str, Any]:
    if boundary.is_empty:
        return {}
    right_side_distances: list[float] = []
    right_rear_distances: list[float] = []
    right_front_distances: list[float] = []
    footprint_distances: list[float] = []
    for pose in segment.get("base_poses", []):
        right_line = _footprint_right_side_line(pose, config)
        rear, front = _footprint_right_corner_points(pose, config)
        footprint = _footprint_polygon(pose, config)
        right_side_distances.append(float(right_line.distance(boundary)))
        right_rear_distances.append(float(Point(rear).distance(boundary)))
        right_front_distances.append(float(Point(front).distance(boundary)))
        footprint_distances.append(float(footprint.distance(boundary)))
    if not right_side_distances:
        return {}
    right_side_distances.sort()
    right_rear_distances.sort()
    right_front_distances.sort()
    footprint_distances.sort()
    right_min = right_side_distances[0]
    right_median = right_side_distances[len(right_side_distances) // 2]
    right_max = right_side_distances[-1]
    return {
        "right_footprint_side_min_clearance_m": round(right_min, 4),
        "right_footprint_side_median_clearance_m": round(right_median, 4),
        "right_footprint_side_max_clearance_m": round(right_max, 4),
        "right_rear_corner_min_clearance_m": round(right_rear_distances[0], 4),
        "right_front_corner_min_clearance_m": round(right_front_distances[0], 4),
        "footprint_min_clearance_m": round(footprint_distances[0], 4),
        "footprint_median_clearance_m": round(footprint_distances[len(footprint_distances) // 2], 4),
        "right_cut_edge_min_clearance_m": round(right_min, 4),
        "right_cut_edge_median_clearance_m": round(right_median, 4),
        "right_cut_edge_max_clearance_m": round(right_max, 4),
    }


def _outline_layer_initial_offset(
    *,
    layer_index: int,
    tool_width: float,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[float, str]:
    offset = float(options.get("outline_offset_m", 0.0))
    layer_offset = int(layer_index) * tool_width
    if str(options.get("outline_generation_mode", "footprint_disk")) == "right_edge_fit":
        return (
            float(options["outline_right_edge_clearance_m"]) + tool_width * 0.5 + offset + layer_offset,
            "right_edge_clearance_plus_half_tool_width",
        )
    return (
        _outline_clearance(config) + offset + layer_offset,
        _outline_clearance_source(config),
    )


def _line_intersection(
    point: tuple[float, float],
    direction: tuple[float, float],
    other_point: tuple[float, float],
    other_direction: tuple[float, float],
) -> tuple[float, float] | None:
    px, py = point
    dx, dy = direction
    qx, qy = other_point
    ux, uy = other_direction
    denom = dx * uy - dy * ux
    if abs(denom) <= EPS:
        return None
    rel_x = qx - px
    rel_y = qy - py
    distance = (rel_x * uy - rel_y * ux) / denom
    return (px + dx * distance, py + dy * distance)


def _outline_vertices(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    vertices = list(points)
    if vertices and vertices[0] == vertices[-1]:
        vertices = vertices[:-1]
    cleaned: list[tuple[float, float]] = []
    for point in vertices:
        if cleaned and math.hypot(point[0] - cleaned[-1][0], point[1] - cleaned[-1][1]) <= EPS:
            continue
        cleaned.append(point)
    if len(cleaned) > 1 and math.hypot(cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]) <= EPS:
        cleaned.pop()
    return cleaned


def _outline_edge_records(vertices: list[tuple[float, float]]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    count = len(vertices)
    for index in range(count):
        start = vertices[index]
        end = vertices[(index + 1) % count]
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length_m = math.hypot(dx, dy)
        if length_m <= EPS:
            records.append({"length_m": 0.0, "valid": False})
            continue
        tx = dx / length_m
        ty = dy / length_m
        records.append(
            {
                "index": index,
                "start": start,
                "end": end,
                "length_m": length_m,
                "valid": True,
                "tangent": (tx, ty),
                "left_normal": (-ty, tx),
                "yaw": math.atan2(ty, tx),
            }
        )
    return records


def _sample_straight_base_poses(
    start: tuple[float, float],
    end: tuple[float, float],
    yaw: float,
    sample_step_m: float,
) -> list[dict[str, float]]:
    distance = math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
    if distance <= EPS:
        return [{"x": float(start[0]), "y": float(start[1]), "yaw": float(yaw)}]
    count = max(1, int(math.ceil(distance / max(sample_step_m, EPS))))
    poses = []
    for index in range(count + 1):
        t = index / count
        poses.append(
            {
                "x": float(start[0]) + (float(end[0]) - float(start[0])) * t,
                "y": float(start[1]) + (float(end[1]) - float(start[1])) * t,
                "yaw": float(yaw),
            }
        )
    return poses


def _base_poses_from_outline_points(
    points: list[tuple[float, float]],
    options: dict[str, Any],
) -> list[dict[str, float]]:
    samples = _sample_polyline_tool_poses(
        points,
        float(options["outline_sample_step_m"]),
        "forward",
        yaw_window_m=0.0,
    )
    return [
        {
            "x": float(sample["x"]),
            "y": float(sample["y"]),
            "yaw": float(sample["yaw"]),
        }
        for sample in samples
    ]


def _sample_corner_arc(
    *,
    center: tuple[float, float],
    radius_m: float,
    start_yaw: float,
    yaw_delta: float,
    sample_step_m: float,
) -> list[dict[str, float]]:
    if radius_m <= EPS or abs(yaw_delta) <= EPS:
        return []
    turn_sign = 1.0 if yaw_delta > 0.0 else -1.0
    count = max(2, int(math.ceil(abs(yaw_delta) * radius_m / max(sample_step_m, EPS))))
    poses = []
    cx, cy = center
    for index in range(count + 1):
        t = index / count
        yaw = _normalize_angle(start_yaw + yaw_delta * t)
        poses.append(
            {
                "x": float(cx) + turn_sign * math.sin(yaw) * radius_m,
                "y": float(cy) - turn_sign * math.cos(yaw) * radius_m,
                "yaw": yaw,
            }
        )
    return poses


def _corner_arc_unsafe_count(
    poses: list[dict[str, float]],
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
) -> int:
    unsafe = 0
    for pose in poses:
        if not _covers_with_tolerance(safety_region, _footprint_polygon(pose, config)):
            unsafe += 1
    return unsafe


def _footprint_points(config: dict[str, Any]) -> list[tuple[float, float]]:
    footprint = config.get("footprint") or []
    if len(footprint) >= 3:
        return [(float(px), float(py)) for px, py in footprint]
    half = _tool_width(config) * 0.5
    return [(-half, half), (half, half), (half, -half), (-half, -half)]


def _rotated_point(point: tuple[float, float], yaw: float) -> tuple[float, float]:
    px, py = point
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return (px * cos_y - py * sin_y, px * sin_y + py * cos_y)


def _footprint_min_projection(
    normal: tuple[float, float],
    yaw: float,
    config: dict[str, Any],
) -> float:
    return min(_dot(normal, _rotated_point(point, yaw)) for point in _footprint_points(config))


def _solve_halfplane_support_pose(
    *,
    vertex: tuple[float, float],
    previous_edge: dict[str, Any],
    next_edge: dict[str, Any],
    right_clearance_m: float,
    yaw: float,
    config: dict[str, Any],
) -> tuple[float, float] | None:
    prev_n = previous_edge["left_normal"]
    next_n = next_edge["left_normal"]
    prev_min = _footprint_min_projection(prev_n, yaw, config)
    next_min = _footprint_min_projection(next_n, yaw, config)
    prev_c = right_clearance_m + _dot(prev_n, vertex) - prev_min
    next_c = right_clearance_m + _dot(next_n, vertex) - next_min
    denom = float(prev_n[0]) * float(next_n[1]) - float(prev_n[1]) * float(next_n[0])
    if abs(denom) <= EPS:
        return None
    return (
        (prev_c * float(next_n[1]) - float(prev_n[1]) * next_c) / denom,
        (float(prev_n[0]) * next_c - prev_c * float(next_n[0])) / denom,
    )


def _build_support_corner_at_offset(
    *,
    vertex: tuple[float, float],
    previous_edge: dict[str, Any],
    next_edge: dict[str, Any],
    base_offset_m: float,
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any] | None:
    yaw_delta = _normalize_angle(float(next_edge["yaw"]) - float(previous_edge["yaw"]))
    abs_yaw_delta = abs(yaw_delta)
    if abs_yaw_delta <= math.radians(1.0) or abs_yaw_delta >= math.pi - math.radians(1.0):
        return None

    extents = _footprint_extents(config)
    right_clearance = max(0.0, float(base_offset_m) - float(extents["right_offset_m"]))
    effective_radius = max(float(base_offset_m), 0.10)
    sample_step = max(float(options["outline_corner_sample_step_m"]), EPS)
    count = max(2, int(math.ceil(abs_yaw_delta * effective_radius / sample_step)))
    poses: list[dict[str, float]] = []
    for index in range(count + 1):
        t = index / count
        yaw = _normalize_angle(float(previous_edge["yaw"]) + yaw_delta * t)
        point = _solve_halfplane_support_pose(
            vertex=vertex,
            previous_edge=previous_edge,
            next_edge=next_edge,
            right_clearance_m=right_clearance,
            yaw=yaw,
            config=config,
        )
        if point is None:
            return None
        poses.append({"x": float(point[0]), "y": float(point[1]), "yaw": yaw})
    unsafe_count = _corner_arc_unsafe_count(poses, safety_region, config)
    first = poses[0]
    last = poses[-1]
    prev_t = previous_edge["tangent"]
    next_t = next_edge["tangent"]
    entry_trim = max(
        0.0,
        _dot((float(vertex[0]) - float(first["x"]), float(vertex[1]) - float(first["y"])), prev_t),
    )
    exit_trim = max(
        0.0,
        _dot((float(last["x"]) - float(vertex[0]), float(last["y"]) - float(vertex[1])), next_t),
    )
    turn_label = "left" if yaw_delta > 0.0 else "right"
    return {
        "entry_point": (float(first["x"]), float(first["y"])),
        "exit_point": (float(last["x"]), float(last["y"])),
        "arc_poses": poses,
        "radius_m": 0.0,
        "trim_m": max(entry_trim, exit_trim),
        "entry_trim_m": entry_trim,
        "exit_trim_m": exit_trim,
        "yaw_delta_deg": round(math.degrees(yaw_delta), 3),
        "unsafe_candidate_count": unsafe_count,
        "candidate_count": 1,
        "candidate_records": [
            {
                "solver": "rotated_footprint_halfplane_support",
                "right_clearance_m": round(right_clearance, 4),
                "sample_count": len(poses),
                "unsafe_sample_count": unsafe_count,
                "entry_trim_m": round(entry_trim, 4),
                "exit_trim_m": round(exit_trim, 4),
                "status": "accepted" if unsafe_count == 0 else "rejected",
                "reason": "safe" if unsafe_count == 0 else "footprint_leaves_outline_safety_region",
            }
        ],
        "corner_solver": "rotated_footprint_halfplane_support",
        "status": f"safe_support_{turn_label}_turn" if unsafe_count == 0 else f"unsafe_support_{turn_label}_turn",
    }


def _append_base_pose(poses: list[dict[str, float]], pose: dict[str, float]) -> None:
    if poses:
        previous = poses[-1]
        distance = math.hypot(float(pose["x"]) - float(previous["x"]), float(pose["y"]) - float(previous["y"]))
        yaw_delta = abs(_normalize_angle(float(pose["yaw"]) - float(previous["yaw"])))
        if distance <= EPS and yaw_delta <= 1e-6:
            return
    poses.append({"x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose["yaw"])})


def _footprint_side_base_offset_m(
    *,
    layer_index: int,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[float, str]:
    extents = _footprint_extents(config)
    layer_offset = int(layer_index) * _tool_width(config)
    offset = max(0.0, float(options.get("outline_offset_m", 0.0)))
    return (
        float(extents["right_offset_m"])
        + float(options["outline_right_footprint_clearance_m"])
        + offset
        + layer_offset,
        "right_footprint_clearance_plus_right_footprint_offset",
    )


def _build_footprint_side_corner_at_offset(
    *,
    vertex: tuple[float, float],
    previous_edge: dict[str, Any],
    next_edge: dict[str, Any],
    base_offset_m: float,
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    prev_t = previous_edge["tangent"]
    next_t = next_edge["tangent"]
    prev_n = previous_edge["left_normal"]
    next_n = next_edge["left_normal"]
    prev_line_point = (
        float(vertex[0]) + float(prev_n[0]) * base_offset_m,
        float(vertex[1]) + float(prev_n[1]) * base_offset_m,
    )
    next_line_point = (
        float(vertex[0]) + float(next_n[0]) * base_offset_m,
        float(vertex[1]) + float(next_n[1]) * base_offset_m,
    )
    miter = _line_intersection(prev_line_point, prev_t, next_line_point, next_t) or next_line_point
    yaw_delta = _normalize_angle(float(next_edge["yaw"]) - float(previous_edge["yaw"]))
    abs_yaw_delta = abs(yaw_delta)
    angle_tolerance = math.radians(1.0)
    if abs_yaw_delta <= angle_tolerance or abs_yaw_delta >= math.pi - angle_tolerance:
        pose = {"x": float(miter[0]), "y": float(miter[1]), "yaw": float(next_edge["yaw"])}
        unsafe_count = _corner_arc_unsafe_count([pose], safety_region, config)
        return {
            "entry_point": miter,
            "exit_point": miter,
            "arc_poses": [pose],
            "radius_m": 0.0,
            "trim_m": 0.0,
            "yaw_delta_deg": round(math.degrees(yaw_delta), 3),
            "unsafe_candidate_count": unsafe_count,
            "candidate_count": 0,
            "status": "straight_or_hairpin" if unsafe_count == 0 else "unsafe_straight_or_hairpin",
        }

    support_corner = _build_support_corner_at_offset(
        vertex=vertex,
        previous_edge=previous_edge,
        next_edge=next_edge,
        base_offset_m=base_offset_m,
        safety_region=safety_region,
        config=config,
        options=options,
    )
    if support_corner is not None and int(support_corner.get("unsafe_candidate_count", 0)) == 0:
        return support_corner

    max_trim = min(float(previous_edge["length_m"]), float(next_edge["length_m"])) * 0.45
    candidate_records: list[dict[str, Any]] = list(support_corner.get("candidate_records", [])) if support_corner else []
    best: dict[str, Any] | None = support_corner
    turn_sign = 1.0 if yaw_delta > 0.0 else -1.0
    turn_label = "left" if turn_sign > 0.0 else "right"
    for radius_m in options.get("outline_corner_candidate_radii_m", []):
        radius = float(radius_m)
        trim = radius * math.tan(abs_yaw_delta * 0.5)
        if trim <= EPS or trim > max_trim:
            candidate_records.append(
                {
                    "radius_m": round(radius, 4),
                    "status": "rejected",
                    "reason": "trim_exceeds_adjacent_edge_length",
                    "trim_m": round(trim, 4),
                }
            )
            continue
        start = (float(miter[0]) - float(prev_t[0]) * trim, float(miter[1]) - float(prev_t[1]) * trim)
        end = (float(miter[0]) + float(next_t[0]) * trim, float(miter[1]) + float(next_t[1]) * trim)
        center = (
            float(start[0]) + turn_sign * float(prev_n[0]) * radius,
            float(start[1]) + turn_sign * float(prev_n[1]) * radius,
        )
        arc_poses = _sample_corner_arc(
            center=center,
            radius_m=radius,
            start_yaw=float(previous_edge["yaw"]),
            yaw_delta=yaw_delta,
            sample_step_m=float(options["outline_corner_sample_step_m"]),
        )
        if arc_poses:
            arc_poses[0]["x"] = start[0]
            arc_poses[0]["y"] = start[1]
            arc_poses[0]["yaw"] = float(previous_edge["yaw"])
            arc_poses[-1]["x"] = end[0]
            arc_poses[-1]["y"] = end[1]
            arc_poses[-1]["yaw"] = float(next_edge["yaw"])
        unsafe_count = _corner_arc_unsafe_count(arc_poses, safety_region, config)
        record = {
            "radius_m": round(radius, 4),
            "trim_m": round(trim, 4),
            "unsafe_sample_count": unsafe_count,
            "status": "accepted" if unsafe_count == 0 else "rejected",
            "reason": "safe" if unsafe_count == 0 else "footprint_leaves_outline_safety_region",
        }
        candidate_records.append(record)
        candidate = {
            "entry_point": start,
            "exit_point": end,
            "arc_poses": arc_poses,
            "radius_m": radius,
            "trim_m": trim,
            "unsafe_candidate_count": unsafe_count,
            "candidate_count": len(candidate_records),
            "candidate_records": candidate_records,
            "yaw_delta_deg": round(math.degrees(yaw_delta), 3),
            "status": f"safe_{turn_label}_turn" if unsafe_count == 0 else f"unsafe_{turn_label}_turn",
        }
        if unsafe_count == 0:
            best = candidate
            break
        if best is None or unsafe_count < int(best.get("unsafe_candidate_count", 10**9)):
            best = candidate
    if best is not None:
        best["candidate_records"] = candidate_records
        best["candidate_count"] = len(candidate_records)
        return best

    pose = {"x": float(miter[0]), "y": float(miter[1]), "yaw": float(next_edge["yaw"])}
    return {
        "entry_point": miter,
        "exit_point": miter,
        "arc_poses": [pose],
        "radius_m": 0.0,
        "trim_m": 0.0,
        "yaw_delta_deg": round(math.degrees(yaw_delta), 3),
        "unsafe_candidate_count": 0,
        "candidate_count": len(candidate_records),
        "candidate_records": candidate_records,
        "status": "fallback_miter",
    }


def _build_footprint_side_corner(
    *,
    vertex: tuple[float, float],
    previous_edge: dict[str, Any],
    next_edge: dict[str, Any],
    base_offset_m: float,
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    fit_step = float(options.get("outline_fit_offset_step_m", 0.05))
    fit_max_extra = float(options.get("outline_corner_fit_max_extra_m", 0.0))
    max_attempts = max(1, int(math.floor(fit_max_extra / fit_step)) + 1)
    best: dict[str, Any] | None = None
    for attempt_index in range(max_attempts):
        local_offset = base_offset_m + attempt_index * fit_step
        corner = _build_footprint_side_corner_at_offset(
            vertex=vertex,
            previous_edge=previous_edge,
            next_edge=next_edge,
            base_offset_m=local_offset,
            safety_region=safety_region,
            config=config,
            options=options,
        )
        corner["base_offset_m"] = round(float(local_offset), 4)
        corner["local_fit_attempt_index"] = attempt_index
        corner["local_fit_extra_offset_m"] = round(float(local_offset - base_offset_m), 4)
        unsafe_count = int(corner.get("unsafe_candidate_count", 0))
        if best is None or unsafe_count < int(best.get("unsafe_candidate_count", 10**9)):
            best = corner
        if unsafe_count == 0:
            return corner
    return best if best is not None else {
        "entry_point": vertex,
        "exit_point": vertex,
        "arc_poses": [],
        "radius_m": 0.0,
        "trim_m": 0.0,
        "yaw_delta_deg": 0.0,
        "unsafe_candidate_count": 0,
        "candidate_count": 0,
        "base_offset_m": round(float(base_offset_m), 4),
        "local_fit_attempt_index": 0,
        "local_fit_extra_offset_m": 0.0,
        "status": "missing_corner",
    }


def _build_footprint_side_base_poses(
    points: list[tuple[float, float]],
    *,
    base_offset_m: float,
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    vertices = _outline_vertices(points)
    if len(vertices) < 3:
        return [], {"status": "too_few_vertices", "corner_records": []}
    edges = _outline_edge_records(vertices)
    if any(not edge.get("valid", False) for edge in edges):
        return [], {"status": "invalid_edge", "corner_records": []}

    corners: list[dict[str, Any]] = []
    for index, vertex in enumerate(vertices):
        corner = _build_footprint_side_corner(
            vertex=vertex,
            previous_edge=edges[(index - 1) % len(edges)],
            next_edge=edges[index],
            base_offset_m=base_offset_m,
            safety_region=safety_region,
            config=config,
            options=options,
        )
        corner["vertex_index"] = index
        corner["vertex"] = _round_point(vertex[0], vertex[1])
        corners.append(corner)

    poses: list[dict[str, float]] = []
    sample_step = float(options["outline_sample_step_m"])
    for edge_index, edge in enumerate(edges):
        current_corner = corners[edge_index]
        next_corner = corners[(edge_index + 1) % len(corners)]
        straight_poses = _sample_straight_base_poses(
            current_corner["exit_point"],
            next_corner["entry_point"],
            float(edge["yaw"]),
            sample_step,
        )
        for pose in straight_poses:
            _append_base_pose(poses, pose)
        for pose in next_corner.get("arc_poses", []):
            _append_base_pose(poses, pose)
    if poses:
        first = poses[0]
        last = poses[-1]
        if (
            math.hypot(float(first["x"]) - float(last["x"]), float(first["y"]) - float(last["y"])) > EPS
            or abs(_normalize_angle(float(first["yaw"]) - float(last["yaw"]))) > 1e-6
        ):
            poses.append(dict(first))
        else:
            poses.append(dict(first))

    safe_corner_count = sum(1 for corner in corners if str(corner.get("status", "")).startswith("safe_"))
    metadata = {
        "status": "ok",
        "corner_count": len(corners),
        "safe_turn_corner_count": safe_corner_count,
        "safe_left_turn_corner_count": safe_corner_count,
        "corner_arc_count": sum(1 for corner in corners if float(corner.get("radius_m", 0.0)) > EPS),
        "corner_unsafe_candidate_count": sum(int(corner.get("unsafe_candidate_count", 0)) for corner in corners),
        "corner_local_fit_max_extra_offset_m": round(
            max((float(corner.get("local_fit_extra_offset_m", 0.0)) for corner in corners), default=0.0),
            4,
        ),
        "corner_records": [
            {
                "vertex_index": int(corner.get("vertex_index", 0)),
                "vertex": corner.get("vertex"),
                "status": corner.get("status"),
                "base_offset_m": round(float(corner.get("base_offset_m", base_offset_m)), 4),
                "local_fit_attempt_index": int(corner.get("local_fit_attempt_index", 0)),
                "local_fit_extra_offset_m": round(float(corner.get("local_fit_extra_offset_m", 0.0)), 4),
                "corner_solver": corner.get("corner_solver", "radius_arc_fallback"),
                "radius_m": round(float(corner.get("radius_m", 0.0)), 4),
                "trim_m": round(float(corner.get("trim_m", 0.0)), 4),
                "entry_trim_m": round(float(corner.get("entry_trim_m", corner.get("trim_m", 0.0))), 4),
                "exit_trim_m": round(float(corner.get("exit_trim_m", corner.get("trim_m", 0.0))), 4),
                "yaw_delta_deg": corner.get("yaw_delta_deg"),
                "unsafe_candidate_count": int(corner.get("unsafe_candidate_count", 0)),
                "candidate_count": int(corner.get("candidate_count", 0)),
            }
            for corner in corners
        ],
    }
    return poses, metadata


def _sample_outline_trajectory_targets(
    points: list[tuple[float, float]],
    *,
    base_offset_m: float,
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    closed = list(points)
    if len(closed) < 3:
        return []
    if closed[0] != closed[-1]:
        closed.append(closed[0])
    segments, total_length_m = _outline_ring_segments(closed)
    if not segments or total_length_m <= EPS:
        return []
    sample_step = max(float(options["outline_sample_step_m"]), EPS)
    count = max(3, int(math.ceil(total_length_m / sample_step)))
    yaw_window = max(float(options.get("outline_trajectory_yaw_window_m", sample_step)), sample_step)
    targets: list[dict[str, Any]] = []
    for index in range(count):
        distance_m = total_length_m * index / count
        point, segment_yaw, segment_index = _outline_point_at_distance(
            segments, total_length_m, distance_m
        )
        before, _before_yaw, _before_index = _outline_point_at_distance(
            segments, total_length_m, distance_m - yaw_window * 0.5
        )
        after, _after_yaw, _after_index = _outline_point_at_distance(
            segments, total_length_m, distance_m + yaw_window * 0.5
        )
        if math.hypot(float(after[0]) - float(before[0]), float(after[1]) - float(before[1])) > EPS:
            yaw = math.atan2(float(after[1]) - float(before[1]), float(after[0]) - float(before[0]))
        else:
            yaw = float(segment_yaw)
        left_normal = (-math.sin(yaw), math.cos(yaw))
        base_pose = {
            "x": float(point[0]) + left_normal[0] * base_offset_m,
            "y": float(point[1]) + left_normal[1] * base_offset_m,
            "yaw": _normalize_angle(yaw),
        }
        targets.append(
            {
                "index": index,
                "s": distance_m,
                "boundary_point": point,
                "segment_index": segment_index,
                "yaw": _normalize_angle(yaw),
                "left_normal": left_normal,
                "base_offset_m": float(base_offset_m),
                "base_pose": base_pose,
            }
        )
    return targets


def _pose_is_footprint_safe(
    pose: dict[str, float],
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
) -> bool:
    return _covers_with_tolerance(safety_region, _footprint_polygon(pose, config))


def _project_pose_inward_for_target(
    pose: dict[str, float],
    target: dict[str, Any],
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    *,
    step_m: float,
    max_extra_m: float,
) -> tuple[dict[str, float], float, bool]:
    candidate = {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "yaw": _normalize_angle(float(pose.get("yaw", 0.0))),
    }
    if _pose_is_footprint_safe(candidate, safety_region, config):
        return candidate, 0.0, True
    normal = target.get("left_normal", (-math.sin(candidate["yaw"]), math.cos(candidate["yaw"])))
    nx = float(normal[0])
    ny = float(normal[1])
    step = max(float(step_m), EPS)
    max_extra = max(float(max_extra_m), 0.0)
    attempts = max(0, int(math.ceil(max_extra / step)))
    for attempt in range(1, attempts + 1):
        extra = min(max_extra, attempt * step)
        shifted = {
            "x": candidate["x"] + nx * extra,
            "y": candidate["y"] + ny * extra,
            "yaw": candidate["yaw"],
        }
        if _pose_is_footprint_safe(shifted, safety_region, config):
            return shifted, extra, True

    anchor = target.get("base_pose", candidate)
    anchor_yaw = _normalize_angle(float(anchor.get("yaw", candidate["yaw"])))
    anchor_normal = (-math.sin(anchor_yaw), math.cos(anchor_yaw))
    for attempt in range(0, attempts + 1):
        extra = min(max_extra, attempt * step)
        shifted = {
            "x": float(anchor.get("x", candidate["x"])) + anchor_normal[0] * extra,
            "y": float(anchor.get("y", candidate["y"])) + anchor_normal[1] * extra,
            "yaw": anchor_yaw,
        }
        if _pose_is_footprint_safe(shifted, safety_region, config):
            return shifted, extra, True

    boundary_point = target.get("boundary_point")
    base_offset_m = float(target.get("base_offset_m", 0.0))
    if boundary_point is not None and base_offset_m > EPS:
        target_yaw = float(target.get("yaw", candidate["yaw"]))
        yaw_step = math.radians(5.0)
        yaw_sweep = math.radians(45.0)
        yaw_offsets = [0.0]
        for step_index in range(1, int(math.floor(yaw_sweep / yaw_step)) + 1):
            yaw_offsets.extend([step_index * yaw_step, -step_index * yaw_step])
        for yaw_offset in yaw_offsets:
            yaw = _normalize_angle(target_yaw + yaw_offset)
            yaw_normal = (-math.sin(yaw), math.cos(yaw))
            for attempt in range(0, attempts + 1):
                extra = min(max_extra, attempt * step)
                repaired = {
                    "x": float(boundary_point[0]) + yaw_normal[0] * (base_offset_m + extra),
                    "y": float(boundary_point[1]) + yaw_normal[1] * (base_offset_m + extra),
                    "yaw": yaw,
                }
                if _pose_is_footprint_safe(repaired, safety_region, config):
                    return repaired, extra, True
    return candidate, max_extra, False


def _weighted_yaw_average(weighted_yaws: list[tuple[float, float]]) -> float:
    sx = 0.0
    sy = 0.0
    for yaw, weight in weighted_yaws:
        sx += math.cos(float(yaw)) * float(weight)
        sy += math.sin(float(yaw)) * float(weight)
    if abs(sx) <= EPS and abs(sy) <= EPS:
        return 0.0
    return _normalize_angle(math.atan2(sy, sx))


def _elastic_outline_anchor_poses(
    targets: list[dict[str, Any]],
    *,
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    poses: list[dict[str, float]] = []
    projected = 0
    failed = 0
    max_extra = 0.0
    for target in targets:
        pose, extra, safe = _project_pose_inward_for_target(
            target["base_pose"],
            target,
            safety_region,
            config,
            step_m=float(options["outline_trajectory_projection_step_m"]),
            max_extra_m=float(options["outline_trajectory_projection_max_extra_m"]),
        )
        if extra > EPS:
            projected += 1
        if not safe:
            failed += 1
        max_extra = max(max_extra, extra)
        poses.append(pose)
    return poses, {
        "anchor_projection_count": projected,
        "anchor_projection_failed_count": failed,
        "anchor_projection_max_extra_m": round(max_extra, 4),
    }


def _cspace_shell_anchor_poses(
    targets: list[dict[str, Any]],
    *,
    base_offset_m: float,
    safety_region: Polygon | MultiPolygon,
    boundary: Any,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    yaw_sweep = math.radians(float(options["outline_cspace_yaw_sweep_deg"]))
    yaw_step = math.radians(float(options["outline_cspace_yaw_step_deg"]))
    offset_step = float(options["outline_cspace_offset_step_m"])
    max_extra = float(options["outline_cspace_max_extra_m"])
    yaw_offsets = [0.0]
    steps = max(0, int(math.floor(yaw_sweep / max(yaw_step, EPS))))
    for step_index in range(1, steps + 1):
        delta = step_index * yaw_step
        yaw_offsets.extend([delta, -delta])
    offset_count = max(0, int(math.ceil(max_extra / max(offset_step, EPS))))
    poses: list[dict[str, float]] = []
    failed = 0
    max_selected_extra = 0.0
    selected_yaw_offsets: list[float] = []
    candidate_counts: list[int] = []
    for target in targets:
        boundary_point = target["boundary_point"]
        target_yaw = float(target["yaw"])
        best: dict[str, Any] | None = None
        candidate_count = 0
        for yaw_offset in yaw_offsets:
            yaw = _normalize_angle(target_yaw + yaw_offset)
            left_normal = (-math.sin(yaw), math.cos(yaw))
            for offset_index in range(offset_count + 1):
                extra = min(max_extra, offset_index * offset_step)
                pose = {
                    "x": float(boundary_point[0]) + left_normal[0] * (base_offset_m + extra),
                    "y": float(boundary_point[1]) + left_normal[1] * (base_offset_m + extra),
                    "yaw": yaw,
                }
                candidate_count += 1
                if not _pose_is_footprint_safe(pose, safety_region, config):
                    continue
                right_distance = float(_footprint_right_side_line(pose, config).distance(boundary))
                yaw_penalty = abs(math.degrees(_normalize_angle(yaw - target_yaw))) * 0.01
                score = right_distance * 20.0 + extra * 6.0 + yaw_penalty
                candidate = {
                    "pose": pose,
                    "extra_m": extra,
                    "yaw_offset": yaw_offset,
                    "score": score,
                }
                if best is None or score < float(best["score"]):
                    best = candidate
        candidate_counts.append(candidate_count)
        if best is None:
            failed += 1
            pose, extra, _safe = _project_pose_inward_for_target(
                target["base_pose"],
                target,
                safety_region,
                config,
                step_m=offset_step,
                max_extra_m=max_extra,
            )
            selected_yaw_offsets.append(0.0)
            max_selected_extra = max(max_selected_extra, extra)
            poses.append(pose)
        else:
            selected_yaw_offsets.append(float(best["yaw_offset"]))
            max_selected_extra = max(max_selected_extra, float(best["extra_m"]))
            poses.append(best["pose"])
    return poses, {
        "cspace_target_count": len(targets),
        "cspace_candidate_count": sum(candidate_counts),
        "cspace_failed_target_count": failed,
        "cspace_max_selected_extra_m": round(max_selected_extra, 4),
        "cspace_max_selected_yaw_offset_deg": round(
            max((abs(math.degrees(value)) for value in selected_yaw_offsets), default=0.0), 4
        ),
    }


def _optimize_outline_pose_loop(
    targets: list[dict[str, Any]],
    anchors: list[dict[str, float]],
    *,
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    if len(anchors) < 3:
        return list(anchors), {"optimizer_projection_count": 0, "optimizer_projection_failed_count": 0}
    poses = [dict(pose) for pose in anchors]
    iterations = int(options["outline_trajectory_iterations"])
    anchor_weight = float(options["outline_trajectory_anchor_weight"])
    smooth_weight = float(options["outline_trajectory_smooth_weight"])
    projection_step = float(options["outline_trajectory_projection_step_m"])
    projection_max = float(options["outline_trajectory_projection_max_extra_m"])
    projection_count = 0
    failed_count = 0
    max_extra = 0.0
    for _iteration in range(iterations):
        updated: list[dict[str, float]] = []
        for index, anchor in enumerate(anchors):
            previous = poses[(index - 1) % len(poses)]
            current = poses[index]
            following = poses[(index + 1) % len(poses)]
            neighbor_x = (float(previous["x"]) + float(following["x"])) * 0.5
            neighbor_y = (float(previous["y"]) + float(following["y"])) * 0.5
            denom = anchor_weight + smooth_weight
            if denom <= EPS:
                raw = dict(current)
            else:
                raw = {
                    "x": (anchor_weight * float(anchor["x"]) + smooth_weight * neighbor_x) / denom,
                    "y": (anchor_weight * float(anchor["y"]) + smooth_weight * neighbor_y) / denom,
                    "yaw": _weighted_yaw_average(
                        [
                            (float(anchor["yaw"]), anchor_weight),
                            (float(previous["yaw"]), smooth_weight * 0.5),
                            (float(following["yaw"]), smooth_weight * 0.5),
                        ]
                    ),
                }
            projected, extra, safe = _project_pose_inward_for_target(
                raw,
                targets[index],
                safety_region,
                config,
                step_m=projection_step,
                max_extra_m=projection_max,
            )
            if extra > EPS:
                projection_count += 1
            if not safe:
                failed_count += 1
            max_extra = max(max_extra, extra)
            updated.append(projected)
        poses = updated
    return poses, {
        "optimizer_iteration_count": iterations,
        "optimizer_projection_count": projection_count,
        "optimizer_projection_failed_count": failed_count,
        "optimizer_projection_max_extra_m": round(max_extra, 4),
    }


def _densify_outline_pose_loop(
    poses: list[dict[str, float]],
    *,
    targets: list[dict[str, Any]],
    safety_region: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    if len(poses) < 2:
        return poses, {"densified_pose_count": len(poses), "densify_projection_failed_count": 0}
    sample_step = max(float(options["outline_sample_step_m"]), EPS)
    max_yaw_step = math.radians(float(options["outline_trajectory_max_yaw_step_deg"]))
    projection_step = float(options["outline_trajectory_projection_step_m"])
    projection_max = float(options["outline_trajectory_projection_max_extra_m"])
    dense: list[dict[str, float]] = []
    projection_failed_count = 0
    projection_count = 0
    max_extra = 0.0
    for index, start in enumerate(poses):
        end = poses[(index + 1) % len(poses)]
        dx = float(end["x"]) - float(start["x"])
        dy = float(end["y"]) - float(start["y"])
        distance = math.hypot(dx, dy)
        yaw_delta = _normalize_angle(float(end["yaw"]) - float(start["yaw"]))
        count = max(
            1,
            int(math.ceil(distance / sample_step)),
            int(math.ceil(abs(yaw_delta) / max(max_yaw_step, EPS))),
        )
        for sample_index in range(count):
            t = sample_index / count
            raw = {
                "x": float(start["x"]) + dx * t,
                "y": float(start["y"]) + dy * t,
                "yaw": _normalize_angle(float(start["yaw"]) + yaw_delta * t),
            }
            target = targets[index % len(targets)]
            projected, extra, safe = _project_pose_inward_for_target(
                raw,
                target,
                safety_region,
                config,
                step_m=projection_step,
                max_extra_m=projection_max,
            )
            if extra > EPS:
                projection_count += 1
            if not safe:
                projection_failed_count += 1
            max_extra = max(max_extra, extra)
            _append_base_pose(dense, projected)
    if dense:
        _append_base_pose(dense, dict(dense[0]))
    return dense, {
        "densified_pose_count": len(dense),
        "densify_projection_count": projection_count,
        "densify_projection_failed_count": projection_failed_count,
        "densify_projection_max_extra_m": round(max_extra, 4),
    }


def _build_trajectory_outline_base_poses(
    points: list[tuple[float, float]],
    *,
    base_offset_m: float,
    safety_region: Polygon | MultiPolygon,
    boundary: Any,
    config: dict[str, Any],
    options: dict[str, Any],
    mode: str,
) -> tuple[list[dict[str, float]], dict[str, Any]]:
    targets = _sample_outline_trajectory_targets(
        points,
        base_offset_m=base_offset_m,
        options=options,
    )
    if len(targets) < 3:
        return [], {"status": "too_few_trajectory_targets", "corner_records": []}
    if mode == "footprint_cspace_shell":
        anchors, anchor_metadata = _cspace_shell_anchor_poses(
            targets,
            base_offset_m=base_offset_m,
            safety_region=safety_region,
            boundary=boundary,
            config=config,
            options=options,
        )
    else:
        anchors, anchor_metadata = _elastic_outline_anchor_poses(
            targets,
            safety_region=safety_region,
            config=config,
            options=options,
        )
    optimized, optimizer_metadata = _optimize_outline_pose_loop(
        targets,
        anchors,
        safety_region=safety_region,
        config=config,
        options=options,
    )
    dense, densify_metadata = _densify_outline_pose_loop(
        optimized,
        targets=targets,
        safety_region=safety_region,
        config=config,
        options=options,
    )
    unsafe_count = _corner_arc_unsafe_count(dense, safety_region, config)
    metadata = {
        "status": "ok" if unsafe_count == 0 else "warning",
        "corner_count": len(_outline_vertices(points)),
        "safe_turn_corner_count": 0,
        "safe_left_turn_corner_count": 0,
        "corner_arc_count": 0,
        "corner_unsafe_candidate_count": unsafe_count,
        "corner_local_fit_max_extra_offset_m": 0.0,
        "corner_records": [],
        "trajectory_solver": mode,
        "trajectory_target_count": len(targets),
        "trajectory_anchor_count": len(anchors),
        "trajectory_unsafe_sample_count": unsafe_count,
        "trajectory_yaw_window_m": round(float(options["outline_trajectory_yaw_window_m"]), 4),
        "trajectory_max_yaw_step_deg": round(float(options["outline_trajectory_max_yaw_step_deg"]), 4),
        **anchor_metadata,
        **optimizer_metadata,
        **densify_metadata,
    }
    return dense, metadata


def _outline_ring_records(centerline: Polygon | MultiPolygon) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for component_index, poly in enumerate(
        sorted(v2_geometry._iter_polygons(centerline), key=lambda item: item.area, reverse=True)
    ):
        exterior = _ring_coords(poly.exterior)
        if _path_length(exterior) > EPS:
            records.append(
                {
                    "component_index": component_index,
                    "ring_type": "exterior",
                    "ring_index": len(records),
                    "points": exterior,
                }
            )
        for interior_index, interior in enumerate(poly.interiors):
            points = _ring_coords(interior)
            if _path_length(points) <= EPS:
                continue
            records.append(
                {
                    "component_index": component_index,
                    "ring_type": "hole",
                    "hole_index": interior_index,
                    "ring_index": len(records),
                    "points": points,
                }
            )
    return records


def _build_footprint_side_outline_paths_for_area(
    area: dict[str, Any],
    conditioned: Polygon | MultiPolygon,
    config: dict[str, Any],
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    cutting_enabled = bool(options.get("outline_cutting_enabled", True))
    generation_mode = str(options.get("outline_generation_mode", "footprint_side_fit"))
    conditioned_boundary = conditioned.boundary
    safety_region = _outline_safety_region(conditioned, options)
    outline_paths: list[dict[str, Any]] = []

    for layer_index in range(int(options["outline_count"])):
        base_offset, clearance_source = _footprint_side_base_offset_m(
            layer_index=layer_index,
            config=config,
            options=options,
        )
        ring_records = _outline_ring_records(conditioned)
        if not ring_records:
            outline_paths.append(
                {
                    "outline_path_id": f"area-{area.get('area_index', 0)}-outline-{layer_index}-empty",
                    "area_index": int(area.get("area_index", 0)),
                    "is_outline": True,
                    "strategy": f"{generation_mode}_boundary_outline",
                    "status": "no_path",
                    "warnings": [
                        "conditioned lawn has no boundary ring for footprint-side outline generation"
                    ],
                    "layer_index": layer_index,
                    "centerline_offset_m": round(float(base_offset), 4),
                    "outline_base_link_offset_m": round(float(base_offset), 4),
                    "segments": [],
                    **_path_summary([]),
                }
            )
            continue

        for record in ring_records:
            ring_type = str(record.get("ring_type", "exterior"))
            oriented_points, orientation_metadata = _orient_outline_ring_for_right_boundary(
                record["points"], ring_type
            )
            points, outline_start_metadata = _select_outline_start(oriented_points, options)
            ring_index = int(record["ring_index"])
            if generation_mode in {"footprint_elastic_band", "footprint_cspace_shell"}:
                base_poses, corner_metadata = _build_trajectory_outline_base_poses(
                    points,
                    base_offset_m=base_offset,
                    safety_region=safety_region,
                    boundary=conditioned_boundary,
                    config=config,
                    options=options,
                    mode=generation_mode,
                )
            else:
                base_poses, corner_metadata = _build_footprint_side_base_poses(
                    points,
                    base_offset_m=base_offset,
                    safety_region=safety_region,
                    config=config,
                    options=options,
                )
            if not base_poses:
                outline_paths.append(
                    {
                        "outline_path_id": f"area-{area.get('area_index', 0)}-outline-{layer_index}-{ring_index}",
                        "area_index": int(area.get("area_index", 0)),
                        "is_outline": True,
                        "strategy": f"{generation_mode}_boundary_outline",
                        "status": "no_path",
                        "warnings": [
                            f"outline ring {ring_index} could not produce footprint-side base-link poses"
                        ],
                        "layer_index": layer_index,
                        "ring_index": ring_index,
                        "ring_type": record.get("ring_type", "exterior"),
                        "component_index": record.get("component_index", 0),
                        "centerline_offset_m": round(float(base_offset), 4),
                        "outline_base_link_offset_m": round(float(base_offset), 4),
                        "clearance_source": clearance_source,
                        "outline_generation_mode": generation_mode,
                        "outline_build_status": corner_metadata.get("status", "unknown"),
                        "segments": [],
                        **_path_summary([]),
                    }
                )
                continue
            segment_metadata = {
                "lane_index": ring_index,
                "ring_type": ring_type,
                "component_index": record.get("component_index", 0),
                "centerline_offset_m": round(float(base_offset), 4),
                "outline_base_link_offset_m": round(float(base_offset), 4),
                "outline_layer_index": layer_index,
                "outline_generation_mode": generation_mode,
                "outline_fit_attempt_index": 0,
                "outline_fit_extra_offset_m": 0.0,
                "outline_target_right_footprint_clearance_m": round(
                    float(options.get("outline_right_footprint_clearance_m", 0.0)), 4
                ),
                "outline_right_footprint_clearance_source": str(
                    options.get("outline_right_footprint_clearance_source", "")
                ),
                "outline_footprint_clearance_m": round(float(options.get("outline_footprint_clearance_m", 0.0)), 4),
                "outline_boundary_simplify_tolerance_m": round(
                    float(options.get("outline_boundary_simplify_tolerance_m", 0.0)), 4
                ),
                "outline_build_status": corner_metadata.get("status", "unknown"),
                "outline_corner_count": int(corner_metadata.get("corner_count", 0)),
                "outline_corner_arc_count": int(corner_metadata.get("corner_arc_count", 0)),
                "outline_safe_turn_corner_count": int(corner_metadata.get("safe_turn_corner_count", 0)),
                "outline_corner_unsafe_candidate_count": int(corner_metadata.get("corner_unsafe_candidate_count", 0)),
                "outline_corner_local_fit_max_extra_offset_m": corner_metadata.get(
                    "corner_local_fit_max_extra_offset_m", 0.0
                ),
                "outline_corner_records": corner_metadata.get("corner_records", []),
                "outline_trajectory_solver": corner_metadata.get("trajectory_solver"),
                "outline_trajectory_target_count": corner_metadata.get("trajectory_target_count"),
                "outline_trajectory_anchor_count": corner_metadata.get("trajectory_anchor_count"),
                "outline_trajectory_unsafe_sample_count": corner_metadata.get(
                    "trajectory_unsafe_sample_count"
                ),
                "outline_trajectory_yaw_window_m": corner_metadata.get("trajectory_yaw_window_m"),
                "outline_trajectory_max_yaw_step_deg": corner_metadata.get(
                    "trajectory_max_yaw_step_deg"
                ),
                "outline_anchor_projection_count": corner_metadata.get("anchor_projection_count"),
                "outline_anchor_projection_failed_count": corner_metadata.get(
                    "anchor_projection_failed_count"
                ),
                "outline_anchor_projection_max_extra_m": corner_metadata.get(
                    "anchor_projection_max_extra_m"
                ),
                "outline_optimizer_iteration_count": corner_metadata.get("optimizer_iteration_count"),
                "outline_optimizer_projection_count": corner_metadata.get("optimizer_projection_count"),
                "outline_optimizer_projection_failed_count": corner_metadata.get(
                    "optimizer_projection_failed_count"
                ),
                "outline_optimizer_projection_max_extra_m": corner_metadata.get(
                    "optimizer_projection_max_extra_m"
                ),
                "outline_densified_pose_count": corner_metadata.get("densified_pose_count"),
                "outline_densify_projection_count": corner_metadata.get("densify_projection_count"),
                "outline_densify_projection_failed_count": corner_metadata.get(
                    "densify_projection_failed_count"
                ),
                "outline_densify_projection_max_extra_m": corner_metadata.get(
                    "densify_projection_max_extra_m"
                ),
                "outline_cspace_target_count": corner_metadata.get("cspace_target_count"),
                "outline_cspace_candidate_count": corner_metadata.get("cspace_candidate_count"),
                "outline_cspace_failed_target_count": corner_metadata.get(
                    "cspace_failed_target_count"
                ),
                "outline_cspace_max_selected_extra_m": corner_metadata.get(
                    "cspace_max_selected_extra_m"
                ),
                "outline_cspace_max_selected_yaw_offset_deg": corner_metadata.get(
                    "cspace_max_selected_yaw_offset_deg"
                ),
                **orientation_metadata,
                **outline_start_metadata,
            }
            segment = _make_base_pose_segment(
                segment_id=f"area-{area.get('area_index', 0)}-outline-{layer_index}-{ring_index}-seg-0",
                phase="outline",
                direction="forward",
                cutting_enabled=cutting_enabled,
                base_poses=base_poses,
                drivable=safety_region,
                config=config,
                metadata=segment_metadata,
            )
            if segment.get("base_poses"):
                first_pose = segment["base_poses"][0]
                segment["outline_start_base_link_point"] = _round_point(
                    float(first_pose.get("x", 0.0)),
                    float(first_pose.get("y", 0.0)),
                )
                segment["outline_start_base_link_yaw"] = round(float(first_pose.get("yaw", 0.0)), 6)
            summary = _path_summary([segment])
            summary.update(_outline_footprint_side_clearance_stats(segment, conditioned_boundary, config))
            unsafe_count = int(summary["unsafe_sample_count"])
            warnings = []
            if unsafe_count:
                warnings.append(
                    f"{unsafe_count} outline footprint samples leave the configured outline safety region; this output is diagnostic, not live-ready"
                )
            outline_paths.append(
                {
                    "outline_path_id": f"area-{area.get('area_index', 0)}-outline-{layer_index}-{ring_index}",
                    "area_index": int(area.get("area_index", 0)),
                    "is_outline": True,
                    "strategy": f"{generation_mode}_boundary_outline",
                    "status": "warning" if unsafe_count else "ok",
                    "warnings": warnings,
                    "layer_index": layer_index,
                    "ring_index": ring_index,
                    "ring_type": record.get("ring_type", "exterior"),
                    "component_index": record.get("component_index", 0),
                    "centerline_offset_m": round(float(base_offset), 4),
                    "outline_base_link_offset_m": round(float(base_offset), 4),
                    "clearance_source": clearance_source,
                    "outline_generation_mode": generation_mode,
                    "outline_target_right_footprint_clearance_m": round(
                        float(options.get("outline_right_footprint_clearance_m", 0.0)), 4
                    ),
                    "outline_right_footprint_clearance_source": str(
                        options.get("outline_right_footprint_clearance_source", "")
                    ),
                    "outline_fit_attempt_index": 0,
                    "outline_fit_extra_offset_m": 0.0,
                    "outline_footprint_clearance_m": round(float(options.get("outline_footprint_clearance_m", 0.0)), 4),
                    "outline_boundary_simplify_tolerance_m": round(
                        float(options.get("outline_boundary_simplify_tolerance_m", 0.0)), 4
                    ),
                    "outline_build_status": segment.get("outline_build_status"),
                    "outline_boundary_side": segment.get("outline_boundary_side"),
                    "outline_ring_orientation": segment.get("outline_ring_orientation"),
                    "outline_ring_was_reversed": bool(segment.get("outline_ring_was_reversed", False)),
                    "outline_start_selector": segment.get("outline_start_selector"),
                    "outline_start_reason": segment.get("outline_start_reason"),
                    "outline_start_distance_m": segment.get("outline_start_distance_m"),
                    "outline_start_straight_run_m": segment.get("outline_start_straight_run_m"),
                    "outline_start_run_segment_count": segment.get("outline_start_run_segment_count"),
                    "outline_start_point": segment.get("outline_start_point"),
                    "outline_start_yaw": segment.get("outline_start_yaw"),
                    "outline_start_base_link_point": segment.get("outline_start_base_link_point"),
                    "outline_start_base_link_yaw": segment.get("outline_start_base_link_yaw"),
                    "outline_start_angle_tolerance_deg": segment.get("outline_start_angle_tolerance_deg"),
                    "outline_start_candidate_count": segment.get("outline_start_candidate_count"),
                    "outline_corner_count": segment.get("outline_corner_count"),
                    "outline_corner_arc_count": segment.get("outline_corner_arc_count"),
                    "outline_safe_turn_corner_count": segment.get("outline_safe_turn_corner_count"),
                    "outline_corner_unsafe_candidate_count": segment.get("outline_corner_unsafe_candidate_count"),
                    "outline_corner_local_fit_max_extra_offset_m": segment.get(
                        "outline_corner_local_fit_max_extra_offset_m"
                    ),
                    "outline_corner_records": segment.get("outline_corner_records", []),
                    "outline_trajectory_solver": segment.get("outline_trajectory_solver"),
                    "outline_trajectory_target_count": segment.get("outline_trajectory_target_count"),
                    "outline_trajectory_anchor_count": segment.get("outline_trajectory_anchor_count"),
                    "outline_trajectory_unsafe_sample_count": segment.get(
                        "outline_trajectory_unsafe_sample_count"
                    ),
                    "outline_trajectory_yaw_window_m": segment.get("outline_trajectory_yaw_window_m"),
                    "outline_trajectory_max_yaw_step_deg": segment.get(
                        "outline_trajectory_max_yaw_step_deg"
                    ),
                    "outline_anchor_projection_count": segment.get("outline_anchor_projection_count"),
                    "outline_anchor_projection_failed_count": segment.get(
                        "outline_anchor_projection_failed_count"
                    ),
                    "outline_anchor_projection_max_extra_m": segment.get(
                        "outline_anchor_projection_max_extra_m"
                    ),
                    "outline_optimizer_iteration_count": segment.get(
                        "outline_optimizer_iteration_count"
                    ),
                    "outline_optimizer_projection_count": segment.get(
                        "outline_optimizer_projection_count"
                    ),
                    "outline_optimizer_projection_failed_count": segment.get(
                        "outline_optimizer_projection_failed_count"
                    ),
                    "outline_optimizer_projection_max_extra_m": segment.get(
                        "outline_optimizer_projection_max_extra_m"
                    ),
                    "outline_densified_pose_count": segment.get("outline_densified_pose_count"),
                    "outline_densify_projection_count": segment.get("outline_densify_projection_count"),
                    "outline_densify_projection_failed_count": segment.get(
                        "outline_densify_projection_failed_count"
                    ),
                    "outline_densify_projection_max_extra_m": segment.get(
                        "outline_densify_projection_max_extra_m"
                    ),
                    "outline_cspace_target_count": segment.get("outline_cspace_target_count"),
                    "outline_cspace_candidate_count": segment.get("outline_cspace_candidate_count"),
                    "outline_cspace_failed_target_count": segment.get(
                        "outline_cspace_failed_target_count"
                    ),
                    "outline_cspace_max_selected_extra_m": segment.get(
                        "outline_cspace_max_selected_extra_m"
                    ),
                    "outline_cspace_max_selected_yaw_offset_deg": segment.get(
                        "outline_cspace_max_selected_yaw_offset_deg"
                    ),
                    "coverage_region": v2_geometry.geometry_to_json(conditioned),
                    "segments": [segment],
                    **summary,
                }
            )
    return outline_paths


def build_outline_paths_for_area(
    area: dict[str, Any],
    config: dict[str, Any],
    options: dict[str, Any],
) -> list[dict[str, Any]]:
    if not bool(options.get("outline_enabled", False)) or int(options.get("outline_count", 0)) <= 0:
        return []
    conditioned = v2_geometry._geometry_from_json(area.get("geometries", {}).get("conditioned_lawn", {}))
    if conditioned.is_empty:
        return []
    generation_mode = str(options.get("outline_generation_mode", "footprint_disk"))
    if generation_mode in {"footprint_side_fit", "footprint_elastic_band", "footprint_cspace_shell"}:
        return _build_footprint_side_outline_paths_for_area(area, conditioned, config, options)
    tool_width = _tool_width(config)
    drivable_clearance = max(0.0, _float_config(config, "v2_drivable_boundary_clearance_m", 0.0))
    half_width = _mower_width(config) * 0.5
    sample_step = float(options["outline_sample_step_m"])
    cutting_enabled = bool(options.get("outline_cutting_enabled", True))
    safety_region = _outline_safety_region(conditioned, options)
    conditioned_boundary = conditioned.boundary
    outline_paths: list[dict[str, Any]] = []

    for layer_index in range(int(options["outline_count"])):
        initial_offset, clearance_source = _outline_layer_initial_offset(
            layer_index=layer_index,
            tool_width=tool_width,
            config=config,
            options=options,
        )
        fit_step = float(options.get("outline_fit_offset_step_m", 0.05))
        fit_max_extra = float(options.get("outline_fit_max_extra_m", 0.0))
        max_attempts = (
            max(1, int(math.floor(fit_max_extra / fit_step)) + 1)
            if generation_mode == "right_edge_fit"
            else 1
        )
        accepted: tuple[float, Polygon | MultiPolygon, list[tuple[dict[str, Any], dict[str, Any], dict[str, Any]]], int, int] | None = None
        last_empty_offset = initial_offset
        for attempt_index in range(max_attempts):
            centerline_offset = initial_offset + attempt_index * fit_step
            centerline = v2_geometry._clean_polygonal(
                conditioned.buffer(-centerline_offset, join_style="round", quad_segs=16)
            )
            last_empty_offset = centerline_offset
            if centerline.is_empty:
                break
            candidate_paths: list[tuple[dict[str, Any], dict[str, Any], dict[str, Any]]] = []
            unsafe_total = 0
            for record in _outline_ring_records(centerline):
                ring_type = str(record.get("ring_type", "exterior"))
                if generation_mode == "right_edge_fit":
                    oriented_points, orientation_metadata = _orient_outline_ring_for_right_boundary(
                        record["points"], ring_type
                    )
                else:
                    oriented_points = record["points"]
                    orientation_metadata = {}
                points, outline_start_metadata = _select_outline_start(oriented_points, options)
                ring_index = int(record["ring_index"])
                segment_metadata = {
                    "lane_index": ring_index,
                    "ring_type": ring_type,
                    "component_index": record.get("component_index", 0),
                    "centerline_offset_m": round(float(centerline_offset), 4),
                    "outline_layer_index": layer_index,
                    "outline_yaw_window_m": round(float(options["outline_yaw_window_m"]), 4),
                    "outline_generation_mode": generation_mode,
                    "outline_fit_attempt_index": attempt_index,
                    "outline_fit_extra_offset_m": round(float(centerline_offset - initial_offset), 4),
                    "outline_footprint_clearance_m": round(float(options.get("outline_footprint_clearance_m", 0.0)), 4),
                    "outline_target_right_edge_clearance_m": round(float(options.get("outline_right_edge_clearance_m", 0.0)), 4),
                    **orientation_metadata,
                    **outline_start_metadata,
                }
                segment = _make_polyline_segment(
                    segment_id=f"area-{area.get('area_index', 0)}-outline-{layer_index}-{ring_index}-seg-0",
                    phase="outline",
                    direction="forward",
                    cutting_enabled=cutting_enabled,
                    points=points,
                    safety_region=safety_region,
                    config=config,
                    options=options,
                    sample_step_m=sample_step,
                    metadata=segment_metadata,
                )
                unsafe_total += len(segment.get("unsafe_samples", []))
                summary = _path_summary([segment])
                summary.update(_outline_right_edge_clearance_stats(segment, conditioned_boundary, config))
                candidate_paths.append((record, segment, summary))
            accepted = (centerline_offset, centerline, candidate_paths, unsafe_total, attempt_index)
            if generation_mode != "right_edge_fit" or unsafe_total == 0:
                break

        if accepted is None:
            outline_paths.append(
                {
                    "outline_path_id": f"area-{area.get('area_index', 0)}-outline-{layer_index}-empty",
                    "area_index": int(area.get("area_index", 0)),
                    "is_outline": True,
                    "strategy": f"{generation_mode}_boundary_outline",
                    "status": "no_path",
                    "warnings": [
                        f"outline centerline offset {last_empty_offset:.2f} m leaves no serviceable ring"
                    ],
                    "layer_index": layer_index,
                    "centerline_offset_m": round(float(last_empty_offset), 4),
                    "segments": [],
                    **_path_summary([]),
                }
            )
            continue
        centerline_offset, centerline, candidate_paths, _unsafe_total, attempt_index = accepted
        for record, segment, summary in candidate_paths:
            ring_index = int(record["ring_index"])
            unsafe_count = int(summary["unsafe_sample_count"])
            warnings = []
            if unsafe_count:
                warnings.append(
                    f"{unsafe_count} outline footprint samples are outside the configured outline safety region"
                )
            outline_paths.append(
                {
                    "outline_path_id": f"area-{area.get('area_index', 0)}-outline-{layer_index}-{ring_index}",
                    "area_index": int(area.get("area_index", 0)),
                    "is_outline": True,
                    "strategy": f"{generation_mode}_boundary_outline",
                    "status": "warning" if unsafe_count else "ok",
                    "warnings": warnings,
                    "layer_index": layer_index,
                    "ring_index": ring_index,
                    "ring_type": record.get("ring_type", "exterior"),
                    "component_index": record.get("component_index", 0),
                    "centerline_offset_m": round(float(centerline_offset), 4),
                    "clearance_source": clearance_source,
                    "drivable_boundary_clearance_m": round(float(drivable_clearance), 4),
                    "mower_half_width_m": round(float(half_width), 4),
                    "outline_generation_mode": generation_mode,
                    "outline_target_right_edge_clearance_m": round(float(options.get("outline_right_edge_clearance_m", 0.0)), 4),
                    "outline_right_edge_clearance_source": str(options.get("outline_right_edge_clearance_source", "")),
                    "outline_fit_attempt_index": attempt_index,
                    "outline_fit_extra_offset_m": round(float(centerline_offset - initial_offset), 4),
                    "outline_footprint_clearance_m": round(float(options.get("outline_footprint_clearance_m", 0.0)), 4),
                    "outline_boundary_side": segment.get("outline_boundary_side"),
                    "outline_ring_orientation": segment.get("outline_ring_orientation"),
                    "outline_ring_was_reversed": bool(segment.get("outline_ring_was_reversed", False)),
                    "outline_start_selector": segment.get("outline_start_selector"),
                    "outline_start_reason": segment.get("outline_start_reason"),
                    "outline_start_distance_m": segment.get("outline_start_distance_m"),
                    "outline_start_straight_run_m": segment.get("outline_start_straight_run_m"),
                    "outline_start_run_segment_count": segment.get("outline_start_run_segment_count"),
                    "outline_start_point": segment.get("outline_start_point"),
                    "outline_start_yaw": segment.get("outline_start_yaw"),
                    "outline_start_angle_tolerance_deg": segment.get("outline_start_angle_tolerance_deg"),
                    "outline_start_candidate_count": segment.get("outline_start_candidate_count"),
                    "coverage_region": v2_geometry.geometry_to_json(centerline),
                    "segments": [segment],
                    **summary,
                }
            )
    return outline_paths


def _summarize_outline_paths(outline_paths: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "total": len(outline_paths),
        "path_length_m": round(sum(float(path.get("path_length_m", 0.0)) for path in outline_paths), 4),
        "forward_length_m": round(sum(float(path.get("forward_length_m", 0.0)) for path in outline_paths), 4),
        "cutting_length_m": round(sum(float(path.get("cutting_length_m", 0.0)) for path in outline_paths), 4),
        "unsafe_sample_count": sum(int(path.get("unsafe_sample_count", 0)) for path in outline_paths),
        "warning_count": sum(len(path.get("warnings", [])) for path in outline_paths),
    }


def _summarize_task_paths(task_paths: list[dict[str, Any]]) -> dict[str, Any]:
    by_type: dict[str, int] = {}
    by_exit_mode: dict[str, int] = {}
    axis_candidates_total = 0
    selected_axis_by_source: dict[str, int] = {}
    region_area = 0.0
    covered_area = 0.0
    warning_count = 0
    forward_length = 0.0
    reverse_length = 0.0
    unsafe_count = 0
    body_pocket_count = 0
    body_pocket_area = 0.0
    compact_connector_count = 0
    compact_connector_length = 0.0
    compact_connector_rejected = 0
    compact_connector_unsafe_rejected = 0
    compact_connector_too_far_rejected = 0
    for path in task_paths:
        task_type = str(path.get("task_type", "unknown"))
        by_type[task_type] = by_type.get(task_type, 0) + 1
        exit_mode = str(path.get("exit_mode", "unknown"))
        by_exit_mode[exit_mode] = by_exit_mode.get(exit_mode, 0) + 1
        candidates = path.get("axis_candidates", [])
        axis_candidates_total += len(candidates)
        selected_axis = path.get("service_axis") or {}
        axis_source = str(selected_axis.get("source", "none"))
        selected_axis_by_source[axis_source] = selected_axis_by_source.get(axis_source, 0) + 1
        region_area += float(path.get("coverage_region_area_m2", 0.0))
        covered_area += float(path.get("estimated_covered_area_m2", 0.0))
        warning_count += len(path.get("warnings", []))
        forward_length += float(path.get("forward_length_m", 0.0))
        reverse_length += float(path.get("reverse_length_m", 0.0))
        unsafe_count += int(path.get("unsafe_sample_count", 0))
        body_pocket_count += int(path.get("body_service_pocket_count", 0))
        body_pocket_area += float(path.get("body_service_pocket_area_m2", 0.0))
        connector_summary = path.get("compact_connector_summary") or {}
        compact_connector_count += int(connector_summary.get("inserted_count", path.get("compact_connector_count", 0)))
        compact_connector_length += float(connector_summary.get("length_m", path.get("compact_connector_length_m", 0.0)))
        compact_connector_rejected += int(connector_summary.get("rejected_count", 0))
        compact_connector_unsafe_rejected += int(connector_summary.get("unsafe_rejected_count", 0))
        compact_connector_too_far_rejected += int(connector_summary.get("too_far_rejected_count", 0))
    return {
        "total": len(task_paths),
        "by_task_type": by_type,
        "by_exit_mode": by_exit_mode,
        "estimated_coverage_percent": round(100.0 * covered_area / region_area, 2) if region_area > EPS else 0.0,
        "coverage_region_area_m2": round(region_area, 4),
        "estimated_covered_area_m2": round(covered_area, 4),
        "forward_length_m": round(forward_length, 4),
        "reverse_length_m": round(reverse_length, 4),
        "unsafe_sample_count": unsafe_count,
        "warning_count": warning_count,
        "axis_candidates_total": axis_candidates_total,
        "selected_axis_by_source": selected_axis_by_source,
        "body_service_pocket_count": body_pocket_count,
        "body_service_pocket_area_m2": round(body_pocket_area, 4),
        "compact_connector_count": compact_connector_count,
        "compact_connector_length_m": round(compact_connector_length, 4),
        "compact_connector_rejected_count": compact_connector_rejected,
        "compact_connector_unsafe_rejected_count": compact_connector_unsafe_rejected,
        "compact_connector_too_far_rejected_count": compact_connector_too_far_rejected,
    }


def build_task_paths_for_area(
    area: dict[str, Any],
    config: dict[str, Any],
    options: dict[str, Any] | None = None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    path_options = task_path_options(config)
    coverage_regions = _coverage_regions_for_area(area, config, path_options)
    if not path_options["enabled"]:
        return coverage_regions, [], _summarize_task_paths([])
    drivable = v2_geometry._geometry_from_json(area.get("geometries", {}).get("drivable_region", {}))
    portal_by_id = {str(portal.get("id", "")): portal for portal in area.get("portal_candidates", [])}
    task_by_id = {str(task.get("id", "")): task for task in area.get("task_proposals", [])}
    task_paths = []
    for region_record in coverage_regions:
        task = task_by_id.get(str(region_record.get("task_id", "")))
        if not task:
            continue
        region = v2_geometry._geometry_from_json(region_record.get("geometry", {}))
        if region.is_empty:
            continue
        task_paths.append(
            _build_task_path(
                area_index=int(area.get("area_index", 0)),
                area=area,
                task=task,
                coverage_region=region,
                drivable=drivable,
                portal_by_id=portal_by_id,
                config=config,
                options=path_options,
            )
        )
    return coverage_regions, task_paths, _summarize_task_paths(task_paths)


def apply_task_paths(result: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    """Attach M2 diagnostic task paths to ``result`` and return standalone JSON."""
    areas_out = []
    path_options = task_path_options(config)
    for area in result.get("areas", []):
        coverage_regions, task_paths, summary = build_task_paths_for_area(area, config, result.get("conditioning", {}))
        outline_paths = build_outline_paths_for_area(area, config, path_options)
        area["task_coverage_regions"] = coverage_regions
        area["outline_paths"] = outline_paths
        area["outline_path_summary"] = _summarize_outline_paths(outline_paths)
        area["task_paths"] = task_paths
        area["task_path_summary"] = summary
        areas_out.append(
            {
                "area_index": area.get("area_index", 0),
                "source_id": area.get("source_id", ""),
                "source_name": area.get("source_name", ""),
                "task_coverage_regions": coverage_regions,
                "outline_paths": outline_paths,
                "outline_path_summary": area["outline_path_summary"],
                "task_paths": task_paths,
                "task_path_summary": summary,
                "ownership_adjustments": area.get("ownership_adjustments", []),
            }
        )
    all_paths = [path for area in areas_out for path in area.get("task_paths", [])]
    all_outline_paths = [path for area in areas_out for path in area.get("outline_paths", [])]
    return {
        "schema": "open_mower.coverage_lab.v2_task_paths.v0",
        "frame_id": result.get("frame_id", "map"),
        "source_map": result.get("source_map", ""),
        "mower_model": result.get("mower_model", {}),
        "areas": areas_out,
        "outline_summary": _summarize_outline_paths(all_outline_paths),
        "summary": _summarize_task_paths(all_paths),
    }
