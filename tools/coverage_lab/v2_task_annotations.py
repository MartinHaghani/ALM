"""Operator task-boundary annotations for coverage-planner V2.

This module stays lab-only. It converts static-report annotations into the
same task proposal shape used by M1/M2 diagnostics.
"""
from __future__ import annotations

import math
from typing import Any

from shapely.geometry import LineString, MultiPolygon, Point, Polygon

import v2_geometry


EPS = 1e-9
SCHEMA = "open_mower.coverage_lab.v2_task_annotations.v0"
VALID_TASK_TYPES = {"corridor", "dead_end_corridor", "notch"}


def _float_config(config: dict[str, Any], key: str, default: float) -> float:
    try:
        return float(config.get(key, default))
    except (TypeError, ValueError):
        return default


def _point(raw: Any) -> tuple[float, float] | None:
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return (float(raw[0]), float(raw[1]))
    if isinstance(raw, dict) and "x" in raw and "y" in raw:
        return (float(raw["x"]), float(raw["y"]))
    return None


def _annotation_list(payload: Any) -> list[dict[str, Any]]:
    if payload is None:
        return []
    if isinstance(payload, list):
        return [item for item in payload if isinstance(item, dict)]
    if isinstance(payload, dict):
        annotations = payload.get("annotations", [])
        if isinstance(annotations, list):
            return [item for item in annotations if isinstance(item, dict)]
    return []


def _polygon_containing_point(polygons: list[Any], point: Point) -> Any | None:
    containing = [poly for poly in polygons if poly.buffer(EPS).covers(point)]
    if containing:
        return max(containing, key=lambda poly: poly.area)
    if not polygons:
        return None
    return min(polygons, key=lambda poly: poly.distance(point))


def _line_width(line: LineString) -> float:
    return max(float(line.length), EPS)


def _half_plane_for_cut(drivable: Any, cut_line: LineString, service_point: Point) -> tuple[Any, LineString]:
    coords = list(cut_line.coords)
    if len(coords) < 2:
        return MultiPolygon([]), cut_line

    start = coords[0]
    end = coords[-1]
    dx = float(end[0] - start[0])
    dy = float(end[1] - start[1])
    length = math.hypot(dx, dy)
    if length <= EPS:
        return MultiPolygon([]), cut_line

    ux = dx / length
    uy = dy / length
    nx = -uy
    ny = ux
    midx = (float(start[0]) + float(end[0])) * 0.5
    midy = (float(start[1]) + float(end[1])) * 0.5
    side = (float(service_point.x) - midx) * nx + (float(service_point.y) - midy) * ny
    if side < 0.0:
        nx *= -1.0
        ny *= -1.0

    minx, miny, maxx, maxy = drivable.bounds
    diagonal = math.hypot(maxx - minx, maxy - miny)
    extent = max(diagonal * 3.0, length * 4.0, 5.0)
    line_start = (midx - ux * extent, midy - uy * extent)
    line_end = (midx + ux * extent, midy + uy * extent)
    far_end = (line_end[0] + nx * extent, line_end[1] + ny * extent)
    far_start = (line_start[0] + nx * extent, line_start[1] + ny * extent)
    half_plane = Polygon([line_start, line_end, far_end, far_start])
    if not half_plane.buffer(EPS).covers(service_point):
        half_plane = Polygon([line_start, line_end, (line_end[0] - nx * extent, line_end[1] - ny * extent), (line_start[0] - nx * extent, line_start[1] - ny * extent)])

    service_side = v2_geometry._clean_polygonal(drivable.intersection(half_plane))
    extended_line = LineString([line_start, line_end])
    return service_side, extended_line


def _body_core_geometry(area: dict[str, Any]) -> Any:
    body_geoms = [
        v2_geometry._geometry_from_json(proposal.get("geometry", {}))
        for proposal in area.get("task_proposals", [])
        if proposal.get("task_type") == "body_core"
    ]
    if not body_geoms:
        return MultiPolygon([])
    return v2_geometry._clean_polygonal(v2_geometry.unary_union(body_geoms))


def _refresh_area_summaries(area: dict[str, Any]) -> None:
    task_by_type: dict[str, int] = {}
    visible_count = 0
    hidden_artifacts = 0
    terminal_caps = 0
    for proposal in area.get("task_proposals", []):
        task_type = str(proposal.get("task_type", "unknown"))
        task_by_type[task_type] = task_by_type.get(task_type, 0) + 1
        if proposal.get("visible_by_default", True):
            visible_count += 1
        elif task_type == "artifact":
            hidden_artifacts += 1
        if proposal.get("terminal_cap"):
            terminal_caps += 1

    portal_by_status: dict[str, int] = {}
    portal_by_reason: dict[str, int] = {}
    accepted_visible = 0
    for portal in area.get("portal_candidates", []):
        status = str(portal.get("status", "unknown"))
        reason = str(portal.get("reason", "unknown"))
        portal_by_status[status] = portal_by_status.get(status, 0) + 1
        portal_by_reason[reason] = portal_by_reason.get(reason, 0) + 1
        if status == "accepted" and portal.get("display_label"):
            accepted_visible += 1

    area["task_proposals_summary"] = {
        "total": len(area.get("task_proposals", [])),
        "by_type": task_by_type,
        "visible_by_default": visible_count,
        "hidden_artifacts": hidden_artifacts,
        "terminal_caps": terminal_caps,
    }
    area["portal_candidates_summary"] = {
        "total": len(area.get("portal_candidates", [])),
        "accepted": portal_by_status.get("accepted", 0),
        "rejected": portal_by_status.get("rejected", 0),
        "accepted_visible": accepted_visible,
        "by_status": portal_by_status,
        "by_reason": portal_by_reason,
    }


def _annotation_task_record(
    *,
    annotation: dict[str, Any],
    annotation_index: int,
    area: dict[str, Any],
    service_region: Any,
    portal: dict[str, Any],
    options: dict[str, Any],
) -> dict[str, Any]:
    task_type = str(annotation.get("task_type", "dead_end_corridor"))
    if task_type not in VALID_TASK_TYPES:
        task_type = "dead_end_corridor"
    area_index = int(area.get("area_index", 0))
    task_id = f"area-{area_index}-annotation-task-{annotation_index}"
    body_core = _body_core_geometry(area)
    measurements = v2_geometry._service_measurements(service_region, portal)
    terminal_cap = v2_geometry._terminal_cap_for_region(
        task_type=task_type,
        region=service_region,
        portal=portal,
        measurements=measurements,
        options=options,
    )
    reason = "operator task-boundary annotation"
    evidence = {
        "source": "operator_annotation",
        "sources": [{"id": str(annotation.get("id", task_id)), "source": "operator_annotation"}],
        "service_area_m2": v2_geometry.area_m2(service_region),
        "service_depth_m": measurements["service_depth_m"],
        "service_width_m": measurements["service_width_m"],
        "portal_count": 1,
        "body_overlap_ratio": round(
            v2_geometry.area_m2(service_region.intersection(body_core)) / max(v2_geometry.area_m2(service_region), EPS),
            4,
        ),
        "reason": reason,
    }
    proposal = {
        "id": task_id,
        "area_index": area_index,
        "task_type": task_type,
        "confidence": 1.0,
        "geometry": v2_geometry.geometry_to_json(service_region),
        "seed_geometry": v2_geometry.geometry_to_json(service_region),
        "line_segments": portal.get("line_segments", []),
        "entry_portals": [portal["id"]],
        "source_ids": [str(annotation.get("id", task_id))],
        "extent_source": "operator_annotation",
        "service_depth_m": measurements["service_depth_m"],
        "service_width_m": measurements["service_width_m"],
        "classification_reason": reason,
        "evidence": evidence,
        "operator_annotation_id": str(annotation.get("id", task_id)),
        "visible_by_default": True,
    }
    if annotation.get("label"):
        proposal["display_label"] = str(annotation["label"])
    if annotation.get("note"):
        proposal["note"] = str(annotation["note"])
    if terminal_cap:
        proposal["terminal_cap"] = terminal_cap
    return proposal


def apply_task_annotations(
    result: dict[str, Any],
    annotations_payload: Any,
    config: dict[str, Any],
) -> dict[str, Any]:
    annotations = _annotation_list(annotations_payload)
    cut_buffer = max(EPS, _float_config(config, "v2_task_annotation_cut_buffer_m", 0.03))
    merge_threshold = float(result.get("conditioning", {}).get("task_region_merge_overlap_ratio", 0.45))
    by_area: dict[int, list[dict[str, Any]]] = {}
    for index, annotation in enumerate(annotations):
        annotation.setdefault("id", f"annotation-{index + 1}")
        area_index = int(annotation.get("area_index", 0))
        by_area.setdefault(area_index, []).append(annotation)

    applied_records: list[dict[str, Any]] = []
    for area in result.get("areas", []):
        area_index = int(area.get("area_index", 0))
        area_annotations = by_area.get(area_index, [])
        area["operator_annotations"] = []
        if not area_annotations:
            continue

        drivable = v2_geometry._geometry_from_json(area.get("geometries", {}).get("drivable_region", {}))
        if drivable.is_empty:
            area.setdefault("warnings", []).append("task annotations ignored because drivable region is empty")
            continue

        new_portals = list(area.get("portal_candidates", []))
        annotation_proposals: list[dict[str, Any]] = []
        annotation_regions: list[Any] = []
        for annotation_index, annotation in enumerate(area_annotations, start=1):
            cut_line_raw = annotation.get("cut_line", [])
            if not isinstance(cut_line_raw, list) or len(cut_line_raw) < 2:
                area["operator_annotations"].append(
                    {**annotation, "status": "rejected", "reason": "cut_line must contain two points"}
                )
                continue
            start = _point(cut_line_raw[0])
            end = _point(cut_line_raw[1])
            service_point = _point(annotation.get("service_point"))
            if start is None or end is None or service_point is None:
                area["operator_annotations"].append(
                    {**annotation, "status": "rejected", "reason": "annotation points are incomplete"}
                )
                continue
            cut_line = LineString([start, end])
            service_point_geom = Point(*service_point)
            half_plane_region, extended_cut_line = _half_plane_for_cut(drivable, cut_line, service_point_geom)
            component = _polygon_containing_point(v2_geometry._iter_polygons(half_plane_region), service_point_geom)
            cut_mode = "service_side_half_plane"
            if component is None or component.is_empty:
                cut_band = cut_line.buffer(cut_buffer, cap_style=2, join_style=2)
                split_region = v2_geometry._clean_polygonal(drivable.difference(cut_band))
                component = _polygon_containing_point(v2_geometry._iter_polygons(split_region), service_point_geom)
                extended_cut_line = cut_line
                cut_mode = "buffered_segment_fallback"
            if component is None or component.is_empty:
                area["operator_annotations"].append(
                    {**annotation, "status": "rejected", "reason": "cut did not leave a service-side component"}
                )
                continue
            service_region = v2_geometry._clean_polygonal(component.intersection(drivable))
            cut_band = extended_cut_line.buffer(cut_buffer, cap_style=2, join_style=2)
            clipped_line = extended_cut_line.intersection(drivable)
            center = cut_line.interpolate(0.5, normalized=True)
            portal_id = f"area-{area_index}-annotation-portal-{annotation_index}"
            portal = {
                "id": portal_id,
                "area_index": area_index,
                "component_index": 0,
                "branch_id": str(annotation.get("id", portal_id)),
                "endpoint_index": 0,
                "status": "accepted",
                "reason": "operator task-boundary annotation",
                "width_m": round(_line_width(cut_line), 4),
                "clearance_m": round(_line_width(cut_line) * 0.5, 4),
                "center": v2_geometry._round_point(float(center.x), float(center.y)),
                "line_segments": v2_geometry._line_segments_to_json(clipped_line if not clipped_line.is_empty else cut_line),
                "operator_annotation_id": str(annotation.get("id", portal_id)),
                "cut_mode": cut_mode,
            }
            proposal = _annotation_task_record(
                annotation=annotation,
                annotation_index=annotation_index,
                area=area,
                service_region=service_region,
                portal=portal,
                options=result.get("conditioning", {}),
            )
            new_portals.append(portal)
            annotation_proposals.append(proposal)
            annotation_regions.append(service_region)
            record = {
                **annotation,
                "status": "accepted",
                "reason": "operator annotation applied",
                "geometry": v2_geometry.geometry_to_json(service_region),
                "cut_band": v2_geometry.geometry_to_json(cut_band.intersection(drivable)),
                "portal_id": portal_id,
                "task_id": proposal["id"],
                "cut_mode": cut_mode,
            }
            area["operator_annotations"].append(record)
            applied_records.append(record)

        if not annotation_proposals:
            continue

        kept: list[dict[str, Any]] = []
        absorbed_sources: dict[str, list[str]] = {proposal["id"]: [] for proposal in annotation_proposals}
        absorbed_line_segments: dict[str, list[Any]] = {proposal["id"]: [] for proposal in annotation_proposals}
        for proposal in area.get("task_proposals", []):
            if proposal.get("task_type") == "body_core":
                kept.append(proposal)
                continue
            geom = v2_geometry._geometry_from_json(proposal.get("geometry", {}))
            absorbed = False
            for annotation_proposal, annotation_region in zip(annotation_proposals, annotation_regions):
                overlap = v2_geometry.area_m2(geom.intersection(annotation_region)) / max(
                    min(v2_geometry.area_m2(geom), v2_geometry.area_m2(annotation_region)), EPS
                )
                if overlap >= merge_threshold:
                    absorbed_sources[annotation_proposal["id"]].extend(str(item) for item in proposal.get("source_ids", []))
                    absorbed_line_segments[annotation_proposal["id"]].extend(proposal.get("source_line_segments", []))
                    absorbed_line_segments[annotation_proposal["id"]].extend(proposal.get("line_segments", []))
                    absorbed = True
                    break
            if not absorbed:
                kept.append(proposal)

        for proposal in annotation_proposals:
            merged_sources = v2_geometry._unique_list(proposal.get("source_ids", []) + absorbed_sources[proposal["id"]])
            proposal["source_ids"] = merged_sources
            source_lines = v2_geometry._unique_list(absorbed_line_segments[proposal["id"]])
            if source_lines:
                proposal["source_line_segments"] = source_lines
            proposal["evidence"]["absorbed_source_ids"] = absorbed_sources[proposal["id"]]
            proposal["evidence"]["absorbed_line_segment_count"] = len(source_lines)
        area["task_proposals"] = kept + annotation_proposals
        area["portal_candidates"] = new_portals
        v2_geometry._decorate_task_display_metadata(area["task_proposals"], area["portal_candidates"])
        _refresh_area_summaries(area)

    result["task_annotations"] = {
        "schema": SCHEMA,
        "total": len(annotations),
        "accepted": len(applied_records),
        "rejected": len(annotations) - len(applied_records),
        "annotations": applied_records,
    }
    if annotations:
        result.setdefault("warnings", []).append(
            f"applied {len(applied_records)} of {len(annotations)} operator task annotations"
        )
    return result
