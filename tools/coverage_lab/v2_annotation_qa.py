"""Operator-vs-automatic task evidence QA for coverage-planner V2.

This module is lab-only. It compares the task proposals produced by the
automatic M1/M2 evidence pipeline against operator task-boundary annotations.
The output is diagnostic: it tells us which automatic regions matched the
manual mouth cuts, which were missed, and which were the wrong type or extent.
"""
from __future__ import annotations

import math
from typing import Any

from shapely.geometry import MultiPolygon

import v2_geometry


EPS = 1e-9
SCHEMA = "open_mower.coverage_lab.v2_annotation_qa.v0"


def _float_config(config: dict[str, Any], key: str, default: float) -> float:
    try:
        return float(config.get(key, default))
    except (TypeError, ValueError):
        return default


def _round(value: float | None, digits: int = 4) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


def _point(raw: Any) -> tuple[float, float] | None:
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        return (float(raw[0]), float(raw[1]))
    return None


def _area(geom: Any) -> float:
    return v2_geometry.area_m2(geom)


def _proposal_geometry(proposal: dict[str, Any]) -> Any:
    return v2_geometry._geometry_from_json(proposal.get("geometry", {}))


def _proposal_area(proposal: dict[str, Any]) -> float:
    return _area(_proposal_geometry(proposal))


def _proposal_label(proposal: dict[str, Any]) -> str:
    label = proposal.get("display_label")
    if label:
        return str(label)
    task_type = str(proposal.get("task_type", ""))
    return v2_geometry.TASK_TYPE_LABELS.get(task_type, task_type.replace("_", " ").upper())


def _proposal_label_point(proposal: dict[str, Any], geom: Any | None = None) -> list[float]:
    raw = _point(proposal.get("label_point"))
    if raw is not None:
        return v2_geometry._round_point(raw[0], raw[1])
    if geom is None:
        geom = _proposal_geometry(proposal)
    if geom.is_empty:
        return [0.0, 0.0]
    point = geom.representative_point()
    return v2_geometry._round_point(float(point.x), float(point.y))


def _portal_by_id(area: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(portal.get("id", "")): portal for portal in area.get("portal_candidates", [])}


def _first_portal(proposal: dict[str, Any], portals: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
    for portal_id in proposal.get("entry_portals", []):
        portal = portals.get(str(portal_id))
        if portal:
            return portal
    return None


def _portal_center(proposal: dict[str, Any], portals: dict[str, dict[str, Any]]) -> tuple[float, float] | None:
    portal = _first_portal(proposal, portals)
    if not portal:
        return None
    return _point(portal.get("center"))


def _portal_label(proposal: dict[str, Any], portals: dict[str, dict[str, Any]]) -> str:
    portal = _first_portal(proposal, portals)
    if not portal:
        return ""
    return str(portal.get("display_label") or portal.get("id") or "")


def _distance(a: tuple[float, float] | None, b: tuple[float, float] | None) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _numeric_evidence(proposal: dict[str, Any], field: str) -> float | None:
    value = proposal.get(field)
    if isinstance(value, (int, float)):
        return float(value)
    evidence = proposal.get("evidence", {})
    value = evidence.get(field)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _metric_delta(operator: dict[str, Any], automatic: dict[str, Any], field: str) -> float | None:
    operator_value = _numeric_evidence(operator, field)
    automatic_value = _numeric_evidence(automatic, field)
    if operator_value is None or automatic_value is None:
        return None
    return automatic_value - operator_value


def _operator_tasks(area: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        proposal
        for proposal in area.get("task_proposals", [])
        if proposal.get("extent_source") == "operator_annotation" or proposal.get("operator_annotation_id")
    ]


def _automatic_tasks(area: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        proposal
        for proposal in area.get("task_proposals", [])
        if proposal.get("task_type") != "body_core"
    ]


def _overlap_record(
    operator: dict[str, Any],
    automatic: dict[str, Any],
    operator_geom: Any,
    auto_geom: Any,
) -> dict[str, float]:
    operator_area = _area(operator_geom)
    auto_area = _area(auto_geom)
    if operator_area <= EPS or auto_area <= EPS:
        return {
            "operator_area_m2": operator_area,
            "auto_area_m2": auto_area,
            "overlap_area_m2": 0.0,
            "iou": 0.0,
            "operator_coverage_ratio": 0.0,
            "auto_coverage_ratio": 0.0,
        }
    overlap_area = _area(operator_geom.intersection(auto_geom))
    union_area = operator_area + auto_area - overlap_area
    return {
        "operator_area_m2": operator_area,
        "auto_area_m2": auto_area,
        "overlap_area_m2": overlap_area,
        "iou": overlap_area / union_area if union_area > EPS else 0.0,
        "operator_coverage_ratio": overlap_area / operator_area,
        "auto_coverage_ratio": overlap_area / auto_area,
    }


def _match_status(
    *,
    operator: dict[str, Any],
    automatic: dict[str, Any] | None,
    overlap: dict[str, float] | None,
    portal_distance_m: float | None,
    min_operator_overlap: float,
    good_overlap: float,
    max_portal_distance_m: float,
) -> tuple[str, str]:
    if automatic is None or overlap is None or overlap["operator_coverage_ratio"] < min_operator_overlap:
        return (
            "missed_by_auto",
            "automatic task evidence did not cover enough of the operator service region",
        )
    if str(operator.get("task_type", "")) != str(automatic.get("task_type", "")):
        return (
            "type_mismatch",
            "automatic evidence overlapped the region but classified the task differently",
        )
    if portal_distance_m is not None and portal_distance_m > max_portal_distance_m:
        return (
            "mouth_misaligned",
            "automatic portal is too far from the operator mouth cut",
        )
    operator_ratio = overlap["operator_coverage_ratio"]
    auto_ratio = overlap["auto_coverage_ratio"]
    if operator_ratio >= good_overlap and auto_ratio >= good_overlap:
        return ("matched", "automatic region agrees with the operator service region")
    if operator_ratio >= good_overlap and auto_ratio < good_overlap:
        return ("auto_too_large", "automatic region includes substantial extra area")
    if operator_ratio < good_overlap and auto_ratio >= good_overlap:
        return ("auto_too_small", "automatic region covers only part of the operator service region")
    return ("partial_match", "automatic and operator regions overlap, but neither extent is a strong match")


def _best_automatic_match(
    operator: dict[str, Any],
    operator_geom: Any,
    candidates: list[dict[str, Any]],
) -> tuple[dict[str, Any] | None, Any, dict[str, float] | None]:
    best_candidate = None
    best_geom = MultiPolygon([])
    best_overlap = None
    best_key = (-1.0, -1.0, -1.0, 0.0)
    operator_type = str(operator.get("task_type", ""))
    for candidate in candidates:
        candidate_geom = _proposal_geometry(candidate)
        overlap = _overlap_record(operator, candidate, operator_geom, candidate_geom)
        key = (
            overlap["iou"],
            overlap["operator_coverage_ratio"],
            overlap["auto_coverage_ratio"],
            1.0 if str(candidate.get("task_type", "")) == operator_type else 0.0,
        )
        if key > best_key:
            best_key = key
            best_candidate = candidate
            best_geom = candidate_geom
            best_overlap = overlap
    return best_candidate, best_geom, best_overlap


def _match_record(
    *,
    area_index: int,
    match_index: int,
    operator: dict[str, Any],
    automatic: dict[str, Any] | None,
    operator_geom: Any,
    auto_geom: Any,
    overlap: dict[str, float] | None,
    operator_portals: dict[str, dict[str, Any]],
    auto_portals: dict[str, dict[str, Any]],
    status: str,
    reason: str,
) -> dict[str, Any]:
    overlap = overlap or {
        "operator_area_m2": _area(operator_geom),
        "auto_area_m2": 0.0,
        "overlap_area_m2": 0.0,
        "iou": 0.0,
        "operator_coverage_ratio": 0.0,
        "auto_coverage_ratio": 0.0,
    }
    operator_center = _portal_center(operator, operator_portals)
    auto_center = _portal_center(automatic, auto_portals) if automatic else None
    portal_distance = _distance(operator_center, auto_center)
    record = {
        "id": f"area-{area_index}-annotation-qa-{match_index}",
        "area_index": area_index,
        "operator_task_id": str(operator.get("id", "")),
        "operator_annotation_id": str(operator.get("operator_annotation_id", "")),
        "operator_label": _proposal_label(operator),
        "operator_task_type": str(operator.get("task_type", "")),
        "operator_portal": _portal_label(operator, operator_portals),
        "operator_geometry": v2_geometry.geometry_to_json(operator_geom),
        "operator_label_point": _proposal_label_point(operator, operator_geom),
        "operator_area_m2": _round(overlap["operator_area_m2"]),
        "status": status,
        "reason": reason,
        "overlap_area_m2": _round(overlap["overlap_area_m2"]),
        "iou": _round(overlap["iou"]),
        "operator_coverage_ratio": _round(overlap["operator_coverage_ratio"]),
        "auto_coverage_ratio": _round(overlap["auto_coverage_ratio"]),
        "portal_distance_m": _round(portal_distance),
    }
    if automatic is not None:
        record.update(
            {
                "matched_auto_id": str(automatic.get("id", "")),
                "matched_auto_label": _proposal_label(automatic),
                "matched_auto_task_type": str(automatic.get("task_type", "")),
                "matched_auto_portal": _portal_label(automatic, auto_portals),
                "matched_auto_geometry": v2_geometry.geometry_to_json(auto_geom),
                "matched_auto_label_point": _proposal_label_point(automatic, auto_geom),
                "auto_area_m2": _round(overlap["auto_area_m2"]),
                "auto_visible_by_default": bool(automatic.get("visible_by_default", True)),
                "type_match": str(operator.get("task_type", "")) == str(automatic.get("task_type", "")),
                "depth_delta_m": _round(_metric_delta(operator, automatic, "service_depth_m")),
                "width_delta_m": _round(_metric_delta(operator, automatic, "service_width_m")),
            }
        )
    else:
        record.update(
            {
                "matched_auto_id": "",
                "matched_auto_label": "",
                "matched_auto_task_type": "",
                "matched_auto_portal": "",
                "matched_auto_geometry": {},
                "matched_auto_label_point": [],
                "auto_area_m2": 0.0,
                "auto_visible_by_default": False,
                "type_match": False,
                "depth_delta_m": None,
                "width_delta_m": None,
            }
        )
    return record


def _unmatched_auto_record(area_index: int, index: int, proposal: dict[str, Any]) -> dict[str, Any]:
    geom = _proposal_geometry(proposal)
    return {
        "id": f"area-{area_index}-unmatched-auto-{index}",
        "area_index": area_index,
        "auto_task_id": str(proposal.get("id", "")),
        "auto_label": _proposal_label(proposal),
        "auto_task_type": str(proposal.get("task_type", "")),
        "geometry": v2_geometry.geometry_to_json(geom),
        "label_point": _proposal_label_point(proposal, geom),
        "area_m2": _round(_area(geom)),
        "visible_by_default": bool(proposal.get("visible_by_default", True)),
        "status": "unmatched_auto_proposal",
        "reason": "automatic task did not overlap any operator task enough",
    }


def _status_counts(matches: list[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for match in matches:
        status = str(match.get("status", "unknown"))
        counts[status] = counts.get(status, 0) + 1
    return counts


def apply_annotation_qa(
    result: dict[str, Any],
    automatic_result: dict[str, Any],
    config: dict[str, Any],
) -> dict[str, Any]:
    """Attach operator-vs-automatic task-evidence QA to ``result``."""
    min_operator_overlap = max(
        0.0,
        min(1.0, _float_config(config, "v2_annotation_qa_min_operator_overlap", 0.25)),
    )
    good_overlap = max(
        min_operator_overlap,
        min(1.0, _float_config(config, "v2_annotation_qa_good_overlap", 0.65)),
    )
    max_portal_distance = max(0.0, _float_config(config, "v2_annotation_qa_max_portal_distance_m", 0.80))
    automatic_areas = {
        int(area.get("area_index", index)): area
        for index, area in enumerate(automatic_result.get("areas", []))
    }

    qa_areas: list[dict[str, Any]] = []
    global_status: dict[str, int] = {}
    total_operator = 0
    total_unmatched_auto = 0
    hidden_auto_artifacts = 0

    for area_index, area in enumerate(result.get("areas", [])):
        actual_area_index = int(area.get("area_index", area_index))
        automatic_area = automatic_areas.get(actual_area_index, {})
        operator_tasks = _operator_tasks(area)
        automatic_tasks = _automatic_tasks(automatic_area)
        operator_portals = _portal_by_id(area)
        automatic_portals = _portal_by_id(automatic_area)
        hidden_auto_artifacts += sum(
            1
            for proposal in automatic_tasks
            if proposal.get("task_type") == "artifact" and not proposal.get("visible_by_default", True)
        )

        matches: list[dict[str, Any]] = []
        matched_auto_ids: set[str] = set()
        for match_index, operator in enumerate(operator_tasks, start=1):
            operator_geom = _proposal_geometry(operator)
            automatic, auto_geom, overlap = _best_automatic_match(operator, operator_geom, automatic_tasks)
            if overlap is None or overlap["operator_coverage_ratio"] < min_operator_overlap:
                automatic = None
                auto_geom = MultiPolygon([])
                overlap = None
                portal_distance = None
            else:
                portal_distance = _distance(
                    _portal_center(operator, operator_portals),
                    _portal_center(automatic, automatic_portals) if automatic else None,
                )
                if automatic:
                    matched_auto_ids.add(str(automatic.get("id", "")))
            status, reason = _match_status(
                operator=operator,
                automatic=automatic,
                overlap=overlap,
                portal_distance_m=portal_distance,
                min_operator_overlap=min_operator_overlap,
                good_overlap=good_overlap,
                max_portal_distance_m=max_portal_distance,
            )
            matches.append(
                _match_record(
                    area_index=actual_area_index,
                    match_index=match_index,
                    operator=operator,
                    automatic=automatic,
                    operator_geom=operator_geom,
                    auto_geom=auto_geom,
                    overlap=overlap,
                    operator_portals=operator_portals,
                    auto_portals=automatic_portals,
                    status=status,
                    reason=reason,
                )
            )

        unmatched_auto = [
            _unmatched_auto_record(actual_area_index, index, proposal)
            for index, proposal in enumerate(automatic_tasks, start=1)
            if str(proposal.get("id", "")) not in matched_auto_ids
            and proposal.get("visible_by_default", True)
        ]
        status_counts = _status_counts(matches)
        for status, count in status_counts.items():
            global_status[status] = global_status.get(status, 0) + count
        total_operator += len(operator_tasks)
        total_unmatched_auto += len(unmatched_auto)
        area_qa = {
            "schema": SCHEMA,
            "area_index": actual_area_index,
            "summary": {
                "operator_tasks": len(operator_tasks),
                "automatic_candidates": len(automatic_tasks),
                "status_counts": status_counts,
                "unmatched_auto": len(unmatched_auto),
                "hidden_auto_artifacts": sum(
                    1
                    for proposal in automatic_tasks
                    if proposal.get("task_type") == "artifact" and not proposal.get("visible_by_default", True)
                ),
            },
            "matches": matches,
            "unmatched_auto_proposals": unmatched_auto,
        }
        area["annotation_qa"] = area_qa
        qa_areas.append(area_qa)

    summary = {
        "operator_tasks": total_operator,
        "matched": global_status.get("matched", 0),
        "missed_by_auto": global_status.get("missed_by_auto", 0),
        "type_mismatch": global_status.get("type_mismatch", 0),
        "mouth_misaligned": global_status.get("mouth_misaligned", 0),
        "auto_too_large": global_status.get("auto_too_large", 0),
        "auto_too_small": global_status.get("auto_too_small", 0),
        "partial_match": global_status.get("partial_match", 0),
        "unmatched_auto": total_unmatched_auto,
        "hidden_auto_artifacts": hidden_auto_artifacts,
        "status_counts": global_status,
        "min_operator_overlap": min_operator_overlap,
        "good_overlap": good_overlap,
        "max_portal_distance_m": max_portal_distance,
    }
    result["annotation_qa"] = {
        "schema": SCHEMA,
        "summary": summary,
        "areas": qa_areas,
    }
    return result
