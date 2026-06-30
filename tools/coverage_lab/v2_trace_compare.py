#!/usr/bin/env python3
"""Compare a live mower trace against a planpath_compat route.

Inputs:
- a V2/legacy ``planpath_compat.json`` with map-frame base_link poses;
- a JSONL trace written by ``v2_live_trace_recorder.py``.

Outputs:
- ``trace_compare.json`` with cross-track and heading metrics;
- ``trace_compare.html`` with an SVG overlay of planned and actual paths.
"""
from __future__ import annotations

import argparse
import bisect
import html
import json
import math
import pathlib
from dataclasses import dataclass
from statistics import mean, median
from typing import Any, Iterable


EPS = 1e-9


@dataclass(frozen=True)
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float | None
    topic: str
    position_accuracy: float | None = None
    orientation_accuracy: float | None = None
    state_name: str | None = None
    current_path_index: int | None = None


@dataclass(frozen=True)
class PlanPose:
    x: float
    y: float
    yaw: float
    path_index: int
    pose_index: int
    is_outline: bool
    label: str


@dataclass(frozen=True)
class PlanSegment:
    ax: float
    ay: float
    bx: float
    by: float
    yaw: float
    length: float
    cumulative_start: float
    path_index: int
    pose_index: int
    is_outline: bool
    label: str


def _read_json(path: pathlib.Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _write_json(path: pathlib.Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
        handle.write("\n")


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _safe_float(value: Any) -> float | None:
    try:
        output = float(value)
    except (TypeError, ValueError):
        return None
    return output if math.isfinite(output) else None


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * percentile / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def _polyline_length(points: list[tuple[float, float]]) -> float:
    return sum(_distance(points[i - 1], points[i]) for i in range(1, len(points)))


def _flatten_plan(plan: dict[str, Any]) -> tuple[list[PlanPose], list[PlanSegment]]:
    poses: list[PlanPose] = []
    segments: list[PlanSegment] = []
    cumulative = 0.0
    for path_index, path in enumerate(plan.get("paths", []) or []):
        is_outline = bool(path.get("is_outline", False))
        label = str(path.get("label") or f"path {path_index}")
        raw_poses = list(path.get("path", {}).get("poses", []) or [])
        path_poses: list[PlanPose] = []
        for pose_index, raw in enumerate(raw_poses):
            try:
                pose = PlanPose(
                    x=float(raw["x"]),
                    y=float(raw["y"]),
                    yaw=float(raw.get("yaw", 0.0)),
                    path_index=path_index,
                    pose_index=pose_index,
                    is_outline=is_outline,
                    label=label,
                )
            except (KeyError, TypeError, ValueError):
                continue
            poses.append(pose)
            path_poses.append(pose)
        for a, b in zip(path_poses, path_poses[1:]):
            length = math.hypot(b.x - a.x, b.y - a.y)
            if length <= EPS:
                continue
            yaw = math.atan2(b.y - a.y, b.x - a.x)
            segments.append(
                PlanSegment(
                    ax=a.x,
                    ay=a.y,
                    bx=b.x,
                    by=b.y,
                    yaw=yaw,
                    length=length,
                    cumulative_start=cumulative,
                    path_index=a.path_index,
                    pose_index=a.pose_index,
                    is_outline=a.is_outline,
                    label=a.label,
                )
            )
            cumulative += length
    return poses, segments


def _load_map_rings(path: pathlib.Path | None) -> list[dict[str, Any]]:
    if path is None:
        return []
    try:
        data = _read_json(path)
    except (OSError, json.JSONDecodeError):
        return []
    rings: list[dict[str, Any]] = []
    for index, area in enumerate(data.get("areas", []) or []):
        outline = []
        for point in area.get("outline", []) or []:
            try:
                outline.append((float(point["x"]), float(point["y"])))
            except (KeyError, TypeError, ValueError):
                continue
        if len(outline) >= 2:
            props = area.get("properties") or {}
            rings.append(
                {
                    "id": str(area.get("id") or f"area-{index}"),
                    "type": str(props.get("type", "")),
                    "name": str(props.get("name") or area.get("id") or f"area {index}"),
                    "points": outline,
                }
            )
    return rings


def _load_trace(path: pathlib.Path, pose_topic: str) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    metadata: dict[str, Any] = {}
    pose_records: list[dict[str, Any]] = []
    state_records: list[dict[str, Any]] = []
    quality_records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("type") == "metadata":
                metadata = record
                continue
            if record.get("type") != "sample":
                continue
            topic = record.get("topic")
            if topic == pose_topic:
                pose_records.append(record)
            elif topic == "/mower_logic/current_state":
                state_records.append(record)
            elif topic == "/hw/position/gps/quality":
                quality_records.append(record)
    pose_records.sort(key=lambda r: float(r.get("wall_time") or r.get("stamp") or 0.0))
    state_records.sort(key=lambda r: float(r.get("wall_time") or r.get("stamp") or 0.0))
    quality_records.sort(key=lambda r: float(r.get("wall_time") or r.get("stamp") or 0.0))
    return metadata, pose_records, state_records, quality_records


def _state_lookup(state_records: list[dict[str, Any]]) -> tuple[list[float], list[dict[str, Any]]]:
    times = [float(record.get("wall_time") or record.get("stamp") or 0.0) for record in state_records]
    states = [record.get("derived", {}) or {} for record in state_records]
    return times, states


def _state_at(t: float, state_times: list[float], states: list[dict[str, Any]]) -> dict[str, Any]:
    if not state_times:
        return {}
    index = bisect.bisect_right(state_times, t) - 1
    if index < 0:
        return {}
    return states[index]


def _pose_samples(pose_records: list[dict[str, Any]], state_records: list[dict[str, Any]]) -> list[PoseSample]:
    state_times, states = _state_lookup(state_records)
    samples: list[PoseSample] = []
    for record in pose_records:
        derived = record.get("derived", {}) or {}
        x = _safe_float(derived.get("x"))
        y = _safe_float(derived.get("y"))
        if x is None or y is None:
            continue
        t = float(record.get("wall_time") or record.get("stamp") or 0.0)
        state = _state_at(t, state_times, states)
        samples.append(
            PoseSample(
                t=t,
                x=x,
                y=y,
                yaw=_safe_float(derived.get("yaw")),
                topic=str(record.get("topic", "")),
                position_accuracy=_safe_float(derived.get("position_accuracy")),
                orientation_accuracy=_safe_float(derived.get("orientation_accuracy")),
                state_name=str(state.get("state_name", "")) if state else None,
                current_path_index=int(state["current_path_index"]) if state and "current_path_index" in state else None,
            )
        )
    return samples


def _is_active_sample(sample: PoseSample, active_states: set[str]) -> bool:
    if not sample.state_name:
        return True
    if sample.state_name in active_states:
        return True
    if sample.current_path_index is not None and sample.current_path_index >= 0:
        return True
    return False


def _nearest_segment(sample: PoseSample, segments: list[PlanSegment]) -> dict[str, Any] | None:
    if not segments:
        return None
    best: dict[str, Any] | None = None
    px, py = sample.x, sample.y
    for seg_index, seg in enumerate(segments):
        vx = seg.bx - seg.ax
        vy = seg.by - seg.ay
        denom = seg.length * seg.length
        u = ((px - seg.ax) * vx + (py - seg.ay) * vy) / denom if denom > EPS else 0.0
        u = max(0.0, min(1.0, u))
        qx = seg.ax + u * vx
        qy = seg.ay + u * vy
        dx = px - qx
        dy = py - qy
        distance = math.hypot(dx, dy)
        signed = (vx * (py - seg.ay) - vy * (px - seg.ax)) / max(seg.length, EPS)
        yaw_error = None if sample.yaw is None else _normalize_angle(sample.yaw - seg.yaw)
        candidate = {
            "segment_index": seg_index,
            "path_index": seg.path_index,
            "pose_index": seg.pose_index,
            "is_outline": seg.is_outline,
            "label": seg.label,
            "projection": {"x": qx, "y": qy},
            "distance_m": distance,
            "signed_cross_track_m": signed,
            "along_track_m": seg.cumulative_start + u * seg.length,
            "plan_yaw": seg.yaw,
            "yaw_error_rad": yaw_error,
        }
        if best is None or distance < float(best["distance_m"]):
            best = candidate
    return best


def _compare_samples(samples: list[PoseSample], segments: list[PlanSegment]) -> list[dict[str, Any]]:
    comparisons: list[dict[str, Any]] = []
    for sample in samples:
        nearest = _nearest_segment(sample, segments)
        if nearest is None:
            continue
        comparisons.append(
            {
                "t": sample.t,
                "x": sample.x,
                "y": sample.y,
                "yaw": sample.yaw,
                "position_accuracy": sample.position_accuracy,
                "orientation_accuracy": sample.orientation_accuracy,
                "state_name": sample.state_name,
                "current_path_index": sample.current_path_index,
                **nearest,
            }
        )
    return comparisons


def _gps_quality_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    if not records:
        return {"sample_count": 0}
    derived = [r.get("derived", {}) or {} for r in records]
    timing = [_safe_float(d.get("parser_timing_warnings")) for d in derived]
    skipped = [_safe_float(d.get("parser_skipped_bytes")) for d in derived]
    h_acc = [_safe_float(d.get("h_acc_m")) for d in derived]
    valid = [d for d in derived if d]
    out: dict[str, Any] = {"sample_count": len(valid)}
    if timing and timing[0] is not None and timing[-1] is not None:
        out["parser_timing_warnings_delta"] = timing[-1] - timing[0]
        out["parser_timing_warnings_last"] = timing[-1]
    if skipped and skipped[0] is not None and skipped[-1] is not None:
        out["parser_skipped_bytes_delta"] = skipped[-1] - skipped[0]
        out["parser_skipped_bytes_last"] = skipped[-1]
    h_acc_values = [v for v in h_acc if v is not None]
    if h_acc_values:
        out["h_acc_m_median"] = median(h_acc_values)
        out["h_acc_m_max"] = max(h_acc_values)
    if valid:
        last = valid[-1]
        for key in ["rtk_type", "num_sv", "vehicle_heading_valid", "motion_heading_valid", "head_acc_rad"]:
            if key in last:
                out[f"last_{key}"] = last[key]
    return out


def _summarize(comparisons: list[dict[str, Any]], samples: list[PoseSample], planned_length: float) -> dict[str, Any]:
    distances = [abs(float(c["distance_m"])) for c in comparisons]
    yaw_errors = [
        abs(float(c["yaw_error_rad"]))
        for c in comparisons
        if c.get("yaw_error_rad") is not None and math.isfinite(float(c["yaw_error_rad"]))
    ]
    points = [(s.x, s.y) for s in samples]
    duration = (samples[-1].t - samples[0].t) if len(samples) >= 2 else 0.0
    intervals = [samples[i].t - samples[i - 1].t for i in range(1, len(samples))]
    return {
        "planned_length_m": round(planned_length, 4),
        "actual_distance_m": round(_polyline_length(points), 4),
        "duration_s": round(duration, 3),
        "sample_count": len(samples),
        "comparison_count": len(comparisons),
        "mean_cross_track_m": round(mean(distances), 4) if distances else None,
        "median_cross_track_m": round(median(distances), 4) if distances else None,
        "p95_cross_track_m": round(_percentile(distances, 95) or 0.0, 4) if distances else None,
        "max_cross_track_m": round(max(distances), 4) if distances else None,
        "median_abs_yaw_error_deg": round(math.degrees(median(yaw_errors)), 2) if yaw_errors else None,
        "p95_abs_yaw_error_deg": round(math.degrees(_percentile(yaw_errors, 95) or 0.0), 2) if yaw_errors else None,
        "max_abs_yaw_error_deg": round(math.degrees(max(yaw_errors)), 2) if yaw_errors else None,
        "max_sample_gap_s": round(max(intervals), 3) if intervals else None,
        "mean_sample_gap_s": round(mean(intervals), 3) if intervals else None,
    }


def _error_class(distance: float) -> str:
    if distance < 0.15:
        return "good"
    if distance < 0.30:
        return "warn"
    return "bad"


def _bounds(*point_sets: Iterable[tuple[float, float]]) -> tuple[float, float, float, float]:
    points = [p for point_set in point_sets for p in point_set]
    if not points:
        return (0.0, 0.0, 1.0, 1.0)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    minx, miny, maxx, maxy = min(xs), min(ys), max(xs), max(ys)
    pad = max(0.5, max(maxx - minx, maxy - miny) * 0.08)
    return (minx - pad, miny - pad, maxx + pad, maxy + pad)


def _svg_transform(bounds: tuple[float, float, float, float], width: int, height: int):
    minx, miny, maxx, maxy = bounds
    span_x = max(maxx - minx, EPS)
    span_y = max(maxy - miny, EPS)
    scale = min(width / span_x, height / span_y)
    used_w = span_x * scale
    used_h = span_y * scale
    margin_x = (width - used_w) * 0.5
    margin_y = (height - used_h) * 0.5

    def tx(point: tuple[float, float]) -> tuple[float, float]:
        x, y = point
        return (margin_x + (x - minx) * scale, margin_y + (maxy - y) * scale)

    return tx


def _polyline(points: list[tuple[float, float]], tx, class_name: str, title: str = "") -> str:
    if len(points) < 2:
        return ""
    pts = " ".join(f"{tx(p)[0]:.2f},{tx(p)[1]:.2f}" for p in points)
    title_tag = f"<title>{html.escape(title)}</title>" if title else ""
    return f'<polyline class="{class_name}" points="{pts}">{title_tag}</polyline>'


def _line(a: tuple[float, float], b: tuple[float, float], tx, class_name: str, title: str = "") -> str:
    ax, ay = tx(a)
    bx, by = tx(b)
    title_tag = f"<title>{html.escape(title)}</title>" if title else ""
    return f'<line class="{class_name}" x1="{ax:.2f}" y1="{ay:.2f}" x2="{bx:.2f}" y2="{by:.2f}">{title_tag}</line>'


def _circle(p: tuple[float, float], tx, radius: float, class_name: str, title: str = "") -> str:
    x, y = tx(p)
    title_tag = f"<title>{html.escape(title)}</title>" if title else ""
    return f'<circle class="{class_name}" cx="{x:.2f}" cy="{y:.2f}" r="{radius:.2f}">{title_tag}</circle>'


def _render_html(
    *,
    plan_path: pathlib.Path,
    trace_path: pathlib.Path,
    map_path: pathlib.Path | None,
    metadata: dict[str, Any],
    summary: dict[str, Any],
    gps_quality: dict[str, Any],
    plan_poses: list[PlanPose],
    comparisons: list[dict[str, Any]],
    active_samples: list[PoseSample],
    map_rings: list[dict[str, Any]],
) -> str:
    width, height = 1120, 760
    plan_points = [(p.x, p.y) for p in plan_poses]
    actual_points = [(s.x, s.y) for s in active_samples]
    ring_points = [p for ring in map_rings for p in ring["points"]]
    tx = _svg_transform(_bounds(plan_points, actual_points, ring_points), width, height)
    plan_svg = _polyline(plan_points, tx, "planned-path", "planned path")

    actual_lines: list[str] = []
    for previous, current in zip(comparisons, comparisons[1:]):
        if math.hypot(float(current["x"]) - float(previous["x"]), float(current["y"]) - float(previous["y"])) > 2.0:
            continue
        err = max(abs(float(previous["distance_m"])), abs(float(current["distance_m"])))
        actual_lines.append(
            _line(
                (float(previous["x"]), float(previous["y"])),
                (float(current["x"]), float(current["y"])),
                tx,
                f"actual-track actual-{_error_class(err)}",
                f"cross-track {err:.2f} m",
            )
        )

    projection_lines: list[str] = []
    worst = sorted(comparisons, key=lambda c: abs(float(c["distance_m"])), reverse=True)[:30]
    for record in worst:
        projection = record.get("projection", {}) or {}
        px = _safe_float(projection.get("x"))
        py = _safe_float(projection.get("y"))
        if px is None or py is None:
            continue
        projection_lines.append(
            _line(
                (float(record["x"]), float(record["y"])),
                (px, py),
                tx,
                "error-stick",
                f"{float(record['distance_m']):.2f} m error",
            )
        )
        projection_lines.append(
            _circle(
                (float(record["x"]), float(record["y"])),
                tx,
                3.5,
                f"sample-dot sample-{_error_class(abs(float(record['distance_m'])))}",
                f"t {float(record['t']):.1f}; error {float(record['distance_m']):.2f} m",
            )
        )

    boundary_svg: list[str] = []
    for ring in map_rings:
        ring_class = "map-obstacle" if ring.get("type") == "obstacle" else "map-boundary"
        boundary_svg.append(_polyline(ring["points"], tx, ring_class, str(ring.get("name") or ring.get("id"))))

    start_end = ""
    if plan_points:
        start_end += _circle(plan_points[0], tx, 5.0, "plan-start", "planned start")
        start_end += _circle(plan_points[-1], tx, 5.0, "plan-end", "planned end")
    if actual_points:
        start_end += _circle(actual_points[0], tx, 4.0, "actual-start", "actual start")
        start_end += _circle(actual_points[-1], tx, 4.0, "actual-end", "actual end")

    metric_rows = "\n".join(
        f"<tr><th>{html.escape(str(key))}</th><td>{html.escape(json.dumps(value, sort_keys=True))}</td></tr>"
        for key, value in summary.items()
    )
    gps_rows = "\n".join(
        f"<tr><th>{html.escape(str(key))}</th><td>{html.escape(json.dumps(value, sort_keys=True))}</td></tr>"
        for key, value in gps_quality.items()
    )
    worst_row_html: list[str] = []
    for row in worst[:20]:
        yaw_error = row.get("yaw_error_rad")
        yaw_error_text = "" if yaw_error is None else f"{math.degrees(float(yaw_error)):.1f}"
        worst_row_html.append(
            "<tr>"
            f"<td>{float(row['distance_m']):.3f}</td>"
            f"<td>{html.escape(str(row.get('state_name') or ''))}</td>"
            f"<td>{float(row['x']):.3f}, {float(row['y']):.3f}</td>"
            f"<td>{float(row['along_track_m']):.2f}</td>"
            f"<td>{yaw_error_text}</td>"
            "</tr>"
        )
    worst_rows = "\n".join(worst_row_html)
    metadata_json = html.escape(json.dumps(metadata, indent=2, sort_keys=True))

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>V2 Live Trace Comparison</title>
  <style>
    :root {{ color-scheme: light; font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }}
    body {{ margin: 0; background: #f5f7fb; color: #111827; }}
    header {{ background: #102033; color: white; padding: 22px 28px; }}
    h1 {{ margin: 0 0 6px; font-size: 28px; }}
    main {{ padding: 24px 28px 42px; display: grid; gap: 22px; }}
    .panel {{ background: white; border: 1px solid #cfd8e3; border-radius: 8px; padding: 16px; }}
    .map-wrap {{ overflow: hidden; }}
    .toolbar {{ display: flex; flex-wrap: wrap; gap: 12px; align-items: center; margin-bottom: 12px; }}
    button {{ border: 1px solid #b8c4d4; background: #fff; border-radius: 6px; padding: 7px 10px; cursor: pointer; }}
    label {{ display: inline-flex; align-items: center; gap: 6px; font-size: 14px; }}
    svg {{ width: 100%; height: min(74vh, 760px); background: #fbfcff; border: 1px solid #d9e2ef; border-radius: 8px; touch-action: none; }}
    .planned-path {{ fill: none; stroke: #2563eb; stroke-width: 3.2; stroke-linecap: round; stroke-linejoin: round; opacity: 0.95; }}
    .actual-track {{ fill: none; stroke-width: 3.0; stroke-linecap: round; opacity: 0.9; }}
    .actual-good {{ stroke: #16a34a; }}
    .actual-warn {{ stroke: #f59e0b; }}
    .actual-bad {{ stroke: #dc2626; }}
    .error-stick {{ stroke: #991b1b; stroke-width: 1.2; stroke-dasharray: 4 4; opacity: 0.55; }}
    .sample-dot {{ stroke: white; stroke-width: 1.5; opacity: 0.9; }}
    .sample-good {{ fill: #16a34a; }}
    .sample-warn {{ fill: #f59e0b; }}
    .sample-bad {{ fill: #dc2626; }}
    .map-boundary {{ fill: none; stroke: #111827; stroke-width: 2; stroke-linejoin: round; opacity: 0.65; }}
    .map-obstacle {{ fill: rgba(17, 24, 39, 0.08); stroke: #374151; stroke-width: 1.5; }}
    .plan-start {{ fill: #2563eb; stroke: white; stroke-width: 2; }}
    .plan-end {{ fill: #7c3aed; stroke: white; stroke-width: 2; }}
    .actual-start {{ fill: #16a34a; stroke: white; stroke-width: 2; }}
    .actual-end {{ fill: #dc2626; stroke: white; stroke-width: 2; }}
    .legend {{ display: flex; flex-wrap: wrap; gap: 12px; font-size: 14px; }}
    .swatch {{ width: 18px; height: 4px; display: inline-block; vertical-align: middle; border-radius: 999px; margin-right: 5px; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 18px; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 14px; }}
    th, td {{ border-bottom: 1px solid #e5e7eb; padding: 7px 8px; text-align: left; vertical-align: top; }}
    th {{ color: #374151; font-weight: 650; }}
    code, pre {{ background: #eef2f7; border-radius: 6px; }}
    code {{ padding: 2px 5px; }}
    pre {{ padding: 12px; overflow: auto; max-height: 320px; }}
    .muted {{ color: #526173; }}
    .hidden-layer {{ display: none; }}
  </style>
</head>
<body>
  <header>
    <h1>V2 Live Trace Comparison</h1>
    <div class="muted">Plan: <code>{html.escape(str(plan_path))}</code></div>
    <div class="muted">Trace: <code>{html.escape(str(trace_path))}</code></div>
    <div class="muted">Map: <code>{html.escape(str(map_path) if map_path else "not supplied")}</code></div>
  </header>
  <main>
    <section class="panel map-wrap">
      <div class="toolbar">
        <button id="zoom-fit" type="button">Fit</button>
        <button id="zoom-in" type="button">+</button>
        <button id="zoom-out" type="button">-</button>
        <label><input type="checkbox" data-layer="actual-layer" checked> actual track</label>
        <label><input type="checkbox" data-layer="plan-layer" checked> planned path</label>
        <label><input type="checkbox" data-layer="error-layer" checked> worst error sticks</label>
        <label><input type="checkbox" data-layer="boundary-layer" checked> map boundary</label>
      </div>
      <div class="legend">
        <span><span class="swatch" style="background:#2563eb"></span>planned</span>
        <span><span class="swatch" style="background:#16a34a"></span>&lt; 0.15 m</span>
        <span><span class="swatch" style="background:#f59e0b"></span>0.15-0.30 m</span>
        <span><span class="swatch" style="background:#dc2626"></span>&gt;= 0.30 m</span>
      </div>
      <svg id="trace-svg" viewBox="0 0 {width} {height}" role="img" aria-label="planned path and actual mower track">
        <g id="boundary-layer">{''.join(boundary_svg)}</g>
        <g id="plan-layer">{plan_svg}</g>
        <g id="actual-layer">{''.join(actual_lines)}</g>
        <g id="error-layer">{''.join(projection_lines)}</g>
        <g id="marker-layer">{start_end}</g>
      </svg>
    </section>
    <section class="grid">
      <div class="panel">
        <h2>Path Metrics</h2>
        <table>{metric_rows}</table>
      </div>
      <div class="panel">
        <h2>GPS Quality</h2>
        <table>{gps_rows}</table>
      </div>
    </section>
    <section class="panel">
      <h2>Worst Samples</h2>
      <table>
        <thead><tr><th>error m</th><th>state</th><th>actual x,y</th><th>along plan m</th><th>yaw err deg</th></tr></thead>
        <tbody>{worst_rows}</tbody>
      </table>
    </section>
    <section class="panel">
      <h2>Trace Metadata</h2>
      <pre>{metadata_json}</pre>
    </section>
  </main>
  <script>
    const svg = document.getElementById("trace-svg");
    const initialViewBox = [0, 0, {width}, {height}];
    let viewBox = initialViewBox.slice();
    function setViewBox() {{
      svg.setAttribute("viewBox", viewBox.map(v => v.toFixed(2)).join(" "));
    }}
    function zoomAt(factor, cx, cy) {{
      const [x, y, w, h] = viewBox;
      const nx = cx - (cx - x) * factor;
      const ny = cy - (cy - y) * factor;
      viewBox = [nx, ny, w * factor, h * factor];
      setViewBox();
    }}
    function clientToSvg(evt) {{
      const rect = svg.getBoundingClientRect();
      return [
        viewBox[0] + ((evt.clientX - rect.left) / rect.width) * viewBox[2],
        viewBox[1] + ((evt.clientY - rect.top) / rect.height) * viewBox[3],
      ];
    }}
    document.getElementById("zoom-fit").addEventListener("click", () => {{ viewBox = initialViewBox.slice(); setViewBox(); }});
    document.getElementById("zoom-in").addEventListener("click", () => zoomAt(0.8, viewBox[0] + viewBox[2] / 2, viewBox[1] + viewBox[3] / 2));
    document.getElementById("zoom-out").addEventListener("click", () => zoomAt(1.25, viewBox[0] + viewBox[2] / 2, viewBox[1] + viewBox[3] / 2));
    svg.addEventListener("wheel", (evt) => {{
      evt.preventDefault();
      const [cx, cy] = clientToSvg(evt);
      zoomAt(evt.deltaY < 0 ? 0.88 : 1.14, cx, cy);
    }}, {{ passive: false }});
    let dragging = false;
    let last = null;
    svg.addEventListener("pointerdown", (evt) => {{ dragging = true; last = [evt.clientX, evt.clientY]; svg.setPointerCapture(evt.pointerId); }});
    svg.addEventListener("pointermove", (evt) => {{
      if (!dragging || !last) return;
      const rect = svg.getBoundingClientRect();
      const dx = (evt.clientX - last[0]) / rect.width * viewBox[2];
      const dy = (evt.clientY - last[1]) / rect.height * viewBox[3];
      viewBox[0] -= dx; viewBox[1] -= dy; last = [evt.clientX, evt.clientY]; setViewBox();
    }});
    svg.addEventListener("pointerup", () => {{ dragging = false; last = null; }});
    document.querySelectorAll("[data-layer]").forEach(input => {{
      input.addEventListener("change", () => {{
        const layer = document.getElementById(input.dataset.layer);
        if (layer) layer.classList.toggle("hidden-layer", !input.checked);
      }});
    }});
  </script>
</body>
</html>
"""


def cmd_compare(args: argparse.Namespace) -> None:
    plan_path = pathlib.Path(args.plan).resolve()
    trace_path = pathlib.Path(args.trace).resolve()
    output_dir = pathlib.Path(args.output_dir).resolve()
    map_path = pathlib.Path(args.map).resolve() if args.map else None

    plan = _read_json(plan_path)
    plan_poses, plan_segments = _flatten_plan(plan)
    if not plan_segments:
        raise SystemExit("trace compare: plan has no usable segments")

    metadata, pose_records, state_records, quality_records = _load_trace(trace_path, args.pose_topic)
    samples = _pose_samples(pose_records, state_records)
    active_states = {s.strip() for s in args.active_state.split(",") if s.strip()}
    active_samples = [s for s in samples if _is_active_sample(s, active_states)]
    if not active_samples:
        active_samples = samples
    comparisons = _compare_samples(active_samples, plan_segments)
    planned_length = sum(seg.length for seg in plan_segments)
    summary = _summarize(comparisons, active_samples, planned_length)
    gps_quality = _gps_quality_summary(quality_records)
    result = {
        "schema": "open_mower.coverage_lab.trace_compare.v0",
        "plan": str(plan_path),
        "trace": str(trace_path),
        "pose_topic": args.pose_topic,
        "map": str(map_path) if map_path else None,
        "metadata": metadata,
        "summary": summary,
        "gps_quality": gps_quality,
        "worst_samples": sorted(comparisons, key=lambda c: abs(float(c["distance_m"])), reverse=True)[:50],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_json(output_dir / "trace_compare.json", result)
    html_report = _render_html(
        plan_path=plan_path,
        trace_path=trace_path,
        map_path=map_path,
        metadata=metadata,
        summary=summary,
        gps_quality=gps_quality,
        plan_poses=plan_poses,
        comparisons=comparisons,
        active_samples=active_samples,
        map_rings=_load_map_rings(map_path),
    )
    (output_dir / "trace_compare.html").write_text(html_report, encoding="utf-8")
    print(f"Wrote {output_dir / 'trace_compare.json'}")
    print(f"Wrote {output_dir / 'trace_compare.html'}")
    print(
        "summary: "
        f"samples={summary['sample_count']} "
        f"p95_error_m={summary['p95_cross_track_m']} "
        f"max_error_m={summary['max_cross_track_m']} "
        f"p95_yaw_error_deg={summary['p95_abs_yaw_error_deg']}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare a live mower JSONL trace to a planpath_compat route.")
    parser.add_argument("--plan", required=True, help="planpath_compat JSON used for the live test")
    parser.add_argument("--trace", required=True, help="JSONL trace written by v2_live_trace_recorder.py")
    parser.add_argument("--output-dir", required=True, help="directory for trace_compare.json/html")
    parser.add_argument("--map", help="optional OpenMower map.json to draw boundary context")
    parser.add_argument("--pose-topic", default="/xbot_positioning/xb_pose", help="pose topic from the trace to compare")
    parser.add_argument(
        "--active-state",
        default="MOWING,AUTONOMOUS",
        help="comma-separated mower state names treated as active; current_path_index >= 0 is always active",
    )
    parser.set_defaults(func=cmd_compare)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
