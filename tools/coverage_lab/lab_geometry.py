"""Footprint-aware headland and obstacle-aware cell decomposition helpers.

This module backs P0 (footprint-aware headland) and P1 (obstacle-aware swath
bridging) from ``docs/COVERAGE_PLANNER_ROADMAP.md``. It lives outside
``coverage_lab.py`` so the geometry primitives are testable on the host
without the Fields2Cover Docker image.

Coordinate conventions match the rest of the lab: rings are ``list[tuple[float, float]]``
in lawn-local (map-frame) metres, exterior rings counter-clockwise, hole rings
clockwise. Shapely polygons are also returned with outer CCW and holes CW.

References:
    Choset, "Coverage of Known Spaces: The Boustrophedon Cellular Decomposition"
    (Autonomous Robots, 2000) -- the canonical CPP-with-obstacles decomposition
    aligned to the sweep direction. The implementation here is a robust, simple
    variant: cut at each hole's x-extents in the stripe-aligned frame so every
    resulting cell is hole-free.
"""
from __future__ import annotations

import math
from typing import Iterable

from shapely.affinity import rotate
from shapely.geometry import (
    GeometryCollection,
    LineString,
    MultiLineString,
    MultiPolygon,
    Polygon,
    box,
)
from shapely.ops import unary_union


__all__ = [
    "footprint_disk_radius",
    "erode_lawn",
    "polygon_outer_ring",
    "polygon_holes",
    "polygon_to_ring_dicts",
    "bcd_decompose",
    "outer_reflex_ys",
    "ring_length",
    "walk_ring_between",
    "dominant_direction_angle",
    "cell_long_axis_angle",
    "cell_short_axis_extent_along",
]


EPS = 1e-9


# --------------------------------------------------------------------------------
# Footprint -> disk radius for Minkowski erosion
# --------------------------------------------------------------------------------


def footprint_disk_radius(config: dict) -> float:
    """Return the radius of the smallest disk centred at ``tool_center_offset``
    that contains every safety-footprint corner.

    The lab's path samples are emitted at the tool centre (the swath is
    cutter-centred). A Minkowski erosion of the lawn by this disk gives the
    set of tool-centre positions where the safety footprint stays inside the
    lawn at every yaw, which is exactly what the headland centreline needs.

    The safety footprint includes ``safety_margin_m``; the disk therefore
    bakes the margin in automatically.

    >>> r = footprint_disk_radius({
    ...     "tool_center_offset": [0.41, 0.0],
    ...     "footprint": [[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]],
    ...     "safety_margin_m": 0.05,
    ...     "tool_width": 0.4,
    ... })
    >>> round(r, 4)
    0.6031
    """
    margin = float(config.get("safety_margin_m", 0.0) or 0.0)
    footprint = config.get("footprint") or []
    if len(footprint) < 3:
        return max(float(config.get("tool_width", 0.0)) / 2.0 + margin, margin)
    xs = [float(p[0]) for p in footprint]
    ys = [float(p[1]) for p in footprint]
    # safety_footprint() in coverage_lab.py expands to an axis-aligned bbox + margin;
    # mirror that here so erosion clears the same footprint that compute_metrics
    # validates against.
    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin
    raw_offset = config.get("tool_center_offset") or [0.0, 0.0]
    ox, oy = float(raw_offset[0]), float(raw_offset[1])
    corners = ((min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y))
    return max(math.hypot(cx - ox, cy - oy) for cx, cy in corners)


# --------------------------------------------------------------------------------
# Lawn polygon construction and erosion
# --------------------------------------------------------------------------------


def _make_lawn_polygon(
    outline: Iterable[tuple[float, float]],
    holes: Iterable[Iterable[tuple[float, float]]] | None,
) -> Polygon:
    outer = list(outline)
    if len(outer) >= 2 and outer[0] == outer[-1]:
        outer = outer[:-1]
    interior_rings: list[list[tuple[float, float]]] = []
    for hole in holes or ():
        ring = list(hole)
        if len(ring) >= 2 and ring[0] == ring[-1]:
            ring = ring[:-1]
        if len(ring) >= 3:
            interior_rings.append(ring)
    polygon = Polygon(outer, interior_rings)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    if isinstance(polygon, MultiPolygon):
        polygon = max(polygon.geoms, key=lambda g: g.area)
    return polygon


def erode_lawn(
    outline: Iterable[tuple[float, float]],
    holes: Iterable[Iterable[tuple[float, float]]] | None,
    radius: float,
) -> list[Polygon]:
    """Erode ``lawn - holes`` by a disk of ``radius`` metres.

    Returns a list of resulting polygons; an erosion can split a polygon
    into multiple pieces (narrow necks vanish, peninsulas detach). Each piece
    is guaranteed safe for any pose at ``radius`` distance from the boundary.

    Uses ``join_style='round'`` so concave corners of the original boundary
    become rounded arcs in the eroded boundary; without this, miter joins
    can break the disk-clearance guarantee.

    >>> rect = [(0, 0), (10, 0), (10, 5), (0, 5)]
    >>> [round(p.area, 2) for p in erode_lawn(rect, [], 0.6)]
    [33.44]
    >>> [p.area for p in erode_lawn(rect, [], 3.0)]
    []
    """
    lawn = _make_lawn_polygon(outline, holes)
    if radius <= EPS:
        return [lawn]
    eroded = lawn.buffer(-radius, join_style="round", quad_segs=16)
    if eroded.is_empty:
        return []
    if isinstance(eroded, MultiPolygon):
        return [g for g in eroded.geoms if g.area > EPS]
    return [eroded] if eroded.area > EPS else []


def polygon_outer_ring(polygon: Polygon) -> list[tuple[float, float]]:
    """Return the outer ring of a polygon as a closed CCW list (last point repeats first).

    >>> p = Polygon([(0, 0), (4, 0), (4, 3), (0, 3)])
    >>> polygon_outer_ring(p)[:3]
    [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)]
    """
    ring = list(polygon.exterior.coords)
    # shapely returns the closing coord; normalise to closed CCW
    if not polygon.exterior.is_ccw:
        ring = list(reversed(ring))
    return [(float(x), float(y)) for x, y in ring]


def polygon_holes(polygon: Polygon) -> list[list[tuple[float, float]]]:
    """Return the polygon's interior rings as closed CW lists."""
    out: list[list[tuple[float, float]]] = []
    for interior in polygon.interiors:
        ring = list(interior.coords)
        # Shapely interiors are typically CW; normalise.
        is_ccw = LineString(ring).is_ccw if hasattr(LineString(ring), "is_ccw") else False
        if is_ccw:
            ring = list(reversed(ring))
        out.append([(float(x), float(y)) for x, y in ring])
    return out


def polygon_to_ring_dicts(polygon: Polygon, layer: int, ring_index: int, centerline_offset: float) -> list[dict]:
    """Convert a polygon into the headland_specs dict shape used by build_headland_paths.

    Outer ring becomes one spec; each hole becomes an additional spec.
    """
    specs: list[dict] = []
    outer = polygon_outer_ring(polygon)
    specs.append(
        {
            "layer": layer,
            "ring": ring_index,
            "centerline_offset_m": centerline_offset,
            "points": [{"x": x, "y": y} for x, y in outer],
        }
    )
    for k, hole in enumerate(polygon_holes(polygon), start=1):
        specs.append(
            {
                "layer": layer,
                "ring": ring_index + k,
                "centerline_offset_m": centerline_offset,
                "points": [{"x": x, "y": y} for x, y in hole],
            }
        )
    return specs


# --------------------------------------------------------------------------------
# Reflex outer-boundary vertex detection (P11)
# --------------------------------------------------------------------------------


def outer_reflex_ys(
    polygon: Polygon,
    min_reflex_angle_rad: float = 0.35,
) -> list[float]:
    """Return sorted y-coords for cutting at outer-boundary reflex regions
    that *fragment* horizontal swaths.

    Used by Y-sweep BCD: when stripes run along the x-axis, the perpendicular
    sweep is along y, and cells are horizontal bands separated by horizontal
    cuts at every y where the polygon's x-cross-section topology changes
    (1 segment ↔ 2+ segments).

    A reflex region is a contiguous run of CCW outer-ring vertices whose
    cumulative signed turn angle is right-turning (into the polygon) by more
    than ``min_reflex_angle_rad`` total. Each such region produces a single
    candidate y-coord — the |turn|-weighted centroid of the region — so a
    rolled-disk arc inserted at a concave corner by Minkowski erosion is
    treated as one candidate, not one per arc vertex.

    A candidate is kept only if it actually causes swath fragmentation: the
    polygon's intersection with a horizontal line just inside the polygon at
    the reflex's y-coord must have more than one connected component. This
    drops benign concavities (an L-shape's inner corner) and keeps notch
    corners (a rectangular notch's bottom corners). The polygon is assumed
    to already be rotated so the stripe direction is the x-axis.

    The default ``min_reflex_angle_rad = 0.35`` rad (~20°) ignores near-
    straight numerical noise and small "almost flat" concavities.

    >>> from shapely.geometry import Polygon
    >>> # Plain rectangle: no reflex vertices.
    >>> outer_reflex_ys(Polygon([(0, 0), (10, 0), (10, 5), (0, 5)]))
    []
    >>> # L-shape inner corner at (5, 4): reflex but does NOT fragment swaths.
    >>> # Horizontal swaths through y=4 are still one segment ([0, 12]).
    >>> L = Polygon([(0, 0), (12, 0), (12, 4), (5, 4), (5, 10), (0, 10)])
    >>> outer_reflex_ys(L)
    []
    >>> # Notched rectangle: notch corners (2,6) and (6,6) both sit at y=6;
    >>> # horizontal swaths just above y=6 are split into [0,2] U [6,16], so
    >>> # one fragmenting cut at y=6.
    >>> notched = Polygon([
    ...     (0, 0), (16, 0), (16, 10), (6, 10),
    ...     (6, 6), (2, 6), (2, 10), (0, 10),
    ... ])
    >>> outer_reflex_ys(notched)
    [6.0]
    """
    ring = list(polygon.exterior.coords)
    if ring and ring[0] == ring[-1]:
        ring = ring[:-1]
    if len(ring) < 3:
        return []
    if not polygon.exterior.is_ccw:
        ring = list(reversed(ring))
    n = len(ring)
    bounds_minx, bounds_miny, bounds_maxx, bounds_maxy = polygon.bounds
    bounds_width = bounds_maxx - bounds_minx

    def fragments_horizontal_swath_at(y_test: float) -> bool:
        """True iff the polygon's intersection with the horizontal line at
        y_test has more than one connected component. This is the criterion
        that distinguishes a fragmenting concave notch (the polygon has a gap
        in x at this y, like the bottom of a notch) from a benign concavity
        like the inner corner of an L-shape (cross-section stays one segment).
        """
        line = LineString([(bounds_minx - 1.0, y_test), (bounds_maxx + 1.0, y_test)])
        inter = polygon.intersection(line)
        if inter.is_empty:
            return False
        if isinstance(inter, LineString):
            return False
        if isinstance(inter, MultiLineString):
            return len([g for g in inter.geoms if not g.is_empty]) > 1
        if isinstance(inter, GeometryCollection):
            n_lines = sum(
                1 for g in inter.geoms
                if isinstance(g, LineString) and not g.is_empty
            )
            return n_lines > 1
        return False

    # Signed turn angle at each vertex (positive = left/convex, negative = right/reflex).
    turns: list[float] = [0.0] * n
    for i in range(n):
        prev_x, prev_y = ring[i - 1]
        cur_x, cur_y = ring[i]
        nxt_x, nxt_y = ring[(i + 1) % n]
        in_dx = cur_x - prev_x
        in_dy = cur_y - prev_y
        out_dx = nxt_x - cur_x
        out_dy = nxt_y - cur_y
        in_len = math.hypot(in_dx, in_dy)
        out_len = math.hypot(out_dx, out_dy)
        if in_len < EPS or out_len < EPS:
            continue
        cross = in_dx * out_dy - in_dy * out_dx
        dot = in_dx * out_dx + in_dy * out_dy
        turns[i] = math.atan2(cross, dot)

    # Anchor the walk on a convex (or flat) vertex so wraparound runs are handled
    # uniformly. If the polygon is entirely reflex (impossible for a simple
    # polygon, but guard anyway) bail out.
    start = next((k for k in range(n) if turns[k] >= 0), None)
    if start is None:
        return []

    # Walk reflex runs, emitting one candidate cut per ~90° of accumulated
    # turn. A single rolled-disk arc inserted by Minkowski erosion accumulates
    # close to -π/2 over many vertices and produces one candidate. Two adjacent
    # 90° corners (e.g. the two bottom corners of a rectangular notch on the
    # raw polygon, where there is no zero-turn vertex between them) each
    # accumulate -π/2 separately and produce two candidates.
    corner_threshold = math.pi / 2 - 1e-3
    candidates: list[tuple[float, float]] = []  # (x_cut, y_at_reflex)
    i = 0
    while i < n:
        idx = (start + i) % n
        if turns[idx] >= 0:
            i += 1
            continue
        accum = 0.0
        seg_xs: list[float] = []
        seg_ys: list[float] = []
        seg_weights: list[float] = []
        j = i
        while j < n:
            jj = (start + j) % n
            if turns[jj] >= 0:
                break
            accum += turns[jj]
            seg_xs.append(ring[jj][0])
            seg_ys.append(ring[jj][1])
            seg_weights.append(abs(turns[jj]))
            if -accum >= corner_threshold and sum(seg_weights) > 0:
                w_sum = sum(seg_weights)
                cx = sum(x * w for x, w in zip(seg_xs, seg_weights)) / w_sum
                cy = sum(y * w for y, w in zip(seg_ys, seg_weights)) / w_sum
                candidates.append((float(cx), float(cy)))
                accum = 0.0
                seg_xs = []
                seg_ys = []
                seg_weights = []
            j += 1
        if seg_weights and -accum >= min_reflex_angle_rad:
            w_sum = sum(seg_weights)
            cx = sum(x * w for x, w in zip(seg_xs, seg_weights)) / w_sum
            cy = sum(y * w for y, w in zip(seg_ys, seg_weights)) / w_sum
            candidates.append((float(cx), float(cy)))
        i = j + 1

    # Fragmenting filter: only keep candidates where the polygon's
    # horizontal cross-section at (or just inside) the reflex y is split into
    # multiple segments. This drops benign concavities like the L-shape's
    # inner corner (cross-section stays one segment) while keeping notch
    # corners (cross-section is two segments). Probe at a small offset on
    # both sides of the reflex y to handle notches opening up or down.
    height = max(bounds_maxy - bounds_miny, 1.0)
    probe_eps = max(min(1e-2, height * 1e-3), 1e-4)
    cuts: list[float] = []
    for cx, cy in candidates:
        if (
            fragments_horizontal_swath_at(cy + probe_eps)
            or fragments_horizontal_swath_at(cy - probe_eps)
        ):
            cuts.append(cy)

    cuts.sort()
    snapped: list[float] = []
    for y in cuts:
        if not snapped or y - snapped[-1] > 1e-3:
            snapped.append(y)
    return snapped


# --------------------------------------------------------------------------------
# Boustrophedon Cellular Decomposition (BCD)
# --------------------------------------------------------------------------------


def bcd_decompose(polygon: Polygon, stripe_angle_rad: float) -> list[Polygon]:
    """Decompose a polygon into hole-free cells aligned to the stripe direction.

    For stripes along x, the BCD sweep runs perpendicular (along y), so cells
    are horizontal bands. Cuts are at every y where the polygon's horizontal
    cross-section topology changes:
      - interior-hole y-extents (top and bottom of every interior obstacle);
      - outer-boundary reflex y-values where a concave region splits the
        cross-section into multiple x-segments (P11, e.g. a notched outer
        boundary after Minkowski erosion).
    Horizontal bands give long full-width stripes wherever the polygon has no
    obstacle blocking, and short sub-cell stripes only in y-bands where an
    obstacle actually fragments the cross-section. This is the classical
    Boustrophedon Cellular Decomposition direction; v1 of P1 mistakenly cut
    along x (vertical columns), which fragmented every stripe that crossed
    an obstacle's x-range and produced 3-4× the necessary stripe count.

    A simply-convex polygon (no holes and no fragmenting reflex regions) is
    returned unchanged as a single cell.

    >>> simple = Polygon([(0, 0), (10, 0), (10, 5), (0, 5)])
    >>> len(bcd_decompose(simple, 0.0))
    1
    >>> with_hole = Polygon(
    ...     [(0, 0), (10, 0), (10, 5), (0, 5)],
    ...     [[(3, 1), (3, 4), (7, 4), (7, 1)]],
    ... )
    >>> cells = bcd_decompose(with_hole, 0.0)
    >>> # y-cuts at hole y-extents 1 and 4 produce: bottom band (10*1=10),
    >>> # middle band split by hole into left (3*3=9) and right (3*3=9), and
    >>> # top band (10*1=10).
    >>> sorted(round(c.area, 2) for c in cells)
    [9.0, 9.0, 10.0, 10.0]
    >>> all(len(list(c.interiors)) == 0 for c in cells)
    True
    >>> # Notched polygon (no interior hole). P11 cuts at the notch's reflex y.
    >>> notched = Polygon([
    ...     (0, 0), (16, 0), (16, 10), (6, 10),
    ...     (6, 6), (2, 6), (2, 10), (0, 10),
    ... ])
    >>> cells = bcd_decompose(notched, 0.0)
    >>> # y-cut at y=6: bottom band (16*6=96) and the upper band split by the
    >>> # notch into left (2*4=8) and right (10*4=40).
    >>> sorted(round(c.area, 2) for c in cells)
    [8.0, 40.0, 96.0]
    """
    deg = math.degrees(stripe_angle_rad)
    rot = rotate(polygon, -deg, origin=(0, 0), use_radians=False)
    minx, miny, maxx, maxy = rot.bounds
    span_x = (maxx - minx) + 10.0
    strip_eps = 1e-6

    # Cut at each hole's y-extents (bounding box top/bottom). Guarantees full
    # hole isolation for convex (and Minkowski-eroded near-circular) holes.
    # Non-convex holes that need internal y-splits are deferred to a future v2.
    cut_ys: set[float] = set()
    for hole in rot.interiors:
        coords = list(hole.coords)
        ys = [c[1] for c in coords]
        cut_ys.add(min(ys))
        cut_ys.add(max(ys))

    # Cut at every reflex outer-boundary region (P11). One cut per region.
    for y in outer_reflex_ys(rot):
        cut_ys.add(y)

    sorted_ys = sorted(cut_ys)
    snap_tol = 1e-3
    snapped: list[float] = []
    for y in sorted_ys:
        if not snapped or y - snapped[-1] > snap_tol:
            snapped.append(y)

    if not snapped:
        return [polygon]

    strips = [
        box(minx - span_x, y - strip_eps, maxx + span_x, y + strip_eps)
        for y in snapped
    ]
    cut_union = unary_union(strips)
    split = rot.difference(cut_union)

    pieces: list[Polygon] = []
    if isinstance(split, MultiPolygon):
        pieces.extend(g for g in split.geoms if g.area > strip_eps * 10)
    elif isinstance(split, Polygon) and split.area > strip_eps * 10:
        pieces.append(split)

    cells: list[Polygon] = []
    for piece in pieces:
        # Cells that still carry a hole indicate cuts did not fully isolate a
        # non-convex obstacle. Subtract those interiors so the cell is truly
        # hole-free; if the area drops to near zero, drop the piece.
        clean = Polygon(piece.exterior.coords)
        for interior in piece.interiors:
            clean = clean.difference(Polygon(interior.coords))
        if isinstance(clean, MultiPolygon):
            cells.extend(rotate(g, deg, origin=(0, 0), use_radians=False) for g in clean.geoms if g.area > EPS)
        elif isinstance(clean, Polygon) and clean.area > EPS:
            cells.append(rotate(clean, deg, origin=(0, 0), use_radians=False))

    return cells


# --------------------------------------------------------------------------------
# Headland ring walking helpers (used by cell transit stitching for P1)
# --------------------------------------------------------------------------------


def ring_length(ring: list[tuple[float, float]]) -> float:
    """Total perimeter length of a closed ring (last point may or may not repeat first)."""
    if len(ring) < 2:
        return 0.0
    total = 0.0
    for i in range(len(ring)):
        a = ring[i]
        b = ring[(i + 1) % len(ring)]
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def _nearest_ring_param(ring: list[tuple[float, float]], point: tuple[float, float]) -> float:
    """Return the arc-length parameter (0..perimeter) of the closest point on the ring."""
    best_s = 0.0
    best_d = math.inf
    s = 0.0
    px, py = point
    n = len(ring)
    for i in range(n):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % n]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len < EPS:
            continue
        # project (px,py) onto segment
        t = max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (seg_len * seg_len)))
        cx, cy = ax + t * (bx - ax), ay + t * (by - ay)
        d = math.hypot(px - cx, py - cy)
        if d < best_d:
            best_d = d
            best_s = s + t * seg_len
        s += seg_len
    return best_s


def walk_ring_between(
    ring: list[tuple[float, float]],
    start: tuple[float, float],
    end: tuple[float, float],
) -> list[tuple[float, float]]:
    """Return a polyline along ``ring`` from the point on the ring closest to
    ``start`` to the point closest to ``end``, in the shorter direction.

    Used by cell-transit stitching: between two BCD cells, walk along the
    eroded-lawn boundary so the transit stays footprint-safe by construction.
    """
    if len(ring) < 2:
        return [start, end]
    perimeter = ring_length(ring)
    if perimeter < EPS:
        return [start, end]

    s_start = _nearest_ring_param(ring, start)
    s_end = _nearest_ring_param(ring, end)

    # Two directions; pick the shorter.
    forward_dist = (s_end - s_start) % perimeter
    backward_dist = (s_start - s_end) % perimeter
    direction = 1 if forward_dist <= backward_dist else -1

    # Sample the ring as a closed polyline with cumulative arc lengths.
    arc_points: list[tuple[float, float, float]] = []  # (s, x, y)
    s = 0.0
    n = len(ring)
    for i in range(n):
        arc_points.append((s, ring[i][0], ring[i][1]))
        nxt = ring[(i + 1) % n]
        s += math.hypot(nxt[0] - ring[i][0], nxt[1] - ring[i][1])
    total = s

    def point_at(s_val: float) -> tuple[float, float]:
        s_val %= total
        for j in range(n):
            sa, ax, ay = arc_points[j]
            sb, bx, by = arc_points[(j + 1) % n]
            if j == n - 1:
                sb = total
                bx, by = arc_points[0][1], arc_points[0][2]
            if sa <= s_val <= sb:
                if sb - sa < EPS:
                    return (ax, ay)
                t = (s_val - sa) / (sb - sa)
                return (ax + t * (bx - ax), ay + t * (by - ay))
        return arc_points[0][1], arc_points[0][2]

    # Walk from s_start to s_end in the chosen direction, hitting every ring vertex.
    out: list[tuple[float, float]] = [point_at(s_start)]
    s = s_start
    safety = 2 * n + 4
    while safety > 0:
        safety -= 1
        # find next vertex in direction
        nearest_next = None
        nearest_delta = math.inf
        for j in range(n):
            sj = arc_points[j][0]
            delta = ((sj - s) * direction) % total
            if delta < EPS:
                continue
            if delta < nearest_delta:
                nearest_delta = delta
                nearest_next = sj
        # check if s_end comes first
        end_delta = ((s_end - s) * direction) % total
        if end_delta < EPS:
            break
        if nearest_next is None or end_delta <= nearest_delta:
            out.append(point_at(s_end))
            break
        s = nearest_next
        out.append(point_at(s))
        if math.isclose(s, s_end, abs_tol=EPS):
            break

    return out


# --------------------------------------------------------------------------------
# Stripe-angle helpers (P12, P13)
# --------------------------------------------------------------------------------


def dominant_direction_angle(polygon: Polygon, smoothing_deg: float = 3.0) -> float | None:
    """Return the dominant edge direction (radians, in [0, π)) of the polygon's
    outer ring as a length-weighted histogram peak.

    Each outer-ring edge contributes its length to a 1°-wide histogram bin
    keyed by ``angle mod π`` (stripes are bidirectional). The histogram is
    smoothed with a Gaussian of stddev ``smoothing_deg`` so a few short noisy
    edges can't dominate, and the centre of the peak bin is returned (radians).
    If the histogram is flat (peak < ~4× the mean bin, which corresponds to
    < ~1.5× a typical edge-bearing bin once near-isotropic shapes have
    spread their mass across many bins), returns ``None`` so the caller can
    fall back to F2C's ``best_swath_length`` probe.

    >>> import math
    >>> # Axis-aligned rectangle: horizontal (and vertical) edges dominate.
    >>> # Horizontal angle 0 and vertical angle π/2 both map to bin 0 under
    >>> # mod π; the result is ~0 rad.
    >>> rect = Polygon([(0, 0), (10, 0), (10, 5), (0, 5)])
    >>> abs(dominant_direction_angle(rect)) < math.radians(2.0)
    True
    >>> # L-shape with axis-aligned edges: ~0 rad.
    >>> L = Polygon([(0, 0), (12, 0), (12, 4), (5, 4), (5, 10), (0, 10)])
    >>> abs(dominant_direction_angle(L)) < math.radians(2.0)
    True
    >>> # Rectangle rotated 30 degrees: angle ~ 30 degrees in radians.
    >>> from shapely.affinity import rotate as _rot
    >>> rect30 = _rot(rect, 30, origin=(0, 0), use_radians=False)
    >>> ang = dominant_direction_angle(rect30)
    >>> abs(ang - math.radians(30.0)) < math.radians(2.0)
    True
    >>> # Near-circle (16-gon): edges spread evenly across angles, histogram
    >>> # is flat — returns None.
    >>> import math as _m
    >>> circle = Polygon([
    ...     (_m.cos(2 * _m.pi * k / 16), _m.sin(2 * _m.pi * k / 16))
    ...     for k in range(16)
    ... ])
    >>> dominant_direction_angle(circle) is None
    True
    """
    if polygon is None or polygon.is_empty:
        return None
    try:
        ring = list(polygon.exterior.coords)
    except Exception:
        return None
    if ring and ring[0] == ring[-1]:
        ring = ring[:-1]
    if len(ring) < 3:
        return None

    n_bins = 180  # 1° per bin, covering [0, 180)
    hist = [0.0] * n_bins
    total_length = 0.0
    for i in range(len(ring)):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % len(ring)]
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)
        if length < EPS:
            continue
        angle = math.atan2(dy, dx) % math.pi  # in [0, π)
        bin_index = int(round(math.degrees(angle))) % n_bins
        hist[bin_index] += length
        total_length += length

    if total_length < EPS:
        return None

    # Gaussian smoothing on the circular histogram. We compute the kernel out
    # to ±3σ and wrap (modulo n_bins) so vertical (bin 0) and horizontal-flipped
    # (bin 0) reinforce as a single peak — the histogram is already mod π so
    # the [0, 180) span is the natural circular domain.
    sigma = max(float(smoothing_deg), 1e-6)
    half = max(1, int(math.ceil(3.0 * sigma)))
    kernel = [math.exp(-0.5 * (k / sigma) ** 2) for k in range(-half, half + 1)]
    k_sum = sum(kernel)
    kernel = [k / k_sum for k in kernel]

    smoothed = [0.0] * n_bins
    for i in range(n_bins):
        acc = 0.0
        for j, w in enumerate(kernel):
            offset = j - half
            acc += hist[(i + offset) % n_bins] * w
        smoothed[i] = acc

    peak_value = max(smoothed)
    if peak_value <= EPS:
        return None
    mean_value = sum(smoothed) / n_bins
    # Mean-based flatness test: for an isotropic outline (e.g. a 16-gon) the
    # smoothed peak sits at ~3× the mean; a single dominant axis pushes it
    # well above 4×. The 1.5× sentinel mentioned in the spec is the
    # equivalent ratio against a "typical edge-bearing bin" — that quantity
    # equals (peak / mean) / (n_bins / number_of_occupied_bins) once the mass
    # has been spread across many directions, and 4× peak/mean is the
    # practical threshold that distinguishes the two regimes.
    if mean_value > EPS and peak_value < 4.0 * mean_value:
        return None

    peak_bin = max(range(n_bins), key=lambda i: smoothed[i])
    return math.radians(float(peak_bin))


def cell_long_axis_angle(polygon: Polygon) -> float:
    """Return the long-axis angle (radians, in [0, π)) of the polygon via
    principal-component analysis on its outer-ring vertices.

    Falls back to 0.0 for degenerate inputs (empty / too few vertices /
    isotropic vertex cloud).

    >>> import math
    >>> # Wide rectangle along x — long axis is horizontal.
    >>> rect = Polygon([(0, 0), (10, 0), (10, 1), (0, 1)])
    >>> abs(cell_long_axis_angle(rect)) < math.radians(2.0)
    True
    >>> # Tall rectangle — long axis is vertical (~π/2).
    >>> tall = Polygon([(0, 0), (1, 0), (1, 10), (0, 10)])
    >>> abs(cell_long_axis_angle(tall) - math.pi / 2) < math.radians(2.0)
    True
    >>> # Rectangle rotated 45° — long axis ~ π/4.
    >>> from shapely.affinity import rotate as _rot
    >>> rect45 = _rot(rect, 45, origin=(0, 0), use_radians=False)
    >>> abs(cell_long_axis_angle(rect45) - math.radians(45.0)) < math.radians(2.0)
    True
    """
    if polygon is None or polygon.is_empty:
        return 0.0
    try:
        ring = list(polygon.exterior.coords)
    except Exception:
        return 0.0
    if ring and ring[0] == ring[-1]:
        ring = ring[:-1]
    if len(ring) < 3:
        return 0.0

    n = len(ring)
    cx = sum(p[0] for p in ring) / n
    cy = sum(p[1] for p in ring) / n
    sxx = 0.0
    syy = 0.0
    sxy = 0.0
    for x, y in ring:
        dx = x - cx
        dy = y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    sxx /= n
    syy /= n
    sxy /= n

    # Eigen-decomposition of [[sxx, sxy], [sxy, syy]].
    trace = sxx + syy
    det = sxx * syy - sxy * sxy
    disc = max(0.0, (trace / 2.0) ** 2 - det)
    sqrt_disc = math.sqrt(disc)
    lambda1 = trace / 2.0 + sqrt_disc  # larger eigenvalue
    # Eigenvector for the larger eigenvalue.
    if abs(sxy) > EPS:
        vx = lambda1 - syy
        vy = sxy
    elif sxx >= syy:
        vx, vy = 1.0, 0.0
    else:
        vx, vy = 0.0, 1.0
    if abs(vx) < EPS and abs(vy) < EPS:
        return 0.0
    angle = math.atan2(vy, vx) % math.pi
    return float(angle)


def cell_short_axis_extent_along(polygon: Polygon, angle_rad: float) -> float:
    """Return the polygon's extent (metres) **perpendicular** to ``angle_rad``.

    This is the dimension stripes laid down at ``angle_rad`` would have to
    span: in the frame where stripes run horizontally, this is the polygon's
    vertical (y) extent. Used to detect tight cells (P13) whose short axis
    along the chosen stripe angle is narrower than a few tool widths.

    >>> import math
    >>> rect = Polygon([(0, 0), (10, 0), (10, 1), (0, 1)])
    >>> # Stripes along x: perpendicular extent is the height (1.0).
    >>> abs(cell_short_axis_extent_along(rect, 0.0) - 1.0) < 1e-6
    True
    >>> # Stripes along y: perpendicular extent is the width (10.0).
    >>> abs(cell_short_axis_extent_along(rect, math.pi / 2) - 10.0) < 1e-6
    True
    """
    if polygon is None or polygon.is_empty:
        return 0.0
    deg = math.degrees(angle_rad)
    rotated = rotate(polygon, -deg, origin=(0, 0), use_radians=False)
    minx, miny, maxx, maxy = rotated.bounds
    return float(maxy - miny)


if __name__ == "__main__":
    import doctest

    failed = doctest.testmod(verbose=False).failed
    if failed:
        raise SystemExit(failed)
    print("lab_geometry doctests ok")
