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
from shapely.geometry import LineString, MultiPolygon, Polygon, box
from shapely.ops import unary_union


__all__ = [
    "footprint_disk_radius",
    "erode_lawn",
    "polygon_outer_ring",
    "polygon_holes",
    "polygon_to_ring_dicts",
    "bcd_decompose",
    "ring_length",
    "walk_ring_between",
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
# Boustrophedon Cellular Decomposition (BCD)
# --------------------------------------------------------------------------------


def bcd_decompose(polygon: Polygon, stripe_angle_rad: float) -> list[Polygon]:
    """Decompose a polygon with holes into hole-free cells aligned to the stripe direction.

    Algorithm: rotate so stripes run along x, cut the polygon along vertical
    strips through each hole's x-extents (and through every hole vertex's x
    for robustness on non-convex holes), split into connected components,
    rotate back. Every returned cell has no interior rings.

    A polygon with no holes is returned as a single cell unchanged.

    >>> simple = Polygon([(0, 0), (10, 0), (10, 5), (0, 5)])
    >>> len(bcd_decompose(simple, 0.0))
    1
    >>> with_hole = Polygon(
    ...     [(0, 0), (10, 0), (10, 5), (0, 5)],
    ...     [[(3, 1), (3, 4), (7, 4), (7, 1)]],
    ... )
    >>> cells = bcd_decompose(with_hole, 0.0)
    >>> sorted(round(c.area, 2) for c in cells)  # 3*5 + 4*1 + 4*1 + 3*5 == 38
    [4.0, 4.0, 15.0, 15.0]
    >>> all(len(list(c.interiors)) == 0 for c in cells)
    True
    """
    if not list(polygon.interiors):
        return [polygon]

    deg = math.degrees(stripe_angle_rad)
    rot = rotate(polygon, -deg, origin=(0, 0), use_radians=False)
    minx, miny, maxx, maxy = rot.bounds
    span_y = (maxy - miny) + 10.0
    strip_eps = 1e-6

    # Cut only at each hole's x-extents (bounding box left/right). This guarantees
    # full hole isolation for convex (and Minkowski-eroded near-circular) holes
    # while avoiding the over-decomposition that would result from cutting at
    # every interior hole vertex when the hole has many sampled boundary points.
    # Non-convex holes that need internal splits are deferred to a future BCD v2.
    cut_xs: set[float] = set()
    for hole in rot.interiors:
        coords = list(hole.coords)
        xs = [c[0] for c in coords]
        cut_xs.add(min(xs))
        cut_xs.add(max(xs))

    # Snap nearly-equal cuts together to avoid sliver cells. Use a wider snap
    # tolerance so adjacent rounded-hole extremes from different holes do not
    # accidentally generate twin cuts within a centimetre of each other.
    sorted_xs = sorted(cut_xs)
    snap_tol = 1e-3
    snapped: list[float] = []
    for x in sorted_xs:
        if not snapped or x - snapped[-1] > snap_tol:
            snapped.append(x)

    if not snapped:
        return [polygon]

    strips = [
        box(x - strip_eps, miny - span_y, x + strip_eps, maxy + span_y)
        for x in snapped
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


if __name__ == "__main__":
    import doctest

    failed = doctest.testmod(verbose=False).failed
    if failed:
        raise SystemExit(failed)
    print("lab_geometry doctests ok")
