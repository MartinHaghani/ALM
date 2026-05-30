// Created by Claude for MartinHaghani fork of OpenMower

#include "PaintedArea.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stack>
#include <unordered_map>

namespace {

int64_t pack_corner(int i, int j) {
  return (static_cast<int64_t>(i) << 32) | static_cast<uint32_t>(j);
}

// Perpendicular distance from point p to the line through a and b.
double perpendicular_distance(const PaintedArea::PointXY& p, const PaintedArea::PointXY& a,
                              const PaintedArea::PointXY& b) {
  const double dx = b.first - a.first;
  const double dy = b.second - a.second;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-18) {
    const double ex = p.first - a.first;
    const double ey = p.second - a.second;
    return std::sqrt(ex * ex + ey * ey);
  }
  const double cross = (p.first - a.first) * dy - (p.second - a.second) * dx;
  return std::abs(cross) / std::sqrt(len2);
}

// Iterative Douglas-Peucker on an open polyline [first, last] inclusive.
// Returns indices into `points` that should be kept (always includes first
// and last). Uses an explicit stack to avoid recursion overflow on long
// polylines.
void douglas_peucker_open(const std::vector<PaintedArea::PointXY>& points, double epsilon,
                          std::vector<bool>& keep) {
  if (points.size() < 2) return;
  keep[0] = true;
  keep[points.size() - 1] = true;
  std::stack<std::pair<std::size_t, std::size_t>> stack;
  stack.emplace(0, points.size() - 1);
  while (!stack.empty()) {
    const std::size_t lo = stack.top().first;
    const std::size_t hi = stack.top().second;
    stack.pop();
    if (hi <= lo + 1) continue;
    double max_d = 0.0;
    std::size_t max_idx = lo;
    for (std::size_t i = lo + 1; i < hi; ++i) {
      const double d = perpendicular_distance(points[i], points[lo], points[hi]);
      if (d > max_d) {
        max_d = d;
        max_idx = i;
      }
    }
    if (max_d > epsilon) {
      keep[max_idx] = true;
      stack.emplace(lo, max_idx);
      stack.emplace(max_idx, hi);
    }
  }
}

}  // namespace

PaintedArea::PaintedArea(double cell_size_m) : cell_size_(cell_size_m) {
  if (cell_size_ <= 0.0) {
    cell_size_ = 0.05;
  }
}

int PaintedArea::world_to_cell_i(double x) const {
  return static_cast<int>(std::floor(x / cell_size_));
}

int PaintedArea::world_to_cell_j(double y) const {
  return static_cast<int>(std::floor(y / cell_size_));
}

double PaintedArea::cell_to_world_x(int i) const {
  return (static_cast<double>(i) + 0.5) * cell_size_;
}

double PaintedArea::cell_to_world_y(int j) const {
  return (static_cast<double>(j) + 0.5) * cell_size_;
}

double PaintedArea::cell_corner_x(int i) const {
  return static_cast<double>(i) * cell_size_;
}

double PaintedArea::cell_corner_y(int j) const {
  return static_cast<double>(j) * cell_size_;
}

bool PaintedArea::get_bounds(int& min_i, int& min_j, int& max_i, int& max_j) const {
  if (cells_.empty()) return false;
  min_i = min_j = std::numeric_limits<int>::max();
  max_i = max_j = std::numeric_limits<int>::min();
  for (const auto key : cells_) {
    int i, j;
    unpack(key, i, j);
    min_i = std::min(min_i, i);
    max_i = std::max(max_i, i);
    min_j = std::min(min_j, j);
    max_j = std::max(max_j, j);
  }
  return true;
}

void PaintedArea::paint_segment(double ax, double ay, double bx, double by) {
  // DDA line rasteriser stepping in whichever axis dominates; paints every
  // cell the segment passes through.
  const int i0 = world_to_cell_i(ax);
  const int j0 = world_to_cell_j(ay);
  const int i1 = world_to_cell_i(bx);
  const int j1 = world_to_cell_j(by);
  const int di = std::abs(i1 - i0);
  const int dj = std::abs(j1 - j0);
  const int steps = std::max(di, dj) + 1;
  for (int k = 0; k <= steps; ++k) {
    const double t = (steps == 0) ? 0.0 : static_cast<double>(k) / steps;
    const double x = ax + t * (bx - ax);
    const double y = ay + t * (by - ay);
    mark_cell(world_to_cell_i(x), world_to_cell_j(y));
  }
}

void PaintedArea::paint_triangle(double ax, double ay, double bx, double by, double cx, double cy) {
  const double area2 = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
  if (std::abs(area2) < 1e-12) {
    paint_segment(ax, ay, bx, by);
    paint_segment(bx, by, cx, cy);
    paint_segment(cx, cy, ax, ay);
    return;
  }
  const double sign = (area2 > 0.0) ? 1.0 : -1.0;

  const double min_x = std::min({ax, bx, cx});
  const double max_x = std::max({ax, bx, cx});
  const double min_y = std::min({ay, by, cy});
  const double max_y = std::max({ay, by, cy});
  const int min_i = world_to_cell_i(min_x);
  const int max_i = world_to_cell_i(max_x);
  const int min_j = world_to_cell_j(min_y);
  const int max_j = world_to_cell_j(max_y);

  for (int j = min_j; j <= max_j; ++j) {
    const double py = cell_to_world_y(j);
    for (int i = min_i; i <= max_i; ++i) {
      const double px = cell_to_world_x(i);
      const double e0 = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
      const double e1 = (cx - bx) * (py - by) - (cy - by) * (px - bx);
      const double e2 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx);
      if (sign * e0 >= 0.0 && sign * e1 >= 0.0 && sign * e2 >= 0.0) {
        mark_cell(i, j);
      }
    }
  }
}

void PaintedArea::paint_swept_quad(double prev_rear_x, double prev_rear_y, double prev_front_x,
                                   double prev_front_y, double curr_rear_x, double curr_rear_y,
                                   double curr_front_x, double curr_front_y) {
  // Split the quadrilateral into two triangles using the prev_rear -> curr_front diagonal.
  paint_triangle(prev_rear_x, prev_rear_y, prev_front_x, prev_front_y, curr_front_x, curr_front_y);
  paint_triangle(prev_rear_x, prev_rear_y, curr_front_x, curr_front_y, curr_rear_x, curr_rear_y);
  // Also rasterise the current segment so the leading edge of the rake is always represented even
  // when the quad is degenerate (mower stationary; prev == curr).
  paint_segment(curr_rear_x, curr_rear_y, curr_front_x, curr_front_y);
}

std::vector<PaintedArea::PointXY> PaintedArea::extract_outer_contour() const {
  if (cells_.empty()) return {};

  // Build the set of boundary edges. For each painted cell, each side
  // bordering an unpainted cell contributes one directed edge oriented so
  // that the painted region lies on the LEFT of the edge. The resulting set
  // is a collection of closed loops; we pick the largest as the outer
  // contour.
  //
  // Edge orientation by side, with cell (i, j) occupying world rectangle
  // [i*cs, (i+1)*cs) x [j*cs, (j+1)*cs):
  //   Bottom side  (neighbour (i, j-1) unpainted): edge corner(i,j)   -> corner(i+1,j)
  //   Right  side  (neighbour (i+1, j) unpainted): edge corner(i+1,j) -> corner(i+1,j+1)
  //   Top    side  (neighbour (i, j+1) unpainted): edge corner(i+1,j+1) -> corner(i,j+1)
  //   Left   side  (neighbour (i-1, j) unpainted): edge corner(i,j+1) -> corner(i,j)
  //
  // Each corner has at most two boundary edges touching it (one incoming,
  // one outgoing), so the next-edge lookup is unambiguous.

  std::unordered_map<int64_t, std::pair<int, int>> next_corner;  // start corner -> end corner
  next_corner.reserve(cells_.size() * 2);

  auto add_edge = [&](int ax, int ay, int bx, int by) {
    next_corner[pack_corner(ax, ay)] = {bx, by};
  };

  for (const auto key : cells_) {
    int i, j;
    unpack(key, i, j);
    if (!is_painted(i, j - 1)) add_edge(i, j, i + 1, j);
    if (!is_painted(i + 1, j)) add_edge(i + 1, j, i + 1, j + 1);
    if (!is_painted(i, j + 1)) add_edge(i + 1, j + 1, i, j + 1);
    if (!is_painted(i - 1, j)) add_edge(i, j + 1, i, j);
  }

  if (next_corner.empty()) return {};

  // Trace every connected loop, then return the largest by perimeter length
  // (in number of edges, which is equivalent for an axis-aligned grid).
  std::vector<std::vector<std::pair<int, int>>> loops;
  while (!next_corner.empty()) {
    auto it = next_corner.begin();
    int sx, sy;
    unpack(it->first, sx, sy);
    std::vector<std::pair<int, int>> loop;
    loop.emplace_back(sx, sy);
    int cx = sx, cy = sy;
    while (true) {
      auto found = next_corner.find(pack_corner(cx, cy));
      if (found == next_corner.end()) break;
      const auto next = found->second;
      next_corner.erase(found);
      cx = next.first;
      cy = next.second;
      if (cx == sx && cy == sy) break;
      loop.emplace_back(cx, cy);
    }
    if (loop.size() >= 3) loops.push_back(std::move(loop));
  }

  if (loops.empty()) return {};

  std::size_t best = 0;
  for (std::size_t k = 1; k < loops.size(); ++k) {
    if (loops[k].size() > loops[best].size()) best = k;
  }

  std::vector<PointXY> result;
  result.reserve(loops[best].size() + 1);
  for (const auto& corner : loops[best]) {
    result.emplace_back(cell_corner_x(corner.first), cell_corner_y(corner.second));
  }
  // Close the polygon explicitly.
  result.push_back(result.front());
  return result;
}

std::vector<PaintedArea::PointXY> PaintedArea::douglas_peucker(
    const std::vector<PaintedArea::PointXY>& points, double epsilon) {
  if (points.size() < 4) return points;

  // Treat the closed polygon (first == last) as two open polylines split at
  // the two indices that lie furthest apart. This gives Douglas-Peucker a
  // well-defined endpoints pair for each half.
  const std::size_t n = points.size() - 1;  // exclude duplicated closing point
  std::size_t a = 0, b = 0;
  double best = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double dx = points[i].first - points[0].first;
    const double dy = points[i].second - points[0].second;
    const double d = dx * dx + dy * dy;
    if (d > best) {
      best = d;
      b = i;
    }
  }
  best = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double dx = points[i].first - points[b].first;
    const double dy = points[i].second - points[b].second;
    const double d = dx * dx + dy * dy;
    if (d > best) {
      best = d;
      a = i;
    }
  }
  if (a > b) std::swap(a, b);
  if (a == b) {
    a = 0;
    b = n / 2;
  }

  std::vector<PointXY> first_half(points.begin() + a, points.begin() + b + 1);
  std::vector<PointXY> second_half;
  second_half.reserve(n - (b - a) + 1);
  for (std::size_t i = b; i < n; ++i) second_half.push_back(points[i]);
  for (std::size_t i = 0; i <= a; ++i) second_half.push_back(points[i]);

  std::vector<bool> keep_first(first_half.size(), false);
  std::vector<bool> keep_second(second_half.size(), false);
  douglas_peucker_open(first_half, epsilon, keep_first);
  douglas_peucker_open(second_half, epsilon, keep_second);

  std::vector<PointXY> result;
  result.reserve(points.size());
  for (std::size_t i = 0; i < first_half.size(); ++i) {
    if (keep_first[i]) result.push_back(first_half[i]);
  }
  // Skip the first point of second_half because it duplicates the last of
  // first_half (both equal points[b]).
  for (std::size_t i = 1; i < second_half.size(); ++i) {
    if (keep_second[i]) result.push_back(second_half[i]);
  }
  if (result.size() < 3) {
    // Degenerate; fall back to the original.
    return points;
  }
  // Close the polygon explicitly.
  result.push_back(result.front());
  return result;
}
