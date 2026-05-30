// Created by Claude for MartinHaghani fork of OpenMower
//
// Sparse 2D grid that accumulates the "painted" area swept by a moving line
// segment (e.g. the right side of the mower body) over time. Used by the
// area recorder to build a polygon by rasterising the swept rake region and
// extracting its outer contour at the end of recording.
//
// Cells live in world-frame coordinates with a fixed cell size and an
// implicit origin at (0,0). The grid is sparse: only painted cells take
// memory, so it scales with painted area, not with workspace size.

#ifndef SRC_PAINTED_AREA_H
#define SRC_PAINTED_AREA_H

#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

class PaintedArea {
 public:
  using PointXY = std::pair<double, double>;

  explicit PaintedArea(double cell_size_m);

  // Rasterise a single line segment into the grid (used for the first tick
  // when there is no previous segment to sweep from).
  void paint_segment(double ax, double ay, double bx, double by);

  // Rasterise the convex quadrilateral swept by the right-side segment
  // between two consecutive ticks. Corners must be supplied in CCW or CW
  // order; the rasteriser detects winding from the signed area.
  //
  //   prev_rear  o─────────────o prev_front
  //              │             │
  //              │ (swept)     │
  //              │             │
  //   curr_rear  o─────────────o curr_front
  void paint_swept_quad(double prev_rear_x, double prev_rear_y,
                        double prev_front_x, double prev_front_y,
                        double curr_rear_x, double curr_rear_y,
                        double curr_front_x, double curr_front_y);

  bool empty() const { return cells_.empty(); }
  std::size_t cell_count() const { return cells_.size(); }
  double cell_size() const { return cell_size_; }

  // Inclusive bounds of painted cells. Returns false if empty.
  bool get_bounds(int& min_i, int& min_j, int& max_i, int& max_j) const;

  // Extract the outer contour of the painted region as a closed polyline in
  // world coordinates (first point repeated at the end). Uses Moore-neighbor
  // tracing starting from the bottom-most, left-most painted cell. The
  // polygon traces the outer side of the cells (cell corners, not centers)
  // so it tightly wraps the painted region.
  std::vector<PointXY> extract_outer_contour() const;

  // Reduce vertex count using the Douglas-Peucker algorithm with the given
  // perpendicular-distance threshold. The polygon is treated as closed (first
  // == last). Result also has first == last; minimum returned size is 4.
  static std::vector<PointXY> douglas_peucker(const std::vector<PointXY>& points, double epsilon);

  // Helpers exposed for OccupancyGrid publishing or contour drawing.
  int world_to_cell_i(double x) const;
  int world_to_cell_j(double y) const;
  double cell_to_world_x(int i) const;  // cell center
  double cell_to_world_y(int j) const;
  double cell_corner_x(int i) const;  // cell lower-left corner
  double cell_corner_y(int j) const;
  bool is_painted(int i, int j) const { return cells_.count(pack(i, j)) > 0; }

 private:
  static int64_t pack(int i, int j) {
    return (static_cast<int64_t>(i) << 32) | (static_cast<uint32_t>(j));
  }
  static void unpack(int64_t key, int& i, int& j) {
    i = static_cast<int>(static_cast<int32_t>(key >> 32));
    j = static_cast<int>(static_cast<int32_t>(key & 0xFFFFFFFFu));
  }

  void mark_cell(int i, int j) { cells_.insert(pack(i, j)); }
  void paint_triangle(double ax, double ay, double bx, double by, double cx, double cy);

  double cell_size_;
  std::unordered_set<int64_t> cells_;
};

#endif  // SRC_PAINTED_AREA_H
