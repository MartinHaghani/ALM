// Full-footprint swept area geometry for mower area recording.
//
// This file is part of OpenMower.
#include "SweptAreaRecorder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <utility>

#include "clipper2/clipper.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace mower_logic {
namespace area_recording {
namespace {

constexpr double kClipperScale = 1000.0;  // millimeters
constexpr double kMinAreaM2 = 1e-6;
constexpr double kPi = 3.14159265358979323846;

using Clipper2Lib::ClipType;
using Clipper2Lib::FillRule;
using Clipper2Lib::Path64;
using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;
using Clipper2Lib::PolyPath64;
using Clipper2Lib::PolyTree64;

bool finitePoint(double x, double y) {
  return std::isfinite(x) && std::isfinite(y);
}

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromPose(const geometry_msgs::Pose& pose) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double signedAreaM2(const std::vector<geometry_msgs::Point32>& points) {
  if (points.size() < 3) {
    return 0.0;
  }
  double area = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& a = points[i];
    const auto& b = points[(i + 1) % points.size()];
    area += static_cast<double>(a.x) * static_cast<double>(b.y) -
            static_cast<double>(b.x) * static_cast<double>(a.y);
  }
  return 0.5 * area;
}

Path64 stripDuplicateVertices(Path64 path) {
  Path64 result;
  result.reserve(path.size());
  for (const auto& point : path) {
    if (result.empty() || result.back() != point) {
      result.push_back(point);
    }
  }
  while (result.size() > 1 && result.front() == result.back()) {
    result.pop_back();
  }
  return result;
}

std::size_t uniqueVertexCount(const Path64& path) {
  Path64 unique;
  unique.reserve(path.size());
  for (const auto& point : path) {
    if (std::find(unique.begin(), unique.end(), point) == unique.end()) {
      unique.push_back(point);
    }
  }
  return unique.size();
}

bool validClipperPath(const Path64& path, double min_area_m2) {
  if (uniqueVertexCount(path) < 3) {
    return false;
  }
  return std::abs(Clipper2Lib::Area(path)) / (kClipperScale * kClipperScale) >= min_area_m2;
}

double pathAreaM2(const Path64& path) {
  return std::abs(Clipper2Lib::Area(path)) / (kClipperScale * kClipperScale);
}

double pathPerimeterM(const Path64& path) {
  if (path.size() < 2) {
    return 0.0;
  }

  double perimeter = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto& a = path[i];
    const auto& b = path[(i + 1) % path.size()];
    perimeter += std::hypot(static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y)) / kClipperScale;
  }
  return perimeter;
}

void setPathBoundsDiagnostics(const Path64& path, SweptAreaResult& result) {
  if (path.empty()) {
    return;
  }

  int64_t min_x = path.front().x;
  int64_t max_x = path.front().x;
  int64_t min_y = path.front().y;
  int64_t max_y = path.front().y;
  for (const auto& point : path) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  result.selected_bbox_width_m = static_cast<double>(max_x - min_x) / kClipperScale;
  result.selected_bbox_height_m = static_cast<double>(max_y - min_y) / kClipperScale;
  result.selected_bbox_area_m2 = result.selected_bbox_width_m * result.selected_bbox_height_m;
  result.selected_bbox_fill_ratio =
      result.selected_bbox_area_m2 > 0.0 ? result.selected_area_m2 / result.selected_bbox_area_m2 : 0.0;
}

Point64 toClipperPoint(double x_m, double y_m) {
  return Point64(static_cast<int64_t>(std::llround(x_m * kClipperScale)),
                 static_cast<int64_t>(std::llround(y_m * kClipperScale)));
}

Path64 footprintAtPose(const std::vector<geometry_msgs::Point32>& footprint, double x, double y, double yaw) {
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  Path64 path;
  path.reserve(footprint.size());
  for (const auto& point : footprint) {
    const double world_x = x + cos_yaw * point.x - sin_yaw * point.y;
    const double world_y = y + sin_yaw * point.x + cos_yaw * point.y;
    path.push_back(toClipperPoint(world_x, world_y));
  }
  path = stripDuplicateVertices(std::move(path));
  if (!path.empty() && !Clipper2Lib::IsPositive(path)) {
    std::reverse(path.begin(), path.end());
  }
  return path;
}

void appendPoseFootprint(const std::vector<geometry_msgs::Point32>& footprint,
                         double x,
                         double y,
                         double yaw,
                         Paths64& paths,
                         std::size_t& interpolated_pose_count) {
  if (!finitePoint(x, y) || !std::isfinite(yaw)) {
    return;
  }
  auto path = footprintAtPose(footprint, x, y, yaw);
  if (!validClipperPath(path, kMinAreaM2)) {
    return;
  }
  paths.emplace_back(std::move(path));
  ++interpolated_pose_count;
}

void appendInterpolatedFootprints(const std::vector<geometry_msgs::Point32>& footprint,
                                  const std::vector<std::vector<geometry_msgs::Pose>>& pose_segments,
                                  const SweptAreaOptions& options,
                                  Paths64& paths,
                                  std::size_t& input_pose_count,
                                  std::size_t& interpolated_pose_count) {
  const double pose_step_m = std::max(0.005, options.pose_step_m);
  const double yaw_step_rad = std::max(0.005, options.yaw_step_rad);

  for (const auto& segment : pose_segments) {
    input_pose_count += segment.size();
    if (segment.empty()) {
      continue;
    }

    const auto& first = segment.front();
    appendPoseFootprint(footprint,
                        first.position.x,
                        first.position.y,
                        yawFromPose(first),
                        paths,
                        interpolated_pose_count);

    for (std::size_t i = 1; i < segment.size(); ++i) {
      const auto& prev = segment[i - 1];
      const auto& curr = segment[i];
      const double prev_yaw = yawFromPose(prev);
      const double curr_yaw = yawFromPose(curr);
      if (!finitePoint(prev.position.x, prev.position.y) || !finitePoint(curr.position.x, curr.position.y) ||
          !std::isfinite(prev_yaw) || !std::isfinite(curr_yaw)) {
        continue;
      }

      const double dx = curr.position.x - prev.position.x;
      const double dy = curr.position.y - prev.position.y;
      const double distance = std::hypot(dx, dy);
      const double yaw_delta = normalizeAngle(curr_yaw - prev_yaw);
      const std::size_t steps = std::max<std::size_t>(
          1,
          static_cast<std::size_t>(std::ceil(std::max(distance / pose_step_m, std::abs(yaw_delta) / yaw_step_rad))));

      for (std::size_t step = 1; step <= steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        appendPoseFootprint(footprint,
                            prev.position.x + dx * t,
                            prev.position.y + dy * t,
                            normalizeAngle(prev_yaw + yaw_delta * t),
                            paths,
                            interpolated_pose_count);
      }
    }
  }
}

std::size_t countNonEmptySegments(const std::vector<std::vector<geometry_msgs::Pose>>& pose_segments) {
  std::size_t count = 0;
  for (const auto& segment : pose_segments) {
    if (!segment.empty()) {
      ++count;
    }
  }
  return count;
}

void collectCandidates(const PolyPath64& node,
                       BoundarySelection selection,
                       std::vector<Path64>& candidates) {
  const bool has_polygon = !node.Polygon().empty();
  const bool is_hole = node.IsHole();
  if (has_polygon &&
      ((selection == BoundarySelection::LARGEST_EXTERIOR && !is_hole) ||
       (selection == BoundarySelection::LARGEST_INTERIOR_HOLE && is_hole))) {
    candidates.push_back(node.Polygon());
  }

  for (std::size_t i = 0; i < node.Count(); ++i) {
    collectCandidates(*node.Child(i), selection, candidates);
  }
}

void collectDiagnostics(const PolyPath64& node, SweptAreaResult& result) {
  const bool has_polygon = !node.Polygon().empty();
  if (has_polygon) {
    const double area_m2 = pathAreaM2(node.Polygon());
    if (node.IsHole()) {
      ++result.interior_hole_count;
      result.largest_hole_area_m2 = std::max(result.largest_hole_area_m2, area_m2);
    } else {
      ++result.exterior_ring_count;
      result.largest_exterior_area_m2 = std::max(result.largest_exterior_area_m2, area_m2);
    }
  }

  for (std::size_t i = 0; i < node.Count(); ++i) {
    collectDiagnostics(*node.Child(i), result);
  }
}

bool simplifiedPathIsSafe(const Path64& original, const Path64& simplified) {
  if (!validClipperPath(simplified, kMinAreaM2)) {
    return false;
  }

  Clipper2Lib::Clipper64 clipper;
  clipper.PreserveCollinear(false);
  clipper.AddSubject(Paths64{simplified});
  Paths64 solution;
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, solution)) {
    return false;
  }
  if (solution.size() != 1 || uniqueVertexCount(solution.front()) < 3) {
    return false;
  }

  const double original_area = std::abs(Clipper2Lib::Area(original));
  const double simplified_area = std::abs(Clipper2Lib::Area(simplified));
  const double allowed_delta = std::max(original_area * 0.05, std::pow(0.05 * kClipperScale, 2.0));
  return std::abs(original_area - simplified_area) <= allowed_delta;
}

Path64 simplifyWithFallback(const Path64& path, double epsilon_m) {
  const double epsilon = std::max(0.0, epsilon_m) * kClipperScale;
  if (epsilon <= 0.0 || path.size() < 4) {
    return path;
  }
  auto simplified = stripDuplicateVertices(Clipper2Lib::SimplifyPath(path, epsilon, true));
  if (simplifiedPathIsSafe(path, simplified)) {
    return simplified;
  }
  return path;
}

geometry_msgs::Polygon toGeometryPolygon(Path64 path, bool positive_winding) {
  path = stripDuplicateVertices(std::move(path));
  if (path.empty()) {
    return geometry_msgs::Polygon();
  }
  const bool is_positive = Clipper2Lib::IsPositive(path);
  if (is_positive != positive_winding) {
    std::reverse(path.begin(), path.end());
  }

  geometry_msgs::Polygon polygon;
  polygon.points.reserve(path.size() + 1);
  for (const auto& point : path) {
    geometry_msgs::Point32 msg_point;
    msg_point.x = static_cast<float>(static_cast<double>(point.x) / kClipperScale);
    msg_point.y = static_cast<float>(static_cast<double>(point.y) / kClipperScale);
    msg_point.z = 0.0f;
    polygon.points.push_back(msg_point);
  }
  if (!polygon.points.empty()) {
    polygon.points.push_back(polygon.points.front());
  }
  return polygon;
}

std::string selectionName(BoundarySelection selection) {
  return selection == BoundarySelection::LARGEST_EXTERIOR ? "exterior boundary" : "interior hole";
}

}  // namespace

SweptAreaRecorder::SweptAreaRecorder(std::vector<geometry_msgs::Point32> footprint)
    : footprint_(std::move(footprint)) {
}

bool SweptAreaRecorder::validFootprint(std::string* error) const {
  if (footprint_.size() < 3) {
    if (error) {
      *error = "footprint has fewer than 3 points";
    }
    return false;
  }
  for (const auto& point : footprint_) {
    if (!finitePoint(point.x, point.y)) {
      if (error) {
        *error = "footprint contains non-finite points";
      }
      return false;
    }
  }
  if (std::abs(signedAreaM2(footprint_)) < kMinAreaM2) {
    if (error) {
      *error = "footprint area is too small";
    }
    return false;
  }
  if (error) {
    error->clear();
  }
  return true;
}

SweptAreaResult SweptAreaRecorder::buildBoundary(const std::vector<std::vector<geometry_msgs::Pose>>& pose_segments,
                                                 const SweptAreaOptions& options,
                                                 BoundarySelection selection) const {
  SweptAreaResult result;
  result.pose_segment_count = countNonEmptySegments(pose_segments);
  std::string footprint_error;
  if (!validFootprint(&footprint_error)) {
    result.error = "invalid footprint: " + footprint_error;
    return result;
  }

  Paths64 footprints;
  appendInterpolatedFootprints(footprint_,
                               pose_segments,
                               options,
                               footprints,
                               result.input_pose_count,
                               result.interpolated_pose_count);
  result.footprint_polygon_count = footprints.size();
  if (result.input_pose_count < 2) {
    result.error = "insufficient poses for swept area";
    return result;
  }
  if (footprints.empty()) {
    result.error = "no valid footprint polygons were generated";
    return result;
  }

  Clipper2Lib::Clipper64 clipper;
  clipper.PreserveCollinear(false);
  clipper.AddSubject(footprints);

  PolyTree64 union_tree;
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, union_tree)) {
    result.error = "Clipper2 union failed";
    return result;
  }

  result.union_area_m2 = std::abs(union_tree.Area()) / (kClipperScale * kClipperScale);
  collectDiagnostics(union_tree, result);
  if (result.union_area_m2 < std::max(kMinAreaM2, options.min_polygon_area_m2)) {
    std::ostringstream stream;
    stream << "swept area is too small (" << result.union_area_m2 << " m^2)";
    result.error = stream.str();
    return result;
  }

  std::vector<Path64> candidates;
  collectCandidates(union_tree, selection, candidates);
  if (candidates.empty()) {
    result.error = "no " + selectionName(selection) + " found in swept area";
    return result;
  }

  auto best = std::max_element(candidates.begin(), candidates.end(), [](const Path64& a, const Path64& b) {
    return std::abs(Clipper2Lib::Area(a)) < std::abs(Clipper2Lib::Area(b));
  });
  if (best == candidates.end()) {
    result.error = "failed to select " + selectionName(selection);
    return result;
  }

  auto selected = stripDuplicateVertices(*best);
  result.selected_area_m2 = pathAreaM2(selected);
  result.selected_perimeter_m = pathPerimeterM(selected);
  setPathBoundsDiagnostics(selected, result);
  if (!validClipperPath(selected, std::max(kMinAreaM2, options.min_polygon_area_m2))) {
    result.error = "selected " + selectionName(selection) + " is too small or invalid";
    return result;
  }

  selected = simplifyWithFallback(selected, options.simplify_epsilon_m);
  result.polygon = toGeometryPolygon(selected, selection == BoundarySelection::LARGEST_EXTERIOR);
  if (result.polygon.points.size() < 4) {
    result.error = "selected " + selectionName(selection) + " has fewer than 3 unique vertices";
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace area_recording
}  // namespace mower_logic
