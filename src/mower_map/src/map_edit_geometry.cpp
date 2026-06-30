#include "mower_map/map_edit_geometry.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>

#include "clipper2/clipper.h"

namespace mower_map {
namespace edit_geometry {
namespace {

constexpr double kClipperScale = 1000.0;
constexpr double kMinAreaM2 = 0.01;
constexpr double kMinBrushDiameterM = 0.05;
constexpr double kMaxBrushDiameterM = 5.0;
constexpr double kPi = 3.14159265358979323846;

using Clipper2Lib::ClipType;
using Clipper2Lib::EndType;
using Clipper2Lib::FillRule;
using Clipper2Lib::JoinType;
using Clipper2Lib::Path64;
using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;
using Clipper2Lib::PolyPath64;
using Clipper2Lib::PolyTree64;

bool finitePoint(double x, double y) {
  return std::isfinite(x) && std::isfinite(y);
}

Point64 toClipperPoint(double x, double y) {
  return Point64(static_cast<int64_t>(std::llround(x * kClipperScale)),
                 static_cast<int64_t>(std::llround(y * kClipperScale)));
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

double pathAreaM2(const Path64& path) {
  return std::abs(Clipper2Lib::Area(path)) / (kClipperScale * kClipperScale);
}

bool validPolygonPath(const Path64& path, double min_area_m2 = kMinAreaM2) {
  return uniqueVertexCount(path) >= 3 && pathAreaM2(path) >= min_area_m2;
}

bool geometryPolygonToPath(const geometry_msgs::Polygon& polygon, Path64& path, std::string& error) {
  path.clear();
  path.reserve(polygon.points.size());
  for (const auto& point : polygon.points) {
    if (!finitePoint(point.x, point.y)) {
      error = "polygon contains non-finite points";
      return false;
    }
    path.push_back(toClipperPoint(point.x, point.y));
  }

  path = stripDuplicateVertices(std::move(path));
  if (!validPolygonPath(path)) {
    error = "polygon has fewer than 3 unique points or is too small";
    return false;
  }
  return true;
}

Path64 geometryStrokeToPath(const geometry_msgs::Polygon& polygon, std::string& error) {
  Path64 path;
  path.reserve(polygon.points.size());
  for (const auto& point : polygon.points) {
    if (!finitePoint(point.x, point.y)) {
      error = "stroke contains non-finite points";
      return {};
    }
    const auto clipper_point = toClipperPoint(point.x, point.y);
    if (path.empty() || path.back() != clipper_point) {
      path.push_back(clipper_point);
    }
  }
  if (path.empty()) {
    error = "stroke has no points";
  }
  return path;
}

geometry_msgs::Polygon pathToGeometryPolygon(Path64 path, bool positive_winding = true) {
  path = stripDuplicateVertices(std::move(path));
  if (!path.empty() && Clipper2Lib::IsPositive(path) != positive_winding) {
    std::reverse(path.begin(), path.end());
  }

  geometry_msgs::Polygon polygon;
  polygon.points.reserve(path.size() + 1);
  for (const auto& point : path) {
    geometry_msgs::Point32 geometry_point;
    geometry_point.x = static_cast<float>(static_cast<double>(point.x) / kClipperScale);
    geometry_point.y = static_cast<float>(static_cast<double>(point.y) / kClipperScale);
    geometry_point.z = 0.0f;
    polygon.points.push_back(geometry_point);
  }
  if (!polygon.points.empty()) {
    polygon.points.push_back(polygon.points.front());
  }
  return polygon;
}

Path64 circleBrushAtPoint(const Point64& center, double radius_scaled) {
  const int steps = 32;
  Path64 circle;
  circle.reserve(steps);
  for (int i = 0; i < steps; ++i) {
    const double angle = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(steps);
    circle.push_back(Point64(center.x + static_cast<int64_t>(std::llround(std::cos(angle) * radius_scaled)),
                             center.y + static_cast<int64_t>(std::llround(std::sin(angle) * radius_scaled))));
  }
  return circle;
}

bool buildBrushPaths(const geometry_msgs::Polygon& stroke_path,
                     double brush_diameter_m,
                     Paths64& brush_paths,
                     std::string& error) {
  if (!std::isfinite(brush_diameter_m) || brush_diameter_m < kMinBrushDiameterM ||
      brush_diameter_m > kMaxBrushDiameterM) {
    std::ostringstream stream;
    stream << "brush diameter must be between " << kMinBrushDiameterM << "m and " << kMaxBrushDiameterM << "m";
    error = stream.str();
    return false;
  }

  Path64 stroke = geometryStrokeToPath(stroke_path, error);
  if (stroke.empty()) {
    return false;
  }

  const double radius_scaled = (brush_diameter_m * 0.5) * kClipperScale;
  if (stroke.size() == 1) {
    brush_paths.push_back(circleBrushAtPoint(stroke.front(), radius_scaled));
    return true;
  }

  brush_paths = Clipper2Lib::InflatePaths(Paths64{stroke},
                                          radius_scaled,
                                          JoinType::Round,
                                          EndType::Round,
                                          2.0,
                                          std::max(1.0, radius_scaled * 0.03));
  brush_paths.erase(std::remove_if(brush_paths.begin(),
                                   brush_paths.end(),
                                   [](const Path64& path) { return !validPolygonPath(path); }),
                    brush_paths.end());
  if (brush_paths.empty()) {
    error = "brush produced no valid geometry";
    return false;
  }
  return true;
}

bool executeUnion(const Paths64& subjects, Paths64& solution, std::string& error) {
  Clipper2Lib::Clipper64 clipper;
  clipper.PreserveCollinear(false);
  clipper.AddSubject(subjects);
  if (!clipper.Execute(ClipType::Union, FillRule::NonZero, solution)) {
    error = "Clipper2 union failed";
    return false;
  }
  solution.erase(std::remove_if(solution.begin(),
                                solution.end(),
                                [](const Path64& path) { return !validPolygonPath(path); }),
                 solution.end());
  std::sort(solution.begin(), solution.end(), [](const Path64& a, const Path64& b) {
    return pathAreaM2(a) > pathAreaM2(b);
  });
  return true;
}

void collectDifferenceRings(const PolyPath64& node, Paths64& exteriors, Paths64& holes) {
  if (!node.Polygon().empty()) {
    if (node.IsHole()) {
      holes.push_back(node.Polygon());
    } else {
      exteriors.push_back(node.Polygon());
    }
  }
  for (std::size_t i = 0; i < node.Count(); ++i) {
    collectDifferenceRings(*node.Child(i), exteriors, holes);
  }
}

bool executeDifference(const Path64& subject, const Paths64& clip, Paths64& exteriors, Paths64& holes, std::string& error) {
  Clipper2Lib::Clipper64 clipper;
  clipper.PreserveCollinear(false);
  clipper.AddSubject(Paths64{subject});
  clipper.AddClip(clip);

  PolyTree64 solution_tree;
  if (!clipper.Execute(ClipType::Difference, FillRule::NonZero, solution_tree)) {
    error = "Clipper2 difference failed";
    return false;
  }

  collectDifferenceRings(solution_tree, exteriors, holes);
  exteriors.erase(std::remove_if(exteriors.begin(),
                                 exteriors.end(),
                                 [](const Path64& path) { return !validPolygonPath(path); }),
                  exteriors.end());
  holes.erase(std::remove_if(holes.begin(),
                             holes.end(),
                             [](const Path64& path) { return !validPolygonPath(path); }),
              holes.end());
  std::sort(exteriors.begin(), exteriors.end(), [](const Path64& a, const Path64& b) {
    return pathAreaM2(a) > pathAreaM2(b);
  });
  std::sort(holes.begin(), holes.end(), [](const Path64& a, const Path64& b) {
    return pathAreaM2(a) > pathAreaM2(b);
  });
  return true;
}

}  // namespace

bool normalizeReplacementPolygon(const geometry_msgs::Polygon& input,
                                 geometry_msgs::Polygon& output,
                                 std::string& error) {
  Path64 path;
  if (!geometryPolygonToPath(input, path, error)) {
    return false;
  }
  output = pathToGeometryPolygon(path, true);
  error.clear();
  return true;
}

EditGeometryResult applyBrushAdd(const geometry_msgs::Polygon& target,
                                 const geometry_msgs::Polygon& stroke_path,
                                 double brush_diameter_m) {
  EditGeometryResult result;
  Path64 target_path;
  if (!geometryPolygonToPath(target, target_path, result.error)) {
    return result;
  }
  if (!Clipper2Lib::IsPositive(target_path)) {
    std::reverse(target_path.begin(), target_path.end());
  }

  Paths64 brush_paths;
  if (!buildBrushPaths(stroke_path, brush_diameter_m, brush_paths, result.error)) {
    return result;
  }

  Paths64 subjects{target_path};
  subjects.insert(subjects.end(), brush_paths.begin(), brush_paths.end());
  Paths64 union_solution;
  if (!executeUnion(subjects, union_solution, result.error)) {
    return result;
  }
  if (union_solution.empty()) {
    result.error = "brush union produced no area";
    return result;
  }
  if (union_solution.size() > 1) {
    result.error = "brush stroke must touch the selected area";
    return result;
  }

  result.outline = pathToGeometryPolygon(union_solution.front(), true);
  result.success = true;
  return result;
}

EditGeometryResult applyBrushErase(const geometry_msgs::Polygon& target,
                                   const geometry_msgs::Polygon& stroke_path,
                                   double brush_diameter_m) {
  EditGeometryResult result;
  Path64 target_path;
  if (!geometryPolygonToPath(target, target_path, result.error)) {
    return result;
  }
  if (!Clipper2Lib::IsPositive(target_path)) {
    std::reverse(target_path.begin(), target_path.end());
  }

  Paths64 brush_paths;
  if (!buildBrushPaths(stroke_path, brush_diameter_m, brush_paths, result.error)) {
    return result;
  }

  Paths64 exteriors;
  Paths64 holes;
  if (!executeDifference(target_path, brush_paths, exteriors, holes, result.error)) {
    return result;
  }
  if (exteriors.empty()) {
    result.error = "erase would remove the entire selected area";
    return result;
  }

  result.outline = pathToGeometryPolygon(exteriors.front(), true);
  for (std::size_t i = 1; i < exteriors.size(); ++i) {
    result.extra_outlines.push_back(pathToGeometryPolygon(exteriors[i], true));
  }
  for (const auto& hole : holes) {
    result.obstacles.push_back(pathToGeometryPolygon(hole, true));
  }
  result.success = true;
  return result;
}

}  // namespace edit_geometry
}  // namespace mower_map
