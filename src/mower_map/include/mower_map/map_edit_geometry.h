#ifndef MOWER_MAP_MAP_EDIT_GEOMETRY_H
#define MOWER_MAP_MAP_EDIT_GEOMETRY_H

#include <string>
#include <vector>

#include "geometry_msgs/Polygon.h"

namespace mower_map {
namespace edit_geometry {

struct EditGeometryResult {
  bool success = false;
  std::string error;
  geometry_msgs::Polygon outline;
  std::vector<geometry_msgs::Polygon> extra_outlines;
  std::vector<geometry_msgs::Polygon> obstacles;
};

bool normalizeReplacementPolygon(const geometry_msgs::Polygon& input,
                                 geometry_msgs::Polygon& output,
                                 std::string& error);

EditGeometryResult applyBrushAdd(const geometry_msgs::Polygon& target,
                                 const geometry_msgs::Polygon& stroke_path,
                                 double brush_diameter_m);

EditGeometryResult applyBrushErase(const geometry_msgs::Polygon& target,
                                   const geometry_msgs::Polygon& stroke_path,
                                   double brush_diameter_m);

}  // namespace edit_geometry
}  // namespace mower_map

#endif  // MOWER_MAP_MAP_EDIT_GEOMETRY_H
