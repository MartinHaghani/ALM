// Full-footprint swept area geometry for mower area recording.
//
// This file is part of OpenMower.
#ifndef SRC_SWEPT_AREA_RECORDER_H
#define SRC_SWEPT_AREA_RECORDER_H

#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Pose.h"

namespace mower_logic {
namespace area_recording {

enum class BoundarySelection {
  LARGEST_EXTERIOR,
  LARGEST_INTERIOR_HOLE,
};

struct SweptAreaOptions {
  double pose_step_m = 0.03;
  double yaw_step_rad = 0.05;
  double simplify_epsilon_m = 0.03;
  double min_polygon_area_m2 = 0.25;
};

struct SweptAreaResult {
  bool success = false;
  std::string error;
  geometry_msgs::Polygon polygon;
  double union_area_m2 = 0.0;
  double selected_area_m2 = 0.0;
  double largest_exterior_area_m2 = 0.0;
  double largest_hole_area_m2 = 0.0;
  double selected_perimeter_m = 0.0;
  double selected_bbox_width_m = 0.0;
  double selected_bbox_height_m = 0.0;
  double selected_bbox_area_m2 = 0.0;
  double selected_bbox_fill_ratio = 0.0;
  std::size_t pose_segment_count = 0;
  std::size_t input_pose_count = 0;
  std::size_t interpolated_pose_count = 0;
  std::size_t footprint_polygon_count = 0;
  std::size_t exterior_ring_count = 0;
  std::size_t interior_hole_count = 0;
};

class SweptAreaRecorder {
 public:
  explicit SweptAreaRecorder(std::vector<geometry_msgs::Point32> footprint);

  bool validFootprint(std::string* error = nullptr) const;

  SweptAreaResult buildBoundary(const std::vector<std::vector<geometry_msgs::Pose>>& pose_segments,
                                const SweptAreaOptions& options,
                                BoundarySelection selection) const;

 private:
  std::vector<geometry_msgs::Point32> footprint_;
};

}  // namespace area_recording
}  // namespace mower_logic

#endif  // SRC_SWEPT_AREA_RECORDER_H
