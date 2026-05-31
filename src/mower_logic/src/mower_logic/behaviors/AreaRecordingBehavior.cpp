// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "AreaRecordingBehavior.h"

#include <XmlRpcValue.h>

#include <algorithm>
#include <atomic>
#include <costmap_2d/footprint.h>
#include <cmath>
#include <limits>

extern ros::ServiceClient dockingPointClient;
extern ros::ServiceClient emergencyClient;
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern ros::NodeHandle* n;
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);
extern bool setMowerEnabled(bool enabled);
extern std::atomic<bool> mowerAllowed;

extern void stop();

extern bool setGPS(bool enabled);

AreaRecordingBehavior AreaRecordingBehavior::INSTANCE;

namespace {
constexpr double kManualMowingStopGuardSec = 0.2;
constexpr double kAreaRecordingLoopPeriodSec = 0.05;
constexpr double kGpsPoseFreshSec = 2.0;

geometry_msgs::Point32 makePoint(double x, double y) {
  geometry_msgs::Point32 point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  return point;
}

double yawFromPose(const geometry_msgs::Pose& pose) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void applyManualMowingState(bool manual_mowing) {
  setMowerEnabled(mowerAllowed.load() && manual_mowing);
}

void stopManualMowing(bool& manual_mowing, ros::Time& manual_mowing_stop_guard_until, bool& manual_mowing_stop_pending) {
  manual_mowing = false;
  manual_mowing_stop_guard_until = ros::Time(0);
  manual_mowing_stop_pending = false;
  applyManualMowingState(false);
}

geometry_msgs::Point32 pointToPoint32(const geometry_msgs::Point& point) {
  return makePoint(point.x, point.y);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double poseDistance2D(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

std::size_t sweptPoseCount(const std::vector<std::vector<geometry_msgs::Pose>>& swept_pose_segments) {
  std::size_t count = 0;
  for (const auto& segment : swept_pose_segments) {
    count += segment.size();
  }
  return count;
}

std::size_t uniquePolygonVertexCount(const geometry_msgs::Polygon& polygon) {
  std::vector<geometry_msgs::Point32> unique;
  unique.reserve(polygon.points.size());
  for (std::size_t i = 0; i < polygon.points.size(); ++i) {
    const auto& point = polygon.points[i];
    if (i + 1 == polygon.points.size() && !polygon.points.empty() && point.x == polygon.points.front().x &&
        point.y == polygon.points.front().y) {
      continue;
    }
    const auto it = std::find_if(unique.begin(), unique.end(), [&](const geometry_msgs::Point32& existing) {
      return existing.x == point.x && existing.y == point.y;
    });
    if (it == unique.end()) {
      unique.push_back(point);
    }
  }
  return unique.size();
}

bool polygonFinite(const geometry_msgs::Polygon& polygon) {
  return std::all_of(polygon.points.begin(), polygon.points.end(), [](const geometry_msgs::Point32& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
  });
}
}

std::string AreaRecordingBehavior::state_name() {
  return "AREA_RECORDING";
}

Behavior* AreaRecordingBehavior::execute() {
  setGPS(true);
  bool error = false;
  ros::Rate inputDelay(ros::Duration().fromSec(kAreaRecordingLoopPeriodSec));

  while (ros::ok() && !aborted) {
    mower_map::MapArea result;
    RecordedPolygon recorded_outline;
    std::vector<RecordedPolygon> recorded_obstacles;
    xbot_msgs::MapOverlay result_overlay;

    // clear overlay
    map_overlay_pub.publish(result_overlay);

    has_outline = false;

    sub_state = 0;
    while (ros::ok() && !finished_all && !error && !aborted) {
      if (manual_mowing && manual_mowing_stop_pending && !is_manual_mowing_stop_guard_active()) {
        ROS_INFO_STREAM("Stopping manual mowing after start/stop chatter guard");
        stopManualMowing(manual_mowing, manual_mowing_stop_guard_until, manual_mowing_stop_pending);
        update_actions();
      }

      if (set_docking_position) {
        geometry_msgs::Pose pos;
        if (getDockingPosition(pos)) {
          ROS_INFO_STREAM("new docking pos = " << pos);

          mower_map::SetDockingPointSrv set_docking_point_srv;
          set_docking_point_srv.request.docking_pose = pos;
          auto result = set_docking_point_client.call(set_docking_point_srv);

          has_first_docking_pos = false;
          update_actions();
        }

        set_docking_position = false;
      }

      if (poly_recording_enabled) {
        update_actions();
        RecordedPolygon poly;
        const bool recording_obstacle = has_outline;
        const uint8_t preview_point_mode =
            recording_obstacle ? mower_map::BoundarySample::POINT_FRONT_LEFT : mower_map::BoundarySample::POINT_FRONT_RIGHT;
        // record poly
        if (has_outline) {
          sub_state = 1;
        } else {
          sub_state = 2;
        }
        bool success = recordNewPolygon(poly, result_overlay, preview_point_mode);
        sub_state = 0;
        if (success) {
          if (!has_outline) {
            // first polygon is outline
            has_outline = true;
            recorded_outline = poly;
            result.area = poly.front_right;

            std_msgs::ColorRGBA color;
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 1.0f;

            marker.color = color;
            marker.id = markers.markers.size() + 1;
            marker.action = visualization_msgs::Marker::ADD;
            markers.markers.push_back(marker);
          } else {
            // we already have an outline, add obstacles
            recorded_obstacles.push_back(poly);
            result.obstacles.push_back(poly.front_left);

            std_msgs::ColorRGBA color;
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
            color.a = 1.0f;

            marker.color = color;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = markers.markers.size() + 1;
            markers.markers.push_back(marker);
          }

        } else {
          error = true;
          ROS_ERROR_STREAM("Error during poly record");
        }
        marker_array_pub.publish(markers);
        update_actions();
      }

      inputDelay.sleep();
    }

    if (!error && has_outline && (is_mowing_area || is_navigation_area)) {
      bool ready_to_add_area = true;
      if (is_mowing_area) {
        ROS_INFO_STREAM("Area recording completed. Adding mowing area.");
        ready_to_add_area = buildSweptBoundary(recorded_outline,
                                               mower_logic::area_recording::BoundarySelection::LARGEST_EXTERIOR,
                                               "mowing outline",
                                               result.area);
      } else if (is_navigation_area) {
        ROS_INFO_STREAM("Area recording completed. Adding navigation area.");
        result.area = recorded_outline.base;
        ready_to_add_area = recordedPolygonValid(result.area, "navigation area");
      }
      result.obstacles.clear();
      if (ready_to_add_area) {
        for (const auto& obstacle : recorded_obstacles) {
          geometry_msgs::Polygon obstacle_polygon;
          if (!buildSweptBoundary(obstacle,
                                  mower_logic::area_recording::BoundarySelection::LARGEST_INTERIOR_HOLE,
                                  "obstacle",
                                  obstacle_polygon)) {
            ready_to_add_area = false;
            break;
          }
          result.obstacles.push_back(obstacle_polygon);
        }
      }

      if (ready_to_add_area) {
        mower_map::AddMowingAreaSrv srv;
        srv.request.isNavigationArea = !is_mowing_area;
        srv.request.area = result;
        if (add_mowing_area_client.call(srv)) {
          ROS_INFO_STREAM("Area added successfully");
          if (is_mowing_area) {
            publishBoundarySamples(recorded_outline.front_right_samples, mower_map::BoundarySample::AREA_MOW);
          } else {
            publishBoundarySamples(recorded_outline.base_samples, mower_map::BoundarySample::AREA_NAV);
          }
          for (const auto& obstacle : recorded_obstacles) {
            publishBoundarySamples(obstacle.front_left_samples, mower_map::BoundarySample::AREA_OBSTACLE);
          }
        } else {
          ROS_ERROR_STREAM("error adding area");
        }
      } else {
        ROS_WARN_STREAM("Area recording was not saved because the recorded geometry was invalid.");
      }
    }

    // reset recording error for next area
    error = false;
    // reset finished all in case we want to record a second area
    finished_all = false;

    has_outline = false;
    update_actions();
  }

  return &IdleBehavior::INSTANCE;
}

void AreaRecordingBehavior::enter() {
  has_outline = false;
  is_mowing_area = false;
  is_navigation_area = false;
  manual_mowing = false;
  manual_mowing_stop_guard_until = ros::Time(0);
  manual_mowing_stop_pending = false;

  update_actions();

  has_first_docking_pos = false;
  has_odom = false;
  poly_recording_enabled = false;
  finished_all = false;
  set_docking_position = false;
  markers = visualization_msgs::MarkerArray();
  paused = aborted = false;

  ros::param::param<double>("/xbot_positioning/max_gps_accuracy", max_recording_gps_accuracy, 0.2);
  loadAreaRecordingParams();
  loadFootprintRecordingPoints();

  add_mowing_area_client = n->serviceClient<mower_map::AddMowingAreaSrv>("mower_map_service/add_mowing_area");
  set_docking_point_client = n->serviceClient<mower_map::SetDockingPointSrv>("mower_map_service/set_docking_point");

  boundary_sample_pub = n->advertise<mower_map::BoundarySample>("area_recorder/boundary_samples", 1000);
  marker_pub = n->advertise<visualization_msgs::Marker>("area_recorder/progress_visualization", 10);
  map_overlay_pub = n->advertise<xbot_msgs::MapOverlay>("xbot_monitoring/map_overlay", 10);
  marker_array_pub = n->advertise<visualization_msgs::MarkerArray>("area_recorder/progress_visualization_array", 10);

  ROS_INFO_STREAM("Starting recording area");

  ROS_INFO_STREAM("Subscribing to /joy for user input");

  joy_sub = n->subscribe("/joy", 100, &AreaRecordingBehavior::joy_received, this);

  dock_sub = n->subscribe("/record_dock", 100, &AreaRecordingBehavior::record_dock_received, this);
  polygon_sub = n->subscribe("/record_polygon", 100, &AreaRecordingBehavior::record_polygon_received, this);
  mow_area_sub = n->subscribe("/record_mowing", 100, &AreaRecordingBehavior::record_mowing_received, this);
  nav_area_sub = n->subscribe("/record_navigation", 100, &AreaRecordingBehavior::record_navigation_received, this);

  auto_point_collecting_sub =
      n->subscribe("/record_auto_point_collecting", 100, &AreaRecordingBehavior::record_auto_point_collecting, this);
  collect_point_sub = n->subscribe("/record_collect_point", 100, &AreaRecordingBehavior::record_collect_point, this);

  pose_sub = n->subscribe("/xbot_positioning/xb_pose", 100, &AreaRecordingBehavior::pose_received, this);
  gps_pose_sub = n->subscribe("/hw/position/gps", 100, &AreaRecordingBehavior::gps_pose_received, this);
}

void AreaRecordingBehavior::exit() {
  stopManualMowing(manual_mowing, manual_mowing_stop_guard_until, manual_mowing_stop_pending);

  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:area_recording", actions);

  map_overlay_pub.shutdown();
  marker_pub.shutdown();
  marker_array_pub.shutdown();
  boundary_sample_pub.shutdown();
  joy_sub.shutdown();
  gps_pose_sub.shutdown();
  dock_sub.shutdown();
  polygon_sub.shutdown();
  mow_area_sub.shutdown();
  nav_area_sub.shutdown();
  auto_point_collecting_sub.shutdown();
  collect_point_sub.shutdown();
  pose_sub.shutdown();
  add_mowing_area_client.shutdown();
  set_docking_point_client.shutdown();
}

void AreaRecordingBehavior::reset() {
}

bool AreaRecordingBehavior::needs_gps() {
  return false;
}

bool AreaRecordingBehavior::mower_enabled() {
  return manual_mowing;
}

bool AreaRecordingBehavior::is_manual_mowing_stop_guard_active() const {
  return manual_mowing && ros::Time::now() <= manual_mowing_stop_guard_until;
}

void AreaRecordingBehavior::pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  last_pose = *msg;
  has_odom = true;
}

void AreaRecordingBehavior::gps_pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  last_gps_pose = *msg;
  last_gps_pose_time = ros::Time::now();
  has_gps_pose = true;
}

bool AreaRecordingBehavior::recordingGpsQualityOk(const xbot_msgs::AbsolutePose& pose, std::string& reason) const {
  auto flags = pose.flags;
  double accuracy = pose.position_accuracy;

  if (has_gps_pose && (ros::Time::now() - last_gps_pose_time).toSec() <= kGpsPoseFreshSec) {
    flags = last_gps_pose.flags;
    accuracy = last_gps_pose.position_accuracy;
  }

  if ((flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) == 0) {
    reason = "RTK fixed GPS is required for area recording";
    return false;
  }
  if (!std::isfinite(accuracy)) {
    reason = "GPS accuracy is unavailable";
    return false;
  }
  if (accuracy > max_recording_gps_accuracy) {
    reason = "GPS accuracy is above the recording limit";
    return false;
  }

  reason.clear();
  return true;
}

void AreaRecordingBehavior::loadAreaRecordingParams() {
  ros::param::param<double>("/mower_logic/area_recording_pose_step_m", swept_area_options.pose_step_m, 0.03);
  ros::param::param<double>("/mower_logic/area_recording_yaw_step_rad", swept_area_options.yaw_step_rad, 0.05);
  ros::param::param<double>("/mower_logic/area_recording_simplify_epsilon_m",
                            swept_area_options.simplify_epsilon_m,
                            0.03);
  ros::param::param<double>("/mower_logic/area_recording_min_polygon_area_m2",
                            swept_area_options.min_polygon_area_m2,
                            0.25);

  swept_area_options.pose_step_m = std::max(0.005, std::min(0.5, swept_area_options.pose_step_m));
  swept_area_options.yaw_step_rad = std::max(0.005, std::min(0.5, swept_area_options.yaw_step_rad));
  swept_area_options.simplify_epsilon_m = std::max(0.0, std::min(0.5, swept_area_options.simplify_epsilon_m));
  swept_area_options.min_polygon_area_m2 =
      std::max(0.001, std::min(100.0, swept_area_options.min_polygon_area_m2));

  ROS_INFO_STREAM("Area recorder swept geometry params: pose_step_m=" << swept_area_options.pose_step_m
                                                                      << ", yaw_step_rad="
                                                                      << swept_area_options.yaw_step_rad
                                                                      << ", simplify_epsilon_m="
                                                                      << swept_area_options.simplify_epsilon_m
                                                                      << ", min_polygon_area_m2="
                                                                      << swept_area_options.min_polygon_area_m2);
}

geometry_msgs::Point32 AreaRecordingBehavior::projectPoint(const geometry_msgs::Pose& pose,
                                                           const geometry_msgs::Point32& offset) const {
  const double yaw = yawFromPose(pose);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return makePoint(
      pose.position.x + cos_yaw * offset.x - sin_yaw * offset.y,
      pose.position.y + sin_yaw * offset.x + cos_yaw * offset.y);
}

void AreaRecordingBehavior::loadFootprintRecordingPoints() {
  footprint_polygon.clear();
  footprint_polygon.push_back(makePoint(0.0, 0.34));
  footprint_polygon.push_back(makePoint(0.82, 0.34));
  footprint_polygon.push_back(makePoint(0.82, -0.34));
  footprint_polygon.push_back(makePoint(0.0, -0.34));
  footprint_front_left = makePoint(0.82, 0.34);
  footprint_front_right = makePoint(0.82, -0.34);

  XmlRpc::XmlRpcValue footprint;
  std::string footprint_param_name;
  for (const auto& param_name :
       {std::string("/move_base_flex/global_costmap/footprint"),
        std::string("/global_costmap/footprint"),
        std::string("/footprint")}) {
    if (ros::param::get(param_name, footprint)) {
      footprint_param_name = param_name;
      break;
    }
  }

  if (footprint_param_name.empty()) {
    ROS_WARN_STREAM("Area recorder could not find costmap footprint; using Mowrator fallback footprint.");
    return;
  }

  std::vector<geometry_msgs::Point> costmap_footprint;
  try {
    if (footprint.getType() == XmlRpc::XmlRpcValue::TypeString) {
      const std::string footprint_string = static_cast<std::string>(footprint);
      if (!costmap_2d::makeFootprintFromString(footprint_string, costmap_footprint)) {
        ROS_WARN_STREAM("Area recorder footprint parameter " << footprint_param_name
                                                             << " is invalid; using Mowrator fallback footprint.");
        return;
      }
    } else if (footprint.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      costmap_footprint = costmap_2d::makeFootprintFromXMLRPC(footprint, footprint_param_name);
    } else {
      ROS_WARN_STREAM("Area recorder footprint parameter " << footprint_param_name
                                                           << " has unsupported type; using Mowrator fallback footprint.");
      return;
    }
  } catch (const std::exception& error) {
    ROS_WARN_STREAM("Area recorder footprint parameter " << footprint_param_name << " is invalid (" << error.what()
                                                         << "); using Mowrator fallback footprint.");
    return;
  }

  if (costmap_footprint.size() < 3) {
    ROS_WARN_STREAM("Area recorder footprint parameter " << footprint_param_name
                                                         << " has fewer than 3 points; using Mowrator fallback footprint.");
    return;
  }

  std::vector<geometry_msgs::Point32> parsed_footprint;
  parsed_footprint.reserve(costmap_footprint.size());
  bool found = false;
  double max_x = -std::numeric_limits<double>::infinity();
  double front_left_y = -std::numeric_limits<double>::infinity();
  double front_right_y = std::numeric_limits<double>::infinity();
  for (const auto& point : costmap_footprint) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }

    parsed_footprint.push_back(pointToPoint32(point));

    if (!found || point.x > max_x + 1e-6) {
      max_x = point.x;
      front_left_y = point.y;
      front_right_y = point.y;
      found = true;
    } else if (std::abs(point.x - max_x) <= 1e-6) {
      front_left_y = std::max(front_left_y, point.y);
      front_right_y = std::min(front_right_y, point.y);
    }
  }

  if (!found) {
    ROS_WARN_STREAM("Area recorder could not parse any footprint points; using Mowrator fallback footprint.");
    return;
  }

  mower_logic::area_recording::SweptAreaRecorder recorder(parsed_footprint);
  std::string footprint_error;
  if (!recorder.validFootprint(&footprint_error)) {
    ROS_WARN_STREAM("Area recorder footprint parameter is invalid (" << footprint_error
                                                                     << "); using Mowrator fallback footprint.");
    return;
  }

  footprint_polygon = parsed_footprint;
  footprint_front_left = makePoint(max_x, front_left_y);
  footprint_front_right = makePoint(max_x, front_right_y);
  ROS_INFO_STREAM("Area recorder footprint loaded from " << footprint_param_name << " with " << footprint_polygon.size()
                                                         << " points; front-left=("
                                                         << footprint_front_left.x << ", " << footprint_front_left.y
                                                         << "), front-right=(" << footprint_front_right.x << ", "
                                                         << footprint_front_right.y << ")");
}

void AreaRecordingBehavior::addRecordedPoint(RecordedPolygon& polygon,
                                             const xbot_msgs::AbsolutePose& pose,
                                             uint32_t index,
                                             bool auto_collected) {
  const auto base_point = makePoint(pose.pose.pose.position.x, pose.pose.pose.position.y);
  const auto front_left_point = projectPoint(pose.pose.pose, footprint_front_left);
  const auto front_right_point = projectPoint(pose.pose.pose, footprint_front_right);

  polygon.base.points.push_back(base_point);
  polygon.front_left.points.push_back(front_left_point);
  polygon.front_right.points.push_back(front_right_point);

  auto make_sample = [&](uint8_t point_mode, const geometry_msgs::Point32& point) {
    mower_map::BoundarySample sample;
    sample.header = pose.header;
    if (sample.header.stamp == ros::Time()) {
      sample.header.stamp = ros::Time::now();
    }
    sample.header.frame_id = "map";
    sample.point_mode = point_mode;
    sample.point_index = index;
    sample.fused_pose = pose.pose.pose;
    sample.gps_point = point;
    sample.gps_flags = pose.flags;
    sample.gps_accuracy = pose.position_accuracy;
    sample.rtk_fixed = (sample.gps_flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
    sample.auto_collected = auto_collected;

    if (has_gps_pose && (ros::Time::now() - last_gps_pose_time).toSec() <= kGpsPoseFreshSec) {
      sample.gps_flags = last_gps_pose.flags;
      sample.gps_accuracy = last_gps_pose.position_accuracy;
      sample.rtk_fixed = (sample.gps_flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
    }
    return sample;
  };

  polygon.base_samples.push_back(make_sample(mower_map::BoundarySample::POINT_BASE, base_point));
  polygon.front_left_samples.push_back(make_sample(mower_map::BoundarySample::POINT_FRONT_LEFT, front_left_point));
  polygon.front_right_samples.push_back(make_sample(mower_map::BoundarySample::POINT_FRONT_RIGHT, front_right_point));
}

void AreaRecordingBehavior::addSweptPose(RecordedPolygon& polygon,
                                         const xbot_msgs::AbsolutePose& pose,
                                         bool start_new_segment) {
  if (start_new_segment || polygon.swept_pose_segments.empty()) {
    polygon.swept_pose_segments.emplace_back();
  }

  auto& segment = polygon.swept_pose_segments.back();
  const auto& pose_in_map = pose.pose.pose;
  if (!std::isfinite(pose_in_map.position.x) || !std::isfinite(pose_in_map.position.y)) {
    return;
  }

  if (!segment.empty()) {
    const auto& previous = segment.back();
    const double distance = poseDistance2D(previous, pose_in_map);
    const double yaw_delta = std::abs(normalizeAngle(yawFromPose(pose_in_map) - yawFromPose(previous)));
    const bool moved_far_enough = distance >= swept_area_options.pose_step_m;
    const bool rotated_far_enough = yaw_delta >= swept_area_options.yaw_step_rad;
    if (!moved_far_enough && !rotated_far_enough) {
      return;
    }
  }

  segment.push_back(pose_in_map);
}

bool AreaRecordingBehavior::buildSweptBoundary(const RecordedPolygon& polygon,
                                               mower_logic::area_recording::BoundarySelection selection,
                                               const std::string& label,
                                               geometry_msgs::Polygon& result) const {
  mower_logic::area_recording::SweptAreaRecorder recorder(footprint_polygon);
  const auto swept_result = recorder.buildBoundary(polygon.swept_pose_segments, swept_area_options, selection);
  if (!swept_result.success) {
    ROS_WARN_STREAM("Area recorder failed to create " << label << ": " << swept_result.error
                                                      << " (input poses="
                                                      << sweptPoseCount(polygon.swept_pose_segments)
                                                      << ", swept segments="
                                                      << swept_result.pose_segment_count
                                                      << ", interpolated poses="
                                                      << swept_result.interpolated_pose_count
                                                      << ", footprint polygons="
                                                      << swept_result.footprint_polygon_count
                                                      << ", union exteriors="
                                                      << swept_result.exterior_ring_count
                                                      << ", union holes="
                                                      << swept_result.interior_hole_count << ")");
    return false;
  }

  result = swept_result.polygon;
  ROS_INFO_STREAM("Area recorder created " << label << " from full-footprint swept union: input poses="
                                           << swept_result.input_pose_count
                                           << ", swept segments=" << swept_result.pose_segment_count
                                           << ", interpolated poses=" << swept_result.interpolated_pose_count
                                           << ", footprint polygons=" << swept_result.footprint_polygon_count
                                           << ", union exteriors=" << swept_result.exterior_ring_count
                                           << ", union holes=" << swept_result.interior_hole_count
                                           << ", union_area_m2=" << swept_result.union_area_m2
                                           << ", largest_exterior_area_m2=" << swept_result.largest_exterior_area_m2
                                           << ", largest_hole_area_m2=" << swept_result.largest_hole_area_m2
                                           << ", selected_area_m2=" << swept_result.selected_area_m2
                                           << ", selected_perimeter_m=" << swept_result.selected_perimeter_m
                                           << ", selected_bbox_m=" << swept_result.selected_bbox_width_m << "x"
                                           << swept_result.selected_bbox_height_m
                                           << ", selected_bbox_fill_ratio="
                                           << swept_result.selected_bbox_fill_ratio
                                           << ", saved_vertices=" << result.points.size());
  if (swept_result.pose_segment_count > 1) {
    ROS_WARN_STREAM("Area recorder diagnostics for " << label << ": recording has "
                                                     << swept_result.pose_segment_count
                                                     << " swept segments; GPS quality dropouts are not bridged.");
  }
  if (swept_result.exterior_ring_count > 1) {
    ROS_WARN_STREAM("Area recorder diagnostics for " << label << ": swept union has "
                                                     << swept_result.exterior_ring_count
                                                     << " disconnected exterior components; only the largest boundary was saved.");
  }
  if (selection == mower_logic::area_recording::BoundarySelection::LARGEST_EXTERIOR &&
      swept_result.selected_bbox_area_m2 > 0.0 && swept_result.selected_bbox_fill_ratio < 0.45) {
    ROS_WARN_STREAM("Area recorder diagnostics for " << label << ": selected exterior fills only "
                                                     << swept_result.selected_bbox_fill_ratio
                                                     << " of its bounding box; this often means the swept loop is open or has a gap.");
  }
  return true;
}

bool AreaRecordingBehavior::recordedPolygonValid(const geometry_msgs::Polygon& polygon, const std::string& label) const {
  if (!polygonFinite(polygon)) {
    ROS_WARN_STREAM("Area recorder failed to create " << label << ": polygon contains non-finite points");
    return false;
  }
  if (uniquePolygonVertexCount(polygon) < 3) {
    ROS_WARN_STREAM("Area recorder failed to create " << label << ": polygon has fewer than 3 unique vertices");
    return false;
  }
  return true;
}

void AreaRecordingBehavior::publishBoundarySamples(const std::vector<mower_map::BoundarySample>& samples,
                                                   uint8_t area_type) {
  for (auto sample : samples) {
    sample.area_type = area_type;
    boundary_sample_pub.publish(sample);
  }
}

void AreaRecordingBehavior::joy_received(const sensor_msgs::Joy& joy_msg) {
  if (joy_msg.buttons[1] && !last_joy.buttons[1]) {
    // B was pressed. We toggle recording state
    ROS_INFO_STREAM("B PRESSED");
    poly_recording_enabled = !poly_recording_enabled;
  }
  // Y + up was pressed, we finish the recording for a navigation area
  if ((joy_msg.buttons[3] && joy_msg.axes[7] > 0.5) && !(last_joy.buttons[3] && last_joy.axes[7] > 0.5)) {
    ROS_INFO_STREAM("Y + UP PRESSED, recording navigation area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  }
  // Y + down was pressed, we finish the recording for a navigation area
  if ((joy_msg.buttons[3] && joy_msg.axes[7] < -0.5) && !(last_joy.buttons[3] && last_joy.axes[7] < -0.5)) {
    ROS_INFO_STREAM("Y + DOWN PRESSED, recording mowing area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  }

  // X was pressed, set base position if we are not currently recording
  if (joy_msg.buttons[2] && !last_joy.buttons[2]) {
    ROS_INFO_STREAM("X PRESSED");
    set_docking_position = true;
  }

  // use RB button for manual point collecting
  // enable/disable auto point collecting with LB+RB
  if (joy_msg.buttons[5] && !last_joy.buttons[5]) {
    if (joy_msg.buttons[4] && !last_joy.buttons[4]) {
      ROS_INFO_STREAM("LB+RB PRESSED, toggle auto point collecting");
      auto_point_collecting = !auto_point_collecting;
      ROS_INFO_STREAM("Auto point collecting: " << auto_point_collecting);
    } else {
      ROS_INFO_STREAM("RB PRESSED, collect point");
      collect_point = true;
    }
  }

  last_joy = joy_msg;
}

void AreaRecordingBehavior::record_dock_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Record dock position");
    set_docking_position = true;
  }
}

void AreaRecordingBehavior::record_polygon_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    // We toggle recording state
    ROS_INFO_STREAM("Toggle record polygon");
    poly_recording_enabled = !poly_recording_enabled;
  }
}

void AreaRecordingBehavior::record_navigation_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Save polygon as navigation area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  }
}

void AreaRecordingBehavior::record_mowing_received(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Save polygon as mowing area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  }
}

bool AreaRecordingBehavior::recordNewPolygon(RecordedPolygon& polygon,
                                             xbot_msgs::MapOverlay& resultOverlay,
                                             uint8_t preview_point_mode) {
  ROS_INFO_STREAM("recordNewPolygon");

  bool success = true;
  marker = visualization_msgs::Marker();
  marker.header.frame_id = "map";
  marker.ns = "area_recorder";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = 0;
  marker.pose.orientation.w = 1.0f;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.frame_locked = true;

  std_msgs::ColorRGBA color;
  color.b = 1.0f;
  color.a = 1.0f;

  marker.color = color;

  ros::Rate updateRate(10);

  has_odom = false;

  // push a new poly to the visualization overlay
  {
    xbot_msgs::MapOverlayPolygon poly_viz;
    poly_viz.closed = false;
    poly_viz.line_width = 0.1;
    poly_viz.color = "blue";
    resultOverlay.polygons.push_back(poly_viz);
  }
  auto& poly_viz = resultOverlay.polygons.back();
  bool start_new_swept_segment = true;
  auto finish_recording = [&]() {
    if (polygon.base.points.size() > 2) {
      // add first point to close the preview/breadcrumb polygons
      polygon.base.points.push_back(polygon.base.points.front());
      polygon.front_left.points.push_back(polygon.front_left.points.front());
      polygon.front_right.points.push_back(polygon.front_right.points.front());
    } else if (sweptPoseCount(polygon.swept_pose_segments) < 2) {
      success = false;
    }
    ROS_INFO_STREAM("Finished Recording polygon");
  };

  while (true) {
    if (!ros::ok() || aborted) {
      ROS_WARN_STREAM("Preempting Area Recorder");
      success = false;
      break;
    }

    updateRate.sleep();

    if (!has_odom) continue;
    if (!poly_recording_enabled) {
      finish_recording();
      break;
    }

    const auto pose_snapshot = last_pose;
    const auto pose_in_map = pose_snapshot.pose.pose;
    const auto preview_point =
        preview_point_mode == mower_map::BoundarySample::POINT_FRONT_LEFT
            ? projectPoint(pose_in_map, footprint_front_left)
            : preview_point_mode == mower_map::BoundarySample::POINT_FRONT_RIGHT
                  ? projectPoint(pose_in_map, footprint_front_right)
                  : makePoint(pose_in_map.position.x, pose_in_map.position.y);

    std::string gps_quality_reason;
    if (!recordingGpsQualityOk(pose_snapshot, gps_quality_reason)) {
      ROS_WARN_THROTTLE(2.0, "Area recorder skipping polygon point: %s", gps_quality_reason.c_str());
      start_new_swept_segment = true;
      continue;
    }
    addSweptPose(polygon, pose_snapshot, start_new_swept_segment);
    start_new_swept_segment = false;

    if (polygon.base.points.empty()) {
      // add the first point
      geometry_msgs::Point32 pt = preview_point;
      //                ROS_INFO_STREAM("Adding First Point: " << pt);

      addRecordedPoint(polygon, pose_snapshot, 0, auto_point_collecting);
      {
        geometry_msgs::Point vpt;
        vpt.x = pt.x;
        vpt.y = pt.y;
        marker.points.push_back(vpt);
      }

      marker.header.seq++;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = "map";

      marker_pub.publish(marker);

      poly_viz.polygon.points.push_back(pt);
      map_overlay_pub.publish(resultOverlay);
    } else {
      auto last = poly_viz.polygon.points.back();
      tf2::Vector3 last_point(last.x, last.y, 0.0);
      tf2::Vector3 current_point(preview_point.x, preview_point.y, 0.0);

      bool is_new_point_far_enough = (current_point - last_point).length() > NEW_POINT_MIN_DISTANCE;
      bool is_point_auto_collected = auto_point_collecting && is_new_point_far_enough;
      bool is_point_manual_collected = !auto_point_collecting && collect_point && is_new_point_far_enough;

      if (is_point_auto_collected || is_point_manual_collected) {
        geometry_msgs::Point32 pt = preview_point;
        //                    ROS_INFO_STREAM("Adding Point: " << pt);
        addRecordedPoint(polygon, pose_snapshot, polygon.base.points.size(), is_point_auto_collected);
        {
          geometry_msgs::Point vpt;
          vpt.x = pt.x;
          vpt.y = pt.y;
          marker.points.push_back(vpt);
        }

        marker.header.seq++;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";

        marker_pub.publish(marker);

        poly_viz.polygon.points.push_back(pt);
        map_overlay_pub.publish(resultOverlay);

        if (is_point_manual_collected) {
          collect_point = false;
        }
      }
    }

    if (!poly_recording_enabled) {
      finish_recording();
      break;
    }
  }

  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  // close poly
  poly_viz.closed = true;
  poly_viz.line_width = 0.05;
  if (resultOverlay.polygons.size() == 1) {
    poly_viz.color = "green";
  } else {
    poly_viz.color = "red";
  }
  map_overlay_pub.publish(resultOverlay);

  return success;
}

bool AreaRecordingBehavior::getDockingPosition(geometry_msgs::Pose& pos) {
  if (!has_first_docking_pos) {
    ROS_INFO_STREAM("Recording first docking position");

    auto odom_ptr =
        ros::topic::waitForMessage<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", ros::Duration(1, 0));

    first_docking_pos = odom_ptr->pose.pose;
    has_first_docking_pos = true;
    update_actions();
    return false;
  } else {
    ROS_INFO_STREAM("Recording second docking position");

    auto odom_ptr =
        ros::topic::waitForMessage<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", ros::Duration(1, 0));

    pos.position = odom_ptr->pose.pose.position;

    double yaw = atan2(pos.position.y - first_docking_pos.position.y, pos.position.x - first_docking_pos.position.x);
    tf2::Quaternion docking_orientation(0.0, 0.0, yaw);
    pos.orientation = tf2::toMsg(docking_orientation);

    update_actions();
    return true;
  }
}

void AreaRecordingBehavior::command_home() {
  abort();
}

void AreaRecordingBehavior::command_start() {
}

void AreaRecordingBehavior::command_s1() {
}

void AreaRecordingBehavior::command_s2() {
}

bool AreaRecordingBehavior::redirect_joystick() {
  return true;
}

uint8_t AreaRecordingBehavior::get_sub_state() {
  return sub_state;
}

uint8_t AreaRecordingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING;
}

std::string AreaRecordingBehavior::sub_state_name() {
  // yes, this doesnt have a sub_sate, but we'll switch to behavior trees anyways. adding a substate here will break
  // stuff
  if (has_first_docking_pos) {
    return "RECORD_DOCKING_POSITION";
  }
  switch (sub_state) {
    case 0: return "";
    case 1: return "RECORD_OUTLINE";
    case 2: return "RECORD_OBSTACLE";
    default: return "";
  }
}

void AreaRecordingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:area_recording/start_recording") {
    ROS_INFO_STREAM("Got start recording");
    poly_recording_enabled = true;
  } else if (action == "mower_logic:area_recording/stop_recording") {
    ROS_INFO_STREAM("Got stop recording");
    poly_recording_enabled = false;
  } else if (action == "mower_logic:area_recording/finish_navigation_area") {
    ROS_INFO_STREAM("Got save navigation area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/finish_mowing_area") {
    ROS_INFO_STREAM("Got save mowing area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/finish_discard") {
    ROS_INFO_STREAM("Got discard recorded area");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = false;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/exit_recording_mode") {
    ROS_INFO_STREAM("Got exit without saving");
    // stop current poly recording
    poly_recording_enabled = false;

    // set finished
    is_mowing_area = false;
    is_navigation_area = false;
    finished_all = true;
    abort();
  } else if (action == "mower_logic:area_recording/record_dock") {
    ROS_INFO_STREAM("Got record dock");
    set_docking_position = true;
  } else if (action == "mower_logic:area_recording/auto_point_collecting_enable") {
    ROS_INFO_STREAM("Got enable auto point collecting");
    auto_point_collecting = true;
  } else if (action == "mower_logic:area_recording/auto_point_collecting_disable") {
    ROS_INFO_STREAM("Got disable auto point collecting");
    auto_point_collecting = false;
  } else if (action == "mower_logic:area_recording/collect_point") {
    ROS_INFO_STREAM("Got collect point");
    collect_point = true;
  } else if (action == "mower_logic:area_recording/start_manual_mowing") {
    const ros::Time now = ros::Time::now();
    if (!manual_mowing) {
      ROS_INFO_STREAM("Starting manual mowing");
      manual_mowing = true;
      applyManualMowingState(manual_mowing);
    }
    manual_mowing_stop_guard_until = now + ros::Duration(kManualMowingStopGuardSec);
    manual_mowing_stop_pending = false;
  } else if (action == "mower_logic:area_recording/stop_manual_mowing") {
    if (!manual_mowing) {
      ROS_INFO_STREAM("Manual mowing is already stopped");
    } else if (is_manual_mowing_stop_guard_active()) {
      ROS_INFO_STREAM("Deferring stop_manual_mowing until start/stop chatter guard expires");
      manual_mowing_stop_pending = true;
    } else {
      ROS_INFO_STREAM("Stopping manual mowing");
      stopManualMowing(manual_mowing, manual_mowing_stop_guard_until, manual_mowing_stop_pending);
    }
  }
  update_actions();
}

AreaRecordingBehavior::AreaRecordingBehavior() {
  xbot_msgs::ActionInfo start_recording_action;
  start_recording_action.action_id = "start_recording";
  start_recording_action.enabled = false;
  start_recording_action.action_name = "Start Recording";

  xbot_msgs::ActionInfo stop_recording_action;
  stop_recording_action.action_id = "stop_recording";
  stop_recording_action.enabled = false;
  stop_recording_action.action_name = "Stop Recording";

  xbot_msgs::ActionInfo finish_navigation_area_action;
  finish_navigation_area_action.action_id = "finish_navigation_area";
  finish_navigation_area_action.enabled = false;
  finish_navigation_area_action.action_name = "Save Navigation Area";

  xbot_msgs::ActionInfo finish_mowing_area_action;
  finish_mowing_area_action.action_id = "finish_mowing_area";
  finish_mowing_area_action.enabled = false;
  finish_mowing_area_action.action_name = "Save Mowing Area";

  xbot_msgs::ActionInfo exit_recording_mode_action;
  exit_recording_mode_action.action_id = "exit_recording_mode";
  exit_recording_mode_action.enabled = false;
  exit_recording_mode_action.action_name = "Exit";

  xbot_msgs::ActionInfo finish_discard_action;
  finish_discard_action.action_id = "finish_discard";
  finish_discard_action.enabled = false;
  finish_discard_action.action_name = "Discard Area";

  xbot_msgs::ActionInfo record_dock_action;
  record_dock_action.action_id = "record_dock";
  record_dock_action.enabled = false;
  record_dock_action.action_name = "Record Docking point";

  xbot_msgs::ActionInfo auto_point_collecting_enable_action;
  auto_point_collecting_enable_action.action_id = "auto_point_collecting_enable";
  auto_point_collecting_enable_action.enabled = false;
  auto_point_collecting_enable_action.action_name = "Enable automatic point collecting";

  xbot_msgs::ActionInfo auto_point_collecting_disable_action;
  auto_point_collecting_disable_action.action_id = "auto_point_collecting_disable";
  auto_point_collecting_disable_action.enabled = false;
  auto_point_collecting_disable_action.action_name = "Disable automatic point collecting";

  xbot_msgs::ActionInfo collect_point_action;
  collect_point_action.action_id = "collect_point";
  collect_point_action.enabled = false;
  collect_point_action.action_name = "Collect point";

  xbot_msgs::ActionInfo start_manual_mowing_action;
  start_manual_mowing_action.action_id = "start_manual_mowing";
  start_manual_mowing_action.enabled = false;
  start_manual_mowing_action.action_name = "Start manual mowing";

  xbot_msgs::ActionInfo stop_manual_mowing_action;
  stop_manual_mowing_action.action_id = "stop_manual_mowing";
  stop_manual_mowing_action.enabled = false;
  stop_manual_mowing_action.action_name = "Stop manual mowing";

  actions.clear();
  actions.push_back(start_recording_action);
  actions.push_back(stop_recording_action);
  actions.push_back(finish_navigation_area_action);
  actions.push_back(finish_mowing_area_action);
  actions.push_back(exit_recording_mode_action);
  actions.push_back(finish_discard_action);
  actions.push_back(record_dock_action);
  actions.push_back(auto_point_collecting_enable_action);
  actions.push_back(auto_point_collecting_disable_action);
  actions.push_back(collect_point_action);
  actions.push_back(start_manual_mowing_action);
  actions.push_back(stop_manual_mowing_action);
}

void AreaRecordingBehavior::update_actions() {
  {
    for (auto& a : actions) {
      a.enabled = false;
    }
    if (has_first_docking_pos) {
      // we have recorded the first docking pose, only option is to finish by recording second one
      actions[6].enabled = true;
    } else if (poly_recording_enabled) {
      // currently recording a polygon, allow stop and save actions
      actions[1].enabled = true;
      actions[2].enabled = true;
      actions[3].enabled = true;
      actions[4].enabled = true;
      actions[5].enabled = true;

      // enable/disable auto point collecting
      actions[7].enabled = !auto_point_collecting;
      actions[8].enabled = auto_point_collecting;
      actions[9].enabled = !auto_point_collecting;
    } else {
      // neither recording a polygon nor docking point. we can save if we have an outline and always discard
      if (has_outline) {
        actions[0].enabled = true;
        actions[2].enabled = true;
        actions[3].enabled = true;
        actions[4].enabled = true;
        actions[5].enabled = true;
      } else {
        // enable start recording, discard area and record dock
        actions[0].enabled = true;
        actions[4].enabled = true;
        actions[6].enabled = true;
      }
    }
    actions[10].enabled = !manual_mowing;
    actions[11].enabled = manual_mowing;

    registerActions("mower_logic:area_recording", actions);
  }
}

void AreaRecordingBehavior::record_auto_point_collecting(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Recording auto point collecting enabled");
    auto_point_collecting = true;
  } else {
    ROS_INFO_STREAM("Recording auto point collecting disabled");
    auto_point_collecting = false;
  }
}

void AreaRecordingBehavior::record_collect_point(std_msgs::Bool state_msg) {
  if (state_msg.data) {
    ROS_INFO_STREAM("Recording collect point");
    collect_point = true;
  }
}
