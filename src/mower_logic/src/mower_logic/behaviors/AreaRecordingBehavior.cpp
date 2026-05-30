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
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

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

bool footprintPointFromXml(XmlRpc::XmlRpcValue& entry, double& x, double& y) {
  if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) {
    return false;
  }

  const auto value_to_double = [](XmlRpc::XmlRpcValue& value, double& result) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      result = static_cast<double>(value);
      return true;
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      result = static_cast<int>(value);
      return true;
    }
    return false;
  };

  if (!value_to_double(entry[0], x) || !value_to_double(entry[1], y)) {
    return false;
  }
  return std::isfinite(x) && std::isfinite(y);
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
      if (is_mowing_area) {
        ROS_INFO_STREAM("Area recording completed. Adding mowing area.");
        result.area = recorded_outline.front_right;
      } else if (is_navigation_area) {
        ROS_INFO_STREAM("Area recording completed. Adding navigation area.");
        result.area = recorded_outline.base;
      }
      result.obstacles.clear();
      for (const auto& obstacle : recorded_obstacles) {
        result.obstacles.push_back(obstacle.front_left);
      }
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
  loadFootprintRecordingPoints();

  add_mowing_area_client = n->serviceClient<mower_map::AddMowingAreaSrv>("mower_map_service/add_mowing_area");
  set_docking_point_client = n->serviceClient<mower_map::SetDockingPointSrv>("mower_map_service/set_docking_point");

  boundary_sample_pub = n->advertise<mower_map::BoundarySample>("area_recorder/boundary_samples", 1000);
  marker_pub = n->advertise<visualization_msgs::Marker>("area_recorder/progress_visualization", 10);
  map_overlay_pub = n->advertise<xbot_msgs::MapOverlay>("xbot_monitoring/map_overlay", 10);
  marker_array_pub = n->advertise<visualization_msgs::MarkerArray>("area_recorder/progress_visualization_array", 10);
  painted_occupancy_pub = n->advertise<nav_msgs::OccupancyGrid>("area_recorder/painted_area", 1, true);

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
  painted_occupancy_pub.shutdown();
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
  // Sensible Mowrator-rectangle defaults; overridden below if the costmap
  // footprint param is available.
  double right_rear_x = 0.0;
  double right_front_x = 0.82;
  double right_y = -0.34;
  double left_rear_x = 0.0;
  double left_front_x = 0.82;
  double left_y = 0.34;

  ros::param::param<double>("mower_logic/area_recording_paint_cell_size_m", paint_cell_size_m, 0.05);
  if (paint_cell_size_m < 0.01) paint_cell_size_m = 0.01;
  if (paint_cell_size_m > 0.5) paint_cell_size_m = 0.5;
  ros::param::param<double>("mower_logic/area_recording_simplify_epsilon_m", paint_simplify_epsilon_m, 0.04);
  if (paint_simplify_epsilon_m < 0.0) paint_simplify_epsilon_m = 0.0;
  ros::param::param<double>("mower_logic/area_recording_boundary_sample_spacing_m", boundary_sample_min_spacing_m,
                            0.15);
  if (boundary_sample_min_spacing_m < 0.05) boundary_sample_min_spacing_m = 0.05;

  XmlRpc::XmlRpcValue footprint;
  const bool have_footprint = ros::param::get("/move_base_flex/global_costmap/footprint", footprint) ||
                              ros::param::get("/global_costmap/footprint", footprint) ||
                              ros::param::get("/footprint", footprint);

  std::vector<std::pair<double, double>> pts;
  if (have_footprint && footprint.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < footprint.size(); ++i) {
      double x = 0.0;
      double y = 0.0;
      if (footprintPointFromXml(footprint[i], x, y)) {
        pts.emplace_back(x, y);
      }
    }
  }
  if (pts.size() >= 3) {
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& p : pts) {
      min_y = std::min(min_y, p.second);
      max_y = std::max(max_y, p.second);
    }
    const double edge_eps = 0.01;
    std::vector<std::pair<double, double>> right_pts;
    std::vector<std::pair<double, double>> left_pts;
    for (const auto& p : pts) {
      if (p.second <= min_y + edge_eps) right_pts.push_back(p);
      if (p.second >= max_y - edge_eps) left_pts.push_back(p);
    }
    if (right_pts.size() >= 2 && left_pts.size() >= 2) {
      auto by_x = [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
        return a.first < b.first;
      };
      std::sort(right_pts.begin(), right_pts.end(), by_x);
      std::sort(left_pts.begin(), left_pts.end(), by_x);
      right_rear_x = right_pts.front().first;
      right_front_x = right_pts.back().first;
      right_y = right_pts.front().second;
      left_rear_x = left_pts.front().first;
      left_front_x = left_pts.back().first;
      left_y = left_pts.front().second;
    } else {
      ROS_WARN_STREAM("Area recorder could not identify side edges from footprint; using Mowrator fallback.");
    }
  } else {
    ROS_WARN_STREAM("Area recorder could not parse footprint param; using Mowrator fallback rectangle.");
  }

  right_side_rear = makePoint(right_rear_x, right_y);
  right_side_front = makePoint(right_front_x, right_y);
  left_side_rear = makePoint(left_rear_x, left_y);
  left_side_front = makePoint(left_front_x, left_y);

  ROS_INFO_STREAM("Area recorder painted-area params: cell=" << paint_cell_size_m << " m, simplify_eps="
                                                              << paint_simplify_epsilon_m << " m. Right side ["
                                                              << right_rear_x << ".." << right_front_x << "] y=" << right_y
                                                              << ", left side [" << left_rear_x << ".." << left_front_x
                                                              << "] y=" << left_y << ".");
}

void AreaRecordingBehavior::emitMotionBoundarySample(const xbot_msgs::AbsolutePose& pose, uint8_t area_type,
                                                     uint32_t index) {
  // Single-sample emission used by the painted-area recording path. Replaces
  // the previous per-vertex sample emission because polygon vertices no
  // longer come from a per-tick rake selection -- they come from contour
  // extraction at finish time. The alignment helper consumes one sample per
  // ~min_sample_spacing m of motion regardless.
  mower_map::BoundarySample sample;
  sample.header = pose.header;
  if (sample.header.stamp == ros::Time()) {
    sample.header.stamp = ros::Time::now();
  }
  sample.header.frame_id = "map";
  sample.area_type = area_type;
  sample.point_mode = mower_map::BoundarySample::POINT_FRONT_RIGHT;  // historical channel selector used by alignment
  sample.point_index = index;
  sample.fused_pose = pose.pose.pose;
  sample.gps_point = makePoint(pose.pose.pose.position.x, pose.pose.pose.position.y);
  sample.gps_flags = pose.flags;
  sample.gps_accuracy = pose.position_accuracy;
  sample.rtk_fixed = (sample.gps_flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
  sample.auto_collected = true;
  if (has_gps_pose && (ros::Time::now() - last_gps_pose_time).toSec() <= kGpsPoseFreshSec) {
    sample.gps_flags = last_gps_pose.flags;
    sample.gps_accuracy = last_gps_pose.position_accuracy;
    sample.rtk_fixed = (sample.gps_flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
  }
  boundary_sample_pub.publish(sample);
}

void AreaRecordingBehavior::publishPaintedContourOverlay(const PaintedArea& painted,
                                                         xbot_msgs::MapOverlay& result_overlay) const {
  if (painted.empty() || result_overlay.polygons.empty()) return;
  auto& poly_viz = result_overlay.polygons.back();
  poly_viz.polygon.points.clear();
  auto contour = painted.extract_outer_contour();
  if (contour.size() < 4) return;
  contour = PaintedArea::douglas_peucker(contour, paint_simplify_epsilon_m);
  poly_viz.polygon.points.reserve(contour.size());
  for (const auto& p : contour) {
    geometry_msgs::Point32 pt;
    pt.x = p.first;
    pt.y = p.second;
    pt.z = 0.0;
    poly_viz.polygon.points.push_back(pt);
  }
  map_overlay_pub.publish(result_overlay);
}

void AreaRecordingBehavior::publishPaintedOccupancyGrid(const PaintedArea& painted) {
  if (painted.empty()) return;
  int min_i, min_j, max_i, max_j;
  if (!painted.get_bounds(min_i, min_j, max_i, max_j)) return;
  // Pad the bounds slightly so the grid extent stays a few cells away from
  // painted edges (helps the WebUI render the boundary cleanly).
  const int pad = 4;
  min_i -= pad;
  max_i += pad;
  min_j -= pad;
  max_j += pad;
  const int width = max_i - min_i + 1;
  const int height = max_j - min_j + 1;
  // Cap grid size to keep network and CPU costs bounded for very large
  // recordings; oversize grids get downsampled.
  const int kMaxCells = 600 * 600;
  int downsample = 1;
  while (static_cast<int64_t>(width / downsample) * (height / downsample) > kMaxCells) {
    ++downsample;
  }
  const int out_w = std::max(1, width / downsample);
  const int out_h = std::max(1, height / downsample);

  nav_msgs::OccupancyGrid grid;
  grid.header.stamp = ros::Time::now();
  grid.header.frame_id = "map";
  grid.info.resolution = painted.cell_size() * downsample;
  grid.info.width = static_cast<uint32_t>(out_w);
  grid.info.height = static_cast<uint32_t>(out_h);
  grid.info.origin.position.x = painted.cell_corner_x(min_i);
  grid.info.origin.position.y = painted.cell_corner_y(min_j);
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(out_w) * out_h, 0);
  for (int j = 0; j < out_h; ++j) {
    for (int i = 0; i < out_w; ++i) {
      // A downsampled cell is occupied if any source cell within it was painted.
      bool any = false;
      for (int dj = 0; dj < downsample && !any; ++dj) {
        for (int di = 0; di < downsample && !any; ++di) {
          if (painted.is_painted(min_i + i * downsample + di, min_j + j * downsample + dj)) any = true;
        }
      }
      grid.data[j * out_w + i] = any ? 100 : 0;
    }
  }
  painted_occupancy_pub.publish(grid);
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
  ROS_INFO_STREAM("recordNewPolygon (painted-area mode)");

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

  // For obstacles we trace the left side (lawn/obstacle on the mower's
  // left); for outlines we trace the right side. The painted area covers the
  // full rake segment swept by the active side over the recording.
  const RakeSide active_side = (preview_point_mode == mower_map::BoundarySample::POINT_FRONT_LEFT)
                                   ? RakeSide::LEFT
                                   : RakeSide::RIGHT;
  const uint8_t boundary_area_type = (active_side == RakeSide::LEFT) ? mower_map::BoundarySample::AREA_OBSTACLE
                                                                     : mower_map::BoundarySample::AREA_MOW;
  const geometry_msgs::Point32 rake_rear_body = (active_side == RakeSide::RIGHT) ? right_side_rear : left_side_rear;
  const geometry_msgs::Point32 rake_front_body =
      (active_side == RakeSide::RIGHT) ? right_side_front : left_side_front;

  PaintedArea painted(paint_cell_size_m);
  bool have_prev_segment = false;
  geometry_msgs::Point32 prev_rear_world;
  geometry_msgs::Point32 prev_front_world;
  uint32_t boundary_sample_index = 0;
  geometry_msgs::Point32 last_boundary_sample_pos;
  bool have_last_boundary_sample_pos = false;
  ros::Time last_overlay_publish(0);
  ros::Time last_grid_publish(0);
  const ros::Duration overlay_period(0.5);
  const ros::Duration grid_period(0.5);

  while (true) {
    if (!ros::ok() || aborted) {
      ROS_WARN_STREAM("Preempting Area Recorder");
      success = false;
      break;
    }

    updateRate.sleep();

    if (!has_odom) {
      if (!poly_recording_enabled) break;
      continue;
    }

    const auto pose_snapshot = last_pose;
    const auto pose_in_map = pose_snapshot.pose.pose;

    std::string gps_quality_reason;
    const bool gps_ok = recordingGpsQualityOk(pose_snapshot, gps_quality_reason);
    if (!gps_ok) {
      // Skip painting and sample emission while GPS is degraded; the painted
      // region must only reflect periods where the recorded pose is trusted.
      ROS_WARN_THROTTLE(2.0, "Area recorder skipping painted tick: %s", gps_quality_reason.c_str());
      // Force re-init of the swept-quad bridge when GPS comes back so the
      // mower does not paint a phantom strip across the gap.
      have_prev_segment = false;
    } else {
      // Project the current rake segment into world frame and paint the
      // quadrilateral swept since the previous valid tick.
      const auto curr_rear_world = projectPoint(pose_in_map, rake_rear_body);
      const auto curr_front_world = projectPoint(pose_in_map, rake_front_body);
      if (have_prev_segment) {
        painted.paint_swept_quad(prev_rear_world.x, prev_rear_world.y, prev_front_world.x, prev_front_world.y,
                                 curr_rear_world.x, curr_rear_world.y, curr_front_world.x, curr_front_world.y);
      } else {
        painted.paint_segment(curr_rear_world.x, curr_rear_world.y, curr_front_world.x, curr_front_world.y);
      }
      prev_rear_world = curr_rear_world;
      prev_front_world = curr_front_world;
      have_prev_segment = true;

      // Record one base-link breadcrumb per tick on polygon.base so that nav
      // areas (which save polygon.base) still get the driven path.
      const auto base_point = makePoint(pose_in_map.position.x, pose_in_map.position.y);
      polygon.base.points.push_back(base_point);

      // Emit a BoundarySample for the alignment helper every
      // boundary_sample_min_spacing_m of base_link motion.
      const geometry_msgs::Point32 base_pos_pt = base_point;
      bool should_emit_sample = !have_last_boundary_sample_pos;
      if (have_last_boundary_sample_pos) {
        const double dx = base_pos_pt.x - last_boundary_sample_pos.x;
        const double dy = base_pos_pt.y - last_boundary_sample_pos.y;
        should_emit_sample = std::hypot(dx, dy) >= boundary_sample_min_spacing_m;
      }
      if (should_emit_sample) {
        emitMotionBoundarySample(pose_snapshot, boundary_area_type, boundary_sample_index++);
        last_boundary_sample_pos = base_pos_pt;
        have_last_boundary_sample_pos = true;
      }
    }

    // Refresh the live preview polygon and occupancy grid roughly twice a
    // second so the UI shows the painted area growing without flooding the
    // network.
    const ros::Time now = ros::Time::now();
    if (!painted.empty() && now - last_overlay_publish >= overlay_period) {
      publishPaintedContourOverlay(painted, resultOverlay);
      last_overlay_publish = now;
    }
    if (!painted.empty() && now - last_grid_publish >= grid_period) {
      publishPaintedOccupancyGrid(painted);
      last_grid_publish = now;
    }

    if (!poly_recording_enabled) {
      ROS_INFO_STREAM("Finished Recording polygon; extracting painted contour ("
                      << painted.cell_count() << " painted cells)");
      auto contour = painted.extract_outer_contour();
      if (contour.size() < 4) {
        ROS_WARN_STREAM("Painted area has no usable contour; recording failed.");
        success = false;
        break;
      }
      contour = PaintedArea::douglas_peucker(contour, paint_simplify_epsilon_m);

      auto& side_poly = (active_side == RakeSide::RIGHT) ? polygon.front_right : polygon.front_left;
      side_poly.points.clear();
      side_poly.points.reserve(contour.size());
      for (const auto& pt : contour) {
        geometry_msgs::Point32 p;
        p.x = pt.first;
        p.y = pt.second;
        p.z = 0.0;
        side_poly.points.push_back(p);
      }

      // Mirror the painted contour into the inactive side too, so that any
      // downstream consumer that reads either side gets the same polygon.
      auto& inactive_poly = (active_side == RakeSide::RIGHT) ? polygon.front_left : polygon.front_right;
      inactive_poly.points = side_poly.points;

      // The base polygon (used when saving as a navigation area) is the
      // raw base_link breadcrumb path. Close it explicitly.
      if (polygon.base.points.size() > 2) {
        polygon.base.points.push_back(polygon.base.points.front());
      } else {
        // Replace the (possibly empty) base path with the painted contour
        // when there is no meaningful base trail recorded.
        polygon.base.points = side_poly.points;
      }

      publishPaintedContourOverlay(painted, resultOverlay);
      break;
    }
  }

  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  // close the live preview polygon overlay and recolour it to indicate the
  // final shape.
  if (!resultOverlay.polygons.empty()) {
    auto& poly_viz = resultOverlay.polygons.back();
    poly_viz.closed = true;
    poly_viz.line_width = 0.05;
    poly_viz.color = (resultOverlay.polygons.size() == 1) ? "green" : "red";
    map_overlay_pub.publish(resultOverlay);
  }

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
