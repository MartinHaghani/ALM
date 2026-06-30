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

#include "mower_map/map_edit_geometry.h"

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

double ageSec(const ros::Time& now, const ros::Time& stamp) {
  if (stamp == ros::Time(0)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, (now - stamp).toSec());
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
        std::string edit_error;
        if (!applyPendingRecordingEdits(result, edit_error)) {
          ready_to_add_area = false;
          ROS_WARN_STREAM("Area recording was not saved because draft edits failed: " << edit_error);
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
    clearPendingRecordingEdits();
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
  has_legacy_pose = false;
  has_fused_pose = false;
  has_gps_pose = false;
  poly_recording_enabled = false;
  finished_all = false;
  set_docking_position = false;
  markers = visualization_msgs::MarkerArray();
  paused = aborted = false;
  clearPendingRecordingEdits();

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

  legacy_pose_sub = n->subscribe(legacy_pose_topic, 100, &AreaRecordingBehavior::legacy_pose_received, this);
  fused_pose_sub = n->subscribe(fused_pose_topic, 100, &AreaRecordingBehavior::fused_pose_received, this);
  gps_pose_sub = n->subscribe("/hw/position/gps", 100, &AreaRecordingBehavior::gps_pose_received, this);
}

void AreaRecordingBehavior::exit() {
  stopManualMowing(manual_mowing, manual_mowing_stop_guard_until, manual_mowing_stop_pending);

  registerActions("mower_logic:area_recording", {});

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
  legacy_pose_sub.shutdown();
  fused_pose_sub.shutdown();
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

void AreaRecordingBehavior::legacy_pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(pose_mutex);
  last_legacy_pose = *msg;
  last_legacy_pose_time = ros::Time::now();
  has_legacy_pose = true;
}

void AreaRecordingBehavior::fused_pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(pose_mutex);
  last_fused_pose = *msg;
  last_fused_pose_time = ros::Time::now();
  has_fused_pose = true;
}

void AreaRecordingBehavior::gps_pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(pose_mutex);
  last_gps_pose = *msg;
  last_gps_pose_time = ros::Time::now();
  has_gps_pose = true;
}

bool AreaRecordingBehavior::selectedRecordingPose(RecordingPoseSnapshot& snapshot, std::string& reason) const {
  xbot_msgs::AbsolutePose legacy_pose;
  xbot_msgs::AbsolutePose fused_pose;
  xbot_msgs::AbsolutePose raw_gps_pose;
  ros::Time legacy_time;
  ros::Time fused_time;
  ros::Time raw_gps_time;
  bool have_legacy = false;
  bool have_fused = false;
  bool have_raw_gps = false;
  {
    std::lock_guard<std::mutex> lock(pose_mutex);
    legacy_pose = last_legacy_pose;
    fused_pose = last_fused_pose;
    raw_gps_pose = last_gps_pose;
    legacy_time = last_legacy_pose_time;
    fused_time = last_fused_pose_time;
    raw_gps_time = last_gps_pose_time;
    have_legacy = has_legacy_pose;
    have_fused = has_fused_pose;
    have_raw_gps = has_gps_pose;
  }

  const ros::Time now = ros::Time::now();
  const double legacy_age_s = ageSec(now, legacy_time);
  const double fused_age_s = ageSec(now, fused_time);
  const double raw_gps_age_s = ageSec(now, raw_gps_time);
  const bool raw_gps_fresh = have_raw_gps && raw_gps_age_s <= kGpsPoseFreshSec;
  const auto* raw_gps = raw_gps_fresh ? &raw_gps_pose : nullptr;

  mower_logic::area_recording::RecordingPoseGateOptions gate_options;
  gate_options.max_gps_accuracy_m = max_recording_gps_accuracy;
  gate_options.max_pose_age_s = max_recording_pose_age_s;
  gate_options.max_fused_position_accuracy_m = max_recording_fused_position_accuracy_m;
  gate_options.max_fused_yaw_accuracy_rad = max_recording_fused_yaw_accuracy_rad;

  auto select_legacy_pose = [&](const std::string& fused_rejection_reason, bool warn_on_fallback) {
    if (!have_legacy) {
      reason = fused_rejection_reason.empty()
                   ? "Legacy recording pose has not been received"
                   : fused_rejection_reason + "; legacy fallback has not been received";
      return false;
    }

    const auto legacy_gate =
        mower_logic::area_recording::evaluateLegacyRecordingPose(legacy_pose, legacy_age_s, raw_gps, gate_options);
    if (!legacy_gate.accepted) {
      reason = fused_rejection_reason.empty()
                   ? legacy_gate.reason
                   : fused_rejection_reason + "; legacy fallback rejected: " + legacy_gate.reason;
      return false;
    }

    if (warn_on_fallback && !fused_rejection_reason.empty()) {
      ROS_WARN_STREAM_THROTTLE(2.0,
                               "Area recorder falling back to legacy pose because fused pose was rejected: "
                                   << fused_rejection_reason);
    }

    snapshot.pose = legacy_pose;
    snapshot.boundary_sample_allowed = legacy_gate.boundary_sample_allowed;
    if (snapshot.boundary_sample_allowed) {
      snapshot.boundary_sample_pose = legacy_pose;
      snapshot.boundary_gps_pose = raw_gps ? raw_gps_pose : legacy_pose;
    }
    reason.clear();
    return true;
  };

  if (use_localization_fusion.load()) {
    if (!have_fused) {
      return select_legacy_pose("Fused recording pose has not been received", true);
    }
    const auto gate =
        mower_logic::area_recording::evaluateFusedRecordingPose(fused_pose, fused_age_s, raw_gps, gate_options);
    if (!gate.accepted) {
      return select_legacy_pose(gate.reason, true);
    }
    snapshot.pose = fused_pose;
    snapshot.boundary_sample_allowed = false;
    if (gate.boundary_sample_allowed && have_legacy && legacy_age_s <= max_recording_pose_age_s &&
        mower_logic::area_recording::absolutePoseFinite(legacy_pose)) {
      snapshot.boundary_sample_allowed = true;
      snapshot.boundary_sample_pose = legacy_pose;
      snapshot.boundary_gps_pose = raw_gps_pose;
    }
    reason.clear();
    return true;
  }

  return select_legacy_pose("", false);
}

void AreaRecordingBehavior::loadAreaRecordingParams() {
  bool use_fused_pose = true;
  ros::param::param<bool>("/mower_logic/area_recording_use_localization_fusion", use_fused_pose, true);
  use_localization_fusion.store(use_fused_pose);
  ros::param::param<std::string>("/mower_logic/area_recording_fused_pose_topic",
                                 fused_pose_topic,
                                 "/localization_fusion/pose");
  ros::param::param<std::string>("/mower_logic/area_recording_legacy_pose_topic",
                                 legacy_pose_topic,
                                 "/xbot_positioning/xb_pose");
  ros::param::param<double>("/mower_logic/area_recording_max_pose_age_sec", max_recording_pose_age_s, 1.0);
  ros::param::param<double>("/mower_logic/area_recording_max_fused_position_accuracy_m",
                            max_recording_fused_position_accuracy_m,
                            0.2);
  ros::param::param<double>("/mower_logic/area_recording_max_fused_yaw_accuracy_rad",
                            max_recording_fused_yaw_accuracy_rad,
                            0.15);
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
  max_recording_pose_age_s = std::max(0.1, std::min(10.0, max_recording_pose_age_s));
  max_recording_fused_position_accuracy_m =
      std::max(0.01, std::min(5.0, max_recording_fused_position_accuracy_m));
  max_recording_fused_yaw_accuracy_rad =
      std::max(0.01, std::min(M_PI, max_recording_fused_yaw_accuracy_rad));
  publishUseLocalizationFusion();

  ROS_INFO_STREAM("Area recorder pose source params: use_localization_fusion="
                  << use_localization_fusion.load() << ", fused_pose_topic=" << fused_pose_topic
                  << ", legacy_pose_topic=" << legacy_pose_topic << ", max_pose_age_s=" << max_recording_pose_age_s
                  << ", max_fused_position_accuracy_m=" << max_recording_fused_position_accuracy_m
                  << ", max_fused_yaw_accuracy_rad=" << max_recording_fused_yaw_accuracy_rad);
  ROS_INFO_STREAM("Area recorder swept geometry params: pose_step_m=" << swept_area_options.pose_step_m
                                                                      << ", yaw_step_rad="
                                                                      << swept_area_options.yaw_step_rad
                                                                      << ", simplify_epsilon_m="
                                                                      << swept_area_options.simplify_epsilon_m
                                                                      << ", min_polygon_area_m2="
                                                                      << swept_area_options.min_polygon_area_m2);
}

void AreaRecordingBehavior::initializeRecordingPoseModePublisher(ros::NodeHandle* node) {
  bool use_fused_pose = true;
  ros::param::param<bool>("/mower_logic/area_recording_use_localization_fusion", use_fused_pose, true);
  use_localization_fusion.store(use_fused_pose);
  if (node) {
    use_fused_pose_pub = node->advertise<std_msgs::Bool>("area_recorder/use_fused_pose", 1, true);
  }
  publishUseLocalizationFusion();
}

void AreaRecordingBehavior::publishUseLocalizationFusion() {
  if (!use_fused_pose_pub) {
    return;
  }
  std_msgs::Bool message;
  message.data = use_localization_fusion.load();
  use_fused_pose_pub.publish(message);
}

bool AreaRecordingBehavior::setUseLocalizationFusion(bool enabled, std::string& message) {
  if (recordingPolygonActive()) {
    message = "Cannot switch area recording pose source while a polygon is recording.";
    return false;
  }

  use_localization_fusion.store(enabled);
  ros::param::set("/mower_logic/area_recording_use_localization_fusion", enabled);
  publishUseLocalizationFusion();
  message = enabled ? "Area recording will use fused localization pose."
                    : "Area recording will use legacy GPS positioning pose.";
  ROS_INFO_STREAM(message);
  return true;
}

bool AreaRecordingBehavior::recordingPolygonActive() const {
  return poly_recording_enabled;
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

geometry_msgs::Polygon AreaRecordingBehavior::footprintAtPose(const geometry_msgs::Pose& pose) const {
  geometry_msgs::Polygon polygon;
  polygon.points.reserve(footprint_polygon.size() + 1);
  for (const auto& offset : footprint_polygon) {
    polygon.points.push_back(projectPoint(pose, offset));
  }
  if (!polygon.points.empty()) {
    polygon.points.push_back(polygon.points.front());
  }
  return polygon;
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
                                             const RecordingPoseSnapshot& snapshot,
                                             uint32_t index,
                                             bool auto_collected) {
  const auto& pose = snapshot.pose;
  const auto base_point = makePoint(pose.pose.pose.position.x, pose.pose.pose.position.y);
  const auto front_left_point = projectPoint(pose.pose.pose, footprint_front_left);
  const auto front_right_point = projectPoint(pose.pose.pose, footprint_front_right);

  polygon.base.points.push_back(base_point);
  polygon.front_left.points.push_back(front_left_point);
  polygon.front_right.points.push_back(front_right_point);

  if (!snapshot.boundary_sample_allowed) {
    return;
  }

  const auto& boundary_pose = snapshot.boundary_sample_pose;
  const auto& boundary_gps = snapshot.boundary_gps_pose;
  const auto boundary_base_point =
      makePoint(boundary_pose.pose.pose.position.x, boundary_pose.pose.pose.position.y);
  const auto boundary_front_left_point = projectPoint(boundary_pose.pose.pose, footprint_front_left);
  const auto boundary_front_right_point = projectPoint(boundary_pose.pose.pose, footprint_front_right);

  auto make_sample = [&](uint8_t point_mode, const geometry_msgs::Point32& point) {
    mower_map::BoundarySample sample;
    sample.header = boundary_pose.header;
    if (sample.header.stamp == ros::Time()) {
      sample.header.stamp = ros::Time::now();
    }
    sample.header.frame_id = "map";
    sample.point_mode = point_mode;
    sample.point_index = index;
    sample.fused_pose = boundary_pose.pose.pose;
    sample.gps_point = point;
    sample.gps_flags = boundary_gps.flags;
    sample.gps_accuracy = boundary_gps.position_accuracy;
    sample.rtk_fixed = (sample.gps_flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
    sample.auto_collected = auto_collected;
    return sample;
  };

  polygon.base_samples.push_back(make_sample(mower_map::BoundarySample::POINT_BASE, boundary_base_point));
  polygon.front_left_samples.push_back(
      make_sample(mower_map::BoundarySample::POINT_FRONT_LEFT, boundary_front_left_point));
  polygon.front_right_samples.push_back(
      make_sample(mower_map::BoundarySample::POINT_FRONT_RIGHT, boundary_front_right_point));
}

bool AreaRecordingBehavior::addSweptPose(RecordedPolygon& polygon,
                                         const xbot_msgs::AbsolutePose& pose,
                                         bool start_new_segment) {
  if (start_new_segment || polygon.swept_pose_segments.empty()) {
    polygon.swept_pose_segments.emplace_back();
  }

  auto& segment = polygon.swept_pose_segments.back();
  const auto& pose_in_map = pose.pose.pose;
  if (!std::isfinite(pose_in_map.position.x) || !std::isfinite(pose_in_map.position.y)) {
    return false;
  }

  if (!segment.empty()) {
    const auto& previous = segment.back();
    const double distance = poseDistance2D(previous, pose_in_map);
    const double yaw_delta = std::abs(normalizeAngle(yawFromPose(pose_in_map) - yawFromPose(previous)));
    const bool moved_far_enough = distance >= swept_area_options.pose_step_m;
    const bool rotated_far_enough = yaw_delta >= swept_area_options.yaw_step_rad;
    if (!moved_far_enough && !rotated_far_enough) {
      return false;
    }
  }

  segment.push_back(pose_in_map);
  return true;
}

void AreaRecordingBehavior::appendFootprintPreview(xbot_msgs::MapOverlay& overlay,
                                                   const geometry_msgs::Pose& pose,
                                                   const std::string& color) {
  xbot_msgs::MapOverlayPolygon footprint_viz;
  footprint_viz.closed = true;
  footprint_viz.line_width = 0.035;
  footprint_viz.color = color;
  footprint_viz.polygon = footprintAtPose(pose);
  overlay.polygons.push_back(footprint_viz);
}

void AreaRecordingBehavior::clearPendingRecordingEdits() {
  std::lock_guard<std::mutex> lock(pending_recording_edits_mutex);
  pending_recording_edits.clear();
}

bool AreaRecordingBehavior::applyPendingRecordingEdits(mower_map::MapArea& result, std::string& error) {
  std::vector<mower_map::MapEditStroke> edits;
  {
    std::lock_guard<std::mutex> lock(pending_recording_edits_mutex);
    edits = pending_recording_edits;
  }
  if (edits.empty()) {
    return true;
  }

  for (const auto& edit : edits) {
    if (edit.operation == mower_map::MapEditStroke::REPLACE_POLYGON) {
      geometry_msgs::Polygon normalized;
      if (!mower_map::edit_geometry::normalizeReplacementPolygon(edit.replacement_polygon, normalized, error)) {
        error = "invalid replacement polygon: " + error;
        return false;
      }
      result.area = normalized;
      continue;
    }

    if (edit.operation == mower_map::MapEditStroke::BRUSH_ADD) {
      const auto edit_result = mower_map::edit_geometry::applyBrushAdd(result.area, edit.path, edit.brush_diameter_m);
      if (!edit_result.success) {
        error = "brush add failed: " + edit_result.error;
        return false;
      }
      result.area = edit_result.outline;
      continue;
    }

    if (edit.operation == mower_map::MapEditStroke::BRUSH_ERASE) {
      const auto edit_result = mower_map::edit_geometry::applyBrushErase(result.area, edit.path, edit.brush_diameter_m);
      if (!edit_result.success) {
        error = "brush erase failed: " + edit_result.error;
        return false;
      }
      if (!edit_result.extra_outlines.empty()) {
        error = "draft erase split the area; finish recording first and edit the saved map";
        return false;
      }
      result.area = edit_result.outline;
      result.obstacles.insert(result.obstacles.end(), edit_result.obstacles.begin(), edit_result.obstacles.end());
      continue;
    }

    error = "unknown recording edit operation";
    return false;
  }

  error.clear();
  return true;
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
                                                     << " swept segments; recording-pose quality gaps are not bridged.");
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

  // push a new poly to the visualization overlay
  {
    xbot_msgs::MapOverlayPolygon poly_viz;
    poly_viz.closed = false;
    poly_viz.line_width = 0.1;
    poly_viz.color = "blue";
    resultOverlay.polygons.push_back(poly_viz);
  }
  const auto poly_viz_index = resultOverlay.polygons.size() - 1;
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

    if (!poly_recording_enabled) {
      finish_recording();
      break;
    }

    RecordingPoseSnapshot pose_snapshot;
    std::string pose_quality_reason;
    if (!selectedRecordingPose(pose_snapshot, pose_quality_reason)) {
      ROS_WARN_THROTTLE(2.0, "Area recorder skipping polygon point: %s", pose_quality_reason.c_str());
      start_new_swept_segment = true;
      continue;
    }
    const auto& pose_in_map = pose_snapshot.pose.pose.pose;
    const auto preview_point =
        preview_point_mode == mower_map::BoundarySample::POINT_FRONT_LEFT
            ? projectPoint(pose_in_map, footprint_front_left)
            : preview_point_mode == mower_map::BoundarySample::POINT_FRONT_RIGHT
                  ? projectPoint(pose_in_map, footprint_front_right)
                  : makePoint(pose_in_map.position.x, pose_in_map.position.y);

    const bool added_swept_pose = addSweptPose(polygon, pose_snapshot.pose, start_new_swept_segment);
    start_new_swept_segment = false;
    if (added_swept_pose) {
      appendFootprintPreview(resultOverlay, pose_in_map, has_outline ? "red" : "green");
      map_overlay_pub.publish(resultOverlay);
    }

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

      resultOverlay.polygons[poly_viz_index].polygon.points.push_back(pt);
      map_overlay_pub.publish(resultOverlay);
    } else {
      auto& poly_viz = resultOverlay.polygons[poly_viz_index];
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
  auto& poly_viz = resultOverlay.polygons[poly_viz_index];
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
  RecordingPoseSnapshot pose_snapshot;
  std::string reason;
  if (!selectedRecordingPose(pose_snapshot, reason)) {
    ROS_WARN_STREAM("Could not record docking position: " << reason);
    return false;
  }

  if (!has_first_docking_pos) {
    ROS_INFO_STREAM("Recording first docking position");

    first_docking_pos = pose_snapshot.pose.pose.pose;
    has_first_docking_pos = true;
    update_actions();
    return false;
  } else {
    ROS_INFO_STREAM("Recording second docking position");

    pos.position = pose_snapshot.pose.pose.pose.position;

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

bool AreaRecordingBehavior::applyRecordingEdit(mower_map::ApplyRecordingEditSrvRequest& req,
                                               mower_map::ApplyRecordingEditSrvResponse& res) {
  if (poly_recording_enabled) {
    res.success = false;
    res.message = "Pause polygon recording before applying draft edits.";
    return true;
  }
  if (!has_outline && !is_mowing_area && !is_navigation_area) {
    res.success = false;
    res.message = "Record an outline before applying draft edits.";
    return true;
  }

  if (req.edit.operation == mower_map::MapEditStroke::REPLACE_POLYGON) {
    geometry_msgs::Polygon normalized;
    std::string error;
    if (!mower_map::edit_geometry::normalizeReplacementPolygon(req.edit.replacement_polygon, normalized, error)) {
      res.success = false;
      res.message = "Invalid replacement polygon: " + error;
      return true;
    }
    req.edit.replacement_polygon = normalized;
  } else if (req.edit.operation != mower_map::MapEditStroke::BRUSH_ADD &&
             req.edit.operation != mower_map::MapEditStroke::BRUSH_ERASE) {
    res.success = false;
    res.message = "Unknown recording edit operation.";
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(pending_recording_edits_mutex);
    pending_recording_edits.push_back(req.edit);
  }

  res.success = true;
  res.message = "Recording edit queued for the next saved area.";
  return true;
}
