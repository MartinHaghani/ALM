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
#ifndef SRC_AREA_RECORDING_BEHAVIOR_H
#define SRC_AREA_RECORDING_BEHAVIOR_H

#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mower_map/GetDockingPointSrv.h>
#include <atomic>
#include <mutex>
#include <string>
#include <tf2/LinearMath/Transform.h>

#include "Behavior.h"
#include "DockingBehavior.h"
#include "IdleBehavior.h"
#include "AreaRecordingPoseGate.h"
#include "SweptAreaRecorder.h"
#include "geometry_msgs/Twist.h"
#include "mower_map/AddMowingAreaSrv.h"
#include "mower_map/ApplyRecordingEditSrv.h"
#include "mower_map/BoundarySample.h"
#include "mower_map/MapArea.h"
#include "mower_map/SetDockingPointSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/ActionInfo.h"
#include "xbot_msgs/MapOverlay.h"

#define NEW_POINT_MIN_DISTANCE 0.1

class AreaRecordingBehavior : public Behavior {
 public:
  static AreaRecordingBehavior INSTANCE;

  AreaRecordingBehavior();

 private:
  struct RecordedPolygon {
    geometry_msgs::Polygon base;
    geometry_msgs::Polygon front_left;
    geometry_msgs::Polygon front_right;
    std::vector<std::vector<geometry_msgs::Pose>> swept_pose_segments;
    std::vector<mower_map::BoundarySample> base_samples;
    std::vector<mower_map::BoundarySample> front_left_samples;
    std::vector<mower_map::BoundarySample> front_right_samples;
  };

  struct RecordingPoseSnapshot {
    xbot_msgs::AbsolutePose pose;
    bool boundary_sample_allowed = false;
    xbot_msgs::AbsolutePose boundary_sample_pose;
    xbot_msgs::AbsolutePose boundary_gps_pose;
  };

  bool has_legacy_pose = false;
  bool has_fused_pose = false;
  bool has_gps_pose = false;

  std::vector<xbot_msgs::ActionInfo> actions;

  sensor_msgs::Joy last_joy;
  xbot_msgs::AbsolutePose last_legacy_pose;
  xbot_msgs::AbsolutePose last_fused_pose;
  xbot_msgs::AbsolutePose last_gps_pose;
  ros::Time last_legacy_pose_time = ros::Time(0);
  ros::Time last_fused_pose_time = ros::Time(0);
  ros::Time last_gps_pose_time = ros::Time(0);

  ros::Publisher boundary_sample_pub;
  ros::Publisher use_fused_pose_pub;
  ros::Publisher map_overlay_pub;
  ros::Publisher marker_pub;
  ros::Publisher marker_array_pub;

  ros::Subscriber joy_sub, legacy_pose_sub, fused_pose_sub, gps_pose_sub;

  ros::Subscriber dock_sub, polygon_sub, mow_area_sub, nav_area_sub, auto_point_collecting_sub, collect_point_sub;

  ros::ServiceClient add_mowing_area_client, set_docking_point_client;

  bool has_first_docking_pos = false;
  geometry_msgs::Pose first_docking_pos;

  // true, if we should be recording the current data into a polygon
  bool poly_recording_enabled = false;

  // true, if all polys were recorded and the complete area is finished
  bool is_mowing_area = false;
  bool is_navigation_area = false;
  bool finished_all = false;
  bool set_docking_position = false;
  bool has_outline = false;

  // auto point collecting enabled to true points are collected automatically
  // if distance is greater than NEW_POINT_MIN_DISTANCE during recording
  // otherwise collect_point has to be set to true manually for each point to be recorded
  bool auto_point_collecting = true;
  bool collect_point = false;

  bool manual_mowing = false;
  ros::Time manual_mowing_stop_guard_until = ros::Time(0);
  bool manual_mowing_stop_pending = false;
  double max_recording_gps_accuracy = 0.2;
  double max_recording_pose_age_s = 1.0;
  double max_recording_fused_position_accuracy_m = 0.2;
  double max_recording_fused_yaw_accuracy_rad = 0.15;
  std::string fused_pose_topic = "/localization_fusion/pose";
  std::string legacy_pose_topic = "/xbot_positioning/xb_pose";
  std::atomic_bool use_localization_fusion{true};
  mower_logic::area_recording::SweptAreaOptions swept_area_options;

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  std::vector<geometry_msgs::Point32> footprint_polygon;
  geometry_msgs::Point32 footprint_front_left;
  geometry_msgs::Point32 footprint_front_right;
  std::vector<mower_map::MapEditStroke> pending_recording_edits;
  std::mutex pending_recording_edits_mutex;
  mutable std::mutex pose_mutex;

 private:
  bool recordNewPolygon(RecordedPolygon& polygon, xbot_msgs::MapOverlay& resultOverlay, uint8_t preview_point_mode);
  bool getDockingPosition(geometry_msgs::Pose& pos);
  bool buildSweptBoundary(const RecordedPolygon& polygon,
                          mower_logic::area_recording::BoundarySelection selection,
                          const std::string& label,
                          geometry_msgs::Polygon& result) const;
  bool recordedPolygonValid(const geometry_msgs::Polygon& polygon, const std::string& label) const;
  bool selectedRecordingPose(RecordingPoseSnapshot& snapshot, std::string& reason) const;
  geometry_msgs::Point32 projectPoint(const geometry_msgs::Pose& pose, const geometry_msgs::Point32& offset) const;
  geometry_msgs::Polygon footprintAtPose(const geometry_msgs::Pose& pose) const;
  void addRecordedPoint(RecordedPolygon& polygon,
                        const RecordingPoseSnapshot& snapshot,
                        uint32_t index,
                        bool auto_collected);
  bool addSweptPose(RecordedPolygon& polygon, const xbot_msgs::AbsolutePose& pose, bool start_new_segment);
  bool applyPendingRecordingEdits(mower_map::MapArea& result, std::string& error);
  void appendFootprintPreview(xbot_msgs::MapOverlay& overlay, const geometry_msgs::Pose& pose, const std::string& color);
  void clearPendingRecordingEdits();
  void fused_pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg);
  void gps_pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg);
  void legacy_pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg);
  void loadAreaRecordingParams();
  void loadFootprintRecordingPoints();
  void publishUseLocalizationFusion();
  void publishBoundarySamples(const std::vector<mower_map::BoundarySample>& samples, uint8_t area_type);
  void joy_received(const sensor_msgs::Joy& joy_msg);
  void record_dock_received(std_msgs::Bool state_msg);
  void record_polygon_received(std_msgs::Bool state_msg);
  void record_mowing_received(std_msgs::Bool state_msg);
  void record_navigation_received(std_msgs::Bool state_msg);
  void record_auto_point_collecting(std_msgs::Bool state_msg);
  void record_collect_point(std_msgs::Bool state_msg);

  bool is_manual_mowing_stop_guard_active() const;
  void update_actions();

 public:
  std::string state_name() override;

  std::string sub_state_name() override;

  Behavior* execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  bool needs_gps() override;

  bool mower_enabled() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;

  uint8_t get_sub_state() override;

  uint8_t get_state() override;

  void handle_action(std::string action) override;

  void initializeRecordingPoseModePublisher(ros::NodeHandle* node);
  bool setUseLocalizationFusion(bool enabled, std::string& message);
  bool recordingPolygonActive() const;

  bool applyRecordingEdit(mower_map::ApplyRecordingEditSrvRequest& req,
                          mower_map::ApplyRecordingEditSrvResponse& res);
};

#endif  // SRC_AREA_RECORDING_BEHAVIOR_H
