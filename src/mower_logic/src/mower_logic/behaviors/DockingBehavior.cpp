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
#include "DockingBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;

extern void stopMoving();
extern bool setGPS(bool enabled);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

DockingBehavior DockingBehavior::INSTANCE;

DockingBehavior::DockingBehavior() {
  xbot_msgs::ActionInfo abort_docking_action;
  abort_docking_action.action_id = "abort_docking";
  abort_docking_action.enabled = true;
  abort_docking_action.action_name = "Stop Parking";

  actions.clear();
  actions.push_back(abort_docking_action);
}

bool DockingBehavior::approach_docking_point() {
  ROS_INFO_STREAM("Calculating approach path");

  // Calculate a docking approaching point behind the actual docking point
  tf2::Quaternion quat;
  tf2::fromMsg(docking_pose_stamped.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Get the approach start point
  {
    geometry_msgs::PoseStamped docking_approach_point = docking_pose_stamped;
    docking_approach_point.pose.position.x -= cos(yaw) * config.docking_approach_distance;
    docking_approach_point.pose.position.y -= sin(yaw) * config.docking_approach_distance;
    mbf_msgs::MoveBaseGoal moveBaseGoal;
    moveBaseGoal.target_pose = docking_approach_point;
    moveBaseGoal.controller = "FTCPlanner";

    auto result = sendGoalAndWaitUnlessAborted(mbfClient, moveBaseGoal);
    if (aborted || result.state_ != result.SUCCEEDED) {
      return false;
    }
  }

  {
    mbf_msgs::ExePathGoal exePathGoal;

    nav_msgs::Path path;

    ros::Time start_wait_time = ros::Time::now();
    ros::Rate loop_rate(100);
    while (ros::ok() && (ros::Time::now() - start_wait_time) < ros::Duration(config.docking_waiting_time)) {
      loop_rate.sleep();
    }

    int dock_point_count = config.docking_approach_distance * 10.0;
    for (int i = 0; i <= dock_point_count; i++) {
      geometry_msgs::PoseStamped docking_pose_stamped_front = docking_pose_stamped;
      docking_pose_stamped_front.pose.position.x -= cos(yaw) * ((dock_point_count - i) / 10.0);
      docking_pose_stamped_front.pose.position.y -= sin(yaw) * ((dock_point_count - i) / 10.0);
      path.poses.push_back(docking_pose_stamped_front);
    }

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = "FTCPlanner";
    ROS_INFO_STREAM("Executing Docking Approach");

    auto approachResult = sendGoalAndWaitUnlessAborted(mbfClientExePath, exePathGoal);
    if (aborted || approachResult.state_ != approachResult.SUCCEEDED) {
      return false;
    }
  }

  return true;
}

bool DockingBehavior::dock_straight() {
  tf2::Quaternion quat;
  tf2::fromMsg(docking_pose_stamped.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  mbf_msgs::ExePathGoal exePathGoal;

  nav_msgs::Path path;

  int dock_point_count = config.docking_distance * 10.0;
  for (int i = 0; i < dock_point_count; i++) {
    geometry_msgs::PoseStamped docking_pose_stamped_front = docking_pose_stamped;
    docking_pose_stamped_front.pose.position.x += cos(yaw) * (i / 10.0);
    docking_pose_stamped_front.pose.position.y += sin(yaw) * (i / 10.0);
    path.poses.push_back(docking_pose_stamped_front);
  }

  exePathGoal.path = path;
  exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
  exePathGoal.dist_tolerance = 0.1;
  exePathGoal.tolerance_from_action = true;
  exePathGoal.controller = "DockingFTCPlanner";

  mbfClientExePath->sendGoal(exePathGoal);

  bool dockingSuccess = false;
  bool waitingForResult = true;

  ros::Rate r(10);

  while (waitingForResult) {
    r.sleep();

    auto mbfState = mbfClientExePath->getState();

    if (aborted) {
      ROS_INFO_STREAM("Parking aborted.");
      mbfClientExePath->cancelGoal();
      stopMoving();
      dockingSuccess = false;
      waitingForResult = false;
    }

    switch (mbfState.state_) {
      case actionlib::SimpleClientGoalState::ACTIVE:
      case actionlib::SimpleClientGoalState::PENDING:
        break;
      case actionlib::SimpleClientGoalState::SUCCEEDED:
        ROS_INFO_STREAM("Parking stopped because the recorded parking path reached its end pose.");
        mbfClientExePath->cancelGoal();
        dockingSuccess = true;
        stopMoving();
        waitingForResult = false;
        break;
      default:
        ROS_WARN_STREAM("Some error during path execution. Parking failed. status value was: " << mbfState.state_);
        waitingForResult = false;
        stopMoving();
        break;
    }
  }

  // to be safe if the planner sent additional commands after cancel
  stopMoving();

  return dockingSuccess;
}

std::string DockingBehavior::state_name() {
  return "PARKING";
}

Behavior* DockingBehavior::execute() {
  while (!isGPSGood) {
    if (aborted) {
      ROS_INFO_STREAM("Parking aborted.");
      stopMoving();
      return &IdleBehavior::INSTANCE;
    }

    ROS_WARN_STREAM("Waiting for good GPS");
    ros::Duration(1.0).sleep();
  }

  bool approachSuccess = approach_docking_point();

  if (aborted) {
    ROS_INFO_STREAM("Parking aborted.");
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  if (!approachSuccess) {
    ROS_ERROR("Error during parking approach.");

    retryCount++;
    if (retryCount <= config.docking_retry_count) {
      ROS_ERROR("Retrying parking approach");
      return &DockingBehavior::INSTANCE;
    }

    ROS_ERROR("Giving up on parking");
    return &IdleBehavior::INSTANCE;
  }

  // Reset retryCount here would cause an infinite loop
  // reset();

  // Disable GPS
  inApproachMode = false;
  setGPS(false);

  bool docked = dock_straight();

  if (aborted) {
    ROS_INFO_STREAM("Parking aborted.");
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  if (!docked) {
    ROS_ERROR("Error during parking.");

    retryCount++;
    if (retryCount <= config.docking_retry_count && !aborted) {
      ROS_ERROR_STREAM("Retrying parking. Try " << retryCount << " / " << config.docking_retry_count);
      return &DockingBehavior::INSTANCE;
    }

    ROS_ERROR("Giving up on parking");
    // Reset retryCount
    reset();
    return &IdleBehavior::INSTANCE;
  }

  // Reset retryCount
  reset();

  return &IdleBehavior::INSTANCE;
}

void DockingBehavior::enter() {
  paused = aborted = false;
  // start with target approach and then dock later
  inApproachMode = true;

  // Get the docking pose in map
  mower_map::GetDockingPointSrv get_docking_point_srv;
  dockingPointClient.call(get_docking_point_srv);
  docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = ros::Time::now();

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:docking", actions);
}

void DockingBehavior::exit() {
  registerActions("mower_logic:docking", {});
}

void DockingBehavior::reset() {
  retryCount = 0;
}

bool DockingBehavior::needs_gps() {
  // we only need GPS if we're in approach mode
  return inApproachMode;
}

bool DockingBehavior::mower_enabled() {
  // No mower during docking
  return false;
}

void DockingBehavior::command_home() {
}

void DockingBehavior::command_start() {
  this->abort();
}

void DockingBehavior::command_s1() {
}

void DockingBehavior::command_s2() {
}

bool DockingBehavior::redirect_joystick() {
  return false;
}

uint8_t DockingBehavior::get_sub_state() {
  return 1;
}

uint8_t DockingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void DockingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:docking/abort_docking") {
    ROS_INFO_STREAM("Got abort docking command");
    command_start();
  }
}
