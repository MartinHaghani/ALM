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

// #define VERBOSE_DEBUG   1

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <mower_logic/PowerConfig.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/HwPower.h>
#include <mower_msgs/HwStatus.h>
#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ios>
#include <limits>
#include <mutex>
#include <sstream>

#include "StateSubscriber.h"
#include "behaviors/AreaRecordingBehavior.h"
#include "behaviors/Behavior.h"
#include "behaviors/IdleBehavior.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include "mbf_msgs/ExePathAction.h"
#include "mbf_msgs/MoveBaseAction.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_map/ApplyMapEditSrv.h"
#include "mower_map/ApplyRecordingEditSrv.h"
#include "mower_map/ClearMapSrv.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/CreateMapSrv.h"
#include "mower_map/DeleteMapSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMapCatalogSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/MapSummary.h"
#include "mower_map/RenameMapSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mower_map/SelectMapSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/MowerControlSrv.h"
#include "ros/ros.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/MapOverlay.h"
#include "xbot_msgs/RegisterActionsSrv.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"

ros::ServiceClient pathClient, mapClient, dockingPointClient, gpsClient, mowClient, emergencyClient, pathProgressClient,
    setNavPointClient, clearNavPointClient, clearMapClient, positioningClient, actionRegistrationClient,
    getMapCatalogClient, createMapClient, selectMapClient, renameMapClient, deleteMapClient, applyMapEditClient;

ros::NodeHandle* n;
ros::NodeHandle* paramNh;

dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>* reconfigServer;
actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;

ros::Publisher cmd_vel_pub, high_level_state_publisher, map_overlay_clear_pub;
mower_logic::MowerLogicConfig last_config;
ll::PowerConfig last_power_config;

StateSubscriber<mower_msgs::Emergency> emergency_state_subscriber{"/hw/emergency"};
StateSubscriber<mower_msgs::HwStatus> status_state_subscriber{"/hw/status"};
StateSubscriber<mower_msgs::HwPower> power_state_subscriber{"/hw/power"};
StateSubscriber<mower_msgs::ESCStatus> left_esc_status_state_subscriber{"/hw/diff_drive/left_esc_status"};
StateSubscriber<mower_msgs::ESCStatus> right_esc_status_state_subscriber{"/hw/diff_drive/right_esc_status"};
StateSubscriber<xbot_msgs::AbsolutePose> pose_state_subscriber{"/xbot_positioning/xb_pose"};
ros::Time joy_vel_time(0.0);

ros::Time last_good_gps(0.0);

std::recursive_mutex mower_logic_mutex;
std::mutex mower_enable_mutex;

mower_msgs::HighLevelStatus high_level_status;

std::atomic<bool> mowerAllowed;

Behavior* currentBehavior = &IdleBehavior::INSTANCE;

std::vector<xbot_msgs::ActionInfo> rootActions;
ros::Time last_v_battery_check;
double max_v_battery_seen = 0.0;

bool mower_has_motor_temp = true;
double manual_drive_linear_scale = 1.0;
double manual_drive_angular_scale = 1.0;
double map_selector_max_start_distance_m = 50.0;
bool map_catalog_clients_ready = false;

/**
 * Some thread safe methods to get a copy of the logic state
 */
ros::Time getLastGoodGPS() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_good_gps;
}

void setLastGoodGPS(ros::Time time) {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  last_good_gps = time;
}

mower_logic::MowerLogicConfig getConfig() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_config;
}

ll::PowerConfig getPowerConfig() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_power_config;
}

void setConfig(mower_logic::MowerLogicConfig c) {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  last_config = c;
  reconfigServer->updateConfig(c);
}

mower_msgs::HwStatus getStatus() {
  return status_state_subscriber.getMessage();
}

mower_msgs::HwPower getPower() {
  return power_state_subscriber.getMessage();
}

xbot_msgs::AbsolutePose getPose() {
  return pose_state_subscriber.getMessage();
}

bool getSelectedMapSummary(mower_map::MapSummary& summary, std::string* error = nullptr) {
  if (!map_catalog_clients_ready) {
    if (error) *error = "map catalog client not initialized";
    return false;
  }

  mower_map::GetMapCatalogSrv catalog_srv;
  if (!getMapCatalogClient.call(catalog_srv) || !catalog_srv.response.success) {
    if (error) {
      *error = catalog_srv.response.message.empty() ? "map catalog unavailable" : catalog_srv.response.message;
    }
    return false;
  }

  for (const auto& candidate : catalog_srv.response.maps) {
    if (candidate.id == catalog_srv.response.selected_map_id || candidate.selected) {
      summary = candidate;
      return true;
    }
  }

  if (error) *error = "no selected map in catalog";
  return false;
}

bool getSelectedMapIdentity(std::string& map_id, std::string& map_hash) {
  mower_map::MapSummary summary;
  if (!getSelectedMapSummary(summary)) {
    map_id.clear();
    map_hash.clear();
    return false;
  }
  map_id = summary.id;
  map_hash = summary.map_hash;
  return true;
}

double distanceToBounds(const mower_map::MapSummary& summary, const xbot_msgs::AbsolutePose& pose) {
  if (!summary.bounds_valid) {
    return std::numeric_limits<double>::infinity();
  }

  const double x = pose.pose.pose.position.x;
  const double y = pose.pose.pose.position.y;
  double dx = 0.0;
  double dy = 0.0;
  if (x < summary.min_x) {
    dx = summary.min_x - x;
  } else if (x > summary.max_x) {
    dx = x - summary.max_x;
  }
  if (y < summary.min_y) {
    dy = summary.min_y - y;
  } else if (y > summary.max_y) {
    dy = y - summary.max_y;
  }
  return std::hypot(dx, dy);
}

bool selectedMapAllowsMowingStart(std::string* reason = nullptr) {
  mower_map::MapSummary summary;
  std::string error;
  if (!getSelectedMapSummary(summary, &error)) {
    if (reason) *reason = error;
    return false;
  }

  if (summary.mowing_area_count == 0) {
    if (reason) *reason = "selected map has no mowing areas";
    return false;
  }

  const auto pose = getPose();
  const double distance = distanceToBounds(summary, pose);
  if (distance <= map_selector_max_start_distance_m) {
    return true;
  }

  std::ostringstream stream;
  stream << "current pose is " << distance << "m from selected map '" << summary.name << "'";
  if (reason) *reason = stream.str();
  return false;
}

bool isMapCatalogMutationAllowed() {
  return currentBehavior == &IdleBehavior::INSTANCE || currentBehavior == &IdleBehavior::DOCKED_INSTANCE;
}

void clearStoredMowingCheckpoint() {
  if (std::remove("checkpoint.bag") != 0 && errno != ENOENT) {
    ROS_WARN_STREAM("Failed to remove checkpoint.bag after map catalog change: " << std::strerror(errno));
  }
}

void clearMapOverlay() {
  if (map_overlay_clear_pub) {
    xbot_msgs::MapOverlay overlay;
    map_overlay_clear_pub.publish(overlay);
  }
}

void afterMapCatalogMutation() {
  clearStoredMowingCheckpoint();
  clearMapOverlay();
}

void setEmergencyMode(bool emergency);

void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions) {
  xbot_msgs::RegisterActionsSrv srv;
  srv.request.node_prefix = prefix;
  srv.request.actions = actions;

  ros::Rate retry_delay(1);
  for (int i = 0; i < 10; i++) {
    if (actionRegistrationClient.call(srv)) {
      ROS_INFO_STREAM("successfully registered actions for " << prefix);
      break;
    }
    ROS_ERROR_STREAM("Error registering actions for " << prefix << ". Retrying.");
    retry_delay.sleep();
  }
}

void clearBehaviorActions() {
  for (const auto* prefix :
       {"mower_logic:idle", "mower_logic:mowing", "mower_logic:docking", "mower_logic:undocking",
        "mower_logic:area_recording"}) {
    registerActions(prefix, {});
  }
}

void setRobotPose(geometry_msgs::Pose& pose) {
  // set the robot pose internally as well. othwerise we need to wait for xbot_positioning to send a new one once it has
  // updated the internal pose.
  auto last_pose = pose_state_subscriber.getMessage();
  last_pose.pose.pose = pose;
  pose_state_subscriber.setMessage(last_pose);

  xbot_positioning::SetPoseSrv pose_srv;
  pose_srv.request.robot_pose = pose;

  ros::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    if (positioningClient.call(pose_srv)) {
      //            ROS_INFO_STREAM("successfully set pose to " << pose);
      success = true;
      break;
    }
    ROS_ERROR_STREAM("Error setting robot pose to " << pose << ". Retrying.");
    retry_delay.sleep();
  }

  if (!success) {
    ROS_ERROR_STREAM("Error setting robot pose. Going to emergency. THIS SHOULD NEVER HAPPEN");
    setEmergencyMode(true);
  }
}

// Abort the currently running behaviour
void abortExecution() {
  if (currentBehavior != nullptr) {
    currentBehavior->abort();
  }
}

bool setGPS(bool enabled) {
  xbot_positioning::GPSControlSrv gps_srv;
  gps_srv.request.gps_enabled = enabled;

  ros::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    if (gpsClient.call(gps_srv)) {
      ROS_INFO_STREAM("successfully set GPS to " << enabled);
      success = true;
      break;
    }
    ROS_ERROR_STREAM("Error setting GPS to " << enabled << ". Retrying.");
    retry_delay.sleep();
  }

  if (!success) {
    ROS_ERROR_STREAM("Error setting GPS. Going to emergency. THIS SHOULD NEVER HAPPEN");
    setEmergencyMode(true);
  }

  return success;
}

/// @brief If the BLADE Motor is not in the requested status (enabled),we call the
///        the mower_service/mow_enabled service to enable/disable. TODO: get feedback about spinup and delay if needed
/// @param enabled
/// @return
bool setMowerEnabled(bool enabled) {
  std::lock_guard<std::mutex> lk{mower_enable_mutex};
  const auto last_config = getConfig();

  if (!last_config.enable_mower && enabled) {
    // ROS_INFO_STREAM("om_mower_logic: setMowerEnabled() - Mower should be enabled but is hard-disabled in the
    // config.");
    enabled = false;
  }

  // status change ?
  const auto last_status = status_state_subscriber.getMessage();
  if (last_status.mow_enabled != enabled) {
    ros::Time started = ros::Time::now();
    mower_msgs::MowerControlSrv mow_srv;
    mow_srv.request.mow_enabled = enabled;
    const bool randomize_mower_direction = paramNh->param<bool>("randomize_mower_direction", false);
    mow_srv.request.mow_direction = randomize_mower_direction ? (started.sec & 0x1) : 1;
    ROS_WARN_STREAM("#### om_mower_logic: setMowerEnabled("
                    << enabled << ", " << static_cast<unsigned>(mow_srv.request.mow_direction) << ") call");

    ros::Rate retry_delay(1);
    bool success = false;
    for (int i = 0; i < 10; i++) {
      if (mowClient.call(mow_srv)) {
        ROS_INFO_STREAM("successfully set mower enabled to "
                        << enabled << " (direction " << static_cast<unsigned>(mow_srv.request.mow_direction) << ")");
        success = true;
        break;
      }
      ROS_ERROR_STREAM("Error setting mower enabled to " << enabled << ". Retrying.");
      retry_delay.sleep();
    }

    if (!success) {
      ROS_ERROR_STREAM("Error setting mower enabled. THIS SHOULD NEVER HAPPEN");
    }

    ROS_WARN_STREAM("#### om_mower_logic: setMowerEnabled("
                    << enabled << ", " << static_cast<unsigned>(mow_srv.request.mow_direction)
                    << ") call completed within " << (ros::Time::now() - started).toSec() << "s");
  }

  // TODO: Spinup feedback & delay
  /*    if (enabled) {
          ROS_INFO_STREAM("enabled mower, waiting for it to speed up");

          // TODO timeout and error
          ros::Time started = ros::Time::now();
          while (true) {
              if (status_time > started) {
                  // we have a current status message, wait for mower to speed up
                  bool mower_running = (last_status.speed_mow_status & 0b10);
                  if (mower_running) {
                      ROS_INFO_STREAM("mower motor started");
                      return true;
                  }
              }
              if (ros::Time::now() - started > ros::Duration(25.0)) {
                  // mower was not able to start
                  ROS_ERROR_STREAM("error starting mower motor...");
                  setMowerEnabled(false);
                  return false;
              }
          }
      }*/

  return true;
}

/// @brief Halt all bot movement
void stopMoving() {
  // ROS_INFO_STREAM("om_mower_logic: stopMoving() - stopping bot movement");
  geometry_msgs::Twist stop;
  stop.angular.z = 0;
  stop.linear.x = 0;
  cmd_vel_pub.publish(stop);
}

/// @brief If the BLADE motor is currently enabled, we stop it
void stopBlade() {
  // ROS_INFO_STREAM("om_mower_logic: stopBlade() - stopping blade motor if running");
  setMowerEnabled(false);
  mowerAllowed = false;
  // ROS_INFO_STREAM("om_mower_logic: stopBlade() - finished");
}

/// @brief Stop BLADE motor and any movement
/// @param emergency
void setEmergencyMode(bool emergency) {
  stopBlade();
  stopMoving();
  mower_msgs::EmergencyStopSrv emergencyStop;
  emergencyStop.request.emergency = emergency;

  ros::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    if (emergencyClient.call(emergencyStop)) {
      ROS_INFO_STREAM("successfully set emergency enabled to " << emergency);
      success = true;
      break;
    }
    ROS_ERROR_STREAM("Error setting emergency enabled to " << emergency << ". Retrying.");
    retry_delay.sleep();
  }

  if (!success) {
    ROS_ERROR_STREAM("Error setting emergency. THIS SHOULD NEVER HAPPEN");
  }
}

void updateUI(const ros::TimerEvent& timer_event) {
  if (currentBehavior == &MowingBehavior::INSTANCE) {
    try {
      high_level_status.current_area = MowingBehavior::INSTANCE.get_current_area();
    } catch (const std::runtime_error& re) {
      // specific handling for runtime_error
      ROS_ERROR_STREAM("Error getting current area: " << re.what());
    }
    try {
      high_level_status.current_path = MowingBehavior::INSTANCE.get_current_path();
    } catch (const std::runtime_error& re) {
      ROS_ERROR_STREAM("Error getting current path: " << re.what());
    }
    try {
      high_level_status.current_path_index = MowingBehavior::INSTANCE.get_current_path_index();
    } catch (const std::runtime_error& re) {
      ROS_ERROR_STREAM("Error getting current path index: " << re.what());
    }
  } else {
    high_level_status.current_area = -1;
    high_level_status.current_path = -1;
    high_level_status.current_path_index = -1;
  }

  if (currentBehavior) {
    high_level_status.state_name = currentBehavior->state_name();
    high_level_status.state = (currentBehavior->get_state() & 0b11111) |
                              (currentBehavior->get_sub_state() << mower_msgs::HighLevelStatus::SUBSTATE_SHIFT);
    high_level_status.sub_state_name = currentBehavior->sub_state_name();
  } else {
    high_level_status.state_name = "NULL";
    high_level_status.sub_state_name = "";
    high_level_status.state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
  }
  high_level_state_publisher.publish(high_level_status);
}

bool isGpsGood() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  // GPS is good if orientation is valid, we have low accuracy and we have a recent GPS update.
  // TODO: think about the "recent gps flag" since it only looks at the time. E.g. if we were standing still this would
  // still pause even if no GPS updates are needed during standstill.
  const auto last_pose = pose_state_subscriber.getMessage();
  return last_pose.orientation_valid && last_pose.position_accuracy < last_config.max_position_accuracy &&
         (last_pose.flags & xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE);
}

/// @brief Called every 0.5s, used to control BLADE motor via mower_enabled variable and stop any movement in case of
/// /odom and /mower/status outages
/// @param timer_event
void checkSafety(const ros::TimerEvent& timer_event) {
  const auto last_status = status_state_subscriber.getMessage();
  const auto last_emergency = emergency_state_subscriber.getMessage();
  const auto last_config = getConfig();
  const auto last_pose = pose_state_subscriber.getMessage();
  const auto last_power = power_state_subscriber.getMessage();
  const auto last_left_esc_state = left_esc_status_state_subscriber.getMessage();
  const auto last_left_esc_state_time = left_esc_status_state_subscriber.getMessageTime();
  const auto last_right_esc_state = right_esc_status_state_subscriber.getMessage();
  const auto last_right_esc_state_time = right_esc_status_state_subscriber.getMessageTime();
  const auto now = ros::Time::now();
  const auto pose_time = pose_state_subscriber.getMessageTime();
  const auto status_time = status_state_subscriber.getMessageTime();
  const auto power_time = power_state_subscriber.getMessageTime();
  const auto last_good_gps = getLastGoodGPS();

  high_level_status.emergency = last_emergency.latched_emergency;
  high_level_status.is_charging = false;

  // Initialize to true, if after all checks it is still true then mower should be enabled.
  mowerAllowed = true;

  // send to idle if emergency and we're not recording
  if (currentBehavior != nullptr) {
    if (last_emergency.latched_emergency) {
      currentBehavior->requestPause(pauseType::PAUSE_EMERGENCY);
    } else {
      currentBehavior->requestContinue(pauseType::PAUSE_EMERGENCY);
    }
  }

  // TODO: Have a single point where we check for this timeout instead of twice (here and in the behavior)
  // check if odometry is current. If not, the GPS was bad so we stop moving.
  // Note that the mowing behavior will pause as well by itself.
  if (now - pose_time > ros::Duration(1.0)) {
    stopBlade();
    stopMoving();
    ROS_WARN_STREAM_THROTTLE(
        5, "om_mower_logic: EMERGENCY pose values stopped. dt was: " << (now - pose_time));
    return;
  }

  // check if status is current. if not, we have a problem since it contains wheel ticks and so on.
  // Since these should never drop out, we enter emergency instead of "only" stopping
  if (now - status_time > ros::Duration(3) || now - power_time > ros::Duration(3)) {
    setEmergencyMode(true);
    ROS_WARN_STREAM_THROTTLE(
        5, "om_mower_logic: EMERGENCY /hw/status or /hw/power values stopped. status dt was: " << (now - status_time)
                                                                                                 << ", power dt was: "
                                                                                                 << (now - power_time));
    return;
  }

  if (now - last_left_esc_state_time > ros::Duration(3) || now - last_right_esc_state_time > ros::Duration(3)) {
    setEmergencyMode(true);
    ROS_WARN_STREAM_THROTTLE(5, "om_mower_logic: EMERGENCY drive ESC telemetry stopped. left dt was: "
                                    << (now - last_left_esc_state_time) << ", right dt was: "
                                    << (now - last_right_esc_state_time));
    return;
  }

  // Mowrator has no low-level-board stop/lift/tilt inputs. Software emergency and drive ESC health
  // are the hardware emergency sources.
  const bool left_drive_esc_disconnected = last_left_esc_state.status == mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  const bool right_drive_esc_disconnected = last_right_esc_state.status == mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  const bool left_drive_esc_fault = last_left_esc_state.status == mower_msgs::ESCStatus::ESC_STATUS_ERROR;
  const bool right_drive_esc_fault = last_right_esc_state.status == mower_msgs::ESCStatus::ESC_STATUS_ERROR;
  const bool drive_esc_fault_or_unexpected_disconnect =
      left_drive_esc_fault || right_drive_esc_fault || left_drive_esc_disconnected || right_drive_esc_disconnected;
  if (drive_esc_fault_or_unexpected_disconnect) {
    setEmergencyMode(true);
    ROS_ERROR_STREAM("EMERGENCY: at least one motor control errored. errors left: "
                     << (last_left_esc_state.status) << ", status right: " << last_right_esc_state.status);
    return;
  }

  // We need orientation and a positional accuracy less than configured
  bool gpsGoodNow = isGpsGood();
  if (gpsGoodNow || last_config.ignore_gps_errors) {
    setLastGoodGPS(ros::Time::now());
    high_level_status.gps_quality_percent =
        1.0 - fmin(1.0, last_pose.position_accuracy / last_config.max_position_accuracy);
    ROS_INFO_STREAM_THROTTLE(10, "GPS quality: " << high_level_status.gps_quality_percent);
  } else {
    // GPS = bad, set quality to 0
    high_level_status.gps_quality_percent = 0;
    if (last_pose.orientation_valid) {
      // set this if we don't even have an orientation
      high_level_status.gps_quality_percent = -1;
    }
    ROS_WARN_STREAM_THROTTLE(1, "Low quality GPS");
  }

  bool gpsTimeout = now - last_good_gps > ros::Duration(last_config.gps_timeout);

  if (gpsTimeout) {
    // GPS = bad, set quality to 0
    high_level_status.gps_quality_percent = 0;
    ROS_WARN_STREAM_THROTTLE(1, "GPS timeout");
  }

  if (currentBehavior != nullptr && currentBehavior->needs_gps()) {
    currentBehavior->setGoodGPS(!gpsTimeout);
    // Stop the mower
    if (gpsTimeout) {
      stopBlade();
      stopMoving();
      return;
    }
  }

  if (currentBehavior != nullptr && currentBehavior->redirect_joystick()) {
    if (now - joy_vel_time > ros::Duration(10)) {
      stopMoving();  // To avoid cmd_vel receive timeout in mower_comms
    }
  }

  // enable the mower (if not aleady) if mowerAllowed is still true after checks and bahavior agrees
  setMowerEnabled(currentBehavior != nullptr && mowerAllowed && currentBehavior->mower_enabled());

  high_level_status.battery_percent = last_power.battery_voltage_valid ? last_power.battery_percentage : 0.0;

  // we are in non emergency, check if we should pause. This could be empty battery or hot mower motor.
  bool dockingNeeded = false;

  std::stringstream dockingReason("Parking: ", std::ios_base::ate | std::ios_base::in | std::ios_base::out);

  if (last_config.manual_pause_mowing) {
    dockingReason << "Manual pause";
    dockingNeeded = true;
  }

  // Dock if below critical voltage to avoid BMS undervoltage protection
  if (!dockingNeeded && last_power.battery_voltage_valid &&
      (last_power.v_battery < last_power_config.battery_critical_voltage)) {
    dockingReason << "Battery voltage min critical: " << last_power.v_battery;
    dockingNeeded = true;
  }

  // Otherwise take the max battery voltage over 20s to ignore droop during short current spikes
  if (last_power.battery_voltage_valid) {
    max_v_battery_seen = std::max<double>(max_v_battery_seen, last_power.v_battery);
  }
  if (now - last_v_battery_check > ros::Duration(20.0)) {
    if (!dockingNeeded && (max_v_battery_seen < last_power_config.battery_empty_voltage)) {
      dockingReason << "Battery average voltage low: " << max_v_battery_seen;
      dockingNeeded = true;
    }
    max_v_battery_seen = 0.0;
    last_v_battery_check = now;
  }

  if (mower_has_motor_temp && !dockingNeeded && last_status.mower_motor_temperature >= last_config.motor_hot_temperature) {
    dockingReason << "Mow motor over temp: " << last_status.mower_motor_temperature;
    dockingNeeded = true;
  }

  if (dockingNeeded && currentBehavior != &DockingBehavior::INSTANCE &&
      currentBehavior != &UndockingBehavior::RETRY_INSTANCE && currentBehavior != &IdleBehavior::INSTANCE &&
      currentBehavior != &IdleBehavior::DOCKED_INSTANCE) {
    ROS_INFO_STREAM(dockingReason.rdbuf());
    abortExecution();
  }
}

void reconfigureCB(mower_logic::MowerLogicConfig& c, uint32_t level) {
  ROS_INFO_STREAM("om_mower_logic: Setting mower_logic config");
  last_config = c;
}

bool highLevelCommand(mower_msgs::HighLevelControlSrvRequest& req, mower_msgs::HighLevelControlSrvResponse& res) {
  switch (req.command) {
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME:
      ROS_INFO_STREAM("COMMAND_HOME");
      if (currentBehavior) {
        currentBehavior->command_home();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_START:
      ROS_INFO_STREAM("COMMAND_START");
      if (currentBehavior) {
        currentBehavior->command_start();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_S1:
      ROS_INFO_STREAM("COMMAND_S1");
      if (currentBehavior) {
        currentBehavior->command_s1();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_S2:
      ROS_INFO_STREAM("COMMAND_S2");
      if (currentBehavior) {
        currentBehavior->command_s2();
      }
      break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS: {
      ROS_WARN_STREAM("COMMAND_DELETE_MAPS");
      if (!isMapCatalogMutationAllowed()) {
        ROS_ERROR_STREAM("Clearing the selected map is only allowed during IDLE!");
        return true;
      }
      mower_map::ClearMapSrv clear_map_srv;
      // TODO check result
      clearMapClient.call(clear_map_srv);
      afterMapCatalogMutation();

      // Abort idle so it refreshes against the now-empty selected map.
      currentBehavior->abort();
    } break;
    case mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY:
      ROS_WARN_STREAM("COMMAND_RESET_EMERGENCY");
      setEmergencyMode(false);
      break;
  }
  return true;
}

bool createMapGated(mower_map::CreateMapSrvRequest& req, mower_map::CreateMapSrvResponse& res) {
  if (!isMapCatalogMutationAllowed()) {
    res.success = false;
    res.message = "Map creation is only allowed while idle.";
    return true;
  }

  if (!createMapClient.call(req, res)) {
    res.success = false;
    res.message = "mower_map_service/create_map unavailable";
    return true;
  }
  if (res.success) afterMapCatalogMutation();
  return true;
}

bool selectMapGated(mower_map::SelectMapSrvRequest& req, mower_map::SelectMapSrvResponse& res) {
  if (!isMapCatalogMutationAllowed()) {
    res.success = false;
    res.message = "Map selection is only allowed while idle.";
    return true;
  }

  if (!selectMapClient.call(req, res)) {
    res.success = false;
    res.message = "mower_map_service/select_map unavailable";
    return true;
  }
  if (res.success) afterMapCatalogMutation();
  return true;
}

bool renameMapGated(mower_map::RenameMapSrvRequest& req, mower_map::RenameMapSrvResponse& res) {
  if (!isMapCatalogMutationAllowed()) {
    res.success = false;
    res.message = "Map rename is only allowed while idle.";
    return true;
  }

  if (!renameMapClient.call(req, res)) {
    res.success = false;
    res.message = "mower_map_service/rename_map unavailable";
    return true;
  }
  return true;
}

bool deleteMapGated(mower_map::DeleteMapSrvRequest& req, mower_map::DeleteMapSrvResponse& res) {
  if (!isMapCatalogMutationAllowed()) {
    res.success = false;
    res.message = "Map deletion is only allowed while idle.";
    res.selected_map_id = "";
    return true;
  }

  if (!deleteMapClient.call(req, res)) {
    res.success = false;
    res.message = "mower_map_service/delete_map unavailable";
    res.selected_map_id = "";
    return true;
  }
  if (res.success) afterMapCatalogMutation();
  return true;
}

bool applyMapEditGated(mower_map::ApplyMapEditSrvRequest& req, mower_map::ApplyMapEditSrvResponse& res) {
  if (!isMapCatalogMutationAllowed()) {
    res.success = false;
    res.message = "Map editing is only allowed while idle.";
    return true;
  }

  if (!applyMapEditClient.call(req, res)) {
    res.success = false;
    res.message = "mower_map_service/apply_map_edit unavailable";
    return true;
  }
  if (res.success) afterMapCatalogMutation();
  return true;
}

bool applyRecordingEdit(mower_map::ApplyRecordingEditSrvRequest& req,
                        mower_map::ApplyRecordingEditSrvResponse& res) {
  if (currentBehavior != &AreaRecordingBehavior::INSTANCE) {
    res.success = false;
    res.message = "Recording edits are only allowed while area recording is active.";
    return true;
  }
  return AreaRecordingBehavior::INSTANCE.applyRecordingEdit(req, res);
}

bool previewMowingPlan(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& res) {
  if (!isMapCatalogMutationAllowed()) {
    res.success = false;
    res.message = "Route preview is only allowed while idle.";
    return true;
  }

  res.success = MowingBehavior::INSTANCE.preview_route_plan(res.message);
  return true;
}

bool setAreaRecordingUseFusedPose(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  res.success = AreaRecordingBehavior::INSTANCE.setUseLocalizationFusion(req.data, res.message);
  return true;
}

void actionReceived(const std_msgs::String::ConstPtr& action) {
  if (action->data == "mower_logic/reset_emergency") {
    ROS_WARN_STREAM("Got reset emergency action.");
    setEmergencyMode(false);
    return;
  }

  if (currentBehavior) {
    currentBehavior->handle_action(action->data);
  }
}

void joyVelReceived(const geometry_msgs::Twist::ConstPtr& joy_vel) {
  joy_vel_time = ros::Time::now();
  if (currentBehavior && currentBehavior->redirect_joystick()) {
    geometry_msgs::Twist scaled = *joy_vel;
    scaled.linear.x *= manual_drive_linear_scale;
    scaled.linear.y *= manual_drive_linear_scale;
    scaled.linear.z *= manual_drive_linear_scale;
    scaled.angular.x *= manual_drive_angular_scale;
    scaled.angular.y *= manual_drive_angular_scale;
    scaled.angular.z *= manual_drive_angular_scale;
    cmd_vel_pub.publish(scaled);
  }
}

void buildRootActions() {
  xbot_msgs::ActionInfo reset_emergency_action;
  reset_emergency_action.action_id = "reset_emergency";
  reset_emergency_action.enabled = true;
  reset_emergency_action.action_name = "Reset Emergency";
  rootActions.push_back(reset_emergency_action);
}

int main(int argc, char** argv) {
  buildRootActions();

  ros::init(argc, argv, "mower_logic");

  n = new ros::NodeHandle();
  paramNh = new ros::NodeHandle("~");
  ros::NodeHandle powerNodeHandle("/hw/services/power");
  mowerAllowed = false;
  mower_has_motor_temp = n->param("/hw/services/diff_drive/mower_xesc/has_motor_temp", true);
  const double default_manual_drive_scale = 1.0;
  manual_drive_linear_scale = n->param("/mower_logic/manual_drive_linear_scale", default_manual_drive_scale);
  manual_drive_angular_scale = n->param("/mower_logic/manual_drive_angular_scale", default_manual_drive_scale);
  map_selector_max_start_distance_m = n->param("/mower_logic/map_selector/max_start_distance_m", 50.0);
  ROS_INFO_STREAM("Manual drive linear scale: " << manual_drive_linear_scale);
  ROS_INFO_STREAM("Manual drive angular scale: " << manual_drive_angular_scale);
  ROS_INFO_STREAM("Map selector max start distance: " << map_selector_max_start_distance_m << "m");

  boost::recursive_mutex mutex;

  reconfigServer = new dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>(mutex, *paramNh);
  reconfigServer->setCallback(reconfigureCB);

  last_power_config = ll::PowerConfig::__getDefault__();
  last_power_config.__fromServer__(powerNodeHandle);

  cmd_vel_pub = n->advertise<geometry_msgs::Twist>("/logic_vel", 1);
  map_overlay_clear_pub = n->advertise<xbot_msgs::MapOverlay>("xbot_monitoring/map_overlay", 10);

  high_level_state_publisher = n->advertise<mower_msgs::HighLevelStatus>("mower_logic/current_state", 100, true);
  MowingBehavior::INSTANCE.advertise_route_plan();
  AreaRecordingBehavior::INSTANCE.initializeRecordingPoseModePublisher(n);

  pathClient = n->serviceClient<slic3r_coverage_planner::PlanPath>("slic3r_coverage_planner/plan_path");
  mapClient = n->serviceClient<mower_map::GetMowingAreaSrv>("mower_map_service/get_mowing_area");
  clearMapClient = n->serviceClient<mower_map::ClearMapSrv>("mower_map_service/clear_map");
  getMapCatalogClient = n->serviceClient<mower_map::GetMapCatalogSrv>("mower_map_service/get_map_catalog");
  createMapClient = n->serviceClient<mower_map::CreateMapSrv>("mower_map_service/create_map");
  selectMapClient = n->serviceClient<mower_map::SelectMapSrv>("mower_map_service/select_map");
  renameMapClient = n->serviceClient<mower_map::RenameMapSrv>("mower_map_service/rename_map");
  deleteMapClient = n->serviceClient<mower_map::DeleteMapSrv>("mower_map_service/delete_map");
  applyMapEditClient = n->serviceClient<mower_map::ApplyMapEditSrv>("mower_map_service/apply_map_edit");
  map_catalog_clients_ready = true;

  gpsClient = n->serviceClient<xbot_positioning::GPSControlSrv>("xbot_positioning/set_gps_state");
  positioningClient = n->serviceClient<xbot_positioning::SetPoseSrv>("xbot_positioning/set_robot_pose");
  actionRegistrationClient = n->serviceClient<xbot_msgs::RegisterActionsSrv>("xbot/register_actions");

  mowClient = n->serviceClient<mower_msgs::MowerControlSrv>("hw/_service/mow_enabled");
  emergencyClient = n->serviceClient<mower_msgs::EmergencyStopSrv>("hw/_service/emergency");

  dockingPointClient = n->serviceClient<mower_map::GetDockingPointSrv>("mower_map_service/get_docking_point");

  pathProgressClient =
      n->serviceClient<ftc_local_planner::PlannerGetProgress>("/move_base_flex/FTCPlanner/planner_get_progress");

  setNavPointClient = n->serviceClient<mower_map::SetNavPointSrv>("mower_map_service/set_nav_point");
  clearNavPointClient = n->serviceClient<mower_map::ClearNavPointSrv>("mower_map_service/clear_nav_point");

  mbfClient = new actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>("/move_base_flex/move_base");
  mbfClientExePath = new actionlib::SimpleActionClient<mbf_msgs::ExePathAction>("/move_base_flex/exe_path");

  emergency_state_subscriber.Start(n);
  status_state_subscriber.Start(n);
  power_state_subscriber.Start(n);
  left_esc_status_state_subscriber.Start(n);
  right_esc_status_state_subscriber.Start(n);
  pose_state_subscriber.Start(n);

  ros::Subscriber joy_cmd = n->subscribe("/joy_vel", 0, joyVelReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber action = n->subscribe("xbot/action", 0, actionReceived, ros::TransportHints().tcpNoDelay(true));

  ros::ServiceServer high_level_control_srv = n->advertiseService("mower_service/high_level_control", highLevelCommand);
  ros::ServiceServer create_map_srv = n->advertiseService("mower_service/create_map", createMapGated);
  ros::ServiceServer select_map_srv = n->advertiseService("mower_service/select_map", selectMapGated);
  ros::ServiceServer rename_map_srv = n->advertiseService("mower_service/rename_map", renameMapGated);
  ros::ServiceServer delete_map_srv = n->advertiseService("mower_service/delete_map", deleteMapGated);
  ros::ServiceServer apply_map_edit_srv = n->advertiseService("mower_service/apply_map_edit", applyMapEditGated);
  ros::ServiceServer apply_recording_edit_srv =
      n->advertiseService("mower_service/apply_recording_edit", applyRecordingEdit);
  ros::ServiceServer preview_mowing_plan_srv =
      n->advertiseService("mower_service/preview_mowing_plan", previewMowingPlan);
  ros::ServiceServer set_area_recording_use_fused_pose_srv =
      n->advertiseService("mower_service/set_area_recording_use_fused_pose", setAreaRecordingUseFusedPose);

  // Keep timers and state subscribers responsive even if one callback is waiting on
  // a service call or transport hiccup during mowing recovery.
  ros::AsyncSpinner asyncSpinner(2);
  asyncSpinner.start();

  ros::Rate r(1.0);

  ROS_INFO("Waiting for emergency message");
  while (!emergency_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }
  ROS_INFO("Waiting for a power message");
  while (!power_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }

  ROS_INFO("Waiting for a status message");
  while (!status_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }

  ROS_INFO("Waiting for a pose message");
  while (!power_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }
  ROS_INFO("Waiting for left ESC status message");
  while (!left_esc_status_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }
  ROS_INFO("Waiting for right ESC status message");
  while (!right_esc_status_state_subscriber.hasMessage()) {
    if (!ros::ok()) {
      delete (reconfigServer);
      delete (mbfClient);
      delete (mbfClientExePath);
      return 1;
    }
    r.sleep();
  }

  ROS_INFO("Waiting for emergency service");
  if (!emergencyClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Emergency server not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }

  ROS_INFO("Checking path server");
  if (!pathClient.waitForExistence(ros::Duration(2.0, 0.0))) {
    ROS_WARN("Path service not available at startup. Mowing plan requests will fail until /slic3r_coverage_planner/plan_path is available.");
  }
  ROS_INFO("Waiting for mower service");
  if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Mower service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }

  ROS_INFO("Waiting for gps service");
  if (!gpsClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("GPS service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }
  ROS_INFO("Waiting for positioning service");
  if (!positioningClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("positioning service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);

    return 1;
  }

  ROS_INFO("Waiting for map server");
  if (!mapClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Map server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }
  ROS_INFO("Waiting for docking point server");
  if (!dockingPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Docking server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }
  ROS_INFO("Waiting for nav point server");
  if (!setNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Set Nav Point server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }
  ROS_INFO("Waiting for clear nav point server");
  if (!clearNavPointClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Clear Nav Point server service not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 2;
  }

  ROS_INFO("Waiting for move base flex");
  if (!mbfClient->waitForServer(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("Move base flex not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 3;
  }

  ROS_INFO("Waiting for mowing path progress server");
  if (!pathProgressClient.waitForExistence(ros::Duration(60.0, 0.0))) {
    ROS_ERROR("FTCLocalPlanner progress server not found.");
    delete (reconfigServer);
    delete (mbfClient);
    delete (mbfClientExePath);
    return 3;
  }

  ros::Time started = ros::Time::now();
  while ((ros::Time::now() - started).toSec() < 10.0) {
    ROS_INFO_STREAM("Waiting for an emergency status message");
    r.sleep();
    if (emergency_state_subscriber.getMessage().latched_emergency) {
      ROS_INFO_STREAM("Got emergency, resetting it");
      setEmergencyMode(false);
      break;
    }
  }

  ROS_INFO("clearing stale behavior actions");
  clearBehaviorActions();

  ROS_INFO("registering actions");
  registerActions("mower_logic", rootActions);

  ROS_INFO("om_mower_logic: Got all servers, we can mow");

  last_v_battery_check = ros::Time::now();
  ros::Timer safety_timer = n->createTimer(ros::Duration(0.5), checkSafety);
  ros::Timer ui_timer = n->createTimer(ros::Duration(1.0), updateUI);

  // release emergency if it was set
  setEmergencyMode(false);

  // initialise the shared state object to be passed into the behaviors
  auto shared_state = std::make_shared<sSharedState>();
  shared_state->active_semiautomatic_task = false;

  // Behavior execution loop
  while (ros::ok()) {
    if (currentBehavior != nullptr) {
      clearBehaviorActions();
      currentBehavior->start(last_config, shared_state);
      Behavior* newBehavior = currentBehavior->execute();
      currentBehavior->exit();
      currentBehavior = newBehavior;
    } else {
      high_level_status.state_name = "NULL";
      high_level_status.state = mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
      high_level_state_publisher.publish(high_level_status);
      // we have no defined behavior, set emergency
      ROS_ERROR_STREAM("null behavior - emergency mode");
      setEmergencyMode(true);
      ros::Rate r(1.0);
      r.sleep();
    }
  }

  delete (n);
  delete (paramNh);
  delete (reconfigServer);
  delete (mbfClient);
  delete (mbfClientExePath);
  return 0;
}
