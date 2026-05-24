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
#include "IdleBehavior.h"

#include <mower_logic/PowerConfig.h>
#include <mower_msgs/HwPower.h>
#include <mower_msgs/HwStatus.h>

#include "MowingBehavior.h"

extern void stopMoving();
extern void stopBlade();
extern void setEmergencyMode(bool emergency);
extern void setGPS(bool enabled);
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

extern ros::ServiceClient dockingPointClient;
extern mower_msgs::HwStatus getStatus();
extern mower_msgs::HwPower getPower();
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);
extern ll::PowerConfig getPowerConfig();
extern dynamic_reconfigure::Server<mower_logic::MowerLogicConfig>* reconfigServer;
extern bool isGpsGood();

extern ros::ServiceClient mapClient;

IdleBehavior IdleBehavior::INSTANCE(false);
IdleBehavior IdleBehavior::DOCKED_INSTANCE(true);

std::string IdleBehavior::state_name() {
  return "IDLE";
}

Behavior* IdleBehavior::execute() {
  // Check, if we have a configured map. If not, print info and go to area recorder
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = 0;
  if (!mapClient.call(mapSrv)) {
    ROS_WARN("We don't have a map configured. Starting Area Recorder!");
    return &AreaRecordingBehavior::INSTANCE;
  }

  // Check, if we have a docking position. If not, print info and go to area recorder
  mower_map::GetDockingPointSrv get_docking_point_srv;
  const bool has_docking_point = dockingPointClient.call(get_docking_point_srv);
  if (!has_docking_point) {
    ROS_WARN_THROTTLE(30, "We don't have a docking point configured. Staying in IDLE until one is recorded.");
  }

  // Keep GPS warm in the normal undocked idle state so a manual mowing start
  // does not trip over a stale "last good GPS" window while xbot_positioning
  // re-accepts RTK updates. Only the docked idle variant should disable GPS.
  setGPS(!stay_docked);

  ros::Rate r(25);
  while (ros::ok()) {
    stopMoving();
    stopBlade();
    const auto last_config = getConfig();
    const auto last_power_config = getPowerConfig();
    const auto last_status = getStatus();
    const auto last_power = getPower();
    const bool gps_ready_for_mowing = last_config.ignore_gps_errors || isGpsGood();
    const bool battery_ready =
        last_power.battery_voltage_valid && last_power.v_battery > last_power_config.battery_empty_voltage;
    const bool start_mowing_enabled = gps_ready_for_mowing && battery_ready;

    if (actions[0].enabled != start_mowing_enabled || !actions[1].enabled) {
      actions[0].enabled = start_mowing_enabled;
      actions[1].enabled = true;
      registerActions("mower_logic:idle", actions);
    }

    const bool automatic_mode = last_config.automatic_mode == eAutoMode::AUTO;
    const bool active_semiautomatic_task =
        last_config.automatic_mode == eAutoMode::SEMIAUTO && shared_state->active_semiautomatic_task;
    const bool mower_ready = battery_ready &&
                             last_status.mower_motor_temperature < last_config.motor_cold_temperature &&
                             !last_config.manual_pause_mowing;

    if (manual_start_mowing.load() || ((automatic_mode || active_semiautomatic_task) && mower_ready)) {
      if (!gps_ready_for_mowing) {
        ROS_WARN_THROTTLE(5, "Cannot start mowing until GPS is good.");
        manual_start_mowing.store(false);
        r.sleep();
        continue;
      }
      if (!battery_ready) {
        ROS_WARN_THROTTLE(5, "Cannot start mowing: battery voltage is below the configured parking threshold or invalid.");
        manual_start_mowing.store(false);
        r.sleep();
        continue;
      }
      if (manual_start_mowing.exchange(false)) {
        MowingBehavior::INSTANCE.start_new_session();
      }
      setGPS(true);
      return &MowingBehavior::INSTANCE;
    }

    if (start_area_recorder.exchange(false)) {
      return &AreaRecordingBehavior::INSTANCE;
    }

    // This gets called if we need to refresh, e.g. on clearing maps
    if (aborted) {
      return &IdleBehavior::INSTANCE;
    }

    r.sleep();
  }

  return nullptr;
}

void IdleBehavior::enter() {
  start_area_recorder.store(false);
  // Reset the docking behavior, to allow docking
  DockingBehavior::INSTANCE.reset();

  // disable it, so that we don't start mowing immediately
  manual_start_mowing.store(false);

  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::reset() {
}

bool IdleBehavior::needs_gps() {
  return false;
}

bool IdleBehavior::mower_enabled() {
  return false;
}

void IdleBehavior::command_home() {
  // IdleBehavior == docked, don't do anything.
}

void IdleBehavior::command_start() {
  // We got start, so we can reset the last manual pause
  auto config = getConfig();
  config.manual_pause_mowing = false;
  setConfig(config);

  manual_start_mowing.store(true);
}

void IdleBehavior::command_s1() {
  start_area_recorder.store(true);
}

void IdleBehavior::command_s2() {
}

bool IdleBehavior::redirect_joystick() {
  return false;
}

uint8_t IdleBehavior::get_sub_state() {
  return 0;
}

uint8_t IdleBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
}

IdleBehavior::IdleBehavior(bool stayDocked) {
  this->stay_docked = stayDocked;

  xbot_msgs::ActionInfo start_mowing_action;
  start_mowing_action.action_id = "start_mowing";
  start_mowing_action.enabled = false;
  start_mowing_action.action_name = "Start Mowing";

  xbot_msgs::ActionInfo start_area_recording_action;
  start_area_recording_action.action_id = "start_area_recording";
  start_area_recording_action.enabled = false;
  start_area_recording_action.action_name = "Start Area Recording";

  actions.clear();
  actions.push_back(start_mowing_action);
  actions.push_back(start_area_recording_action);
}

void IdleBehavior::handle_action(std::string action) {
  if (action == "mower_logic:idle/start_mowing") {
    ROS_INFO_STREAM("Got start_mowing command");
    command_start();
  } else if (action == "mower_logic:idle/start_area_recording") {
    ROS_INFO_STREAM("Got start_area_recording command");
    command_s1();
  }
}
