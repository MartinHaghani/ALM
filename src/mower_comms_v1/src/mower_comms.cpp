//
// Created by Clemens Elflein on 15.03.22.
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
#include <dynamic_reconfigure/client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mower_logic/PowerConfig.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/Power.h>
#include <mower_msgs/Status.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <xbot_msgs/WheelTick.h>
#include <xesc_driver/xesc_driver.h>
#include <xesc_msgs/XescStateStamped.h>

#include <algorithm>
#include <bitset>
#include <cmath>

#include "COBS.h"
#include "boost/crc.hpp"
#include "ll_datatypes.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/ImuRaw.h"
#include "mower_msgs/MowerControlSrv.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

namespace {
constexpr const char* kLogicReconfigureService = "/mower_logic/set_parameters";
constexpr const char* kPowerReconfigureService = "/ll/services/power/set_parameters";
constexpr double kLlEmergencyReleaseGraceSeconds = 0.35;

enum class MowControlMode {
  DUTY,
  CURRENT,
  RPM,
};

struct ActuatorCommands {
  float left_duty = 0.0f;
  float right_duty = 0.0f;
  float mow_duty = 0.0f;
  float mow_current = 0.0f;
  float mow_brake_current = 0.0f;
  float mow_rpm = 0.0f;
  bool mow_use_current_override = false;
};

bool isMowCommandEnabled(MowControlMode mode, float target_duty, float target_current, float target_rpm) {
  switch (mode) {
    case MowControlMode::RPM:
      return target_rpm != 0.0f;
    case MowControlMode::CURRENT:
      return target_current != 0.0f;
    case MowControlMode::DUTY:
    default:
      return target_duty != 0.0f;
  }
}
}

ros::Publisher status_pub;
ros::Publisher power_pub;
ros::Publisher emergency_pub;
ros::Publisher actual_twist_pub;
ros::Publisher status_left_esc_pub;
ros::Publisher status_right_esc_pub;
ros::Publisher sensor_imu_pub;

COBS cobs;

// True, if ROS thinks there sould be an emergency
bool emergency_high_level = false;
// True, if the LL board thinks there should be an emergency
bool emergency_low_level = false;
// Bits are set showing which emergency is active
uint8_t active_low_level_emergency = 0;

// True, if the LL emergency should be cleared in the next request
bool ll_clear_emergency = false;
ros::Time ll_clear_requested_at(0.0);

// True, if we can send to the low level board
bool allow_send = false;
bool configured_ignore_low_level_emergency_inputs = false;

// Current speeds (duty cycle) for the drive ESCs, plus mower command state
float speed_l = 0, speed_r = 0;
float applied_speed_l = 0, applied_speed_r = 0;
float target_speed_mow = 0;
float target_current_mow = 0;
float target_rpm_mow = 0;
float configured_target_duty_mow = 1;
float configured_target_current_mow = 15;
float configured_target_rpm_mow = 3200;
float configured_stop_brake_current_mow = 0;
float configured_startup_boost_duty_mow = 0;
float configured_startup_boost_current_mow = 0;
double configured_target_duty_ramp_seconds = 0.8;
double configured_drive_command_scale = 1.0;
double configured_mowing_drive_command_scale = 1.0;
double configured_drive_command_ramp_up_seconds = 0.0;
double configured_drive_command_ramp_down_seconds = 0.0;
double configured_startup_boost_duration_seconds = 0.0;
double configured_startup_boost_rpm_threshold = 0.0;
double configured_startup_boost_release_rpm_threshold = 0.0;
double configured_stop_brake_duration_seconds = 0.0;
MowControlMode mow_control_mode = MowControlMode::DUTY;
ros::Time mow_enable_started_at(0.0);
ros::Time mow_disable_started_at(0.0);
double last_observed_mower_motor_rpm = 0.0;
bool mow_started_from_rest = false;

// Ticks / m and wheel distance for this robot
double wheel_ticks_per_m = 0.0;
double wheel_distance_m = 0.0;
bool left_xesc_invert_direction = false;
bool right_xesc_invert_direction = true;

// LL/HL configuration
struct ll_high_level_config llhl_config;

dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>* reconfigClient;
dynamic_reconfigure::Client<ll::PowerConfig>* powerReconfigClient;
mower_logic::MowerLogicConfig mower_logic_config;
ll::PowerConfig power_config;

// Serial port and buffer for the low level connection
serial::Serial serial_port;
uint8_t out_buf[1000];
ros::Time last_cmd_vel(0.0);
ros::Time last_drive_command_update(0.0);

boost::crc_ccitt_type crc;

mower_msgs::HighLevelStatus last_high_level_status;

xesc_driver::XescDriver* mow_xesc_interface;
xesc_driver::XescDriver* left_xesc_interface;
xesc_driver::XescDriver* right_xesc_interface;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
uint32_t last_ticks_l = 0;
uint32_t last_ticks_r = 0;
ros::Time last_ticks_stamp{};
geometry_msgs::TwistStamped measured_twist_msg{};

std::mutex ll_status_mutex;
struct ll_status last_ll_status = {0};

sensor_msgs::Imu sensor_imu_msg;

ros::ServiceClient highLevelClient;

bool is_emergency() {
  return emergency_high_level || emergency_low_level;
}

float clampDriveDuty(float duty) {
  if (duty >= 1.0f) {
    return 1.0f;
  }
  if (duty <= -1.0f) {
    return -1.0f;
  }
  return duty;
}

float moveToward(float current, float target, float max_delta) {
  if (max_delta <= 0.0f) {
    return target;
  }
  if (target > current) {
    return std::min(target, current + max_delta);
  }
  return std::max(target, current - max_delta);
}

float getDriveCommandScale() {
  const bool mowing_state = last_high_level_status.state_name == "MOWING";
  const double configured_scale = mowing_state ? configured_mowing_drive_command_scale : configured_drive_command_scale;
  return clampDriveDuty(static_cast<float>(configured_scale));
}

void updateDriveDutyCommands(float target_left, float target_right, const ros::Time& now, bool force_stop) {
  target_left = clampDriveDuty(target_left);
  target_right = clampDriveDuty(target_right);

  double dt = 0.02;
  if (last_drive_command_update != ros::Time(0.0)) {
    dt = std::max(0.0, (now - last_drive_command_update).toSec());
  }
  last_drive_command_update = now;

  if (force_stop) {
    applied_speed_l = 0.0f;
    applied_speed_r = 0.0f;
    return;
  }

  const auto ramp_channel = [dt](float current, float target) {
    const bool changing_direction = current != 0.0f && target != 0.0f && ((current > 0.0f) != (target > 0.0f));
    const bool increasing_magnitude = std::abs(target) > std::abs(current);
    const double ramp_seconds =
        (changing_direction || !increasing_magnitude) ? configured_drive_command_ramp_down_seconds
                                                      : configured_drive_command_ramp_up_seconds;
    if (ramp_seconds <= 0.0) {
      return target;
    }
    return moveToward(current, target, static_cast<float>(dt / ramp_seconds));
  };

  applied_speed_l = ramp_channel(applied_speed_l, target_left);
  applied_speed_r = ramp_channel(applied_speed_r, target_right);
}

ActuatorCommands getActuatorCommands() {
  ActuatorCommands commands;
  const bool drive_force_stop = is_emergency() || (ros::Time::now() - last_cmd_vel > ros::Duration(1.0));
  const float drive_scale = getDriveCommandScale();
  updateDriveDutyCommands(speed_l * drive_scale, speed_r * drive_scale, ros::Time::now(), drive_force_stop);

  commands.left_duty = applied_speed_l;
  commands.right_duty = applied_speed_r;
  commands.mow_duty = target_speed_mow;
  commands.mow_current = target_current_mow;
  commands.mow_rpm = target_rpm_mow;

  // emergency or timeout -> send 0 speeds
  if (is_emergency()) {
    commands.left_duty = 0;
    commands.right_duty = 0;
    commands.mow_duty = 0;
    commands.mow_current = 0;
    commands.mow_rpm = 0;
  }
  if (ros::Time::now() - last_cmd_vel > ros::Duration(1.0)) {
    commands.left_duty = 0;
    commands.right_duty = 0;
  }
  const double observed_mower_motor_rpm = std::abs(last_observed_mower_motor_rpm);
  const bool startup_boost_active =
      mow_control_mode == MowControlMode::DUTY && mow_started_from_rest && commands.mow_duty != 0.0f &&
      configured_startup_boost_duration_seconds > 0.0 &&
      ros::Time::now() - mow_enable_started_at <= ros::Duration(configured_startup_boost_duration_seconds) &&
      (configured_startup_boost_release_rpm_threshold <= 0.0 ||
       observed_mower_motor_rpm < configured_startup_boost_release_rpm_threshold);
  if (startup_boost_active && configured_startup_boost_current_mow > 0.0f) {
    commands.mow_current = std::copysign(configured_startup_boost_current_mow, commands.mow_duty);
    commands.mow_use_current_override = true;
  }
  if (mow_control_mode == MowControlMode::DUTY && commands.mow_duty != 0.0f && configured_target_duty_ramp_seconds > 0.0) {
    const float target_mow_duty = commands.mow_duty;
    const float target_mow_duty_abs = std::abs(target_mow_duty);
    float ramp_start_abs = 0.0f;
    if (mow_started_from_rest && configured_startup_boost_duty_mow > 0.0f) {
      ramp_start_abs = std::min(configured_startup_boost_duty_mow, target_mow_duty_abs);
    }
    const double ramp_progress =
        std::min(1.0, (ros::Time::now() - mow_enable_started_at).toSec() / configured_target_duty_ramp_seconds);
    const float ramped_mow_duty_abs =
        ramp_start_abs + ((target_mow_duty_abs - ramp_start_abs) * static_cast<float>(ramp_progress));
    commands.mow_duty = std::copysign(ramped_mow_duty_abs, target_mow_duty);
  }
  if (!isMowCommandEnabled(mow_control_mode, commands.mow_duty, commands.mow_current, commands.mow_rpm) &&
      configured_stop_brake_current_mow > 0.0f && configured_stop_brake_duration_seconds > 0.0 &&
      mow_disable_started_at != ros::Time(0.0) &&
      ros::Time::now() - mow_disable_started_at <= ros::Duration(configured_stop_brake_duration_seconds)) {
    commands.mow_brake_current = configured_stop_brake_current_mow;
  }

  return commands;
}

void publishEscActuators(const ActuatorCommands& commands) {
  // Keep drive commands first so mower-side startup stalls cannot make teleop feel sluggish.
  left_xesc_interface->setDutyCycle(left_xesc_invert_direction ? -commands.left_duty : commands.left_duty);
  right_xesc_interface->setDutyCycle(right_xesc_invert_direction ? -commands.right_duty : commands.right_duty);

  if (mow_xesc_interface) {
    if (commands.mow_brake_current != 0.0f) {
      mow_xesc_interface->setBrake(commands.mow_brake_current);
    } else if (commands.mow_use_current_override) {
      mow_xesc_interface->setCurrent(commands.mow_current);
    } else if (mow_control_mode == MowControlMode::RPM) {
      mow_xesc_interface->setSpeed(commands.mow_rpm);
    } else if (mow_control_mode == MowControlMode::CURRENT) {
      mow_xesc_interface->setCurrent(commands.mow_current);
    } else {
      mow_xesc_interface->setDutyCycle(commands.mow_duty);
    }
  }
}

void publishActuators() {
  publishEscActuators(getActuatorCommands());

  struct ll_heartbeat heartbeat = {.type = PACKET_ID_LL_HEARTBEAT,
                                   // If high level has emergency and LL does not know yet, we set it
                                   .emergency_requested = (!emergency_low_level && emergency_high_level),
                                   .emergency_release_requested = ll_clear_emergency};

  crc.reset();
  crc.process_bytes(&heartbeat, sizeof(struct ll_heartbeat) - 2);
  heartbeat.crc = crc.checksum();

  size_t encoded_size = cobs.encode((uint8_t*)&heartbeat, sizeof(struct ll_heartbeat), out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  if (serial_port.isOpen() && allow_send) {
    try {
      serial_port.write(out_buf, encoded_size);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Error writing to serial port");
    }
  }
}

void convertStatus(xesc_msgs::XescStateStamped& vesc_status, mower_msgs::ESCStatus& ros_esc_status) {
  if (vesc_status.state.connection_state != xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED &&
      vesc_status.state.connection_state != xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW) {
    // ESC is disconnected
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (vesc_status.state.fault_code) {
    ROS_ERROR_STREAM_THROTTLE(1, "Motor controller fault code: " << vesc_status.state.fault_code);
    // ESC has a fault
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
  } else {
    // ESC is OK but standing still
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
  }
  ros_esc_status.tacho = vesc_status.state.tacho;
  ros_esc_status.rpm = vesc_status.state.rpm;
  ros_esc_status.current = vesc_status.state.current_input;
  ros_esc_status.temperature_motor = vesc_status.state.temperature_motor;
  ros_esc_status.temperature_pcb = vesc_status.state.temperature_pcb;
}

void convertStatus(xesc_msgs::XescStateStamped& vesc_status, uint8_t& esc_status, double& esc_temperature,
                   double& esc_current, double& motor_temperature, double& motor_rpm) {
  if (vesc_status.state.connection_state != xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED &&
      vesc_status.state.connection_state != xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW) {
    // ESC is disconnected
    esc_status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (vesc_status.state.fault_code) {
    ROS_ERROR_STREAM_THROTTLE(1, "Motor controller fault code: " << vesc_status.state.fault_code);
    // ESC has a fault
    esc_status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
  } else {
    // ESC is OK but standing still
    esc_status = mower_msgs::ESCStatus::ESC_STATUS_OK;
  }
  motor_rpm = vesc_status.state.rpm;
  esc_current = vesc_status.state.current_input;
  motor_temperature = vesc_status.state.temperature_motor;
  esc_temperature = vesc_status.state.temperature_pcb;
}

void publishStatus() {
  mower_msgs::Status status_msg;
  status_msg.stamp = ros::Time::now();

  if (last_ll_status.status_bitmask & 1) {
    // LL OK, fill the message
    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
  } else {
    // LL initializing
    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_INITIALIZING;
  }

  status_msg.raspberry_pi_power = (last_ll_status.status_bitmask & 0b00000010) != 0;
  status_msg.is_charging = (last_ll_status.status_bitmask & 0b00000100) != 0;
  status_msg.esc_power = (last_ll_status.status_bitmask & 0b00001000) != 0;
  status_msg.rain_detected = (last_ll_status.status_bitmask & 0b00010000) != 0;
  status_msg.sound_module_available = (last_ll_status.status_bitmask & 0b00100000) != 0;
  status_msg.sound_module_busy = (last_ll_status.status_bitmask & 0b01000000) != 0;
  status_msg.ui_board_available = (last_ll_status.status_bitmask & 0b10000000) != 0;
  status_msg.mow_enabled = isMowCommandEnabled(mow_control_mode, target_speed_mow, target_current_mow, target_rpm_mow);

  // Distinguish active emergency inputs from the low-level board's aggregate latch bit.
  // Some LL firmware revisions can keep the latch bit asserted briefly after a clear request
  // even when no active stop/lift source remains. Keep the release request asserted, but stop
  // blocking the mower once we only see a stale latch after an operator-triggered clear.
  const uint8_t low_level_emergency_bitmask = last_ll_status.emergency_bitmask;
  const bool low_level_latch_bit = (low_level_emergency_bitmask & LL_EMERGENCY_BIT_LATCH) != 0;
  active_low_level_emergency = low_level_emergency_bitmask & ~LL_EMERGENCY_BIT_LATCH;
  const bool latch_only_emergency = low_level_latch_bit && active_low_level_emergency == 0;
  const bool stale_latch_after_release =
      latch_only_emergency && ll_clear_emergency && ll_clear_requested_at != ros::Time(0.0) &&
      ros::Time::now() - ll_clear_requested_at > ros::Duration(kLlEmergencyReleaseGraceSeconds);

  if (configured_ignore_low_level_emergency_inputs) {
    active_low_level_emergency = 0;
    emergency_low_level = false;
    ll_clear_emergency = false;
    ll_clear_requested_at = ros::Time(0.0);
  } else {
    emergency_low_level = active_low_level_emergency > 0 || (low_level_latch_bit && !stale_latch_after_release);
    if (!low_level_latch_bit && active_low_level_emergency == 0) {
      // The low-level board has fully released the emergency latch; stop requesting release.
      ll_clear_emergency = false;
      ll_clear_requested_at = ros::Time(0.0);
    } else if (stale_latch_after_release) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Ignoring stale low-level emergency latch bit after release request. Bitmask was: "
                   << static_cast<int>(low_level_emergency_bitmask));
    } else {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Low Level Emergency. Bitmask was: " << static_cast<int>(low_level_emergency_bitmask));
    }
  }

  // True, if high or low level emergency condition is present
  mower_msgs::Emergency emergency_msg{};
  emergency_msg.stamp = status_msg.stamp;
  emergency_msg.active_emergency = active_low_level_emergency > 0;
  emergency_msg.latched_emergency = is_emergency();
  emergency_msg.reason = "";
  emergency_pub.publish(emergency_msg);

  mower_msgs::Power power_msg{};
  power_msg.stamp = status_msg.stamp;
  power_msg.v_battery = last_ll_status.v_system;
  power_msg.v_charge = last_ll_status.v_charge;
  power_msg.charge_current = last_ll_status.charging_current;
  power_msg.charger_enabled = (last_ll_status.status_bitmask & LL_STATUS_BIT_CHARGING) != 0;
  power_msg.charger_status = "N/A";
  power_pub.publish(power_msg);

  xesc_msgs::XescStateStamped mow_status{}, left_status{}, right_status{};
  if (mow_xesc_interface) {
    mow_xesc_interface->getStatus(mow_status);
  } else {
    mow_status.state.connection_state = xesc_msgs::XescState::XESC_CONNECTION_STATE_DISCONNECTED;
  }
  left_xesc_interface->getStatus(left_status);
  right_xesc_interface->getStatus(right_status);

  mower_msgs::ESCStatus left_esc_status{};
  mower_msgs::ESCStatus right_esc_status{};

  // convertStatus(mow_status, status_msg.mow_esc_status);
  convertStatus(left_status, left_esc_status);
  convertStatus(right_status, right_esc_status);

  status_msg.mower_esc_current = static_cast<float>(mow_status.state.current_input);
  status_msg.mower_esc_status = mow_status.state.connection_state;
  status_msg.mower_motor_rpm = mow_status.state.rpm;
  last_observed_mower_motor_rpm = mow_status.state.rpm;
  status_msg.mower_esc_temperature = static_cast<float>(mow_status.state.temperature_pcb);
  status_msg.mower_motor_temperature = static_cast<float>(mow_status.state.temperature_motor);

  status_pub.publish(status_msg);
  status_left_esc_pub.publish(left_esc_status);
  status_right_esc_pub.publish(right_esc_status);

  if (!has_ticks) {
    last_ticks_stamp = status_msg.stamp;
    last_ticks_l = left_status.state.tacho_absolute;
    last_ticks_r = right_status.state.tacho_absolute;
    has_ticks = true;
  } else {
    bool wheel_direction_l =
        (left_status.state.direction != left_xesc_invert_direction) && abs(left_status.state.duty_cycle) > 0;
    bool wheel_direction_r =
        (right_status.state.direction != right_xesc_invert_direction) && abs(right_status.state.duty_cycle) > 0;

    double dt = (status_msg.stamp - last_ticks_stamp).toSec();

    double d_wheel_l = (double)(left_status.state.tacho_absolute - last_ticks_l) * (1 / wheel_ticks_per_m);
    double d_wheel_r = (double)(right_status.state.tacho_absolute - last_ticks_r) * (1 / wheel_ticks_per_m);

    if (wheel_direction_l) {
      d_wheel_l *= -1.0;
    }
    if (wheel_direction_r) {
      d_wheel_r *= -1.0;
    }

    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    double vx = d_ticks / dt;
    double vr = -(d_wheel_l + d_wheel_r) / (2.0f * dt);
    last_ticks_stamp = status_msg.stamp;
    last_ticks_l = left_status.state.tacho_absolute;
    last_ticks_r = right_status.state.tacho_absolute;

    measured_twist_msg.header.frame_id = "base_link";
    measured_twist_msg.header.stamp = status_msg.stamp;
    measured_twist_msg.header.seq++;
    measured_twist_msg.twist.linear.x = vx;
    measured_twist_msg.twist.angular.z = vr;

    actual_twist_pub.publish(measured_twist_msg);
  }
}

std::string getHallConfigsString(const HallConfig* hall_configs, const size_t size) {
  std::string str;

  // Parse hall_configs and build a readable string
  for (size_t i = 0; i < size; i++) {
    if (str.length()) str.append(", ");
    if (hall_configs->active_low) str.append("!");
    switch (hall_configs->mode) {
      case HallMode::OFF: str.append("I"); break;
      case HallMode::LIFT_TILT: str.append("L"); break;
      case HallMode::STOP: str.append("S"); break;
      case HallMode::UNDEFINED: str.append("U"); break;
      default: break;
    }
    hall_configs++;
  }

  return str;
}

void publishLowLevelConfig(const uint8_t pkt_type) {
  if (!serial_port.isOpen() || !allow_send) return;

  // Prepare the pkt
  size_t size = sizeof(struct ll_high_level_config) + 3;  // +1 type, +2 crc
  uint8_t buf[size];

  // Send config and request a config answer
  buf[0] = pkt_type;

  // Copy our live config into the message (behind type)
  memcpy(&buf[1], &llhl_config, sizeof(struct ll_high_level_config));

  // Member access to buffer
  struct ll_high_level_config* buf_config = (struct ll_high_level_config*)&buf[1];

  // CRC
  crc.reset();
  crc.process_bytes(buf, sizeof(struct ll_high_level_config) + 1);  // + type
  buf[size - 1] = (crc.checksum() >> 8) & 0xFF;
  buf[size - 2] = crc.checksum() & 0xFF;

  // COBS
  size_t encoded_size = cobs.encode(buf, size, out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  // Send
  try {
    // Let's be verbose for easier follow-up
    ROS_INFO(
        "Send ll_high_level_config packet %#04x\n"
        "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
        "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
        "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
        "\t lift_period=%d, tilt_period=%d,\n"
        "\t shutdown_esc_max_pitch=%d,\n"
        "\t language=\"%.2s\", volume=%d\n"
        "\t hall_configs=\"%s\"",
        buf[0], (int)buf_config->options.dfp_is_5v, (int)buf_config->options.background_sounds,
        (int)buf_config->options.ignore_charging_current, buf_config->v_charge_cutoff, buf_config->i_charge_cutoff,
        buf_config->v_battery_cutoff, buf_config->v_battery_empty, buf_config->v_battery_full, buf_config->lift_period,
        buf_config->tilt_period, buf_config->shutdown_esc_max_pitch, buf_config->language, buf_config->volume,
        getHallConfigsString(buf_config->hall_configs, MAX_HALL_INPUTS).c_str());

    serial_port.write(out_buf, encoded_size);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Error writing to serial port");
  }
}

/**
 * @brief A simple config tracker (struct-class) for managing lost response packets as well as simpler handling of
 * LowLevel reboots or flash period.
 */
struct {
  ros::Time last_config_req;    // Time when last config request was sent
  unsigned int tries_left = 0;  // Remaining request tries before giving up

  void ackResponse() {
    // Call this on receive of a response packet to stop monitoring
    tries_left = 0;
  };

  void setDirty() {
    // Call this for indicating that config packet need to be resend, i.e. die to LL-reboot
    tries_left = 5;
  };

  void check() {
    if (!tries_left ||                                            // No request tries left (probably old LL-FW)
        !serial_port.isOpen() || !allow_send ||                   // Serial not ready
        ros::Time::now() - last_config_req < ros::Duration(0.5))  // Timeout waiting for response not reached
      return;
    publishLowLevelConfig(PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);
    last_config_req = ros::Time::now();
    tries_left--;
    ROS_WARN_STREAM_COND(
        !tries_left, "Didn't received a config packet from LowLevel in time. Is your LowLevel firmware up-to-date?");
  };
} configTracker;

void publishActuatorsTimerTask(const ros::TimerEvent& timer_event) {
  publishActuators();
  publishStatus();
  configTracker.check();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest& req, mower_msgs::MowerControlSrvResponse& res) {
  const bool was_enabled = isMowCommandEnabled(mow_control_mode, target_speed_mow, target_current_mow, target_rpm_mow);

  if (req.mow_enabled && !is_emergency()) {
    if (mow_control_mode == MowControlMode::RPM) {
      target_speed_mow = 0;
      target_current_mow = 0;
      target_rpm_mow = req.mow_direction ? configured_target_rpm_mow : -configured_target_rpm_mow;
    } else if (mow_control_mode == MowControlMode::CURRENT) {
      target_speed_mow = 0;
      target_current_mow = req.mow_direction ? configured_target_current_mow : -configured_target_current_mow;
      target_rpm_mow = 0;
    } else {
      target_speed_mow = req.mow_direction ? configured_target_duty_mow : -configured_target_duty_mow;
      target_current_mow = 0;
      target_rpm_mow = 0;
    }
  } else {
    target_speed_mow = 0;
    target_current_mow = 0;
    target_rpm_mow = 0;
  }

  const bool is_enabled = isMowCommandEnabled(mow_control_mode, target_speed_mow, target_current_mow, target_rpm_mow);
  if (is_enabled && !was_enabled) {
    mow_started_from_rest = std::abs(last_observed_mower_motor_rpm) <= configured_startup_boost_rpm_threshold;
    mow_enable_started_at = ros::Time::now();
    mow_disable_started_at = ros::Time(0.0);
  } else if (!is_enabled && was_enabled) {
    mow_started_from_rest = false;
    mow_enable_started_at = ros::Time(0.0);
    mow_disable_started_at = ros::Time::now();
  }

  ROS_INFO_STREAM("Setting mow enabled to "
                  << (mow_control_mode == MowControlMode::RPM
                          ? target_rpm_mow
                          : (mow_control_mode == MowControlMode::CURRENT ? target_current_mow : target_speed_mow))
                  << " in "
                  << (mow_control_mode == MowControlMode::RPM
                          ? "rpm"
                          : (mow_control_mode == MowControlMode::CURRENT ? "current" : "duty"))
                  << " mode");
  publishEscActuators(getActuatorCommands());
  return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest& req, mower_msgs::EmergencyStopSrvResponse& res) {
  if (req.emergency) {
    ROS_ERROR_STREAM("Setting emergency!!");
    ll_clear_emergency = false;
    ll_clear_requested_at = ros::Time(0.0);
  } else {
    ll_clear_emergency = true;
    ll_clear_requested_at = ros::Time::now();
    if ((last_ll_status.emergency_bitmask & ~LL_EMERGENCY_BIT_LATCH) == 0) {
      last_ll_status.emergency_bitmask = 0;
      active_low_level_emergency = 0;
      emergency_low_level = false;
    }
  }
  // Set the high level emergency instantly. Low level value will be set on next update.
  emergency_high_level = req.emergency;
  publishActuators();
  return true;
}

void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
  last_high_level_status = *msg;
  struct ll_high_level_state hl_state = {.type = PACKET_ID_LL_HIGH_LEVEL_STATE,
                                         .current_mode = msg->state,
                                         .gps_quality = static_cast<uint8_t>(msg->gps_quality_percent * 100.0)};

  crc.reset();
  crc.process_bytes(&hl_state, sizeof(struct ll_high_level_state) - 2);
  hl_state.crc = crc.checksum();

  size_t encoded_size = cobs.encode((uint8_t*)&hl_state, sizeof(struct ll_high_level_state), out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  if (serial_port.isOpen() && allow_send) {
    try {
      serial_port.write(out_buf, encoded_size);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Error writing to serial port");
    }
  }
}

void velReceived(const geometry_msgs::Twist::ConstPtr& msg) {
  // TODO: update this to rad/s values and implement xESC speed control
  last_cmd_vel = ros::Time::now();
  speed_r = msg->linear.x + 0.5 * wheel_distance_m * msg->angular.z;
  speed_l = msg->linear.x - 0.5 * wheel_distance_m * msg->angular.z;

  if (speed_l >= 1.0) {
    speed_l = 1.0;
  } else if (speed_l <= -1.0) {
    speed_l = -1.0;
  }
  if (speed_r >= 1.0) {
    speed_r = 1.0;
  } else if (speed_r <= -1.0) {
    speed_r = -1.0;
  }

  publishEscActuators(getActuatorCommands());
}

void handleLowLevelUIEvent(struct ll_ui_event* ui_event) {
  ROS_INFO_STREAM("Got UI button with code:" << +ui_event->button_id << " and duration: " << +ui_event->press_duration);

  mower_msgs::HighLevelControlSrv srv;

  switch (ui_event->button_id) {
    case 2:
      // Home
      srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME;
      break;
    case 3:
      // Play
      srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_START;
      break;
    case 4:
      // S1
      srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S1;
      break;
    case 5:
      // S2
      if (ui_event->press_duration == 2) {
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS;
      } else {
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S2;
      }
      break;
    case 6:
      // LOCK
      if (ui_event->press_duration == 2) {
        // very long press on lock
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY;
      }
      break;
    default:
      // Return, don't call the service.
      return;
  }

  if (!highLevelClient.call(srv)) {
    ROS_ERROR_STREAM("Error calling high level control service");
  }
}

/**
 * @brief getNewSetChanged return t_new and checks if the value changed in comparison to t_cur.
 * t_new can't be a reference because the same function is also used for packed structures.
 * @param t_cur source value
 * @param t_new reference
 * @return &bool get set to true if t_cur and t_new differ, otherwise changed doesn't get touched
 */
template <typename T>
T getNewSetChanged(const T t_cur, const T t_new, bool& changed) {
  bool equal;
  if (std::is_floating_point<T>::value)
    equal = fabs(t_cur - t_new) < std::numeric_limits<T>::epsilon();
  else
    equal = t_cur == t_new;

  if (!equal) changed = true;

  // ROS_INFO_STREAM("DEBUG mower_comms comp. member: cur " << t_cur << " ?= " << t_new << " == equal " << equal << ",
  // changed " << changed);

  return t_new;
}

/**
 * Handle config packet on receive from LL (LL->HL config packet response)
 */
void handleLowLevelConfig(const uint8_t* buffer, const size_t size) {
  // This is a flexible length packet where the size may vary when ll_high_level_config struct got enhanced only on one
  // side. If payload size is larger than our struct size, ensure that we only copy those we know of = our struct size.
  // If payload size is smaller than our struct size, copy only the payload we got, but ensure that the unsent member(s)
  // have reasonable defaults.
  size_t payload_size = std::min(sizeof(ll_high_level_config), size - 3);  // exclude type & crc

  // Copy payload to separated ll_config
  memcpy(&llhl_config, buffer + 1, payload_size);

  // Let's be verbose for easier follow-up
  ROS_INFO(
      "Received ll_high_level_config packet %#04x\n"
      "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
      "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
      "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
      "\t lift_period=%d, tilt_period=%d,\n"
      "\t shutdown_esc_max_pitch=%d,\n"
      "\t language=\"%.2s\", volume=%d\n"
      "\t hall_configs=\"%s\"",
      *buffer, (int)llhl_config.options.dfp_is_5v, (int)llhl_config.options.background_sounds,
      (int)llhl_config.options.ignore_charging_current, llhl_config.v_charge_cutoff, llhl_config.i_charge_cutoff,
      llhl_config.v_battery_cutoff, llhl_config.v_battery_empty, llhl_config.v_battery_full, llhl_config.lift_period,
      llhl_config.tilt_period, llhl_config.shutdown_esc_max_pitch, llhl_config.language, llhl_config.volume,
      getHallConfigsString(llhl_config.hall_configs, MAX_HALL_INPUTS).c_str());

  // Inform config packet tracker about the response
  configTracker.ackResponse();

  // Copy received config values from LL to mower_logic's related dynamic reconfigure variables and
  // decide if mower_logic's dynamic reconfigure need to be updated with probably changed values
  bool logic_config_dirty = false;
  bool power_config_dirty = false;
  // clang-format off
  power_config.charge_critical_high_voltage = getNewSetChanged<double>(power_config.charge_critical_high_voltage, llhl_config.v_charge_cutoff, power_config_dirty);
  power_config.charge_critical_high_current = getNewSetChanged<double>(power_config.charge_critical_high_current, llhl_config.i_charge_cutoff, power_config_dirty);
  power_config.battery_critical_high_voltage = getNewSetChanged<double>(power_config.battery_critical_high_voltage, llhl_config.v_battery_cutoff, power_config_dirty);
  power_config.battery_empty_voltage = getNewSetChanged<double>(power_config.battery_empty_voltage, llhl_config.v_battery_empty, power_config_dirty);
  power_config.battery_full_voltage = getNewSetChanged<double>(power_config.battery_full_voltage, llhl_config.v_battery_full, power_config_dirty);
  mower_logic_config.cu_rain_threshold = getNewSetChanged<int>(mower_logic_config.cu_rain_threshold, llhl_config.rain_threshold, logic_config_dirty);
  mower_logic_config.emergency_lift_period = getNewSetChanged<int>(mower_logic_config.emergency_lift_period, llhl_config.lift_period, logic_config_dirty);
  mower_logic_config.emergency_tilt_period = getNewSetChanged<int>(mower_logic_config.emergency_tilt_period, llhl_config.tilt_period, logic_config_dirty);
  mower_logic_config.shutdown_esc_max_pitch = getNewSetChanged<int>(mower_logic_config.shutdown_esc_max_pitch, llhl_config.shutdown_esc_max_pitch, logic_config_dirty);
  // clang-format on

  if (logic_config_dirty && ros::service::exists(kLogicReconfigureService, false)) {
    reconfigClient->setConfiguration(mower_logic_config);
  }
  if (power_config_dirty && powerReconfigClient != nullptr && ros::service::exists(kPowerReconfigureService, false)) {
    powerReconfigClient->setConfiguration(power_config);
  }
}

void handleLowLevelStatus(struct ll_status* status) {
  static ros::Time last_ll_status_update(ros::Time::now());

  std::unique_lock<std::mutex> lk(ll_status_mutex);
  last_ll_status = *status;

  // LL status get send at 100ms cycle. If we miss 10 packets, we can assume that it got restarted or flashed with a
  // new FW. In either case we should ensure that it has the right config and update/re-align with us.
  if (ros::Time::now() - last_ll_status_update > ros::Duration(1.0)) configTracker.setDirty();
  last_ll_status_update = ros::Time::now();
}

void handleLowLevelIMU(struct ll_imu* imu) {
  mower_msgs::ImuRaw imu_msg;
  imu_msg.dt = imu->dt_millis;
  imu_msg.ax = imu->acceleration_mss[0];
  imu_msg.ay = imu->acceleration_mss[1];
  imu_msg.az = imu->acceleration_mss[2];
  imu_msg.gx = imu->gyro_rads[0];
  imu_msg.gy = imu->gyro_rads[1];
  imu_msg.gz = imu->gyro_rads[2];
  imu_msg.mx = imu->mag_uT[0];
  imu_msg.my = imu->mag_uT[1];
  imu_msg.mz = imu->mag_uT[2];

  sensor_imu_msg.header.stamp = ros::Time::now();
  sensor_imu_msg.header.seq++;
  sensor_imu_msg.header.frame_id = "base_link";
  sensor_imu_msg.linear_acceleration.x = imu_msg.ax;
  sensor_imu_msg.linear_acceleration.y = imu_msg.ay;
  sensor_imu_msg.linear_acceleration.z = imu_msg.az;
  sensor_imu_msg.angular_velocity.x = imu_msg.gx;
  sensor_imu_msg.angular_velocity.y = imu_msg.gy;
  sensor_imu_msg.angular_velocity.z = imu_msg.gz;

  sensor_imu_pub.publish(sensor_imu_msg);
}

void checkAndSendConfig() {
  // Copy changed mower_config's values to the related llhl_config values and
  // decide if LL need to be informed with a new config packet
  bool dirty = false;

  // clang-format off
  llhl_config.rain_threshold = getNewSetChanged<int>(llhl_config.rain_threshold, mower_logic_config.cu_rain_threshold, dirty);
  llhl_config.v_charge_cutoff = getNewSetChanged<double>(llhl_config.v_charge_cutoff, power_config.charge_critical_high_voltage, dirty);
  llhl_config.i_charge_cutoff = getNewSetChanged<double>(llhl_config.i_charge_cutoff, power_config.charge_critical_high_current, dirty);
  llhl_config.v_battery_cutoff = getNewSetChanged<double>(llhl_config.v_battery_cutoff, power_config.battery_critical_high_voltage, dirty);
  llhl_config.v_battery_empty = getNewSetChanged<double>(llhl_config.v_battery_empty, power_config.battery_empty_voltage, dirty);
  llhl_config.v_battery_full = getNewSetChanged<double>(llhl_config.v_battery_full, power_config.battery_full_voltage, dirty);
  llhl_config.lift_period = getNewSetChanged<int>(llhl_config.lift_period, mower_logic_config.emergency_lift_period, dirty);
  llhl_config.tilt_period = getNewSetChanged<int>(llhl_config.tilt_period, mower_logic_config.emergency_tilt_period, dirty);
  llhl_config.shutdown_esc_max_pitch = getNewSetChanged<int>(llhl_config.shutdown_esc_max_pitch, mower_logic_config.shutdown_esc_max_pitch, dirty);
  // clang-format on

  // Parse emergency_input_config and set hall_configs
  char* token = strtok(strdup(mower_logic_config.emergency_input_config.c_str()), ",");
  bool low_active;
  unsigned int hall_idx = 0;
  while (token != NULL) {
    low_active = false;
    while (*token != 0) {
      switch (std::toupper(*token)) {
        case '!': low_active = true; break;
        case 'I': llhl_config.hall_configs[hall_idx] = {HallMode::OFF, low_active}; break;
        case 'L': llhl_config.hall_configs[hall_idx] = {HallMode::LIFT_TILT, low_active}; break;
        case 'S': llhl_config.hall_configs[hall_idx] = {HallMode::STOP, low_active}; break;
        case 'U': llhl_config.hall_configs[hall_idx] = {HallMode::UNDEFINED, low_active}; break;
        default: break;
      }
      token++;
    }
    token = strtok(NULL, ",");
    hall_idx++;
  }

  if (dirty) configTracker.setDirty();
}

void reconfigCB(const mower_logic::MowerLogicConfig& config) {
  ROS_INFO_STREAM("mower_comms received new mower_logic config");

  mower_logic_config = config;

  checkAndSendConfig();
}

void powerReconfigCB(const ll::PowerConfig& config) {
  ROS_INFO_STREAM("mower_comms received new mower_logic config");

  power_config = config;

  checkAndSendConfig();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_comms_v1");

  sensor_imu_msg.header.seq = 0;

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");
  ros::NodeHandle leftParamNh("~/services/diff_drive/left_xesc");
  ros::NodeHandle mowerParamNh("~/services/diff_drive/mower_xesc");
  ros::NodeHandle rightParamNh("~/services/diff_drive/right_xesc");
  ros::NodeHandle mowerLogicParamNh("/mower_logic");
  ros::NodeHandle powerParamNh("~/services/power");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");

  mower_logic_config = mower_logic::MowerLogicConfig::__getDefault__();
  mower_logic_config.__fromServer__(mowerLogicParamNh);
  reconfigClient = new dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>("/mower_logic", reconfigCB);

  power_config = ll::PowerConfig::__getDefault__();
  power_config.__fromServer__(powerParamNh);
  if (ros::service::exists(kPowerReconfigureService, false)) {
    powerReconfigClient = new dynamic_reconfigure::Client<ll::PowerConfig>("/ll/services/power", powerReconfigCB);
  } else {
    powerReconfigClient = nullptr;
  }

  std::string ll_serial_port_name;
  if (!paramNh.getParam("ll_serial_port", ll_serial_port_name)) {
    ROS_ERROR_STREAM("Error getting low level serial port parameter. Quitting.");
    return 1;
  }

  paramNh.getParam("services/diff_drive/ticks_per_m", wheel_ticks_per_m);
  paramNh.getParam("services/diff_drive/wheel_distance_m", wheel_distance_m);
  leftParamNh.param("invert_direction", left_xesc_invert_direction, false);
  rightParamNh.param("invert_direction", right_xesc_invert_direction, true);
  {
    const auto mower_mode = mowerParamNh.param<std::string>("control_mode", "duty");
    if (mower_mode == "rpm") {
      mow_control_mode = MowControlMode::RPM;
    } else if (mower_mode == "current") {
      mow_control_mode = MowControlMode::CURRENT;
    } else if (mower_mode == "duty") {
      mow_control_mode = MowControlMode::DUTY;
    } else {
      ROS_WARN_STREAM("Unknown mower_xesc control_mode '" << mower_mode << "', falling back to duty.");
      mow_control_mode = MowControlMode::DUTY;
    }
  }
  mowerParamNh.param("target_duty_cycle", configured_target_duty_mow, 1.0f);
  mowerParamNh.param("target_motor_current", configured_target_current_mow, 15.0f);
  mowerParamNh.param("target_motor_rpm", configured_target_rpm_mow, 3200.0f);
  mowerParamNh.param("startup_boost_duty_cycle", configured_startup_boost_duty_mow, 0.0f);
  mowerParamNh.param("startup_boost_current", configured_startup_boost_current_mow, 0.0f);
  mowerParamNh.param("stop_brake_current", configured_stop_brake_current_mow, 0.0f);
  mowerParamNh.param("target_duty_ramp_seconds", configured_target_duty_ramp_seconds, 0.8);
  paramNh.param("services/diff_drive/drive_command_scale", configured_drive_command_scale, 1.0);
  paramNh.param("services/diff_drive/mowing_drive_command_scale", configured_mowing_drive_command_scale,
                configured_drive_command_scale);
  paramNh.param("services/diff_drive/ignore_low_level_emergency_inputs", configured_ignore_low_level_emergency_inputs,
                false);
  paramNh.param("services/diff_drive/drive_command_ramp_up_seconds", configured_drive_command_ramp_up_seconds, 0.0);
  paramNh.param("services/diff_drive/drive_command_ramp_down_seconds", configured_drive_command_ramp_down_seconds, 0.0);
  mowerParamNh.param("startup_boost_duration_seconds", configured_startup_boost_duration_seconds, 0.0);
  mowerParamNh.param("startup_boost_rpm_threshold", configured_startup_boost_rpm_threshold, 0.0);
  mowerParamNh.param("startup_boost_release_rpm_threshold", configured_startup_boost_release_rpm_threshold,
                     configured_startup_boost_rpm_threshold);
  mowerParamNh.param("stop_brake_duration_seconds", configured_stop_brake_duration_seconds, 0.0);

  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);
  ROS_INFO_STREAM("Drive command scale: " << configured_drive_command_scale);
  ROS_INFO_STREAM("Mowing drive command scale: " << configured_mowing_drive_command_scale);
  ROS_INFO_STREAM("Ignore low-level emergency inputs: "
                  << (configured_ignore_low_level_emergency_inputs ? "true" : "false"));
  ROS_INFO_STREAM("Drive command ramp up seconds: " << configured_drive_command_ramp_up_seconds);
  ROS_INFO_STREAM("Drive command ramp down seconds: " << configured_drive_command_ramp_down_seconds);
  ROS_INFO_STREAM("Left drive ESC invert direction: " << (left_xesc_invert_direction ? "true" : "false"));
  ROS_INFO_STREAM("Right drive ESC invert direction: " << (right_xesc_invert_direction ? "true" : "false"));
  ROS_INFO_STREAM("Mower ESC control mode: "
                  << (mow_control_mode == MowControlMode::RPM
                          ? "rpm"
                          : (mow_control_mode == MowControlMode::CURRENT ? "current" : "duty")));
  if (mow_control_mode == MowControlMode::DUTY) {
    ROS_INFO_STREAM("Mower ESC target duty cycle: " << configured_target_duty_mow);
    ROS_INFO_STREAM("Mower ESC duty ramp seconds: " << configured_target_duty_ramp_seconds);
    ROS_INFO_STREAM("Mower ESC startup boost duty cycle: " << configured_startup_boost_duty_mow);
    ROS_INFO_STREAM("Mower ESC startup boost current: " << configured_startup_boost_current_mow);
    ROS_INFO_STREAM("Mower ESC startup boost duration seconds: " << configured_startup_boost_duration_seconds);
    ROS_INFO_STREAM("Mower ESC startup boost rpm threshold: " << configured_startup_boost_rpm_threshold);
    ROS_INFO_STREAM("Mower ESC startup boost release rpm threshold: " << configured_startup_boost_release_rpm_threshold);
  } else if (mow_control_mode == MowControlMode::CURRENT) {
    ROS_INFO_STREAM("Mower ESC target motor current: " << configured_target_current_mow);
  } else if (mow_control_mode == MowControlMode::RPM) {
    ROS_INFO_STREAM("Mower ESC target motor rpm: " << configured_target_rpm_mow);
  }
  ROS_INFO_STREAM("Mower ESC stop brake current: " << configured_stop_brake_current_mow);
  ROS_INFO_STREAM("Mower ESC stop brake duration seconds: " << configured_stop_brake_duration_seconds);

  speed_l = speed_r = applied_speed_l = applied_speed_r = target_speed_mow = target_current_mow = target_rpm_mow = 0;

  // Some generic settings from param server (non- dynamic)
  llhl_config.options.ignore_charging_current =
      paramNh.param("/mower_logic/ignore_charging_current", false) ? OptionState::ON : OptionState::OFF;
  llhl_config.options.dfp_is_5v = paramNh.param("services/sound/dfp_is_5v", false) ? OptionState::ON : OptionState::OFF;
  llhl_config.volume = paramNh.param("services/sound/volume", -1);
  llhl_config.options.background_sounds =
      paramNh.param("services/sound/background_sounds", false) ? OptionState::ON : OptionState::OFF;
  // ISO-639-1 (2 char) language code
  strncpy(llhl_config.language, paramNh.param<std::string>("services/sound/language", "en").c_str(), 2);

  // Setup XESC interfaces
  if (mowerParamNh.hasParam("xesc_type")) {
    mow_xesc_interface = new xesc_driver::XescDriver(n, mowerParamNh);
  } else {
    mow_xesc_interface = nullptr;
  }

  left_xesc_interface = new xesc_driver::XescDriver(n, leftParamNh);
  right_xesc_interface = new xesc_driver::XescDriver(n, rightParamNh);

  emergency_pub = n.advertise<mower_msgs::Emergency>("ll/emergency", 1);

  // Diff drive service
  actual_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ll/diff_drive/measured_twist", 1);
  status_left_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/left_esc_status", 1);
  status_right_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/right_esc_status", 1);

  status_pub = n.advertise<mower_msgs::Status>("ll/mower_status", 1);
  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("ll/imu/data_raw", 1);
  power_pub = n.advertise<mower_msgs::Power>("ll/power", 1);

  ros::ServiceServer mow_service = n.advertiseService("ll/_service/mow_enabled", setMowEnabled);
  ros::ServiceServer emergency_service = n.advertiseService("ll/_service/emergency", setEmergencyStop);
  ros::Subscriber cmd_vel_sub = n.subscribe("ll/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);

  size_t buflen = 1000;
  uint8_t buffer[buflen];
  uint8_t buffer_decoded[buflen];
  size_t read = 0;
  // don't change, we need to wait for arduino to boot before actually sending stuff
  ros::Duration retryDelay(5, 0);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok()) {
    if (!serial_port.isOpen()) {
      ROS_INFO_STREAM("connecting serial interface: " << ll_serial_port_name);
      allow_send = false;
      try {
        serial_port.setPort(ll_serial_port_name);
        serial_port.setBaudrate(115200);
        auto to = serial::Timeout::simpleTimeout(100);
        serial_port.setTimeout(to);
        serial_port.open();

        // wait for controller to boot
        retryDelay.sleep();
        // this will only be set if no error was set

        allow_send = true;
      } catch (std::exception& e) {
        retryDelay.sleep();
        ROS_ERROR_STREAM("Error during reconnect.");
      }
    }
    size_t bytes_read = 0;
    try {
      bytes_read = serial_port.read(buffer + read, 1);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Error reading serial_port. Closing Connection.");
      serial_port.close();
      retryDelay.sleep();
    }
    if (read + bytes_read >= buflen) {
      read = 0;
      bytes_read = 0;
      ROS_ERROR_STREAM("Prevented buffer overflow. There is a problem with the serial comms.");
    }
    if (bytes_read) {
      if (buffer[read] == 0) {
        // end of packet found
        size_t data_size = cobs.decode(buffer, read, buffer_decoded);

        // first, check the CRC
        if (data_size < 3) {
          // We don't even have one byte of data
          // (type + crc = 3 bytes already)
          ROS_INFO_STREAM("Got empty packet from Low Level Board");
        } else {
          // We have at least 1 byte of data, check the CRC
          crc.reset();
          // We start at the second byte (ignore the type) and process (data_size- byte for type - 2 bytes for CRC)
          // bytes.
          crc.process_bytes(buffer_decoded, data_size - 2);
          uint16_t checksum = crc.checksum();
          uint16_t received_checksum = *(uint16_t*)(buffer_decoded + data_size - 2);
          if (checksum == received_checksum) {
            // Packet checksum is OK, process it
            switch (buffer_decoded[0]) {
              case PACKET_ID_LL_STATUS:
                if (data_size == sizeof(struct ll_status)) {
                  handleLowLevelStatus((struct ll_status*)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was STATUS");
                }
                break;
              case PACKET_ID_LL_IMU:
                if (data_size == sizeof(struct ll_imu)) {
                  handleLowLevelIMU((struct ll_imu*)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was IMU");
                }
                break;
              case PACKET_ID_LL_UI_EVENT:
                if (data_size == sizeof(struct ll_ui_event)) {
                  handleLowLevelUIEvent((struct ll_ui_event*)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was UI_EVENT");
                }
                break;
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ:
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP: handleLowLevelConfig(buffer_decoded, data_size); break;
              default: ROS_INFO_STREAM("Got unknown packet from Low Level Board"); break;
            }
          } else {
            ROS_INFO_STREAM("Got invalid checksum from Low Level Board");
          }
        }

        read = 0;
      } else {
        read += bytes_read;
      }
    }
  }

  spinner.stop();

  if (mow_xesc_interface) {
    if (mow_control_mode == MowControlMode::RPM) {
      mow_xesc_interface->setSpeed(0.0);
    } else if (mow_control_mode == MowControlMode::CURRENT) {
      mow_xesc_interface->setCurrent(0.0);
    } else {
      mow_xesc_interface->setDutyCycle(0.0);
    }
    mow_xesc_interface->stop();
  }
  left_xesc_interface->setDutyCycle(0.0);
  right_xesc_interface->setDutyCycle(0.0);
  left_xesc_interface->stop();
  right_xesc_interface->stop();

  if (mow_xesc_interface) {
    delete mow_xesc_interface;
  }
  delete left_xesc_interface;
  delete right_xesc_interface;

  return 0;
}
