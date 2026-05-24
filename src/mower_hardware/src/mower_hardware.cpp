#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/HwPower.h>
#include <mower_msgs/HwStatus.h>
#include <mower_msgs/MowerControlSrv.h>
#include <ros/ros.h>
#include <xesc_driver/xesc_driver.h>
#include <xesc_msgs/XescStateStamped.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

namespace {

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

struct BatteryTelemetry {
  float voltage = 0.0f;
  bool valid = false;
  float percentage = 0.0f;
  std::string source = "none";
  bool mismatch = false;
  std::string warning;
};

ros::Publisher status_pub;
ros::Publisher power_pub;
ros::Publisher emergency_pub;
ros::Publisher actual_twist_pub;
ros::Publisher status_left_esc_pub;
ros::Publisher status_right_esc_pub;

std::unique_ptr<xesc_driver::XescDriver> mow_xesc_interface;
std::unique_ptr<xesc_driver::XescDriver> left_xesc_interface;
std::unique_ptr<xesc_driver::XescDriver> right_xesc_interface;

bool emergency_high_level = true;
bool emergency_latched = true;
std::string emergency_reason = "Hardware bridge starting";

float speed_l = 0.0f;
float speed_r = 0.0f;
float applied_speed_l = 0.0f;
float applied_speed_r = 0.0f;
float target_speed_mow = 0.0f;
float target_current_mow = 0.0f;
float target_rpm_mow = 0.0f;
float configured_target_duty_mow = 1.0f;
float configured_target_current_mow = 15.0f;
float configured_target_rpm_mow = 3200.0f;
float configured_stop_brake_current_mow = 0.0f;
float configured_startup_boost_duty_mow = 0.0f;
float configured_startup_boost_current_mow = 0.0f;
double configured_target_duty_ramp_seconds = 0.8;
double configured_drive_command_scale = 1.0;
double configured_mowing_drive_command_scale = 1.0;
double configured_drive_command_ramp_up_seconds = 0.0;
double configured_drive_command_ramp_down_seconds = 0.0;
double configured_startup_boost_duration_seconds = 0.0;
double configured_startup_boost_rpm_threshold = 0.0;
double configured_startup_boost_release_rpm_threshold = 0.0;
double configured_stop_brake_duration_seconds = 0.0;
double configured_battery_full_voltage = 58.4;
double configured_battery_empty_voltage = 45.0;
double configured_voltage_mismatch_warn_v = 1.0;
MowControlMode mow_control_mode = MowControlMode::DUTY;
ros::Time mow_enable_started_at(0.0);
ros::Time mow_disable_started_at(0.0);
double last_observed_mower_motor_rpm = 0.0;
bool mow_started_from_rest = false;

double wheel_ticks_per_m = 0.0;
double wheel_distance_m = 0.0;
bool left_xesc_invert_direction = false;
bool right_xesc_invert_direction = true;
ros::Time last_cmd_vel(0.0);
ros::Time last_drive_command_update(0.0);
mower_msgs::HighLevelStatus last_high_level_status;

bool has_ticks = false;
uint32_t last_ticks_l = 0;
uint32_t last_ticks_r = 0;
ros::Time last_ticks_stamp{};
geometry_msgs::TwistStamped measured_twist_msg{};

bool isMowCommandEnabled(MowControlMode mode, float target_duty, float target_current, float target_rpm) {
  switch (mode) {
    case MowControlMode::RPM: return target_rpm != 0.0f;
    case MowControlMode::CURRENT: return target_current != 0.0f;
    case MowControlMode::DUTY:
    default: return target_duty != 0.0f;
  }
}

bool isEmergency() {
  return emergency_latched || emergency_high_level;
}

float clampDuty(float duty) {
  return std::max(-1.0f, std::min(1.0f, duty));
}

float moveToward(float current, float target, float max_delta) {
  if (max_delta <= 0.0f) return target;
  if (target > current) return std::min(target, current + max_delta);
  return std::max(target, current - max_delta);
}

bool isEscConnected(const xesc_msgs::XescStateStamped& status) {
  return status.state.connection_state == xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED ||
         status.state.connection_state == xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW;
}

bool hasValidVoltage(const xesc_msgs::XescStateStamped& status) {
  return isEscConnected(status) && status.state.voltage_input > 1.0;
}

float getDriveCommandScale() {
  const bool mowing_state = last_high_level_status.state_name == "MOWING";
  const double configured_scale = mowing_state ? configured_mowing_drive_command_scale : configured_drive_command_scale;
  return clampDuty(static_cast<float>(configured_scale));
}

void updateDriveDutyCommands(float target_left, float target_right, const ros::Time& now, bool force_stop) {
  target_left = clampDuty(target_left);
  target_right = clampDuty(target_right);

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
    if (ramp_seconds <= 0.0) return target;
    return moveToward(current, target, static_cast<float>(dt / ramp_seconds));
  };

  applied_speed_l = ramp_channel(applied_speed_l, target_left);
  applied_speed_r = ramp_channel(applied_speed_r, target_right);
}

ActuatorCommands getActuatorCommands() {
  ActuatorCommands commands;
  const bool drive_force_stop = isEmergency() || (ros::Time::now() - last_cmd_vel > ros::Duration(1.0));
  const float drive_scale = getDriveCommandScale();
  updateDriveDutyCommands(speed_l * drive_scale, speed_r * drive_scale, ros::Time::now(), drive_force_stop);

  commands.left_duty = applied_speed_l;
  commands.right_duty = applied_speed_r;
  commands.mow_duty = target_speed_mow;
  commands.mow_current = target_current_mow;
  commands.mow_rpm = target_rpm_mow;

  if (isEmergency()) {
    commands.left_duty = 0.0f;
    commands.right_duty = 0.0f;
    commands.mow_duty = 0.0f;
    commands.mow_current = 0.0f;
    commands.mow_rpm = 0.0f;
  }
  if (ros::Time::now() - last_cmd_vel > ros::Duration(1.0)) {
    commands.left_duty = 0.0f;
    commands.right_duty = 0.0f;
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
    commands.mow_duty = std::copysign(
        ramp_start_abs + ((target_mow_duty_abs - ramp_start_abs) * static_cast<float>(ramp_progress)), target_mow_duty);
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
  left_xesc_interface->setDutyCycle(left_xesc_invert_direction ? -commands.left_duty : commands.left_duty);
  right_xesc_interface->setDutyCycle(right_xesc_invert_direction ? -commands.right_duty : commands.right_duty);

  if (!mow_xesc_interface) return;
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

mower_msgs::ESCStatus toEscStatus(const xesc_msgs::XescStateStamped& vesc_status) {
  mower_msgs::ESCStatus ros_esc_status{};
  if (!isEscConnected(vesc_status)) {
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (vesc_status.state.fault_code) {
    ROS_ERROR_STREAM_THROTTLE(1, "Motor controller fault code: " << vesc_status.state.fault_code);
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
  } else {
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
  }
  ros_esc_status.tacho = vesc_status.state.tacho;
  ros_esc_status.rpm = vesc_status.state.rpm;
  ros_esc_status.current = vesc_status.state.current_input;
  ros_esc_status.temperature_motor = vesc_status.state.temperature_motor;
  ros_esc_status.temperature_pcb = vesc_status.state.temperature_pcb;
  return ros_esc_status;
}

BatteryTelemetry buildBatteryTelemetry(const xesc_msgs::XescStateStamped& left_status,
                                       const xesc_msgs::XescStateStamped& right_status) {
  BatteryTelemetry telemetry;
  const bool left_valid = hasValidVoltage(left_status);
  const bool right_valid = hasValidVoltage(right_status);

  if (left_valid && right_valid) {
    telemetry.voltage = static_cast<float>((left_status.state.voltage_input + right_status.state.voltage_input) / 2.0);
    telemetry.valid = true;
    telemetry.source = "drive_pair";
    const double diff = std::abs(left_status.state.voltage_input - right_status.state.voltage_input);
    if (diff > configured_voltage_mismatch_warn_v) {
      telemetry.mismatch = true;
      std::ostringstream warning;
      warning << "Drive ESC voltage mismatch: left=" << left_status.state.voltage_input
              << "V right=" << right_status.state.voltage_input << "V";
      telemetry.warning = warning.str();
      ROS_WARN_STREAM_THROTTLE(5.0, telemetry.warning);
    }
  } else if (left_valid) {
    telemetry.voltage = static_cast<float>(left_status.state.voltage_input);
    telemetry.valid = true;
    telemetry.source = "left_drive";
    telemetry.warning = "Only left drive ESC battery voltage is available";
    ROS_WARN_STREAM_THROTTLE(5.0, telemetry.warning);
  } else if (right_valid) {
    telemetry.voltage = static_cast<float>(right_status.state.voltage_input);
    telemetry.valid = true;
    telemetry.source = "right_drive";
    telemetry.warning = "Only right drive ESC battery voltage is available";
    ROS_WARN_STREAM_THROTTLE(5.0, telemetry.warning);
  } else {
    telemetry.warning = "No drive ESC battery voltage is available";
    ROS_WARN_STREAM_THROTTLE(5.0, telemetry.warning);
  }

  const double denominator = configured_battery_full_voltage - configured_battery_empty_voltage;
  if (telemetry.valid && denominator > 0.0) {
    telemetry.percentage = static_cast<float>(
        std::max(0.0, std::min(1.0, (telemetry.voltage - configured_battery_empty_voltage) / denominator)));
  }
  return telemetry;
}

void publishEmergencyState() {
  mower_msgs::Emergency emergency_msg{};
  emergency_msg.stamp = ros::Time::now();
  emergency_latched = emergency_latched || emergency_high_level;
  emergency_msg.active_emergency = emergency_high_level;
  emergency_msg.latched_emergency = emergency_latched;
  emergency_msg.reason = emergency_reason;
  emergency_pub.publish(emergency_msg);
}

void publishTelemetry(const ros::TimerEvent&) {
  publishEscActuators(getActuatorCommands());

  xesc_msgs::XescStateStamped mow_status{}, left_status{}, right_status{};
  if (mow_xesc_interface) {
    mow_xesc_interface->getStatus(mow_status);
  } else {
    mow_status.state.connection_state = xesc_msgs::XescState::XESC_CONNECTION_STATE_DISCONNECTED;
  }
  left_xesc_interface->getStatus(left_status);
  right_xesc_interface->getStatus(right_status);

  const mower_msgs::ESCStatus left_esc_status = toEscStatus(left_status);
  const mower_msgs::ESCStatus right_esc_status = toEscStatus(right_status);
  status_left_esc_pub.publish(left_esc_status);
  status_right_esc_pub.publish(right_esc_status);

  const BatteryTelemetry battery = buildBatteryTelemetry(left_status, right_status);
  mower_msgs::HwPower power_msg{};
  power_msg.stamp = ros::Time::now();
  power_msg.v_battery = battery.voltage;
  power_msg.battery_voltage_valid = battery.valid;
  power_msg.battery_percentage = battery.percentage;
  power_msg.battery_source = battery.source;
  power_msg.left_drive_voltage = static_cast<float>(left_status.state.voltage_input);
  power_msg.left_drive_voltage_valid = hasValidVoltage(left_status);
  power_msg.right_drive_voltage = static_cast<float>(right_status.state.voltage_input);
  power_msg.right_drive_voltage_valid = hasValidVoltage(right_status);
  power_msg.mower_esc_voltage = static_cast<float>(mow_status.state.voltage_input);
  power_msg.mower_esc_voltage_valid = hasValidVoltage(mow_status);
  power_msg.drive_voltage_mismatch = battery.mismatch;
  power_msg.warning = battery.warning;
  power_pub.publish(power_msg);

  mower_msgs::HwStatus status_msg{};
  status_msg.stamp = power_msg.stamp;
  status_msg.hardware_status =
      (isEscConnected(left_status) && isEscConnected(right_status)) ? mower_msgs::HwStatus::HARDWARE_STATUS_OK
                                                                    : mower_msgs::HwStatus::HARDWARE_STATUS_INITIALIZING;
  status_msg.esc_power = isEscConnected(left_status) || isEscConnected(right_status) || isEscConnected(mow_status);
  status_msg.mow_enabled = isMowCommandEnabled(mow_control_mode, target_speed_mow, target_current_mow, target_rpm_mow);
  status_msg.mower_esc_current = static_cast<float>(mow_status.state.current_input);
  status_msg.mower_esc_status = toEscStatus(mow_status).status;
  status_msg.mower_motor_rpm = mow_status.state.rpm;
  last_observed_mower_motor_rpm = mow_status.state.rpm;
  status_msg.mower_esc_temperature = static_cast<float>(mow_status.state.temperature_pcb);
  status_msg.mower_motor_temperature = static_cast<float>(mow_status.state.temperature_motor);
  if (left_esc_status.status == mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED ||
      right_esc_status.status == mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED) {
    status_msg.warning = "At least one drive ESC is disconnected";
  } else if (!battery.warning.empty()) {
    status_msg.warning = battery.warning;
  }
  status_pub.publish(status_msg);

  if (!has_ticks) {
    last_ticks_stamp = status_msg.stamp;
    last_ticks_l = left_status.state.tacho_absolute;
    last_ticks_r = right_status.state.tacho_absolute;
    has_ticks = true;
  } else {
    const bool wheel_direction_l =
        (left_status.state.direction != left_xesc_invert_direction) && std::abs(left_status.state.duty_cycle) > 0;
    const bool wheel_direction_r =
        (right_status.state.direction != right_xesc_invert_direction) && std::abs(right_status.state.duty_cycle) > 0;
    const double dt = (status_msg.stamp - last_ticks_stamp).toSec();
    if (dt > 0.001 && wheel_ticks_per_m > 0.0) {
      double d_wheel_l = static_cast<double>(left_status.state.tacho_absolute - last_ticks_l) * (1.0 / wheel_ticks_per_m);
      double d_wheel_r = static_cast<double>(right_status.state.tacho_absolute - last_ticks_r) * (1.0 / wheel_ticks_per_m);
      if (wheel_direction_l) d_wheel_l *= -1.0;
      if (wheel_direction_r) d_wheel_r *= -1.0;

      const double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
      measured_twist_msg.header.frame_id = "base_link";
      measured_twist_msg.header.stamp = status_msg.stamp;
      measured_twist_msg.header.seq++;
      measured_twist_msg.twist.linear.x = d_ticks / dt;
      measured_twist_msg.twist.angular.z = -(d_wheel_l + d_wheel_r) / (2.0 * dt);
      actual_twist_pub.publish(measured_twist_msg);
    }
    last_ticks_stamp = status_msg.stamp;
    last_ticks_l = left_status.state.tacho_absolute;
    last_ticks_r = right_status.state.tacho_absolute;
  }

  publishEmergencyState();
}

void velReceived(const geometry_msgs::Twist::ConstPtr& msg) {
  last_cmd_vel = ros::Time::now();
  speed_r = msg->linear.x + 0.5 * wheel_distance_m * msg->angular.z;
  speed_l = msg->linear.x - 0.5 * wheel_distance_m * msg->angular.z;
  speed_l = clampDuty(speed_l);
  speed_r = clampDuty(speed_r);
  publishEscActuators(getActuatorCommands());
}

void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
  last_high_level_status = *msg;
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest& req, mower_msgs::MowerControlSrvResponse&) {
  const bool was_enabled = isMowCommandEnabled(mow_control_mode, target_speed_mow, target_current_mow, target_rpm_mow);

  if (req.mow_enabled && !isEmergency()) {
    if (mow_control_mode == MowControlMode::RPM) {
      target_speed_mow = 0.0f;
      target_current_mow = 0.0f;
      target_rpm_mow = req.mow_direction ? configured_target_rpm_mow : -configured_target_rpm_mow;
    } else if (mow_control_mode == MowControlMode::CURRENT) {
      target_speed_mow = 0.0f;
      target_current_mow = req.mow_direction ? configured_target_current_mow : -configured_target_current_mow;
      target_rpm_mow = 0.0f;
    } else {
      target_speed_mow = req.mow_direction ? configured_target_duty_mow : -configured_target_duty_mow;
      target_current_mow = 0.0f;
      target_rpm_mow = 0.0f;
    }
  } else {
    target_speed_mow = 0.0f;
    target_current_mow = 0.0f;
    target_rpm_mow = 0.0f;
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

  publishEscActuators(getActuatorCommands());
  return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest& req, mower_msgs::EmergencyStopSrvResponse&) {
  emergency_high_level = req.emergency;
  if (req.emergency) {
    emergency_latched = true;
    emergency_reason = "Software emergency";
  } else {
    emergency_latched = false;
    emergency_reason = "";
  }
  publishEscActuators(getActuatorCommands());
  publishEmergencyState();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_hardware");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");
  ros::NodeHandle leftParamNh("~/services/diff_drive/left_xesc");
  ros::NodeHandle mowerParamNh("~/services/diff_drive/mower_xesc");
  ros::NodeHandle rightParamNh("~/services/diff_drive/right_xesc");

  paramNh.param("services/diff_drive/ticks_per_m", wheel_ticks_per_m, 0.0);
  paramNh.param("services/diff_drive/wheel_distance_m", wheel_distance_m, 0.0);
  leftParamNh.param("invert_direction", left_xesc_invert_direction, false);
  rightParamNh.param("invert_direction", right_xesc_invert_direction, true);

  const auto mower_mode = mowerParamNh.param<std::string>("control_mode", "duty");
  if (mower_mode == "rpm") {
    mow_control_mode = MowControlMode::RPM;
  } else if (mower_mode == "current") {
    mow_control_mode = MowControlMode::CURRENT;
  } else {
    mow_control_mode = MowControlMode::DUTY;
  }

  mowerParamNh.param("target_duty_cycle", configured_target_duty_mow, 1.0f);
  mowerParamNh.param("target_motor_current", configured_target_current_mow, 15.0f);
  mowerParamNh.param("target_motor_rpm", configured_target_rpm_mow, 3200.0f);
  mowerParamNh.param("startup_boost_duty_cycle", configured_startup_boost_duty_mow, 0.0f);
  mowerParamNh.param("startup_boost_current", configured_startup_boost_current_mow, 0.0f);
  mowerParamNh.param("stop_brake_current", configured_stop_brake_current_mow, 0.0f);
  mowerParamNh.param("target_duty_ramp_seconds", configured_target_duty_ramp_seconds, 0.8);
  mowerParamNh.param("startup_boost_duration_seconds", configured_startup_boost_duration_seconds, 0.0);
  mowerParamNh.param("startup_boost_rpm_threshold", configured_startup_boost_rpm_threshold, 0.0);
  mowerParamNh.param("startup_boost_release_rpm_threshold", configured_startup_boost_release_rpm_threshold,
                     configured_startup_boost_rpm_threshold);
  mowerParamNh.param("stop_brake_duration_seconds", configured_stop_brake_duration_seconds, 0.0);
  paramNh.param("services/diff_drive/drive_command_scale", configured_drive_command_scale, 1.0);
  paramNh.param("services/diff_drive/mowing_drive_command_scale", configured_mowing_drive_command_scale,
                configured_drive_command_scale);
  paramNh.param("services/diff_drive/drive_command_ramp_up_seconds", configured_drive_command_ramp_up_seconds, 0.0);
  paramNh.param("services/diff_drive/drive_command_ramp_down_seconds", configured_drive_command_ramp_down_seconds, 0.0);
  paramNh.param("services/power/battery_full_voltage", configured_battery_full_voltage, 58.4);
  paramNh.param("services/power/battery_empty_voltage", configured_battery_empty_voltage, 45.0);
  paramNh.param("services/power/drive_voltage_mismatch_warn_v", configured_voltage_mismatch_warn_v, 1.0);

  ROS_INFO_STREAM("Mowrator direct hardware bridge active");
  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);
  ROS_INFO_STREAM("Battery full/empty [V]: " << configured_battery_full_voltage << " / "
                                             << configured_battery_empty_voltage);

  if (mowerParamNh.hasParam("xesc_type")) {
    mow_xesc_interface.reset(new xesc_driver::XescDriver(n, mowerParamNh));
  }
  left_xesc_interface.reset(new xesc_driver::XescDriver(n, leftParamNh));
  right_xesc_interface.reset(new xesc_driver::XescDriver(n, rightParamNh));

  emergency_pub = n.advertise<mower_msgs::Emergency>("hw/emergency", 1);
  status_pub = n.advertise<mower_msgs::HwStatus>("hw/status", 1);
  power_pub = n.advertise<mower_msgs::HwPower>("hw/power", 1);
  actual_twist_pub = n.advertise<geometry_msgs::TwistStamped>("hw/diff_drive/measured_twist", 1);
  status_left_esc_pub = n.advertise<mower_msgs::ESCStatus>("hw/diff_drive/left_esc_status", 1);
  status_right_esc_pub = n.advertise<mower_msgs::ESCStatus>("hw/diff_drive/right_esc_status", 1);

  ros::ServiceServer mow_service = n.advertiseService("hw/_service/mow_enabled", setMowEnabled);
  ros::ServiceServer emergency_service = n.advertiseService("hw/_service/emergency", setEmergencyStop);
  ros::Subscriber cmd_vel_sub = n.subscribe("hw/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishTelemetry);

  ros::spin();
  return 0;
}
