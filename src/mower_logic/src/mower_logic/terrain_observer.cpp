#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>

#include "ftc_local_planner/PID.h"
#include "mower_logic/terrain/TerrainMath.h"
#include "mower_logic/terrain/TerrainMemory.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/TerrainState.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"

namespace {
using mower_logic::terrain::ObserverTuning;
using mower_logic::terrain::SlipInputs;
using mower_logic::terrain::TerrainMemory;
using mower_logic::terrain::TerrainMode;
using mower_logic::terrain::TerrainProjection;
using mower_logic::terrain::TerrainSample;

class TerrainObserverNode {
 public:
  TerrainObserverNode() : nh_(), param_nh_("~"), memory_(param_nh_.param("terrain_memory_cell_size", 0.5)) {
    loadParams();
    loadMemory();

    terrain_pub_ = nh_.advertise<mower_msgs::TerrainState>("mower_logic/terrain_state", 10);
    clear_memory_srv_ = nh_.advertiseService("terrain_observer/clear_memory", &TerrainObserverNode::clearMemory, this);

    imu_sub_ = nh_.subscribe("terrain/imu/data", 10, &TerrainObserverNode::imuCallback, this);
    measured_twist_sub_ =
        nh_.subscribe("ll/diff_drive/measured_twist", 10, &TerrainObserverNode::measuredTwistCallback, this);
    command_twist_sub_ = nh_.subscribe("nav_vel", 10, &TerrainObserverNode::commandTwistCallback, this);
    pose_sub_ = nh_.subscribe("xbot_positioning/xb_pose", 10, &TerrainObserverNode::poseCallback, this);
    status_sub_ = nh_.subscribe("mower_logic/current_state", 10, &TerrainObserverNode::statusCallback, this);
    pid_sub_ = nh_.subscribe("move_base_flex/FTCPlanner/debug_pid", 10, &TerrainObserverNode::pidCallback, this);
    global_plan_sub_ = nh_.subscribe("move_base_flex/FTCPlanner/global_plan", 1, &TerrainObserverNode::planCallback, this);
    global_point_sub_ =
        nh_.subscribe("move_base_flex/FTCPlanner/global_point", 10, &TerrainObserverNode::globalPointCallback, this);

    const double update_rate_hz = param_nh_.param("update_rate_hz", 10.0);
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, update_rate_hz)),
                                    &TerrainObserverNode::update, this);
  }

 private:
  void loadParams() {
    tuning_.slip_lat_error_weight = param_nh_.param("slip_lat_error_weight", tuning_.slip_lat_error_weight);
    tuning_.slip_error_rate_weight =
        param_nh_.param("slip_error_rate_weight", tuning_.slip_error_rate_weight);
    tuning_.slip_speed_error_weight =
        param_nh_.param("slip_speed_error_weight", tuning_.slip_speed_error_weight);
    tuning_.slip_cross_slope_weight =
        param_nh_.param("slip_cross_slope_weight", tuning_.slip_cross_slope_weight);
    tuning_.compensation_slip_threshold =
        param_nh_.param("compensation_slip_threshold", tuning_.compensation_slip_threshold);
    tuning_.recovery_slip_threshold = param_nh_.param("recovery_slip_threshold", tuning_.recovery_slip_threshold);
    tuning_.hold_slip_threshold = param_nh_.param("hold_slip_threshold", tuning_.hold_slip_threshold);
    tuning_.compensation_cross_slope_deg =
        param_nh_.param("compensation_cross_slope_deg", tuning_.compensation_cross_slope_deg);
    tuning_.speed_scale_min = param_nh_.param("speed_scale_min", tuning_.speed_scale_min);
    tuning_.recovery_speed_scale = param_nh_.param("recovery_speed_scale", tuning_.recovery_speed_scale);
    tuning_.speed_scale_downhill_per_deg =
        param_nh_.param("speed_scale_downhill_per_deg", tuning_.speed_scale_downhill_per_deg);
    tuning_.speed_scale_cross_per_deg =
        param_nh_.param("speed_scale_cross_per_deg", tuning_.speed_scale_cross_per_deg);
    tuning_.uphill_speed_bonus_per_deg =
        param_nh_.param("uphill_speed_bonus_per_deg", tuning_.uphill_speed_bonus_per_deg);
    tuning_.speed_scale_max_uphill =
        param_nh_.param("speed_scale_max_uphill", tuning_.speed_scale_max_uphill);
    tuning_.heading_bias_gain = param_nh_.param("heading_bias_gain", tuning_.heading_bias_gain);
    tuning_.heading_bias_max_rad = param_nh_.param("heading_bias_max_rad", tuning_.heading_bias_max_rad);

    terrain_memory_file_ = param_nh_.param("terrain_memory_file", std::string("terrain_memory.json"));
    terrain_memory_lookahead_poses_ = param_nh_.param("terrain_memory_lookahead_poses", 20);
    terrain_memory_caution_threshold_ = param_nh_.param("terrain_memory_caution_threshold", 0.35);
    terrain_memory_save_seconds_ = param_nh_.param("terrain_memory_save_seconds", 5.0);
    memory_learning_min_speed_ = param_nh_.param("memory_learning_min_speed", 0.05);
  }

  void loadMemory() {
    if (memory_.load(terrain_memory_file_)) {
      ROS_INFO_STREAM("Terrain observer loaded terrain memory from " << terrain_memory_file_ << " with "
                                                                     << memory_.cellCount() << " cells");
    } else {
      ROS_INFO_STREAM("Terrain observer starting with empty terrain memory at " << terrain_memory_file_);
    }
  }

  bool clearMemory(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    memory_.clear();
    std::ifstream file(terrain_memory_file_);
    const bool exists = file.good();
    file.close();
    const bool removed = !exists || std::remove(terrain_memory_file_.c_str()) == 0;
    res.success = removed;
    res.message = removed ? "Cleared terrain memory" : "Failed to clear terrain memory file";
    return true;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    last_imu_ = *msg;
    have_imu_ = true;
  }

  void measuredTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    last_measured_twist_ = *msg;
    have_measured_twist_ = true;
  }

  void commandTwistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_command_twist_ = *msg;
    have_command_twist_ = true;
  }

  void poseCallback(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
    last_pose_ = *msg;
    have_pose_ = true;
  }

  void statusCallback(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
    last_status_ = *msg;
    have_status_ = true;
  }

  void pidCallback(const ftc_local_planner::PID::ConstPtr& msg) {
    last_pid_ = *msg;
    have_pid_ = true;
  }

  void planCallback(const nav_msgs::Path::ConstPtr& msg) {
    last_global_plan_ = *msg;
    have_global_plan_ = true;
  }

  void globalPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_global_point_ = *msg;
    have_global_point_ = true;
  }

  double resolvePathHeading(double fallback_heading) const {
    if (have_global_point_) {
      tf2::Quaternion q;
      tf2::fromMsg(last_global_point_.pose.orientation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return yaw;
    }
    if (have_global_plan_ && last_global_plan_.poses.size() >= 2) {
      size_t nearest_index = 0;
      double nearest_distance = std::numeric_limits<double>::max();
      const double x = last_pose_.pose.pose.position.x;
      const double y = last_pose_.pose.pose.position.y;
      for (size_t i = 0; i < last_global_plan_.poses.size(); ++i) {
        const double dx = last_global_plan_.poses[i].pose.position.x - x;
        const double dy = last_global_plan_.poses[i].pose.position.y - y;
        const double dist = dx * dx + dy * dy;
        if (dist < nearest_distance) {
          nearest_distance = dist;
          nearest_index = i;
        }
      }
      const size_t next_index = std::min(last_global_plan_.poses.size() - 1, nearest_index + 1);
      const auto& current = last_global_plan_.poses[nearest_index].pose.position;
      const auto& next = last_global_plan_.poses[next_index].pose.position;
      return std::atan2(next.y - current.y, next.x - current.x);
    }
    return fallback_heading;
  }

  void update(const ros::TimerEvent& event) {
    if (!have_imu_ || !have_pose_ || !have_status_) {
      return;
    }

    const double dt = last_publish_time_.isZero() ? 0.1 : std::max(1e-3, (event.current_real - last_publish_time_).toSec());
    last_publish_time_ = event.current_real;

    tf2::Quaternion q;
    tf2::fromMsg(last_imu_.orientation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw_unused = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_unused);

    const double robot_heading = last_pose_.vehicle_heading;
    const double path_heading = resolvePathHeading(robot_heading);
    const TerrainProjection projection =
        mower_logic::terrain::projectSlopeToPath(roll, pitch, robot_heading, path_heading);

    const double cross_track_error = have_pid_ ? last_pid_.lat_err : 0.0;
    const double cross_track_error_rate = (cross_track_error - last_cross_track_error_) / dt;
    last_cross_track_error_ = cross_track_error;

    const double commanded_speed = have_command_twist_ ? last_command_twist_.linear.x : 0.0;
    const double measured_speed = have_measured_twist_ ? last_measured_twist_.twist.linear.x : 0.0;
    const double speed_error = std::max(0.0, commanded_speed - measured_speed);

    const SlipInputs slip_inputs = {
        cross_track_error,
        cross_track_error_rate,
        speed_error,
        projection.cross_deg,
    };
    const double terrain_memory_risk =
        memory_.riskAt(last_pose_.pose.pose.position.x, last_pose_.pose.pose.position.y);
    const double risk_ahead =
        have_global_plan_ ? memory_.riskAlongPath(last_global_plan_, last_pose_.pose.pose.position.x,
                                                  last_pose_.pose.pose.position.y, terrain_memory_lookahead_poses_)
                          : 0.0;
    const double slip_score = mower_logic::terrain::computeSlipScore(slip_inputs, tuning_);
    const bool autonomous_active = last_status_.state == mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS &&
                                   last_status_.state_name == "MOWING";
    const TerrainMode mode =
        mower_logic::terrain::determineMode(autonomous_active, projection.cross_deg, slip_score,
                                            std::max(risk_ahead, terrain_memory_risk), tuning_);
    const double speed_scale =
        mower_logic::terrain::computeSpeedScale(projection.uphill_deg, projection.cross_deg, slip_score,
                                                std::max(risk_ahead, terrain_memory_risk), tuning_);
    const double heading_bias = mower_logic::terrain::computeHeadingBias(cross_track_error, cross_track_error_rate,
                                                                         projection.cross_deg, slip_score, tuning_);

    if (autonomous_active && std::abs(measured_speed) >= memory_learning_min_speed_) {
      memory_.observe(last_pose_.pose.pose.position.x, last_pose_.pose.pose.position.y,
                      TerrainSample{roll * 180.0 / M_PI, pitch * 180.0 / M_PI, projection.uphill_deg,
                                    projection.cross_deg, slip_score, mode == TerrainMode::RECOVERY,
                                    mode == TerrainMode::HOLD});
    }

    mower_msgs::TerrainState state_msg;
    state_msg.stamp = event.current_real;
    state_msg.mode = static_cast<uint8_t>(mode);
    state_msg.roll_deg = roll * 180.0 / M_PI;
    state_msg.pitch_deg = pitch * 180.0 / M_PI;
    state_msg.uphill_slope_deg = projection.uphill_deg;
    state_msg.cross_slope_deg = projection.cross_deg;
    state_msg.commanded_speed = commanded_speed;
    state_msg.measured_speed = measured_speed;
    state_msg.speed_error = speed_error;
    state_msg.cross_track_error = cross_track_error;
    state_msg.cross_track_error_rate = cross_track_error_rate;
    state_msg.slip_score = slip_score;
    state_msg.slip_direction = (std::abs(cross_track_error_rate) > 1e-3) ? std::copysign(1.0, cross_track_error_rate)
                                                                          : std::copysign(1.0, cross_track_error);
    state_msg.speed_scale = speed_scale;
    state_msg.heading_bias = heading_bias;
    state_msg.risk_ahead = risk_ahead;
    state_msg.terrain_memory_risk = terrain_memory_risk;
    state_msg.slip_confidence = mower_logic::terrain::clamp_value(
        std::abs(cross_track_error) + std::abs(cross_track_error_rate) + std::abs(projection.cross_deg) / 10.0, 0.0,
        1.0);
    if ((mode == TerrainMode::RECOVERY || mode == TerrainMode::HOLD) &&
        last_mode_ != TerrainMode::RECOVERY && last_mode_ != TerrainMode::HOLD) {
      recovery_count_ += 1;
    }
    last_mode_ = mode;
    state_msg.recovery_count = recovery_count_;
    state_msg.memory_active =
        terrain_memory_risk >= terrain_memory_caution_threshold_ || risk_ahead >= terrain_memory_caution_threshold_;
    terrain_pub_.publish(state_msg);

    if ((event.current_real - last_memory_save_).toSec() >= terrain_memory_save_seconds_) {
      memory_.save(terrain_memory_file_);
      last_memory_save_ = event.current_real;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle param_nh_;
  ObserverTuning tuning_;
  TerrainMemory memory_;

  ros::Publisher terrain_pub_;
  ros::ServiceServer clear_memory_srv_;
  ros::Subscriber imu_sub_;
  ros::Subscriber measured_twist_sub_;
  ros::Subscriber command_twist_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber pid_sub_;
  ros::Subscriber global_plan_sub_;
  ros::Subscriber global_point_sub_;
  ros::Timer update_timer_;

  sensor_msgs::Imu last_imu_;
  geometry_msgs::TwistStamped last_measured_twist_;
  geometry_msgs::Twist last_command_twist_;
  xbot_msgs::AbsolutePose last_pose_;
  mower_msgs::HighLevelStatus last_status_;
  ftc_local_planner::PID last_pid_;
  nav_msgs::Path last_global_plan_;
  geometry_msgs::PoseStamped last_global_point_;

  bool have_imu_ = false;
  bool have_measured_twist_ = false;
  bool have_command_twist_ = false;
  bool have_pose_ = false;
  bool have_status_ = false;
  bool have_pid_ = false;
  bool have_global_plan_ = false;
  bool have_global_point_ = false;

  std::string terrain_memory_file_;
  int terrain_memory_lookahead_poses_ = 20;
  double terrain_memory_caution_threshold_ = 0.35;
  double terrain_memory_save_seconds_ = 5.0;
  double memory_learning_min_speed_ = 0.05;

  double last_cross_track_error_ = 0.0;
  uint16_t recovery_count_ = 0;
  TerrainMode last_mode_ = TerrainMode::NORMAL;
  ros::Time last_publish_time_;
  ros::Time last_memory_save_;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_observer");
  TerrainObserverNode node;
  ros::spin();
  return 0;
}
