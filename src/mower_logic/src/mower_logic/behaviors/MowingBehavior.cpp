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
#include "MowingBehavior.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <iomanip>
#include <limits>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sstream>
#include <std_msgs/String.h>

#include "mower_logic/CheckPoint.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "xbot_msgs/MapOverlay.h"
#include "IdleBehavior.h"

extern ros::NodeHandle* n;
extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;
extern ros::ServiceClient dockingPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern xbot_msgs::AbsolutePose getPose();
extern bool isGpsGood();
extern bool getSelectedMapIdentity(std::string& map_id, std::string& map_hash);
extern void setConfig(mower_logic::MowerLogicConfig);
extern void stopBlade();
extern void stopMoving();

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

MowingBehavior MowingBehavior::INSTANCE;

namespace {
// The compiled WebUI bundle currently maps only red/green/blue overlay color names.
constexpr char kFullPlanOverlayColor[] = "blue";
constexpr char kRemainingPlanOverlayColor[] = "green";
constexpr float kFullPlanOverlayLineWidth = 0.05f;
constexpr float kRemainingPlanOverlayLineWidth = 0.10f;
constexpr double kFirstPointMaxStartDistanceM = 0.30;
constexpr double kFirstPointMaxStartYawErrorRad = 35.0 * M_PI / 180.0;
constexpr double kClosedOutlineDuplicateMaxDistanceM = 0.15;
constexpr double kOutlineEntryLeadInLengthM = 1.20;
constexpr double kOutlineEntryLeadInInsetM = 0.80;
constexpr size_t kOutlineEntryLeadInSamples = 8;

double yaw_from_quaternion(const geometry_msgs::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalized_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::Quaternion quaternion_from_yaw(double yaw) {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void append_json_string(std::ostringstream& out, const std::string& value) {
  out << '"';
  for (const char ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(ch))
              << std::dec << std::setfill(' ');
        } else {
          out << ch;
        }
    }
  }
  out << '"';
}

bool live_pose_reached_target(const xbot_msgs::AbsolutePose& pose, const geometry_msgs::PoseStamped& target,
                              double& distance, double& yaw_error) {
  const auto& current_position = pose.pose.pose.position;
  const auto& target_position = target.pose.position;
  distance = std::hypot(current_position.x - target_position.x, current_position.y - target_position.y);
  yaw_error = normalized_angle(yaw_from_quaternion(pose.pose.pose.orientation) - yaw_from_quaternion(target.pose.orientation));
  return distance <= kFirstPointMaxStartDistanceM && std::abs(yaw_error) <= kFirstPointMaxStartYawErrorRad;
}

double pose_distance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

size_t unique_closed_pose_count(const std::vector<geometry_msgs::PoseStamped>& poses) {
  if (poses.size() < 2) {
    return poses.size();
  }
  const bool has_duplicate_end =
      pose_distance(poses.front().pose, poses.back().pose) <= kClosedOutlineDuplicateMaxDistanceM;
  return has_duplicate_end ? poses.size() - 1 : poses.size();
}

double signed_outline_area(const std::vector<geometry_msgs::PoseStamped>& poses, size_t unique_pose_count) {
  double area = 0.0;
  if (unique_pose_count < 3) {
    return area;
  }
  for (size_t index = 0; index < unique_pose_count; ++index) {
    const auto& a = poses[index].pose.position;
    const auto& b = poses[(index + 1) % unique_pose_count].pose.position;
    area += a.x * b.y - b.x * a.y;
  }
  return 0.5 * area;
}

bool rotate_closed_outline_start(slic3r_coverage_planner::Path& path, const xbot_msgs::AbsolutePose& current_pose) {
  auto& poses = path.path.poses;
  if (!path.is_outline || poses.size() < 4) {
    return false;
  }

  const size_t unique_pose_count = unique_closed_pose_count(poses);
  const bool has_duplicate_end = unique_pose_count + 1 == poses.size();
  if (unique_pose_count < 3) {
    return false;
  }

  const auto& current_position = current_pose.pose.pose.position;
  const double current_yaw = yaw_from_quaternion(current_pose.pose.pose.orientation);
  size_t best_index = 0;
  double best_score = std::numeric_limits<double>::infinity();
  double best_distance = 0.0;
  double best_heading_error = 0.0;

  for (size_t index = 0; index < unique_pose_count; ++index) {
    const auto& candidate = poses[index].pose;
    const double dx = candidate.position.x - current_position.x;
    const double dy = candidate.position.y - current_position.y;
    const double distance = std::hypot(dx, dy);
    const double candidate_yaw = yaw_from_quaternion(candidate.orientation);
    const double heading_error = std::abs(normalized_angle(current_yaw - candidate_yaw));
    const double forward_projection = dx * std::cos(candidate_yaw) + dy * std::sin(candidate_yaw);
    const double lateral_projection = std::abs(-dx * std::sin(candidate_yaw) + dy * std::cos(candidate_yaw));

    const double behind_penalty = std::max(0.0, -forward_projection);
    const double score = distance + 0.6 * heading_error + 0.35 * behind_penalty + 0.15 * lateral_projection;
    if (score < best_score) {
      best_score = score;
      best_index = index;
      best_distance = distance;
      best_heading_error = heading_error;
    }
  }

  if (best_index == 0) {
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> rotated;
  rotated.reserve(unique_pose_count + (has_duplicate_end ? 1 : 0));
  for (size_t offset = 0; offset < unique_pose_count; ++offset) {
    rotated.push_back(poses[(best_index + offset) % unique_pose_count]);
  }
  if (has_duplicate_end) {
    rotated.push_back(rotated.front());
  }
  poses = rotated;

  ROS_WARN_STREAM("MowingBehavior: Rotated closed outline start to pose index "
                  << best_index << " based on current mower pose. distance=" << best_distance
                  << " heading_error_deg=" << best_heading_error * (180.0 / M_PI)
                  << " score=" << best_score);
  return true;
}

bool prepend_closed_outline_entry_lead_in(slic3r_coverage_planner::Path& path) {
  auto& poses = path.path.poses;
  if (!path.is_outline || poses.size() < 4) {
    return false;
  }

  const size_t unique_pose_count = unique_closed_pose_count(poses);
  if (unique_pose_count < 3) {
    return false;
  }

  const auto entry_pose = poses.front();
  const double entry_yaw = yaw_from_quaternion(entry_pose.pose.orientation);
  const double tangent_x = std::cos(entry_yaw);
  const double tangent_y = std::sin(entry_yaw);

  double inward_x = -tangent_y;
  double inward_y = tangent_x;
  if (signed_outline_area(poses, unique_pose_count) < 0.0) {
    inward_x *= -1.0;
    inward_y *= -1.0;
  }

  const auto& entry_position = entry_pose.pose.position;
  geometry_msgs::Point p0;
  p0.x = entry_position.x - tangent_x * kOutlineEntryLeadInLengthM + inward_x * kOutlineEntryLeadInInsetM;
  p0.y = entry_position.y - tangent_y * kOutlineEntryLeadInLengthM + inward_y * kOutlineEntryLeadInInsetM;
  p0.z = entry_position.z;

  geometry_msgs::Point p1;
  p1.x = p0.x + tangent_x * (kOutlineEntryLeadInLengthM * 0.55);
  p1.y = p0.y + tangent_y * (kOutlineEntryLeadInLengthM * 0.55);
  p1.z = entry_position.z;

  geometry_msgs::Point p2;
  p2.x = entry_position.x - tangent_x * (kOutlineEntryLeadInLengthM * 0.55);
  p2.y = entry_position.y - tangent_y * (kOutlineEntryLeadInLengthM * 0.55);
  p2.z = entry_position.z;

  geometry_msgs::Point p3 = entry_position;

  std::vector<geometry_msgs::PoseStamped> lead_in;
  lead_in.reserve(kOutlineEntryLeadInSamples + 1);

  auto make_pose = [&](const geometry_msgs::Point& point, double yaw) {
    geometry_msgs::PoseStamped pose = entry_pose;
    pose.pose.position = point;
    pose.pose.orientation = quaternion_from_yaw(yaw);
    return pose;
  };

  lead_in.push_back(make_pose(p0, entry_yaw));
  for (size_t sample = 1; sample <= kOutlineEntryLeadInSamples; ++sample) {
    const double u = static_cast<double>(sample) / static_cast<double>(kOutlineEntryLeadInSamples + 1);
    const double one_minus_u = 1.0 - u;
    geometry_msgs::Point point;
    point.x = one_minus_u * one_minus_u * one_minus_u * p0.x +
              3.0 * one_minus_u * one_minus_u * u * p1.x +
              3.0 * one_minus_u * u * u * p2.x + u * u * u * p3.x;
    point.y = one_minus_u * one_minus_u * one_minus_u * p0.y +
              3.0 * one_minus_u * one_minus_u * u * p1.y +
              3.0 * one_minus_u * u * u * p2.y + u * u * u * p3.y;
    point.z = entry_position.z;

    const double dx = 3.0 * one_minus_u * one_minus_u * (p1.x - p0.x) +
                      6.0 * one_minus_u * u * (p2.x - p1.x) + 3.0 * u * u * (p3.x - p2.x);
    const double dy = 3.0 * one_minus_u * one_minus_u * (p1.y - p0.y) +
                      6.0 * one_minus_u * u * (p2.y - p1.y) + 3.0 * u * u * (p3.y - p2.y);
    lead_in.push_back(make_pose(point, std::atan2(dy, dx)));
  }

  std::vector<geometry_msgs::PoseStamped> with_lead_in;
  with_lead_in.reserve(lead_in.size() + poses.size());
  with_lead_in.insert(with_lead_in.end(), lead_in.begin(), lead_in.end());
  with_lead_in.insert(with_lead_in.end(), poses.begin(), poses.end());
  poses = with_lead_in;

  ROS_WARN_STREAM("MowingBehavior: Prepended outline entry lead-in. staging=("
                  << p0.x << ", " << p0.y << ") entry=(" << entry_position.x << ", " << entry_position.y
                  << ") length=" << kOutlineEntryLeadInLengthM << " inset=" << kOutlineEntryLeadInInsetM
                  << " samples=" << kOutlineEntryLeadInSamples);
  return true;
}

const char* nav_state_name(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING:
      return "PENDING";
    case actionlib::SimpleClientGoalState::ACTIVE:
      return "ACTIVE";
    case actionlib::SimpleClientGoalState::RECALLED:
      return "RECALLED";
    case actionlib::SimpleClientGoalState::REJECTED:
      return "REJECTED";
    case actionlib::SimpleClientGoalState::PREEMPTED:
      return "PREEMPTED";
    case actionlib::SimpleClientGoalState::ABORTED:
      return "ABORTED";
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      return "SUCCEEDED";
    case actionlib::SimpleClientGoalState::LOST:
      return "LOST";
    default:
      return "UNKNOWN";
  }
}

void add_overlay_polyline(xbot_msgs::MapOverlay& overlay, const std::vector<geometry_msgs::PoseStamped>& poses,
                          size_t start_index, const char* color, float line_width) {
  if (start_index >= poses.size() || (poses.size() - start_index) < 2) {
    return;
  }

  xbot_msgs::MapOverlayPolygon polyline;
  polyline.closed = false;
  polyline.color = color;
  polyline.line_width = line_width;

  for (size_t pose_index = start_index; pose_index < poses.size(); ++pose_index) {
    const auto& pose = poses[pose_index].pose.position;
    geometry_msgs::Point32 pt;
    pt.x = pose.x;
    pt.y = pose.y;
    pt.z = 0.0f;
    polyline.polygon.points.push_back(pt);
  }

  overlay.polygons.push_back(polyline);
}

Behavior* getPostMowingBehavior() {
  mower_map::GetDockingPointSrv get_docking_point_srv;
  if (!dockingPointClient.call(get_docking_point_srv)) {
    ROS_WARN_STREAM("MowingBehavior: No parking point configured, returning to IDLE.");
    return &IdleBehavior::INSTANCE;
  }
  return &DockingBehavior::INSTANCE;
}
}  // namespace

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

Behavior* MowingBehavior::execute() {
  shared_state->active_semiautomatic_task = true;

  while (ros::ok() && !aborted) {
    if (currentMowingPaths.empty() && !create_mowing_plan(currentMowingArea)) {
      ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, leaving mowing state");
      // Start again from first area next time.
      reset();
      // We cannot create a plan, so we're probably done.
      return getPostMowingBehavior();
    }

    // No plan will be created if the area is skipped
    if (currentMowingPaths.empty()) {
      clear_mowing_overlay();
      currentMowingArea++;
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      continue;
    }

    // We have a plan, execute it
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    bool finished = execute_mowing_plan();
    if (finished) {
      // skip to next area if current
      ROS_INFO_STREAM("MowingBehavior: Executing mowing plan - finished");
      publish_route_plan(false);
      currentMowingArea++;
      currentMowingPaths.clear();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      clear_mowing_overlay();
    }
  }

  if (!ros::ok()) {
    // something went wrong
    return nullptr;
  }
  // we got aborted, or manual mowing completed, choose the safest reachable next state.
  return getPostMowingBehavior();
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  paused = aborted = false;
  map_overlay_pub = n->advertise<xbot_msgs::MapOverlay>("xbot_monitoring/map_overlay", 10);
  advertise_route_plan();
  clear_mowing_overlay();
  restore_checkpoint();

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
  clear_mowing_overlay();
  publish_route_plan(false);
  if (map_overlay_pub) {
    map_overlay_pub.shutdown();
  }
  registerActions("mower_logic:mowing", {});
}

void MowingBehavior::reset() {
  publish_route_plan(false);
  currentMowingPaths.clear();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  set_mowing_diagnostic("reset");
  clear_mowing_overlay();
  // increase cumulative mowing angle offset increment
  currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
  checkpoint();

  if (config.automatic_mode == eAutoMode::SEMIAUTO) {
    ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto& a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}

bool MowingBehavior::create_mowing_plan(int area_index) {
  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);
  // Delete old plan and progress.
  currentMowingPaths.clear();
  clear_mowing_overlay();

  // get the mowing area
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = area_index;
  if (!mapClient.call(mapSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
    return false;
  }

  if (mapSrv.response.area.area.points.empty()) {
    ROS_INFO_STREAM("MowingBehavior: Skipping inactive mowing area");
    return true;
  }

  // Area orientation is the same as the first point
  double angle = 0;
  auto points = mapSrv.response.area.area.points;
  if (points.size() >= 2) {
    tf2::Vector3 first(points[0].x, points[0].y, 0);
    for (auto point : points) {
      tf2::Vector3 second(point.x, point.y, 0);
      auto diff = second - first;
      if (diff.length() > 2.0) {
        // we have found a point that has a distance of > 1 m, calculate the angle
        angle = atan2(diff.y(), diff.x());
        ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
        break;
      }
    }
  }

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
  if (config.mow_angle_offset_is_absolute) {
    angle = mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
  } else {
    angle = angle + mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
  }

  // calculate coverage
  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = angle;
  pathSrv.request.outline_count = config.outline_count;
  pathSrv.request.outline_overlap_count = config.outline_overlap_count;
  pathSrv.request.outline = mapSrv.response.area.area;
  pathSrv.request.holes = mapSrv.response.area.obstacles;
  pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
  pathSrv.request.outer_offset = config.outline_offset;
  pathSrv.request.distance = config.tool_width;
  if (!pathClient.waitForExistence(ros::Duration(5.0, 0.0))) {
    ROS_ERROR_STREAM("MowingBehavior: Coverage planning service is unavailable");
    return false;
  }
  if (!pathClient.call(pathSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
    return false;
  }

  currentMowingPaths = pathSrv.response.paths;
  for (auto& mowing_path : currentMowingPaths) {
    if (!mowing_path.is_outline) {
      continue;
    }
    const auto current_pose = getPose();
    rotate_closed_outline_start(mowing_path, current_pose);
    prepend_closed_outline_entry_lead_in(mowing_path);
    break;
  }
  set_mowing_diagnostic("plan_created");

  // Calculate mowing plan digest from the poses
  // TODO: move to slic3r_coverage_planner
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& path : currentMowingPaths) {
    for (const auto& pose_stamped : path.path.poses) {
      hash.Update(reinterpret_cast<const byte*>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
    }
  }
  hash.Final((byte*)&digest[0]);
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << currentMowingPath
                                                                      << " index: " << currentMowingPathIndex);
  } else {
    ROS_INFO_STREAM("MowingBehavior: Ignoring checkpoint for plan ("
                    << currentMowingPlanDigest << ") current mowing plan is (" << mowingPlanDigest << ")");
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
  }

  publish_mowing_overlay();
  return true;
}

int getCurrentMowPathIndex() {
  ftc_local_planner::PlannerGetProgress progressSrv;
  int currentIndex = -1;
  if (pathProgressClient.call(progressSrv)) {
    currentIndex = progressSrv.response.index;
  } else {
    ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
    case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
    case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
    case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
    case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
    case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
    case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
    case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
    default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
  }
}

bool MowingBehavior::execute_mowing_plan() {
  ros::Time paused_time(0.0);

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      u_int8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          pause_reason += "waiting for CONTINUE";
        }
        ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: PAUSED (" << pause_reason << ")");
        ros::Rate r(1.0);
        r.sleep();
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = ros::Time::now();
      while (!(getConfig().ignore_gps_errors || this->hasGoodGPS() || isGpsGood()) &&
             !aborted)  // while no good GPS we wait
      {
        ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now() - paused_time).toSec()
                                                   << "s) (waiting for GPS)");
        ros::Rate r(1.0);
        r.sleep();
      }
      ROS_INFO_STREAM("MowingBehavior: CONTINUING");
      paused = false;
      update_actions();
    }

    auto& path = currentMowingPaths[currentMowingPath];
    ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= path.path.poses.size()) {
      ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
      currentMowingPath++;
      currentMowingPathIndex = 0;
      publish_mowing_overlay();
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    //
    // * we have n attempts, if we fail we go to pause() mode because most likely it was GPS problems that
    //   prevented us from reaching the inital pose
    // * after n attempts, we fail the mow area and skip to the next one
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
      if (path.is_outline && getConfig().add_fake_obstacle) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = path.path.poses[currentMowingPathIndex];
      moveBaseGoal.controller = "FTCPlanner";
      set_mowing_diagnostic("first_point_goal_sent");
      publish_route_plan(true);
      mbfClient->sendGoal(moveBaseGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClient->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            mbfClientExePath->cancelAllGoals();
            currentMowingPaths.clear();
            skip_area = false;
            clear_mowing_overlay();
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            publish_mowing_overlay();
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status "
                          << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      bool first_point_reached = current_status.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;
      if (first_point_reached) {
        double first_point_distance = 0.0;
        double first_point_yaw_error = 0.0;
        if (!live_pose_reached_target(getPose(), path.path.poses[currentMowingPathIndex], first_point_distance,
                                      first_point_yaw_error)) {
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) FTCPlanner reported success, but live pose is still "
                          << first_point_distance << " m and " << first_point_yaw_error * (180.0 / M_PI)
                          << " deg from the target. Pausing for inspection.");
          stopMoving();
          first_point_reached = false;
        }
      }

      if (!first_point_reached) {
        // we cannot reach the start point
        std::ostringstream first_point_error;
        first_point_error << "Could not reach first point. MBF state=" << nav_state_name(current_status.state_)
                          << " progress_index=" << currentMowingPathIndex << " path=" << currentMowingPath
                          << " area=" << currentMowingArea;
        set_mowing_diagnostic("first_point_error_pause", first_point_error.str(), current_status.state_);
        publish_route_plan(true);
        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - " << first_point_error.str());
        ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Pausing without retrying or trimming the route.");
        mowerEnabled = false;
        stopBlade();
        stopMoving();
        requestPause(pauseType::PAUSE_MANUAL);
        update_actions();
        continue;
      }

      mower_map::ClearNavPointSrv clear_nav_point_srv;
      clearNavPointClient.call(clear_nav_point_srv);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      mowerEnabled = true;

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath.header = path.path.header;
      exePath.poses = std::vector<geometry_msgs::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex,
                                                              path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

      ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with "
                      << path.path.poses.size() << " poses, from index " << exePathStartIndex);
      set_mowing_diagnostic("mow_path_goal_sent");
      publish_route_plan(true);
      mbfClientExePath->sendGoal(exePathGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClientExePath->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            publish_route_plan(false);
            currentMowingPaths.clear();
            skip_area = false;
            clear_mowing_overlay();
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            publish_mowing_overlay();
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
            // show progress
            int currentIndex = getCurrentMowPathIndex();
            if (currentIndex != -1) {
              int nextMowingPathIndex = exePathStartIndex + currentIndex;
              if (nextMowingPathIndex != currentMowingPathIndex) {
                currentMowingPathIndex = nextMowingPathIndex;
                publish_mowing_overlay();
              }
            }
            ROS_INFO_STREAM_THROTTLE(
                5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
            if (ros::Time::now() - last_checkpoint > ros::Duration(30.0)) checkpoint();
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_
                                                               << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      if (aborted) {
        return false;
      }
      if (requested_pause_flag) {
        publish_mowing_overlay();
        continue;
      }

      // Only skip/trim if goal execution began
      if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
          current_status.state_ != actionlib::SimpleClientGoalState::RECALLED) {
        ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = "
                        << currentMowingPathIndex << " of " << path.path.poses.size());
        printNavState(current_status.state_);
        const bool planner_succeeded = current_status.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;
        const int path_pose_count = static_cast<int>(path.path.poses.size());
        const int remaining_pose_count = path_pose_count - currentMowingPathIndex;
        const bool progress_at_end = currentMowingPathIndex >= path_pose_count || remaining_pose_count < 5;
        // if we have fully processed the segment or we have encountered an error, drop the path segment
        /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
            the currentIndex far from the size of the poses ! (BUG in planner ?)
            instead we trust only the currentIndex vs. poses.size() */
        if (planner_succeeded && progress_at_end)  // fully mowed the path ?
        {
          ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          set_mowing_diagnostic("mow_path_finished");
          currentMowingPath++;
          currentMowingPathIndex = 0;
          publish_mowing_overlay();
          // continue with next segment
        } else {
          std::ostringstream mow_error;
          mow_error << "MBF/FTC did not complete mow path. state=" << nav_state_name(current_status.state_)
                    << " progress=" << currentMowingPathIndex << "/" << path_pose_count
                    << " remaining=" << std::max(remaining_pose_count, 0) << " path=" << currentMowingPath
                    << " area=" << currentMowingArea;
          if (planner_succeeded) {
            mow_error << " note=planner_reported_success_before_progress_end";
          }
          ROS_ERROR_STREAM("MowingBehavior: (MOW) " << mow_error.str()
                                                    << ". Holding manual pause for inspection instead of auto-resuming.");
          set_mowing_diagnostic("mow_path_error_pause", mow_error.str(), current_status.state_);

          publish_mowing_overlay();
          if (!requested_pause_flag) {
            mowerEnabled = false;
            stopBlade();
            stopMoving();
            requestPause(pauseType::PAUSE_MANUAL);
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF/FTC error at " << currentMowingPathIndex);
            update_actions();
          }
        }
      }
    }
  }

  mowerEnabled = false;

  // true, if we have executed all paths
  return currentMowingPath >= currentMowingPaths.size();
}

void MowingBehavior::command_home() {
  auto config = getConfig();
  if (config.manual_pause_mowing) {
    ROS_INFO_STREAM("MowingBehavior: clearing manual pause because stop/home was requested");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  if (shared_state->active_semiautomatic_task) {
    ROS_INFO_STREAM("MowingBehavior: stopping semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
  if (paused) {
    // Request continue to wait for odom
    this->requestContinue();
    // Then instantly abort i.e. go to dock.
  }
  this->abort();
}

void MowingBehavior::command_start() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");
  auto config = getConfig();
  if (shared_state->active_semiautomatic_task && config.manual_pause_mowing) {
    // We are in semiautomatic task and paused, user wants to resume, so store that immediately.
    // This way, once we are docked the mower will continue as soon as all other conditions are g2g
    ROS_INFO_STREAM("Resuming semiautomatic task");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  this->requestContinue();
}

void MowingBehavior::command_s1() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() {
  return currentMowingArea;
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = ros::Time(0.0);
  currentMowingPath = 0;
  currentMowingArea = 0;
  currentMowingPathIndex = 0;
  currentMowingAngleIncrementSum = 0.0;
  lastMbfState = -1;
  xbot_msgs::ActionInfo pause_action;
  pause_action.action_id = "pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::ActionInfo continue_action;
  continue_action.action_id = "continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::ActionInfo skip_area_action;
  skip_area_action.action_id = "skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::ActionInfo skip_path_action;
  skip_path_action.action_id = "skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
}

void MowingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:mowing/pause") {
    ROS_INFO_STREAM("got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    ROS_INFO_STREAM("got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    ROS_INFO_STREAM("got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/skip_area") {
    ROS_INFO_STREAM("got skip_area command");
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    ROS_INFO_STREAM("got skip_path command");
    skip_path = true;
  }
  update_actions();
}

void MowingBehavior::checkpoint() {
  rosbag::Bag bag;
  mower_logic::CheckPoint cp;
  cp.currentMowingPath = currentMowingPath;
  cp.currentMowingArea = currentMowingArea;
  cp.currentMowingPathIndex = currentMowingPathIndex;
  cp.currentMowingPlanDigest = currentMowingPlanDigest;
  cp.currentMowingAngleIncrementSum = currentMowingAngleIncrementSum;
  getSelectedMapIdentity(cp.selected_map_id, cp.selected_map_hash);
  bag.open("checkpoint.bag", rosbag::bagmode::Write);
  bag.write("checkpoint", ros::Time::now(), cp);
  bag.close();
  last_checkpoint = ros::Time::now();
}

bool MowingBehavior::restore_checkpoint() {
  rosbag::Bag bag;
  bool found = false;
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  currentMowingAngleIncrementSum = 0;
  currentMowingPlanDigest.clear();
  try {
    bag.open("checkpoint.bag");
  } catch (rosbag::BagIOException& e) {
    // Checkpoint does not exist or is corrupt, start at the very beginning
    return false;
  }
  {
    rosbag::View view(bag, rosbag::TopicQuery("checkpoint"));
    for (rosbag::MessageInstance const m : view) {
      auto cp = m.instantiate<mower_logic::CheckPoint>();
      if (cp) {
        std::string active_map_id;
        std::string active_map_hash;
        if (!getSelectedMapIdentity(active_map_id, active_map_hash) || cp->selected_map_id.empty() ||
            cp->selected_map_hash.empty() || cp->selected_map_id != active_map_id ||
            cp->selected_map_hash != active_map_hash) {
          ROS_WARN_STREAM("MowingBehavior: Ignoring checkpoint because it belongs to map "
                          << cp->selected_map_id << " (" << cp->selected_map_hash << ") but active map is "
                          << active_map_id << " (" << active_map_hash << ")");
          continue;
        }
        ROS_INFO_STREAM("Restoring checkpoint for plan ("
                        << cp->currentMowingPlanDigest << ")"
                        << " area: " << cp->currentMowingArea << " path: " << cp->currentMowingPath
                        << " index: " << cp->currentMowingPathIndex
                        << " angle increment sum: " << cp->currentMowingAngleIncrementSum);
        currentMowingPath = cp->currentMowingPath;
        currentMowingArea = cp->currentMowingArea;
        currentMowingPathIndex = cp->currentMowingPathIndex;
        currentMowingPlanDigest = cp->currentMowingPlanDigest;
        currentMowingAngleIncrementSum = cp->currentMowingAngleIncrementSum;
        found = true;
        break;
      }
    }
    bag.close();
  }
  return found;
}

void MowingBehavior::start_new_session() {
  ROS_INFO_STREAM("MowingBehavior: Starting a fresh mowing session, clearing any stored checkpoint.");
  currentMowingPaths.clear();
  currentMowingPath = 0;
  currentMowingArea = 0;
  currentMowingPathIndex = 0;
  currentMowingPlanDigest.clear();
  currentMowingAngleIncrementSum = 0.0;
  set_mowing_diagnostic("fresh_session");
  last_checkpoint = ros::Time(0.0);

  if (std::remove("checkpoint.bag") != 0 && errno != ENOENT) {
    ROS_WARN_STREAM("MowingBehavior: Failed to remove checkpoint.bag: " << std::strerror(errno));
  }
}

void MowingBehavior::set_mowing_diagnostic(const std::string& event, const std::string& error, int mbf_state) {
  lastMowingEvent = event;
  lastMowingError = error;
  lastMbfState = mbf_state;
}

void MowingBehavior::publish_mowing_overlay() {
  if (!map_overlay_pub) {
    return;
  }

  if (currentMowingPaths.empty()) {
    clear_mowing_overlay();
    return;
  }

  const size_t current_path_index = static_cast<size_t>(currentMowingPath);
  if (current_path_index >= currentMowingPaths.size()) {
    clear_mowing_overlay();
    return;
  }

  xbot_msgs::MapOverlay overlay;

  for (const auto& mowing_path : currentMowingPaths) {
    add_overlay_polyline(
        overlay, mowing_path.path.poses, 0, kFullPlanOverlayColor, kFullPlanOverlayLineWidth);
  }

  for (size_t path_index = current_path_index; path_index < currentMowingPaths.size(); ++path_index) {
    const auto& mowing_path = currentMowingPaths[path_index];
    const size_t start_index = path_index == current_path_index ? static_cast<size_t>(std::max(currentMowingPathIndex, 0))
                                                                : 0;
    add_overlay_polyline(
        overlay, mowing_path.path.poses, start_index, kRemainingPlanOverlayColor, kRemainingPlanOverlayLineWidth);
  }

  map_overlay_pub.publish(overlay);
  publish_route_plan(true);
}

void MowingBehavior::clear_mowing_overlay() {
  if (!map_overlay_pub) {
    return;
  }

  xbot_msgs::MapOverlay overlay;
  map_overlay_pub.publish(overlay);
}

void MowingBehavior::advertise_route_plan() {
  if (!route_plan_pub) {
    route_plan_pub = n->advertise<std_msgs::String>("mower_logic/route_plan_json", 1, true);
  }
}

bool MowingBehavior::preview_route_plan(std::string& message) {
  advertise_route_plan();

  const auto saved_paths = currentMowingPaths;
  const int saved_area = currentMowingArea;
  const int saved_path = currentMowingPath;
  const int saved_path_index = currentMowingPathIndex;
  const std::string saved_digest = currentMowingPlanDigest;

  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;

  const bool created = create_mowing_plan(currentMowingArea);
  const bool preview_available = created && !currentMowingPaths.empty();
  if (created && !currentMowingPaths.empty()) {
    publish_route_plan(false);
    std::ostringstream status;
    status << "Previewed " << currentMowingPaths.size() << " path";
    if (currentMowingPaths.size() != 1) {
      status << "s";
    }
    status << " before mowing.";
    message = status.str();
  } else if (created) {
    message = "No active mowing area was available to preview.";
  } else {
    message = "Could not create route preview. Check the selected map and PlanPath service.";
  }

  currentMowingPaths = saved_paths;
  currentMowingArea = saved_area;
  currentMowingPath = saved_path;
  currentMowingPathIndex = saved_path_index;
  currentMowingPlanDigest = saved_digest;

  return preview_available;
}

void MowingBehavior::publish_route_plan(bool active) {
  advertise_route_plan();
  if (!route_plan_pub || currentMowingPaths.empty()) {
    return;
  }

  std::string frame_id = "map";
  for (const auto& mowing_path : currentMowingPaths) {
    if (!mowing_path.path.header.frame_id.empty()) {
      frame_id = mowing_path.path.header.frame_id;
      break;
    }
  }

  const ros::Time stamp = ros::Time::now();
  std::string map_id;
  std::string map_hash;
  const bool has_map_identity = getSelectedMapIdentity(map_id, map_hash);

  std::ostringstream out;
  out << std::setprecision(10);
  out << "{";
  out << "\"schema\":\"open_mower.route_plan.v0\",";
  out << "\"source\":\"mower_logic\",";
  out << "\"plan_id\":";
  append_json_string(out, currentMowingPlanDigest);
  out << ",\"frame_id\":";
  append_json_string(out, frame_id);
  out << ",\"stamp\":{\"secs\":" << stamp.sec << ",\"nsecs\":" << stamp.nsec << "},";
  out << "\"active\":" << (active ? "true" : "false") << ",";
  out << "\"current_area_index\":" << currentMowingArea << ",";
  out << "\"current_path_index\":" << currentMowingPath << ",";
  out << "\"current_pose_index\":" << currentMowingPathIndex;
  out << ",\"last_event\":";
  append_json_string(out, lastMowingEvent);
  out << ",\"last_error\":";
  append_json_string(out, lastMowingError);
  out << ",\"last_mbf_state\":" << lastMbfState;
  if (has_map_identity) {
    out << ",\"map_id\":";
    append_json_string(out, map_id);
    out << ",\"map_hash\":";
    append_json_string(out, map_hash);
  }
  out << ",\"paths\":[";
  for (size_t path_index = 0; path_index < currentMowingPaths.size(); ++path_index) {
    const auto& mowing_path = currentMowingPaths[path_index];
    const std::string label = std::string("path ") + std::to_string(path_index + 1) +
                              (mowing_path.is_outline ? " outline" : " fill");
    const std::string path_frame_id = mowing_path.path.header.frame_id.empty() ? frame_id : mowing_path.path.header.frame_id;
    if (path_index > 0) {
      out << ",";
    }
    out << "{\"path_index\":" << path_index << ",";
    out << "\"label\":";
    append_json_string(out, label);
    out << ",\"is_outline\":" << (mowing_path.is_outline ? "true" : "false") << ",";
    out << "\"frame_id\":";
    append_json_string(out, path_frame_id);
    out << ",\"poses\":[";
    const auto& poses = mowing_path.path.poses;
    for (size_t pose_index = 0; pose_index < poses.size(); ++pose_index) {
      const auto& pose = poses[pose_index].pose;
      if (pose_index > 0) {
        out << ",";
      }
      out << "{\"pose_index\":" << pose_index << ",";
      out << "\"x\":" << pose.position.x << ",";
      out << "\"y\":" << pose.position.y << ",";
      out << "\"yaw\":" << yaw_from_quaternion(pose.orientation) << "}";
    }
    out << "]}";
  }
  out << "]}";

  std_msgs::String message;
  message.data = out.str();
  route_plan_pub.publish(message);
}
