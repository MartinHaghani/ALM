#ifndef SRC_AREA_RECORDING_POSE_GATE_H
#define SRC_AREA_RECORDING_POSE_GATE_H

#include <cmath>
#include <string>

#include "xbot_msgs/AbsolutePose.h"

namespace mower_logic::area_recording {

struct RecordingPoseGateOptions {
  double max_gps_accuracy_m = 0.2;
  double max_pose_age_s = 1.0;
  double max_fused_position_accuracy_m = 0.2;
  double max_fused_yaw_accuracy_rad = 0.15;
};

struct RecordingPoseGateResult {
  bool accepted = false;
  bool boundary_sample_allowed = false;
  std::string reason;
};

inline bool absolutePoseFinite(const xbot_msgs::AbsolutePose& pose) {
  const auto& position = pose.pose.pose.position;
  const auto& orientation = pose.pose.pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) && std::isfinite(orientation.w);
}

inline bool gpsQualityOk(const xbot_msgs::AbsolutePose& gps_pose,
                         double max_gps_accuracy_m,
                         std::string* reason = nullptr) {
  if ((gps_pose.flags & xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED) == 0) {
    if (reason) *reason = "RTK fixed GPS is required for boundary samples";
    return false;
  }
  if (!std::isfinite(gps_pose.position_accuracy)) {
    if (reason) *reason = "GPS accuracy is unavailable";
    return false;
  }
  if (gps_pose.position_accuracy > max_gps_accuracy_m) {
    if (reason) *reason = "GPS accuracy is above the recording limit";
    return false;
  }
  if (reason) reason->clear();
  return true;
}

inline RecordingPoseGateResult evaluateLegacyRecordingPose(const xbot_msgs::AbsolutePose& legacy_pose,
                                                           double legacy_pose_age_s,
                                                           const xbot_msgs::AbsolutePose* raw_gps_pose,
                                                           const RecordingPoseGateOptions& options) {
  RecordingPoseGateResult result;
  if (!absolutePoseFinite(legacy_pose)) {
    result.reason = "Legacy recording pose is invalid";
    return result;
  }
  if (!std::isfinite(legacy_pose_age_s) || legacy_pose_age_s > options.max_pose_age_s) {
    result.reason = "Legacy recording pose is stale";
    return result;
  }

  const xbot_msgs::AbsolutePose& gps_quality_pose = raw_gps_pose ? *raw_gps_pose : legacy_pose;
  std::string gps_reason;
  if (!gpsQualityOk(gps_quality_pose, options.max_gps_accuracy_m, &gps_reason)) {
    result.reason = gps_reason.empty() ? "GPS quality is not acceptable for legacy recording" : gps_reason;
    return result;
  }

  result.accepted = true;
  result.boundary_sample_allowed = true;
  return result;
}

inline RecordingPoseGateResult evaluateFusedRecordingPose(const xbot_msgs::AbsolutePose& fused_pose,
                                                          double fused_pose_age_s,
                                                          const xbot_msgs::AbsolutePose* raw_gps_pose,
                                                          const RecordingPoseGateOptions& options) {
  RecordingPoseGateResult result;
  if (!absolutePoseFinite(fused_pose)) {
    result.reason = "Fused recording pose is invalid";
    return result;
  }
  if (!std::isfinite(fused_pose_age_s) || fused_pose_age_s > options.max_pose_age_s) {
    result.reason = "Fused recording pose is stale";
    return result;
  }
  if (!fused_pose.orientation_valid) {
    result.reason = "Fused recording pose orientation is unavailable";
    return result;
  }
  if ((fused_pose.flags & xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE) == 0) {
    result.reason = "Fused recording pose is not backed by a recent trusted source";
    return result;
  }
  if (!std::isfinite(fused_pose.position_accuracy) ||
      fused_pose.position_accuracy > options.max_fused_position_accuracy_m) {
    result.reason = "Fused recording pose position accuracy is above the recording limit";
    return result;
  }
  if (!std::isfinite(fused_pose.orientation_accuracy) ||
      fused_pose.orientation_accuracy > options.max_fused_yaw_accuracy_rad) {
    result.reason = "Fused recording pose yaw accuracy is above the recording limit";
    return result;
  }

  result.accepted = true;
  result.boundary_sample_allowed = raw_gps_pose && gpsQualityOk(*raw_gps_pose, options.max_gps_accuracy_m);
  return result;
}

}  // namespace mower_logic::area_recording

#endif  // SRC_AREA_RECORDING_POSE_GATE_H
