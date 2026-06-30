#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mower_logic/behaviors/AreaRecordingPoseGate.h"

namespace {

xbot_msgs::AbsolutePose makePose(uint16_t flags,
                                 double position_accuracy,
                                 double orientation_accuracy,
                                 bool orientation_valid = true) {
  xbot_msgs::AbsolutePose pose;
  pose.flags = flags;
  pose.orientation_valid = orientation_valid ? 1 : 0;
  pose.position_accuracy = position_accuracy;
  pose.orientation_accuracy = orientation_accuracy;
  pose.pose.pose.position.x = 1.0;
  pose.pose.pose.position.y = 2.0;
  pose.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.25);
  pose.pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

mower_logic::area_recording::RecordingPoseGateOptions options() {
  mower_logic::area_recording::RecordingPoseGateOptions result;
  result.max_gps_accuracy_m = 0.2;
  result.max_pose_age_s = 1.0;
  result.max_fused_position_accuracy_m = 0.2;
  result.max_fused_yaw_accuracy_rad = 0.15;
  return result;
}

}  // namespace

TEST(AreaRecordingPoseGate, FusedGoodAllowsRecordingWithoutBoundaryWhenRawGpsFloat) {
  const auto fused = makePose(
      xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE, 0.05, 0.04);
  const auto raw_gps = makePose(xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT, 0.05, 0.04);

  const auto result = mower_logic::area_recording::evaluateFusedRecordingPose(fused, 0.2, &raw_gps, options());

  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_FALSE(result.boundary_sample_allowed);
}

TEST(AreaRecordingPoseGate, FusedGoodWithRawRtkAllowsBoundarySample) {
  const auto fused = makePose(
      xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE, 0.05, 0.04);
  const auto raw_gps = makePose(xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED, 0.05, 0.04);

  const auto result = mower_logic::area_recording::evaluateFusedRecordingPose(fused, 0.2, &raw_gps, options());

  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.boundary_sample_allowed);
}

TEST(AreaRecordingPoseGate, RejectsFusedStalePose) {
  const auto fused = makePose(
      xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE, 0.05, 0.04);

  const auto result = mower_logic::area_recording::evaluateFusedRecordingPose(fused, 1.2, nullptr, options());

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.reason.empty());
}

TEST(AreaRecordingPoseGate, RejectsFusedHighPositionSigma) {
  const auto fused = makePose(
      xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE, 0.25, 0.04);

  const auto result = mower_logic::area_recording::evaluateFusedRecordingPose(fused, 0.2, nullptr, options());

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.reason.empty());
}

TEST(AreaRecordingPoseGate, RejectsFusedMissingRecentTrustedFlag) {
  const auto fused = makePose(xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING, 0.05, 0.04);

  const auto result = mower_logic::area_recording::evaluateFusedRecordingPose(fused, 0.2, nullptr, options());

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.reason.empty());
}

TEST(AreaRecordingPoseGate, LegacyRtkGoodAccepted) {
  const auto legacy = makePose(0, 0.05, 0.04);
  const auto raw_gps = makePose(xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED, 0.05, 0.04);

  const auto result = mower_logic::area_recording::evaluateLegacyRecordingPose(legacy, 0.2, &raw_gps, options());

  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.boundary_sample_allowed);
}

TEST(AreaRecordingPoseGate, LegacyRtkBadRejected) {
  const auto legacy = makePose(0, 0.05, 0.04);
  const auto raw_gps = makePose(xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT, 0.05, 0.04);

  const auto result = mower_logic::area_recording::evaluateLegacyRecordingPose(legacy, 0.2, &raw_gps, options());

  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.boundary_sample_allowed);
  EXPECT_FALSE(result.reason.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
