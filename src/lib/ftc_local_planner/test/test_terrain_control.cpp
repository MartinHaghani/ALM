#include <gtest/gtest.h>

#include "ftc_local_planner/terrain_control.h"

TEST(TerrainControlTest, FrozenProgressInRecovery) {
  ftc_local_planner::TerrainControlInput input;
  input.enabled = true;
  input.fresh = true;
  input.mode = 2;
  input.slip_score = 0.7;
  input.speed_scale = 0.4;

  const auto modifiers =
      ftc_local_planner::computeTerrainCommandModifiers(input, ftc_local_planner::TerrainControlConfig{});
  EXPECT_TRUE(modifiers.freeze_progress);
  EXPECT_TRUE(modifiers.recovery_requested);
}

TEST(TerrainControlTest, HeadingBiasGetsClamped) {
  ftc_local_planner::TerrainControlInput input;
  input.enabled = true;
  input.fresh = true;
  input.heading_bias = 10.0;

  ftc_local_planner::TerrainControlConfig config;
  config.heading_bias_max_rad = 0.2;
  const auto modifiers = ftc_local_planner::computeTerrainCommandModifiers(input, config);
  EXPECT_NEAR(modifiers.heading_bias, 0.2, 1e-9);
}

TEST(TerrainControlTest, RecoveryReleaseNeedsLowSlipAndLowError) {
  ftc_local_planner::TerrainControlInput input;
  input.fresh = true;
  input.slip_score = 0.2;
  input.cross_track_error = 0.05;
  EXPECT_TRUE(ftc_local_planner::shouldReleaseTerrainRecovery(input, ftc_local_planner::TerrainControlConfig{}));

  input.cross_track_error = 0.5;
  EXPECT_FALSE(ftc_local_planner::shouldReleaseTerrainRecovery(input, ftc_local_planner::TerrainControlConfig{}));
}
