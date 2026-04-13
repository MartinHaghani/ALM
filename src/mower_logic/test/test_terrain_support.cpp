#include <gtest/gtest.h>

#include "mower_logic/terrain/TerrainMath.h"
#include "mower_logic/terrain/TerrainMemory.h"

TEST(TerrainMathTest, ProjectsSlopeIntoPathFrame) {
  const auto projection = mower_logic::terrain::projectSlopeToPath(0.1, 0.2, 0.0, 0.0);
  EXPECT_NEAR(projection.uphill_deg, 0.2 * 180.0 / M_PI, 1e-3);
  EXPECT_NEAR(projection.cross_deg, 0.1 * 180.0 / M_PI, 1e-3);
}

TEST(TerrainMathTest, SlipScoreIsClamped) {
  mower_logic::terrain::SlipInputs inputs;
  inputs.cross_track_error = 10.0;
  inputs.cross_track_error_rate = 10.0;
  inputs.speed_error = 10.0;
  inputs.cross_slope_deg = 30.0;
  EXPECT_DOUBLE_EQ(mower_logic::terrain::computeSlipScore(inputs, mower_logic::terrain::ObserverTuning{}), 1.0);
}

TEST(TerrainMathTest, HigherRiskReducesSpeedScale) {
  const auto low_risk = mower_logic::terrain::computeSpeedScale(0.0, 3.0, 0.1, 0.0, mower_logic::terrain::ObserverTuning{});
  const auto high_risk = mower_logic::terrain::computeSpeedScale(0.0, 3.0, 0.7, 0.8, mower_logic::terrain::ObserverTuning{});
  EXPECT_GT(low_risk, high_risk);
}

TEST(TerrainMemoryTest, LearnsAndExportsRiskCells) {
  mower_logic::terrain::TerrainMemory memory(0.5);
  mower_logic::terrain::TerrainSample sample;
  sample.cross_deg = 12.0;
  sample.slip_score = 0.9;
  sample.recovery = true;

  for (int i = 0; i < 8; ++i) {
    memory.observe(-1.0, 2.0, sample);
  }

  EXPECT_GT(memory.riskAt(-1.0, 2.0), 0.5);

  geometry_msgs::Polygon outline;
  geometry_msgs::Point32 point;
  point.x = -2.0;
  point.y = 1.0;
  outline.points.push_back(point);
  point.x = 0.0;
  point.y = 1.0;
  outline.points.push_back(point);
  point.x = 0.0;
  point.y = 3.0;
  outline.points.push_back(point);
  point.x = -2.0;
  point.y = 3.0;
  outline.points.push_back(point);

  const auto polygons = memory.buildExclusionPolygons(outline, 0.5, 0.1, 10);
  EXPECT_FALSE(polygons.empty());
}
