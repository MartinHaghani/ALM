#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mower_logic/behaviors/SweptAreaRecorder.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::Point32 point(double x, double y) {
  geometry_msgs::Point32 result;
  result.x = x;
  result.y = y;
  result.z = 0.0;
  return result;
}

geometry_msgs::Pose pose(double x, double y, double yaw) {
  geometry_msgs::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  result.orientation = tf2::toMsg(q);
  return result;
}

std::vector<geometry_msgs::Point32> centeredFootprint() {
  return {
      point(-0.35, 0.25),
      point(0.35, 0.25),
      point(0.35, -0.25),
      point(-0.35, -0.25),
  };
}

mower_logic::area_recording::SweptAreaOptions options() {
  mower_logic::area_recording::SweptAreaOptions result;
  result.pose_step_m = 0.10;
  result.yaw_step_rad = 0.10;
  result.simplify_epsilon_m = 0.02;
  result.min_polygon_area_m2 = 0.01;
  return result;
}

bool polygonClosed(const geometry_msgs::Polygon& polygon) {
  if (polygon.points.size() < 4) {
    return false;
  }
  const auto& first = polygon.points.front();
  const auto& last = polygon.points.back();
  return first.x == last.x && first.y == last.y;
}

double minX(const geometry_msgs::Polygon& polygon) {
  double result = std::numeric_limits<double>::infinity();
  for (const auto& p : polygon.points) {
    result = std::min(result, static_cast<double>(p.x));
  }
  return result;
}

double maxX(const geometry_msgs::Polygon& polygon) {
  double result = -std::numeric_limits<double>::infinity();
  for (const auto& p : polygon.points) {
    result = std::max(result, static_cast<double>(p.x));
  }
  return result;
}

bool hasPointInBox(const geometry_msgs::Polygon& polygon, double min_x, double max_x, double min_y, double max_y) {
  return std::any_of(polygon.points.begin(), polygon.points.end(), [&](const geometry_msgs::Point32& p) {
    return p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y;
  });
}

}  // namespace

TEST(SweptAreaRecorder, RejectsDegenerateFootprint) {
  mower_logic::area_recording::SweptAreaRecorder recorder({point(0.0, 0.0), point(1.0, 0.0)});

  std::string error;
  EXPECT_FALSE(recorder.validFootprint(&error));
  EXPECT_FALSE(error.empty());
}

TEST(SweptAreaRecorder, RectanglePerimeterProducesExteriorBoundary) {
  mower_logic::area_recording::SweptAreaRecorder recorder(centeredFootprint());
  std::vector<std::vector<geometry_msgs::Pose>> segments = {{
      pose(0.0, 0.0, 0.0),
      pose(4.0, 0.0, 0.0),
      pose(4.0, 3.0, kPi / 2.0),
      pose(0.0, 3.0, kPi),
      pose(0.0, 0.0, -kPi / 2.0),
  }};

  const auto result = recorder.buildBoundary(
      segments, options(), mower_logic::area_recording::BoundarySelection::LARGEST_EXTERIOR);

  ASSERT_TRUE(result.success) << result.error;
  EXPECT_TRUE(polygonClosed(result.polygon));
  EXPECT_GT(result.polygon.points.size(), 4u);
  EXPECT_LT(minX(result.polygon), -0.2);
  EXPECT_GT(maxX(result.polygon), 4.2);
}

TEST(SweptAreaRecorder, ConcaveNotchSurvivesBoundaryExtraction) {
  mower_logic::area_recording::SweptAreaRecorder recorder(centeredFootprint());
  auto test_options = options();
  test_options.simplify_epsilon_m = 0.0;
  std::vector<std::vector<geometry_msgs::Pose>> segments = {{
      pose(0.0, 0.0, 0.0),
      pose(4.0, 0.0, 0.0),
      pose(4.0, 4.0, kPi / 2.0),
      pose(2.5, 4.0, kPi),
      pose(2.5, 2.0, -kPi / 2.0),
      pose(1.5, 2.0, kPi),
      pose(1.5, 4.0, kPi / 2.0),
      pose(0.0, 4.0, kPi),
      pose(0.0, 0.0, -kPi / 2.0),
  }};

  const auto result = recorder.buildBoundary(
      segments, test_options, mower_logic::area_recording::BoundarySelection::LARGEST_EXTERIOR);

  ASSERT_TRUE(result.success) << result.error;
  EXPECT_TRUE(polygonClosed(result.polygon));
  EXPECT_TRUE(hasPointInBox(result.polygon, 1.0, 3.0, 1.5, 2.5));
}

TEST(SweptAreaRecorder, PivotInPlaceProducesConnectedSweptArea) {
  mower_logic::area_recording::SweptAreaRecorder recorder(centeredFootprint());
  std::vector<std::vector<geometry_msgs::Pose>> segments = {{
      pose(0.0, 0.0, 0.0),
      pose(0.0, 0.0, kPi / 2.0),
      pose(0.0, 0.0, kPi),
      pose(0.0, 0.0, -kPi / 2.0),
      pose(0.0, 0.0, 0.0),
  }};

  const auto result = recorder.buildBoundary(
      segments, options(), mower_logic::area_recording::BoundarySelection::LARGEST_EXTERIOR);

  ASSERT_TRUE(result.success) << result.error;
  EXPECT_TRUE(polygonClosed(result.polygon));
  EXPECT_GT(result.union_area_m2, 0.30);
}

TEST(SweptAreaRecorder, DropoutSegmentsDoNotBridgeGap) {
  mower_logic::area_recording::SweptAreaRecorder recorder(centeredFootprint());
  std::vector<std::vector<geometry_msgs::Pose>> segments = {
      {pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0)},
      {pose(5.0, 0.0, 0.0), pose(6.0, 0.0, 0.0)},
  };

  const auto result = recorder.buildBoundary(
      segments, options(), mower_logic::area_recording::BoundarySelection::LARGEST_EXTERIOR);

  ASSERT_TRUE(result.success) << result.error;
  EXPECT_TRUE(polygonClosed(result.polygon));
  EXPECT_LT(result.selected_area_m2, 1.0);
}

TEST(SweptAreaRecorder, ClosedObstacleLoopExtractsInteriorHole) {
  mower_logic::area_recording::SweptAreaRecorder recorder(centeredFootprint());
  std::vector<std::vector<geometry_msgs::Pose>> segments = {{
      pose(0.0, 0.0, 0.0),
      pose(4.0, 0.0, 0.0),
      pose(4.0, 4.0, kPi / 2.0),
      pose(0.0, 4.0, kPi),
      pose(0.0, 0.0, -kPi / 2.0),
  }};

  const auto result = recorder.buildBoundary(
      segments, options(), mower_logic::area_recording::BoundarySelection::LARGEST_INTERIOR_HOLE);

  ASSERT_TRUE(result.success) << result.error;
  EXPECT_TRUE(polygonClosed(result.polygon));
  EXPECT_GT(result.selected_area_m2, 8.0);
}

TEST(SweptAreaRecorder, OpenObstacleLoopFailsCleanly) {
  mower_logic::area_recording::SweptAreaRecorder recorder(centeredFootprint());
  std::vector<std::vector<geometry_msgs::Pose>> segments = {{
      pose(0.0, 0.0, 0.0),
      pose(4.0, 0.0, 0.0),
      pose(4.0, 4.0, kPi / 2.0),
  }};

  const auto result = recorder.buildBoundary(
      segments, options(), mower_logic::area_recording::BoundarySelection::LARGEST_INTERIOR_HOLE);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error.find("interior hole"), std::string::npos);
}
