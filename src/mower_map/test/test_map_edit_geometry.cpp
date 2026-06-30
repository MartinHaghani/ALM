#include <gtest/gtest.h>

#include "geometry_msgs/Point32.h"
#include "mower_map/map_edit_geometry.h"

namespace {

geometry_msgs::Point32 point(double x, double y) {
  geometry_msgs::Point32 p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

geometry_msgs::Polygon polygon(std::initializer_list<std::pair<double, double>> points) {
  geometry_msgs::Polygon result;
  for (const auto& [x, y] : points) {
    result.points.push_back(point(x, y));
  }
  result.points.push_back(result.points.front());
  return result;
}

geometry_msgs::Polygon stroke(std::initializer_list<std::pair<double, double>> points) {
  geometry_msgs::Polygon result;
  for (const auto& [x, y] : points) {
    result.points.push_back(point(x, y));
  }
  return result;
}

}  // namespace

TEST(MapEditGeometry, BrushAddRequiresStrokeToTouchTarget) {
  const auto target = polygon({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
  const auto edit = mower_map::edit_geometry::applyBrushAdd(target, stroke({{1.8, 1.0}, {2.8, 1.0}}), 0.6);

  ASSERT_TRUE(edit.success) << edit.error;
  EXPECT_GE(edit.outline.points.size(), 4u);

  const auto disconnected = mower_map::edit_geometry::applyBrushAdd(target, stroke({{5, 5}, {6, 5}}), 0.5);
  EXPECT_FALSE(disconnected.success);
}

TEST(MapEditGeometry, BoundaryEraseReshapesOutline) {
  const auto target = polygon({{0, 0}, {4, 0}, {4, 4}, {0, 4}});
  const auto edit = mower_map::edit_geometry::applyBrushErase(target, stroke({{3.8, 1.0}, {3.8, 3.0}}), 0.8);

  ASSERT_TRUE(edit.success) << edit.error;
  EXPECT_TRUE(edit.obstacles.empty());
  EXPECT_GE(edit.outline.points.size(), 4u);
}

TEST(MapEditGeometry, InteriorEraseCreatesObstacle) {
  const auto target = polygon({{0, 0}, {6, 0}, {6, 6}, {0, 6}});
  const auto edit = mower_map::edit_geometry::applyBrushErase(target, stroke({{3, 3}}), 1.0);

  ASSERT_TRUE(edit.success) << edit.error;
  EXPECT_EQ(edit.obstacles.size(), 1u);
}

TEST(MapEditGeometry, ReplacementRejectsInvalidPolygon) {
  geometry_msgs::Polygon output;
  std::string error;
  EXPECT_FALSE(mower_map::edit_geometry::normalizeReplacementPolygon(stroke({{0, 0}, {1, 1}}), output, error));
  EXPECT_FALSE(error.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
