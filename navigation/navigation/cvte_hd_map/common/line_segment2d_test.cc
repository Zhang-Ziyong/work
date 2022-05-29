/**
 * @file line_segment2d_test.cc
 * @author linyanlong (linyanlong@cvte.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "line_segment2d.h"

#include <cmath>

#include "gtest/gtest.h"

namespace cvte {
namespace hdmap {

TEST(LineSegment2dTest, Accessors) {
  const LineSegment2d ls({1, 2}, {5, 4});
  EXPECT_NEAR(ls.length(), std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.length_sqr(), 20.0, 1e-5);
  EXPECT_NEAR(ls.center().x(), 3, 1e-5);
  EXPECT_NEAR(ls.center().y(), 3, 1e-5);
  EXPECT_NEAR(ls.heading(), std::atan2(2, 4), 1e-5);
  EXPECT_NEAR(ls.cos_heading(), 4.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.sin_heading(), 2.0 / std::sqrt(20.0), 1e-5);
}

TEST(LineSegment2dTest, DistanceTo) {
  const LineSegment2d ls({1, 2}, {5, 4});
  Vec2d nearest_pt;
  EXPECT_NEAR(ls.DistanceTo({0, 0}, &nearest_pt), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(ls.DistanceTo({0, 0}), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(ls.DistanceSquareTo({0, 0}), 5.0, 1e-5);
  EXPECT_NEAR(ls.DistanceSquareTo({0, 0}, &nearest_pt), 5.0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({10, 10}, &nearest_pt), std::sqrt(61.0), 1e-5);
  EXPECT_NEAR(ls.DistanceTo({1, 2}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({5, 4}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({3, 3}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({4, 4}, &nearest_pt), 2.0 / std::sqrt(20.0), 1e-5);
}

TEST(LineSegment2dTest, GetPerpendicularFoot) {
  const LineSegment2d ls({1, 2}, {5, 4});
  Vec2d foot_pt;
  EXPECT_NEAR(ls.GetPerpendicularFoot({0, 0}, &foot_pt), 0.6 * std::sqrt(5.0),
              1e-5);
  EXPECT_NEAR(foot_pt.x(), -0.6, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 1.2, 1e-5);
  EXPECT_NEAR(ls.GetPerpendicularFoot({3, 3}, &foot_pt), 0.0, 1e-5);
  EXPECT_NEAR(foot_pt.x(), 3.0, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 3.0, 1e-5);
}

TEST(LineSegment2dTest, ProjectOntoUnit) {
  const LineSegment2d ls({1, 2}, {5, 4});
  EXPECT_NEAR(ls.ProjectOntoUnit({1, 2}), 0.0, 1e-5);
  EXPECT_NEAR(ls.ProjectOntoUnit({5, 4}), std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.ProjectOntoUnit({-2, -3}), -22.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.ProjectOntoUnit({6, 7}), 30.0 / std::sqrt(20.0), 1e-5);
}

TEST(LineSegment2dTest, GetIntersect) {
  const LineSegment2d ls({1, 2}, {5, 4});
  Vec2d point;
  EXPECT_FALSE(ls.GetIntersect({{1, 3}, {5, 5}}, &point));
  EXPECT_FALSE(ls.GetIntersect({{2, 2}, {6, 4}}, &point));

  EXPECT_TRUE(ls.GetIntersect({{1, 2}, {-3, 0}}, &point));
  EXPECT_NEAR(point.x(), 1, 1e-5);
  EXPECT_NEAR(point.y(), 2, 1e-5);
  EXPECT_TRUE(ls.GetIntersect({{5, 4}, {9, 6}}, &point));
  EXPECT_NEAR(point.x(), 5, 1e-5);
  EXPECT_NEAR(point.y(), 4, 1e-5);

  EXPECT_TRUE(ls.GetIntersect({{3, 0}, {3, 10}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_TRUE(ls.GetIntersect({{3, 10}, {3, 0}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_FALSE(ls.GetIntersect({{3, 5}, {3, 10}}, &point));
  EXPECT_FALSE(ls.GetIntersect({{3, 2}, {3, 0}}, &point));
  EXPECT_TRUE(ls.GetIntersect({{3, 3}, {3, 3}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_FALSE(ls.GetIntersect({{4, 4}, {4, 4}}, &point));
}

TEST(LineSegment2dTest, IsPointIn) {
  const LineSegment2d ls({1, 2}, {5, 4});
  EXPECT_TRUE(ls.IsPointIn({1, 2}));
  EXPECT_TRUE(ls.IsPointIn({5, 4}));
  EXPECT_TRUE(ls.IsPointIn({3, 3}));
  EXPECT_FALSE(ls.IsPointIn({-1, 1}));
  EXPECT_FALSE(ls.IsPointIn({7, 5}));
  EXPECT_FALSE(ls.IsPointIn({0, 0}));
  EXPECT_FALSE(ls.IsPointIn({6, 6}));
}

}  // namespace CVTE_BABOT
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
