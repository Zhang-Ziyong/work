/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file test_path_recovery.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.7
 *@data 2020-07-10
 ************************************************************************/
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "costmap_2d.hpp"
#include "recovery/path_recovery.hpp"

namespace CVTE_BABOT {

void setOccupy() {}

double toRad(const double &angle) { return angle / 180.000 * M_PI; }

double toDegree(const double &angle) { return angle / M_PI * 180.000; }

class PathRecoveryTester : public testing::Test {
 public:
  virtual void TestBody() {}
  PathRecoveryTester() {
    std::vector<WorldmapPoint> footprint;
    footprint.push_back({-0.30, 0.35});
    footprint.push_back({0.90, 0.35});
    footprint.push_back({0.90, -0.35});
    footprint.push_back({-0.30, -0.35});
    path_recovery_.ptr_obstacle_map_ =
        std::make_shared<ObstacleMap>(footprint, 0.1);
    ptr_costmap_ =
        std::make_shared<Costmap2d>(200, 200, 0.1, -10.00, -10.00, FREE_SPACE);
    Pose2d current_pose(0.0, 0.0, 0.0);
    path_recovery_.updateState(ptr_costmap_, current_pose);
  }

  void rotateRecoveryTest() {
    WorldmapPoint wp_point(0.5, 0.3);
    CostmapPoint cp_point;
    EXPECT_TRUE(ptr_costmap_->worldToMap(wp_point, cp_point));
    ptr_costmap_->setCost(cp_point.ui_x, cp_point.ui_y, LETHAL_OBSTACLE);

    Pose2d current_pose(0.0, 0.0, 0.0);

    path_recovery_.updateState(ptr_costmap_, current_pose);
  }

  void computeAngleError() {
    // 判断范围在[-pi,pi]
    for (double i = M_PI; i > -M_PI; i -= 0.05) {
      for (double j = M_PI; j > -M_PI; j -= 0.05) {
        double angle_error = path_recovery_.computeAngleError(i, j);
        EXPECT_GT(angle_error, -M_PI);
        EXPECT_LT(angle_error, M_PI);
      }
    }

    // 检查i在(0,pi]
    for (double i = M_PI; i > 0; i -= 0.05) {
      // i与j差值的绝对值大于pi时的结果
      for (double j = -M_PI + i - 0.001; j > -M_PI; j -= 0.05) {
        double angle_error = path_recovery_.computeAngleError(i, j);
        EXPECT_GT(angle_error, 0.0);
      }

      // i与j差值的绝对值小于pi时的结果
      for (double j = -M_PI + i + 0.001; j < 0; j += 0.05) {
        double angle_error = path_recovery_.computeAngleError(i, j);
        EXPECT_LT(angle_error, 0.0);
      }
    }

    // 检查i在(-pi,0)
    for (double i = 0; i > -M_PI; i -= 0.05) {
      // i与j差值的绝对值大于pi时的结果
      for (double j = M_PI + i + 0.001; j < M_PI; j += 0.05) {
        double angle_error = path_recovery_.computeAngleError(i, j);
        EXPECT_LT(angle_error, 0.0);
      }

      // i与j差值的绝对值小于pi时的结果
      for (double j = M_PI + i - 0.001; j > 0; j -= 0.05) {
        double angle_error = path_recovery_.computeAngleError(i, j);
        EXPECT_GT(angle_error, 0.0);
      }
    }

    double angle_error = path_recovery_.computeAngleError(0, -M_PI);
    EXPECT_LT(angle_error, 0.0);

    angle_error = path_recovery_.computeAngleError(0, M_PI);
    EXPECT_GT(angle_error, 0.0);
  }

 private:
  PathRecovery path_recovery_;
  std::shared_ptr<Costmap2d> ptr_costmap_;
};  // namespace CVTE_BABOT

TEST(PathRecoveryTester, rotateRecoveryTest) {
  PathRecoveryTester path_recovery_tester;
  path_recovery_tester.rotateRecoveryTest();
}

TEST(PathRecoveryTester, computeAngleErrorTest) {
  PathRecoveryTester path_recovery_tester;
  path_recovery_tester.computeAngleError();
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
