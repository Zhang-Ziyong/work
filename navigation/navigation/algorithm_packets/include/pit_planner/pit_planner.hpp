/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file pit_planner.hpp
 *
 *@brief 过坎规划算法的头文件
 *
 *@modified by
 *
 *@author lizhongjia(lizhongjia@cvte.com)
 *@version Navigation-v2.0
 *@data 2022-04-18
 ************************************************************************/
#ifndef PIT_PLANNER_HPP_
#define PIT_PLANNER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <time.h>
#include <vector>

#include "costmap_2d.hpp"
#include "pose2d/pose2d.hpp"

namespace CVTE_BABOT {

struct PitScore {
  int index;
  double grade;
};

class PitPlanner {
 public:
  PitPlanner(double edge_dist, double access_interval)
      : edge_dist_(edge_dist), access_interval_(access_interval) {}
  PitPlanner(const PitPlanner &obj) = delete;
  PitPlanner &operator=(const PitPlanner &obj) = delete;

  bool GetAllAccessPath(std::vector<Eigen::Vector2d> &start_points,
                        std::vector<Eigen::Vector2d> &end_points);

  bool GetOptimalAccessPoint(Eigen::Vector2d &start_point,
                             Eigen::Vector2d &end_point);

  void SetCostMap(const std::shared_ptr<Costmap2d> &costmap);

  void SetWayPoint(const Pose2d &pit_start, const Pose2d &pit_end);

  void SetBox(Eigen::Vector2d box_center, double length, double width,
              double heading);

 private:
  void buildConversionMatrix(void);

  void optimalPathIndex(std::vector<PitScore> &scores, int &index);

  bool pathScore(const double dangerous_cost, size_t path_num,
                 std::vector<PitScore> &scores);

 private:
  double length_ = 0;
  double width_ = 0;
  double heading_ = 0;
  double edge_dist_ = 0;
  double access_interval_ = 0;
  Eigen::Matrix<double, 2, 3> T_wb_;
  Eigen::Vector2d box_center_;
  Eigen::Vector2d pit_start_;
  Eigen::Vector2d pit_end_;
  std::shared_ptr<Costmap2d> ptr_costmap_;
  std::vector<Eigen::Vector2d> start_points_;
  std::vector<Eigen::Vector2d> end_points_;
};

}  // namespace CVTE_BABOT
#endif
