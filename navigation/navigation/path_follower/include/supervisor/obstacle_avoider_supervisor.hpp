/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file obstacle_avoider_supervisor.hpp
 *
 *@brief
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2020-01-06
 ************************************************************************/

#ifndef __OBSTACLE_AVOIDER_SUPERVISOR_HPP
#define __OBSTACLE_AVOIDER_SUPERVISOR_HPP

#include "supervisor.hpp"

namespace CVTE_BABOT {
class ObstacleAvoiderSupervisor : public Supervisor {
 public:
  ObstacleAvoiderSupervisor(double obstacle_distance_front,
                            double obstacle_distance_behind,
                            double dangerous_value);

  std::string getName() const override { return "ObstacleAvoider"; }

  // void setFootprint(const std::vector<Eigen::Vector2d> &foot_print);

  void supervise(const SupervisorState &state, SupervisorResult &out) override;

 private:
  // enum FootPrintArea { Positive = 0, Negative = 1 };

  int checkPointCostValue(const Pose2d &check_point,
                          std::shared_ptr<Costmap2d> ptr_costmap_2d);
  // bool findLastPointAsTarget(const std::vector<Pose2d> &path,
  //                            const int &current_index);
  // int calcMinestDisPathIndex(std::shared_ptr<SubPath> path,
  //                            const Pose2d &robot_pose, const size_t &start,
  //                            const size_t &end);

  double obstacle_distance_front_ = 4.0;  // 触发绕障距离
  // double obstacle_distance_behind_ = 2.0;  // 绕障目标点在障碍物后方距离
  int dangerous_value_ = 155;  // 判断发生碰撞的代价值
  // std::vector<Eigen::Vector2d> foot_print_;
  // size_t findAvoidStartPoint(const std::vector<Pose2d> &path,
  //                            std::shared_ptr<Costmap2d> ptr_costmap_2d,
  //                            size_t start_index, size_t crash_index);
};
}  // namespace CVTE_BABOT

#endif