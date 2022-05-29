/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file final_goal_supervisor.hpp
 *
 *@brief 监督是否到达最终目标点
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev.1.0
 *@data 2021-02-19
 ************************************************************************/

#ifndef __FINAL_GOAL_SUPERVISOR_HPP
#define __FINAL_GOAL_SUPERVISOR_HPP

#include "supervisor.hpp"

namespace CVTE_BABOT {

class FinalGoalSupervisor : public Supervisor {
 public:
  FinalGoalSupervisor() = default;

  std::string getName() const override { return "FinalGoal"; }

  void supervise(const SupervisorState &state, SupervisorResult &out) override;

  void setSuperviseReferPath(const std::vector<Pose2d> &refer_path);

  inline void setGoalXYTolerance(const double &xy_tolerance) {
    xy_tolerance_ = xy_tolerance;
  }
  // 优化停靠精度
  inline void setTargetPose(const Pose2d &target_pose) {
    target_goal_ = target_pose;
    LOG(INFO) << "FinalGoalSupervisor target_pose"
              << " x:" << target_pose.getX() << " y:" << target_pose.getY()
              << " yaw:" << target_pose.getYaw();
  }

 private:
  Pose2d target_goal_;
  bool need_check_index_;
  double xy_tolerance_ = 0.1;
  std::vector<Pose2d> refer_path_;
  size_t path_size_ = 0;
};

}  // namespace CVTE_BABOT

#endif  // end of __FINAL_GOAL_SUPERVISOR_HPP