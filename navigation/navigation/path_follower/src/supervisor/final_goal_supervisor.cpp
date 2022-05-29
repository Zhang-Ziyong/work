/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file final_goal_supervisor.cpp
 *
 *@brief 监督是否到达最后目标点
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev.1.0
 *@data 2021-02-19
 ************************************************************************/

#include "supervisor/final_goal_supervisor.hpp"
#include "path_follower.hpp"
#include <glog/logging.h>
#include <float.h>

namespace CVTE_BABOT {

void FinalGoalSupervisor::supervise(const SupervisorState &state,
                                    SupervisorResult &out) {
  // if (state.state_ == PathFollowerStates::FOLLOW_REFERENCE) {
  // 优化停靠精度 调试用
  // double dis_actual = state.robot_pose_.distanceTo(target_goal_);

  // double dX = target_goal_.getX() - state.robot_pose_.getX();
  // double dY = target_goal_.getY() - state.robot_pose_.getY();

  // double dXe = dX * cos(state.robot_pose_.getYaw()) +
  //              dY * sin(state.robot_pose_.getYaw());
  // double dYe = (-1.0) * dX * sin(state.robot_pose_.getYaw()) +
  //              dY * cos(state.robot_pose_.getYaw());

  // LOG(INFO) << "FinalGoalSupervisor: " << dis_actual
  //           << "\n current_pose_x:" << state.robot_pose_.getX()
  //           << "current_pose_y:" << state.robot_pose_.getY()
  //           << "current_pose_yaw:" << state.robot_pose_.getYaw()
  //           << "\n target_goal_x:" << target_goal_.getX()
  //           << "target_goal_y:" << target_goal_.getY()
  //           << "target_goal_yaw:" << target_goal_.getYaw()
  //           << "\n dXe: " << dXe << "dYe: " << dYe;

  // 优化停靠精度

  double dist = state.pf_ptr_->getReferRemainLength();
  LOG(INFO) << "refer path remain length: " << dist;
  if (dist <= xy_tolerance_) {
    out.can_continue = false;
    out.new_local_goal = false;
    out.target_point.position = target_goal_;
    out.target_point.velocity = WayPointInfo(0.0, 0.0, 0.0);
    out.target_point.index = path_size_ - 1;
    out.status = FINALGOAL;
    out.supervisor_name = "FinalGoal";
    LOG(INFO) << "final goal";
  }
  // }
  // double dis = state.robot_pose_.distanceTo(target_goal_);
  // int index = state.pf_ptr_->getPmPtr()->getReferIndex();
  // LOG(INFO) << "FinalGoalSupervisor: " << dis
  //           << "  check_index_: " << need_check_index_;
  // LOG(INFO) << "refer index: " << index << " total size: " << path_size_;
  // if (path_size_ - index <= 2) {
  //   out.can_continue = false;
  //   out.new_local_goal = false;
  //   out.target_point.position = target_goal_;
  //   out.target_point.velocity = WayPointInfo(0.0, 0.0, 0.0);
  //   out.target_point.index = path_size_ - 1;
  //   out.status = FINALGOAL;
  //   out.supervisor_name = "FinalGoal";
  //   LOG(INFO) << "path size final goal";
  // }
  // double sum_dist = 0;
  // Pose2d path_point = refer_path_[index];
  // for (; index < path_size_; index++) {
  //   Eigen::Vector2d dist_vec(refer_path_[index].getX() - path_point.getX(),
  //                            refer_path_[index].getY() - path_point.getY());
  //   sum_dist += dist_vec.norm();
  //   path_point = refer_path_[index];
  //   if (sum_dist > xy_tolerance_) {
  //     break;
  //   }
  // }
  // if (sum_dist <= xy_tolerance_) {
  //   out.can_continue = false;
  //   out.new_local_goal = false;
  //   out.target_point.position = target_goal_;
  //   out.target_point.velocity = WayPointInfo(0.0, 0.0, 0.0);
  //   out.target_point.index = path_size_ - 1;
  //   out.status = FINALGOAL;
  //   out.supervisor_name = "FinalGoal";
  //   LOG(INFO) << "sum dist final goal";
  // }
}

void FinalGoalSupervisor::setSuperviseReferPath(
    const std::vector<Pose2d> &refer_path) {
  // refer_path_ = refer_path;
  double sum_distance = 0.0;
  if (refer_path.size() >= 2) {  // 如果只有一个点的路径，则直接不需判断总长度
    for (size_t i = 1; i < refer_path.size(); ++i) {
      sum_distance += refer_path[i].distanceTo(refer_path[i - 1]);
    }
  }
  target_goal_ = refer_path.back();
  Pose2d path_begin = refer_path[0];
  Eigen::Vector2d dist_vec(path_begin.getX() - target_goal_.getX(),
                           path_begin.getY() - target_goal_.getY());
  // need_check_index_ = dist_vec.norm() < 2 * xy_tolerance_ ? true : false;
  need_check_index_ = true;
  LOG(INFO) << "Refer begin: " << path_begin.getX() << ", " << path_begin.getY()
            << ", " << path_begin.getYaw();
  LOG(INFO) << "FinalGoal Target: " << target_goal_.getX() << ", "
            << target_goal_.getY() << ", " << target_goal_.getYaw();
  path_size_ = refer_path.size();
  // 如果任务路径总长度大于3米，到达最终点的判断条件需要考虑索引，针对回环任务路径的情况（否则一下发就到达）
  // need_check_index_ = sum_distance > 3.0 ? true : false;
  // LOG(INFO) << "FinalGoal Target: " << target_goal_.getX() << ", "
  //           << target_goal_.getY() << ", " << target_goal_.getYaw()
  //           << " (size: " << path_size_ << " sum dis: " << sum_distance <<
  //           ")";
}
}  // namespace CVTE_BABOT