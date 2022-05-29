/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file over_time_supervisor.cpp
 *
 *@brief 监督是否超过允许执行时间，为每条任务路径设置允许的执行时间，
 *  如果超过这个时间，则向上返回错误。避免一条路径执行期间出现特殊异常而无法通知任务模块
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev.1.0
 *@data 2021-02-19
 ************************************************************************/

#include "supervisor/over_time_supervisor.hpp"
#include "path_follower.hpp"
#include <glog/logging.h>
#include <float.h>
#include "pnc_map.hpp"

#define SHORT_DIS 3.0  // 短路径长度

namespace CVTE_BABOT {

void OverTimeSupervisor::supervise(const SupervisorState &state,
                                   SupervisorResult &out) {
  // double dis = state.robot_pose_.distanceTo(target_goal_);
  // if (dis < short_dist_) {
  LOG(INFO) << "overtime get refer path remain length: "
            << state.pf_ptr_->getReferRemainLength();
  Pose2d cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  PncMap pnc_map;
  if ((state.state_ == PathFollowerStates::PAUSE ||
       state.state_ == PathFollowerStates::RECOVER) &&
      !pause_state_flag_ && !is_abs_reach_ &&
      !pnc_map.InElevatorArea(
          math_utils::Vec2d(cur_pose.getX(), cur_pose.getY()))) {
    pause_state_flag_ = true;
    pause_state_start_ = std::chrono::system_clock::now();
  } else if (state.state_ != PathFollowerStates::PAUSE &&
             state.state_ != PathFollowerStates::RECOVER) {
    pause_state_flag_ = false;
  }

  if (pause_state_flag_) {
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = now - pause_state_start_;
    LOG(INFO) << "OverTime Supervisor Pause State: " << diff.count();
    if (diff.count() > pause_wait_sec_) {
      out.can_continue = false;
      out.new_local_goal = false;
      out.target_point.position = target_goal_;
      out.target_point.velocity = WayPointInfo(0.0, 0.0, 0.0);
      out.target_point.index = path_size_ - 1;
      out.status = OVERTIME;
      out.supervisor_name = "OverTime";
    }
  }

  if (state.state_ == PathFollowerStates::ROTATE && !rotate_flag_) {
    rotate_flag_ = true;
    rotate_start_ = std::chrono::system_clock::now();
  }

  if (rotate_flag_) {
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = now - rotate_start_;
    LOG(INFO) << "OverTime Supervisor in rotate state: " << diff.count();
    if (diff.count() > rotate_wait_sec_) {
      out.can_continue = false;
      out.new_local_goal = false;
      out.target_point.position = target_goal_;
      out.target_point.velocity = WayPointInfo(0.0, 0.0, 0.0);
      out.target_point.index = path_size_ - 1;
      out.status = OVERTIME;
      out.supervisor_name = "OverTime";
    }
  }

  if (!close_target_update_flag_ &&
      state.pf_ptr_->getReferRemainLength() < short_dist_) {
    LOG(INFO) << "update decount :" << decount_sec_ << " to " << wait_sec_;
    decount_sec_ = wait_sec_;
    overtime_start_ = std::chrono::system_clock::now();
    close_target_update_flag_ = true;
  }
  // }
  if (close_target_update_flag_ && !is_abs_reach_) {
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = now - overtime_start_;
    LOG(INFO) << "OverTime Supervisor: " << diff.count();
    if (diff.count() > decount_sec_) {
      out.can_continue = false;
      out.new_local_goal = false;
      out.target_point.position = target_goal_;
      out.target_point.velocity = WayPointInfo(0.0, 0.0, 0.0);
      out.target_point.index = path_size_ - 1;
      out.status = OVERTIME;
      out.supervisor_name = "OverTime";
    }
  }
}

void OverTimeSupervisor::setSuperviseReferPath(
    const std::vector<Pose2d> &refer_path) {
  target_goal_ = refer_path.back();
  path_size_ = refer_path.size();
  close_target_update_flag_ = false;
  pause_state_flag_ = false;
  rotate_flag_ = false;

  // double sum_distance = 0.0;
  // if (refer_path.size() >= 2) {  //
  // 如果只有一个点的路径，则直接不需判断总长度
  //   for (size_t i = 1; i < refer_path.size(); ++i) {
  //     sum_distance += refer_path[i].distanceTo(refer_path[i - 1]);
  //   }
  // }

  // // 总长度SHORT_DIS米以内的路径，统一设置限时1分钟
  // if (sum_distance < short_dist_) {
  //   decount_sec_ = wait_sec_;  // 秒
  // } else {
  //   // 其它长度的，则以总长除以最低运行速度计算时长上限（Tmax = S / Vmin）
  //   // 尽量设置长一点时间是因为期间可能出现绕障和停障倒退的情况
  //   // 0.1
  //   是假设全程都以0.1米每秒的速度走完整条路径的最大时长（实际不会这么慢）
  //   // 30 * 2 是代表起点和终点位置旋转时的最大时间，认为每次旋转30秒内完成
  //   decount_sec_ = static_cast<int>(sum_distance / 0.05) + 30 * 2;
  // }

  // // 设置任务路径之后，更新到达终点附近的标志
  // close_target_update_flag_ = false;

  // LOG(INFO) << "OverTime Target: " << target_goal_.getX() << ", "
  //           << target_goal_.getY() << ", " << target_goal_.getYaw()
  //           << " (decount_sec: " << decount_sec_ << " sum dis: " <<
  //           sum_distance
  //           << ")";
}
}  // namespace CVTE_BABOT