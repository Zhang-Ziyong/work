/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file lypu_planner.cpp
 *
 *@brief LYPU 跟踪算法的实现
 *
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-07-16
 ************************************************************************/

#include "lypu/lypu_planner.hpp"
#include <float.h>
#include <glog/logging.h>
#include <algorithm>
#include <mutex>

#define LOG_HZ 20

namespace CVTE_BABOT {
LYPULocalController::LYPULocalController() {
  ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
  init_flag_ = false;
  getParams();
}

void LYPULocalController::getParams() {
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }

  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.dKx", dKx_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.dKp", dKp_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.dKy", dKy_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.dKq", dKq_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.dKd", dKd_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.dKth", dKth_, 1.0);

  // ptr_navigation_mediator_->getParam(
  //     "path_follower.controller_algorithm_params.lypu.max_target_v",
  //     max_target_v_, 1.0);
  // ptr_navigation_mediator_->getParam(
  //     "path_follower.controller_algorithm_params.lypu.max_target_w",
  //     max_target_w_, 1.0);
  // ptr_navigation_mediator_->getParam(
  //     "path_follower.controller_algorithm_params.lypu.min_target_v",
  //     min_target_v_, 0.0);
  // ptr_navigation_mediator_->getParam(
  //     "path_follower.controller_algorithm_params.lypu.min_target_w",
  //     min_target_w_, 0.0);

  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.max_x", max_v_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.max_th", max_w_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.min_x", min_v_, 0.05);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.min_th", min_w_, 0.05);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.acc", acc_, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.lypu.a_acc", a_acc_, 1.0);
  LOG(INFO) << "read lypu params: ";
  LOG(INFO) << "dKx_: " << dKx_ << " dKy: " << dKy_ << " dKq: " << dKq_
            << " dKd: " << dKd_ << " dKp_: " << dKp_;
  // LOG(INFO) << "max_target_v: " << max_target_v_
  //           << " max_target_w: " << max_target_w_
  //           << " min_target_v: " << min_target_v_
  //           << " min_target_w_: " << min_target_w_;
  LOG(INFO) << "max_v: " << max_v_ << " max_w:" << max_w_
            << " min_v: " << min_v_ << " min_w: " << min_w_ << " acc: " << acc_
            << " a_acc: " << a_acc_;
  setMoveSpeedRange(0.0, max_v_, 0.0, 0.0, 0.0, max_w_);
}

MoveCommand::MoveCommandStatus LYPULocalController::computeMoveComand(
    MoveCommand &cmd) {
  if (local_path_ == nullptr || local_path_->wps.empty()) {
    LOG(INFO) << "MoveCommand::MoveCommandStatus::ERROR1" << std::endl;
    return MoveCommand::MoveCommandStatus::ERROR;
  }
  getMaxSpeedLimit(
      max_v_, max_y_,
      max_w_);  // 更新当前允许的最速度值（不同速度等级可能需要对最大值调整）
  Pose2d current_pose = getCurrentPose();
  TrackCtrl result;
  size_t find_index = 0;
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (findTrackTarget(current_pose, find_index)) {
    // 1.计算差量
    next_target_ = local_path_->wps[find_index];
    // lypu闭环
    trackingLYPU(next_target_, current_pose, local_path_->wpis[find_index].v,
                 local_path_->wpis[find_index].w, result);
    LOG_EVERY_N(INFO, LOG_HZ)
        << "Target pose: " << next_target_.getX() << ", " << next_target_.getY()
        << ", " << next_target_.getYaw();
    LOG_EVERY_N(INFO, LOG_HZ)
        << "Calc:  V = " << result.v_velocity << "  W = " << result.w_velocity;
    cmd.setVelX(result.v_velocity);
    cmd.setVelY(0);
    cmd.setVelTH(result.w_velocity);
    cmd.setCarrotPose(next_target_);

    return MoveCommand::MoveCommandStatus::OKAY;
  } else {
    // //当找到最后一个点时
    if (local_path_->wps.size() - 1 == find_index) {
      Pose2d target_pose = local_path_->wps[find_index];
      double dX = target_pose.getX() - current_pose.getX();
      double dY = target_pose.getY() - current_pose.getY();
      double target_yaw = atan2(dY, dX);
      double dQ = AngleCalculate::angle_diff(target_yaw, current_pose.getYaw());
      result.v_velocity = dKp_ * sqrt(dX * dX + dY * dY) * cos(dQ);
      result.w_velocity = dKth_ * (dQ);
      checkMaxVel(result.v_velocity, result.w_velocity);
      checkMinVel(result.v_velocity, result.w_velocity);
      cmd.setVelX(result.v_velocity);
      cmd.setVelY(0);
      cmd.setVelTH(result.w_velocity);
      cmd.setCarrotPose(target_pose);
      return MoveCommand::MoveCommandStatus::OKAY;
    }
  }

  LOG(INFO) << "MoveCommand::MoveCommandStatus::ERROR2" << std::endl;
  return MoveCommand::MoveCommandStatus::ERROR;
}
double LYPULocalController::getRemainCtrlPathLength() {
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (local_path_ == nullptr) {
    return 0.0;
  }
  double dist = local_path_->wpis.back().path_length -
                local_path_->wpis[last_target_index_].path_length;
  LOG(INFO) << "get remain ctrl path length: " << dist;
  return dist;
}

size_t LYPULocalController::getReferIndex() {
  return last_target_index_;
}

// 不计算速度, 不插值
bool LYPULocalController::setPath(std::shared_ptr<SubPath> path,
                                  size_t begin_index) {
  LOG(INFO) << "******************* update path ********************";
  if (path == nullptr || path->wps.size() == 0) {
    LOG(ERROR) << "No sub Paths.";
    return false;
  }

  // 直接获取路径集合中的当前路径，外面根据情况做逻辑判断挑选需要执行的路径
  std::lock_guard<std::mutex> lock(path_mutex_);
  local_path_ = path;
  if (local_path_->wps.size() <= 0) {
    LOG(ERROR) << "Lypu Received an Empty Target Plan.";
    return false;
  }
  if (local_path_->wps.size() != local_path_->wpis.size()) {
    LOG(ERROR) << "Lypu Received an Plan whitout path info.";
    return false;
  }
  LOG(INFO) << "set subPath size = " << local_path_->wps.size() << ", "
            << local_path_->wpis.size();

  last_target_index_ = begin_index;
  return true;
}

bool LYPULocalController::findTrackTarget(const Pose2d &c_pose,
                                          size_t &path_point_index) {
  if (local_path_ == nullptr || local_path_->wps.empty()) {
    LOG(ERROR) << "TrackPath is empty.";
    return false;
  }

  if (local_path_->wps.size() == 1) {
    path_point_index = 0;
    LOG(INFO) << "path size 1 = " << local_path_->wps.size();
    // return false;
    return true;
  }
  // 查找路径上离机器人最近的点
  size_t min_index = last_target_index_;
  double distance = 0.0;
  double min_distance = DBL_MAX;
  // double max_inc_index = 100;
  double sum_dist = 1.0;
  for (size_t i = last_target_index_;
       i < local_path_->wps.size() - 1 &&
       local_path_->wpis[i].path_length -
               local_path_->wpis[last_target_index_].path_length <
           sum_dist;
       i++) {
    distance = c_pose.distanceTo(local_path_->wps[i]);
    if (min_distance > distance) {
      min_distance = distance;
      min_index = i;
    }
  }

  last_target_index_ = min_index;
  LOG_EVERY_N(INFO, LOG_HZ)
      << "Minest_distance = " << min_distance << "  min_index = " << min_index;

  // TODO: 胡萝卜点在当前位置后面时将导致机器人不移动
  double sum = 0;
  for (size_t i = min_index; i < local_path_->wps.size() - 1; i++) {
    sum += local_path_->wps[i + 1].distanceTo(local_path_->wps[i]);
    if (sum > 0.1) {
      path_point_index = i;
      LOG(INFO) << "find targer point: " << path_point_index;
      return true;
    }
  }
  path_point_index = local_path_->wps.size() - 1;
  LOG(INFO) << " find the last point as target.";
  return true;
}

bool LYPULocalController::trackingLYPU(const Pose2d &traget_pos,
                                       const Pose2d &current_pos,
                                       const double &target_v,
                                       const double &target_w,
                                       TrackCtrl &result) {
  // 1.计算差量
  double dX = traget_pos.getX() - current_pos.getX();
  double dY = traget_pos.getY() - current_pos.getY();
  double dQ =
      AngleCalculate::angle_diff(traget_pos.getYaw(), current_pos.getYaw());
  LOG_EVERY_N(INFO, LOG_HZ)
      << "dX = " << dX << " dY = " << dY << " dQ = " << dQ;
  LOG_EVERY_N(INFO, LOG_HZ)
      << "target_v: " << target_v << "   target_w: " << target_w;

  double dXe = dX * cos(current_pos.getYaw()) + dY * sin(current_pos.getYaw());
  double dYe =
      (-1.0) * dX * sin(current_pos.getYaw()) + dY * cos(current_pos.getYaw());

  LOG_EVERY_N(INFO, LOG_HZ) << "dXe = " << dXe << " dYe = " << dYe;

  // 用于复位一次上次记录的夹角差，dQ值一般较小
  if (fabs(last_dQ_) > M_PI / 2 || init_flag_ == true) {
    init_flag_ = false;
    last_dQ_ = dQ;
  }

  double ddQ = dQ - last_dQ_;
  last_dQ_ = dQ;
  LOG_EVERY_N(INFO, LOG_HZ) << "ddQ = " << ddQ;

  // 2.计算输出
  if (target_v > 0) {
    dXe = fabs(dXe);
  } else {
    dXe = -fabs(dXe);
  }
  double limit_w = target_w;
  double max_target_w = 0.8;
  if (target_w > max_target_w_) {
    limit_w = max_target_w_;
  } else if (target_w < -max_target_w_) {
    limit_w = -max_target_w_;
  }
  //     v          = v_r     *  cos\sigma + K_x * x_e
  result.v_velocity = target_v * cos(dQ) + dKx_ * dXe;
  //     w          = k_q  * \sigma  +   路径上的前向反馈  +
  result.w_velocity = dKq_ * dQ + limit_w +
                      //                  v_r      * (K_y *y_c   + K_{\sigma} *
                      //                  sin(\simgma_c)) + 一个D项
                      target_v * (dKy_ * dYe + dKq_ * sin(dQ)) + dKd_ * ddQ;
  //不能计算出与期望速度相反的控制速度
  if (target_v > 0 && result.v_velocity < 0) {
    result.v_velocity = 0.001;
  }
  if (target_v < 0 && result.v_velocity > 0) {
    result.v_velocity = -0.001;
  }

  result.msg = "ok";
  // checkMaxVel(result.v_velocity, result.w_velocity);
  checkMinVel(result.v_velocity, result.w_velocity);
  return true;
}

void LYPULocalController::checkMaxVel(double &v_vel, double &w_vel) {
  LOG_EVERY_N(INFO, LOG_HZ) << "org: v = " << v_vel << "  w = " << w_vel;
  double multi_vel = 1.0;
  double multi_w = 1.0;
  if (v_vel > max_v_) {
    multi_vel = max_v_ / v_vel;
    v_vel = max_v_;
  } else if (v_vel < -max_v_) {
    multi_vel = -max_v_ / v_vel;
    v_vel = -max_v_;
  }
  w_vel = multi_vel * w_vel;
  if (std::fabs(w_vel) > max_w_) {
    if (w_vel > 0) {
      multi_w = max_w_ / w_vel;
    } else {
      multi_w = -max_w_ / w_vel;
    }
    w_vel = w_vel > 0 ? max_w_ : -max_w_;
  }
  v_vel = multi_w * v_vel;
  LOG_EVERY_N(INFO, LOG_HZ) << "Check max: v = " << v_vel << "  w = " << w_vel;
}

void LYPULocalController::checkMinVel(double &v_vel, double &w_vel) {
  bool adjust_min_v =
      false;  // 调整线速度标志位，只有线速度被最小值调整时再考虑w是否需要被修改
  if (std::fabs(v_vel) < min_v_) {
    v_vel = v_vel > 0 ? min_v_ : -min_v_;
    adjust_min_v = true;
  }
  if (adjust_min_v && std::fabs(w_vel) < min_w_ &&
      std::fabs(w_vel) > 0.2 * min_w_) {
    w_vel = w_vel > 0 ? min_w_ : -min_w_;
  }
  LOG_EVERY_N(INFO, LOG_HZ) << "Check min: v = " << v_vel << "  w = " << w_vel;
}
void LYPULocalController::clearCache() {
  init_flag_ = true;
}

}  // namespace CVTE_BABOT
