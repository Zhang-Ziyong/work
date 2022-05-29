/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath:
 /src/navigation/navigation/local_controller/src/mpc/mpc_controller.cpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2021-01-12 10:51:10
 ************************************************************************/

#include "mpc/mpc_controller.hpp"
#include <glog/logging.h>

#include <algorithm>
#include <mutex>

namespace CVTE_BABOT {
MPCLocalController::MPCLocalController() {
  ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
  initParams();
  getParams();
}

void MPCLocalController::initParams() {
  matrix_q_prams_ = {10., 50., 100.};
  matrix_r_prams_ = {5, 5};
  matrix_q_ = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  matrix_r_ = Eigen::MatrixXd::Zero(controls_dim_, controls_dim_);
  matrix_initial_x_ = Eigen::MatrixXd::Zero(state_dim_, 1);
  matrix_x_lower_ = Eigen::MatrixXd::Zero(state_dim_, 1);
  matrix_x_upper_ = Eigen::MatrixXd::Zero(state_dim_, 1);
  matrix_u_lower_ = Eigen::MatrixXd::Zero(controls_dim_, 1);
  matrix_u_upper_ = Eigen::MatrixXd::Zero(controls_dim_, 1);
  matrix_x_lower_ << -1e6, -1e6, -M_PI;
  matrix_x_upper_ << 1e6, 1e6, M_PI;
  matrix_u_lower_ << -1, -1.;
  matrix_u_upper_ << 1.5, 1.;
  for (unsigned i = 0; i < state_dim_; ++i) {
    matrix_q_(i, i) = matrix_q_prams_.at(i);
  }
  for (unsigned i = 0; i < controls_dim_; ++i) {
    matrix_r_(i, i) = matrix_r_prams_.at(i);
  }
}

void MPCLocalController::getParams() {
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }
  double xy_tolerance = 0.15;
  double yaw_tolerance = 0.15;
  std::vector<double> matrix_q_prams{10., 50., 100.};
  std::vector<double> matrix_r_prams{5., 5.};
  std::vector<double> u_bound_prams{1., 0.7};
  ptr_navigation_mediator_->getParam("controller_frequency",
                                     controller_frequency_, 10.);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.mpc.horizon", horizon_, 15);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.mpc.matrix_q_prams",
      matrix_q_prams_, matrix_q_prams);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.mpc.matrix_r_prams",
      matrix_r_prams_, matrix_r_prams);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.mpc.u_bound_prams",
      u_bound_prams_, u_bound_prams);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.mpc.kappa_limit_cofficient",
      kappa_limit_cofficient_, 0.2);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.mpc.min_velocity",
      min_velocity_, 0.2);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.xy_goal_tolerance",
      xy_tolerance, 0.2);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.yaw_goal_tolerance",
      yaw_tolerance, 0.2);
  limits_->setXYTolerance(xy_tolerance);
  limits_->setYawTolerance(yaw_tolerance);

  matrix_u_lower_ << -u_bound_prams_[0], -u_bound_prams_[1];
  matrix_u_upper_ << u_bound_prams_[0], u_bound_prams_[1];
  for (unsigned i = 0; i < state_dim_; ++i) {
    matrix_q_(i, i) = matrix_q_prams_.at(i);
  }
  LOG(WARNING) << "matrix Q:\n" << matrix_q_;
  for (unsigned i = 0; i < controls_dim_; ++i) {
    matrix_r_(i, i) = matrix_r_prams_.at(i);
  }
  LOG(WARNING) << "matrix R:\n" << matrix_r_;
  LOG(WARNING) << "x_min x_max:\n"
               << matrix_x_lower_.transpose() << " "
               << matrix_x_upper_.transpose();
  LOG(WARNING) << "u_min u_max:\n"
               << matrix_u_lower_.transpose() << " "
               << matrix_u_upper_.transpose();
}

bool MPCLocalController::setPath(Path path) {
  if (path.subPathCount() == 0) {
    LOG(ERROR) << "No sub Paths.";
    return false;
  }
  path_mutex_.lock();
  path_.clear();
  path_ = path;
  path_.getCurrentSubPath().wps.pop_back();
  path_.getCurrentSubPath().wpis.pop_back();
  last_target_index_ = 0;
  path_mutex_.unlock();

  // 直接获取路径集合中的当前路径，外面根据情况做逻辑判断挑选需要执行的路径
  SubPath current_subpath = path.getCurrentSubPath();
  if (current_subpath.size() <= 0) {
    LOG(ERROR) << "MPC Received an Empty Target Plan.";
    return false;
  }
  LOG(INFO) << "set subPath size = " << current_subpath.wps.size() << ", "
            << current_subpath.wpis.size();

  goal_mutex_.lock();
  goal_ = current_subpath.back();
  goal_mutex_.unlock();

  return true;
}

bool MPCLocalController::isGoalReached(const Pose2d &current_pose) {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  // TODO ： 目前只判断了直线距离的到达情况，需要添加角度判断
  double goal_xy_distance = current_pose.distanceTo(goal_);
  if (goal_xy_distance < limits_->getXYTolerance()) {
    LOG(INFO) << "XY goal reach! distance = " << goal_xy_distance;
    return true;
  } else {
    LOG(INFO) << "XY Not reach. distance = " << goal_xy_distance;
    return false;
  }
}

std::vector<MpcState> MPCLocalController::getTargetWaypoints(
    const Pose2d &pose, const Velocity &velocity) {
  std::lock_guard<std::mutex> lock(path_mutex_);
  std::vector<MpcState> target_waypoints;
  target_waypoints.reserve(horizon_ + 1);
  const SubPath &sub_path = path_.getCurrentSubPath();
  last_target_index_ = 0;  // TODO: for test
  size_t min_index = last_target_index_;
  double min_distance = 100;
  static double forward_feedback = 0;
  static bool increase = true;
  bool go_forward = true;
  for (size_t i = last_target_index_; i < sub_path.size(); i++) {
    const double &distance = pose.distanceTo(sub_path.wps[i]);
    if (min_distance > distance) {
      min_distance = distance;
      min_index = i;
    } else {
      i++;  // 远离目标点，就加快搜索
    }
  }
  Pose2d delta_pose = pose.inverse() * sub_path.wps[min_index];
  if (delta_pose.getX() < 0 && velocity.d_x < -0.1) {
    go_forward = false;
    LOG(WARNING) << "Go back";
  }
  last_target_index_ = min_index;
  double target_vel = velocity.d_x;
  double distance_step = 0;  // 前进步长，由速度决定
  LOG(INFO) << "Target velocity: ";
  for (size_t i = 0; i < horizon_ + 1; ++i) {
    Pose2d next_target_pose;
    WayPointInfo waypoint_info;
    double sum_dist = 0;
    double curve = 0;
    for (size_t j = last_target_index_; j < sub_path.size() - 1; j++) {
      const double &dist = sub_path.wps[j].distanceTo(sub_path.wps[j + 1]);
      sum_dist += dist;
      curve = AngleCalculate::normalizeAngle(sub_path.wps[j + 1].getYaw() -
                                             sub_path.wps[j].getYaw()) /
              (dist + 1e-4);
      if (0 == i) {
        next_target_pose = sub_path.wps[last_target_index_];
        waypoint_info = sub_path.wpis[last_target_index_];
        break;
      }
      if (sum_dist >= distance_step) {
        last_target_index_ = j + 1;
        next_target_pose = sub_path.wps[last_target_index_];
        waypoint_info = sub_path.wpis[last_target_index_];
        break;
      }
    }
    if (sum_dist < distance_step) {
      next_target_pose = sub_path.wps.back();
      waypoint_info = sub_path.wpis.back();
    }

    if (increase) {
      if (!go_forward) {  // 倒车情况
        if (target_vel - matrix_u_lower_(0, 0) > 0.05) {
          target_vel -=
              0.5 / controller_frequency_;  // 0.5 is better in mpc mode
        } else {
          target_vel = matrix_u_lower_(0, 0);
        }
      } else {
        if (matrix_u_upper_(0, 0) - target_vel > 0.05) {
          target_vel += 1 / controller_frequency_;  // 0.5 is better in mpc mode
        } else {
          target_vel = matrix_u_upper_(0, 0);
        }
      }

    } else {
      target_vel -= forward_feedback;
    }

    // TODO:速度约束
    const double curvature_limit =
        kappa_limit_cofficient_ / (std::fabs(curve) + 0.001);
    if (target_vel >= 0) {
      target_vel = std::min(target_vel, curvature_limit);
      target_vel = std::max(target_vel, min_velocity_);
      target_vel = std::min(target_vel, waypoint_info.v + 0.1);
    } else {
      target_vel = std::max(target_vel, -curvature_limit);
      target_vel = std::min(target_vel, -min_velocity_);
      target_vel = std::max(target_vel, -(waypoint_info.v + 0.1));
    }
    std::cout << target_vel << " " << curve << " " << waypoint_info.v
              << std::endl;
    distance_step = std::fabs(target_vel) / controller_frequency_;
    Pose2d target_in_body = pose.inverse() * next_target_pose;
    target_waypoints.emplace_back(target_in_body.getX(), target_in_body.getY(),
                                  target_in_body.getYaw(), target_vel,
                                  0.5 * curve);
  }
  std::cout << std::endl;
  const double delta_vel = velocity.d_x - target_vel;
  if (delta_vel > 0.2 && velocity.d_x > 0) {
    forward_feedback = delta_vel / horizon_;
    increase = false;
    LOG(WARNING) << "decreasing velocity";
  } else if (delta_vel < -0.2 && velocity.d_x < 0) {
    forward_feedback = delta_vel / horizon_;
    increase = false;
    LOG(WARNING) << "decreasing velocity";
  } else {
    increase = true;
  }
  return target_waypoints;
}

MoveCommand::MoveCommandStatus MPCLocalController::computeMoveComand(
    MoveCommand &cmd) {
  // 更新机器人当前位置
  Pose2d robot_pose = getCurrentPose();
  // 更新机器人当前速度
  Velocity cur_vel = getCurrentVelocity();

  // 判断与目标点的直线距离是否已经到达
  if (isGoalReached(robot_pose)) {
    LOG(INFO) << "lypu reached local goal";
    cmd.setVelX(0);
    cmd.setVelY(0);
    cmd.setVelTH(0);
    return MoveCommand::MoveCommandStatus::REACHED_GOAL;
  }
  auto start = std::chrono::system_clock::now();
  // mpc compute result
  // TODO: 重点是计算ref_states
  std::vector<MpcState> ref_states = getTargetWaypoints(robot_pose, cur_vel);
  MpcOsqp mpc_osqp(matrix_q_, matrix_r_, matrix_initial_x_, matrix_u_lower_,
                   matrix_u_upper_, matrix_x_lower_, matrix_x_upper_,
                   state_dim_, controls_dim_, 1 / controller_frequency_, 100,
                   horizon_, 1e-6);
  mpc_osqp.UpdateState(ref_states);
  std::vector<double> predict_states;
  std::vector<double> control_cmd;
  predict_states.resize(state_dim_ * (horizon_ + 1));
  control_cmd.resize(controls_dim_);
  if (mpc_osqp.Solve(&predict_states, &control_cmd)) {
    std::vector<Pose2d> predict_pose;
    predict_pose.reserve(horizon_ + 1);
    for (size_t i = 0; i < horizon_ + 1; ++i) {
      predict_pose.emplace_back(predict_states[i * state_dim_ + 0],
                                predict_states[i * state_dim_ + 1],
                                predict_states[i * state_dim_ + 2]);
    }
    LOG(INFO) << "current_pose: = " << robot_pose.getX() << ","
              << robot_pose.getY() << "," << robot_pose.getYaw();
    // LOG(INFO) << "Target pose: " << next_target_.getX() << ", "
    //           << next_target_.getY() << ", " << next_target_.getYaw();
    LOG(INFO) << "Calc:  V = " << control_cmd[0] << "  W = " << control_cmd[1];
    cmd.setVelX(control_cmd[0]);
    cmd.setVelY(0);
    cmd.setVelTH(control_cmd[1]);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end - start;
    LOG(INFO) << "mpc spin time :" << static_cast<double>(diff.count()) * 100.0
              << "ms";
    return MoveCommand::MoveCommandStatus::OKAY;

  } else {
    LOG(ERROR) << "MPC solver failed";
    return MoveCommand::MoveCommandStatus::ERROR;
  }
}

}  // namespace CVTE_BABOT
