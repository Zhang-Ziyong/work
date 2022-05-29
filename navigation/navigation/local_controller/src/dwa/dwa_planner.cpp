/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file dwa_planner.cpp
 *
 *@brief DWA算法实现文件
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-18
 ************************************************************************/

#include "dwa/dwa_planner.hpp"

#include "controller_ultis/controller_limits.hpp"
#include "dwa/cost_function/angle_cost_function.hpp"
#include "dwa/cost_function/map_grid_cost_function.hpp"
#include "dwa/cost_function/obstacle_cost_function.hpp"
#include "dwa/cost_function/oscillation_cost_function.hpp"
#include "dwa/cost_function/twirling_cost_function.hpp"
#include "dwa/rotation/rotation_check.hpp"
#include "dwa/trajectory/simple_trajectory_generator.hpp"
#include "dwa/trajectory/simple_trajectory_scorer.hpp"
#include "dwa/world_model/costmap_model.hpp"

#include <glog/logging.h>
#include "costmap_2d.hpp"
#include "costmap_mediator.hpp"

namespace CVTE_BABOT {

DWALocalController::DWALocalController(std::shared_ptr<Costmap2d> costmap) {
  getParams();

  CostmapModel::getPtrInstance()->setPtrCostmap(costmap);
  map_resolution_ = costmap->getResolution();
  path_costs_ = std::make_shared<MapGridCostFunction>(0.0, 0.0);
  path_costs_->setScale(map_resolution_ * pdist_scale_);
  path_costs_->setName(std::string("path_cost"));

  alignment_costs_ = std::make_shared<MapGridCostFunction>(0.0, 0.0);
  alignment_costs_->setScale(map_resolution_ * pdist_scale_);
  alignment_costs_->setXShift(forward_point_distance_);
  alignment_costs_->setStopOnFailure(true);
  alignment_costs_->setName(std::string("alignment_costs"));

  goal_costs_ = std::make_shared<MapGridCostFunction>(0.0, 0.0, true);
  goal_costs_->setScale(map_resolution_ * gdist_scale_);
  goal_costs_->setName(std::string("goal_costs"));

  goal_front_costs_ = std::make_shared<MapGridCostFunction>(0.0, 0.0, true);
  goal_front_costs_->setScale(map_resolution_ * gdist_scale_);
  goal_front_costs_->setXShift(forward_point_distance_);
  goal_front_costs_->setStopOnFailure(false);
  goal_front_costs_->setName(std::string("goal_front_costs"));

  obstacle_costs_ = std::make_shared<ObstacleCostFunction>();
  obstacle_costs_->setScale(map_resolution_ * occdist_scale_);
  obstacle_costs_->setParams(limits_->getMaxTransVel(), max_scaling_factor_,
                             scaling_speed_);
  obstacle_costs_->setSumScores(true);
  obstacle_costs_->setName(std::string("obstacle_costs"));

  oscillation_costs_ = std::make_shared<OscillationCostFunction>();
  oscillation_costs_->setOscillationResetDist(oscillation_reset_dist_,
                                              oscillation_reset_angle_);
  oscillation_costs_->resetOscillationFlags();
  oscillation_costs_->setName(std::string("oscillation_costs"));

  angle_costs_ = std::make_shared<AngleCostFunction>();
  angle_costs_->setName(std::string("angle_costs"));

  // 未使用
  // twirling_costs_ = std::make_shared<TwirlingCostFunction>();

  vsamples_[0] = vx_samples_;
  vsamples_[1] = vy_samples_;
  vsamples_[2] = vth_samples_;

  // set up all the cost functions that will be applied in order
  // (any function returning negative values will abort scoring, so the order
  // can improve performance)
  std::vector<std::shared_ptr<CostFunction>> critics;
  // discards oscillating motions (assisgns cost -1)
  critics.push_back(oscillation_costs_);
  // discards trajectories that move into obstacles
  critics.push_back(obstacle_costs_);
  // prefers trajectories that make the nose go towards (local) nose goal
  critics.push_back(goal_front_costs_);
  // prefers trajectories that keep the robot nose on nose path
  critics.push_back(alignment_costs_);
  // prefers trajectories on global path
  critics.push_back(path_costs_);
  // prefers trajectories that go towards (local) goal, based on wave
  critics.push_back(goal_costs_);
  // prefers trajectories that more near to the goal angle.
  critics.push_back(angle_costs_);

  scored_sampling_planner_ =
      std::make_shared<SimpleTrajectoryScorer>(generator_, critics);

  all_explored_ptr_ = std::make_shared<std::vector<Trajectory>>();
}

void DWALocalController::getParams() {
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }

  double sim_granularity = 0.1;
  double sim_period = 0.1;
  double angular_sim_granularity = 0.05;
  double slow_vv = 0.0;
  double slow_ww = 0.0;
  double sim_dis = 0.0;
  double xy_tolerance = 0.0;
  double yaw_tolerance = 0.0;
  double max_x = 0.0;
  double min_x = 0.0;
  double max_y = 0.0;
  double min_y = 0.0;
  double max_th = 0.0;
  double min_th = 0.0;
  double acc_x = 0.0;
  double acc_y = 0.0;
  double acc_yaw = 0.0;
  double min_velocity_threshold = 0.0;
  double min_rotate_threshold = 0.0;

  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.close_goal_distance",
      close_goal_distance_, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.slow_down_v", slow_vv,
      0.3);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.slow_down_w", slow_ww,
      0.3);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.sim_distance", sim_dis,
      1.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.sim_time", sim_time_, 1.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.vx_samples", vx_samples_,
      3);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.vy_samples", vy_samples_,
      5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.vth_samples", vth_samples_,
      10);
  ptr_navigation_mediator_->getParam("path_follower.xy_goal_tolerance",
                                     xy_tolerance, 0.2);
  ptr_navigation_mediator_->getParam("path_follower.yaw_goal_tolerance",
                                     yaw_tolerance, 0.2);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.max_x", max_x, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.min_x", min_x, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.max_y", max_y, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.min_y", min_y, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.max_th", max_th, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.min_th", min_th, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.acc_x", acc_x, 0.2);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.acc_y", acc_y, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.acc_yaw", acc_yaw, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.min_velocity_threshold",
      min_velocity_threshold, 0.1);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.dwa.min_rotate_threshold",
      min_rotate_threshold, 0.05);

  setMoveSpeedRange(min_x, max_x, min_y, max_y, min_th, max_th);
  setMoveAccLimit(acc_x, acc_y, acc_yaw);
  limits_->setMinVelocityThreshold(min_velocity_threshold);
  limits_->setMinRotateThreshold(min_rotate_threshold);
  limits_->setXYTolerance(xy_tolerance);
  limits_->setYawTolerance(yaw_tolerance);

  std::vector<WorldmapPoint> vwp_footprint;
  CostmapMediator::getPtrInstance()->getParam("footprint", vwp_footprint,
                                              vwp_footprint);

  assert(vwp_footprint.size() == 4);
  for (size_t i = 0; i < vwp_footprint.size(); i++) {
    footprint_spec_[i] =
        Pose2d(vwp_footprint[i].d_x, vwp_footprint[i].d_y, 0.00);

    LOG(INFO) << "footprint point" << i << ": (" << vwp_footprint[i].d_x << ", "
              << vwp_footprint[i].d_y << ") ";
  }

  generator_ = std::make_shared<SimpleTrajectoryGenerator>();
  generator_->setParameters(sim_time_, sim_granularity, angular_sim_granularity,
                            sim_period, sim_dis, slow_vv, slow_ww);
}

void DWALocalController::updateCostMap(std::shared_ptr<Costmap2d> ptr_costmap) {
  CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap);
}

void DWALocalController::setRobotFootPrint(
    const std::array<Pose2d, 4> &footprint) {
  footprint_spec_[0] = footprint[0];
  footprint_spec_[1] = footprint[1];
  footprint_spec_[2] = footprint[2];
  footprint_spec_[3] = footprint[3];
}

bool DWALocalController::findBestPath(const Pose2d &global_pose,
                                      const std::array<double, 3> &global_vel,
                                      std::array<Pose2d, 4> &footprint_spec,
                                      Trajectory &best_traject) {
  bool close_goal = isCloseToGoal(global_pose);
  obstacle_costs_->setFootprint(footprint_spec);
  // prepare cost functions and generators for this run
  generator_->initialise(global_pose, global_vel, limits_, vsamples_,
                         close_goal, true);
  best_traject.cost_ = -7;
  // find best trajectory by sampling and scoring the samples
  all_explored_ptr_->clear();
  bool find_result = scored_sampling_planner_->findBestTrajectory(
      best_traject, all_explored_ptr_);
  return find_result;
}

bool DWALocalController::getAllTraj(std::vector<Trajectory> &traj_result) {
  if ((*all_explored_ptr_).size() <= 0) {
    return false;
  }
  traj_result.resize((*all_explored_ptr_).size());
  for (unsigned int i = 0; i < (*all_explored_ptr_).size(); ++i) {
    traj_result.push_back((*all_explored_ptr_)[i]);
  }
  return (traj_result.size() > 0);
}

bool DWALocalController::setPath(std::shared_ptr<SubPath> path,
                                 size_t begin_index) {
  if (path == nullptr || path->wps.empty()) {
    LOG(ERROR) << "No sub Paths.";
    return false;
  }

  // 直接获取路径集合中的当前路径，外面根据情况做逻辑判断挑选需要执行的路径
  // SubPath current_subpath = (*path.getCurrentSubPath());
  // if (path.size() <= 0 || path.wps.empty()) {
  //   LOG(ERROR) << "DWA Received an Empty Target Plan.";
  //   return false;
  // }
  LOG(INFO) << "set subPath size = " << path->wps.size();

  goal_mutex_.lock();
  goal_ = path->wps.back();
  goal_mutex_.unlock();

  double path_angle = path->wps.front().getYaw();
  // 设置目标点的角度,让机器人倾向于转角最小的方向旋转
  angle_costs_->setTargetAngle(path_angle);

  updatePlanAndLocalCosts(path->wps);
  return true;
}

void DWALocalController::updatePlanAndLocalCosts(
    const std::vector<Pose2d> &new_plan) {
  Pose2d global_pose = getCurrentPose();

  std::vector<Pose2d> target_path(new_plan);

  // costs for going away from path
  path_costs_->setTargetPoses(target_path);
  // costs for not going towards the local goal as much as possible
  goal_costs_->setTargetPoses(target_path);

  Pose2d goal_pose = target_path.back();

  double sq_dist = global_pose.distanceTo(goal_pose);
  LOG(INFO) << "sq_dist = " << sq_dist;
  double angle_to_goal = atan2(goal_pose.getY() - global_pose.getY(),
                               goal_pose.getX() - global_pose.getX());

  target_path.back().setX(target_path.back().getX() +
                          forward_point_distance_ * cos(angle_to_goal));

  target_path.back().setY(target_path.back().getY() +
                          forward_point_distance_ * sin(angle_to_goal));
  goal_front_costs_->setTargetPoses(target_path);

  // keeping the nose on the path
  if (sq_dist > forward_point_distance_ * cheat_factor_) {
    alignment_costs_->setScale(map_resolution_ * pdist_scale_);
    alignment_costs_->setTargetPoses(target_path);
  } else {
    // once we are close to goal, trying to keep the nose close to anything
    // destabilizes behavior.
    alignment_costs_->setScale(0.0);
  }
}

bool DWALocalController::isGoalReached(const Pose2d &current_pose) {
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

bool DWALocalController::isCloseToGoal(const Pose2d &current_pose) {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  double goal_xy_distance = current_pose.distanceTo(goal_);
  // 当前位置与目标点直线距离小于close_goal_distance_时，认为已经比较靠近目标点，需要减速
  if (goal_xy_distance < close_goal_distance_) {
    LOG(INFO) << "Close To Goal = " << goal_xy_distance;
    return true;
  } else {
    return false;
  }
}

MoveCommand::MoveCommandStatus DWALocalController::computeMoveComand(
    MoveCommand &cmd) {
  // 更新机器人当前位置
  Pose2d robot_pose = getCurrentPose();
  // 更新机器人当前速度
  Velocity cur_vel = getCurrentVelocity();
  std::array<double, 3> current_vel = {cur_vel.d_x, cur_vel.d_y, cur_vel.d_yaw};

  // 判断与目标点的直线距离是否已经到达
  if (isGoalReached(robot_pose)) {
    return MoveCommand::MoveCommandStatus::REACHED_GOAL;
  }

  // DWA 进行采样获取最优局部路径
  // auto start = std::chrono::system_clock::now();
  Trajectory best_tmp_traj;
  bool find_result =
      findBestPath(robot_pose, current_vel, footprint_spec_, best_tmp_traj);
  if (find_result && !best_tmp_traj.empty()) {
    double xx = 0.0, yy = 0.0, tt = 0.0, vx = 0.0, vy = 0.0, vth = 0.0;
    best_tmp_traj.getPoint(1, xx, yy, tt, vx, vy, vth);
    cmd.setVelX(vx);
    cmd.setVelY(vy);
    cmd.setVelTH(vth);
    cmd.setBestTraject(best_tmp_traj);
    LOG(INFO) << "DWA calc vel: " << vx << ", " << vth;

    auto end = std::chrono::system_clock::now();
    // std::chrono::duration<double> diff = end - start;
    // LOG(INFO) << "dwa spin time :" << static_cast<double>(diff.count()) *
    // 100.0
    //           << "ms";

    // 获取此次采样的所有轨迹结果
    std::vector<Trajectory> traj_results;
    if (getAllTraj(traj_results)) {
      cmd.setAllTrajects(traj_results);
    } else {
      LOG(WARNING) << "Can't get All Trajects.";
    }

    return MoveCommand::MoveCommandStatus::OKAY;
  }

  return MoveCommand::MoveCommandStatus::ERROR;
}

}  // namespace CVTE_BABOT
