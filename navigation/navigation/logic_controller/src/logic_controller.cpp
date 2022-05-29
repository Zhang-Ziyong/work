/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file logic_controller.cpp
 *
 *@brief
 *
 *@modified by chenmingjina(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-16
 ************************************************************************/
#include "logic_controller.hpp"
#include "bsline/path_optimizer.hpp"
#include "controller_ultis/move_command.hpp"
#include "costmap_2d.hpp"
#include "costmap_mediator.hpp"
#include "global_planner.hpp"
#include "global_planner_factory.hpp"

#include <float.h>
#include <glog/logging.h>

namespace CVTE_BABOT {

LogicController::LogicController(std::shared_ptr<Costmap2d> costmap) {
  updateCostMap(costmap);
  ptr_path_follower_ = std::make_shared<PathFollower>(costmap);
  ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
  getParams();
  ptr_speed_decision_ = SpeedDecisionBase::getInstance();
  ptr_speed_decision_->setCostmap(costmap);
}

void LogicController::updateGlobalPlanner(bool combine_local_costmap) {
  if (!ptr_global_costmap_2d_) {
    LOG(WARNING) << "no global costmap, cant updateGlobalPlanner !";
    return;
  }

  GBPlannerFactory global_planner_factory;
  ptr_global_planner_ = global_planner_factory.createGlobalPlanner(
      global_planner_algorithm_, ptr_global_costmap_2d_->getSizeInCellsX(),
      ptr_global_costmap_2d_->getSizeInCellsY());
  LOG(INFO) << " update global planner costmap. size: "
            << ptr_global_costmap_2d_->getSizeInCellsX() << " * "
            << ptr_global_costmap_2d_->getSizeInCellsY();
  std::shared_ptr<Costmap2d> ptr_global_costmap(new Costmap2d);
  *ptr_global_costmap = *ptr_global_costmap_2d_;

  // 换算坐标
  WorldmapPoint start_pose =
      WorldmapPoint(getCurrentPose().x, getCurrentPose().y);
  int size_x = ptr_global_costmap_2d_->getSizeInCellsX();
  int size_y = ptr_global_costmap_2d_->getSizeInCellsY();

  LOG(INFO) << "start_pose x: " << start_pose.d_x << " y: " << start_pose.d_y;

  CostmapPoint start_grid;
  ptr_global_costmap_2d_->worldToMap(start_pose, start_grid);
  LOG(INFO) << "start_grid x: " << start_grid.ui_x << " y: " << start_grid.ui_y;

  // 删除起点附近的代价值
  int clean_bound = 20.0;  // 1m
  int clean_start_x =
      (start_grid.ui_x - clean_bound) < 0 ? 0 : start_grid.ui_x - clean_bound;
  int clean_start_y =
      (start_grid.ui_y - clean_bound) < 0 ? 0 : start_grid.ui_y - clean_bound;

  int clean_end_x = (start_grid.ui_x + clean_bound) > size_x - 1
                        ? size_x - 1
                        : start_grid.ui_x + clean_bound;
  int clean_end_y = (start_grid.ui_y + clean_bound) > size_y - 1
                        ? size_y - 1
                        : start_grid.ui_y + clean_bound;
  LOG(INFO) << "clean_start x: " << clean_start_x << " y: " << clean_start_y;
  LOG(INFO) << "clean_end x: " << clean_end_x << " y: " << clean_end_y;

  for (int x = clean_start_x; x < clean_end_x; x++) {
    for (int y = clean_start_y; y < clean_end_y; y++) {
      ptr_global_costmap.get()->getCharMap().get()[x + size_x * y] = 0;
    }
  }
  if (ptr_costmap_2d_ != nullptr &&
      RobotInfo::getPtrInstance()->getMissionType() != MISSIONTYPE::ELVATOR &&
      combine_local_costmap) {
    ptr_global_costmap->combineCostmap(ptr_costmap_2d_);
    LOG(INFO) << "combine local costmap";
  }
  ptr_global_planner_->updateCostMap(ptr_global_costmap);
}

void LogicController::getParams() {
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }
  ptr_navigation_mediator_->getParam("logic_controller.max_ditance_to_path",
                                     max_ditance_to_path_, 5.0);
  ptr_navigation_mediator_->getParam("logic_controller.max_path_resolution",
                                     max_path_resolution_, 0.5);
  ptr_navigation_mediator_->getParam(
      "logic_controller.global_planner_algorithm", global_planner_algorithm_,
      std::string("dijkstra"));

  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.optimize_global_path",
      optimize_global_path_, true);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.calc_vel_limit_ctrl",
      global_calc_vel_limit_ctrl_, true);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.calc_optimize_ctrl",
      global_optimize_ctrl_, true);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.v_limit",
                                     global_v_limit_, 1.2);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.w_limit",
                                     global_w_limit_, 0.6);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.acc_limit",
                                     global_acc_limit_, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.bsp_delta_time", global_bsp_delta_time_,
      0.4);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.sample_delta_time",
      global_sample_delta_time_, 0.1);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.lambda1",
                                     lambda1_, 1.0);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.lambda2",
                                     lambda2_, 10.0);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.lambda3",
                                     lambda3_, 10.0);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.lambda4",
                                     lambda4_, 10.0);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.use_endpt_opt", use_endpt_opt_, false);
  ptr_navigation_mediator_->getParam("path_optimize.global_optimizer.dist",
                                     obstacle_dist0_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.min_rot_radiu", min_rot_radiu_, 0.4);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.max_iteration_num", max_iter_nums_, 10);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.max_iteration_time", max_iter_time_, 0.1);
  ptr_navigation_mediator_->getParam(
      "path_optimize.global_optimizer.jump_point_size", jump_point_size_, 1);

  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.optimize_global_path",
      optimize_global_path_, true);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.calc_vel_limit_ctrl",
      edge_calc_vel_limit_ctrl_, true);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.calc_optimize_ctrl", edge_optimize_ctrl_,
      true);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.v_limit",
                                     edge_v_limit_, 1.2);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.w_limit",
                                     edge_w_limit_, 0.6);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.acc_limit",
                                     edge_acc_limit_, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.bsp_delta_time", edge_bsp_delta_time_, 0.4);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.sample_delta_time", edge_sample_delta_time_,
      0.1);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.lambda1",
                                     edge_lambda1_, 1.0);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.lambda2",
                                     edge_lambda2_, 10.0);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.lambda3",
                                     edge_lambda3_, 10.0);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.lambda4",
                                     edge_lambda4_, 10.0);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.use_endpt_opt", edge_use_endpt_opt_, false);
  ptr_navigation_mediator_->getParam("path_optimize.edge_optimizer.dist",
                                     edge_obstacle_dist0_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.min_rot_radiu", edge_min_rot_radiu_, 0.4);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.max_iteration_num", edge_max_iter_nums_,
      10);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.max_iteration_time", edge_max_iter_time_,
      0.1);
  ptr_navigation_mediator_->getParam(
      "path_optimize.edge_optimizer.jump_point_size", edge_jump_point_size_, 1);

  ptr_navigation_mediator_->getParam("controller_frequency",
                                     controller_frequency_, 10.0);
  ptr_navigation_mediator_->getParam("edge_dist", edge_dist_, -0.5);
}

double LogicController::getPathEstimateTime(const SubPath &path) {
  PathOptimizer path_opter;
  double speed_level = ptr_speed_decision_->getMissionManagerSpeedValue();
  CtrlPointOptParams ctrl_pt_opt_params(
      lambda1_, lambda2_, lambda3_, lambda4_, obstacle_dist0_, min_rot_radiu_,
      max_iter_nums_, max_iter_time_, use_endpt_opt_);
  OptPathParams opt_params(global_v_limit_, global_w_limit_, global_acc_limit_,
                           global_bsp_delta_time_ / speed_level,
                           global_sample_delta_time_, 0.1, 0.0, 1,
                           global_calc_vel_limit_ctrl_, global_optimize_ctrl_,
                           ctrl_pt_opt_params);

  path_opter.curveFitPath(path, opt_params, nullptr);
  return path_opter.getPathTime();
}

bool LogicController::setGoalPath(const SubPath &goal_path) {
  if (!isValidGoalPath(goal_path.wps)) {
    LOG(ERROR) << "goal path is invalid." << std::endl;
    return false;
  } else {
    std::shared_ptr<SubPath> refer_path = nullptr;
    double speed_level = ptr_speed_decision_->getMissionManagerSpeedValue();
    if (optimize_global_path_) {
      LOG(INFO) << "optimize_global_path_: " << optimize_global_path_;

      if (!ptr_global_costmap_2d_) {
        LOG(WARNING) << "no global costmap, cant optimize goal path !";
        return false;
      }

      PathOptimizer path_opter;
      CtrlPointOptParams ctrl_pt_opt_params;
      CtrlPointOptParams edge_ctrl_pt_opt_params;
      OptPathParams opt_params;
      OptPathParams edge_opt_params;
      SubPath opt_path;
      ctrl_pt_opt_params = CtrlPointOptParams(
          lambda1_, lambda2_, lambda3_, lambda4_, obstacle_dist0_,
          min_rot_radiu_, max_iter_nums_, max_iter_time_, use_endpt_opt_);
      opt_params = OptPathParams(
          global_v_limit_, global_w_limit_, global_acc_limit_,
          global_bsp_delta_time_ / speed_level, global_sample_delta_time_, 0.0,
          0.0, jump_point_size_, global_calc_vel_limit_ctrl_,
          global_optimize_ctrl_, ctrl_pt_opt_params);
      edge_ctrl_pt_opt_params = CtrlPointOptParams(
          edge_lambda1_, edge_lambda2_, edge_lambda3_, edge_lambda4_,
          edge_obstacle_dist0_, edge_min_rot_radiu_, edge_max_iter_nums_,
          edge_max_iter_time_, edge_use_endpt_opt_);
      edge_opt_params =
          OptPathParams(edge_v_limit_, edge_w_limit_, edge_acc_limit_,
                        edge_bsp_delta_time_, edge_sample_delta_time_, 0.0, 0.0,
                        edge_jump_point_size_, edge_calc_vel_limit_ctrl_,
                        edge_optimize_ctrl_, edge_ctrl_pt_opt_params);
      if (RobotInfo::getPtrInstance()->getMissionType() != MISSIONTYPE::EDGE) {
        refer_path = std::make_shared<SubPath>(path_opter.curveFitPath(
            goal_path, opt_params, ptr_global_costmap_2d_));
      } else {
        refer_path = std::make_shared<SubPath>();
        std::vector<SubPath> segment_path_vec;
        std::vector<SubPath> segment_path_type_vec;
        SubPath segment_path;
        SubPath segment_path_type;

        segment_path_type.wps.push_back(goal_path.wps.front());
        segment_path_type.wpis.push_back(goal_path.wpis.front());

        for (size_t index = 0; index < goal_path.wps.size() - 1; index++) {
          // FIXME: 新方式
          segment_path.wps.push_back(goal_path.wps[index]);
          segment_path.wpis.push_back(goal_path.wpis[index]);
          if (goal_path.wpis[index].is_edge_wise !=
              goal_path.wpis[index + 1].is_edge_wise) {
            segment_path_type.wps.push_back(goal_path.wps[index + 1]);
            segment_path_type.wpis.push_back(goal_path.wpis[index + 1]);
            LOG(INFO) << "creat type "
                      << segment_path_type.wpis.back().is_edge_wise;
          }
        }

        segment_path.push_back(goal_path.wps.back());
        segment_path.push_back(goal_path.wpis.back());
        segment_path_vec.push_back(segment_path);
        segment_path_type_vec.push_back(segment_path_type);
        segment_path.clear();
        segment_path_type.clear();

        LOG(INFO) << "segment_path_size: " << segment_path_vec.size();
        std::vector<SubPath> segment_opt_path_vec;
        for (size_t index = 0; index < segment_path_vec.size(); index++) {
          LOG(INFO) << "index: " << index << "segment_path_size: "
                    << segment_path_vec[index].wps.size();
          // 优化
          SubPath segment_path_type = segment_path_type_vec[index];
          SubPath segment_opt_path = path_opter.curveFitPath(
              segment_path_vec[index], edge_opt_params, ptr_global_costmap_2d_);

          // 给优化后的节点 赋 贴边属性
          size_t opt_path_begin_index = 0;
          size_t path_type_index = 0;

          size_t last_point_index = 0;
          size_t last_type_index = 1;
          for (size_t point_index = 0; point_index < segment_opt_path.size();
               point_index++) {
            if (segment_path_vec[index].wpis[0].is_edge_wise > 0) {
              segment_opt_path.wpis[point_index].edge_dist = edge_dist_;
            }
            if (last_type_index < segment_path_type.size()) {
              if (last_point_index < point_index) {
                if (segment_path_type.wps[last_type_index].distanceTo(
                        segment_opt_path.wps[point_index]) < 0.3) {
                  // 找最近的点
                  double min_distance = DBL_MAX;
                  size_t min_point_index = last_point_index;
                  size_t end_point_index =
                      point_index + 100 < segment_opt_path.size()
                          ? point_index + 100
                          : segment_opt_path.size() - 1;

                  for (size_t i = last_point_index; i < end_point_index; i++) {
                    double distance =
                        segment_path_type.wps[last_type_index].distanceTo(
                            segment_opt_path.wps[i]);
                    if (min_distance > distance) {
                      min_distance = distance;
                      min_point_index = i;
                    }
                  }
                  LOG(INFO) << "min_point_index " << min_point_index;
                  for (; last_point_index <= min_point_index;
                       ++last_point_index) {
                    segment_opt_path.wpis[last_point_index].is_edge_wise =
                        segment_path_type.wpis[last_type_index - 1]
                            .is_edge_wise;
                  }
                  last_type_index++;
                  // last_point_index = min_point_index;
                }
              }
            } else {
              segment_opt_path.wpis[point_index].is_edge_wise =
                  segment_path_type.wpis.back().is_edge_wise;
            }
          }

          segment_opt_path_vec.push_back(segment_opt_path);
        }
        for (size_t path_index = 0; path_index < segment_opt_path_vec.size();
             path_index++) {
          for (size_t point_index = 0;
               point_index < segment_opt_path_vec[path_index].size();
               point_index++) {
            refer_path->wps.push_back(
                segment_opt_path_vec[path_index].wps[point_index]);
            if (path_index > 0) {
              segment_opt_path_vec[path_index].wpis[point_index].path_length +=
                  segment_opt_path_vec[path_index - 1].wpis.back().path_length;
            }
            refer_path->wpis.push_back(
                segment_opt_path_vec[path_index].wpis[point_index]);
          }
        }
      }
      // 避免优化将末端位置值修改，最终点到点精度以下发路径的末端点为准
      Pose2d l_p = goal_path.wps.back();
      refer_path->wps.back().setPose(
          l_p.getX(), l_p.getY(),
          refer_path->wps.back().getYaw());  // 优化停靠精度
      ptr_path_follower_->setReferencePath(refer_path);
      ptr_path_follower_->setTargetPose(goal_path.wps.back());
      setOptimizeMissionPath(refer_path);
      LOG(INFO) << "goal optimize path set success." << std::endl;
    } else {
      refer_path = std::make_shared<SubPath>(goal_path);
      ptr_path_follower_->setReferencePath(refer_path);
      LOG(INFO) << "goal path set success." << std::endl;
    }

    return true;
  }
}

bool LogicController::makeGlobalPlan(const Pose2d &start_pose,
                                     const Pose2d &goal_pose, SubPath &result) {
  MakePlanInfo make_plan_info;
  make_plan_info.start_point = start_pose;
  make_plan_info.target_point = goal_pose;

  auto vel = RobotInfo::getPtrInstance()->getCurrentVel();
  Eigen::Vector3d start_vel(vel.d_x * cos(start_pose.getYaw()),
                            vel.d_x * sin(start_pose.getYaw()), 0);
  Eigen::Vector3d end_vel(0.1 * cos(goal_pose.getYaw()),
                          0.1 * sin(goal_pose.getYaw()), 0.0);
  LOG(INFO) << "start pose: " << start_pose.getX() << " " << start_pose.getY()
            << " " << start_pose.getYaw();
  LOG(INFO) << "goal pose: " << goal_pose.getX() << " " << goal_pose.getY()
            << " " << goal_pose.getYaw();
  LOG(INFO) << "start vel: " << start_vel.transpose();
  LOG(INFO) << "end vel: " << end_vel.transpose();
  make_plan_info.start_vel = start_vel;
  make_plan_info.target_vel = end_vel;

  return ptr_global_planner_->makePlan(make_plan_info, result);
}

SubPath LogicController::setGoalPose(const Pose2d &goal_pose,
                                     bool combine_local_costmap) {
  Pose2d start_pose = getCurrentPose();
  goal_pose_ = goal_pose;
  SubPath path_pose =
      makePlanBetweenPoints(start_pose, goal_pose, combine_local_costmap);
  std::shared_ptr<SubPath> refer_path = nullptr;

  if (optimize_global_path_ && !path_pose.empty()) {
    LOG(INFO) << "optimize_global_path_: " << optimize_global_path_;
    auto end_point = path_pose.wps[path_pose.wps.size() - 1];
    auto end_point_info = path_pose.wpis[path_pose.wpis.size() - 1];
    path_pose.wps.push_back(end_point);
    path_pose.wpis.push_back(end_point_info);
    refer_path = std::make_shared<SubPath>(optimizeGlobalPath(path_pose));
    setOptimizeMissionPath(refer_path);
  }

  if (refer_path != nullptr) {
    ptr_path_follower_->setReferencePath(refer_path);
    ptr_path_follower_->setTargetPose(goal_pose);  // 优化停靠精度
  }
  return path_pose;
}

SubPath LogicController::optimizeGlobalPath(SubPath path_pose) {
  if (path_pose.empty()) {
    LOG(WARNING) << "Can't optimize an empty path.";
    return path_pose;
  }
  PathOptimizer path_opter;
  //点导航使用更长的优化时间和迭代次数,以得到更加平滑的曲线
  CtrlPointOptParams ctrl_pt_opt_params(
      lambda1_, lambda2_, lambda3_, lambda4_, obstacle_dist0_, min_rot_radiu_,
      4 * max_iter_nums_, 4 * max_iter_time_, use_endpt_opt_);
  double speed_level = ptr_speed_decision_->getMissionManagerSpeedValue();
  if (global_planner_algorithm_ == "kinodynamic") {
    // Kinodynamic 全局规划的时候可能有前进后退的情况，优化时需要分开处理
    std::vector<SubPath> segment_result;
    if (path_opter.judgeSegmentPath(path_pose, segment_result)) {
      OptPathParams back_opt_params(
          global_v_limit_, global_w_limit_, global_acc_limit_,
          2.0 * global_bsp_delta_time_ / speed_level, global_sample_delta_time_,
          0.0, 0.0, 0, true, true, ctrl_pt_opt_params);
      OptPathParams forward_opt_params(
          global_v_limit_, global_w_limit_, global_acc_limit_,
          2.0 * global_bsp_delta_time_ / speed_level, global_sample_delta_time_,
          0.0, 0.0, 1, true, true, ctrl_pt_opt_params);
      SubPath total_path;
      for (size_t index = 0; index < segment_result.size(); index++) {
        SubPath sub_path;
        if (segment_result[index].wpis[1].v >= 0) {
          LOG(INFO) << "forward segment";
          sub_path =
              path_opter.curveFitPath(segment_result[index], forward_opt_params,
                                      ptr_global_costmap_2d_);
          for (size_t point_index = 0; point_index < sub_path.wps.size();
               point_index++) {
            total_path.wps.emplace_back(sub_path.wps[point_index]);
            total_path.wpis.emplace_back(sub_path.wpis[point_index]);
          }
        } else {
          LOG(INFO) << "backward segment";
          sub_path = path_opter.curveFitPath(
              segment_result[index], back_opt_params, ptr_global_costmap_2d_);
          for (size_t point_index = 0; point_index < sub_path.wps.size();
               point_index++) {
            Pose2d back_point(sub_path.wps[point_index].getX(),
                              sub_path.wps[point_index].getY(),
                              AngleCalculate::normalizeAngle(
                                  sub_path.wps[point_index].getYaw() + M_PI));
            WayPointInfo back_vel(-fabs(sub_path.wpis[point_index].v),
                                  sub_path.wpis[point_index].w,
                                  sub_path.wpis[point_index].curve);
            total_path.wps.emplace_back(back_point);
            total_path.wpis.emplace_back(back_vel);
          }
        }
      }
      return total_path;
    }
  }
  OptPathParams opt_params(global_v_limit_, global_w_limit_, global_acc_limit_,
                           2.0 * global_bsp_delta_time_ / speed_level,
                           global_sample_delta_time_, 0.0, 0.0, 1,
                           global_calc_vel_limit_ctrl_, global_optimize_ctrl_,
                           ctrl_pt_opt_params);
  SubPath opt_path =
      path_opter.curveFitPath(path_pose, opt_params, ptr_global_costmap_2d_);

  // 避免优化将末端位置值修改，最终点到点精度以下发路径的末端点为准
  Pose2d l_p = path_pose.wps.back();
  opt_path.wps.back().setPose(l_p.getX(), l_p.getY(),
                              opt_path.wps.back().getYaw());
  LOG(INFO) << "Optimize make global plan result: ";
  for (auto &pose : opt_path.wps) {
    LOG(INFO) << " = " << pose.getX() << ", " << pose.getY() << ", "
              << pose.getYaw();
  }
  return opt_path;
}

SubPath LogicController::makePlanBetweenPoints(const Pose2d &point_a,
                                               const Pose2d &point_b,
                                               bool combine_local_costmap) {
  updateGlobalPlanner(combine_local_costmap);
  SubPath global_path;
  if (!isValidPose(point_a, point_b)) {
    LOG(ERROR) << "input pose is invalid." << std::endl;
    return global_path;
  }

  if (!makeGlobalPlan(point_a, point_b, global_path)) {
    plan_error_msg_ = PlanErrorMsg::CALC_FAILED;
    LOG(ERROR) << "can not make global path." << std::endl;
    return global_path;
  } else {
    plan_error_msg_ = PlanErrorMsg::OK;
    LOG(INFO) << "goal pose set success." << std::endl;
    return global_path;
  }
}

SubPath LogicController::makePlanToPoints(const Pose2d &target) {
  Pose2d start = getCurrentPose();
  return makePlanBetweenPoints(start, target);
}

FollowerStateRes LogicController::plannerUpdate() {
  return ptr_path_follower_->followerUpdate();
}

void LogicController::controllerUpdate(
    MoveCommand &move_command,
    MoveCommand::MoveCommandStatus &move_command_status) {
  // 获取速度 （调用闭环或旋转）
  PathFollowerStates state =
      ptr_path_follower_->controllerExecute(move_command, move_command_status);

  if (state != PathFollowerStates::RECOVER) {
    // 获取当前计算的速度
    Velocity controller_vel;
    controller_vel.d_x = move_command.getVelX();
    controller_vel.d_y = move_command.getVelY();
    controller_vel.d_yaw = move_command.getVelTH();

    // 速度限制 (根据减速框)
    Velocity actual_vel = ptr_speed_decision_->getActualVel(controller_vel);
    if (ptr_speed_decision_->getSpeedResult().speed_radio == SpeedRadio::STOP) {
      move_command_status = MCMDSTATE::OBSTACLE_STOP;
    }

    move_command.setVelX(actual_vel.d_x);
    move_command.setVelY(actual_vel.d_y);
    move_command.setVelTH(actual_vel.d_yaw);
  }
}

bool LogicController::isValidPose(const Pose2d &start_pose,
                                  const Pose2d &goal_pose) {
  auto getCostValue = [this](Pose2d point) {
    WorldmapPoint wm_point = {point.getX(), point.getY()};
    CostmapPoint cp_point;
    int cost_value = 0;
    if (ptr_global_costmap_2d_->worldToMap(wm_point, cp_point)) {
      cost_value = static_cast<int>(
          ptr_global_costmap_2d_->getCost(cp_point.ui_x, cp_point.ui_y));
      return cost_value;
    } else {
      LOG(WARNING) << "over transmis.";
      return 256;
    }
  };

  int start_value = -1;
  start_value = getCostValue(start_pose);
  LOG(INFO) << "start_pose: " << start_pose.getX() << " " << start_pose.getY()
            << " " << start_pose.getYaw();
  LOG(INFO) << "cost val: " << start_value;
  // if (250 < start_value && start_value <= 254) {
  //   plan_error_msg_ = PlanErrorMsg::START_OBSTACLE;
  //   return false;
  // }

  int target_value = -1;
  target_value = getCostValue(goal_pose);
  LOG(INFO) << "goal_pose: " << goal_pose.getX() << " " << goal_pose.getY()
            << " " << goal_pose.getYaw();
  LOG(INFO) << "cost val: " << target_value;
  if (250 < target_value && target_value <= 254) {
    plan_error_msg_ = PlanErrorMsg::TARGET_OBSTACLE;
    return false;
  } else if (target_value == 255) {
    plan_error_msg_ = PlanErrorMsg::TARGET_UNKNOW;
    return false;
  } else if (target_value == 256) {
    plan_error_msg_ = PlanErrorMsg::TARGET_OVERMAP;
    return false;
  }

  return true;
}

bool LogicController::isValidGoalPath(const std::vector<Pose2d> &goal_path) {
  LOG(INFO) << "Receive Mission Path: ";
  for (auto &pose : goal_path) {
    LOG(INFO) << " - " << pose.getX() << ", " << pose.getY() << ", "
              << pose.getYaw();
  }

  if (goal_path.size() <= 0) {
    LOG(ERROR) << "goal path is empty." << std::endl;
    return false;
  }
  path_resolution_ = calcPathResolution(goal_path);
  if (path_resolution_ > max_path_resolution_) {
    LOG(ERROR) << "path_resolution > max_path_resolution." << std::endl;
    return false;
  }

  int index = 0;
  double distance = 0.0;
  Pose2d pose = getCurrentPose();
  distance = calcPoseToPathDistance(pose, goal_path, index);

  if (distance > max_ditance_to_path_) {
    LOG(ERROR) << "the robot pose is too far from the path." << std::endl;
    return false;
  }

  return true;
}

double LogicController::calcPathResolution(const std::vector<Pose2d> &path) {
  if (path.size() < 2) {
    LOG(ERROR) << "Less than 2 points to calc path resolution.";
    return 0.0;
  }

  double delta = 0.0;
  double delta_sum = 0.0;
  int path_size = path.size();
  int size = std::min(20, path_size);  //计算前20个点的分辨率.

  for (int i = 1; i < size; ++i) {
    delta = path[i - 1].distanceTo(path[i]);
    if (delta < 0.01) {
      LOG(WARNING) << "path resolution is too small:  (" << path[i - 1].x
                   << ", " << path[i - 1].y << ") --- (" << path[i].x << ", "
                   << path[i].y << ")." << std::endl;
    }
    delta_sum += delta;
  }

  double path_resolution = delta_sum / (static_cast<double>(size - 1));
  LOG(INFO) << "path resolution: " << path_resolution << std::endl;

  return path_resolution;
}

double LogicController::calcPoseToPathDistance(const Pose2d &pose,
                                               const std::vector<Pose2d> &path,
                                               int &index) {
  int min_index = 0;
  double distance = 0.0;
  double min_distance = DBL_MAX;

  for (size_t i = 0; i < path.size(); ++i) {
    distance = pose.distanceTo(path[i]);
    if (min_distance > distance) {
      min_distance = distance;
      min_index = i;
    }
  }
  LOG(INFO) << "min_index: " << index << "  min_distance: " << min_distance;
  index = min_index;
  return min_distance;
}

void LogicController::updatePlannerGlobalCostMap() {
  CostmapMediator::getPtrInstance()->getData("static_costmap",
                                             ptr_global_costmap_2d_);
  ptr_path_follower_->updatePlanerGlobalCostMap(ptr_global_costmap_2d_);
}

}  // namespace CVTE_BABOT
