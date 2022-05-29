/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file path_follower.cpp
 *
 *@brief 用于跟踪路径
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-18
 ************************************************************************/

#include "path_follower.hpp"
#include "controller_ultis/move_command.hpp"
#include "costmap_2d.hpp"
#include "costmap_mediator.hpp"
#include "local_controller_base.hpp"
#include "local_controller_factory.hpp"
#include "path_manager.hpp"
#include "turn_lighter.hpp"
#include "planner_decision/planner_decision.hpp"

#include "recovery/front_path_recovery.hpp"
#include "recovery/rear_path_recovery.hpp"
#include "recovery/round_path_recovery.hpp"
// #include "supervisor/car_avoider_supervisor.hpp"
#include "supervisor/dist_topath_supervisor.hpp"
#include "supervisor/angle_topath_supervisor.hpp"
#include "supervisor/final_goal_supervisor.hpp"
#include "supervisor/obstacle_avoider_supervisor.hpp"
#include "supervisor/over_time_supervisor.hpp"
#include "supervisor/speed_level_supervisor.hpp"
#include "supervisor/marker_map_supervisor.hpp"
#include "supervisor/supervisor.hpp"
#include "supervisor/supervisor_chain.hpp"
#include "supervisor/pass_pit_supervisor.hpp"

// #include "state/avoidcar_state.hpp"
#include "state/done_state.hpp"
#include "state/free_state.hpp"
#include "state/local_state.hpp"
#include "state/pause_state.hpp"
#include "state/refer_state.hpp"
#include "state/rotate_state.hpp"
#include "state/recover_state.hpp"
#include "state/pit_state.hpp"
#include "state/state.hpp"

#include "obstacle_map.hpp"

#include "speed_decision_base.hpp"

#include <float.h>
#include <glog/logging.h>

namespace CVTE_BABOT {

PathFollower::PathFollower(std::shared_ptr<Costmap2d> costmap) {
  // recovery构造函数，请放在读参前，读参函数会直接使用这个指针设置，避免过多成员变量
  if (RobotInfo::getPtrInstance()->getRobotType() == ROBOTTYPE::KAVA_CLEAN_C5) {
    LOG(INFO) << "make path recovery for clean_c5";
    ptr_path_recovery_ = std::make_shared<RearPathRecovery>();
  } else if (RobotInfo::getPtrInstance()->getRobotType() ==
             ROBOTTYPE::KAVA_CLEAN_C3) {
    // TODO: change to round
    LOG(INFO) << "make path recovery for clean_c3";
    ptr_path_recovery_ = std::make_shared<RoundPathRecovery>();
  } else {
    ptr_path_recovery_ = std::make_shared<FrontPathRecovery>();
    LOG(INFO) << "make path recovery for security robot";
  }

  // 读取参数
  updateParams();
  ptr_costmap_2d_ = costmap;
  LocalControllerFactory local_controller_factory;
  ptr_local_controller_ = local_controller_factory.createLocalController(
      local_controller_algorithm_, costmap);

  ptr_path_manager_ =
      std::make_shared<PathManager>(costmap, local_planner_algorithm_);
  PlannerDecision::getInstance()->setConfig(planner_decision_config_);
  setPathManagerParams();

  // 构造监督器
  unsigned int ui_size_x = costmap->getSizeInCellsX();
  unsigned int ui_size_y = costmap->getSizeInCellsY();
  constructSupervisor(ui_size_x, ui_size_y);

  // 将当前状态置成空闲
  changeState(FreeState::getPtrInstance());
}

void PathFollower::constructSupervisor(const unsigned int &ui_size_x,
                                       const unsigned int &ui_size_y) {
  /*各监督器之间也需要有优先级，依据每个监督器的效果来排序.
  car_avoider_supervisor：切换全局目标点，用于避让车辆（暂时没用上），
  pedestrian_avoider_supervisor：改变速度，智能避让行人（暂时没用上），

  SpeedLevelSupervisor： 速度等级监督，用于停障和减速
  ObstacleAvoiderSupervisor：检查当前巡逻路径上是否有障碍物被占用
  DistanceToPathSupervisor：检查机器人当前位置与跟踪路径是否已经偏离
  AngleTopathSupervisor：检查机器人是否与路径偏移的角度过大
  FinalGoalSupervisor：检查机器人是否已经到达任务路径最终点
  */

  /*
   * 监督器的内容，详细内容见kb文档(2.3 监督器)
   *  文档链接： https://kb.cvte.com/pages/viewpage.action?pageId=218851057
   */

  // 使用make_shared比new更安全，出现异常时不会内存泄漏
  //　按优先级存入

  // 车辆主动避让
  // SupervisorChain::getPtrInstance()->addSupervisor(
  //     std::make_shared<CarAvoiderSupervisor>(ui_size_x, ui_size_y));

  ptr_time_supor_ = std::make_shared<OverTimeSupervisor>();
  ptr_time_supor_->setSuperviseFrquence(planner_frequence_, 1.0);
  ptr_time_supor_->setOverTimeDist(over_time_wait_dist_);
  ptr_time_supor_->setOverTimeSec(over_time_wait_sec_);
  ptr_time_supor_->setPauseTimeSec(pause_wait_sec_);
  ptr_time_supor_->setRotateTimeSec(rotate_wait_sec_);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_time_supor_);

  ptr_speed_supor_ = std::make_shared<SpeedLevelSupervisor>();
  ptr_speed_supor_->setSuperviseFrquence(planner_frequence_,
                                         planner_frequence_);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_speed_supor_);

  ptr_obstacle_supor_ = std::make_shared<ObstacleAvoiderSupervisor>(
      obstacle_avoid_distance_, obstacle_avoid_behind_, dangerous_value_);
  ptr_obstacle_supor_->setSuperviseFrquence(planner_frequence_, 2.0);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_obstacle_supor_);

  ptr_pit_supor_ =
      std::make_shared<PassPitSupervisor>(pit_avoid_distance_, enable_pit_);
  ptr_pit_supor_->setSuperviseFrquence(planner_frequence_, 2.0);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_pit_supor_);

  ptr_distance_supor_ =
      std::make_shared<DistanceToPathSupervisor>(max_distance_to_path_);
  ptr_distance_supor_->setSuperviseFrquence(planner_frequence_, 0.5);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_distance_supor_);

  ptr_angle_supor_ =
      std::make_shared<AngleTopathSupervisor>(max_angle_to_path_);
  ptr_angle_supor_->setSuperviseFrquence(planner_frequence_, 0.5);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_angle_supor_);

  ptr_goal_supor_ = std::make_shared<FinalGoalSupervisor>();
  ptr_goal_supor_->setGoalXYTolerance(xy_tolerance_);
  ptr_goal_supor_->setSuperviseFrquence(planner_frequence_, 5.0);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_goal_supor_);

  // 添加标记地图监督器
  ptr_marker_supor_ = std::make_shared<MarkerMapSupervisor>(
      narrow_enabled_, path_dir_length_, neighbor_expand_length_);
  SupervisorChain::getPtrInstance()->addSupervisor(ptr_marker_supor_);

  // SupervisorChain::getPtrInstance()->addSupervisor(
  //     Supervisor::Ptr(std::make_shared<PedestrianAvoiderSupervisor>()));
}

void PathFollower::updateParams() {
  ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }
  ptr_navigation_mediator_->getParam("path_follower.enable_recover",
                                     enable_recover_, false);
  ptr_navigation_mediator_->getParam("path_follower.enable_rotate",
                                     enable_rotate_, false);
  ptr_navigation_mediator_->getParam("path_follower.robot_can_rotate",
                                     robot_can_rotate_, true);
  ptr_navigation_mediator_->getParam("path_follower.local_planner_algorithm",
                                     local_planner_algorithm_,
                                     std::string("dijkstra"));
  ptr_navigation_mediator_->getParam("path_follower.local_controller_algorithm",
                                     local_controller_algorithm_,
                                     std::string("pid"));
  ptr_navigation_mediator_->getParam("path_follower.edge_controller_algorithm",
                                     edge_controller_algorithm_,
                                     std::string("pid"));
  ptr_navigation_mediator_->getParam("path_follower.max_distance_to_path",
                                     max_distance_to_path_, 1.0);
  ptr_navigation_mediator_->getParam("path_follower.max_angle_to_path",
                                     max_angle_to_path_, 1.0);
  ptr_navigation_mediator_->getParam("path_follower.obstacle_avoid_distance",
                                     obstacle_avoid_distance_, 4.0);
  ptr_navigation_mediator_->getParam("path_follower.obstacle_avoid_behind",
                                     obstacle_avoid_behind_, 2.0);
  ptr_navigation_mediator_->getParam("path_follower.dangerous_value",
                                     dangerous_value_, 150);
  ptr_navigation_mediator_->getParam("path_follower.min_rotate_threshold",
                                     min_rotate_threshold_, 0.05);
  ptr_navigation_mediator_->getParam("path_follower.min_velocity_threshold",
                                     min_velocity_threshold_, 0.05);
  ptr_navigation_mediator_->getParam("path_follower.max_recover_distance",
                                     max_recover_distance_, 1.0);
  ptr_navigation_mediator_->getParam("planner_frequency", planner_frequence_,
                                     2.0);
  ptr_navigation_mediator_->getParam("controller_frequency",
                                     controller_frequence_, 10.0);
  ptr_navigation_mediator_->getParam("narrow.enabled", narrow_enabled_, false);
  ptr_navigation_mediator_->getParam("narrow.path_dir_length", path_dir_length_,
                                     1.0);
  ptr_navigation_mediator_->getParam("narrow.neighbor_expand_length",
                                     neighbor_expand_length_, 1.0);
  PauseState::getPtrInstance()->setPlannerFrequence(
      static_cast<int>(planner_frequence_));

  ptr_navigation_mediator_->getParam("path_follower.pit.enabled", enable_pit_,
                                     enable_pit_);
  ptr_navigation_mediator_->getParam("path_follower.pit.avoid_distance",
                                     pit_avoid_distance_, pit_avoid_distance_);
  double edge_dist = 0;
  ptr_navigation_mediator_->getParam("path_follower.pit.edge_dist", edge_dist,
                                     edge_dist);
  PitState::getPtrInstance()->setParams(edge_dist);

  int avoid_wait_sec = 0;
  int recover_wait_sec = 0;
  int unreach_wait_sec = 0;
  ptr_navigation_mediator_->getParam("path_follower.avoid_wait_sec",
                                     avoid_wait_sec, 0);
  ptr_navigation_mediator_->getParam("path_follower.recover_wait_sec",
                                     recover_wait_sec, 2);
  ptr_navigation_mediator_->getParam("path_follower.unreach_wait_sec",
                                     unreach_wait_sec, 5);
  ptr_navigation_mediator_->getParam("path_follower.over_time_wait_sec",
                                     over_time_wait_sec_, over_time_wait_sec_);
  ptr_navigation_mediator_->getParam("path_follower.pause_wait_sec",
                                     pause_wait_sec_, pause_wait_sec_);
  ptr_navigation_mediator_->getParam("path_follower.rotate_wait_sec",
                                     rotate_wait_sec_, rotate_wait_sec_);
  ptr_navigation_mediator_->getParam("path_follower.over_time_wait_dist",
                                     over_time_wait_dist_,
                                     over_time_wait_dist_);
  ReferState::getPtrInstance()->setPauseTime(avoid_wait_sec, recover_wait_sec,
                                             unreach_wait_sec);

  // 读取当前控制算法的最大速度值，后续绕障调速按照此最大速度的百分比做调整
  std::string ctrl_vel_name = "path_follower.controller_algorithm_params." +
                              local_controller_algorithm_;
  double max_x = 0.0, max_y = 0.0, max_th = 0.0;
  ptr_navigation_mediator_->getParam(ctrl_vel_name + ".max_x", max_x, 1.0);
  ptr_navigation_mediator_->getParam(ctrl_vel_name + ".max_y", max_y, 0.0);
  ptr_navigation_mediator_->getParam(ctrl_vel_name + ".max_th", max_th, 0.5);
  LOG(INFO) << "Max Vel(" << max_x << ", " << max_y << ", " << max_th << ")";
  // SpeedController::getPtrInstance()->setMaxVelocity(max_x, max_y, max_th);

  // 获取旋转角度的容忍值
  double head_length = 0;
  double tail_length = 0;
  ptr_navigation_mediator_->getParam("path_follower.yaw_goal_tolerance",
                                     yaw_tolerance_, 0.3);
  ptr_navigation_mediator_->getParam("path_follower.xy_goal_tolerance",
                                     xy_tolerance_, 0.3);
  ptr_navigation_mediator_->getParam("path_follower.head_length", head_length,
                                     0.3);
  ptr_navigation_mediator_->getParam("path_follower.tail_length", tail_length,
                                     0.3);
  ptr_path_recovery_->setMaxRecoverDistance(max_recover_distance_);
  RotateState::getPtrInstance()->setAngleTolerance(yaw_tolerance_);
  double ratate_dangerous_value = 1.8 * dangerous_value_ < 0.9 * LETHAL_OBSTACLE
                                      ? 1.8 * dangerous_value_
                                      : 0.9 * LETHAL_OBSTACLE;
  RotateState::getPtrInstance()->setDangerValue(ratate_dangerous_value);
  RotateState::getPtrInstance()->setFootprint(head_length, tail_length);

  planner_decision_config_.foot_print =
      SpeedDecisionBase::getInstance()->getBoundingbox("stop");

  ptr_navigation_mediator_->getParam(
      "path_follower.planner_decision.bussiness_dist",
      planner_decision_config_.bussiness_dist,
      planner_decision_config_.bussiness_dist);
  ptr_navigation_mediator_->getParam(
      "path_follower.planner_decision.dynamic_dist",
      planner_decision_config_.dynamic_dist,
      planner_decision_config_.dynamic_dist);
  ptr_navigation_mediator_->getParam(
      "path_follower.planner_decision.bussiness_range",
      planner_decision_config_.bussiness_range,
      planner_decision_config_.bussiness_range);
  ptr_navigation_mediator_->getParam("path_follower.planner_decision.k_safe",
                                     planner_decision_config_.k_safe,
                                     planner_decision_config_.k_safe);
  ptr_navigation_mediator_->getParam(
      "path_follower.planner_decision.k_bussiness",
      planner_decision_config_.k_bussiness,
      planner_decision_config_.k_bussiness);
  ptr_navigation_mediator_->getParam("path_follower.planner_decision.k_dynamic",
                                     planner_decision_config_.k_dynamic,
                                     planner_decision_config_.k_dynamic);
  ptr_navigation_mediator_->getParam("path_follower.planner_decision.k_mission",
                                     planner_decision_config_.k_mission,
                                     planner_decision_config_.k_mission);
  ptr_navigation_mediator_->getParam("path_follower.planner_decision.map_range",
                                     planner_decision_config_.map_range,
                                     planner_decision_config_.map_range);
  ptr_navigation_mediator_->getParam("path_follower.planner_decision.step",
                                     planner_decision_config_.step,
                                     planner_decision_config_.step);
  ptr_navigation_mediator_->getParam(
      "path_follower.planner_decision.check_map_range",
      planner_decision_config_.check_map_range,
      planner_decision_config_.check_map_range);
  ptr_navigation_mediator_->getParam("path_follower.planner_decision.min_cost",
                                     planner_decision_config_.min_cost,
                                     planner_decision_config_.min_cost);
  ptr_navigation_mediator_->getParam("path_follower.planner_decision.max_cost",
                                     planner_decision_config_.max_cost,
                                     planner_decision_config_.max_cost);

  planner_decision_config_.dangerous_value = dangerous_value_;
}

void PathFollower::setPathManagerParams() {
  bool optimize_local_path = true;
  bool calc_vel_limit_ctrl = true;
  bool calc_optimize_ctrl = true;
  double v_limit = 0.0;
  double w_limit = 0.0;
  double acc_limit = 0.0;
  double sample_delta_time = 0.0;
  double bsp_delta_time = 0.0;

  double lambda1 = 0;
  double lambda2 = 0;
  double lambda3 = 0;
  double lambda4 = 0.0;
  bool use_endpt_opt = false;
  double dist0 = 0;
  double min_rot_radiu = 0;
  int max_iter_nums = 0;
  double max_iter_time = 0;

  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.optimize_local_path", optimize_local_path,
      true);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.calc_vel_limit_ctrl", calc_vel_limit_ctrl,
      true);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.calc_optimize_ctrl", calc_optimize_ctrl,
      true);
  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.v_limit",
                                     v_limit, 1.2);
  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.w_limit",
                                     w_limit, 0.6);
  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.acc_limit",
                                     acc_limit, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.sample_delta_time", sample_delta_time,
      0.1);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.bsp_delta_time", bsp_delta_time, 0.4);

  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.lambda1",
                                     lambda1, 1.0);
  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.lambda2",
                                     lambda2, 10.0);
  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.lambda3",
                                     lambda3, 10.0);
  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.lambda4",
                                     lambda4, 10.0);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.use_endpt_opt", use_endpt_opt, false);
  ptr_navigation_mediator_->getParam("path_optimize.local_optimizer.dist",
                                     dist0, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.min_rot_radiu", min_rot_radiu, 0.4);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.max_iteration_num", max_iter_nums, 10);
  ptr_navigation_mediator_->getParam(
      "path_optimize.local_optimizer.max_iteration_time", max_iter_time, 0.1);
  ptr_path_manager_->setMaxDistance(max_distance_to_path_);
  ptr_path_manager_->setOptimizeLocalPathFlag(
      optimize_local_path, calc_vel_limit_ctrl, calc_optimize_ctrl);

  ptr_path_manager_->setOptimizeLocalParams(v_limit, w_limit, acc_limit,
                                            sample_delta_time, bsp_delta_time);
  ptr_path_manager_->setOptCtrlPointsParams(lambda1, lambda2, lambda3, lambda4,
                                            use_endpt_opt, dist0, min_rot_radiu,
                                            max_iter_nums, max_iter_time);
  double edge_dist = 0;
  double access_interval = 0;
  ptr_navigation_mediator_->getParam("path_follower.pit.access_interval",
                                     access_interval, access_interval);
  ptr_navigation_mediator_->getParam("path_follower.pit.edge_dist", edge_dist,
                                     edge_dist);
  ptr_path_manager_->setPitParams(edge_dist, access_interval);
}

void PathFollower::setTargetPose(const Pose2d &target_pose) {
  // 优化停靠精度
  ptr_goal_supor_->setTargetPose(target_pose);
}

void PathFollower::setReferencePath(std::shared_ptr<SubPath> reference_path) {
  // SubPath path_pose = reference_path;
  if (!forward_) {
    for (unsigned int i = 0; i < reference_path->wps.size(); i++) {
      reference_path->wps[i].yaw =
          AngleCalculate::normalizeAngle(M_PI + reference_path->wps[i].yaw);
    }

    for (unsigned int i = 0; i < reference_path->wps.size(); i++) {
      reference_path->wpis[i].v = -reference_path->wpis[i].v;
    }
  }
  ptr_path_manager_->setReferPath(reference_path);
  ptr_goal_supor_->setSuperviseReferPath(reference_path->wps);
  ptr_time_supor_->setSuperviseReferPath(reference_path->wps);
  ptr_local_controller_->setGlobalGoal(reference_path->wps.back());
  PlannerDecision::getInstance()->inputReferPath(reference_path);

  // 设置任务路径的角度和起点属性，Rotate状态会根据此旋转到目标角度
  RotateState::getPtrInstance()->setTargetAngleWithType(
      ptr_path_manager_->getReferPathAngle(), "start");
  changeState(RotateState::getPtrInstance());
}

FollowerStateRes PathFollower::followerUpdate() {
  /***************************************************************************
  //
  根据状态指针指向，处理对应的状态，状态之间的切换条件和关系，详细请看kb文档（2.4状态关系图）
  // 文档链接： https://kb.cvte.com/pages/viewpage.action?pageId=218851057
  ****************************************************************************/
  // 对应状态处理 当前是什么状态切换为什么指针
  // refer->跟随 recover->后退
  // timeOutCallBack();

  updateVisCtrlPath();

  return ptr_current_state_->dealState(this);
}

PathFollowerStates PathFollower::controllerExecute(
    MoveCommand &move_command, MCMDSTATE &move_command_status) {
  PathFollowerStates current_state = getState();
  showMeDebugMessage();  // 调试用，将机器人位置和速度信息打印
  // 根据类型选择控制
  // timeOut CallBack
  switch (current_state) {
    case PathFollowerStates::DONE: {
      doneZeroMoveCmd(move_command, move_command_status);
    } break;

    case PathFollowerStates::PAUSE: {
      errorZeroMoveCmd(move_command, move_command_status);
    } break;

    case PathFollowerStates::ROTATE: {
      processRotateCtrl(move_command, move_command_status);
    } break;

    case PathFollowerStates::RECOVER: {
      processRecoverCtrl(move_command, move_command_status);
    } break;
    case PathFollowerStates::PASS_PIT:
    case PathFollowerStates::FOLLOW_REFERENCE:
    case PathFollowerStates::FOLLOW_LOCAL: {
      // 正常跟随路径，lypu闭环 或 Stanley贴边
      processFollowCtrl(move_command, move_command_status);
    } break;

    default:
      break;
  }
  return current_state;
}

void PathFollower::processRotateCtrl(MoveCommand &move_command,
                                     MCMDSTATE &move_command_status) {
  double w = getRotateVelocity();
  if (fabs(w) < 1e-5) {
    setMoveCmd(move_command, 0.0, 0.0, 0.0);
    move_command_status = MCMDSTATE::OBSTACLE_STOP;
    if (RotateState::getPtrInstance()->getRotateType() == "start" &&
        SpeedDecisionBase::getInstance()->getSpeedResult().speed_radio ==
            SpeedRadio::STOP) {
      LOG(INFO) << "rotate obstacle stop change to refer";
      changeState(ReferState::getPtrInstance());
    }
  } else {
    if (fabs(w) < min_rotate_threshold_) {
      w = w > 0 ? min_rotate_threshold_ : -min_rotate_threshold_;
    }
    setMoveCmd(move_command, 0.0, 0.0, w);
  }
}

void PathFollower::processRecoverCtrl(MoveCommand &move_command,
                                      MCMDSTATE &move_command_status) {
  double recover_v = 0.00;
  double recover_w = 0.00;
  // recover 获取速度
  if (ptr_path_recovery_->calculateRecoverVelocity(recover_v, recover_w) ==
      RECOVERYERROR) {
    LOG(ERROR) << "recover to path failed!! （recover）";
    setMoveCmd(move_command, 0.0, 0.0, 0.0);
    move_command_status = MCMDSTATE::RECOVERY_ERROR;
    return;
  }
  LOG(INFO) << "recover velocity: = " << recover_v;
  // recover 发布速度
  setMoveCmd(move_command, recover_v, 0.0, recover_w);
}

void PathFollower::processFollowCtrl(MoveCommand &move_command,
                                     MCMDSTATE &move_command_status) {
  PathFollowerStates current_state = getState();
  if (current_state == PathFollowerStates::FOLLOW_REFERENCE) {
    if (ptr_path_manager_->isNeedUpdateReferCtrlPath()) {
      SpeedDecisionBase::getInstance()->setReferPath(
          ptr_path_manager_->getReferPath());
      if (!ptr_local_controller_->setPath(ptr_path_manager_->getReferPath(),
                                          ptr_path_manager_->getReferIndex())) {
        errorZeroMoveCmd(move_command, move_command_status);
        return;
      }
    }
    ptr_path_manager_->setReferIndex(ptr_local_controller_->getReferIndex());
    SpeedDecisionBase::getInstance()->setReferIndex(
        ptr_local_controller_->getReferIndex());
  } else if (current_state == PathFollowerStates::FOLLOW_LOCAL ||
             current_state == PathFollowerStates::PASS_PIT) {
    // ptr_path_manager_->setLocalIndex(ptr_local_controller_->getReferIndex());
    if (ptr_path_manager_->isNeedUpdateLocalCtrlPath()) {
      SpeedDecisionBase::getInstance()->setReferPath(
          ptr_path_manager_->getLocalPath());
      if (!ptr_local_controller_->setPath(ptr_path_manager_->getLocalPath(),
                                          ptr_path_manager_->getLocalIndex())) {
        errorZeroMoveCmd(move_command, move_command_status);
        return;
      }
    }
    ptr_path_manager_->setLocalIndex(ptr_local_controller_->getReferIndex());
    double avoider_begin_index = ptr_path_manager_->getAvoiderReferIndex();
    double target_index = ptr_path_manager_->getTargetIndex();
    double local_index = ptr_path_manager_->getLocalIndex();
    size_t local_path_size = ptr_path_manager_->getLocalPathSize();
    if (local_path_size > 0) {
      size_t map_refer_index = static_cast<size_t>(
          avoider_begin_index +
          local_index * (target_index - avoider_begin_index) / local_path_size);
      ptr_path_manager_->setReferIndex(map_refer_index);
      LOG(INFO) << "avoider begin: " << avoider_begin_index
                << " target index: " << target_index
                << " local index: " << local_index
                << " map refer index: " << map_refer_index;
    }
    SpeedDecisionBase::getInstance()->setReferIndex(
        ptr_local_controller_->getReferIndex());
  }
  localControllerUpate(move_command, move_command_status);
}

bool PathFollower::updateVisCtrlPath() {
  if (!isTimeToUpdateVisCtrlPath()) {
    return true;
  }
  // 根据不同的状态，将待执行的任务赋值给controller路径，用来计算使用
  PathFollowerStates current_state = getState();
  if (current_state == PathFollowerStates::FOLLOW_REFERENCE) {
    ptr_path_manager_->updateCtrolPath("refer");
  } else if (current_state == PathFollowerStates::FOLLOW_LOCAL ||
             current_state == PathFollowerStates::PASS_PIT) {
    ptr_path_manager_->updateCtrolPath("local");
  } else {
    LOG(ERROR) << "This State needn't to update Ctrl path.";
    return false;
  }

  return true;
}

void PathFollower::localControllerUpate(MoveCommand &move_command,
                                        MCMDSTATE &move_command_status) {
  move_command_status = ptr_local_controller_->computeMoveComand(move_command);
}

bool PathFollower::isTimeToUpdateVisCtrlPath() {
  int frequence = 20;
  if (++update_ctrl_path_count_ < frequence) {
    return false;
  }
  update_ctrl_path_count_ = 0;
  return true;
}

void PathFollower::showMeDebugMessage() {
  static bool first_pose = false;
  static Pose2d last_pose;

  Pose2d pose = RobotInfo::getPtrInstance()->getCurrentPose();
  Velocity vel = RobotInfo::getPtrInstance()->getCurrentVel();

  if (!first_pose) {
    first_pose = true;
    last_pose = pose;
  }

  LOG(INFO) << "Cur pose & vel: (" << pose.getX() << ", " << pose.getY() << ", "
            << pose.getYaw() << " || " << vel.d_x << "m/s, " << vel.d_y << ", "
            << vel.d_yaw << "rad/s)";

  Pose2d diff_pose = last_pose.inverse() * pose;
  LOG(INFO) << "diff pose: (" << diff_pose.getX() << ", " << diff_pose.getY()
            << ", " << diff_pose.getYaw();
  last_pose = pose;
}

void PathFollower::setMoveCmd(MoveCommand &move_command, const double &x,
                              const double &y, const double &th) {
  move_command.setVelX(x);
  move_command.setVelY(y);
  move_command.setVelTH(th);
}

void PathFollower::errorZeroMoveCmd(MoveCommand &move_command,
                                    MCMDSTATE &move_command_status) {
  setMoveCmd(move_command, 0.0, 0.0, 0.0);
  move_command_status = MCMDSTATE::ERROR;
}

void PathFollower::normalZeroMoveCmd(MoveCommand &move_command,
                                     MCMDSTATE &move_command_status) {
  setMoveCmd(move_command, 0.0, 0.0, 0.0);
  move_command_status = MCMDSTATE::OKAY;
}

void PathFollower::doneZeroMoveCmd(MoveCommand &move_command,
                                   MCMDSTATE &move_command_status) {
  setMoveCmd(move_command, 0.0, 0.0, 0.0);
  move_command_status = MCMDSTATE::REACHED_GOAL;
}

void PathFollower::setRobotType(ROBOTTYPE type) {
  RobotInfo::getPtrInstance()->setRobotType(type);
}

void PathFollower::setCurrentPose(const Pose2d &current_pose) {
  RobotInfo::getPtrInstance()->setCurrentPose(current_pose);
  ptr_local_controller_->setCurrentPose(current_pose);
}

void PathFollower::setCurrentVelocity(const Velocity &current_velocity) {
  RobotInfo::getPtrInstance()->setCurrentVelocity(current_velocity);
  ptr_local_controller_->setCurrentVelocity(current_velocity);
}

PathFollowerStates PathFollower::getState() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return ptr_current_state_->getState();
}

void PathFollower::changeState(const std::shared_ptr<State> state_ptr) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  ptr_current_state_ = state_ptr;
}

void PathFollower::setRotateVelocity(const double &velocity) {
  std::lock_guard<std::mutex> lock(rotate_velocity_mutex_);
  rotate_velocity_ = velocity;
}

double PathFollower::getRotateVelocity() {
  std::lock_guard<std::mutex> lock(rotate_velocity_mutex_);
  return rotate_velocity_;
}

double PathFollower::getControllerPathRemainLength() {
  return ptr_local_controller_->getRemainCtrlPathLength();
}

double PathFollower::getReferRemainLength() {
  PathFollowerStates follow_state = getState();
  if (follow_state == PathFollowerStates::FOLLOW_REFERENCE) {
    size_t target_index = ptr_path_manager_->getReferIndex();
    std::shared_ptr<SubPath> refer_path = ptr_path_manager_->getReferPath();
    if (target_index < refer_path->wpis.size() && refer_path != nullptr) {
      double target_dist = refer_path->wpis.back().path_length -
                           refer_path->wpis[target_index].path_length;
      return target_dist;
    } else {
      LOG(ERROR) << "refer out of range";
      return 0.0;
    }
    // return getControllerPathRemainLength();
  } else if (follow_state == PathFollowerStates::FOLLOW_LOCAL) {
    size_t target_index = ptr_path_manager_->getTargetIndex();
    std::shared_ptr<SubPath> refer_path = ptr_path_manager_->getReferPath();
    if (target_index < refer_path->wpis.size() && refer_path != nullptr) {
      double target_dist = refer_path->wpis.back().path_length -
                           refer_path->wpis[target_index].path_length;
      return target_dist + getControllerPathRemainLength();
    } else {
      LOG(ERROR) << "local out of range";
    }
  } else {
    std::shared_ptr<SubPath> refer_path = ptr_path_manager_->getReferPath();
    if (refer_path != nullptr && !refer_path->wpis.empty() &&
        ptr_path_manager_->getReferIndex() < refer_path->wpis.size()) {
      return refer_path->wpis.back().path_length -
             refer_path->wpis[ptr_path_manager_->getReferIndex()].path_length;
    } else {
      return 0.0;
    }
  }
}
void PathFollower::recoverClear() {
  RecoverState::getPtrInstance()->clearIntoState();
  return ptr_path_recovery_->recoverClear();
}

void PathFollower::setAbsReachFlag(bool is_abs_reach) {
  PauseState::getPtrInstance()->setAbsReachFlag(is_abs_reach);
  ptr_time_supor_->setAbsReach(is_abs_reach);
}

bool PathFollower::checkIfNeedToRecover() {
  return ptr_path_recovery_->checkIfNeedToRecover();
}

std::vector<Pose2d> PathFollower::getOriginalLocalPath() {
  return ptr_path_manager_->getOriginalLocalPath();
}

std::vector<Pose2d> PathFollower::getCtrlPath() {
  return ptr_path_manager_->getCtrolPath()->wps;
}

}  // namespace CVTE_BABOT
