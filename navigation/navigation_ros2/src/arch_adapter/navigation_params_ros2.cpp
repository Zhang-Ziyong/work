/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file
 *
 *@brief
 *
 *@author chenmingjian (chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2020-01-16
 ************************************************************************/
#include <glog/logging.h>

#include "arch_adapter/navigation_params_ros2.hpp"

namespace CVTE_BABOT {
NavigationParamsRos2::NavigationParamsRos2(
    const rclcpp::Node::SharedPtr ptr_node,
    const std::shared_ptr<NavigationParameters> ptr_navigation_params)
    : ptr_navigation_params_(ptr_navigation_params), node_(ptr_node) {}

void NavigationParamsRos2::loadParams() {
  loadLogicControllerParams();
  loadPathFollowerParams();
  loadDWAControllerParams();
  loadPIDControllerParams();
  loadLYPUControllerParams();
  loadMPCcontrollerParams();
  loadStanleyControllerParams();
  loadPathOptimizeParams();
  loadGlobalParams();
  loadNarrowParams();
  loadPitParams();
  loadPlannerDecisionParams();
  loadAdditionCleanParams();
}

void NavigationParamsRos2::loadAdditionCleanParams() {
  double waypoint_resolution = 0.3;
  double changeColosCost = 80.0;
  double turn_offset = 0.9;
  double obs_offset = 0.4;
  double inflation_dis = 0.6;
  double dis_of_path = 0.7;
  getAndSetParam("addition_clean.waypoint_resolution", waypoint_resolution,
                 0.3);
  getAndSetParam("addition_clean.changeColosCost", changeColosCost, 80.0);
  getAndSetParam("addition_clean.turn_offset", turn_offset, 0.9);
  getAndSetParam("addition_clean.obs_offset", obs_offset, 0.4);
  getAndSetParam("addition_clean.inflation_dis", inflation_dis, 0.6);
  getAndSetParam("addition_clean.dis_of_path", dis_of_path, 0.7);
}

void NavigationParamsRos2::loadGlobalParams() {
  double planner_frequence = 0.0;
  double controller_frequency = 10.0;
  double edge_dist = -0.43;
  getAndSetParam("planner_frequency", planner_frequence, 2.0);
  getAndSetParam("controller_frequency", controller_frequency, 10.0);
  getAndSetParam("edge_dist", edge_dist, -0.43);
}

void NavigationParamsRos2::loadPathOptimizeParams() {
  bool optimize_global_path = true;
  bool optimize_local_path = false;
  bool optimize_edge_path = true;

  bool global_calc_vel_limit_ctrl = false;
  bool global_optimize_ctrl = false;
  double global_v_limit = 0.0;
  double global_w_limit = 0.0;
  double global_acc_limit = 0.0;
  double global_bsp_delta_time = 0.0;
  double global_sample_delta_time = 0.0;
  double global_lambda1 = 0.0;
  double global_lambda2 = 0.0;
  double global_lambda3 = 0.0;
  double global_lambda4 = 0.0;
  bool global_use_endpt_opt = false;
  double global_dist = 0.0;
  double global_min_rot_radiu = 0.0;
  double global_max_iteration_time = 0.0;
  int global_max_iteration_num = 0;
  int jump_point_size = 0;

  bool local_calc_vel_limit_ctrl = false;
  bool local_optimize_ctrl = false;
  double local_v_limit = 0.0;
  double local_w_limit = 0.0;
  double local_acc_limit = 0.0;
  double local_bsp_delta_time = 0.0;
  double local_sample_delta_time = 0.0;
  double local_lambda1 = 0.0;
  double local_lambda2 = 0.0;
  double local_lambda3 = 0.0;
  double local_lambda4 = 0.0;
  bool local_use_endpt_opt = false;
  double local_dist = 0.0;
  double local_min_rot_radiu = 0.0;
  int local_max_iteration_num = 0;
  double local_max_iteration_time = 0.0;

  bool edge_calc_vel_limit_ctrl = false;
  bool edge_optimize_ctrl = false;
  double edge_v_limit = 0.0;
  double edge_w_limit = 0.0;
  double edge_acc_limit = 0.0;
  double edge_bsp_delta_time = 0.0;
  double edge_sample_delta_time = 0.0;
  double edge_lambda1 = 0.0;
  double edge_lambda2 = 0.0;
  double edge_lambda3 = 0.0;
  double edge_lambda4 = 0.0;
  bool edge_use_endpt_opt = false;
  double edge_dist = 0.0;
  double edge_min_rot_radiu = 0.0;
  double edge_max_iteration_time = 0.0;
  int edge_max_iteration_num = 0;
  int edge_jump_point_size = 0;

  getAndSetParam("path_optimize.global_optimizer.optimize_global_path",
                 optimize_global_path, true);
  getAndSetParam("path_optimize.global_optimizer.calc_vel_limit_ctrl",
                 global_calc_vel_limit_ctrl, true);
  getAndSetParam("path_optimize.global_optimizer.calc_optimize_ctrl",
                 global_optimize_ctrl, true);
  getAndSetParam("path_optimize.global_optimizer.v_limit", global_v_limit, 1.2);
  getAndSetParam("path_optimize.global_optimizer.w_limit", global_w_limit, 0.6);
  getAndSetParam("path_optimize.global_optimizer.acc_limit", global_acc_limit,
                 0.6);
  getAndSetParam("path_optimize.global_optimizer.bsp_delta_time",
                 global_bsp_delta_time, 0.4);
  getAndSetParam("path_optimize.global_optimizer.sample_delta_time",
                 global_sample_delta_time, 0.1);
  getAndSetParam("path_optimize.global_optimizer.lambda1", global_lambda1, 0.1);
  getAndSetParam("path_optimize.global_optimizer.lambda2", global_lambda2,
                 10.0);
  getAndSetParam("path_optimize.global_optimizer.lambda3", global_lambda3, 0.0);
  getAndSetParam("path_optimize.global_optimizer.lambda4", global_lambda4, 0.0);
  getAndSetParam("path_optimize.global_optimizer.use_endpt_opt",
                 global_use_endpt_opt, false);
  getAndSetParam("path_optimize.global_optimizer.dist", global_dist, 1.0);
  getAndSetParam("path_optimize.global_optimizer.min_rot_radiu",
                 global_min_rot_radiu, 0.4);
  getAndSetParam("path_optimize.global_optimizer.max_iteration_num",
                 global_max_iteration_num, 5);
  getAndSetParam("path_optimize.global_optimizer.max_iteration_time",
                 global_max_iteration_time, 0.5);
  getAndSetParam("path_optimize.global_optimizer.jump_point_size",
                 jump_point_size, 1);

  getAndSetParam("path_optimize.edge_optimizer.optimize_global_path",
                 optimize_edge_path, true);
  getAndSetParam("path_optimize.edge_optimizer.calc_vel_limit_ctrl",
                 edge_calc_vel_limit_ctrl, true);
  getAndSetParam("path_optimize.edge_optimizer.calc_optimize_ctrl",
                 edge_optimize_ctrl, true);
  getAndSetParam("path_optimize.edge_optimizer.v_limit", edge_v_limit, 1.2);
  getAndSetParam("path_optimize.edge_optimizer.w_limit", edge_w_limit, 0.6);
  getAndSetParam("path_optimize.edge_optimizer.acc_limit", edge_acc_limit, 0.6);
  getAndSetParam("path_optimize.edge_optimizer.bsp_delta_time",
                 edge_bsp_delta_time, 0.4);
  getAndSetParam("path_optimize.edge_optimizer.sample_delta_time",
                 edge_sample_delta_time, 0.1);
  getAndSetParam("path_optimize.edge_optimizer.lambda1", edge_lambda1, 0.1);
  getAndSetParam("path_optimize.edge_optimizer.lambda2", edge_lambda2, 10.0);
  getAndSetParam("path_optimize.edge_optimizer.lambda3", edge_lambda3, 0.0);
  getAndSetParam("path_optimize.edge_optimizer.lambda4", edge_lambda4, 0.0);
  getAndSetParam("path_optimize.edge_optimizer.use_endpt_opt",
                 edge_use_endpt_opt, false);
  getAndSetParam("path_optimize.edge_optimizer.dist", edge_dist, 1.0);
  getAndSetParam("path_optimize.edge_optimizer.min_rot_radiu",
                 edge_min_rot_radiu, 0.4);
  getAndSetParam("path_optimize.edge_optimizer.max_iteration_num",
                 edge_max_iteration_num, 5);
  getAndSetParam("path_optimize.edge_optimizer.max_iteration_time",
                 edge_max_iteration_time, 0.5);
  getAndSetParam("path_optimize.edge_optimizer.jump_point_size",
                 edge_jump_point_size, 1);

  getAndSetParam("path_optimize.local_optimizer.optimize_local_path",
                 optimize_local_path, true);
  getAndSetParam("path_optimize.local_optimizer.calc_vel_limit_ctrl",
                 local_calc_vel_limit_ctrl, true);
  getAndSetParam("path_optimize.local_optimizer.calc_optimize_ctrl",
                 local_optimize_ctrl, true);
  getAndSetParam("path_optimize.local_optimizer.v_limit", local_v_limit, 1.2);
  getAndSetParam("path_optimize.local_optimizer.w_limit", local_w_limit, 0.6);
  getAndSetParam("path_optimize.local_optimizer.acc_limit", local_acc_limit,
                 0.6);
  getAndSetParam("path_optimize.local_optimizer.bsp_delta_time",
                 local_bsp_delta_time, 0.4);
  getAndSetParam("path_optimize.local_optimizer.sample_delta_time",
                 local_sample_delta_time, 0.1);
  getAndSetParam("path_optimize.local_optimizer.lambda1", local_lambda1, 1.0);
  getAndSetParam("path_optimize.local_optimizer.lambda2", local_lambda2, 10.0);
  getAndSetParam("path_optimize.local_optimizer.lambda3", local_lambda3, 10.0);
  getAndSetParam("path_optimize.local_optimizer.lambda4", local_lambda4, 100.0);
  getAndSetParam("path_optimize.local_optimizer.use_endpt_opt",
                 local_use_endpt_opt, false);
  getAndSetParam("path_optimize.local_optimizer.dist", local_dist, 1.5);
  getAndSetParam("path_optimize.local_optimizer.min_rot_radiu",
                 local_min_rot_radiu, 0.4);
  getAndSetParam("path_optimize.local_optimizer.max_iteration_num",
                 local_max_iteration_num, 10);
  getAndSetParam("path_optimize.local_optimizer.max_iteration_time",
                 local_max_iteration_time, 0.1);
}

void NavigationParamsRos2::loadLogicControllerParams() {
  double max_ditance_to_path = 0.0;
  double max_path_resolution = 0.0;

  std::string global_planner_algorithm;

  getAndSetParam("logic_controller.max_ditance_to_path", max_ditance_to_path,
                 10.0);
  getAndSetParam("logic_controller.max_path_resolution", max_path_resolution,
                 0.5);
  getAndSetParam("logic_controller.global_planner_algorithm",
                 global_planner_algorithm, std::string("dijkstra"));
}

void NavigationParamsRos2::loadPathFollowerParams() {
  std::string local_planner_algorithm;
  std::string local_controller_algorithm;
  std::string edge_controller_algorithm;
  double max_distance_to_path = 0.0;
  double max_angle_to_path = 0.0;
  double robot_rotate_radius = 0.0;
  double obstacle_avoid_distance = 0.0;
  double obstacle_avoid_behind = 0.0;
  bool robot_can_rotate = true;
  bool enable_recover = false;
  bool enable_rotate = false;
  int cell_distance_to_obs = 5;
  int car_waiting_count = 0;
  int dangerous_value = 150;
  int avoid_wait_sec = 0;
  int recover_wait_sec = 0;
  int unreach_wait_sec = 0;
  double xy_goal_tolerance = 0.0;
  double yaw_goal_tolerance = 0.0;
  double max_recover_dis = 0.0;
  double min_velocity_threshold = 0.0;
  double min_rotate_threshold = 0.0;
  double over_time_wait_sec = 20.0;
  double over_time_wait_dist = 2.0;
  double pause_wait_sec = 60.0;
  double rotate_wait_sec = 5.0;
  double head_length = 0.4;
  double tail_length = -0.1;

  getAndSetParam("path_follower.local_planner_algorithm",
                 local_planner_algorithm, std::string("dijkstra"));
  getAndSetParam("path_follower.local_controller_algorithm",
                 local_controller_algorithm, std::string("pid"));
  getAndSetParam("path_follower.edge_controller_algorithm",
                 edge_controller_algorithm, std::string("pid"));
  getAndSetParam("path_follower.max_distance_to_path", max_distance_to_path,
                 1.0);
  getAndSetParam("path_follower.max_angle_to_path", max_angle_to_path, 1.0);
  getAndSetParam("path_follower.obstacle_avoid_distance",
                 obstacle_avoid_distance, 4.0);
  getAndSetParam("path_follower.obstacle_avoid_behind", obstacle_avoid_behind,
                 2.0);
  getAndSetParam("path_follower.dangerous_value", dangerous_value, 150);
  getAndSetParam("path_follower.avoid_wait_sec", avoid_wait_sec, 0);
  getAndSetParam("path_follower.recover_wait_sec", recover_wait_sec, 2);
  getAndSetParam("path_follower.unreach_wait_sec", unreach_wait_sec, 5);
  getAndSetParam("path_follower.enable_recover", enable_recover, false);
  getAndSetParam("path_follower.enable_rotate", enable_rotate, false);
  getAndSetParam("path_follower.robot_can_rotate", robot_can_rotate, true);
  getAndSetParam("path_follower.robot_rotate_radius", robot_rotate_radius, 1.0);
  getAndSetParam("path_follower.max_recover_distance", max_recover_dis, 1.0);
  getAndSetParam("path_follower.cell_distance_to_obs", cell_distance_to_obs, 4);
  getAndSetParam("path_follower.car_waiting_count", car_waiting_count, 50);
  getAndSetParam("path_follower.xy_goal_tolerance", xy_goal_tolerance, 0.2);
  getAndSetParam("path_follower.yaw_goal_tolerance", yaw_goal_tolerance, 0.2);
  getAndSetParam("path_follower.min_velocity_threshold", min_velocity_threshold,
                 0.1);
  getAndSetParam("path_follower.min_rotate_threshold", min_rotate_threshold,
                 0.05);
  getAndSetParam("path_follower.over_time_wait_sec", over_time_wait_sec, 20.0);
  getAndSetParam("path_follower.over_time_wait_dist", over_time_wait_dist, 2.0);
  getAndSetParam("path_follower.pause_wait_sec", pause_wait_sec, 60.0);
  getAndSetParam("path_follower.rotate_wait_sec", rotate_wait_sec, 5.0);
  getAndSetParam("path_follower.head_length", head_length, 0.33);
  getAndSetParam("path_follower.tail_length", tail_length, -0.1);
}

void NavigationParamsRos2::loadPlannerDecisionParams() {
  double bussiness_dist = 1.5;
  double dynamic_dist = 2.0;
  double bussiness_range = 0.5;
  double k_safe = 0.5;
  double k_bussiness = 0.5;
  double k_dynamic = 0.5;
  double k_mission = 0.5;
  double map_range = 7.0;
  double step = 0.3;
  bool check_map_range = true;
  int max_cost = 0;
  int min_cost = 0;
  getAndSetParam("path_follower.planner_decision.bussiness_dist",
                 bussiness_dist, 1.5);
  getAndSetParam("path_follower.planner_decision.dynamic_dist", dynamic_dist,
                 2.0);
  getAndSetParam("path_follower.planner_decision.bussiness_range",
                 bussiness_range, 0.5);
  getAndSetParam("path_follower.planner_decision.k_safe", k_safe, 0.5);
  getAndSetParam("path_follower.planner_decision.k_bussiness", k_bussiness,
                 0.5);
  getAndSetParam("path_follower.planner_decision.k_dynamic", k_dynamic, 0.5);
  getAndSetParam("path_follower.planner_decision.k_mission", k_mission, 0.5);
  getAndSetParam("path_follower.planner_decision.map_range", map_range, 7.0);
  getAndSetParam("path_follower.planner_decision.step", step, 0.3);
  getAndSetParam("path_follower.planner_decision.check_map_range",
                 check_map_range, true);
  getAndSetParam("path_follower.planner_decision.max_cost", max_cost, 1);
  getAndSetParam("path_follower.planner_decision.min_cost", min_cost, 1);
}

void NavigationParamsRos2::loadDWAControllerParams() {
  double close_goal_distance = 0.0;
  double slow_down_v = 0.0;
  double slow_down_w = 0.0;
  double sim_distance = 0.0;
  double sim_time = 0.0;
  int vx_samples = 0;
  int vy_samples = 0;
  int vth_samples = 0;
  double max_x = 0.0;
  double min_x = 0.0;
  double max_y = 0.0;
  double min_y = 0.0;
  double max_th = 0.0;
  double min_th = 0.0;
  double acc_x = 0.0;
  double acc_y = 0.0;
  double acc_th = 0.0;

  getAndSetParam(
      "path_follower.controller_algorithm_params.dwa.close_goal_distance",
      close_goal_distance, 0.7);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.slow_down_v",
                 slow_down_v, 0.3);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.slow_down_w",
                 slow_down_w, 0.3);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.sim_distance",
                 sim_distance, 1.5);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.sim_time",
                 sim_time, 1.5);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.vx_samples",
                 vx_samples, 3);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.vy_samples",
                 vy_samples, 5);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.vth_samples",
                 vth_samples, 10);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.max_x", max_x,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.min_x", min_x,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.max_y", max_y,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.min_y", min_y,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.acc_x", acc_x,
                 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.acc_y", acc_y,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.acc_th", acc_th,
                 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.max_th", max_th,
                 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.dwa.min_th", min_th,
                 0.0);
}

void NavigationParamsRos2::loadPIDControllerParams() {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double kt = 0.0;
  double wp_tolerance = 0.0;
  double max_steer = 0;
  double car_length = 0;
  double steer_slow_threshold = 0;
  double back_multiple = 0.0;
  double max_x = 0.0;
  double min_x = 0.0;
  double max_y = 0.0;
  double min_y = 0.0;
  double max_th = 0.0;
  double min_th = 0.0;
  double acc_x = 0.0;
  double acc_y = 0.0;
  double acc_th = 0.0;

  getAndSetParam("path_follower.controller_algorithm_params.pid.kp", kp, 1.5);
  getAndSetParam("path_follower.controller_algorithm_params.pid.ki", ki, 0.001);
  getAndSetParam("path_follower.controller_algorithm_params.pid.kd", kd, 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.kt", kt, 0.03);
  getAndSetParam("path_follower.controller_algorithm_params.pid.wp_tolerance",
                 wp_tolerance, 1.5);
  getAndSetParam("path_follower.controller_algorithm_params.pid.max_steer",
                 max_steer, 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.car_length",
                 car_length, 0.38);
  getAndSetParam(
      "path_follower.controller_algorithm_params.pid.steer_slow_threshold",
      steer_slow_threshold, 0.25);
  getAndSetParam("path_follower.controller_algorithm_params.pid.back_multiple",
                 back_multiple, 1.5);
  getAndSetParam("path_follower.controller_algorithm_params.pid.max_x", max_x,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.min_x", min_x,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.max_y", max_y,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.min_y", min_y,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.max_th", max_th,
                 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.pid.min_th", min_th,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.acc_x", acc_x,
                 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.pid.acc_y", acc_y,
                 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.pid.acc_th", acc_th,
                 0.5);
}

void NavigationParamsRos2::loadLYPUControllerParams() {
  double dKp = 1;
  double dKx = 1;
  double dKy = 1;
  double dKq = 1;
  double dKd = 1;
  double dKth = 1;
  double max_target_v = 0.5;
  double max_target_w = 1.0;
  double min_target_v = 0.1;
  double min_target_w = 0.1;
  double max_v = 0.5;
  double max_w = 1.0;
  double min_v = 0.05;
  double min_w = 0.05;
  double acc = 0.5;
  double a_acc = 1.0;
  getAndSetParam("path_follower.controller_algorithm_params.lypu.dKp", dKp,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.dKx", dKx,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.dKy", dKy,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.dKq", dKq,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.dKd", dKd,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.dKth", dKth,
                 1.0);
  // getAndSetParam("path_follower.controller_algorithm_params.lypu.max_target_v",
  //                max_target_v, 1.0);
  // getAndSetParam("path_follower.controller_algorithm_params.lypu.max_target_w",
  //                max_target_w, 1.0);
  // getAndSetParam("path_follower.controller_algorithm_params.lypu.min_target_v",
  //                min_target_v, 0.0);
  // getAndSetParam("path_follower.controller_algorithm_params.lypu.min_target_w",
  //                min_target_w, 0.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.max_x", max_v,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.max_th", max_w,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.min_x", min_v,
                 0.01);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.min_th", min_w,
                 0.02);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.acc", acc,
                 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.lypu.a_acc", a_acc,
                 1.0);
}

void NavigationParamsRos2::loadStanleyControllerParams() {
  double kag = 0.2;
  double kp_e = 2.0;
  double kp_soft = 0.2;
  double kd_yaw = 0.5;
  double kp_v = 0.3;
  double car_length = 1.0;
  double max_w = 0.5;
  double max_vel = 0.5;
  double min_w = 0.04;
  double min_vel = 0.02;
  std::string edge_sensor;
  double psd_forward_install = 0.2;
  double psd_back_install = 0.2;

  getAndSetParam("path_follower.controller_algorithm_params.stanley.kag", kag,
                 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.kp_e", kp_e,
                 2.0);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.kp_soft",
                 kp_soft, 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.kd_yaw",
                 kd_yaw, 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.kp_v", kp_v,
                 0.3);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.car_length",
                 car_length, 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.max_w",
                 max_w, 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.max_vel",
                 max_vel, 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.min_w",
                 min_w, 0.04);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.min_vel",
                 min_vel, 0.02);
  getAndSetParam("path_follower.controller_algorithm_params.stanley.sensor",
                 edge_sensor, std::string("PSD"));
  getAndSetParam(
      "path_follower.controller_algorithm_params.stanley.psd_forward_install",
      psd_forward_install, 0.02);
  getAndSetParam(
      "path_follower.controller_algorithm_params.stanley.psd_back_install",
      psd_back_install, 0.02);

  getAndSetParam("path_follower.controller_algorithm_params.edge.kag", kag,
                 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.edge.kp_e", kp_e,
                 2.0);
  getAndSetParam("path_follower.controller_algorithm_params.edge.kp_soft",
                 kp_soft, 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.edge.kd_yaw",
                 kd_yaw, 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.edge.kp_v", kp_v,
                 0.3);
  getAndSetParam("path_follower.controller_algorithm_params.edge.car_length",
                 car_length, 1.0);
  getAndSetParam("path_follower.controller_algorithm_params.edge.max_w", max_w,
                 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.edge.max_vel",
                 max_vel, 0.5);
  getAndSetParam("path_follower.controller_algorithm_params.edge.min_w", min_w,
                 0.04);
  getAndSetParam("path_follower.controller_algorithm_params.edge.min_vel",
                 min_vel, 0.02);
  getAndSetParam("path_follower.controller_algorithm_params.edge.sensor",
                 edge_sensor, std::string("PSD"));
  getAndSetParam(
      "path_follower.controller_algorithm_params.edge.psd_forward_install",
      psd_forward_install, 0.02);
  getAndSetParam(
      "path_follower.controller_algorithm_params.edge.psd_back_install",
      psd_back_install, 0.02);
}

void NavigationParamsRos2::loadMPCcontrollerParams() {
  std::vector<double> matrix_q_prams;
  std::vector<double> matrix_r_prams;
  std::vector<double> u_bound_prams;
  double kappa_limit_cofficient;
  double min_velocity;
  int horizon;
  getAndSetParam("path_follower.controller_algorithm_params.mpc.matrix_q_prams",
                 matrix_q_prams, {10., 50., 100.});
  getAndSetParam("path_follower.controller_algorithm_params.mpc.matrix_r_prams",
                 matrix_r_prams, {5., 5.});
  getAndSetParam("path_follower.controller_algorithm_params.mpc.u_bound_prams",
                 u_bound_prams, {1., 0.7});
  getAndSetParam(
      "path_follower.controller_algorithm_params.mpc.kappa_limit_cofficient",
      kappa_limit_cofficient, 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.mpc.min_velocity",
                 min_velocity, 0.2);
  getAndSetParam("path_follower.controller_algorithm_params.mpc.horizon",
                 horizon, 0);
}

void NavigationParamsRos2::loadNarrowParams() {
  bool enabled;
  double width;
  double path_dir_length;
  double neighbor_expand_length;
  getAndSetParam("narrow.enabled", enabled, false);
  getAndSetParam("narrow.width", width, 1.2);
  getAndSetParam("narrow.path_dir_length", path_dir_length, 1.0);
  getAndSetParam("narrow.neighbor_expand_length", neighbor_expand_length, 1.0);
}

void NavigationParamsRos2::loadPitParams() {
  bool enabled;
  double temp;
  getAndSetParam("path_follower.pit.enabled", enabled, false);
  getAndSetParam("path_follower.pit.avoid_distance", temp, 1.0);
  getAndSetParam("path_follower.pit.access_interval", temp, 0.1);
  getAndSetParam("path_follower.pit.edge_dist", temp, 1.0);
}

}  // namespace CVTE_BABOT
