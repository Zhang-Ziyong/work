/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file logic_controller.hpp
 *
 *@brief 用于一些逻辑控制
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-16
 ************************************************************************/
#ifndef __LOGIC_CONTROLLER_HPP
#define __LOGIC_CONTROLLER_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <vector>

#include "navigation_mediator.hpp"
#include "path_follower.hpp"
#include "planner_utils.hpp"
#include "pose2d/pose2d.hpp"
#include "slope_mark.hpp"
#include "speed_decision_base.hpp"

namespace CVTE_BABOT {

class Costmap2d;
class GlobalPlanner;

enum class PlanErrorMsg {
  OK = 0,
  START_OBSTACLE,
  TARGET_OBSTACLE,
  TARGET_UNKNOW,
  TARGET_OVERMAP,
  CALC_FAILED
};

class LogicController {
 public:
  LogicController() = delete;
  ~LogicController() = default;
  LogicController(std::shared_ptr<Costmap2d> costmap);

  SubPath setGoalPose(const Pose2d &goal_pose,
                      bool combine_local_costmap = false);
  SubPath makePlanBetweenPoints(const Pose2d &point_a, const Pose2d &point_b,
                                bool combine_local_costmap = false);
  SubPath makePlanToPoints(const Pose2d &target);
  bool setGoalPath(const SubPath &goal_path);
  void updateGlobalPlanner(bool combine_local_costmap = false);
  double calcPathResolution(const std::vector<Pose2d> &path);

  FollowerStateRes plannerUpdate();
  void controllerUpdate(MoveCommand &move_command,
                        MoveCommand::MoveCommandStatus &move_command_status);

  inline void setOptimizeMissionPath(std::shared_ptr<SubPath> opt_path) {
    std::lock_guard<std::mutex> lock(optimize_path_mutex_);
    optimize_mission_path_ = opt_path;
  }

  inline std::shared_ptr<SubPath> getOptimizeMissionPath() {
    std::lock_guard<std::mutex> lock(optimize_path_mutex_);
    return optimize_mission_path_;
  }

  inline std::vector<Pose2d> getOriginalLocalPath() {
    return ptr_path_follower_->getOriginalLocalPath();
  }

  inline std::vector<Pose2d> getCtrlPath() {
    return ptr_path_follower_->getCtrlPath();
  }

  /**
   * @brief 获取轨迹跟踪索引
   *
   * @return size_t
   */
  size_t getReferIndex() const { return ptr_path_follower_->getReferIndex(); }

  inline void updateCostMap(std::shared_ptr<Costmap2d> costmap) {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    ptr_costmap_2d_ = costmap;
    SpeedDecisionBase::getInstance()->setCostmap(costmap);
  }

  inline void updatePlannerCostMap() {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    ptr_path_follower_->updatePlanerCostMap(ptr_costmap_2d_);
  }

  void updatePlannerGlobalCostMap();

  inline void updateControllerCostMap() {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    ptr_path_follower_->updateControllerCostMap(ptr_costmap_2d_);
  }

  inline void setCurrentPose(const Pose2d &current_pose) {
    current_pose_mutex_.lock();
    current_pose_ = current_pose;
    current_pose_mutex_.unlock();
    ptr_path_follower_->setCurrentPose(current_pose);
  }

  inline void setRobotType(ROBOTTYPE type) {
    ptr_path_follower_->setRobotType(type);
  }

  inline void setCurrentVelocity(const Velocity &current_velocity) {
    current_velocity_mutex_.lock();
    current_velocity_ = current_velocity;
    current_velocity_mutex_.unlock();
    ptr_path_follower_->setCurrentVelocity(current_velocity);
  }

  inline PlanErrorMsg getPlanErrorMsg() { return plan_error_msg_; }

  inline void setForwardStatus(const bool forward) {
    forward_ = forward;
    ptr_path_follower_->setForwardStatus(forward);
  }

  inline void setPathStatus(MISSIONTYPE mission_type) {
    mission_type_ = mission_type;
    ptr_path_follower_->setPathStatus(mission_type);
  }

  inline void setFinalRotateFlag(bool need_final_rotate) {
    need_final_rotate_ = need_final_rotate;
    ptr_path_follower_->setFinalRotateFlag(need_final_rotate);
  }

  inline void setAbsReachFlag(bool is_abs_reach) {
    ptr_path_follower_->setAbsReachFlag(is_abs_reach);
  }

  inline double getCompletion() {
    return ptr_path_follower_->getPmPtr()->getCompletion();
  }

  inline double getReferRemainLength() {
    return ptr_path_follower_->getReferRemainLength();
  }
  inline Pose2d getCurrentGoalPose() { return goal_pose_; }
  double getPathEstimateTime(const SubPath &path);

 private:
  Pose2d current_pose_;
  Pose2d goal_pose_;
  Velocity current_velocity_;

  std::mutex current_pose_mutex_;
  std::mutex current_velocity_mutex_;
  std::mutex costmap_mutex_;
  std::mutex optimize_path_mutex_;

  std::shared_ptr<SubPath> optimize_mission_path_;

  std::shared_ptr<PathFollower> ptr_path_follower_ = nullptr;
  std::shared_ptr<NavigationMediator> ptr_navigation_mediator_ = nullptr;

  double path_resolution_ = 0.0;
  double max_ditance_to_path_ = 5.0;
  double max_path_resolution_ = 0.5;
  bool init_global_planner_ = false;

  ///< 路径优化相关参数
  bool optimize_global_path_ = true;
  bool global_calc_vel_limit_ctrl_ = true;
  bool global_optimize_ctrl_ = true;
  double global_v_limit_ = 0.0;
  double global_w_limit_ = 0.0;
  double global_acc_limit_ = 0.0;
  double global_bsp_delta_time_ = 0.0;
  double global_sample_delta_time_ = 0.0;
  double controller_frequency_ = 10.0;
  double edge_dist_ = -0.5;

  double lambda1_ = 0;
  double lambda2_ = 0;
  double lambda3_ = 0;
  double lambda4_ = 0;
  bool use_endpt_opt_ = false;
  double obstacle_dist0_ = 0;
  double min_rot_radiu_ = 0;
  int max_iter_nums_ = 0;
  int jump_point_size_ = 0;
  double max_iter_time_ = 0;

  ///< 贴边路径优化相关参数
  bool optimize_edge_path_ = true;
  bool edge_calc_vel_limit_ctrl_ = true;
  bool edge_optimize_ctrl_ = true;
  double edge_v_limit_ = 0.0;
  double edge_w_limit_ = 0.0;
  double edge_acc_limit_ = 0.0;
  double edge_bsp_delta_time_ = 0.0;
  double edge_sample_delta_time_ = 0.0;

  double edge_lambda1_ = 0;
  double edge_lambda2_ = 0;
  double edge_lambda3_ = 0;
  double edge_lambda4_ = 0;
  bool edge_use_endpt_opt_ = false;
  double edge_obstacle_dist0_ = 0;
  double edge_min_rot_radiu_ = 0;
  int edge_max_iter_nums_ = 0;
  int edge_jump_point_size_ = 0;
  double edge_max_iter_time_ = 0;

  bool forward_ = true;  // 是否反向执行轨迹
  // bool normal_path_ = true;  // 区分是否贴边，是->不贴边，否->贴边
  MISSIONTYPE mission_type_;  //任务类型
  bool need_final_rotate_ = false;

  std::shared_ptr<Costmap2d> ptr_global_costmap_2d_ =
      nullptr;                                           ///< 全局代价地图
  std::shared_ptr<Costmap2d> ptr_costmap_2d_ = nullptr;  ///< 局部代价地图

  std::string global_planner_algorithm_ = "";
  PlanErrorMsg plan_error_msg_ = PlanErrorMsg::OK;
  std::shared_ptr<GlobalPlanner> ptr_global_planner_ = nullptr;

  std::shared_ptr<SpeedDecisionBase> ptr_speed_decision_ = nullptr;

  void getParams();

  bool isValidPose(const Pose2d &start_pose, const Pose2d &goal_pose);
  bool isValidGoalPath(const std::vector<Pose2d> &goal_path);
  bool makeGlobalPlan(const Pose2d &start_pose, const Pose2d &goal_pose,
                      SubPath &result);

  SubPath optimizeGlobalPath(SubPath path_pose);

  inline Pose2d getCurrentPose() {
    current_pose_mutex_.lock();
    Pose2d current_pose = current_pose_;
    current_pose_mutex_.unlock();
    return current_pose;
  }

  double calcPoseToPathDistance(const Pose2d &pose,
                                const std::vector<Pose2d> &path, int &index);
};

}  // namespace CVTE_BABOT

#endif  // __LOGIC_CONTROLLER_HPP
