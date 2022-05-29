/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file dwa_planner.hpp
 *
 *@brief DWA算法的头文件函数
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-18
 ************************************************************************/

#ifndef __DWA_PLANNER_HPP
#define __DWA_PLANNER_HPP

#include "local_controller_base.hpp"
#include "trajectory.hpp"

namespace CVTE_BABOT {

class Costmap2d;

class Trajectory;
class SimpleTrajectoryGenerator;
class SimpleTrajectoryScorer;

class OscillationCostFunction;
class ObstacleCostFunction;
class MapGridCostFunction;
class TwirlingCostFunction;
class AngleCostFunction;

class ControllerLimits;

class RotationCheck;

class DWALocalController : public LocalController {
  friend class DWALocalControllerTest;

 public:
  DWALocalController(std::shared_ptr<Costmap2d> costmap);
  DWALocalController() = delete;
  DWALocalController(const DWALocalController &obj) = delete;
  DWALocalController &operator=(const DWALocalController &obj) = delete;
  ~DWALocalController() = default;

  /**
   * findBestPath
   * @brief
   *   根据机器人当前的速度和位置，在路径集合中寻找最优的可执行路径
   *
   * @param[in] global_pose-机器人位置
   * @param[in] global_vel-机器人速度
   * @param[in] footprint_spec-机器人的footpr
   * @param[out] best_traject-找到的最优路径
   * @return 评分最高的路径
   * */
  bool findBestPath(const Pose2d &global_pose,
                    const std::array<double, 3> &global_vel,
                    std::array<Pose2d, 4> &footprint_spec,
                    Trajectory &best_traject);

  /**
   * updatePlanAndLocalCosts
   * @brief
   *   更新DWA局部路径规划的需要跟踪的全局路径
   *
   * @param[in] global_pose-机器人位置
   * @param[in] new_plan-新的全局路径
   * */
  void updatePlanAndLocalCosts(const std::vector<Pose2d> &new_plan);
  /**
   * setPath
   * @brief
   *   设置新的全局路径
   *
   * @param[in] orig_global_plan-需要传入给DWA的新全局路径
   * @return true-代表设置成功
   * */
  bool setPath(std::shared_ptr<SubPath> path, size_t begin_index) override;

  /**
   * updateCostMap
   * @brief
   *   设置costmap
   *
   * @param[in] ptr_costmap-cost_map指针
   * */
  void updateCostMap(std::shared_ptr<Costmap2d> ptr_costmap) override;

  /**
   * getAllTraj
   * @brief
   *   获取DWA的所有采样路径，提供可视化调试使用
   *
   * @param[out] traj_result-所有路径结果
   * @return true - 代表此时有路径结果
   * */
  bool getAllTraj(std::vector<Trajectory> &traj_result);

  /**
   * computeMoveComand
   * @brief
   *   控制器根据当前位置和实时速度反馈，计算跟踪路径过程中需要执行的线速度和角速度
   *   计算之前确保已经将当前位置和速度通过setCurrentPose 和 setCurrentVelocity
   * 输入。
   *
   * @param[out] cmd_ptr-计算指令的集合
   * */
  MoveCommand::MoveCommandStatus computeMoveComand(MoveCommand &cmd) override;

  /**
   * setRobotFootPrint
   * @brief
   *   可以从costmap获取当前标示机器人轮廓的四个footprint点，并设置到dwa中进行采样评分
   * @param[in] footprint-机器人footprint
   * */
  void setRobotFootPrint(const std::array<Pose2d, 4> &footprint);

  /**
   * isGoalReached
   * @brief
   *   计算机器人当前位置与目标的距离，判断是否已经到达
   * @param[in] current_pose-机器人的当前位置
   * */
  bool isGoalReached(const Pose2d &current_pose);

  /**
   * isCloseToGoal
   * @brief
   *   计算机器人当前位置与目标的距离，判断是否已经比较靠近
   * @param[in] current_pose-机器人的当前位置
   * */
  bool isCloseToGoal(const Pose2d &current_pose);

  /**
   * getParams
   * @brief
   *   读取参数
   * */
  void getParams();

 private:
  bool ifNeedToRotate();
  double calculateRotateW();

  double oscillation_reset_dist_ = 0.25;
  double oscillation_reset_angle_ = 0.1;
  double max_scaling_factor_ = 0.2;
  double scaling_speed_ = 0.25;
  double pdist_scale_ = 100.0;             ///< path 比例
  double gdist_scale_ = 32.0;              ///< 目标点 比例
  double occdist_scale_ = 0.8;             ///< 障碍物 比例
  double forward_point_distance_ = 0.325;  ///< 前进点距离
  double map_resolution_ = 0.0;            ///< 地图分辨率
  double cheat_factor_ = 1.0;
  double close_goal_distance_ = 0.7;  ///< 低于这个值开始减速
  double sim_time_ = 0.0;
  int vx_samples_ = 0;
  int vy_samples_ = 0;
  int vth_samples_ = 0;

  std::mutex goal_mutex_;  ///< 获取与设置目标点锁

  std::array<int, 3> vsamples_;           ///< 速度样本
  std::array<Pose2d, 4> footprint_spec_;  ///< 机器人的footprint
  Pose2d goal_;  ///< 目标点（设置任务路径中的最后一点）

  std::shared_ptr<std::vector<Trajectory>> all_explored_ptr_ = nullptr;

  std::shared_ptr<MapGridCostFunction> path_costs_ = nullptr;
  std::shared_ptr<MapGridCostFunction> goal_costs_ = nullptr;
  std::shared_ptr<ObstacleCostFunction> obstacle_costs_ = nullptr;
  std::shared_ptr<MapGridCostFunction> goal_front_costs_ = nullptr;
  std::shared_ptr<MapGridCostFunction> alignment_costs_ = nullptr;
  std::shared_ptr<TwirlingCostFunction> twirling_costs_ = nullptr;
  std::shared_ptr<AngleCostFunction> angle_costs_ = nullptr;
  std::shared_ptr<OscillationCostFunction> oscillation_costs_ = nullptr;

  std::shared_ptr<SimpleTrajectoryGenerator> generator_ = nullptr;
  std::shared_ptr<SimpleTrajectoryScorer> scored_sampling_planner_ = nullptr;
};

}  // namespace CVTE_BABOT

#endif  // end of __DWA_PLANNER_HPP
