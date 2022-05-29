/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file car_avoider_supervisor.hpp
 *
 *@brief 实现获取车辆避让避让点的功能
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-27
 ************************************************************************/
#ifndef __CAR_AVOIDER_SUPERVISOR_HPP
#define __CAR_AVOIDER_SUPERVISOR_HPP
#include "processor_utils.hpp"
#include "supervisor.hpp"

namespace CVTE_BABOT {
class Costmap2d;
class PerceptionMap;
class PointSampler;
class PointValidityChecker;
class Visualizer;
/**
* CarAvoiderSupervisor
* @brief
* 1.实现车辆主动避让停靠点检测
* 2.先采样,再依据制定的规则评分得到最优点
**/
class CarAvoiderSupervisor : public Supervisor {
 public:
  CarAvoiderSupervisor(const unsigned int &ui_size_x,
                       const unsigned int &ui_size_y);
  ~CarAvoiderSupervisor();

  CarAvoiderSupervisor(const CarAvoiderSupervisor &) = delete;
  CarAvoiderSupervisor &operator=(const CarAvoiderSupervisor &) = delete;

  std::string getName() const override { return "CarAvoiderSupervisor"; }

  /**
   *supervise
   *@brief
   *输入机器人状态,输出状态及目标点等
   *
   *@param[in] state-机器人的一些状态,作为数据传递
   *@param[in] out-包含状态,目标点等
  **/
  void supervise(const SupervisorState &state, SupervisorResult &out) override;

 private:
  /**
   *checkIfNeedToAvoid
   *@brief
   *查询是否需要避让车辆,保存目标车辆的位置作为后续目标点评分依据
   *输出车辆避让专用的costmap, 因为会以汽车的位置作为规划的终点,
   *所以在上面清除了目标汽车的栅格, 不然无法规划.
   *
   *@param[in] current_pose-机器人当前的位置
   *@param[in] sub_path-当前执行的路径
   *@param[out] ptr_costmap-用于计算避让点的costmap
   *@param[out] car_pose-要避让的车辆的位置
   *@return true-需要避让车辆, false-不需要
  **/
  bool checkIfNeedToAvoid(const SupervisorState &state,
                          std::shared_ptr<Costmap2d> &ptr_costmap,
                          Pose2d &car_pose);

  /**
   *checkIfCarApproachRobot
   *@brief
   *判断汽车是否正在靠近机器人, 待验证功能
   *
   *@param[in] current_pose-机器人的位置
   *@param[in] car_pose-汽车的位置
   *@param[out] car_velocity-汽车的速度(x,y)
   *@return true-是, false-否
  **/
  bool checkIfCarApproachRobot(const Pose2d &current_pose,
                               const Pose2d &car_pose,
                               const ProcessorPoint &car_velocity);
  /**
   *scoreTargetPoints
   *@brief
   *利用定义的代价函数,对生成的每个目标点评分, 未实现
   *
   *@param[in] in-
   *@param[in] out-
   *@return true-, false-
  **/
  void scoreTargetPoints();

  /**
   *getBestTargetPoints
   *@brief
   *剃除一些不合理的点,选取综合评分最高的点作为目标点.
   *
   *@param[in] state-当前的输入
   *@param[in] v_sample_points-采样得到的候选点,已经确保是执行的
   *@param[out] target_point-最优的避让点
   *@return true-获取成功, false-获取失败,候选点为空或没有符合条件的
  **/
  bool getBestTargetPoints(const Costmap2d &costmap,
                           const std::vector<CostmapPoint> &v_sample_points,
                           const Pose2d &current_pose, const Pose2d &car_pose,
                           CostmapPoint &target_point);

  std::shared_ptr<PerceptionMap> ptr_perception_map_;

  std::shared_ptr<PointSampler> ptr_point_sampler_;
  std::shared_ptr<PointValidityChecker> ptr_point_validity_checker_;
  std::shared_ptr<Visualizer> ptr_visualizer_;
};
}  // namespace CVTE_BABOT
#endif
