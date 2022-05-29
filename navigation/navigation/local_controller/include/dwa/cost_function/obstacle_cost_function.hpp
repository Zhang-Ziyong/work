/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file obstacle_cost_function.hpp
 *
 *@brief DWA中计算与障碍物距离的评价函数
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/

#ifndef __OBSTACLE_COST_FUNCTION_HPP
#define __OBSTACLE_COST_FUNCTION_HPP

#include "cost_function.hpp"
#include "trajectory.hpp"

#include <pose2d/pose2d.hpp>
#include <array>
#include <memory>
#include <vector>

namespace CVTE_BABOT {
class Costmap2d;
class Pose2d;
class CostmapModel;

/**
* ObstacleCostFunction
* @brief
*   DWA中计算与障碍物距离的评价函数
* */

class ObstacleCostFunction : public CostFunction {
 public:
  ObstacleCostFunction();
  ObstacleCostFunction(const ObstacleCostFunction &obj);
  ObstacleCostFunction &operator=(const ObstacleCostFunction &obj);
  ~ObstacleCostFunction() = default;

  /**
* scoreTrajectory
* @brief
*   计算一条路径的得分
*
* @param[in] traj-需要计算分数的路径
* */
  double scoreTrajectory(const Trajectory &traj);

  /**
* setSumScores
* @brief
*   设置总得分
*
* @param[in] score_sum-总得分
* */
  inline void setSumScores(const bool &score_sum) { sum_scores_ = score_sum; }

  /**
* setParams
* @brief
*   设置几个参数
*
* @param[in] max_trans_vel-最大移动速度
* @param[in] max_scaling_factor-最大切割因子
* @param[in] scaling_speed-采样速度
* */
  inline void setParams(const double &max_trans_vel,
                        const double &max_scaling_factor,
                        const double &scaling_speed) {
    max_trans_vel_ = max_trans_vel;
    max_scaling_factor_ = max_scaling_factor;
    scaling_speed_ = scaling_speed;
  }

  /**
* setFootprint
* @brief
*   设置机器人footprint的坐标点
*
* @param[in] footprint_spec-表示机器人几何形状的坐标点集合
* */
  inline void setFootprint(std::array<Pose2d, 4> &footprint_spec) {
    footprint_spec_ = footprint_spec;
  }

  /**
* getScalingFactor
* @brief
*   获取比例因子
*
* @param[in] traj-路径点
* @param[in] scaling_speed-比例速度
* @param[in] max_trans_vel-最大移动速度
* @param[in] max_scaling_factor-最大比例因子
*
* @return 返回比例因子
* */
  double getScalingFactor(const Trajectory &traj, const double &scaling_speed,
                          const double &max_trans_vel,
                          const double &max_scaling_factor);

  /**
* footprintCost
* @brief
*   计算路径点相对于机器人footprint的cost值
*
* @param[in] x-路径点的x坐标
* @param[in] y-路径点的y坐标
* @param[in] th-路径点的角度
* @param[in] footprint_spec-机器人的footprint集合
* @param[in] costmap-代价地图
* @param[in] world_model-代价地图模型
* */
  double footprintCost(const double &x, const double &y, const double &th,
                       const std::array<Pose2d, 4> &footprint_spec);

 private:
  std::array<Pose2d, 4> footprint_spec_;       ///< 机器人footprint集合

  double max_trans_vel_ = 0.0;       ///< 最大线速度
  double max_scaling_factor_ = 0.0;  ///< 最大比例因子
  double scaling_speed_ = 0.0;       ///< 速度比例
  bool sum_scores_ = false;          ///< 总评分
};

}  // namespace CVTE_BABOT

#endif  // __OBSTACLE_COST_FUNCTION_HPP