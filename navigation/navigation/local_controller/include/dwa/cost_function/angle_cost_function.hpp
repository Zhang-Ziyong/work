/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file angle_cost_function.hpp
 *
 *@brief 计算转向角，使转向角较小的路径代价较低
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-03-30
 ************************************************************************/
#ifndef __ANGLE_COST_FUNCTION_HPP
#define __ANGLE_COST_FUNCTION_HPP

#include "cost_function.hpp"
#include "trajectory.hpp"

#include <pose2d/pose2d.hpp>

namespace CVTE_BABOT {
/**
* AngleCostFunction
* @brief
*   DWA中计算角度与目标角度的评价函数
* */

class AngleCostFunction : public CostFunction {
 public:
  AngleCostFunction();
  ~AngleCostFunction() = default;

  AngleCostFunction &operator=(const AngleCostFunction &obj) = delete;
  AngleCostFunction(const AngleCostFunction &obj) = delete;

  /**
   *setTargetAngle
   *@brief
   *设置目标点的角度,归一化到[-pi,pi]
   *
   *@param[in] in-目标点的角度
  **/
  void setTargetAngle(const double &target_angle) {
    target_angle_ = AngleCalculate::normalizeAngle(target_angle);
  }

  /**
* scoreTrajectory
* @brief
*   计算一条路径的得分
*
* @param[in] traj-需要计算分数的路径
* */
  double scoreTrajectory(const Trajectory &traj) override;

 private:
  double target_angle_;
};

}  // namespace CVTE_BABOT

#endif
