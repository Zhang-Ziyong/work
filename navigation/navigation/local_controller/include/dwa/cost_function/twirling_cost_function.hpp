/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file twirling_cost_function.hpp
 *
 *@brief DWA中计算转动的评价函数
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/

#ifndef __TWIRLING_COST_FUNCTION_HPP
#define __TWIRLING_COST_FUNCTION_HPP

#include "cost_function.hpp"
#include "trajectory.hpp"

namespace CVTE_BABOT {
/**
* TwirlingCostFunction
* @brief
*   DWA中计算转动的评价函数
* */

class TwirlingCostFunction : public CostFunction {
 public:
  TwirlingCostFunction();
  TwirlingCostFunction &operator=(const TwirlingCostFunction &obj);
  TwirlingCostFunction(const TwirlingCostFunction &obj);
  ~TwirlingCostFunction() {}

  /**
* scoreTrajectory
* @brief
*   计算一条路径的得分
*
* @param[in] traj-需要计算分数的路径
* */
  double scoreTrajectory(const Trajectory &traj) override;
};

}  // namespace CVTE_BABOT

#endif  // __TWIRLING_COST_FUNCTION_HPP