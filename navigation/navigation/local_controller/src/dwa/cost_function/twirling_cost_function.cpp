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
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author caoyong(caoyong@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-15
 ************************************************************************/

#include "dwa/cost_function/twirling_cost_function.hpp"
#include <math.h>

namespace CVTE_BABOT {

TwirlingCostFunction::TwirlingCostFunction() : CostFunction(1.0) {}

TwirlingCostFunction::TwirlingCostFunction(const TwirlingCostFunction &obj)
    : CostFunction(1.0) {
  if (this == &obj) {
    // no warning
  }
}

TwirlingCostFunction &TwirlingCostFunction::operator=(
    const TwirlingCostFunction &obj) {
  if (this == &obj) {
    return *this;
  }
  CostFunction::operator=(obj);
  return *this;
}

double TwirlingCostFunction::scoreTrajectory(const Trajectory &traj) {
  return fabs(traj.thetav_);
  // add cost for making the robot spin
}
}  // namespace CVTE_BABOT