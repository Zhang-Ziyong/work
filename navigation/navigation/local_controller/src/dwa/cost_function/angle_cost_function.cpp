/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file angle_cost_function.cpp
 *
 *@brief 计算转向角，使转向角较小的路径代价较低
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-03-30
 ************************************************************************/
#include "dwa/cost_function/angle_cost_function.hpp"
#include <glog/logging.h>
#include <math.h>

namespace CVTE_BABOT {

AngleCostFunction::AngleCostFunction() : CostFunction(1.0) {}

double AngleCostFunction::scoreTrajectory(const Trajectory &traj) {
  if (traj.getPointsSize() == 0) {
    return 0.00;
  }

  int end_point_it = traj.getPointsSize() - 1;
  static double px, py, pth;
  traj.getPoint(end_point_it, px, py, pth);

  pth = AngleCalculate::normalizeAngle(pth);

  static double angle_cost = 0.00;
  angle_cost = fabs(target_angle_ - pth);
  // 超过180度的话，转换到180度以内．
  if (angle_cost > M_PI) {
    angle_cost = 2 * M_PI - angle_cost;
  }

  // LOG(ERROR) << "target angle: " << target_angle_ << ", current angle: " <<
  // pth
  //            << ", angle_cost: " << angle_cost;
  return angle_cost;
}

}  // namespace CVTE_BABOT
