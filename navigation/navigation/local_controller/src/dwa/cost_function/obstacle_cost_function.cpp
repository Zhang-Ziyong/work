/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file map_grid.hpp
*
*@author Eitan Marder-Eppstein

*@modified caoyong (caoyong@cvte.com)
*@current_algo.dev.1.0
*@data 2019-04-15
************************************************************************/

#include "dwa/cost_function/obstacle_cost_function.hpp"
#include "costmap_2d.hpp"
#include "dwa/world_model/costmap_model.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {
ObstacleCostFunction::ObstacleCostFunction()
    : CostFunction(1.0), sum_scores_(false) {}

ObstacleCostFunction::ObstacleCostFunction(const ObstacleCostFunction &obj)
    : CostFunction(1.0) {
  footprint_spec_ = obj.footprint_spec_;
  max_trans_vel_ = obj.max_trans_vel_;
  sum_scores_ = obj.sum_scores_;
  // footprint scaling with velocity;
  max_scaling_factor_ = obj.max_scaling_factor_;
  scaling_speed_ = obj.scaling_speed_;
}

ObstacleCostFunction &ObstacleCostFunction::operator=(
    const ObstacleCostFunction &obj) {
  if (this == &obj) {
    return *this;
  }
  CostFunction::operator=(obj);
  footprint_spec_ = obj.footprint_spec_;
  max_trans_vel_ = obj.max_trans_vel_;
  sum_scores_ = obj.sum_scores_;
  // footprint scaling with velocity;
  max_scaling_factor_ = obj.max_scaling_factor_;
  scaling_speed_ = obj.scaling_speed_;
  return *this;
}

double ObstacleCostFunction::scoreTrajectory(const Trajectory &traj) {
  double cost = 0;
  double px = 0.0;
  double py = 0.0;
  double pth = 0.0;

  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    LOG(ERROR)
        << "Footprint spec is empty, maybe missing call to setFootprint?";
    return -9;
  }

  double f_cost = 0.0;
  for (unsigned int i = 0; i < traj.getPointsSize(); i += 4) {
    traj.getPoint(i, px, py, pth);
    f_cost = footprintCost(px, py, pth, footprint_spec_);

    if (f_cost < 0) {
      // LOG(ERROR) << "crash into obstacles, descard it.";
      return f_cost;
    }

    if (sum_scores_)
      cost += f_cost;
    else
      cost = f_cost;
  }
  return cost;
}

double ObstacleCostFunction::getScalingFactor(
    const Trajectory &traj, const double &scaling_speed,
    const double &max_trans_vel, const double &max_scaling_factor) {
  double vmag = hypot(traj.xv_, traj.yv_);

  // if we're over a certain speed threshold, we'll scale the robot's
  // footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    // scale up to the max scaling factor linearly... this could be changed
    // later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double ObstacleCostFunction::footprintCost(
    const double &x, const double &y, const double &th,
    const std::array<Pose2d, 4> &footprint_spec) {
  // check if the footprint is legal
  // TODO: Cache inscribed radius
  // if (scale == 0) {
  //   // no warning
  // }
  double footprint_cost =
      CostmapModel::getPtrInstance()->footprintCost(x, y, th, footprint_spec);
  if (footprint_cost < 0) {
    return -6.0;
  }
  unsigned int ui_x = 0;
  unsigned int ui_y = 0;

  // we won't allow trajectories that go off the map... shouldn't happen that
  // often anyways
  if (!CostmapModel::getPtrInstance()->worldToMap(x, y, ui_x, ui_y)) {
    return -7.0;
  }

  double occ_cost =
      std::max(std::max(0.0, footprint_cost),
               double(CostmapModel::getPtrInstance()->getCost(ui_x, ui_y)));

  return occ_cost;
}
}  // namespace CVTE_BABOT