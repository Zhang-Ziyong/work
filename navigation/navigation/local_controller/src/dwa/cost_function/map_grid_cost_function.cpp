/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file map_grid.hpp
*
*@author TKruse

*@modified current_algo.dev.1.0
*@version 1.2.1
*@data 2019-04-15
************************************************************************/
#include "dwa/cost_function/map_grid_cost_function.hpp"
#include "dwa/map_grid/map_grid.hpp"
#include "dwa/world_model/costmap_model.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {
MapGridCostFunction::MapGridCostFunction(double xshift, double yshift,
                                         bool is_local_goal_function,
                                         CostAggregationType aggregationType)
    : CostFunction(1.0),
      aggregationType_(aggregationType),
      xshift_(xshift),
      yshift_(yshift),
      is_local_goal_function_(is_local_goal_function),
      stop_on_failure_(true) {
  ptr_map_ = std::make_shared<MapGrid>(
      CostmapModel::getPtrInstance()->getSizeInCellsX(),
      CostmapModel::getPtrInstance()->getSizeInCellsY());
}

MapGridCostFunction::MapGridCostFunction(const MapGridCostFunction &obj)
    : CostFunction(1.0) {
  target_poses_ = obj.target_poses_;
  ptr_map_ = obj.ptr_map_;
  aggregationType_ = obj.aggregationType_;
  xshift_ = obj.xshift_;
  yshift_ = obj.yshift_;
  is_local_goal_function_ = obj.is_local_goal_function_;
  stop_on_failure_ = obj.stop_on_failure_;
}

MapGridCostFunction &MapGridCostFunction::operator=(
    const MapGridCostFunction &obj) {
  if (this == &obj) {
    return *this;
  }
  CostFunction::operator=(obj);
  target_poses_ = obj.target_poses_;
  ptr_map_ = obj.ptr_map_;
  aggregationType_ = obj.aggregationType_;
  xshift_ = obj.xshift_;
  yshift_ = obj.yshift_;
  is_local_goal_function_ = obj.is_local_goal_function_;
  stop_on_failure_ = obj.stop_on_failure_;
  return *this;
}

void MapGridCostFunction::setTargetPoses(
    const std::vector<Pose2d> &target_poses) {
  target_poses_.assign(target_poses.begin(), target_poses.end());
}

bool MapGridCostFunction::prepare() {
  ptr_map_->resetPathDist();
  if (is_local_goal_function_) {
    ptr_map_->setLocalGoal(CostmapModel::getPtrInstance()->getCostmap(),
                           target_poses_);
  } else {
    ptr_map_->setTargetCells(CostmapModel::getPtrInstance()->getCostmap(),
                             target_poses_);
  }
  return true;
}

double MapGridCostFunction::getCellCosts(const unsigned int &px,
                                         const unsigned int &py) {
  double grid_dist = (*ptr_map_)(px, py).target_dist_;
  return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(const Trajectory &traj) {
  double cost = 0.0;
  if (aggregationType_ == Product) {
    cost = 1.0;
  }
  double px, py, pth;
  double grid_dist;
  unsigned int ui_x, ui_y;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);

    // translate point forward if specified
    if (xshift_ != 0.0) {
      px = px + xshift_ * cos(pth);
      py = py + xshift_ * sin(pth);
    }
    // translate point sideways if specified
    if (yshift_ != 0.0) {
      px = px + yshift_ * cos(pth + M_PI_2);
      py = py + yshift_ * sin(pth + M_PI_2);
    }

    // we won't allow trajectories that go off the map... shouldn't happen
    // that often anyways
    if (!CostmapModel::getPtrInstance()->worldToMap(px, py, ui_x, ui_y)) {
      // we're off the map
      LOG(WARNING) << "Off Map" << px << "," << py;
      return -4.0;
    }
    grid_dist = getCellCosts(ui_x, ui_y);
    // if a point on this trajectory has no clear path to the goal... it may
    // be invalid
    if (stop_on_failure_) {
      if (grid_dist == ptr_map_->obstacleCosts()) {
        return -3.0;
      } else if (grid_dist == ptr_map_->unreachableCellCosts()) {
        return -2.0;
      }
    }

    switch (aggregationType_) {
      case Last:
        cost = grid_dist;
        break;
      case Sum:
        cost += grid_dist;
        break;
      case Product:
        if (cost > 0) {
          cost *= grid_dist;
        }
        break;
    }
  }
  return cost;
}

}  // namespace CVTE_BABOT