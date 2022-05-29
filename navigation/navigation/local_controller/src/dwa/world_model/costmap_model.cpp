/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file costmap_model.cpp
*
*@author Eitan Marder-Eppstein

*@modified caoyong (caoyong@cvte.com)
*@current_algo.dev.1.0
*@data 2019-04-15
************************************************************************/
#include "dwa/world_model/costmap_model.hpp"
#include <glog/logging.h>
#include <queue>
#include "costmap_2d.hpp"
#include "dwa/iterator/line_iterator.hpp"
#include "navigation_mediator.hpp"

namespace CVTE_BABOT {

std::shared_ptr<CostmapModel> CostmapModel::ptr_instance_ = nullptr;

CostmapModel::CostmapModel() {
  int cell_distance_to_obs = 0;
  NavigationMediator::getPtrInstance()->getParam(
      "path_follower.cell_distance_to_obs", cell_distance_to_obs, 4);
  ui_cell_distance_to_obs_ = cell_distance_to_obs;
}

std::shared_ptr<CostmapModel> CostmapModel::getPtrInstance() {
  if (!ptr_instance_.get()) {
    ptr_instance_.reset(new CostmapModel());
  }
  return ptr_instance_;
}

double CostmapModel::footprintCost(
    const double &x, const double &y, const double &theta,
    const std::array<Pose2d, 4> &footprint_spec) {
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  std::array<Pose2d, 4> oriented_footprint;
  Pose2d new_pt;
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    new_pt.setX(x + (footprint_spec[i].getX() * cos_th -
                     footprint_spec[i].getY() * sin_th));
    new_pt.setY(y + (footprint_spec[i].getX() * sin_th +
                     footprint_spec[i].getY() * cos_th));
    oriented_footprint[i] = new_pt;
  }

  Pose2d robot_position;
  robot_position.setX(x);
  robot_position.setY(y);

  return footprintCost(robot_position, oriented_footprint);
}

double CostmapModel::footprintCost(const Pose2d &position,
                                   const std::array<Pose2d, 4> &footprint) {
  // used to put things into grid coordinates
  unsigned int cell_x = 0;
  unsigned int cell_y = 0;
  // get the cell coord of the center point of the robot
  if (!worldToMap(position.getX(), position.getY(), cell_x, cell_y))
    return -1.0;

  // if number of points in the footprint is less than 3, we'll just assume a
  // circular robot
  if (footprint.size() < 3) {
    unsigned char cost = getCost(cell_x, cell_y);
    // if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE)
    if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE ||
        cost == NO_INFORMATION)
      return -1.0;
    return cost;
  }

  // now we really have to lay down the footprint in the costmap grid
  double line_cost = 0.0;
  double footprint_cost = 0.0;
  unsigned int x0 = 0;
  unsigned int x1 = 0;
  unsigned int y0 = 0;
  unsigned int y1 = 0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!worldToMap(footprint[i].getX(), footprint[i].getY(), x0, y0))
      return -1.0;

    // get the cell coord of the second point
    if (!worldToMap(footprint[i + 1].getX(), footprint[i + 1].getY(), x1, y1))
      return -1.0;

    line_cost = lineCost(x0, y0, x1, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    // if there is an obstacle that hits the line... we know that we can return
    // false right away
    if (line_cost < 0) return -1.0;
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!worldToMap(footprint.back().getX(), footprint.back().getY(), x0, y0))
    return -1.0;

  // get the cell coord of the first point
  if (!worldToMap(footprint.front().getX(), footprint.front().getY(), x1, y1))
    return -1.0;

  line_cost = lineCost(x0, y0, x1, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  if (line_cost < 0) return -1.0;

  // if all line costs are legal... then we can return that the footprint is
  // legal
  return footprint_cost;
}

// calculate the cost of a ray-traced line
double CostmapModel::lineCost(const int &x0, const int &y0, const int &x1,
                              const int &y1) {
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost =
        pointCost(line.getX(), line.getY());  // Score the current point

    if (point_cost < 0) return -1;

    if (line_cost < point_cost) line_cost = point_cost;
  }

  return line_cost;
}

// calculate the cost of a ray-traced line
double CostmapModel::lineCostStrict(const int &x0, const int &y0, const int &x1,
                                    const int &y1) {
  double line_cost = 0.0;
  double point_cost = -1.0;
  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    unsigned char cost = getCost(line.getX(), line.getY());

    // 可能会发生碰撞的代价阀值
    static const unsigned char IMPACT_COST = 235;

    if (cost >= IMPACT_COST) {
      point_cost = -1;
    } else {
      point_cost = cost;
    }

    if (point_cost < 0) {
      return -1;
    }

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }
  return line_cost;
}

bool CostmapModel::lineCost(const int &x0, const int &y0, const int &x1,
                            const int &y1, CostmapPoint &compact_point) {
  double point_cost = -1.0;
  std::queue<int> last_x, last_y;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    last_x.push(line.getX());
    last_y.push(line.getY());
    // 离costmap多少栅格
    if (last_x.size() > ui_cell_distance_to_obs_) {
      last_x.pop();
      last_y.pop();
    }

    point_cost =
        pointCost(line.getX(), line.getY());  // Score the current point

    // if (point_cost < 0 || point_cost > 50) {
    if (point_cost != FREE_SPACE) {
      compact_point.ui_x = last_x.front();
      compact_point.ui_y = last_y.front();
      return true;
    }
  }
  return false;
}

double CostmapModel::pointCost(const int &x, const int &y) {
  unsigned char cost = getCost(x, y);
  // if the cell is in an obstacle the path is invalid
  // if(cost == LETHAL_OBSTACLE){
  if (cost == LETHAL_OBSTACLE || cost == NO_INFORMATION) {
    return -1;
  }
  return cost;
}

bool CostmapModel::worldToMap(const double &w_x, const double &w_y,
                              unsigned int &m_x, unsigned int &m_y) {
  WorldmapPoint wm_point_temp = {w_x, w_y};
  CostmapPoint cp_point_temp;
  if (costmap_->worldToMap(wm_point_temp, cp_point_temp)) {
    m_x = cp_point_temp.ui_x;
    m_y = cp_point_temp.ui_y;
    return true;
  }
  m_x = 0.0;
  m_y = 0.0;
  return false;
}

void CostmapModel::mapToWorld(const unsigned int &m_x, const unsigned int &m_y,
                              double &w_x, double &w_y) {
  WorldmapPoint wm_point_temp;
  CostmapPoint cp_point_temp = {m_x, m_y};
  costmap_->mapToWorld(cp_point_temp, wm_point_temp);
  w_x = wm_point_temp.d_x;
  w_y = wm_point_temp.d_y;
}

}  // namespace CVTE_BABOT
