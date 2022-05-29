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
*@data 2019-04-13
************************************************************************/

#include "dwa/map_grid/map_grid.hpp"
#include "costmap_2d.hpp"

#include <glog/logging.h>
#include <assert.h>
#include <list>

namespace CVTE_BABOT {
MapGrid::MapGrid(const unsigned int &size_x, const unsigned int &size_y)
    : size_x_(size_x), size_y_(size_y) {
  commonInit();
}

void MapGrid::commonInit() {
  // don't allow construction of zero size grid
  assert(size_y_ != 0 && size_x_ != 0);
  map_.resize(size_y_ * size_x_);
  reset_map_.resize(size_y_ * size_x_);
  double unreachCellCost = unreachableCellCosts();
  unsigned int id = 0;
  for (unsigned int i = 0; i < size_y_; ++i) {
    for (unsigned int j = 0; j < size_x_; ++j) {
      id = size_x_ * i + j;
      map_[id].cx_ = j;
      map_[id].cy_ = i;
      map_[id].target_dist_ = unreachCellCost;
      map_[id].target_mark_ = false;
      map_[id].within_robot_ = false;
      reset_map_[id].cx_ = j;
      reset_map_[id].cy_ = i;
      reset_map_[id].target_dist_ = unreachCellCost;
      reset_map_[id].target_mark_ = false;
      reset_map_[id].within_robot_ = false;
    }
  }
}

void MapGrid::sizeCheck(const unsigned int &size_x,
                        const unsigned int &size_y) {
  if (map_.size() != size_x * size_y) {
    map_.resize(size_x * size_y);
    reset_map_.resize(size_x * size_y);
  }
  if (size_x_ != size_x || size_y_ != size_y) {
    size_x_ = size_x;
    size_y_ = size_y;
    int index = 0;
    for (unsigned int i = 0; i < size_y_; ++i) {
      for (unsigned int j = 0; j < size_x_; ++j) {
        index = size_x_ * i + j;
        map_[index].cx_ = j;
        map_[index].cy_ = i;
        reset_map_[index].cx_ = j;
        reset_map_[index].cy_ = i;
      }
    }
  }
}

bool MapGrid::updatePathCell(
    MapCell *current_cell, MapCell *check_cell,
    const std::shared_ptr<Costmap2d> costmap) {
  unsigned char cost = costmap->getCost(check_cell->cx_, check_cell->cy_);

  if (!getCell(check_cell->cx_, check_cell->cy_).within_robot_ &&
      (cost == LETHAL_OBSTACLE ||
       cost == INSCRIBED_INFLATED_OBSTACLE ||
       cost == NO_INFORMATION)) {
    check_cell->target_dist_ = obstacleCosts();
    return false;
  }
  double new_target_dist = current_cell->target_dist_ + 1;
  if (new_target_dist < check_cell->target_dist_) {
    check_cell->target_dist_ = new_target_dist;
  }
  return true;
}

// reset the path_dist and goal_dist fields for all cells
void MapGrid::resetPathDist() {
  // 方法一：直接将一张标志位为空的地图给目前地图
  // map_.assign(reset_map_.begin(), reset_map_.end());

  // 方法二：将整张地图从头到位复位
  // for (unsigned int i = 0; i < map_.size(); ++i) {
  //   map_[i].target_dist_ = unreachableCellCosts();
  //   map_[i].target_mark_ = false;
  //   map_[i].within_robot_ = false;
  // }

  // 方法三：通过拷贝的方法将地图标志位清除
  // std::copy(reset_map_.begin(), reset_map_.end(), map_.begin());

  // 方法四：抠出中间一部分进行清除，因为周围那些没有被修改过
  double unreachCellCost = unreachableCellCosts();
  // 复位遍厉时只搜索行列的20% 到 80% 的中间部分，无需将整张地图置位
  unsigned int start_row = size_x_ * 0.2; 
  unsigned int start_col = size_y_ * 0.2;
  unsigned int end_row = size_x_ * 0.8;
  unsigned int end_col = size_y_ * 0.8;
  unsigned int index = 0;
  for (unsigned int i = start_row; i < end_row; ++i) {
    for (unsigned int j = start_col; j < end_col; ++j) {
      index = i * size_y_ + j;
      map_[index].resetCell(unreachCellCost);
      // map_[index].target_dist_ = unreachCellCost;
      // map_[index].target_mark_ = false;
      // map_[index].within_robot_ = false;
    }
  }
}

void MapGrid::adjustPlanResolution(const std::vector<Pose2d> &global_plan_in,
                                   std::vector<Pose2d> &global_plan_out,
                                   const double &resolution) {
  if (global_plan_in.size() == 0) {
    return;
  }

  global_plan_out.clear();
  double last_x = global_plan_in[0].getX();
  double last_y = global_plan_in[0].getY();
  global_plan_out.push_back(global_plan_in[0]);

  // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
  double min_sq_resolution = resolution * resolution * 4;

  double loop_x = 0.0;
  double loop_y = 0.0;
  double sqdist = 0.0;
  int steps = 0;
  double deltax = 0.0;
  double deltay = 0.0;
  Pose2d pose;
  for (unsigned int i = 1; i < global_plan_in.size(); ++i) {
    loop_x = global_plan_in[i].getX();
    loop_y = global_plan_in[i].getY();
    sqdist = (loop_x - last_x) * (loop_x - last_x) +
             (loop_y - last_y) * (loop_y - last_y);
    if (sqdist > min_sq_resolution) {
      steps = ((sqrt(sqdist) - sqrt(min_sq_resolution)) / resolution) - 1;
      // add a points in-between
      deltax = (loop_x - last_x) / steps;
      deltay = (loop_y - last_y) / steps;
      // TODO: Interpolate orientation
      for (int j = 1; j < steps; ++j) {
        pose.setPose(last_x + j * deltax, last_y + j * deltay,
                     global_plan_in[i].getYaw());
        global_plan_out.push_back(pose);
      }
    }
    global_plan_out.push_back(global_plan_in[i]);
    last_x = loop_x;
    last_y = loop_y;
  }
}

void MapGrid::computeTargetDistance(
    std::queue<MapCell *> &dist_queue,
    const std::shared_ptr<Costmap2d> costmap) {
  MapCell *current_cell;
  MapCell *check_cell;
  unsigned int divied_scale = 6; // 将costmap划分的比例
  unsigned int one_col_size = (size_x_ - 1) / divied_scale; // 划分后一列的大小
  unsigned int one_row_size = (size_y_ - 1) / divied_scale; // 划分后一横的大小
  unsigned int middle_x_pose = (size_x_ - 1) / 2;  // 地图中间列的位置
  unsigned int middle_y_pose = (size_y_ - 1) / 2;  // 地图中间行的位置
  unsigned int start_col = middle_x_pose - one_col_size;  // 遍历起始列
  unsigned int start_row = middle_y_pose - one_row_size;  // 遍历起始行
  unsigned int last_col = middle_x_pose + one_col_size;   // 遍历结束列
  unsigned int last_row = middle_y_pose + one_row_size;   // 遍历结束行
  // unsigned int start_col = 0;  // 遍历起始列
  // unsigned int start_row = 0;  // 遍历起始行
  // unsigned int last_col = size_x_ - 1;   // 遍历结束列
  // unsigned int last_row = size_y_ - 1;   // 遍历结束行
  while (!dist_queue.empty()) {
    current_cell = dist_queue.front();
    dist_queue.pop();
    if (current_cell->cx_ > start_col) {
      check_cell = current_cell - 1;

      if (!check_cell->target_mark_) {
        // mark the cell as visisted
        check_cell->target_mark_ = true;
        if (updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }
    if (current_cell->cx_ < last_col) {
      check_cell = current_cell + 1;
      if (!check_cell->target_mark_) {
        check_cell->target_mark_ = true;
        if (updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }
    if (current_cell->cy_ > start_row) {
      check_cell = current_cell - size_x_;
      if (!check_cell->target_mark_) {
        check_cell->target_mark_ = true;
        if (updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }

    if (current_cell->cy_ < last_row) {
      check_cell = current_cell + size_x_;
      if (!check_cell->target_mark_) {
        check_cell->target_mark_ = true;
        if (updatePathCell(current_cell, check_cell, costmap)) {
          dist_queue.push(check_cell);
        }
      }
    }
  }
}  // computeTargetDistance

void MapGrid::setTargetCells(
    const std::shared_ptr<Costmap2d> costmap,
    const std::vector<Pose2d> &global_plan) {
  sizeCheck(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  bool started_path = false;
  std::queue<MapCell *> path_dist_queue;

  adjusted_global_plan_.clear();
  adjustPlanResolution(global_plan, adjusted_global_plan_,
                       costmap->getResolution());

  if (adjusted_global_plan_.size() != global_plan.size()) {
    LOG(WARNING) << "Adjusted global plan resolution, added "
                 << adjusted_global_plan_.size() - global_plan.size() << " points";
    // LOG(INFO) << "After: " << adjusted_global_plan_.size() << "  Before: " << global_plan.size();
  }
  // unsigned int calc_path_size = std::min(15, static_cast<int>(adjusted_global_plan_.size()));
  unsigned int i = 0;
  // put global path points into local map until we reach the border of the
  // local map
  // LOG(ERROR)<<"before MapCell"<<std::endl;
 
  WorldmapPoint wm_point;
  CostmapPoint cp_point;
  for (i = 0; i < adjusted_global_plan_.size(); ++i) {
    wm_point = {adjusted_global_plan_[i].getX(), adjusted_global_plan_[i].getY()};
    if (costmap->worldToMap(wm_point, cp_point) &&
        costmap->getCost(cp_point.ui_x, cp_point.ui_y) != NO_INFORMATION) {
      MapCell &current = getCell(cp_point.ui_x, cp_point.ui_y);
      if (!current.target_mark_) {
        current.target_dist_ = 0.0;
        current.target_mark_ = true;
      }
      path_dist_queue.push(&current);
      started_path = true;
    } else if (started_path) {
      LOG(INFO) << "started_path is true at : " << i;
      break;
    }
  }
  if (!started_path) {
    LOG(ERROR)
        << "None of the" << i << " first of" << adjusted_global_plan_.size()
        << "(" << global_plan.size()
        << ") points of the global plan were in the local costmap and free"
        << std::endl;
    return;
  }
  computeTargetDistance(path_dist_queue, costmap);
}

// mark the point of the costmap as local goal where global_plan first leaves
// the area (or its last point)
void MapGrid::setLocalGoal(
    const std::shared_ptr<Costmap2d> costmap,
    const std::vector<Pose2d> &global_plan) {
  sizeCheck(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  int local_goal_x = -1;
  int local_goal_y = -1;
  bool started_path = false;
  goal_adjusted_global_plan_.clear();
  adjustPlanResolution(global_plan, goal_adjusted_global_plan_,
                       costmap->getResolution());

  WorldmapPoint wm_point;
  CostmapPoint cp_point;
  // skip global path points until we reach the border of the local map
  for (unsigned int i = 0; i < goal_adjusted_global_plan_.size(); ++i) {
    wm_point = {goal_adjusted_global_plan_[i].getX(), goal_adjusted_global_plan_[i].getY()};
    if (costmap->worldToMap(wm_point, cp_point) &&
        costmap->getCost(cp_point.ui_x, cp_point.ui_y) != NO_INFORMATION) {
      local_goal_x = cp_point.ui_x;
      local_goal_y = cp_point.ui_y;
      started_path = true;
    } else {
      if (started_path) {
        break;
      }  // else we might have a non pruned path, so we just continue
    }
  }
  if (!started_path) {
    LOG(ERROR) << "None of the points of the global plan were in the local "
                  "costmap, global plan points too far from robot"
               << std::endl;
    return;
  }
  // LOG(ERROR)<<"before MapCell"<<std::endl;
  std::queue<MapCell *> path_dist_queue;
  if (local_goal_x >= 0 && local_goal_y >= 0) {
    MapCell &current = getCell(local_goal_x, local_goal_y);
    WorldmapPoint wm_point_temp;
    CostmapPoint cp_point_temp = {static_cast<unsigned int>(local_goal_x), 
      static_cast<unsigned int>(local_goal_y)};
    costmap->mapToWorld(cp_point_temp, wm_point_temp);
    current.target_dist_ = 0.0;
    current.target_mark_ = true;
    path_dist_queue.push(&current);
  }
  computeTargetDistance(path_dist_queue, costmap);
}

}  // namespace CVTE_BABOT