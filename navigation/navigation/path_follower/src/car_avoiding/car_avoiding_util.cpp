/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file car_avoiding_util.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#include "car_avoiding/car_avoiding_util.hpp"

namespace CVTE_BABOT {
bool computeNeareatFreePoint(const Costmap2d &costmap,
                             const CostmapPoint &init_point,
                             const unsigned int &ui_radius,
                             CostmapPoint &free_point) {
  WorldmapPoint wp_free;

  // 初始点未被占用,直接返回
  if (costmap.getCost(init_point) == FREE_SPACE) {
    free_point = init_point;
    return true;
  }

  unsigned int size_x = costmap.getSizeInCellsX(),
               size_y = costmap.getSizeInCellsY();

  // 如果radius过大,会出现得到的点穿墙的问题
  int ix_min = std::max(0, static_cast<int>(init_point.ui_x - ui_radius));
  int iy_min = std::max(0, static_cast<int>(init_point.ui_y - ui_radius));
  int ix_max = std::min(static_cast<int>(size_x),
                        static_cast<int>(init_point.ui_x + ui_radius));
  int iy_max = std::min(static_cast<int>(size_y),
                        static_cast<int>(init_point.ui_y + ui_radius));

  // 初始化为初始点
  CostmapPoint cp_nearest_point = init_point;
  double min_distance = 10e7;
  double distance = 10e7;
  int robot_x = size_x / 2, robot_y = size_y / 2;
  for (int x = ix_min; x < ix_max; ++x) {
    for (int y = iy_min; y < iy_max; ++y) {
      if (costmap.getCost(x, y) == FREE_SPACE) {
        // 取离原点最近且离目标点近的点
        double distance_to_point =
            getDistance(x, y, init_point.ui_x, init_point.ui_y);
        double distance_to_origin = getDistance(x, y, robot_x, robot_y);
        distance = distance_to_point + distance_to_origin;

        if (distance < min_distance) {
          min_distance = distance;
          // 显示转换数据类型
          cp_nearest_point.ui_x = x;
          cp_nearest_point.ui_y = y;
        }
      }
    }
  }

  // 检查是否找到目标点
  if (costmap.getCost(cp_nearest_point) == FREE_SPACE) {
    free_point = cp_nearest_point;
    return true;
  } else {
    return false;
  }
}

void inflate(const unsigned int &ui_x, const unsigned int &ui_y,
             const unsigned int &inflate_radius, Costmap2d &costmap) {
  auto size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

  int ix_min = std::max(0, static_cast<int>(ui_x - inflate_radius));
  int iy_min = std::max(0, static_cast<int>(ui_y - inflate_radius));
  int ix_max = std::min(static_cast<int>(size_x),
                        static_cast<int>(ui_x + inflate_radius));
  int iy_max = std::min(static_cast<int>(size_y),
                        static_cast<int>(ui_y + inflate_radius));

  // 膨胀某点的周围栅格
  for (int ix = ix_min; ix < ix_max; ++ix) {
    for (int iy = iy_min; iy < iy_max; ++iy) {
      auto old_cost = costmap.getCost(ui_x, ui_y);
      if (old_cost == LETHAL_OBSTACLE) {
        continue;
      }

      costmap.setCost(ui_x, ui_y, INSCRIBED_INFLATED_OBSTACLE);
    }
  }
}

}  // namespace CVTE_BABOT
