/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file car_avoiding_util.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#ifndef __CAR_AVOIDING_UTIL_HPP
#define __CAR_AVOIDING_UTIL_HPP
#include <glog/logging.h>

#include "costmap_2d.hpp"
#include "pose2d/pose2d.hpp"

namespace CVTE_BABOT {
const uint16_t FREE_POINT_SEARCH_RADIUS = 13;

bool computeNeareatFreePoint(const Costmap2d &costmap,
                             const CostmapPoint &init_point,
                             const unsigned int &ui_radius,
                             CostmapPoint &free_point);

void inflate(const unsigned int &ui_x, const unsigned int &ui_y,
             const unsigned int &inflate_radius, Costmap2d &costmap);

inline double getDistance(const int &x1, const int &y1, const int &x2,
                          const int &y2) {
  int dx = x1 - x2;
  int dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}

}  // namespace CVTE_BABOT
#endif
