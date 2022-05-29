/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file planner_utils.hpp
 *
 *@brief planner_utils.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-01-06
 ************************************************************************/
#ifndef __PLANNER_UTILS_HPP
#define __PLANNER_UTILS_HPP

namespace CVTE_BABOT {

struct Velocity {
  double d_x = 0.000;
  double d_y = 0.000;
  double d_yaw = 0.000;
};

} // namespace CVTE_BABOT

#endif