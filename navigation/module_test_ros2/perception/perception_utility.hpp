/*
 * @Author: chennuo@cvte.com
 * @Date: 2021-07-16 12:51:56
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-07-16 14:52:38
 */

#pragma once

#include <iostream>
#include <memory>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "costmap_2d.hpp"

void generateCostmapMsg(
    const std::shared_ptr<const CVTE_BABOT::Costmap2d> &ptr_costmap,
    nav_msgs::msg::OccupancyGrid &grid_map);