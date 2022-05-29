/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_utils.cpp
 *
 *@brief 一些公共函数
 *
 *@author chenmingjian (chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#include "costmap_utils.hpp"
#include <iostream>

namespace CVTE_BABOT {

double distanceToLine(const double &pX, const double &pY, const double &x0,
                      const double &y0, const double &x1, const double &y1) {
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;

  if (param < 0) {
    xx = x0;
    yy = y0;
  } else if (param > 1) {
    xx = x1;
    yy = y1;
  } else {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  return distance(pX, pY, xx, yy);
}

void calculateMinAndMaxDistances(const std::vector<WorldmapPoint> &v_footprint,
                                 double &d_min_dist, double &d_max_dist) {
  // 计算内接圆和外接圆
  d_min_dist = std::numeric_limits<double>::max();
  d_max_dist = 0.0;

  if (v_footprint.size() <= 2) {
    return;
  }

  for (unsigned int i = 0; i < v_footprint.size() - 1; ++i) {
    // check the distance from the robot center point to the first vertex
    // 计算到顶点的距离
    double d_vertex_dist =
        distance(0.0, 0.0, v_footprint[i].d_x, v_footprint[i].d_y);
    // 计算到边的距离
    double d_edge_dist =
        distanceToLine(0.0, 0.0, v_footprint[i].d_x, v_footprint[i].d_y,
                       v_footprint[i + 1].d_x, v_footprint[i + 1].d_y);
    d_min_dist = std::min(d_min_dist, std::min(d_vertex_dist, d_edge_dist));
    d_max_dist = std::max(d_max_dist, std::max(d_vertex_dist, d_edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  // 计算最后一个顶点与边
  double d_vertex_dist =
      distance(0.0, 0.0, v_footprint.back().d_x, v_footprint.back().d_y);
  double d_edge_dist =
      distanceToLine(0.0, 0.0, v_footprint.back().d_x, v_footprint.back().d_y,
                     v_footprint.front().d_x, v_footprint.front().d_y);

  // 获取内切圆与外切圆半径
  d_min_dist = std::min(d_min_dist, std::min(d_vertex_dist, d_edge_dist));
  d_max_dist = std::max(d_max_dist, std::max(d_vertex_dist, d_edge_dist));
}

void transformFootprint(
    const WorldmapPose &wp_robot_pose,
    const std::vector<WorldmapPoint> &v_wp_footprint,
    std::vector<WorldmapPoint> &v_wp_transformed_footprint) {
  v_wp_transformed_footprint.clear();
  double d_cos_th = cos(wp_robot_pose.d_yaw);
  double d_sin_th = sin(wp_robot_pose.d_yaw);
  WorldmapPoint new_point;
  for (unsigned int i = 0; i < v_wp_footprint.size(); ++i) {
    new_point.d_x = wp_robot_pose.d_x + (v_wp_footprint[i].d_x * d_cos_th -
                                         v_wp_footprint[i].d_y * d_sin_th);
    new_point.d_y = wp_robot_pose.d_y + (v_wp_footprint[i].d_x * d_sin_th +
                                         v_wp_footprint[i].d_y * d_cos_th);
    v_wp_transformed_footprint.push_back(new_point);
  }
}

}  // namespace CVTE_BABOT
