/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file point_validity_checker.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#include "car_avoiding/point_validity_checker.hpp"
#include "costmap_mediator.hpp"

#include "local_planner.hpp"
#include "local_planner_factory.hpp"

namespace CVTE_BABOT {
PointValidityChecker::PointValidityChecker() {
  auto ptr_costmap = CostmapMediator::getPtrInstance()->getCostmap();
  auto size_x = ptr_costmap->getSizeInCellsX();
  auto size_y = ptr_costmap->getSizeInCellsY();

  LCPlannerFactory local_planner_factory;
  ptr_local_planner_ =
      local_planner_factory.createLocalPlanner("dijkstra", size_x, size_y);
}

PointValidityChecker::~PointValidityChecker() {}

void PointValidityChecker::checkPointsValidity(
    const Costmap2d &costmap, const std::vector<CostmapPoint> &v_sample_points,
    const Pose2d &car_pose, std::vector<CostmapPoint> &v_valid_points) {
  if (v_sample_points.empty()) {
    return;
  }

  CostmapPoint car_point;
  if (!costmap.worldToMap({car_pose.getX(), car_pose.getY()}, car_point)) {
    LOG(ERROR) << "The car is out of map.";
    return;
  }

  // 1.以汽车半径膨胀地图, 剩余的自由区域为使汽车能通行的区域
  Costmap2d costmap_expand_obs = costmap;
  expandObstacleArea(costmap_expand_obs);

  for (const auto &point : v_sample_points) {
    // 计算汽车是否能在规划出一条到达机器人当前位置的路径, 如果能,
    // 认为汽车能通过
   SubPath v_path;
    if (checkPassablity(costmap_expand_obs, point, car_point, v_path)) {
      v_valid_points.push_back(point);
      // 可视化得到的轨迹, 与功能无关
      // Mat costmap_mat_path;
      // std::vector<CostmapPoint> v_cp_path;
      // if (transformPathToMap(map_grid, v_path, v_cp_path)) {
      //   drawMatPoint(map_grid, v_cp_path, costmap_mat_path);

      //   // 转换成mat的坐标,x和y相反
      //   unsigned int size_x = costmap->getSizeInCellsX();
      //   unsigned int size_y = costmap->getSizeInCellsY();
      //   CostmapPoint mat_point;
      //   mat_point.ui_x = size_y - cp_point_end_right.ui_y;
      //   mat_point.ui_y = size_x - cp_point_end_right.ui_x;
      //   v_draw_points_.push_back(mat_point);

      //   // v_mats_.push_back(costmap_mat_path);
      //   // LOG(ERROR) << "make path success.";
      // } else {
      //   LOG(ERROR) << "transform path to map error.";
      // }
    }
  }

  // 可视化, 与功能无关
  // getMapMat(costmap, costmap_mat_);

  // Mat costmap_mat_point;
  // drawMatPoint(costmap, v_draw_points_, costmap_mat_point);
  // v_mats_.push_back(costmap_mat_point);
  // expandRobotArea(v_path, costmap);
}

// bool PointValidityChecker::transformPathToMap(
//     const std::shared_ptr<Costmap2d> &costmap,
//     const std::vector<Pose2d> &v_path, std::vector<CostmapPoint> &v_cp_path)
//     {
//   CostmapPoint cp_point;
//   unsigned int size_x = costmap->getSizeInCellsX();
//   unsigned int size_y = costmap->getSizeInCellsY();
//   for (size_t i = 0; i < v_path.size(); i++) {
//     if (costmap->worldToMap({v_path[i].x, v_path[i].y}, cp_point)) {
//       v_cp_path.push_back({size_y - cp_point.ui_y, size_x - cp_point.ui_x});
//     } else {
//       return false;
//     }
//   }
//   return true;
// }

bool PointValidityChecker::checkPassablity(const Costmap2d &costmap,
                                           const CostmapPoint &sample_point,
                                           const CostmapPoint &car_point,
                                          SubPath &v_path) {
  // 拓展采样点的costmap
  // 2.膨胀机器人所在的位置, 得到汽车的可行空间地图.
  auto costmap_expand_point = costmap;
  inflate(sample_point.ui_x, sample_point.ui_y, CAR_RADIUS,
          costmap_expand_point);

  // 机器人的位置附近寻找未占用点,因为膨胀后可能被占用了
  CostmapPoint start_point{costmap_expand_point.getSizeInCellsX() / 2,
                           costmap_expand_point.getSizeInCellsY() / 2};

  CostmapPoint robot_point;
  if (!computeNeareatFreePoint(costmap_expand_point, start_point,
                               FREE_POINT_SEARCH_RADIUS, robot_point)) {
    LOG(ERROR) << "compute robot point failed.";
    return false;
  }

  Pose2d robot_pose(robot_point.ui_x, robot_point.ui_y, 0.00);
  Pose2d car_pose(car_point.ui_x, car_point.ui_y, 0.00);
  auto ptr_char_map = costmap_expand_point.getCharMap();
  if (ptr_local_planner_->makePlan(ptr_char_map, robot_pose, car_pose,
                                   v_path)) {
    return true;
  } else {
    // LOG(ERROR) << "find path error, the car can't pass.";
    return false;
  }
}

void PointValidityChecker::expandObstacleArea(Costmap2d &costmap) {
  unsigned int size_x = costmap.getSizeInCellsX(),
               size_y = costmap.getSizeInCellsY();

  // 搜索地图上需要膨胀的点
  for (size_t x = 0; x < size_x; x++) {
    for (size_t y = 0; y < size_y; y++) {
      auto cost = costmap.getCost(x, y);

      if (cost == LETHAL_OBSTACLE) {
        inflate(x, y, CAR_RADIUS, costmap);
      }
    }
  }
}

}  // namespace CVTE_BABOT
