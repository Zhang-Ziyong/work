/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file point_validity_checker.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-17
 ************************************************************************/
#ifndef __POINT_VALIDITY_CHECKER_HPP
#define __POINT_VALIDITY_CHECKER_HPP
#include "car_avoiding/car_avoiding_util.hpp"
#include "path.hpp"
namespace CVTE_BABOT {
const uint16_t CAR_RADIUS = 13;

class LocalPlanner;
class PointValidityChecker {
 public:
  PointValidityChecker();
  ~PointValidityChecker();

  PointValidityChecker(const PointValidityChecker &) = delete;
  PointValidityChecker &operator=(const PointValidityChecker &) = delete;

  void checkPointsValidity(const Costmap2d &costmap,
                           const std::vector<CostmapPoint> &v_sample_points,
                           const Pose2d &car_pose,
                           std::vector<CostmapPoint> &v_valid_points);

 private:
  bool checkPassablity(const Costmap2d &costmap,
                       const CostmapPoint &sample_point,
                       const CostmapPoint &car_point,
                       SubPath &v_path);

  /**
   *expandObstacleArea
   *@brief
   *以汽车半径膨胀地图, 剩余的自由区域为使汽车能通行的区域
   *
   *@param[in] master_grid-要膨胀的costmap
   *@param[in] 膨胀半径-目前没使用
  **/
  void expandObstacleArea(Costmap2d &master_grid);

  std::shared_ptr<LocalPlanner> ptr_local_planner_;
};
}  // namespace CVTE_BABOT
#endif
