/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file obstacle_map.hpp
 *
 *@brief 障碍信息地图
 *
 *@author caoyong(caoyong@cvte.com)
 *@modified caoyong (caoyong@cvte.com)
 *@data 2020-01-25
 ************************************************************************/
#ifndef __OBSTACLE_MAP_HPP
#define __OBSTACLE_MAP_HPP
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "costmap_utils.hpp"
#include "pose2d/pose2d.hpp"
namespace CVTE_BABOT {
enum ROTATETURN { NOTURN = -100, RIGHTTURN = -1, ALLTURN = 0, LEFTTURN = 1 };

class Costmap2d;
class CostmapRangeData;
class ObstacleMap {
 public:
  ObstacleMap();
  ~ObstacleMap() = default;

  // 禁止拷贝
  ObstacleMap(const ObstacleMap &) = delete;
  ObstacleMap &operator=(const ObstacleMap &) = delete;

  /**
   *computeRotateTurn
   *@brief
   *核心函数, 获取可旋转的方向
   *
   *@param[in] target_angle-目标角度
   **/
  ROTATETURN computeRotateTurn(const double &target_angle);

  bool isThereObsFront();

  bool isThereObsBack();

  bool isThereObsLeft();

  bool isThereObsRight();

  bool isThereObsRightFront();

  bool isThereObsLeftFront();

  bool isThereObsRightBack();

  bool isThereObsLeftBack();
};
}  // namespace CVTE_BABOT
#endif
