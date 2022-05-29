/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file obstacle_map.cpp
 *
 *@brief 障碍信息地图
 *
 *@author caoyong(caoyong@cvte.com)
 *@modified caoyong (caoyong@cvte.com)
 *@data 2020-01-25
 ************************************************************************/
#include <glog/logging.h>
#include <algorithm>
#include <iostream>
#include <set>
#include "obstacle_map.hpp"
#include "costmap_mediator.hpp"

namespace CVTE_BABOT {

ObstacleMap::ObstacleMap() {}

ROTATETURN ObstacleMap::computeRotateTurn(const double &target_angle) {
  return ALLTURN;
}

bool ObstacleMap::isThereObsFront() {
  return CostmapMediator::getPtrInstance()->isThereObsFront();
}

bool ObstacleMap::isThereObsBack() {
  return CostmapMediator::getPtrInstance()->isThereObsBack();
}

bool ObstacleMap::isThereObsLeft() {
  return CostmapMediator::getPtrInstance()->isThereObsLeft();
}

bool ObstacleMap::isThereObsRight() {
  return CostmapMediator::getPtrInstance()->isThereObsRight();
}

bool ObstacleMap::isThereObsRightFront() {
  return CostmapMediator::getPtrInstance()->isThereObsRightFront();
}

bool ObstacleMap::isThereObsLeftFront() {
  return CostmapMediator::getPtrInstance()->isThereObsLeftFront();
}

bool ObstacleMap::isThereObsRightBack() {
  return CostmapMediator::getPtrInstance()->isThereObsRightBack();
}

bool ObstacleMap::isThereObsLeftBack() {
  return CostmapMediator::getPtrInstance()->isThereObsLeftBack();
}

}  // namespace CVTE_BABOT