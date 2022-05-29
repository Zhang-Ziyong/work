/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file layer_creator.cpp
 *
 *@brief layer creator基类.工厂模式
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#include "layer_creator.hpp"
#include "collision_layer.hpp"
#include "costmap_mediator.hpp"
#include "inflation_layer.hpp"
#include "negative_obstacles_layer.hpp"
#include "obstacle_layer.hpp"
#include "probability_voxel_layer.hpp"
#include "range_sensor_layer.hpp"
#include "range_sensor_layer2.hpp"
#include "static_layer.hpp"
#include "voxel_layer.hpp"
namespace CVTE_BABOT {

std::shared_ptr<Layer> StaticLayerCreator::creatLayer() {
  return std::make_shared<StaticLayer>();
}

std::shared_ptr<Layer> InflationLayerCreator::creatLayer() {
  return std::make_shared<InflationLayer>();
}

std::shared_ptr<Layer> ObstacleLayerCreator::creatLayer() {
  return std::make_shared<ObstacleLayer>();
}

std::shared_ptr<Layer> VoxelLayerCreator::creatLayer() {
  return std::make_shared<VoxelLayer>();
}

std::shared_ptr<Layer> ProbabilityVoxelLayerCreator::creatLayer() {
  return std::make_shared<ProbabilityVoxelLayer>();
}

std::shared_ptr<Layer> RangeSensorLayerCreator::creatLayer() {
  return std::make_shared<RangeSensorLayer2>();
}

std::shared_ptr<Layer> NegativeObstaclesLayerCreator::creatLayer() {
  return std::make_shared<NegativeObstaclesLayer>();
}

std::shared_ptr<Layer> CollisionLayerCreator::creatLayer() {
  return std::make_shared<CollisionLayer>();
}

std::shared_ptr<Layer> RangeSensorLayer2Creator::creatLayer() {
  return std::make_shared<RangeSensorLayer2>();
}

}  // namespace CVTE_BABOT
