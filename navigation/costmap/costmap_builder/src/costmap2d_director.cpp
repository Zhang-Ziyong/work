/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap2d_director.cpp
 *
 *@brief 2dcostmap生成器director类的具体实现.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#include "costmap2d_director.hpp"
#include <iostream>
#include "costmap_utils.hpp"
namespace CVTE_BABOT {
Costmap2dDirector::Costmap2dDirector(std::shared_ptr<CostmapBuilder> builder) {
  if (nullptr != builder) {
    builder_ = builder;
  } else {
    // std::cout << "Costmap2dDirector build failed" << std::endl;
  }
}

bool Costmap2dDirector::buildCostmap(const LayerParammeter &lp) {
  bool build_flag = false;
  for (auto it = lp.v_pair.begin(); it != lp.v_pair.end(); it++) {
    switch (it->first) {
      case StaticLayerType:
        build_flag = builder_->buildStaticLayer(it->second);
        break;
      case ObstacleLayerType:
        build_flag = builder_->buildObstacleLayer(it->second);
        break;
      case VoxelLayerType:
        build_flag = builder_->buildVoxelLayer(it->second);
        break;
      case ProbabilityVoxelLayerType:
        build_flag = builder_->buildProbabilityVoxelLayer(it->second);
        break;
      case RangeSensorLayerType:
        build_flag = builder_->buildRangeSensorLayer(it->second);
        break;
      case InflationLayerType:
        build_flag = builder_->buildInflationLayer(it->second);
        break;
      case NegativeObstaclesLayerType:
        build_flag = builder_->buildNegativeObstaclesLayer(it->second);
        break;
      case CollisionLayerType:
        build_flag = builder_->buildCollisionLayer(it->second);
        break;

      case RangeSensorLayer2Type:
        build_flag = builder_->buildRangeSensorLayer2(it->second);
        break;
      default:
        break;
    }
    if (!build_flag) {
      break;
    }
  }
  return true;
}

}  // namespace CVTE_BABOT
