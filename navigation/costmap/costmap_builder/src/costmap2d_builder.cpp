/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap2d_builder.cpp
 *
 *@brief 2dcostmap生成器builder类的具体实现.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#include <glog/logging.h>

#include "costmap2d_builder.hpp"

namespace CVTE_BABOT {

Costmap2dBuilder::Costmap2dBuilder(
    const std::shared_ptr<LayeredCostmap> ptr_layered_costmap) {
  ptr_layered_costmap_ = ptr_layered_costmap;
}
Costmap2dBuilder::~Costmap2dBuilder() {}

bool Costmap2dBuilder::buildInflationLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<InflationLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildInflationLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildInflationLayer failed.";
    return false;
  }
}

bool Costmap2dBuilder::buildObstacleLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<ObstacleLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildObstacleLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildObstacleLayer failed.";
    return false;
  }
}

bool Costmap2dBuilder::buildVoxelLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<VoxelLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildVoxelLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildVoxelLayer failed.";
    return false;
  }
}

bool Costmap2dBuilder::buildProbabilityVoxelLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<ProbabilityVoxelLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildProbabilityVoxelLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildProbabilityVoxelLayer failed.";
    return false;
  }
}

bool Costmap2dBuilder::buildStaticLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<StaticLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildStaticLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildStaticLayer failed.";
    return false;
  }
}
bool Costmap2dBuilder::buildRangeSensorLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<RangeSensorLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildRangeSensorLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildRangeSensorLayer failed.";
    return false;
  }
}

bool Costmap2dBuilder::buildNegativeObstaclesLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<NegativeObstaclesLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildNegativeObstaclesLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildNegativeObstaclesLayer failed.";
    return false;
  }
}

bool Costmap2dBuilder::buildCollisionLayer(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<CollisionLayerCreator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildCollisionLayer succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildCollisionLayer failed.";
    return false;
  }
}

bool Costmap2dBuilder::buildRangeSensorLayer2(const std::string &s_name) {
  std::shared_ptr<LayerCreator> ptr_layer_creator =
      std::make_shared<RangeSensorLayer2Creator>();
  std::shared_ptr<Layer> ptr_layer = ptr_layer_creator->creatLayer();
  ptr_layered_costmap_->addLayer(ptr_layer);
  if (ptr_layer->initialize(s_name)) {
    LOG(INFO) << "buildRangeSensorLayer2 succeed.";
    return true;
  } else {
    LOG(ERROR) << "buildRangeSensorLayer2 failed.";
    return false;
  }
}

std::shared_ptr<LayeredCostmap> Costmap2dBuilder::getLayeredCostmap() {
  return ptr_layered_costmap_;
}

}  // namespace CVTE_BABOT