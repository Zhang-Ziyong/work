/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file layer_creator.hpp
 *
 *@brief 工厂模式创建costmap layer, 隐藏创建细节.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#ifndef __LAYER_CREATOR_HPP
#define __LAYER_CREATOR_HPP
#include <memory>
#include "layer.hpp"

namespace CVTE_BABOT {

/**
 * LayerCreator
 * @brief
 * layer 工厂模式的creator,实际上不需要这么多layer的子类,
 * 可以通过参数决定创建的类型,只是单个类的复杂度就增加了
 *
 **/
class LayerCreator {
 public:
  LayerCreator() = default;
  ~LayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() = 0;
};

/**
 * StaticLayerCreator
 * @brief
 * StaticLayer 工厂模式的creator
 *
 **/
class StaticLayerCreator final : public LayerCreator {
 public:
  StaticLayerCreator() = default;
  ~StaticLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

/**
 * InflationLayerCreator
 * @brief
 * InflationLayer 工厂模式的creator
 *
 **/
class InflationLayerCreator final : public LayerCreator {
 public:
  InflationLayerCreator() = default;
  ~InflationLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

/**
 * ObstacleLayerCreator
 * @brief
 * ObstacleLayer 工厂模式的creator
 *
 **/
class ObstacleLayerCreator final : public LayerCreator {
 public:
  ObstacleLayerCreator() = default;
  ~ObstacleLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

/**
 * VoxelLayerCreator
 * @brief
 * ObstacleLayer 工厂模式的creator
 *
 **/
class VoxelLayerCreator final : public LayerCreator {
 public:
  VoxelLayerCreator() = default;
  ~VoxelLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

/**
 * VoxelLayerCreator
 * @brief
 * ObstacleLayer 工厂模式的creator
 *
 **/
class ProbabilityVoxelLayerCreator final : public LayerCreator {
 public:
  ProbabilityVoxelLayerCreator() = default;
  ~ProbabilityVoxelLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

/**
 * RangeSensorLayerCreator
 * @brief
 * RangeSensorLayer 工厂模式的creator
 *
 **/
class RangeSensorLayerCreator final : public LayerCreator {
 public:
  RangeSensorLayerCreator() = default;
  ~RangeSensorLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

/**
 * RangeSensorLayer2Creator
 * @brief
 * RangeSensor2Layer 工厂模式的creator
 *
 **/
class RangeSensorLayer2Creator final : public LayerCreator {
 public:
  RangeSensorLayer2Creator() = default;
  ~RangeSensorLayer2Creator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

class NegativeObstaclesLayerCreator final : public LayerCreator {
 public:
  NegativeObstaclesLayerCreator() = default;
  ~NegativeObstaclesLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

/**
 * CollisionLayerCreator
 * @brief
 * CollisionLayer 工厂模式的creator
 *
 **/
class CollisionLayerCreator final : public LayerCreator {
 public:
  CollisionLayerCreator() = default;
  ~CollisionLayerCreator() = default;

  /**
   * creatLayer
   * @brief
   * 新建一个layer
   *
   **/
  virtual std::shared_ptr<Layer> creatLayer() override;
};

}  // namespace CVTE_BABOT
#endif