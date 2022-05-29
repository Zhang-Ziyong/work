/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_builder.hpp
 *
 *@brief 构造costmap生成器builder基类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#ifndef __COSTMAP_BUILDER_HPP
#define __COSTMAP_BUILDER_HPP

#include <memory>
#include <string>
/**
 * CostmapBuilder
 * @brief costmap各层生成器的builder：
 * 1.生成器的builder
 * 2.用于builder不同类型的inflationLayer等,但是目前看来不需要,inflationLayer
 * 这些都只有一种,不需要基类,而且如果通过工厂模式创建各layer,可以通过参数决定创建的
 * layer类型,不需要这么多buildInflationLayer,buildVoxelLayer
 **/

namespace CVTE_BABOT {
class LayeredCostmap;

class CostmapBuilder {
 public:
  CostmapBuilder() {}
  CostmapBuilder(const CostmapBuilder &obj) = delete;
  CostmapBuilder &operator=(const CostmapBuilder &obj) = delete;
  ~CostmapBuilder() = default;

  /**
   * buildInflationLayer
   * @brief 生成膨胀层
   * */
  virtual bool buildInflationLayer(const std::string &) { return true; }

  /**
   * buildObstacleLayer
   * @brief 生成障碍物层
   * */
  virtual bool buildObstacleLayer(const std::string &) { return true; }

  /**
   * buildVoxelLayer
   * @brief 生成体素层
   * */
  virtual bool buildVoxelLayer(const std::string &) { return true; }

  /**
   * buildStaticLayer
   * @brief 生成静态层
   * */
  virtual bool buildStaticLayer(const std::string &) { return true; }

  /**
   * buildRangeSensorLayer
   * @brief 生成超声层
   * */
  virtual bool buildRangeSensorLayer(const std::string &) { return true; }

  /**
   * buildNegativeObstaclesLayer
   * @brief 生成2d costmap负障碍物层
   * */
  virtual bool buildNegativeObstaclesLayer(const std::string &s_name) {
    return true;
  }

  /**
   * buildProbabilityVoxelLayer
   * @brief 生成概率体素障碍物层
   * */
  virtual bool buildProbabilityVoxelLayer(const std::string &s_name) {
    return true;
  }

  /**
   * buildCollisionLayer
   * @brief 生成碰撞障碍物层
   * */
  virtual bool buildCollisionLayer(const std::string &s_name) { return true; }

  virtual bool buildRangeSensorLayer2(const std::string &s_name) {
    return true;
  }

  /**
   * getLayeredCostmap
   * @brief 获取LayeredCostma(对外的接口)
   *
   * @return layered_costmap_- LayerCostmap(用于管理所有的层)
   * */
  virtual std::shared_ptr<LayeredCostmap> getLayeredCostmap() {
    return nullptr;
  }
};

}  // namespace CVTE_BABOT

#endif