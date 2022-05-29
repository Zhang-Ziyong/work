/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap2d_builder.hpp
 *
 *@brief 构造2d costmap生成器builder类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#ifndef __COSTMAP2D_BUILDER_HPP
#define __COSTMAP2D_BUILDER_HPP

#include <memory>

#include "costmap_builder.hpp"
#include "layered_costmap.hpp"

#include "layer_creator.hpp"

namespace CVTE_BABOT {

class Costmap2dBuilder final : public CostmapBuilder {
 public:
  Costmap2dBuilder(const std::shared_ptr<LayeredCostmap> ptr_layered_costmap);
  Costmap2dBuilder(const Costmap2dBuilder &obj) = delete;
  Costmap2dBuilder &operator=(const Costmap2dBuilder &obj) = delete;

  ~Costmap2dBuilder();

  /**
   * buildInflationLayer
   * @brief 生成2d costmap膨胀层
   * */
  bool buildInflationLayer(const std::string &s_name) final;

  /**
   * buildObstacleLayer
   * @brief 生成2d costmap障碍物层
   * */
  bool buildObstacleLayer(const std::string &s_name) final;

  /**
   * buildVoxelLayer
   * @brief 生成3d costmap体素层
   * */
  bool buildVoxelLayer(const std::string &s_name) final;

  /**
   * buildStaticLayer
   * @brief 生成2d costmap静态层
   * */
  bool buildStaticLayer(const std::string &s_name) final;

  /**
   * buildRangeSensorLayer
   * @brief 生成2d costmap超声层
   * */
  bool buildRangeSensorLayer(const std::string &s_name) final;

  /**
   * buildNegativeObstaclesLayer
   * @brief 生成2d costmap负障碍物层
   * */
  bool buildNegativeObstaclesLayer(const std::string &s_name) final;

  /**
   * buildProbabilityVoxelLayer
   * @brief 生成概率体素障碍物层
   * */
  bool buildProbabilityVoxelLayer(const std::string &s_name) final;

  /**
   * buildCollisionLayer
   * @brief 生成2d costmap碰撞开关层
   * */
  bool buildCollisionLayer(const std::string &s_name) final;

  bool buildRangeSensorLayer2(const std::string &s_name) final;
  /**
   * getLayeredCostmap
   * @brief 获取LayeredCostma(对外的接口)
   *
   * @return layered_costmap_- LayerCostmap(用于管理所有的层)
   * */
  std::shared_ptr<LayeredCostmap> getLayeredCostmap() final;

 private:
  std::shared_ptr<LayeredCostmap> ptr_layered_costmap_ =
      nullptr;  ///< 用于管理所有layer
};

}  // namespace CVTE_BABOT

#endif