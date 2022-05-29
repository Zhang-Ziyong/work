/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap2d_director.hpp
 *
 *@brief 构造2d costmap生成器director类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#ifndef __COSTMAP2D_DIRECTOR_HPP
#define __COSTMAP2D_DIRECTOR_HPP

#include <memory>
#include "costmap_builder.hpp"

namespace CVTE_BABOT {
class LayerParammeter;
class Costmap2dDirector {
 public:
  explicit Costmap2dDirector(std::shared_ptr<CostmapBuilder> builder);
  Costmap2dDirector(const Costmap2dDirector &obj) = delete;
  Costmap2dDirector &operator=(const Costmap2dDirector &obj) = delete;

  ~Costmap2dDirector() {}

  /**
   * buildCostmap
   * @brief 生成一个LayeredCostmap
   * */
  bool buildCostmap(const LayerParammeter &lp);

  /**
   * setCostmap2dBuilder
   * @brief 设置builder
   * @param[in] builder-生成器
   * */
  void setCostmap2dBuilder(std::shared_ptr<CostmapBuilder> builder);

 private:
  std::shared_ptr<CostmapBuilder> builder_ = nullptr;  ///< 生成器builder
};
}  // namespace CVTE_BABOT

#endif