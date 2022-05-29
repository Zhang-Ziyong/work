/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file static_layer.hpp
 *
 *@brief costmap static layer.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-18
 ************************************************************************/
#ifndef __STATIC_LAYER_HPP
#define __STATIC_LAYER_HPP

#include <world_map_data.hpp>
#include "costmap_layer.hpp"
#include "inflation_layer.hpp"

namespace CVTE_BABOT {

/**
 * StaticLayer
 * @brief
 * 加载静态地图，生成膨胀层
 *
 **/
class StaticLayer final : public CostmapLayer {
  friend class Static_layer_tester;

 public:
  StaticLayer() : ptr_worldmap_data_(nullptr) {}
  ~StaticLayer() = default;

  StaticLayer &operator=(const StaticLayer &obj) = delete;
  StaticLayer(const StaticLayer &obj) = delete;

  /**
   * reset
   * @brief
   * 重置该层
   *
   * */
  void reset() override;

  /**
   *getParams
   *@brief
   *配置参数
   *
   * */
  void getParams() override;
  /**
   * matchSize
   * @brief
   * 使该层大小和父层大小一致
   *
   * */
  void matchSize() override;

  /**
   * activate
   * @brief
   * 激活该层
   *
   * */
  void activate() override;

  /**
   * deactivate
   * @brief
   * 关闭该层
   *
   * */
  void deactivate() override;

  /**
   * onInitialize
   * @brief
   * 用于该层内部的初始化
   *
   * */
  bool onInitialize() override;

  unsigned char interpretValue(const unsigned char &value);

  /**
   * updateBounds
   * @brief
   * 根据机器人坐标以及层的大小，更新layer的边界
   *
   * @param[in] wp_robot_pose-包含机器人x，y坐标,单位m;角度，单位rad
   * @param[out] cb_costmap_bound-指示边界范围数据
   *
   * */
  bool updateBounds(const WorldmapPose &wp_robot_pose,
                    CostmapBound &cb_costmap_bound) override;

  /**
   * updateCosts
   * @brief
   * 更新costmap的代价值
   *
   * @param[in] master_grid-更新的costmap对象
   * @param[in] i_min_i-更新范围的最小x
   * @param[in] i_min_j-更新范围的最小x
   * @param[in] i_max_i-更新范围的最大y
   * @param[in] i_max_j-更新范围的最大y
   *
   * */
  bool updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                   const int &i_min_i, const int &i_min_j, const int &i_max_i,
                   const int &i_max_j) override;

  typedef std::shared_ptr<StaticLayer> StaticLayerPtr;  ///< 声明一个智能指针

 private:
  /**
   * getMap
   * @brief
   * 加载一张全局静态地图
   *
   * */
  void loadMap();

  bool resetMap();

  /**
   *inflateStaticMap
   *@brief
   *膨用静态地图,使静态栅格连续,用于判断点云是否与静态地图上的障碍重叠
   *
   *@param[in] inflate_radius-膨胀的栅格半径
   *@param[in] inflate_cost-膨胀的代价
   *@param[in/out] master_grid-原始静态地图指针,膨胀结果直接更新在该地图上
   **/
  void inflateStaticMap(const unsigned int &ui_inflate_radius,
                        const unsigned char &inflate_cost,
                        const std::shared_ptr<Costmap2d> &master_grid);

  /**
   *inflateStaticMapGridient
   *@brief
   *膨胀静态地图，以梯度下降的方式来对静态地图进行膨胀
   *
   *@param[in/out] master_grid-原始静态地图指针,膨胀结果直接更新在该地图上
   **/
  void inflateStaticMapGridient(const std::shared_ptr<Costmap2d> &master_grid);

  bool has_updated_data_;      ///< 是否已经更新数据
  bool b_mark_unknown_space_;  ///< 是否扩展未知区域
  bool use_maximum_;           ///< 是否使用最大值
  bool trinary_costmap_;       ///< 是否是三元代码地图

  CostmapPoint
      cp_update_origin_;  ///<
                          ///地图更新的起点,传入新地图时,地图可叠加,cp_update_origin_指示了新的更新起点
  unsigned int width_;    ///< 地图的宽
  unsigned int height_;   ///< 地图的高
  unsigned char lethal_threshold_;    ///< 障碍物阈值
  unsigned char unknown_cost_value_;  ///< 不确定区域代价
  unsigned char uc_inflate_cost_;     ///< 膨胀的代价值

  WorldmapPose robot_current_pose_;                  ///< 机器人当前姿态
  std::shared_ptr<WorldmapData> ptr_worldmap_data_;  ///< 原始地图数据

  InflationLayer inflater_;  ///< 地图膨胀器
};
}  // namespace CVTE_BABOT

#endif
