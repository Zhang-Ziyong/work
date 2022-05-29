/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file obstacle_layer.hpp
 *
 *@brief costmap obstacle layer.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-08-22
 ************************************************************************/
#ifndef __OBSTACLE_LAYER_HPP
#define __OBSTACLE_LAYER_HPP
#include <mutex>

#include "costmap_cloud.hpp"
#include "costmap_layer.hpp"
namespace CVTE_BABOT {
class CostmapCloudBuffer;
/**
 * ObstacleLayer
 * @brief 实现costmap障碍层，有如下特征：
 * 获取点云数据costmap_cloud更新地图，可由2d、3d、相机数据转化得来
 **/
class ObstacleLayer : public CostmapLayer {
  friend class obstacle_layer_tester;

 public:
  ObstacleLayer();

  virtual ~ObstacleLayer() = default;

  ObstacleLayer(const ObstacleLayer &obj) = delete;

  ObstacleLayer &operator=(const ObstacleLayer &obj) = delete;

  /**
   * onInitialize
   * @brief
   * 初始化当前层的参数，在基类的initialize函数中被调用
   *
   * @return true-初始成功，false-初始失败
   * */
  bool onInitialize() override;

  /**
   * activate
   * @brief
   * 使能当前层
   *
   * */
  void activate() override;

  /**
   * deactivate
   * @brief
   * 使当前层失效不更新
   *
   * */
  void deactivate() override;

  /**
   * reset
   * @brief
   * 重置当前层的数据
   *
   * */
  void reset() override;

  /**
   * getParams
   * @brief
   * 初始化当前层的参数
   *
   * */
  void getParams() override;

  /**
   * updateBounds
   * @brief
   * 根据机器人坐标以及层的大小，更新layer的边界
   *
   * @param[in] wp_robot_pose-包含机器人x，y坐标,单位m;角度，单位rad
   * @param[out] cb_costmap_bound-指示边界范围数据
   * @return true-更新成功，false-更新失败
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
   * @return true-更新成功，false-更新失败
   * */
  bool updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                   const int &i_min_i, const int &i_min_j, const int &i_max_i,
                   const int &i_max_j) override;

 protected:
  /**
   * resetMap
   * @brief
   * 清除costmap的数据
   *
   * */
  bool resetMap();

  /**
   * createSubscribers
   * @brief
   * 向mediator注册数据类型及传入数据的函数
   *
   * */
  void createSubscribers();

  /**
   * getMarkingClouds
   * @brief
   * 获取用于标记地图的数据
   * @param[out] marking_clouds-用于标记地图的数据
   *
   * @return true-获取成功，false-获取失败
   * */
  virtual bool getMarkingClouds(
      std::vector<std::shared_ptr<const CostmapCloud>> &marking_clouds) const;

  /**
   * getClearingClouds
   * @brief
   * 获取用于标记地图的数据
   * @param[out] clearing_clouds-用于清除地图的数据
   *
   * @return true-获取成功，false-获取失败
   * */
  virtual bool getClearingClouds(
      std::vector<std::shared_ptr<const CostmapCloud>> &clearing_clouds) const;

  /**
   * getAvoidanceClouds
   * @brief
   * 获取用于避障的数据
   * @param[out] avoidance_clouds-用于避障的数据
   *
   * @return true-获取成功，false-获取失败
   * */
  bool getAvoidanceClouds(
      std::vector<std::shared_ptr<const CostmapCloud>> &avoidance_clouds) const;

  /**
   * updateFootprint
   * @brief
   * 根据足迹获取需要更新的范围
   * @param[in] WorldmapPose robot_pose-当前位置
   * @param[out] CostmapBound costmap_bound-需要更新的范围
   *
   * */
  void updateFootprint(const WorldmapPose &, CostmapBound &);

  /**
   * raytraceFreespace
   * @brief
   * 根据点云获取需要更新的范围
   * @param[in] clearing_clouds-点云数据
   * @param[out] costmap_bound-需要更新的范围
   *
   * */
  virtual void raytraceFreespace(const CostmapCloud &clearing_clouds,
                                 CostmapBound &costmap_bound);

  /**
   * updateRaytraceBounds
   * @brief
   * 根据两点组成的直线获取需要更新的范围
   * @param[in] wp_origin-世界地图更新起点
   * @param[in] wp_point-世界地图更新终点
   * @param[in] range-更新距离
   * @param[out] costmap_bound-需要更新的范围
   *
   * */
  void updateRaytraceBounds(const WorldmapPoint &wp_origin,
                            const WorldmapPoint &wp_point, const double &range,
                            CostmapBound &costmap_bound);

  void resetObstacleArea(std::vector<bool> &obstacle_area) {
    if (obstacle_area.size() != OBSTACLE_DIREC_AREA) {
      obstacle_area.resize(OBSTACLE_DIREC_AREA);
    }

    for (size_t i = 0; i < OBSTACLE_DIREC_AREA; i++) {
      obstacle_area[i] = false;
    }
  }

  bool b_rolling_window_;

  bool b_footprint_clearing_enabled_;  ///< 是否清除足迹里的障碍标志
  std::vector<WorldmapPoint> v_transformed_footprint_;  ///< 坐标转换后的足迹

  double d_max_obstacle_height_;  ///< 使用数据的最大高度
  double d_min_obstacle_height_;  ///< 使用数据的最小高度

  double d_obstacle_range_ = 0.00;
  double d_raytrace_range_ = 0.00;

  int i_combination_method_;  ///< 目标地图的更新方式

  std::vector<std::string> v_marking_cloud_names_;  ///< 用于标记数据的数据名
  std::vector<std::string> v_clearing_cloud_names_;  ///< 用于清除数据的数据名
  std::vector<std::string> v_avoidance_cloud_names_;  ///< 用于避障的数据名
  std::recursive_mutex mutex_;
};
}  // namespace CVTE_BABOT

#endif