/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file layered_costmap.hpp
 *
 *@brief 用于合并costmap所有layer.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-03
 ************************************************************************/
#ifndef __COSTMAP_HPP
#define __COSTMAP_HPP

#include <iostream>
#include <memory>
#include <vector>

#include "costmap_2d.hpp"
#include "costmap_utils.hpp"
#include "layer.hpp"
namespace CVTE_BABOT {

class Layer;
class CostmapMediator;
/**
 * LayeredCostmap
 * @brief 用于合并所有layer
 **/

enum LayerName {
  laser_layer,
  probabiltiy_voxel_layer,
  negative_obstacle_layer,
  sonar_layer,
  range_layer,
  collision_layer,
};

class LayeredCostmap {
 public:
  LayeredCostmap(const bool &rolling_window, const bool &track_unknown);

  LayeredCostmap(const LayeredCostmap &ob) = delete;
  LayeredCostmap &operator=(const LayeredCostmap &ob) = delete;
  ~LayeredCostmap() = default;

  /**
   * isInitialized
   * @brief
   * 初始化参数、costmap等
   *
   * @return true-初始化正确， false-初始化不正确
   * */
  inline bool isInitialized() const { return initialized_; }

  /**
   * isRolling
   * @brief
   * 初始化
   *
   * @return true-是滚动 false-不是滚动
   * */
  inline bool isRolling() const { return rolling_window_; }

  /**
   * getCostmap
   * @brief
   * 获得costmap,即使返回的是const
   * 仍有可能会被修改的风险,但是返回拷贝的话成本太高,每被调用一次都需要重新拷贝
   *
   * @return costmap智能指针
   * */
  inline std::shared_ptr<const Costmap2d> getCostmap() const {
    std::unique_lock<std::recursive_mutex> lock(*(costmap_->getMutx()));
    return costmap_;
  }

  /**
   * isSizeLocked
   * @brief
   * 判断地图大小是否被锁住
   *
   * @return  true-已锁住，false-没有锁住
   * */
  inline bool isSizeLocked() const { return size_locked_; }

  /**
   * isMarkingUnknown
   * @brief
   * 判断地图类型是否跟踪未知区域
   *
   * @return true-是，false-否
   * */
  inline bool isMarkingUnknown() const {
    return costmap_->getDefaultValue() == CVTE_BABOT::NO_INFORMATION;
  }

  /**
   * getInscribedRadius
   * @brief
   * 获得以机器人原点为中心的半径
   *
   * @return inscribed_radius_-以机器人原点为中心的半径
   * */
  inline double getInscribedRadius() const { return inscribed_radius_; }

  /**
   * getInscribedRadius
   * @brief
   * 获得以机器人外切圆半径
   *
   * @return circumscribed_radius_-以机器人外切圆半径
   * */
  inline double getCircumscribedRadius() const { return circumscribed_radius_; }

  /**
   * setFootprint
   * @brief
   * 设置机器人的足迹
   *
   * @param[in] footprint_spec-机器人的足迹
   * */
  void setFootprint(const std::vector<WorldmapPoint> &footprint_spec);

  void getFootprint(std::vector<WorldmapPoint> &footprint_spec);
  /**
   * getFootprint
   * @brief
   * 获取机器人的足迹
   *
   * @return[in] footprint_-机器人的足迹
   * */
  const std::vector<WorldmapPoint> &getFootprint() const { return footprint_; }

  /**
   * getBounds
   * @brief
   * 获得地图的边界
   *
   * @param[out] bound-机器人边界
   * */
  void getBounds(CostmapBound &bound) const { bound = cb_bound_; }

  /**
   *getLayers
   * @brief
   * 获得所有的layer
   *
   * @return layers_-所有的layer
   * */
  inline std::shared_ptr<std::vector<std::shared_ptr<Layer>>> getLayers() {
    return layers_;
  }

  /**
   * updateMap
   * @brief
   * 根据机器人坐标更新costmap
   *
   * @param[in] robot_pose-机器人姿态
   * @return true-成功，false-失败
   * */
  bool updateMap(const WorldmapPose &robot_pose);

  /**
   * resizeMap
   * @brief
   * 根据机器人坐标更新costmap
   *
   * @param[in] size_x-地图x坐标大小,单位个
   * @param[in] size_y-地图y坐标大小,单位个
   * @param[in] resolution-地图分辨率,单位m
   * @param[in] origin_x-地图原点x坐标,
   * @param[in] origin_y-地图原点y坐标,
   * @param[in] size_locked-地图是否锁住
   * @return true-成功，false-失败
   * */
  bool resizeMap(const unsigned int &size_x, const unsigned int &size_y,
                 const double &resolution, const double &origin_x,
                 const double &origin_y, const bool &size_locked = false);

  /**
   * addLayer
   * @brief
   * 增加layer
   *
   * @param[in] layer-具体层
   * */
  bool addLayer(std::shared_ptr<Layer> layer);

  void setLayerCurrent(const std::string &layer_name, const bool &current);

  std::shared_ptr<Costmap2d> getLayerCostmapByname(
      const std::string layer_name);

  void activeLayer(LayerName layer);

  void deactiveLayer(LayerName layer);

  void resetLayer(LayerName layer);

  inline void setCleanAreaEalbe() { clear_by_areas_ = true; }

  bool setUpdateInShope(const bool update_in_shope);

  bool setCleanAreas(const std::vector<std::vector<WorldmapPose>> &clear_areas);

  bool cleanMapByAreas(const WorldmapPose &robot_pose,
                       const std::shared_ptr<Costmap2d> cost_map);
  bool isPointInPolygon(const int x, const int y,
                        const std::vector<std::vector<int>> &points);
  bool isPointInPolygon(const double x, const double y,
                        const std::vector<WorldmapPose> &points);

  bool isPointInAreas(const WorldmapPose &point);

 private:
  bool initialized_;            ///< 是否已经初始化
  bool size_locked_;            ///< 是否被锁住
  bool rolling_window_ = true;  ///< 是否滚动
  CostmapBound cb_bound_;       ///< costmap的边界

  CostmapBound cb_extremum_bound_;  ///< costmap坐标的最大最小值
  std::shared_ptr<Costmap2d> costmap_ = nullptr;  ///< costmap2d
  std::recursive_mutex mutex_;
  double inscribed_radius_;      ///< 以及机器人圆形为起点的半径
  double circumscribed_radius_;  ///<以及机器人外切圆为起点的半径
  std::vector<WorldmapPoint> footprint_;  ///< 机器人足迹
  std::shared_ptr<std::vector<std::shared_ptr<Layer>>> layers_ =
      nullptr;                           ///< 所有层
  SpeedLevel speedLevel_ = SpeedMax;     ///<当前可行驶速度
  boost::shared_array<bool> ptr_field_;  ///< 磁力场
  std::vector<bool> stop_obstacle_area_;  ///< 障碍物出现的区域，如前后左右

  std::vector<std::vector<WorldmapPose>> clear_areas_;
  std::map<LayerName, std::string> map_layer_names_;
  bool clear_by_areas_ = false;
  bool update_in_shope_ = false;
  bool reset_shope_layer_ = false;
};

}  // namespace CVTE_BABOT

#endif