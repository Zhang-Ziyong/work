/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file layer.hpp
 *
 *@brief costmap layer基类.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#ifndef __LAYER_HPP
#define __LAYER_HPP
#include <boost/smart_ptr.hpp>
#include <map>
#include <memory>
#include <vector>
#include "costmap_utils.hpp"

namespace CVTE_BABOT {
class Costmap2d;
class CostmapMediator;
/**
 * Layer
 * @brief
 * 1.构建costmap layer的基类
 **/

class Layer {
 public:
  Layer();
  Layer(const Layer &obj) = delete;
  Layer &operator=(const Layer &obj) = delete;

  virtual ~Layer() = default;

  /**
   * initialize
   * @brief
   * 初始化layer
   *
   * @param[in] s_name-该层的名字
   * @return true-初始成功，false-初始失败
   * */
  bool initialize(const std::string &s_name);

  /**
   * useForAvoidance
   * @brief
   * 该层是否用于避障
   *
   * @return true-用于避障，false-不用于避障
   * */
  inline bool useForAvoidance() { return b_use_for_avoidance_; }

  inline bool useForNavigation() { return b_use_for_navigation_; }

  inline std::string getName() { return s_name_; }

  /**
   * getParams
   * @brief
   * 配置参数
   *
   * */
  virtual void getParams() = 0;

  /**
   * updateBounds
   * @brief
   * 根据机器人坐标以及层的大小，更新layer的边界
   *
   * @param[in] wp_robot_pose-包含机器人x，y坐标,单位m;角度，单位rad
   * @param[out] cb_costmap_bound-指示边界范围数据
   * @return true-初始成功，false-初始失败
   * */
  virtual bool updateBounds(const WorldmapPose &wp_robot_pose,
                            CostmapBound &cb_costmap_bound) = 0;

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
   * @return true-初始成功，false-初始失败
   * */
  virtual bool updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                           const int &i_min_i, const int &i_min_j,
                           const int &i_max_i, const int &i_max_j) = 0;

  virtual std::shared_ptr<Costmap2d> getLayerCostMap() = 0;

  /**
   * deactivate
   * @brief
   * 关闭该层
   *
   * */
  virtual void deactivate() {}

  /**
   * activate
   * @brief
   * 激活该层
   *
   * */
  virtual void activate() {}
  /**
   * @brief 是否已激活
   *
   */
  virtual bool isActivate() { return b_enabled_; }

  /**
   * reset
   * @brief
   * 重置该层
   *
   * */
  virtual void reset() {}

  /**
   * setCurrent
   * @brief
   *
   * 设置该层是否是最新的
   * @param[in] current-是否最新
   * */
  void setCurrent(const bool &current) { b_current_ = current; }

  /**
   * isCurrent
   * @brief
   *
   * 判断该层是否是最新的
   * @return true-是最新，false-不是
   * */
  inline bool isCurrent() const { return b_current_; }

  /**
   * matchSize
   * @brief
   * 使该层大小和父层大小一致
   *
   * */
  virtual void matchSize() {}

  /**
   * getName
   * @brief
   * 获取该层的名字
   *
   * */
  inline std::string getName() const { return s_name_; }

  /**
   * getFootprint
   * @brief
   * 获取机器人的足迹
   *
   * @return 返回机器人形状的所有点
   * */
  inline const std::vector<CostmapPoint> &getFootprint() const;

  /**
   * onFootprintChanged
   * @brief
   * 通知机器人的足迹已发生变化
   * */
  virtual void onFootprintChanged() {}

  /**
   * onInitialize
   * @brief
   * 用于后面的子类初始化
   *
   * @return true-初始成功，false-初始失败
   * */
  virtual bool onInitialize() { return true; }

  /**
   *isConvexPolygonFill
   *@brief
   *判断机器人多边形内有无障碍物
   *
   *@param[in] std::vector<WorldmapPoint> polygon-机器人的多边形
   *@return true-机器人多边形内有障碍物，false-机器人多边形内无障碍物
   * */
  virtual bool isConvexPolygonFill(const std::vector<WorldmapPoint> &) {
    return false;
  }

  inline void setLayerUpdate(bool update) { update_ = update; }
  inline bool getLayerUpdate() { return update_; }

  boost::shared_array<bool> ptr_field_;  ///< 磁力场

 protected:
  bool b_current_ = false;                ///< 判断是否是最新
  bool b_enabled_ = false;                ///< 判断是否已经使能
  std::string s_name_;                    ///< 层的名字
  bool b_use_for_navigation_ = true;      ///< 是否用于导航
  bool b_use_for_avoidance_ = false;      ///< 是否用于避障
  bool b_can_use_for_avoidance_ = false;  ///< 是否可用于避障

  std::map<std::string, SpeedLevel> m_speed_level_;  ///< 是否可用于避障
  std::map<std::string, std::vector<unsigned int>>
      m_v_field_index_;  ///< 每个传感器对应一个引力场
  std::map<std::string, std::vector<bool>>
      m_v_obstacle_area_;  ///< 每一个传感器对应一个障碍区域

  std::vector<WorldmapPoint> slow_bound_;  ///< 减速区
  std::vector<WorldmapPoint>
      origin_slow_bound_;  ///< 初始的减速区,未考虑运动方向
  std::vector<WorldmapPoint> stop_bound_;  ///< 停障区
  std::vector<WorldmapPoint>
      origin_stop_bound_;  ///< 初始的停障区,未考虑运动方向
  double extra_area_after_stop_size_x_;  ///< 停障后考虑的距离
  RobotType robot_type_;  ///< 机器人类型，前驱，后驱，中驱

 private:
  std::vector<CostmapPoint> footprint_spec_;  ///<机器人足迹

  bool update_ = true;
};

}  // namespace CVTE_BABOT

#endif
