/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file collision_layer.hpp
 *
 *@brief
 * 该层将碰撞开关信息转换成障碍物信息，用于导航避开该障碍物
 * 另外添加了超时机制，超时清除障碍物信息，减少动态障碍物影响
 *
 *@modified by chennuo(chennuo@cvte.com)
 *
 *@author chennuo(chennuo@cvte.com)
 *@data 2021-08-24
 ************************************************************************/
#ifndef __COLLISION_LAYER_HPP
#define __COLLISION_LAYER_HPP

#include <deque>
#include <unordered_set>
#include "costmap_cloud.hpp"
#include "costmap_layer.hpp"
#include "eigen3/Eigen/Core"

namespace CVTE_BABOT {
class CostmapCloudBuffer;
/**
 * CollisionLayer
 * @brief 实现costmap碰撞开关层
 **/
class CollisionLayer : public CostmapLayer {
 public:
  CollisionLayer();

  virtual ~CollisionLayer() = default;

  CollisionLayer(const CollisionLayer &obj) = delete;

  CollisionLayer &operator=(const CollisionLayer &obj) = delete;

  /**
   * onInitialize
   * @brief
   * 初始化当前层的参数，在基类的initialize函数中被调用
   *
   * @return true-初始成功，false-初始失败
   * */
  bool onInitialize() override;

  /**
   * matchSize
   * @brief
   * 根据master层地图大小调整当前障碍物层尺寸
   * */
  void matchSize() override;

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

  /**
   *updateOrigin
   *@brief
   *重写更新costmap原点接口
   *
   *@param[in] d_new_origin_x-原点x轴坐标
   *@param[in] d_new_origin_y-原点y轴坐标
   * */
  void updateOrigin(const double &d_new_origin_x,
                    const double &d_new_origin_y) override;

 protected:
  /**
   * resetMap
   * @brief
   * 清除costmap的数据
   *
   * */
  bool resetMap();

  bool b_rolling_window_;

  int i_combination_method_{1};  ///< 目标地图的更新方式

  // 碰撞开关话题
  std::string switch_topic_{""};

  // 碰撞开关上的三个特征点
  std::vector<double> switch_lp_{2, 0.0};
  std::vector<double> switch_mp_{2, 0.0};
  std::vector<double> switch_rp_{2, 0.0};
  double obs_keep_duration_{120.0};           // 障碍物保留时间
  boost::shared_array<time_t> ptr_obs_time_;  // 记录障碍物添加时刻
  std::unordered_set<unsigned int> hash_crash_index_;
  struct CollisionData {
    CollisionData(const time_t &_time, const Eigen::Vector3d &_pose) : 
      time(_time), pose(_pose){}
    time_t time;
    Eigen::Vector3d pose;
  };

  std::deque<CollisionData> q_collision_data_;
};
}  // namespace CVTE_BABOT

#endif