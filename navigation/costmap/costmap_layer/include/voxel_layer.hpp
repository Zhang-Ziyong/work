/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file voxel_layer.hpp
 *
 *@brief costmap voxel_grid layer.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.2.1
 *@data 2019-08-22
 ************************************************************************/
#ifndef __VOXEL_LAYER_HPP
#define __VOXEL_LAYER_HPP

#include <time.h>
#include "obstacle_layer.hpp"
#include "voxel_grid.hpp"

namespace CVTE_BABOT {
typedef time_t timeType;
class VoxelLayer : public ObstacleLayer {
  friend class voxel_layer_tester;

 public:
  VoxelLayer() : voxel_grid_(0, 0, 0) {}

  virtual ~VoxelLayer() = default;

  VoxelLayer(const VoxelLayer& obj) = delete;
  VoxelLayer& operator=(const VoxelLayer& obj) = delete;

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
  bool updateBounds(const WorldmapPose& wp_robot_pose,
                    CostmapBound& cb_costmap_bound) override;

  /**
   *updateOrigin
   *@brief
   *更新costmap原点
   *
   *@param[in] d_new_origin_x-原点x轴坐标
   *@param[in] d_new_origin_y-原点y轴坐标
   * */
  void updateOrigin(const double& d_new_origin_x,
                    const double& d_new_origin_y) override;

  /**
   * matchSize
   * @brief
   * 使该层大小和父层大小一致
   *
   * */
  void matchSize() override;

 private:
  /**
  *resetMaps
  *@brief
  *将costmap重新赋值默认代价
  *
  * */
  void resetMaps() override;

  /**
   * raytraceFreespace
   * @brief
   * 根据点云获取需要更新的范围
   * @param[in] clearing_clouds-点云数据
   * @param[out] costmap_bound-需要更新的范围
   *
   * */
  void raytraceFreespace(const CostmapCloud& clearing_clouds,
                         CostmapBound& costmap_bound) override;

  /**
   * clearNonLethal
   * @brief 清除范围内不正常的点，置为FREE_SPACE
   *
   * @param[in] wp_point-原点
   * @param[in] wp_size-以原点开始清除的半径
   * @param[in] clear_no_info-是否把NO_INFORMATION的点清除
   */
  void clearNonLethal(const WorldmapPoint& wp_point,
                      const WorldmapPoint& wp_size, const bool& clear_no_info);
  /**
   * worldToMap3DFloat
   * @brief 世界坐标转地图浮点坐标
   *
   * @param[in] world_point-世界坐标xyz
   * @param[out] map_point-地图浮点坐标xyz
   * @return true-在地图内
   * @return false-超出地图
   */
  inline bool worldToMap3DFloat(const CostmapPointXYZ& cp_world,
                                CostmapPointXYZ& cp_map) {
    if (cp_world.d_x < d_origin_x_ || cp_world.d_y < d_origin_y_ ||
        cp_world.d_z < d_origin_z_) {
      return false;
    }

    cp_map.d_x = ((cp_world.d_x - d_origin_x_) / d_resolution_);
    cp_map.d_y = ((cp_world.d_y - d_origin_y_) / d_resolution_);
    cp_map.d_z = ((cp_world.d_z - d_origin_z_) / d_resolution_z_);

    if (cp_map.d_x < ui_size_x_ && cp_map.d_y < ui_size_y_ &&
        cp_map.d_z < ui_size_z_) {
      return true;
    }

    return false;
  }

  /**
   * worldToMap3D
   * @brief 世界坐标转地图整型坐标
   *
   * @param[in] cp_world-世界坐标xyz
   * @param[out] vp_map-地图坐标xyz
   * @return true-在地图内
   * @return false-超出地图
   */
  inline bool worldToMap3D(const CostmapPointXYZ& cp_world,
                           VoxelPoint& vp_map) {
    if (cp_world.d_x < d_origin_x_ || cp_world.d_y < d_origin_y_ ||
        cp_world.d_z < d_origin_z_) {
      return false;
    }
    vp_map.ui_x =
        static_cast<int>((cp_world.d_x - d_origin_x_) / d_resolution_);
    vp_map.ui_y =
        static_cast<int>((cp_world.d_y - d_origin_y_) / d_resolution_);
    vp_map.ui_z =
        static_cast<int>((cp_world.d_z - d_origin_z_) / d_resolution_z_);

    if (vp_map.ui_x < ui_size_x_ && vp_map.ui_y < ui_size_y_ &&
        vp_map.ui_z < ui_size_z_) {
      return true;
    }

    return false;
  }

  /**
   * mapToWorld3D
   * @brief 地图坐标转世界坐标
   *
   * @param[in] vp_map-地图坐标xyz
   * @param[out] cp_world-世界坐标xyz
   */
  inline void mapToWorld3D(const VoxelPoint& vp_map,
                           CostmapPointXYZ& cp_world) {
    // returns the center point of the cell
    cp_world.d_x = d_origin_x_ + (vp_map.ui_x + 0.5) * d_resolution_;
    cp_world.d_y = d_origin_y_ + (vp_map.ui_y + 0.5) * d_resolution_;
    cp_world.d_z = d_origin_z_ + (vp_map.ui_z + 0.5) * d_resolution_z_;
  }

  /**
   * dist
   * @brief
   *
   * @param[in] wp0-地图点1
   * @param[in] wp1-地图点2
   * @return double-两点的距离
   */
  inline double dist(const CostmapPointXYZ& wp0, const CostmapPointXYZ& wp1) {
    return sqrt((wp1.d_x - wp0.d_x) * (wp1.d_x - wp0.d_x) +
                (wp1.d_y - wp0.d_y) * (wp1.d_y - wp0.d_y) +
                (wp1.d_z - wp0.d_z) * (wp1.d_z - wp0.d_z));
  }

 private:
  bool b_publish_voxel_;               ///< 可视化体素
  bool b_publish_clearing_points_;     ///< 可视化点云波束末端
  std::string s_cleraing_cloud_name_;  ///< 点云波束末端的topic

  std::vector<CostmapPointXYZ> v_clearing_endpoints_;

  VoxelGrid voxel_grid_;
  boost::shared_array<timeType> ptr_d_obstacle_time_;
  double d_resolution_z_, d_origin_z_;
  unsigned int unknown_threshold_, mark_threshold_, ui_size_z_;
  double d_clearing_time_threshold_;
  double d_quick_clearing_time_threshold_;
  int i_quick_clearing_bit_threshold_;
};

}  // namespace CVTE_BABOT

#endif  // __VOXEL_LAYER_HPP
