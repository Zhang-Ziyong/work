/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_cloud.hpp
 *
 *@brief sensor data storage class.
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *
 *@modified huabo wu(wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __COSTMAP_CLOUD_HPP
#define __COSTMAP_CLOUD_HPP
#include <memory>
#include <vector>

#include "costmap_utils.hpp"

namespace CVTE_BABOT {

/**
 * CostmapCloud
 * @brief
 * costmap中点云数据格式，比如激光雷达
 *
 **/
struct CostmapCloud {
  CostmapCloud()
      : obstacle_range_(0.0),
        raytrace_range_(0.0),
        ptr_v_cloud_(std::make_shared<CostmapPointCloud>()) {}

  /*deep copy*/
  CostmapCloud(const CostmapCloud &obs)
      : obstacle_range_(obs.obstacle_range_),
        raytrace_range_(obs.obstacle_range_),
        origin_(obs.origin_),
        ptr_v_cloud_(std::make_shared<CostmapPointCloud>(*(obs.ptr_v_cloud_))) {
  }

  /*deep copy*/
  CostmapCloud &operator=(const CostmapCloud &obs) {
    origin_ = obs.origin_;
    obstacle_range_ = obs.obstacle_range_;
    raytrace_range_ = obs.obstacle_range_;
    world_pose_ = obs.world_pose_;
    ptr_v_cloud_ = std::make_shared<CostmapPointCloud>(*(obs.ptr_v_cloud_));
    return *this;
  }

  ~CostmapCloud() = default;

  std::shared_ptr<CostmapCloud> transformToMap();

  std::shared_ptr<CostmapCloud> transformToBaseLink();

  std::string s_topic_name_ = "empty";
  double obstacle_range_ = 0.0;  ///< 障碍物范围
  double raytrace_range_ = 0.0;  ///< 射线范围
  double v_fov_ = 0;             ///<  垂直fov
  double h_fov_ = 0;             ///< 水平fov
  double min_d_ = 0;             ///<  最小测量距离
  double max_d_ = 0;             ///< 最大测量距离
  double negative_min_d_ = 0;    ///< 负向障碍最小测量距离
  double negative_max_d_ = 0;    ///< 负向障碍物最大测量距离
  double min_h_ = 0;
  double max_h_ = 1.0;
  CostmapPointXYZ origin_;  ///< 传感器位置

  SensorPose sensor_pose_;   // 传感器到定位数据相对位置
  WorldmapPose world_pose_;  // 转换到世界坐标的变换

  bool b_need_transformed_;  // 是否需要转换到世界坐标
  std::shared_ptr<CostmapPointCloud> ptr_v_cloud_ = nullptr;  ///< 点云
  bool is_used_to_mark_ = true;
  bool is_used_to_clear_ = true;
};

}  // namespace CVTE_BABOT
#endif  // __COSTMAP_CLOUD_HPP