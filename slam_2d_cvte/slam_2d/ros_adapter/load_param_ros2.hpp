/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file load_param_ros2.hpp
 *
 *@brief
 * 相关参数配置类
 *
 *@author caoyong(caoyong@cvte.com)
 *@version v1.0
 *@data 2022-04-27
 ************************************************************************/
#ifndef _NEW_LIDAR_SLAM_LOAD_PARAM_
#define _NEW_LIDAR_SLAM_LOAD_PARAM_

#include <rclcpp/rclcpp.hpp>
#include "common/config/system_config.hpp"
#include "occupancy_map/depth_camera_occ_map.hpp"

namespace cvte_lidar_slam {

class LoadParamsRos2 {
 public:
  explicit LoadParamsRos2(const rclcpp::Node::SharedPtr node_ptr);
  ~LoadParamsRos2();

  LoadParamsRos2(const LoadParamsRos2 &) = delete;
  LoadParamsRos2 &operator=(const LoadParamsRos2 &) = delete;

  void loadBackConfig();

  void loadSystemConfig();

  void loadLoopConfig();

  void loadMSFConfig();

  void loadOccMapConfig();

  void loadDepthCameraParams();

  void loadFeatureConfig();

  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;

  DepthCameraOptions depth_camera_options_;
  DepthOccupancyMapOptions depth_occupancy_map_options_;

  template <typename ParameterT>
  void getParameter(const std::string &name, ParameterT &value,
                    const ParameterT &alternative_value) const;

 private:
  rclcpp::Node::SharedPtr node_ptr_;  ///< 节点
};

}  // namespace cvte_lidar_slam

#endif
