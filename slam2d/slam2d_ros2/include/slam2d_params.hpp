/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file slam2d_params.hpp
 *
 *@brief
 * slam2d_core相关参数配置类
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version v1.0
 *@data 2021-04-01
 ************************************************************************/
#ifndef _SLAM2D_PARAMS_ROS2_H_
#define _SLAM2D_PARAMS_ROS2_H_

#include <rclcpp/rclcpp.hpp>

#include "amcl/amcl.hpp"
#include "backend/pose_graph.hpp"
#include "frontend/local_trajectory_builder.hpp"
#include "msf/multi_sensors_fusion.hpp"
#include "occupancy_map/depth_camera_option.hpp"
#include "scan_matcher/global_scan_matcher_2d.hpp"
#include "slam_system/slam_system.hpp"

namespace slam2d_ros2 {

class Slam2dParamsRos2 {
 public:
  explicit Slam2dParamsRos2(const rclcpp::Node::SharedPtr node_ptr);
  ~Slam2dParamsRos2();

  Slam2dParamsRos2(const Slam2dParamsRos2 &) = delete;
  Slam2dParamsRos2 &operator=(const Slam2dParamsRos2 &) = delete;

  void loadPoseGraphParams();
  void loadLocalTrajectoryBuilderParams();
  void loadLocalizationParams();

  void loadAmclParams();
  void loadMSFParams();
  void loadGSMParams();

  void loadDepthCameraParams();

  slam2d_core::backend::PoseGraphOptions pose_graph_options_;
  slam2d_core::frontend::LocalTrajectoryBuilderOptions
      local_trajectory_options_;

  slam2d_core::slam_system::LocalizationOptions loc_options;

  slam2d_core::amcl::AmclOptions amcl_options_;
  slam2d_core::msf::MultiSensorsFusionOptions msf_options_;
  slam2d_core::scan_matcher::GlobalScanMatcher2DOptions gsm_options_;

  slam2d_core::occupancy_map::DepthCameraOptions depth_camera_options_;
  slam2d_core::occupancy_map::DepthOccupancyMapOptions
      depth_occupancy_map_options_;

 private:
  rclcpp::Node::SharedPtr node_ptr_;  ///< 节点
  template <typename ParameterT>
  void getParameter(const std::string &name, ParameterT &value,
                    const ParameterT &alternative_value) const;
};

}  // namespace slam2d_ros2

#endif