/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: depth_camera_calibrate.hpp
 * @brief:
 * @
 * @
 * @author: caoyong(caoyong@cvte.com)
 * @version: v 1.0
 * @Date: 2021-04-09 16:28:33
 ************************************************************************/
#ifndef DEPTH_CAMERA_CALIBRATE
#define DEPTH_CAMERA_CALIBRATE

#include "occupancy_map/depth_camera_option.hpp"
#include "occupancy_map/occupancy_map_common.hpp"
#include "glog/logging.h"

namespace slam2d_core {
namespace occupancy_map {

class DepthCameraCalibrate {
 public:
  explicit DepthCameraCalibrate(const DepthCameraOptions &calib_config)
      : calib_config_(calib_config) {}

  bool segmentGround(const laserCloud::Ptr &cloud,
                     const Eigen::Matrix4f &transform,
                     laserCloud::Ptr &ground_cloud,
                     laserCloud::Ptr &obstacle_cloud);
  bool getAttitude(const laserCloud::Ptr &cloud,
                   Eigen::Vector4d &plane_coefficients);
  bool getWallAttitude(const laserCloud::Ptr &cloud,
                       Eigen::Vector4d &plane_coefficients);
  bool getTargetCloud(const laserCloud::Ptr &cloud,
                      laserCloud::Ptr &target_cloud, const bool is_ground);

  Eigen::Matrix4f frontUpGetTransform() const { return front_up_transform; }
  Eigen::Matrix4f frontDownGetTransform() const { return front_down_transform; }

  bool setTF(const DepthCameraOptions &calib_config) {
    for (unsigned int i = 0; i < calib_config.front_up_transform.size(); ++i) {
      front_up_transform(i / 4, i % 4) = calib_config.front_up_transform[i];
    }

    for (unsigned int i = 0; i < calib_config.front_down_transform.size();
         ++i) {
      front_down_transform(i / 4, i % 4) = calib_config.front_down_transform[i];
    }

    depth_dist_value = calib_config.depth_dist_value;
    depth_seg_ratio = calib_config.depth_seg_ratio;

    return true;
  }

  double depth_dist_value = 0.0;
  double depth_seg_ratio = 0.0;

 private:
  DepthCameraOptions calib_config_;
  Eigen::Vector4d default_plane_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d default_horizon_plane_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d plane_coefficients_ = Eigen::Vector4d::Zero();

  Eigen::Matrix4f front_up_transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f front_down_transform = Eigen::Matrix4f::Identity();
};
}  // namespace occupancy_map
}  // namespace slam2d_core

#endif