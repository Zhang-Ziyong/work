/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: depth_camera_calibrate.cpp
 * @brief:
 * @
 * @
 * @author: caoyong(caoyong@cvte.com)
 * @version: v 1.0
 * @Date: 2021-04-09 16:28:14
 ************************************************************************/

#include "occupancy_map/depth_camera_calibrate.hpp"
namespace cvte_lidar_slam {
bool DepthCameraCalibrate::getTargetCloud(const laserCloud::Ptr &cloud,
                                          laserCloud::Ptr &target_cloud,
                                          const bool is_ground) {
  const size_t &number = cloud->size();
  if (100 > number) {
    LOG(ERROR) << "Too small points in cloud  " << number;
    return false;
  }

  target_cloud->points.reserve(200);
  for (const auto &p : cloud->points) {
    if (p.x > calib_config_.min_x && p.x < calib_config_.max_x &&
        p.y > calib_config_.min_y && p.y < calib_config_.max_y) {
      if (is_ground && p.z < 0.0) {
        target_cloud->points.push_back(p);
      } else if (!is_ground && p.z >= 0.0) {
        target_cloud->points.push_back(p);
      }
    }
  }
  LOG(INFO) << "Target cloud size: " << target_cloud->size() << std::endl;
  if (target_cloud->size() < 100) {
    return false;
  }
  return true;
}

bool DepthCameraCalibrate::segmentGround(const laserCloud::Ptr &cloud,
                                         const Eigen::Matrix4f &transform,
                                         laserCloud::Ptr &ground_cloud,
                                         laserCloud::Ptr &obstacle_cloud) {
  const size_t &number = cloud->size();
  if (1 > number) {
    LOG(ERROR) << "Too small points in cloud";
    return false;
  }
  laserCloud::Ptr target_cloud(new laserCloud());
  pcl::transformPointCloud(*cloud, *target_cloud, transform);
  ground_cloud->points.reserve(500);
  obstacle_cloud->points.reserve(1000);
  for (size_t i = 0; i < number; ++i) {
    const auto pi = cloud->points[i];
    const auto po = target_cloud->points[i];
    const double dis = sqrt(pi.x * pi.x + pi.y * pi.y + pi.z * pi.z);
    if (dis > 2.0 || po.z > 2.0) {
      continue;
    }
    const double ratio = std::max(1., dis * 0.7);

    if (std::fabs(po.z) < 0.1 * ratio) {
      ground_cloud->points.push_back(po);
    } else {
      obstacle_cloud->points.push_back(po);
    }
  }
  return true;
}
}  // namespace cvte_lidar_slam