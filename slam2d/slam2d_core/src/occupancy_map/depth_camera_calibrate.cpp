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
namespace slam2d_core {
namespace occupancy_map {
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

bool DepthCameraCalibrate::getAttitude(const laserCloud::Ptr &cloud,
                                       Eigen::Vector4d &plane_coefficients) {
  const size_t &number = cloud->size();
  if (50 > number) {
    LOG(ERROR) << "Too small points in target cloud";
    return false;
  }
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.015f);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (coefficients->values[3] > 0) {
    plane_coefficients << coefficients->values[0], coefficients->values[1],
        coefficients->values[2], coefficients->values[3];
  } else {
    plane_coefficients << -coefficients->values[0], -coefficients->values[1],
        -coefficients->values[2], -coefficients->values[3];
  }

  Eigen::Vector3d normal = plane_coefficients.head<3>();
  LOG(INFO) << "default " << default_plane_.transpose() << std::endl;
  Eigen::Vector3d default_normal = default_plane_.head<3>();
  Eigen::Vector3d correct_normal = Mathbox::g2Normal(normal);
  LOG(INFO) << "normal: " << correct_normal.transpose() << " "
            << -coefficients->values[3] << std::endl;
  double radian_angle = atan2(correct_normal.cross(default_normal).norm(),
                              correct_normal.transpose() * default_normal);
  double distance_diff = std::fabs(default_plane_[3] - plane_coefficients[3]);
  LOG(INFO) << "error  " << radian_angle << " " << distance_diff << std::endl;
  if (std::fabs(radian_angle) < 0.01 && distance_diff < 0.02) {
    LOG(INFO) << "Calibrated t_co: [0.0, 0.0," << plane_coefficients[3] << ','
              << correct_normal[0] << ',' << correct_normal[1] << ','
              << correct_normal[2] << ']';
    return true;
  }
  // TODO: t_cg means transform from camera to ground
  LOG(WARNING) << "Calibrated t_co: [0.0, 0.0," << plane_coefficients[3] << ','
               << correct_normal[0] << ',' << correct_normal[1] << ','
               << correct_normal[2] << ']';
  return true;
}

bool DepthCameraCalibrate::getWallAttitude(
    const laserCloud::Ptr &cloud, Eigen::Vector4d &plane_coefficients) {
  (void) plane_coefficients;
  size_t number = cloud->size();
  if (50 > number) {
    LOG(ERROR) << "Too small points in target cloud";
    return false;
  }
  std::vector<Eigen::Vector3d> v_plane_norm(2, Eigen::Vector3d(0, 0, 0));
  for (int i = 0; i < 2; ++i) {
    if (cloud->size() < number / 3) {
      LOG(ERROR) << "Too small points in seg cloud";
      return false;
    }
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    laserCloud::Ptr segmented_cloud(new laserCloud());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.015f);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  //如果设为true,可以提取指定index之外的点云
    extract.filter(*segmented_cloud);
    *cloud = *segmented_cloud;
    v_plane_norm[i] << coefficients->values[0], coefficients->values[1],
        coefficients->values[2];
  }

  Eigen::Vector3d g_estimated = v_plane_norm[0].cross(v_plane_norm[1]);
  if (g_estimated[2] < 0) {
    g_estimated = -g_estimated;
  }
  LOG(ERROR) << "Real:   " << g_estimated.transpose() << std::endl;
  Eigen::Vector3d correct_normal = Mathbox::g2Normal(g_estimated);
  Eigen::Vector3d gravity{0, 0, 1};
  double radian_angle = atan2(g_estimated.cross(gravity).norm(),
                              g_estimated.transpose() * gravity);
  LOG(INFO) << "error  " << radian_angle << std::endl;
  if (std::fabs(radian_angle) < 0.03) {
    // TODO: t_ch means transform from camera to horizon
    LOG(WARNING) << "Calibrated R_ch: [" << correct_normal[0] << ','
                 << correct_normal[1] << ',' << correct_normal[2] << ']';
    return true;
  }

  return false;
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

}  // namespace occupancy_map
}  // namespace slam2d_core