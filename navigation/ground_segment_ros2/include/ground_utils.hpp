
/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file ground_utils.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-09-12
 ************************************************************************/
#ifndef GROUND_UTILS_HPP_
#define GROUND_UTILS_HPP_
#define PCL_NO_PRECOMPILE

#include <math.h>
#include <iostream>

#include <Eigen/Dense>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#define M_PI 3.14159265358979323846

namespace livox_pointcloud {
struct PointXYZIRD {               // 定义点类型结构
  PCL_ADD_POINT4D;                 // 该点类型有4个元素xyz
  float intensity;                 // Laser intensity
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保new操作符对齐操作
} EIGEN_ALIGN16;                   // 强制SSE对齐
}  // namespace livox_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(  // 注册点类型宏
    livox_pointcloud::PointXYZIRD,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

// #define VPoint livox_pointcloud::PointXYZIRD
typedef pcl::PointXYZI VPoint;

typedef pcl::PointCloud<VPoint>::Ptr VPointsPtr;

#endif
