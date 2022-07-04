/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file pc_base.hpp
 *
 *@brief
 * 点云处理基础头文件
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/
#ifndef PC_BASE_HPP_
#define PC_BASE_HPP_

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/registration/transformation_estimation_2D.h>

namespace EIOLIDAR_SLAM {
// struct PointXYZIRT
// {
//   PCL_ADD_POINT4D;
//   float intensity;
//   uint16_t ring = 0;
//   double timestamp = 0;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)(float, y,
// y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double,
// timestamp, timestamp))

struct RsPointXYZIRT {
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZINormal PointType;

typedef pcl::PointXYZINormal FramePosition;
typedef pcl::PointCloud<PointType> laserCloud;
typedef pcl::KdTreeFLANN<PointType> pclKdTree;
typedef pcl::VoxelGrid<PointType> pclDownsampler;

typedef pcl::PointXYZ depthPointType;
typedef pcl::PointCloud<depthPointType> depthCloud;

}  // namespace EIOLIDAR_SLAM

POINT_CLOUD_REGISTER_POINT_STRUCT(
    EIOLIDAR_SLAM::RsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint16_t, ring, ring)(double, timestamp, timestamp))

#endif  // PC_BASE_HPP_
