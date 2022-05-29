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


namespace cvte_lidar_slam{
    
typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZI FramePosition;
typedef pcl::PointCloud<PointType> laserCloud;
typedef pcl::KdTreeFLANN<PointType> pclKdTree;
typedef pcl::VoxelGrid<PointType> pclDownsampler;


}  // namespace cvte_lidar_slam

#endif  // PC_BASE_HPP_
