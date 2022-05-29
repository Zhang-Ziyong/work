/*
 * @Author: your name
 * @Date: 2020-10-22 19:26:40
 * @LastEditTime: 2020-11-14 11:07:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/object_track/laser_object_track/pc_base.hpp
 */
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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace TRACKING_MOTION {
typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZI FramePosition;
typedef pcl::PointCloud<PointType> laserCloud;
typedef pcl::KdTreeFLANN<PointType> pclKdTree;
typedef pcl::VoxelGrid<PointType> pclDownsampler;
}  // namespace TRACKING_MOTION

#endif  // PC_BASE_HPP_
