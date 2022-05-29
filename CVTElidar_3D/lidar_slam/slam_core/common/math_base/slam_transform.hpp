/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file slam_transform.hpp
 *
 *@brief
 * 点云坐标变换类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/
#ifndef SLAM_TRANSFORM_HPP_
#define SLAM_TRANSFORM_HPP_

#include "slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace cvte_lidar_slam {

/**
 * Transformbox
 * @brief 点云坐标变换类
 *
 **/
class Transformbox {
 public:
  /**
   *pointToStart
   *@brief
   *将点变换到起始坐标系
   *
   *@param[in] pi-输入点
   *@param[in] pose-变换pose
   *@param[out] po-输出点
   **/
  // static void pointToStart(PointType const *const pi, PointType *const po,
  //                          const Mat34d &pose) {
  //   // float s = 10 * (pi->intensity - int(pi->intensity));
  //   po->intensity = pi->intensity;
  // }

  /**
   *pointToEnd
   *@brief
   *将点变换到结束坐标系
   *
   *@param[in] pi-输入点
   *@param[in] pose-变换pose
   *@param[out] po-输出点
   **/
  // static void pointToEnd(PointType const *const pi, PointType *const po,
  //                        const Mat34d &pose) {
  //   // float s = 10 * (pi->intensity - int(pi->intensity));
  //   po->intensity = int(pi->intensity);
  // }

  /**
   *pointAssociateToMap
   *@brief
   *将点变换到地图坐标系
   *
   *@param[in] pi-输入点
   *@param[in] pose-变换pose
   *@param[out] po-输出点
   **/
  static void pointAssociateToMap(PointType const *const pi,
                                  PointType *const po, const Mat34d &pose) {
    Vec3d point_curr(pi->x, pi->y, pi->z);
    Vec3d point_w = Mathbox::multiplePoint(pose, point_curr);
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
  }

  /**
   *transformPointCloud
   *@brief
   *将点云变换到地图坐标系
   *
   *@param[in] cloudIn-输入点云
   *@param[in] pose-变换pose
   *@return laserCloud::Ptr
   **/
  static laserCloud::Ptr transformPointCloud(const Mat34d &pose,
                                             laserCloud::Ptr cloudIn) {
    laserCloud::Ptr cloudOut(new laserCloud());
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    for (int i = 0; i < cloudSize; ++i) {
      pointAssociateToMap(&cloudIn->points[i], &pointTo, pose);
      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

};  // end of class

}  // namespace cvte_lidar_slam

#endif  // SLAM_TRANSFORM_HPP_