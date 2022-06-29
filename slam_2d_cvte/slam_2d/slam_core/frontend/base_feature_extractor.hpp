/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file feature_extractor.hpp
 *
 *@brief
 * 1.前端BaseFeatureExtractor基类
 *
 *@author caoyong(caoyong@cvte.com)
 *@modified zhangwei(zhangwei@cvte.com)
 *@version V1.0
 *@data 2020-02-15
 ************************************************************************/
#ifndef BASE_FEATURE_EXTRACTOR_HPP
#define BASE_FEATURE_EXTRACTOR_HPP

#include "opencv2/opencv.hpp"

#include <vector>
#include <mutex>

#include "common/data_struct/pc_base.hpp"
#include "common/config/system_config.hpp"
#include "common/math_base/slam_transform.hpp"

namespace cvte_lidar_slam {
/**
 * BaseFeatureExtractor
 * @brief 点云特征提取类
 * 1. 点云前处理
 * 2. 提取所需的特征点云
 **/
class BaseFeatureExtractor {
 public:
  BaseFeatureExtractor() = default;
  virtual ~BaseFeatureExtractor() = default;

  /**
   *setInputCloud
   *@brief
   *获取输入点云数据
   *
   *@param[in] cloud_in-输入点云
   **/
  virtual bool setInputCloud(const laserCloud &cloud_in) = 0;

  virtual bool setInputCloud(const laserCloud::Ptr ptr_cloud) = 0;

  /**
   *resetVariables
   *@brief
   *重置变量
   *
   **/
  virtual void resetVariables() = 0;

  /**
   *adjustDistortion
   *@brief
   *将点云按匀速模型插补进行畸变矫正
   *
   **/
  virtual void adjustDistortion(const Mat34d &delta_pose) = 0;

  /**
   *cloudExtractor
   *@brief
   *提取所需的特征点云
   *
   *@param[out] corner_cloud-角点
   *@param[out] surf_cloud-面点
   *@param[out] without_ground_cloud-移除地面点的点云
   **/
  virtual bool cloudExtractor(laserCloud::Ptr corner_cloud,
                              laserCloud::Ptr surf_cloud,
                              laserCloud::Ptr without_ground_cloud,
                              int &environment_flag, Vec6d &h_matrix) = 0;
  virtual bool cloudExtractor(laserCloud::Ptr corner_cloud,
                              laserCloud::Ptr surf_cloud,
                              laserCloud::Ptr corner_cloud_top,
                              laserCloud::Ptr surf_cloud_top,
                              laserCloud::Ptr raw_cloud,
                              laserCloud::Ptr without_ground_cloud) = 0;

 protected:
  std::mutex mutex_;  ///< 数据锁>

};  // end of class

}  // namespace cvte_lidar_slam

#endif  // BASE_FEATURE_EXTRACTOR_HPP