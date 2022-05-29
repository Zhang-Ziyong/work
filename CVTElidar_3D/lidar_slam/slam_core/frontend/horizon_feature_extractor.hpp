/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file feature_extractor.hpp
 *
 *@brief
 * 1.前端HorizonFeatureExtractor类
 *
 *@author zhangwei(zhangwei@cvte.com)
 *@version V1.0
 *@data 2020-02-12
 ************************************************************************/
#ifndef HORIZON_FEATURE_EXTRACTOR_HPP
#define HORIZON_FEATURE_EXTRACTOR_HPP

#include "frontend/base_feature_extractor.hpp"

namespace cvte_lidar_slam {

class KeyFrame;

struct grid {
  grid(const int x, const int y, const int size) : x(x), y(y), size(size) {}
  int x;
  int y;
  int size;
};

/**
 * HorizonFeatureExtractor
 * @brief 点云特征提取类
 * 1. 点云前处理
 * 2. 提取所需的特征点云
 **/
class HorizonFeatureExtractor final : public BaseFeatureExtractor {
 public:
  HorizonFeatureExtractor();
  virtual ~HorizonFeatureExtractor() {}

  void setGridRange(const double x, const double y);

  void setGridResolution(const double resolution);

  /**
   *setInputCloud
   *@brief
   *获取输入点云数据
   *
   *@param[in] cloud_in-输入点云
   **/
  virtual bool setInputCloud(const laserCloud &cloud_in);

  virtual bool setInputCloud(const laserCloud::Ptr ptr_cloud);

  /**
   *removeOutlier
   *@brief
   *去除外点
   *
   **/
  void removeOutlier();

  /**
   *resetVariables
   *@brief
   *重置变量
   *
   **/
  virtual void resetVariables();

  /**
   *cloudExtractor
   *@brief
   *提取所需的特征点云
   *
   *@param[out] corner_cloud-角点
   *@param[out] surf_cloud-面点
   *@param[out] outlier_cloud-异常点
   **/

  virtual bool cloudExtractor(laserCloud::Ptr corner_cloud,
                              laserCloud::Ptr surf_cloud,
                              laserCloud::Ptr without_ground_cloud);

  bool cloudExtractor(laserCloud::Ptr surf_cloud);  ///<>

 private:
  /**
   *allocateMemory
   *@brief
   *提前分配内存
   *
   * */
  void Initialize();

  bool downsampleCloud();

  /**
   *groundRemoval
   *@brief
   *提取地面点
   *
   **/
  virtual void groundRemoval();

  /**
   *adjustDistortion
   *@brief
   *将点云进行插补等工作
   *
   **/
  virtual void adjustDistortion();
  virtual void adjustDistortion(const Mat34d &delta_pose) {}
  /**
   *extractFeatures
   *@brief
   *提取特征点
   *
   **/
  virtual void extractFeatures();

 private:
  laserCloud::Ptr laser_cloud_in_;    ///< 输入点云
  laserCloud::Ptr ds_cloud_;          ///<  projected velodyne raw cloud
  laserCloud::Ptr ground_cloud_;      ///< 地面点云
  laserCloud::Ptr not_ground_cloud_;  ///< 分割好的点云，不包括地面点
  laserCloud::Ptr surf_cloud_;        ///< 平面点
  laserCloud::Ptr corner_cloud_;
  laserCloud::Ptr outlier_cloud_;  ///< 异常点

  std::vector<std::vector<std::vector<PointType>>> vv_grid_pointts_;
  std::vector<grid> v_corner_grid_;
  const HorizonFeatureConfig config_;

  laserCloud::Ptr corner_points_;  ///< 角点
  laserCloud::Ptr surf_points_;    ///< 面点

  pcl::VoxelGrid<PointType> downsize_filter_;  ///< 下采样滤波器
  pclKdTree::Ptr ptr_kdtree_surf_points_;      ///<>

};  // end of class

}  // namespace cvte_lidar_slam

#endif