/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file feature_extractor.hpp
 *
 *@brief
 * 1.前端FeatureExtractor类
 *
 *@author caoyong(caoyong@cvte.com)
 *@version V1.0
 *@data 2019-11-06
 ************************************************************************/
#ifndef FEATURE_EXTRACTOR_HPP
#define FEATURE_EXTRACTOR_HPP

#include "frontend/base_feature_extractor.hpp"

namespace cvte_lidar_slam {

class KeyFrame;

/**
 * CloudInfo
 * @brief 存储点云信息
 *
 **/
struct CloudInfo {
  std::vector<int> start_ring_index;
  std::vector<int> end_ring_index;

  float start_orientation;
  float end_orientation;
  float orientation_diff;

  std::vector<bool> segmented_cloud_ground_flag;  ///< true - ground point,
                                                  ///< false - other points
  std::vector<unsigned int>
      segmented_cloud_col_ind;  ///< point column index in range image
  std::vector<float> segmented_cloud_range;  ///< point range
};

struct smoothness_t {
  float value;
  size_t ind;
};

struct by_value {
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

/**
 * FeatureExtractor
 * @brief 点云特征提取类
 * 1. 点云前处理
 * 2. 提取所需的特征点云
 **/
class FeatureExtractor final : public BaseFeatureExtractor {
 public:
  FeatureExtractor();
  virtual ~FeatureExtractor() {
    delete[] all_pushed_ind_x_;
    delete[] all_pushed_ind_y_;
    delete[] queue_ind_x_;
    delete[] queue_ind_y_;
  }

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
   *@param[out] without_ground_cloud-移除地面点的点云
   **/
  bool cloudExtractor(laserCloud::Ptr corner_cloud, laserCloud::Ptr surf_cloud,
                      laserCloud::Ptr without_ground_cloud);

  /**
   *fullInfoExtractor
   *@brief
   *提取fullInfoCloud
   *
   *@param[out] full_info_cloud-带range信息的所有点云
   **/
  bool fullInfoExtractor(laserCloud::Ptr full_info_cloud);

  /**
   *adjustDistortion
   *@brief
   *将点云进行插补等工作
   *
   **/
  virtual void adjustDistortion();
  virtual void adjustDistortion(const Mat34d &delta_pose);

  /**
   *groudRemoveExtractor
   *@brief
   *提取没有地面的点云
   *
   *@param[out] without_ground_cloud-没有地面的点云
   **/
  bool groudRemoveExtractor(laserCloud::Ptr without_ground_cloud);

 private:
  /**
   *allocateMemory
   *@brief
   *提前分配内存
   *
   * */
  virtual void allocateMemory();

  /**
   *findStartEndAngle
   *@brief
   *计算点云开始和结束的角度
   *
   **/
  void findStartEndAngle();

  /**
   *projectPointCloud
   *@brief
   *将激光雷达得到的数据投影成一个16x1800的点云矩阵
   *
   **/
  void projectPointCloud();

  /**
   *groundRemoval
   *@brief
   *提取地面点
   *
   **/
  virtual void groundRemoval();

  /**
   *labelComponents
   *@brief
   *判断点云的标签
   *
   *@param[in] row-点所在行
   *@param[in] col-点所在列
   **/
  void labelComponents(int row, int col);

  /**
   *cloudSegmentation
   *@brief
   *点云聚类分割
   *
   **/
  void cloudSegmentation();

  /**
   *calculateSmoothness
   *@brief
   *计算光滑性
   *
   **/
  void calculateSmoothness();

  /**
   *markOccludedPoints
   *@brief
   *标记遮挡点
   *
   **/
  void markOccludedPoints();

  /**
   *extractFeatures
   *@brief
   *提取特征点
   *
   **/
  virtual void extractFeatures();

 private:
  laserCloud::Ptr laser_cloud_in_;   ///< 输入点云
  laserCloud::Ptr full_cloud_;       ///<  projected velodyne raw cloud
  laserCloud::Ptr full_info_cloud_;  ///< same as fullCloud, but with range
  laserCloud::Ptr ground_cloud_;     ///< 地面点云
  laserCloud::Ptr segmented_cloud_pure_;  ///< 分割好的点云，不包括地面点
  laserCloud::Ptr segmented_cloud_;  ///< 分割好的点云，包括地面点
  laserCloud::Ptr outlier_cloud_;    ///< 异常点
  laserCloud::Ptr without_ground_cloud_;  ///< 移除地面点的点云

  int label_count_;  ///< 点云标签计数

  PointType nan_point_;  ///< fill in fullCloud at each iteration

  CloudInfo seg_info_;  ///< 用于分割的信息

  FeatureConfig config_;  ///< 参数信息

  cv::Mat range_mat_;   ///< range matrix for range image
  cv::Mat ground_mat_;  ///< ground matrix for ground cloud marking
  cv::Mat label_mat_;   ///< label matrix for segmentaiton marking

  std::vector<std::pair<uint8_t, uint8_t>>
      neighbor_iterator_;  ///< neighbor iterator for segmentaiton process

  uint16_t
      *all_pushed_ind_x_;  ///< array for tracking points of a segmented object
  uint16_t
      *all_pushed_ind_y_;  ///< array for tracking points of a segmented object

  uint16_t *
      queue_ind_x_;  ///< array for breadth-first search process of segmentation
  uint16_t *
      queue_ind_y_;  ///< array for breadth-first search process of segmentation

  std::vector<float> cloud_curvature_;      ///< 曲率
  std::vector<int> cloud_neighbor_picked_;  ///< 选择的邻域
  std::vector<int> cloud_label_;
  std::vector<smoothness_t> cloud_smoothness_;

  laserCloud::Ptr corner_points_;  ///< 角点
  laserCloud::Ptr surf_points_;    ///< 面点

  laserCloud::Ptr surf_points_scan_;
  laserCloud::Ptr surf_points_scan_DS_;

  pcl::VoxelGrid<PointType> downSize_filter_;  ///< 下采样滤波器

};  // end of class

}  // namespace cvte_lidar_slam

#endif