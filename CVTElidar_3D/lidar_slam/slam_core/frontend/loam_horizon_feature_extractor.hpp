#ifndef LOAM_HORIZON_FEATURE_EXTRACTOR_HPP
#define LOAM_HORIZON_FEATURE_EXTRACTOR_HPP

#include "frontend/base_feature_extractor.hpp"

namespace cvte_lidar_slam {
class KeyFrame;

struct smoothness_ind {
  float value;
  size_t ind;
};

struct com_value {
  bool operator()(smoothness_ind const &left, smoothness_ind const &right) {
    return left.value < right.value;
  }
};

class LoamHorizonFeatureExtractor : public BaseFeatureExtractor {
 public:
  LoamHorizonFeatureExtractor();
  LoamHorizonFeatureExtractor(const LoamHorizonFeatureExtractor &obj) = delete;
  const LoamHorizonFeatureExtractor &operator=(
      const LoamHorizonFeatureExtractor &obj) = delete;
  virtual ~LoamHorizonFeatureExtractor(){};

  /**
   *setInputCloud
   *@brief
   *获取输入点云数据
   *
   *@param[in] cloud_in-输入点云
   **/
  virtual bool setInputCloud(const laserCloud &cloud_in);

  virtual bool setInputCloud(const laserCloud::Ptr ptr_cloud);

  template <typename PointT>
  void removeClosedTooFarPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                    pcl::PointCloud<PointT> &cloud_out,
                                    float thres1, float thres2);

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

 private:
  /**
   *allocateMemory
   *@brief
   *提前分配内存
   *
   * */
  virtual void allocateMemory();

  /**
   *groundRemoval
   *@brief
   *提取地面点
   *
   **/
  virtual void groundRemoval() {}

  /**
   *adjustDistortion
   *@brief
   *将点云进行插补等工作
   *
   **/
  virtual void adjustDistortion() {}
  virtual void adjustDistortion(const Mat34d &delta_pose) {}
  /**
   *extractFeatures
   *@brief
   *提取特征点
   *
   **/
  virtual void extractFeatures();

 private:
  pcl::VoxelGrid<PointType> downSize_filter_;  ///< 下采样滤波器

  laserCloud::Ptr surf_points_scan_;
  laserCloud::Ptr surf_points_scan_DS_;
  laserCloud::Ptr outlier_points_scan_;
  laserCloud::Ptr outlier_points_scan_DS_;
  laserCloud::Ptr without_ground_cloud_;  ///< 移除地面点的点云
  laserCloud::Ptr outlier_cloud_;
  laserCloud::Ptr corner_cloud_;
  laserCloud::Ptr surf_cloud_;
  laserCloud::Ptr full_cloud_;
  laserCloud::Ptr laser_cloud_in_;

  std::vector<laserCloud> laser_scan_;
  std::vector<std::vector<int>> cloudNeighborPicked_;
  std::vector<std::vector<double>> cloud_curvature_;
  std::vector<std::vector<int>> cloud_label_;
  std::vector<std::vector<int>> cloud_sort_ind_;
  std::vector<std::vector<int>> points_reflectivity_;
  std::vector<std::vector<smoothness_ind>> scan_smooth_vec_;
};
}  // namespace cvte_lidar_slam
#endif