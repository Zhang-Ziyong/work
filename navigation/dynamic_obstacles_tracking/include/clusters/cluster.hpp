/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file cluster.hpp
 *
 *@brief 实现聚类功能
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-21
 ************************************************************************/
#ifndef __CLUSTER_HPP
#define __CLUSTER_HPP
#include <pcl/kdtree/kdtree_flann.h>
#include <memory>

#include "common/tracking_utils.hpp"

namespace CVTE_BABOT {
struct ClusterParams {
  double cluster_radius = 0.15;
  double increment_radius = 0.15;
  uint16_t min_cluster_size = 8;
};

/**
 *PointCloudCluster
 *@brief
 *实现聚类功能
 *
**/
class PointCloudCluster {
 public:
  PointCloudCluster() = default;

  ~PointCloudCluster() = default;

  PointCloudCluster(const PointCloudCluster& PointCloudCluster) = delete;
  PointCloudCluster& operator=(const PointCloudCluster& PointCloudCluster) =
      delete;

  /**
   *updateParams
   *@brief
   *设置参数
   *
   *@param[in] cp-参数
  **/
  void updateParams(const ClusterParams& cp);

  /**
   *setInputCloud
   *@brief
   *输入需要聚类的点云指针
   *
   *@param[in] ptr_input_pointcloud-需要聚类的点云指针
  **/
  void setInputCloud(const pcl::PointCloud<VPoint>::Ptr& ptr_input_pointcloud);

  /**
   *computeClustersPointCloud
   *@brief
   *对当前点云聚类
   *
   *@return 聚类结果
  **/
  std::vector<pcl::PointCloud<VPoint>::Ptr> computeClustersPointCloud();

 private:
  /**
   * recursiveClustering
   * @brief 递归搜索, 聚类点云
   *
   * @param[in] kdtree-存储要聚类点云的kdtree
   * @param[in] label_name-这一次得到的类标签名
   * @param[in] search_point-搜索的起始点
   * @param[in] radius-邻域的搜索半径
   */
  void recursiveClustering(const pcl::KdTreeFLANN<VPoint2D>& kdtree2d,
                           const int& label_name, const VPoint2D& search_point,
                           const float& radius,
                           std::vector<int>& vi_label_index,
                           std::vector<int>& vi_cluster_size);

  /**
   *extractSuitedCluster
   *@brief
   *从聚类结果提取出符合条件的类
   *
   *@param[in] in-
   *@param[out] out-
   *@return true-, false-
  **/
  void extractSuitedCluster(
      const std::vector<std::vector<int>>& vvi_cluster_size,
      std::vector<pcl::PointCloud<VPoint>::Ptr>& v_ptr_cluster_points);

  ClusterParams cluster_params_;

  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud_ = nullptr;
  pcl::PointCloud<VPoint2D>::Ptr ptr_point_cloud_2d_ = nullptr;
};

}  // namespace CVTE_BABOT
#endif
