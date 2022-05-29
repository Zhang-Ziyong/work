/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file recon_cluster.hpp
 *
 *@brief 该段代码定义了类“Point_Cloud_Cluster”的声明，用来完成点云的聚类。
 * 类“Point_Cloud_Cluster”包含两个成员，ptr_point_cloud_保存待处理的点云数据，vi_label_index_保存聚类的结果。
 * 类的构造函数仅赋予ptr_point_cloud_空指针。
 * 定义了三个功能函数：setInputCloud对ptr_point_cloud_赋值，
 *                     recursiveDiffusion构造遍历数据的递归，
 *                     clusteringFunction实现聚类功能.
 *
 *@author Mingzi Tao(i_taoziming@cvte.com)
 *@modified caoyong (caoyong@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-09-04
 ************************************************************************/
#ifndef RECON_CLUSTER_HPP_
#define RECON_CLUSTER_HPP_
#include <map>

#include <pcl/common/transforms.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ground_utils.hpp"

#include <pcl/features/normal_3d.h>

namespace CVTE_BABOT {
struct ClusterParams {
  double radius = 0.15;
  double increment_distance_radius = 0.15;
  double clusters_radius = 0.25;
  bool use_multi_radius = true;
  bool use_last_cluters = true;
  int min_cluster_size = 8;
};

class PointCloudCluster {
 public:
  PointCloudCluster(const rclcpp::Clock::SharedPtr& ptr_clock);

  ~PointCloudCluster() = default;

  PointCloudCluster(const PointCloudCluster& PointCloudCluster) = delete;
  PointCloudCluster& operator=(const PointCloudCluster& PointCloudCluster) =
      delete;

  /**
   * @brief Set the Input Point Cloud object
   *
   * @param ptr_input_pointcloud
   */
  void setInputCloud(const pcl::PointCloud<VPoint>::Ptr& ptr_input_pointcloud,
                     const pcl::PointCloud<VPoint>::Ptr& ptr_laser_pointcloud);

  /**
   * @brief 构造遍历数据的递归
   *
   * @param kdtree 待聚类点云的数据集
   * @param center_point 待搜索的点
   * @param label_name 标签名
   * @param radius 邻域的搜索半径
   */
  void recursiveDiffusion(const pcl::KdTreeFLANN<VPoint>& kdtree,
                          const VPoint& center_point, const int& label_name,
                          const float& radius);

  /**
   * @brief 用于聚类的函数，基本原理基于欧式聚类
   *
   * @param Output_fitting_points
   * 根据vi_label_index_对原点云进行聚类，其结果的点云集作为输出
   * @param radius 聚类准则的半径
   */
  void clusteringFunction(
      std::vector<pcl::PointCloud<VPoint>>& output_fitting_points,
      std::vector<int>& v_index_output);

  void updateParams(const ClusterParams& cp);

  /**
   * @brief 以矩形拟合聚类结果集中同类点云的边界
   *
   * @param fitting_points 聚类结果的点云集
   * @param BoxSetmsgs
   * ROS下的消息类型，可直接发送至rviz可视化，其type为BoundingBoxArray
   */
  void FindingBox(const std::vector<pcl::PointCloud<VPoint>>& fitting_points,
                  std::vector<int>& v_index,
                  visualization_msgs::msg::MarkerArray& boxset_msgs,
                  geometry_msgs::msg::PoseArray& poses_msg);

 private:
  /**
   * @brief 待聚类的点云目标。
   */
  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud_ = nullptr;
  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud_2d_ = nullptr;
  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud_laser_ = nullptr;
  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud_and_laser_ = nullptr;

  /**
   * @brief
   *  * 根据输入的点云记录分类的结果。
   *  * 以vector的形式顺序记录点云序列的标签号。
   */
  std::vector<int> vi_label_index_;

  std::vector<int> vi_cluster_size_;

  int label_name_;

  std::vector<int> v_index_;

  std::vector<pcl::PointCloud<VPoint>> v_cluster_cloud_;

  ClusterParams cluster_params_;

  pcl::PointCloud<pcl::PointNormal>::Ptr ptr_cloud_with_normals_ =
      pcl::PointCloud<pcl::PointNormal>::Ptr(
          new pcl::PointCloud<pcl::PointNormal>);

  pcl::KdTreeFLANN<VPoint> kdtree_2d_with_laser_;
  pcl::KdTreeFLANN<VPoint> kdtree_2d_;

  std::vector<bool> vb_if_new_cluster_;
  std::map<int, std::vector<geometry_msgs::msg::Point>> mv_previou_poses_;

  rclcpp::Clock::SharedPtr ptr_clock_;
};

typedef std::shared_ptr<PointCloudCluster> PointCloudClusterPtr;

}  // namespace CVTE_BABOT
#endif
