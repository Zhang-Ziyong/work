/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file tracking_node.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-21
 ************************************************************************/
#ifndef __TRACKING_NODE_HPP
#define __TRACKING_NODE_HPP
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_perception_msgs/msg/tracking_object_array.hpp"

#include "common/tracking_utils.hpp"

namespace CVTE_BABOT {
class PointCloudCluster;
class TrackingWorker;
class Classifier;

class TrackingNode {
 public:
  TrackingNode();
  ~TrackingNode() = default;

  TrackingNode(const TrackingNode &) = delete;
  TrackingNode &operator=(const TrackingNode &) = delete;

  void updateParameters();

  inline rclcpp::Node::SharedPtr getNodeHander() { return node_; }

 private:
  void fromPointCloudToVpoints(
      const sensor_msgs::msg::PointCloud::SharedPtr &ptr_point_cloud_msg,
      const VPointsPtr &ptr_vp_cloud);

  void pointCloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr);

  void computeAndPubObjects(const std::vector<ConstObjectPtr> &v_ptr_object,
                            const builtin_interfaces::msg::Time &time);

  void publishCloudMsg(const VPointsPtr &ptr_output_cloud);

  /*============================可视化=================================*/
  void computeAndPubVelMsg(const std::vector<ConstObjectPtr> &v_ptr_object);

  void computeClearingVelMarkers(
      const std::vector<int> &v_valid_id,
      visualization_msgs::msg::MarkerArray &boxset_msgs);

  void computeAndPubBoxMsg(const std::vector<ConstObjectPtr> &v_ptr_object);

  void computeClearingMarkers(
      const std::vector<int> &v_valid_id,
      visualization_msgs::msg::MarkerArray &boxset_msgs);

  void computeBoundingBoxMarker(const ConstObjectPtr &ptr_object,
                                visualization_msgs::msg::Marker &bounding_box);

  void computeTextMarker(const ConstObjectPtr &ptr_object,
                         visualization_msgs::msg::Marker &text_marker);

  void computeTrajMarker(const ConstObjectPtr &ptr_object,
                         visualization_msgs::msg::Marker &trajectory_marker);

  /*==================================================================*/

  rclcpp::Node::SharedPtr node_;  ///< 用于收发数据的节点
  rclcpp::Clock::SharedPtr ptr_clock_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr
      point_cloud_sub_;

  rclcpp::Publisher<lidar_perception_msgs::msg::TrackingObjectArray>::SharedPtr
      tracking_output_objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      static_points_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      boundmsgs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      tracking_objects_velocity_pub_;

  bool b_print_log_;

  std::string s_cloud_topic_;

  double d_dynamic_height_threshhold_ = 0.0;

  double d_threshold_contian_IoU_ = 0.0;

  std::shared_ptr<PointCloudCluster> ptr_point_cluster_;
  std::shared_ptr<TrackingWorker> ptr_tracking_worker_;
  std::shared_ptr<Classifier> ptr_classifier_;
};
}  // namespace CVTE_BABOT
#endif
