/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file costmap_visualizer_ros2.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-08-10
 ************************************************************************/
#include "visualizer_ros2/costmap_visualizer_ros2.hpp"

#include "costmap_2d.hpp"
#include "costmap_cloud.hpp"
#include "glog/logging.h"
namespace CVTE_BABOT {
CostmapVisualizerRos2::CostmapVisualizerRos2(
    const rclcpp::Node::SharedPtr &ptr_node)
    : node_(ptr_node) {
  cost_translation_table_[0] = 0;      // NO obstacle
  cost_translation_table_[253] = 99;   // INSCRIBED obstacle
  cost_translation_table_[254] = 100;  // LETHAL obstacle
  cost_translation_table_[255] = -1;   // UNKNOWN
  for (int i = 1; i < 253; i++) {
    cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
  }
}

void CostmapVisualizerRos2::updateParameter() {}

void CostmapVisualizerRos2::systemInit() {
  ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  if (nullptr == ptr_footprint_pub_) {
    ptr_footprint_pub_ =
        node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "footprint", rclcpp::QoS(10).best_effort());
  }

  bool b_publish_costmap = true;
  node_->get_parameter_or("publish_costmap", b_publish_costmap, true);
  if (b_publish_costmap && nullptr == costmap_pub_) {
    costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "costmap", rclcpp::QoS(10).best_effort());
  }

  bool b_publish_voxel = false;
  node_->get_parameter_or("publish_voxel", b_publish_voxel, false);
  if (b_publish_voxel && nullptr == voxel_grid_pub_) {
    voxel_grid_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "voxel_grid", rclcpp::QoS(10).best_effort());
  }

  bool b_publish_clearing_points = false;
  node_->get_parameter_or("publish_clearing_points", b_publish_clearing_points,
                          false);
  if (b_publish_clearing_points && nullptr == clearing_cloud_pub_) {
    clearing_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>(
        "clearing_points", rclcpp::QoS(10).best_effort());
  }
}

void CostmapVisualizerRos2::visVoxelGrid(
    const std::shared_ptr<const CostmapCloud> &ptr_cloud) {
  visualization_msgs::msg::Marker voxel_grid;
  voxel_grid.header.frame_id = "map";
  voxel_grid.header.stamp = ptr_clock_->now();
  voxel_grid.ns = "voxel_grid";
  voxel_grid.type = visualization_msgs::msg::Marker::CUBE_LIST;
  voxel_grid.id = 0;

  // clear all voxel_grid first.
  voxel_grid.action = visualization_msgs::msg::Marker::DELETEALL;
  voxel_grid_pub_->publish(voxel_grid);

  voxel_grid.action = visualization_msgs::msg::Marker::ADD;

  voxel_grid.pose.orientation.x = 0.0;
  voxel_grid.pose.orientation.y = 0.0;
  voxel_grid.pose.orientation.z = 0.0;
  voxel_grid.pose.orientation.w = 1.0;
  voxel_grid.color.a = 0.5;
  voxel_grid.color.r = 1.0;
  voxel_grid.color.g = 0.0;
  voxel_grid.color.b = 0.0;

  voxel_grid.scale.x = 0.1;
  voxel_grid.scale.y = 0.1;
  voxel_grid.scale.z = 0.1;

  geometry_msgs::msg::Point pt;
  for (const auto &coord : *ptr_cloud->ptr_v_cloud_) {
    pt.x = coord.d_x;
    pt.y = coord.d_y;
    pt.z = coord.d_z;
    voxel_grid.points.push_back(pt);
  }
  if (voxel_grid_pub_) {
    try {
      voxel_grid_pub_->publish(voxel_grid);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what();
    }
  }
}

void CostmapVisualizerRos2::visClearingPoints(
    const std::shared_ptr<const CostmapCloud> &ptr_cloud) {
  sensor_msgs::msg::PointCloud clearing_cloud;
  for (const auto &cloud_point : *ptr_cloud->ptr_v_cloud_) {
    geometry_msgs::msg::Point32 point;
    point.x = cloud_point.d_x;
    point.y = cloud_point.d_y;
    point.z = cloud_point.d_z;
    clearing_cloud.points.push_back(point);
  }

  clearing_cloud.header.frame_id = "map";
  clearing_cloud.header.stamp = ptr_clock_->now();
  if (clearing_cloud_pub_ != nullptr) {
    try {
      clearing_cloud_pub_->publish(clearing_cloud);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what();
    }
  }
}

void CostmapVisualizerRos2::visCostmap(
    const std::shared_ptr<const Costmap2d> &ptr_costmap) {
  std::unique_lock<std::recursive_mutex> lock(*(ptr_costmap->getMutx()));
  static nav_msgs::msg::OccupancyGrid grid_map;  ///< 当前转换后的栅格地图
  grid_map.header.frame_id = "map";
  grid_map.header.stamp = ptr_clock_->now();

  double d_resolution = ptr_costmap->getResolution();
  grid_map.info.resolution = d_resolution;
  grid_map.info.width = ptr_costmap->getSizeInCellsX();
  grid_map.info.height = ptr_costmap->getSizeInCellsY();

  CostmapPoint cm_point;
  cm_point.ui_x = 0;
  cm_point.ui_y = 0;
  WorldmapPoint wc_point;
  ptr_costmap->mapToWorld(cm_point, wc_point);
  grid_map.info.origin.position.x = wc_point.d_x - d_resolution / 2;
  grid_map.info.origin.position.y = wc_point.d_y - d_resolution / 2;
  grid_map.info.origin.position.z = 0.0;
  grid_map.info.origin.orientation.w = 1.0;

  grid_map.data.resize(grid_map.info.width * grid_map.info.height);
  auto ptr_data = ptr_costmap->getCharMap();
  for (size_t i = 0; i < grid_map.data.size(); i++) {
    grid_map.data[i] = cost_translation_table_[ptr_data[i]];
  }
  if (costmap_pub_ != nullptr) {
    try {
      costmap_pub_->publish(grid_map);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what();
    }
  }
}

void CostmapVisualizerRos2::visFootprint(
    const WorldmapPose &current_pose,
    const std::vector<WorldmapPoint> &v_padded_footprint) {
  double d_cos_th = cos(current_pose.d_yaw);
  double d_sin_th = sin(current_pose.d_yaw);

  static geometry_msgs::msg::PolygonStamped footprint;
  footprint.header.frame_id = "map";
  footprint.header.stamp = ptr_clock_->now();
  footprint.polygon.points.reserve(v_padded_footprint.size());
  static geometry_msgs::msg::Point32 new_pt;
  for (const auto &point : v_padded_footprint) {
    new_pt.x = (point.d_x * d_cos_th - point.d_y * d_sin_th) + current_pose.d_x;
    new_pt.y = (point.d_x * d_sin_th + point.d_y * d_cos_th) + current_pose.d_y;
    footprint.polygon.points.push_back(new_pt);
  }
  if (ptr_footprint_pub_ != nullptr) {
    try {
      ptr_footprint_pub_->publish(footprint);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what();
    }
  }

  // clear the data
  footprint.polygon.points.clear();
}

}  // namespace CVTE_BABOT
