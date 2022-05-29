/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file costmap_cloud.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-01
 ************************************************************************/
#include "costmap_cloud.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <glog/logging.h>
namespace CVTE_BABOT {
std::shared_ptr<CostmapCloud> CostmapCloud::transformToMap() {
  if (!b_need_transformed_) {
    return std::make_shared<CostmapCloud>(*this);
  }

  auto ptr_map_cloud = std::make_shared<CostmapCloud>();

  ptr_map_cloud->s_topic_name_ = s_topic_name_ + "_map";
  ptr_map_cloud->origin_ = origin_;

  ptr_map_cloud->obstacle_range_ = obstacle_range_;
  ptr_map_cloud->raytrace_range_ = raytrace_range_;
  ptr_map_cloud->v_fov_ = v_fov_;
  ptr_map_cloud->h_fov_ = h_fov_;
  ptr_map_cloud->min_d_ = min_d_;
  ptr_map_cloud->max_d_ = max_d_;
  ptr_map_cloud->negative_min_d_ = negative_min_d_;
  ptr_map_cloud->negative_max_d_ = negative_max_d_;
  ptr_map_cloud->min_h_ = min_h_;
  ptr_map_cloud->max_h_ = max_h_;
  ptr_map_cloud->sensor_pose_ = sensor_pose_;
  ptr_map_cloud->world_pose_ = world_pose_;
  double wsyaw = std::sin(world_pose_.d_yaw);
  double wcyaw = std::cos(world_pose_.d_yaw);
  Eigen::Matrix4f T_wo;
  T_wo << wcyaw, -wsyaw, 0, world_pose_.d_x, wsyaw, wcyaw, 0, world_pose_.d_y,
      0, 0, 1, 0, 0, 0, 0, 1;

  // 计算传感器相对于世界坐标系(地图坐标系)的坐标转换
  Eigen::Matrix4f T_os = Eigen::Matrix4f::Identity();
  T_os.block<3,3>(0, 0) = Eigen::Matrix3f(
    Eigen::AngleAxisf(sensor_pose_.yaw, Eigen::Vector3f::UnitZ()) * 
    Eigen::AngleAxisf(sensor_pose_.pitch, Eigen::Vector3f::UnitY()) * 
    Eigen::AngleAxisf(sensor_pose_.roll, Eigen::Vector3f::UnitX()));
  T_os(0, 3) = sensor_pose_.x;
  T_os(1, 3) = sensor_pose_.y;
  T_os(2, 3) = sensor_pose_.z;

  Eigen::Matrix4f T_ws = T_wo * T_os;

  ptr_map_cloud->ptr_v_cloud_->reserve(ptr_v_cloud_->size());
  for (const auto &sensor_point : *ptr_v_cloud_) {
    Eigen::Vector4f scp;
    scp << sensor_point.d_x, sensor_point.d_y, sensor_point.d_z, 1;
    Eigen::Vector4f wcp = T_ws * scp;
    CostmapPointXYZ world_point = {wcp[0], wcp[1], wcp[2]};

    ptr_map_cloud->ptr_v_cloud_->push_back(world_point);
  }
  ptr_map_cloud->b_need_transformed_ = false;
  return ptr_map_cloud;
}

std::shared_ptr<CostmapCloud> CostmapCloud::transformToBaseLink() {
  auto ptr_base_link_cloud = std::make_shared<CostmapCloud>();

  ptr_base_link_cloud->s_topic_name_ = s_topic_name_ + "_map";
  ptr_base_link_cloud->origin_ = origin_;

  ptr_base_link_cloud->obstacle_range_ = obstacle_range_;
  ptr_base_link_cloud->raytrace_range_ = raytrace_range_;

  double sin_yaw = sin(sensor_pose_.yaw);
  double cos_yaw = cos(sensor_pose_.yaw);

  ptr_base_link_cloud->ptr_v_cloud_->reserve(ptr_v_cloud_->size());
  for (const auto &sensor_point : *ptr_v_cloud_) {
    CostmapPointXYZ world_point = {
        cos_yaw * sensor_point.d_x - sin_yaw * sensor_point.d_y +
            sensor_pose_.x,
        sin_yaw * sensor_point.d_x + cos_yaw * sensor_point.d_y +
            sensor_pose_.y,
        sensor_point.d_z};

    ptr_base_link_cloud->ptr_v_cloud_->push_back(world_point);
  }

  return ptr_base_link_cloud;
}
}  // namespace CVTE_BABOT
