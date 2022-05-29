/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file conversion.hpp
 *
 *@brief
 * 与slam2d_core与ros2的一些数据类型转换函数
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version v1.0
 *@data 2021-04-01
 ************************************************************************/
#ifndef _CONVERSION_H_
#define _CONVERSION_H_

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <Eigen/Core>

#include "amcl/occupancy_grid.hpp"
#include "cairo/cairo.h"
#include "common/point_cloud.hpp"
#include "frontend/submap_painter.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "chassis_interfaces/msg/mapping_png.hpp"
#include <opencv2/opencv.hpp>
#include "occupancy_map/occupancy_map_common.hpp"

namespace slam2d_ros2 {

std_msgs::msg::ColorRGBA getColor(int id);
std_msgs::msg::ColorRGBA colorBar(double xbrt);

void scanToPointCloud(
    const sensor_msgs::msg::LaserScan &msg,
    const slam2d_core::common::Rigid3 &laser_tf,
    slam2d_core::common::TimedPointCloudData &time_point_cloud,
    sensor_msgs::msg::PointCloud &cloud_out);

builtin_interfaces::msg::Time toRos(const slam2d_core::common::Time &time);

slam2d_core::common::Time fromRos(const builtin_interfaces::msg::Time &time);

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 &vector3);

Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion &quaternion);

slam2d_core::common::Rigid3 toRigid3(
    const geometry_msgs::msg::TransformStamped &transform);

slam2d_core::common::Rigid3 toRigid3(const geometry_msgs::msg::Pose &pose);

geometry_msgs::msg::Pose toGeometryMsgPose(
    const slam2d_core::common::Rigid3 &rigid3);

geometry_msgs::msg::Transform toGeometryMsgTransform(
    const slam2d_core::common::Rigid3 &rigid3);

geometry_msgs::msg::Point toGeometryMsgPoint(const Eigen::Vector3d &vector3d);

std::unique_ptr<nav_msgs::msg::OccupancyGrid> createOccupancyGridMsg(
    const slam2d_core::frontend::PaintSubmapSlicesResult &painted_slices,
    const double &resolution, const std::string &frame_id,
    const builtin_interfaces::msg::Time &time);

void createOccupancyGridMsg(
    nav_msgs::msg::OccupancyGrid &occupancy_grid,
    std::shared_ptr<slam2d_core::amcl::OccupancyGrid> map_ptr,
    const builtin_interfaces::msg::Time &time);

bool saveOccupancyGridToMapfile(
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> ptr_map,
    const std::string &file_name);

void base64Encode(const std::string &input, std::string &output);
bool getPngFromOccupancyGrid(nav_msgs::msg::OccupancyGrid *ptr_map,
                             chassis_interfaces::msg::MappingPng &mapping_png,
                             const std::string &map_temp_path);
bool getPngFromDepthOccupancyGrid(
    const slam2d_core::occupancy_map::UserOccupancyGrid &globalmap,
    chassis_interfaces::msg::MappingPng &mapping_png,
    const std::string &map_temp_path);

}  // namespace slam2d_ros2

#endif
