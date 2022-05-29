/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file costmap_visualizer_ros2.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-08-10
 ************************************************************************/
#ifndef __COSTMAP_VISUALIZER_ROS2_HPP
#define __COSTMAP_VISUALIZER_ROS2_HPP
#include <memory>
#include <string>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "costmap_utils.hpp"
namespace CVTE_BABOT {
class Costmap2d;
class CostmapCloud;

class CostmapVisualizerRos2 {
 public:
  CostmapVisualizerRos2(const rclcpp::Node::SharedPtr &ptr_node);
  ~CostmapVisualizerRos2() = default;

  CostmapVisualizerRos2(const CostmapVisualizerRos2 &) = delete;
  CostmapVisualizerRos2 &operator=(const CostmapVisualizerRos2 &) = delete;

  /**
   *systemInit
   *@brief
   *架构相关通讯的初始化
   *
   *@param[in] none
   *@param[out] bool类型，true初始化成功，false为失败
   * */
  void systemInit();

  /**
   * updateParameter
   * @brief
   * 用于从传入的yaml文件中更新base_parameter的参数
   *
   * @param[in] none
   * @param[out] none
   * */
  void updateParameter();

  void visVoxelGrid(const std::shared_ptr<const CostmapCloud> &);

  void visClearingPoints(const std::shared_ptr<const CostmapCloud> &);

  /**
   *visCostmap
   *@brief
   *将costmap转换成栅格地图.
   *
   * */
  void visCostmap(const std::shared_ptr<const Costmap2d> &ptr_costmap);

  /**
   * visFootprint
   * @brief
   * 发布避障框的位置
   *
   * @param[in] robot_current_pose-机器人当前的位置
   * */
  void visFootprint(const WorldmapPose &current_pose,
                    const std::vector<WorldmapPoint> &v_padded_footprint);

 private:
  std::array<char, 256> cost_translation_table_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      ptr_footprint_pub_ = nullptr;  //发布footprint

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_ =
      nullptr;  ///< 发布代价地图

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      voxel_grid_pub_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr
      clearing_cloud_pub_ = nullptr;  ///< 发布3d点云

  rclcpp::Node::SharedPtr node_ = nullptr;  ///< 用于收发数据的节点
  rclcpp::Clock::SharedPtr ptr_clock_ = nullptr;  ///< 时钟指针
};
}  // namespace CVTE_BABOT
#endif
