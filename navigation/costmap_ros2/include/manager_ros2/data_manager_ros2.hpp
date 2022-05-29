/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file data_manager_ros2.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-08-24
 ************************************************************************/
#ifndef __DATA_MANAGER_ROS2_HPP
#define __DATA_MANAGER_ROS2_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/costmap_params_ros2.hpp"
#include "costmap_mediator.hpp"
#include "costmap_params.hpp"
#include "lidar_perception_msgs/msg/tracking_object_array.hpp"
#include "processor_utils.hpp"
#include "eigen3/Eigen/Core"
#include <geometry_msgs/msg/twist.hpp>
#include "common/data_filter.hpp"

// #include "cvte_sensor_msgs/msg/sensor_switch.hpp"
namespace CVTE_BABOT {
class DataManagerRos2 {
 public:
  DataManagerRos2(
      const rclcpp::Node::SharedPtr &ptr_node,
      const std::shared_ptr<CostmapParamsRos2> &costmap_params_ros2);
  ~DataManagerRos2() = default;

  /**
   * @brief 系统初始化 构建中间件数据更新
   *
   * @return true
   * @return false
   */
  bool systemInit();

  /**
   * @brief 获取配置参数
   *
   */
  void updateParameter();

  /**
   * @brief 开始 costmap 数据更新(订阅话题)
   *
   */
  void startProcessorTimer();

  /**
   * @brief 关闭costmap 停止数据更新(reset 话题)
   *
   */
  void stopProcessorTimer();

  void startRangeIntput(std::string source);  // 启动某个topic
  void stopRangeIntput(std::string source);   // 关闭某各topic

  void startDirRangeIntput(int dir);
  void stopDirRangeIntput(int dir);

 private:
  DataManagerRos2(const DataManagerRos2 &) = delete;
  DataManagerRos2 &operator=(const DataManagerRos2 &) = delete;

  /**
   * @brief 测距红外回调
   *
   * @param range_msg
   * @param topic_name 话题名称
   */
  void irSensorCallback(const sensor_msgs::msg::Range::SharedPtr range_msg,
                        const std::string &topic_name);

  /**
   * @brief 距离传感器回调(超声或红外)
   *
   * @param range_msg
   * @param topic_name
   */
  void rangeSensorCallback(const sensor_msgs::msg::Range::SharedPtr range_msg,
                           const std::string &topic_name);

  /**
   * @brief 雷达回调
   *
   * @param laser_scan_msg
   * @param frame_id
   */
  void laserScanCallback(
      const sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg,
      const std::string &frame_id);

  /**
   * @brief 点云消息回调函数.处理原始点云与分离后的动静态点云
   *
   * @param[in] ptr_message - 点云数据消息
   * @param[in] ptr_transform - 用于转换点云数据到全局坐标的变换
   * */
  void pointCloud2Callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msg,
      const std::string &frame_id);

  /**
   * @brief 碰撞开关消息回调函数.
   *
   * @param[in] switch_msg - 碰撞开关消息
   * */
  void collisionSwitchCallback(const std_msgs::msg::Bool::SharedPtr switch_msg,
                               const std::string &topic_name);

  /**
   * @brief 断崖红外消息回调函数.
   *
   * @param[in] switch_msg - 断崖消息
   * */
  void collisionSwitchRangeCallback(
      sensor_msgs::msg::Range::SharedPtr switch_msg,
      const std::string &topic_name);

  /**
   * @brief 避障红外消息回调
   * @param[in] switch_msg - 避障红外消息
   *
   */
  void infraredObstacleSwitchRangeCallback(
      sensor_msgs::msg::Range::SharedPtr switch_msg,
      const std::string &topic_name);

  /**
   * @brief 动态障碍物回调函数
   *
   * @param dynamic_object_msg
   * @param frame_id
   */
  void dynamicObjectCallback(
      const lidar_perception_msgs::msg::TrackingObjectArray::SharedPtr
          dynamic_object_msg,
      const std::string &frame_id);

  void getDynamicObsCloudMsg(
      sensor_msgs::msg::PointCloud &dynamic_obstacle_cloud);

  void convertTrackingObjsToDynamicObs(
      const lidar_perception_msgs::msg::TrackingObjectArray &dynamic_object_msg,
      DynamicObstacles &dynamic_obstacles);

  /// 变量
  std::shared_ptr<CostmapParamsRos2> costmap_params_ros2_ =
      nullptr;  ///< 和ros2的参数接口

  rclcpp::Node::SharedPtr node_ = nullptr;

  //防跌落
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr switch_range_sub;

  std::set<std::string> s_enanble_ranges_;
  std::map<std::string, std::string> m_ranges_topic_name_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      v_point_cloud2_subs_;  ///< 用于订阅scan数据

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
      v_laser_scan_subs_;  ///< 用于订阅scan数据

  rclcpp::Subscription<lidar_perception_msgs::msg::TrackingObjectArray>::
      SharedPtr dynamic_objects_sub_ = nullptr;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_switch_sub_ =
      nullptr;
  //触碰开关信号量系列
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
      v_collision_switch_sub_;

  //断崖红外系列
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr>
      v_collision_switch_range_sub_;

  //避障红外系列？ 该句作用
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr>
      v_infrared_obstacle_switch_range_sub_;  //订阅避障红外数据？

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sensor_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr>
      v_range_sensor_sub_;

  std::string s_origin_obstacle_clouds_name_;
  int i_detach_dynamic_point_cost_threshold_ = 128;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr
      dynamic_obstacle_clouds_pub_ = nullptr;  ///< 发布动态点云

  std::map<std::string, DataFilter<double>> filter_value;

  std::function<void(const std::string &)> update_func_;

  bool b_subscribe_point_cloud_topic_ = false;
  bool b_subscribe_range_topic_ = false;
  bool b_subscribe_dynamic_obs_topic_ = false;
  bool b_subscribe_switch_topic_ = false;
  bool b_subscribe_switch_range_topic_ = false;
  bool b_subscribe_infrared_obstacle_switch_range_topic_ = false;
  bool b_subscribe_ir_topic_ = false;

  //发布避障红外速度控制
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_ =
      nullptr;
  bool not_obs_front_ = true;
  std::thread pub_stop_vel_thread_;

  void pubStopVel();
};
}  // namespace CVTE_BABOT
#endif