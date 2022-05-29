/*
 * @Author: your name
 * @Date: 2021-06-01 16:49:22
 * @LastEditTime: 2021-06-09 11:11:44
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /navigation/navigation/speed_decision/include/speed_decision_adapter_ros2.hpp
 */
#ifndef SPEED_DECISION_ADAPTER_ROS2_HPP_
#define SPEED_DECISION_ADAPTER_ROS2_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "eigen3/Eigen/Core"
#include <functional>

namespace CVTE_BABOT {
class SpeedDecisionAdapterRos2 {
 public:
  SpeedDecisionAdapterRos2();
  SpeedDecisionAdapterRos2(const SpeedDecisionAdapterRos2 &obj) = delete;
  SpeedDecisionAdapterRos2 &operator=(const SpeedDecisionAdapterRos2 &obj) =
      delete;

  void init();
  void stop();
  void spin();
  void registerScanCallback(
      std::function<void(const std::vector<Eigen::Vector2d> &scan)>);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  void laserScanMsgCallback(
      const sensor_msgs::msg::LaserScan::SharedPtr ptr_laser_scan_msg);
  std::function<void(const std::vector<Eigen::Vector2d> &scan)>
      p_scan_callback_;
};
}  // namespace CVTE_BABOT

#endif