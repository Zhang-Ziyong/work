/*
 * @Author: your name
 * @Date: 2021-06-01 16:50:04
 * @LastEditTime: 2021-06-09 11:10:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /navigation/navigation/speed_decision/src/speed_decision_adapter_ros2.cpp
 */
#include "speed_decision_adapter_ros2.hpp"
#include "glog/logging.h"

namespace CVTE_BABOT {
SpeedDecisionAdapterRos2::SpeedDecisionAdapterRos2() {
  node_ = std::make_shared<rclcpp::Node>("SpeedDecisionControllerNode");
  LOG(INFO) << "construt speed decision controller";
}

void SpeedDecisionAdapterRos2::init() {
  sub_laser_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::QoS(5).best_effort(),
      std::bind(&SpeedDecisionAdapterRos2::laserScanMsgCallback, this,
                std::placeholders::_1));
  LOG(INFO) << "init speed decision adapter ros2";
}

void SpeedDecisionAdapterRos2::stop() {
  sub_laser_scan_.reset();
}

void SpeedDecisionAdapterRos2::spin() {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.spin();
  exec.remove_node(node_);
}

void SpeedDecisionAdapterRos2::registerScanCallback(
    std::function<void(const std::vector<Eigen::Vector2d> &scan)>
        p_scan_callback) {
  p_scan_callback_ = p_scan_callback;
}

void SpeedDecisionAdapterRos2::laserScanMsgCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr ptr_laser_scan_msg) {
  // LOG(ERROR) << "receive scan msg";
  std::vector<Eigen::Vector2d> scan;
  double angle_increment = ptr_laser_scan_msg->angle_increment;
  double angle_min = ptr_laser_scan_msg->angle_min;
  for (int index = 0; index < ptr_laser_scan_msg->ranges.size(); index++) {
    double angle = angle_min + angle_increment * index;
    scan.push_back(
        Eigen::Vector2d(ptr_laser_scan_msg->ranges[index] * cos(angle),
                        ptr_laser_scan_msg->ranges[index] * sin(angle)));
  }
  if (p_scan_callback_ != nullptr) {
    p_scan_callback_(scan);
  } else {
    LOG(INFO) << "scan callback is nullptr";
  }
}

}  // namespace CVTE_BABOT