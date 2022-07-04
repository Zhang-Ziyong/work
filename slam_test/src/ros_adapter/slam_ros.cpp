#include "slam_ros.hpp"

namespace EIOLIDAR_SLAM {

SLAM_ROS::SLAM_ROS(rclcpp::Node::SharedPtr node) {
  node_ = node;

  topicsRegister();
}

void SLAM_ROS::loadParams() {}

void SLAM_ROS::odomDataCallBack(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  // 获取里程计数据，发布差值后的fusion_pose


}
void SLAM_ROS::imuDataCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
}

void SLAM_ROS::topicsRegister() {
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&SLAM_ROS::odomDataCallBack, this, std::placeholders::_1));

  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&SLAM_ROS::imuDataCallBack, this, std::placeholders::_1));

  pub_pose_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          fusion_pose_topic_, rclcpp::QoS(10).best_effort());
}
}  // namespace EIOLIDAR_SLAM

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  CVTE_BABOT::initGoogleLog("eiolidar_slam", "info");
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr eiolidar_slam_node_ptr =
      std::make_shared<rclcpp::Node>("eiolidar_slam", options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(eiolidar_slam_node_ptr);
  exec.spin();
  exec.remove_node(eiolidar_slam_node_ptr);

  rclcpp::shutdown();

  return 0;
}
