#ifndef _SLAM_ROS_HPP_
#define _SLAM_ROS_HPP_

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <string>
#include <thread>

#include "slam_system/slam_system.hpp"

#include "log.hpp"

namespace EIOLIDAR_SLAM {
class SLAM_ROS {
 public:
  explicit SLAM_ROS(rclcpp::Node::SharedPtr node);
  SLAM_ROS() = delete;
  ~SLAM_ROS();
  SLAM_ROS(const SLAM_ROS &obj) = delete;
  SLAM_ROS &operator=(const SLAM_ROS &obj) = delete;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_pose_;

  std::string odom_topic_ = "odom";
  std::string imu_topic_ = "imu";
  std::string fusion_pose_topic_ = "fusion_pose";

  void odomDataCallBack(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void imuDataCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void loadParams();
  void topicsRegister();

  // 数据格式转换
  static inline Mat34d fromGeometryMsg(const geometry_msgs::msg::Pose &msg) {
    Mat34d pose;
    Vec3d translation(msg.position.x, msg.position.y, msg.position.z);
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                         msg.orientation.y, msg.orientation.z);
    q.normalize();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = translation;
    return pose;
  }

  static inline Mat34d fromOdometryMsg(
      const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = odom_msg->pose.pose.orientation.x;
    msg.orientation.y = odom_msg->pose.pose.orientation.y;
    msg.orientation.z = odom_msg->pose.pose.orientation.z;
    msg.orientation.w = odom_msg->pose.pose.orientation.w;
    msg.position.x = odom_msg->pose.pose.position.x;
    msg.position.y = odom_msg->pose.pose.position.y;
    msg.position.z = odom_msg->pose.pose.position.z;
    Mat34d pose;
    pose = fromGeometryMsg(msg);
    return pose;
  }
};
}  // namespace EIOLIDAR_SLAM
#endif