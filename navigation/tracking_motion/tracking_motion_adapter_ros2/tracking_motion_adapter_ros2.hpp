/*
 * @Author: your name
 * @Date: 2020-10-29 17:54:53
 * @LastEditTime: 2021-07-02 10:54:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /src/tracking_motion/tracking_motion_ros2_adapter/tracking_motion_adapter_ros2.hpp
 */
#ifndef TRACKING_ADAPTER_ROS2_HPP_
#define TRACKING_ADAPTER_ROS2_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "mission_manager_msgs/srv/mission_manager.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "slam_math.hpp"
#include "data_struct.hpp"
#include "pc_base.hpp"
#include <string>
#include <functional>
#include <memory>
#include <mutex>
namespace TRACKING_MOTION {

class TrackingAdapterRos2 {
 public:
  TrackingAdapterRos2();
  TrackingAdapterRos2(const TrackingAdapterRos2 &obj) = delete;
  const TrackingAdapterRos2 &operator=(const TrackingAdapterRos2 &obj) = delete;
  void registerMissionManagerCallback(
      const std::function<void(std::string &, std::string &)>
          &p_mission_manager_callback);
  void registerOdomCallback(
      const std::function<void(const Mat34d &pose, const CmdVel &rev_vel)>
          &p_odom_callback);
  void registerScanCallback(
      const std::function<void(const std::vector<Eigen::Vector2d> &scan)>
          &p_scan_callback);
  void registerPointCloudCallback(
      const std::function<void(const laserCloud::Ptr ptr_point_cloud)>
          &p_point_cloud_callback);
  void registerTargetCallback(
      const std::function<void(const Mat34d &pose)> &p_target_callback);
  void updateParameter(std::string &config_file);
  void publishCmdVel(const CmdVel &cmd_vel);
  void publishPose(const Mat34d &pose);
  void init();
  void stop();
  void spin();
  // static std::shared_ptr<TrackingAdapterRos2> getInstance();
 private:
  // static std::shared_ptr<TrackingAdapterRos2> ptr_tracking_adapter_ros2_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr mission_manager_node_;
  rclcpp::Service<mission_manager_msgs::srv::MissionManager>::SharedPtr
      mission_manager_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      pub_target_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_target_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_point_cloud2_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  void missionManagerCallback(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
          request,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
          response);
  void odomMsgCallback(const nav_msgs::msg::Odometry::SharedPtr ptr_odom_msg);
  void pointCloudMsgCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr ptr_point_cloud2);
  void laserScanMsgCallback(
      const sensor_msgs::msg::LaserScan::SharedPtr ptr_laser_scan_msg);
  void targetMsgCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr ptr_target_msg);

  std::function<void(const Mat34d &pose, const CmdVel &rev_vel)>
      p_odom_callback_;
  std::function<void(const std::vector<Eigen::Vector2d> &scan)>
      p_scan_callback_;
  std::function<void(const laserCloud::Ptr ptr_point_cloud)>
      p_point_cloud_callback_;
  std::function<void(const Mat34d &pose)> p_target_callback_;
  std::function<void(std::string &, std::string &)> p_mission_manager_callback_;

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

  static inline geometry_msgs::msg::Pose toGeometryMsg(const Mat34d &in) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = in(0, 3);
    msg.position.y = in(1, 3);
    msg.position.z = in(2, 3);
    Eigen::Quaterniond q(in.block<3, 3>(0, 0));
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    return msg;
  }
};

}  // namespace TRACKING_MOTION

#endif