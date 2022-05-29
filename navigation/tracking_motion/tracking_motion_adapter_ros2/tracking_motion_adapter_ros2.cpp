/*
 * @Author: your name
 * @Date: 2020-10-30 11:07:53
 * @LastEditTime: 2021-07-02 10:52:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /src/tracking_motion/tracking_motion_ros2_adapter/tracking_motion_adapter_ros2.cpp
 */
#include "tracking_motion_adapter_ros2.hpp"
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include "log.hpp"
#include "pc_base.hpp"
namespace TRACKING_MOTION {

// TrackingAdapterRos2::ptr_tracking_adapter_ros2_ = nullptr;

// std::shared_ptr<TrackingAdapterRos2> TrackingAdapterRos2::getInstance() {
//     if(ptr_tracking_adapter_ros2_ == nullptr) {
//         ptr_tracking_adapter_ros2.reset(new TrackingAdapterRos2);
//     }
//     return ptr_tracking_adapter_ros2_;
// }

TrackingAdapterRos2::TrackingAdapterRos2() {
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>("tracking_motion_node", options);
  // mission_manager_node_ =
  // std::make_shared<rclcpp::Node>("tracking_motion_mission_manager_node");
  // pub_cmd_vel_ =
  // node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
  // pub_target_pose_ =
  // node_->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose",
  // 5); sub_odom_ =
  // node_->create_subscription<nav_msgs::msg::Odometry>("/odom",
  //     std::bind(&TrackingAdapterRos2::odomMsgCallback, this,
  //     std::placeholders::_1));
  // sub_point_cloud2_ =
  // node_->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points",
  //     std::bind(&TrackingAdapterRos2::pointCloudMsgCallback, this,
  //     std::placeholders::_1));
  // sub_laser_scan_ =
  // node_->create_subscription<sensor_msgs::msg::LaserScan>("/scan",
  //     std::bind(&TrackingAdapterRos2::laserScanMsgCallback, this,
  //     std::placeholders::_1));
  mission_manager_server_ =
      node_->create_service<mission_manager_msgs::srv::MissionManager>(
          "/tracking_motion_mission_manager",
          std::bind(&TrackingAdapterRos2::missionManagerCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));
  LOG(ERROR) << "construct tracking adapter ros2";
}

void TrackingAdapterRos2::updateParameter(std::string &config_file) {
  if (node_ == nullptr) {
    LOG(ERROR) << "node is nullptr";
    config_file = "./install/tracking_motion/params/tracking_motion.json";
    return;
  } else {
    node_->get_parameter_or(
        "config_file", config_file,
        std::string("./install/tracking_motion/params/tracking_motion.json"));
  }
}

void TrackingAdapterRos2::init() {
  LOG(ERROR) << "motion adapter ros2 init";
  pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/navi_vel", rclcpp::QoS(5).best_effort());
  pub_target_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/target_pose", rclcpp::QoS(5).best_effort());
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(5).best_effort(),
      std::bind(&TrackingAdapterRos2::odomMsgCallback, this,
                std::placeholders::_1));
  sub_point_cloud2_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", rclcpp::QoS(5).best_effort(),
      std::bind(&TrackingAdapterRos2::pointCloudMsgCallback, this,
                std::placeholders::_1));
  sub_laser_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::QoS(5).best_effort(),
      std::bind(&TrackingAdapterRos2::laserScanMsgCallback, this,
                std::placeholders::_1));
  sub_target_pose_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/target", rclcpp::QoS(5).best_effort(),
          std::bind(&TrackingAdapterRos2::targetMsgCallback, this,
                    std::placeholders::_1));
}

void TrackingAdapterRos2::stop() {
  pub_cmd_vel_.reset();
  pub_target_pose_.reset();
  sub_odom_.reset();
  sub_point_cloud2_.reset();
  sub_laser_scan_.reset();
}

void TrackingAdapterRos2::spin() {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  // exec.add_node(mission_manager_node_);
  exec.spin();
  exec.remove_node(node_);
  // exec.remove_node(mission_manager_node_);
}

void TrackingAdapterRos2::registerMissionManagerCallback(
    const std::function<void(std::string &, std::string &)>
        &p_mission_manager_callback) {
  p_mission_manager_callback_ = p_mission_manager_callback;
}
void TrackingAdapterRos2::registerOdomCallback(
    const std::function<void(const Mat34d &pose, const CmdVel &rev_vel)>
        &p_odom_callback) {
  p_odom_callback_ = p_odom_callback;
}
void TrackingAdapterRos2::registerScanCallback(
    const std::function<void(const std::vector<Eigen::Vector2d> &scan)>
        &p_scan_callback) {
  p_scan_callback_ = p_scan_callback;
}
void TrackingAdapterRos2::registerPointCloudCallback(
    const std::function<void(const laserCloud::Ptr ptr_laser_cloud)>
        &p_point_cloud_callback) {
  p_point_cloud_callback_ = p_point_cloud_callback;
}

void TrackingAdapterRos2::registerTargetCallback(
    const std::function<void(const Mat34d &pose)> &p_target_callback) {
  p_target_callback_ = p_target_callback;
}
void TrackingAdapterRos2::publishCmdVel(const CmdVel &cmd_vel) {
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = cmd_vel.vel_;
  cmd_vel_msg.angular.z = cmd_vel.w_;
  if (pub_cmd_vel_ != nullptr) {
    pub_cmd_vel_->publish(cmd_vel_msg);
  } else {
    LOG(ERROR) << "cmd vel publisher is nullptr";
  }
}

void TrackingAdapterRos2::publishPose(const Mat34d &pose) {
  geometry_msgs::msg::Pose pose_msg = toGeometryMsg(pose);
  geometry_msgs::msg::PoseStamped pose_time_stamp;
  pose_time_stamp.header.frame_id = "/map";
  pose_time_stamp.pose = pose_msg;
  if (pub_target_pose_ != nullptr) {
    pub_target_pose_->publish(pose_time_stamp);
  } else {
    LOG(ERROR) << "pub target pose is nullptr";
  }
}

void TrackingAdapterRos2::missionManagerCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
        request,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
        response) {
  LOG(INFO) << "receive mission manager msg";
  if (p_mission_manager_callback_ != nullptr) {
    p_mission_manager_callback_(request->send_data, response->ack_data);
  } else {
    LOG(ERROR) << "mission manager callback is nullptr";
  }
}
void TrackingAdapterRos2::odomMsgCallback(
    const nav_msgs::msg::Odometry::SharedPtr ptr_odom_msg) {
  // LOG(ERROR) << "receive odom msg";
  Mat34d pose = fromOdometryMsg(ptr_odom_msg);
  CmdVel rev_vel(ptr_odom_msg->twist.twist.linear.x,
                 ptr_odom_msg->twist.twist.angular.z);
  if (p_odom_callback_ != nullptr) {
    p_odom_callback_(pose, rev_vel);
  } else {
    LOG(ERROR) << "odom callback is nullptr";
  }
}
void TrackingAdapterRos2::pointCloudMsgCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr ptr_point_cloud2) {
  // LOG(ERROR) << "receive pointcloud";
  laserCloud::Ptr ptr_laser_cloud(new laserCloud);
  pcl::fromROSMsg(*ptr_point_cloud2, *ptr_laser_cloud);
  if (p_point_cloud_callback_ != nullptr) {
    p_point_cloud_callback_(ptr_laser_cloud);
  } else {
    LOG(ERROR) << "point cloud callback is nullptr";
  }
}
void TrackingAdapterRos2::laserScanMsgCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr ptr_laser_scan_msg) {
  // LOG(ERROR) << "receive scan msg";
  std::vector<Eigen::Vector2d> scan_data;
  double angle_inc = ptr_laser_scan_msg->angle_increment;
  double begin_angle = ptr_laser_scan_msg->angle_min;
  for (size_t index = 0; index < ptr_laser_scan_msg->ranges.size(); index++) {
    if (ptr_laser_scan_msg->ranges[index] > 0.001) {
      double angle = index * angle_inc + begin_angle;
      scan_data.push_back(
          Eigen::Vector2d(ptr_laser_scan_msg->ranges[index] * cos(angle),
                          ptr_laser_scan_msg->ranges[index] * sin(angle)));
    }
  }
  if (p_scan_callback_ != nullptr) {
    p_scan_callback_(scan_data);
  } else {
    LOG(ERROR) << "scan callback is nullptr";
  }
}

void TrackingAdapterRos2::targetMsgCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr ptr_target_msg) {
  Mat34d pose = fromGeometryMsg(ptr_target_msg->pose);
  if (p_target_callback_ != nullptr) {
    p_target_callback_(pose);
  }
}

}  // namespace TRACKING_MOTION