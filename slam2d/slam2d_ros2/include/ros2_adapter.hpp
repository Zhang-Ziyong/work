#ifndef _ROS2_ADPATERH_
#define _ROS2_ADPATERH_

#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "occupancy_map/depth_camera_occ_map.hpp"
#include "slam2d_params.hpp"
#include "slam_system/slam_system.hpp"
#include "state_machine/slam_state_machine.hpp"
#include "chassis_interfaces/msg/mapping_png.hpp"
#include "chassis_interfaces/msg/upload_run_log.hpp"
#include "chassis_interfaces/msg/take_lift_area_status.hpp"

namespace slam2d_ros2 {

using namespace slam2d_core::common;

class Ros2Adapter {
 public:
  explicit Ros2Adapter(rclcpp::Node::SharedPtr node);
  ~Ros2Adapter();

  Ros2Adapter(const Ros2Adapter &) = delete;
  Ros2Adapter &operator=(const Ros2Adapter &) = delete;

  void registerDataCallback();
  void resetDataCallback();

 private:
  void init();
  void loadParams();
  void publishMap();
  void publishData();
  void publishNodeList();
  void publishConstraintList();
  bool publishLocalizationResult();
  void caluateSeneorTime();

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void depthCloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
      const int &flag);
  void elevatorStatusCallback(
      const chassis_interfaces::msg::TakeLiftAreaStatus::SharedPtr
          elevator_status_msg);
  void publishLocalizationResult(const builtin_interfaces::msg::Time &time,
                                 const slam2d_core::common::Rigid3 &pose,
                                 const Eigen::Vector3d &cov);
  void publishSensorStatus();
  //   void publishOdometryDataUpdate();

  slam2d_core::slam_system::SlamSystem *ptr_slam_system_ = nullptr;
  std::shared_ptr<slam2d_core::occupancy_map::DepthOccupancyMap>
      ptr_depth_occ_map_ = nullptr;  ///< 深度相机避障地图

  Rigid3 laser_tf_;  ///< 激光雷达tf

  std::string scan_topic_ = "scan";  ///< 激光雷达topic
  std::string odom_topic_ = "odom";  ///< 里程计topic
  std::string imu_topic_ = "imu";    ///< IMU topic
  std::string depth_camera_front_up_topic_ =
      "obstacle_cloud_front_up";  ///< 深度相机topic
  std::string depth_camera_front_down_topic_ =
      "obstacle_cloud_front_down";  ///< 深度相机topic
  std::string step_depth_camera_front_up_topic_ = "step_cloud_front_up";
  std::string step_depth_camera_front_down_topic_ = "step_cloud_front_down";
  std::string elevator_status_topic_ = "/navi_manager/lift_area";

  std::string localization_pose_topic_ = "fusion_pose";  ///< 对外发布的定位频率
  bool use_imu_ = false;           ///< 是否使用imu的数据
  bool use_depth_camera_ = false;  ///< 是否使用深度相机的数据

  rclcpp::Node::SharedPtr node_;  ///< 用于收发数据的节点
  rclcpp::Node::SharedPtr node_pub_;

  std::thread pub_data_thread_;
  std::thread timer_monitor_thread_;

  std::chrono::system_clock::time_point last_laser_time_;
  bool obtained_laser_time_ = false;
  std::chrono::system_clock::time_point last_odom_time_;
  bool obtained_odom_time_ = false;

  rclcpp::Clock::SharedPtr ptr_clock_;

  std::unique_ptr<Slam2dParamsRos2> slam2d_params_ptr_ = nullptr;

  bool publish_tf_ = true;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tfb_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> ptr_tf2_buffer_ = nullptr;
  std::shared_ptr<tf2_ros::TransformListener> ptr_tfl_ = nullptr;

  std::shared_ptr<slam2d_core::state_machine::SlamStateMachine>
      ptr_state_machine_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odom_sub_;  ///<里程计订阅器
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
      imu_sub_;  ///< IMU订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_cloud_sub_;  ///<深度相机订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_cloud_front_up_sub_;  ///<深度相机订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_cloud_front_down_sub_;  ///<深度相机订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_step_cloud_front_down_sub_;  ///<深度相机订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      depth_step_cloud_front_up_sub_;  ///<深度相机订阅器
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_sub_;  ///<激光雷达订阅器
  rclcpp::Subscription<chassis_interfaces::msg::TakeLiftAreaStatus>::SharedPtr
      elevator_status_sub_;  ///< 电梯状态订阅器

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr loc_pf_set_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      trajectory_node_list_pub_;  ///< node list 发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      trajectory_constraint_list_pub_;  ///< constraint list发布器
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      occupancy_grid_publisher_;  ///< 地图发布器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_cloud_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr scan_cloud_pub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      laser_odom_pub;  //激光里程计发布

  slam2d_core::occupancy_map::UserOccupancyGrid global_occ_map_;

  bool debug_mode_ = true;
  rclcpp::Publisher<chassis_interfaces::msg::MappingPng>::SharedPtr
      mapping_png_publisher_;  ///< 实时构图png发布器

  std::map<slam2d_core::state_machine::SENSOR_STATUS, std::string>
      sensor_status_;  ///< 传感器状态map

  rclcpp::Publisher<chassis_interfaces::msg::UploadRunLog>::SharedPtr
      sensor_state_publisher_;  ///< 传感器状态发布器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_local_map_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_pre_aligned_cloud_;

  int publish_sensor_state_count_ = 0;
  slam2d_core::state_machine::SENSOR_STATUS cur_sensor_statu_ =
      slam2d_core::state_machine::SENSOR_STATUS::NORMAL;  ///< 传感器当前状态

  bool open_sensor_statu_pub_ = false;
  int sensor_statu_pub_delay_ = 60;

  // for odom real time delay
  std::string delay_real_time_;
  //   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
  //       odometry_data_update_publisher_;  ///< 实时odometry_data_update_ 发布

  tf2::Transform tf_laserodom_cloud_;
  bool is_obtained_transfrom_ = false;
};

}  // namespace slam2d_ros2

#endif