#ifndef _NEW_LIDAR_SLAM_ROS2_
#define _NEW_LIDAR_SLAM_ROS2_

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <time.h>
#include <fstream>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "occupancy_map/new_occupancy_map.hpp"
#include "occupancy_map/depth_camera_occ_map.hpp"
#include "common/math_base/slam_math.hpp"
#include "system/system.hpp"
#include "state_machine/slam_state_machine.hpp"
#include "chassis_interfaces/msg/mapping_png.hpp"
#include "chassis_interfaces/msg/upload_run_log.hpp"
#include "public_parameters/PublicParameters.hpp"
#include "load_param_ros2.hpp"

namespace cvte_lidar_slam {
class LidarSLAM {
 public:
  LidarSLAM(rclcpp::Node::SharedPtr sub_odom_node,
            rclcpp::Node::SharedPtr sub_lidar_node,
            rclcpp::Node::SharedPtr pub_node);
  LidarSLAM() = delete;
  ~LidarSLAM();
  LidarSLAM(const LidarSLAM &obj) = delete;
  LidarSLAM &operator=(const LidarSLAM &obj) = delete;
  void spin();
  void registerDataCallback();
  void resetDataCallback();
  bool testReLocalization();
  bool getMappingPngMsg(const UserOccupancyGrid &global_map,
                        chassis_interfaces::msg::MappingPng &mapping_png,
                        const std::string &map_temp_path);
  void base64Encode(const std::string &input, std::string &output);

 private:
  void init();
  void updateParameter();
  void clear();
  void laserScanDataCallback(
      const sensor_msgs::msg::LaserScan::SharedPtr laserScanMsg);
  void odomDataCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void IMUDataCallback(const sensor_msgs::msg::Imu::SharedPtr ImuMsg);
  void depthCloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
      const int &flag);

  void publishMap();

  void publishAllCloud();

  void publishPath(const Mat34d &slam_pose, const Mat34d &wheel_pose,
                   double time);

  void publishTf(const Mat34d &pose, double time);

  bool publishLocalizationResult(const Mat34d &pose, double time,
                                 const double cov);

  void publishSensorStatus();

  void visualizeLoopConstraintEdge(double time);

  void pointcloud2laserscanInitialization();

  void updateLaserScan();

  void caluateSeneorAbnormalValue();

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

  static inline nav_msgs::msg::Odometry toOdometryMsg(const Mat34d &in) {
    geometry_msgs::msg::Pose msg;
    msg = toGeometryMsg(in);
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.orientation.x = msg.orientation.x;
    odom_msg.pose.pose.orientation.y = msg.orientation.y;
    odom_msg.pose.pose.orientation.z = msg.orientation.z;
    odom_msg.pose.pose.orientation.w = msg.orientation.w;
    odom_msg.pose.pose.position.x = msg.position.x;
    odom_msg.pose.pose.position.y = msg.position.y;
    odom_msg.pose.pose.position.z = msg.position.z;
    return odom_msg;
  }

  inline Mat34d transformOdom(const Mat34d &in) {
    // 里程计坐标系下转换到世界坐标系下
    Mat34d T_ol = Mathbox::inversePose34d(T_lo_);
    Mat34d T_o_0w = Mathbox::inversePose34d(T_wo_0_);

    Mat34d To_0_i;
    To_0_i = Mathbox::multiplePose34d(T_o_0w, in);
    Mat34d out;
    out =
        Mathbox::multiplePose34d(Mathbox::multiplePose34d(T_lo_, To_0_i), T_ol);
    return out;
  }

 private:
  std::shared_ptr<SlamStateMachine> ptr_state_machine_;
  rclcpp::Node::SharedPtr sub_odom_node_;  ///< 用于收发数据的节点
  rclcpp::Node::SharedPtr sub_lidar_node_;
  rclcpp::Node::SharedPtr sub_depth_node_;
  rclcpp::Node::SharedPtr pub_node_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_laser_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_msg_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_depth_cloud_front_up_;  ///<深度相机订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_depth_cloud_front_down_;  ///<深度相机订阅器

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_depth_step_cloud_front_down_;  ///<深度相机订阅器
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_depth_step_cloud_front_up_;  ///<深度相机订阅器

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr
      sub_relocalization_msg_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_laser_odometry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_surroud_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corner_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_corner_cloud_top_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_surf_cloud_top_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_raw_cloud_top_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_loop_constrain_edge_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gps_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ref_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_slam_odom_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_lidar_odom_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_wheel_odom_path_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_debug_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_visual_hi_pub_;
  rclcpp::Publisher<chassis_interfaces::msg::UploadRunLog>::SharedPtr
      sensor_state_publisher_;  ///< 传感器状态发布器
  std::shared_ptr<DepthOccupancyMap> ptr_depth_occ_map_ = nullptr;

  rclcpp::Publisher<chassis_interfaces::msg::MappingPng>::SharedPtr
      mapping_png_publisher_;  ///< 实时构图png发布器
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_cloud_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tfb_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tfl_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_camerainit_map_ =
      nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_baselink_camera_ =
      nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_laser_rslidar_ =
      nullptr;

  std::shared_ptr<tf2_ros::Buffer> ptr_tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> ptr_tf_;

  geometry_msgs::msg::PoseStamped current_pose_stamped_;
  geometry_msgs::msg::TransformStamped robot_transform_;
  geometry_msgs::msg::TransformStamped laser_transform_;

  nav_msgs::msg::Path slam_odom_path_;
  nav_msgs::msg::Path wheel_odom_path_;
  nav_msgs::msg::Path lidar_odom_path_;

  laserCloud::Ptr laser_cloud_in_;  ///< 输入点云
  laserCloud::Ptr global_cloud_;    ///< 输出全局点云
  laserCloud::Ptr frame_pose_;      ///< 点云帧的位姿
  laserCloud::Ptr surround_cloud_;  ///< 输出全局点云

  laserCloud::Ptr corner_cloud_;
  laserCloud::Ptr surf_cloud_;
  laserCloud::Ptr corner_cloud_top_;
  laserCloud::Ptr surf_cloud_top_;

  sensor_msgs::msg::LaserScan laser_scan_;

  double last_keyframe_time_ = 0.;

  Mat34d T_wo_0_;  ///< 第一个odom数据
  Mat34d T_wo_;    ///< odom数据

  Mat34d T_frame_;               ///< 当前frame对应的odom位姿
  Mat34d T_lo_;                  ///< odom外参
  Mat34d smooth_pose_;           ///< 经过平滑的Pose
  Mat34d cur_odom_pose_;         ///< 当前的odom数据，最新的一个
  Mat34d last_raw_odom_pose_;    ///< 记录上一次odom的位置
  Mat34d last_odom_pose_;        ///< 记录上一次odom的位置
  Mat34d last_delta_odom_pose_;  ///< 记录上一次odom的位置

  bool is_first_cloud_ = true;
  bool is_first_odom_ = true;
  bool is_first_imu_ = true;
  bool is_first_map_;           ///< 判断是否发布地图
  bool is_first_localization_;  ///<第一次定位结果
  bool is_debug_;
  bool print_debug_;
  bool publish_tf_;
  bool publish_cloud_;
  bool publish_cloud_loc_;
  bool publish_global_cloud_;
  bool publish_global_cloud_loc_;
  bool publish_path_;
  size_t odom_count_;   ///<>
  size_t lidar_count_;  ///<>
  size_t imu_count_;    ///<>
  double first_odom_time_;
  double curr_odom_time_;
  double curr_lidar_time_;

  std::string lidar_topic_name_;       ///< 雷达点云topic name
  std::string wheel_odom_topic_name_;  ///< wheel_odom topic name
  std::string imu_topic_name_;         ///< imu topic name
  std::string path_file_;              ///< 轨迹保存路径

  std::string depth_camera_front_up_topic_ =
      "obstacle_cloud_front_up";  ///< 深度相机topic
  std::string depth_camera_front_down_topic_ =
      "obstacle_cloud_front_down";  ///< 深度相机topic
  std::string step_depth_camera_front_up_topic_ = "step_cloud_front_up";
  std::string step_depth_camera_front_down_topic_ = "step_cloud_front_down";
  bool use_depth_camera_ = false;  ///< 是否使用深度相机的数据

  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;
  System *ptr_slam_system_ = nullptr;  ///< 当前帧指针

  unsigned int sched_priority_ = 10;

  laserCloud::Ptr traver_cloud_in_;
  laserCloud::Ptr traver_cloud_obstacle_;
  laserCloud::Ptr traver_cloud_out_;
  Mat34d current_frame_pose_;
  Mat34d last_scan_frame_pose_;
  bool is_first_pose_;

  std::atomic<double> localization_pose_x_;
  std::atomic<double> localization_pose_y_;
  std::atomic<double> localization_pose_z_;
  std::atomic<double> localization_pose_yaw_;

  std::mutex robot_transform_mutex_;

  std::thread pub_cloud_thread_;
  std::thread pub_map_thread_;
  std::thread timer_monitor_thread_;
  //   std::ofstream pose_result_;
  unsigned int same_scan_count_ = 0;

  std::vector<std::pair<PointType, PointType>> curr_loop_pos_;

  SENSOR_STATUS cur_sensor_jump_ = SENSOR_STATUS::NORMAL;  ///< 传感器跳变判断
  SENSOR_STATUS cur_sensor_statu_ = SENSOR_STATUS::NORMAL;  ///< 传感器当前状态
  SENSOR_STATUS last_sensor_statu_ = SENSOR_STATUS::NORMAL;  ///< 传感器当前状态
  std::map<SENSOR_STATUS, std::string> sensor_status_;  ///< 传感器状态map

  std::chrono::system_clock::time_point last_laser_status_time_;
  bool obtained_laser_time_ = false;
  std::chrono::system_clock::time_point last_odom_status_time_;
  bool obtained_odom_time_ = false;

  bool open_sensor_statu_pub_ = false;
  int sensor_statu_pub_delay_ = 60;
  int publish_sensor_state_count_ = 0;
  std::string delay_real_time_;
  std::string odom_time_jump_;
  std::string odom_pose_jump_;
  std::string laser_time_jump_;

  std::unique_ptr<LoadParamsRos2> slam_params_ptr_ = nullptr;

  // depth down sample
  size_t depth_downsample_count_ = 0;
  double depthcloud_sample_frequent_ = 1.0;
  void inline depthcloudDownSample(depthCloud::Ptr cloud_in) {
    // 先将点云转换为PCLPointCloud2
    pcl::PCLPointCloud2::Ptr pcl_pointcloud2_tmp(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pcl_pointcloud2_downsample(
        new pcl::PCLPointCloud2());
    toPCLPointCloud2(*cloud_in, *pcl_pointcloud2_tmp);

    // 进行降采样
    pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;
    downsample.setInputCloud(pcl_pointcloud2_tmp);
    downsample.setLeafSize(0.05f, 0.05f, 0.05f);
    downsample.filter(*pcl_pointcloud2_downsample);

    // 转换会原始数据形式
    fromPCLPointCloud2(*pcl_pointcloud2_downsample, *cloud_in);
  }
  double publish_map_cost_all_time_ = 0.0;
  size_t map_numbers_ = 0;
};
}  // namespace cvte_lidar_slam
#endif