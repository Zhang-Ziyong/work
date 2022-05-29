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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include "occupancy_map/new_occupancy_map.hpp"
#include "common/math_base/slam_math.hpp"
#include "system/system.hpp"
#include "occupancy_map/traversability_filter.hpp"
#include "state_machine/slam_state_machine.hpp"

namespace cvte_lidar_slam {

class LidarSLAM {
 public:
  LidarSLAM(rclcpp::Node::SharedPtr node);
  LidarSLAM() = delete;
  ~LidarSLAM();
  LidarSLAM(const LidarSLAM &obj) = delete;
  LidarSLAM &operator=(const LidarSLAM &obj) = delete;
  void spin();
  void registerDataCallback();
  void resetDataCallback();

 private:
  void init();
  void updateParameter();
  void clear();
  void pubRefPath(nav_msgs::msg::Path &path);
  void pointCloudDataCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
  void odomDataCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);

  void publishAllCloud();

  void publishOdometryAndTF(const Mat34d &pose, double time);

  void publishMap();

  void publishCloud(double time);

  bool publishLocalizationResult(const Mat34d &pose, double time,
                                 const double cov);

  void traverFilter(const laserCloud::Ptr laser_cloud_in, double time);

  void pointcloud2laserscanInitialization();

  void updateLaserScan();

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
    Mat34d To_0_i;
    To_0_i = Mathbox::multiplePose34d(Mathbox::inversePose34d(T_wo_0_), in);
    Mat34d out;
    out = Mathbox::multiplePose34d(Mathbox::multiplePose34d(T_lo_, To_0_i),
                                   Mathbox::inversePose34d(T_lo_));
    return out;
  }

 private:
  std::shared_ptr<SlamStateMachine> ptr_state_machine_;
  rclcpp::Node::SharedPtr node_;  ///< 用于收发数据的节点
  rclcpp::Node::SharedPtr node_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_laser_cloud_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_msg_;

  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr
      sub_relocalization_msg_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_laser_odometry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gps_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_pose_;
  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_vi_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ref_path_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
  //           pub_full_info_cloud_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_visual_hi_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tfb_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_camerainit_map_ =
      nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> ptr_tf_baselink_camera_ =
      nullptr;

  std::shared_ptr<tf2_ros::Buffer> ptr_tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> ptr_tf_;

  geometry_msgs::msg::PoseStamped current_pose_stamped_;
  geometry_msgs::msg::TransformStamped robot_transform_;
  geometry_msgs::msg::TransformStamped laser_transform_;

  laserCloud::Ptr laser_cloud_in_;  ///< 输入点云
  laserCloud::Ptr global_cloud_;    ///< 输出全局点云
  laserCloud::Ptr frame_pose_;      ///< 点云帧的位姿

  sensor_msgs::msg::LaserScan laser_scan_;

  double last_keyframe_time_ = 0.;

  Mat34d T_wo_0_;          ///< 第一个odom数据
  Mat34d T_frame_;         ///< 当前frame对应的odom位姿
  Mat34d T_lo_;            ///< odom外参
  Mat34d smooth_pose_;     ///< 经过平滑的Pose
  Mat34d cur_odom_pose_;   ///< 当前的odom数据，最新的一个
  Mat34d last_odom_pose_;  ///< 记录上一次odom的位置

  bool is_first_odom_ = true;
  bool is_first_map_;           ///< 判断是否发布地图
  bool is_first_localization_;  ///<第一次定位结果
  bool is_debug_;
  size_t odom_count_;   ///<>
  size_t lidar_count_;  ///<>

  std::string config_file_;       ///< 配置文件路径
  std::string lidar_topic_name_;  ///< 雷达点云topic name
  std::string path_file_;         ///< 轨迹保存路径

  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;
  System *ptr_slam_system_ = nullptr;  ///< 当前帧指针

  unsigned int sched_priority_ = 10;

  TraversabilityFilter traver_filter_;

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
  std::thread query_state_machine_thread_;
  std::ofstream pose_result_;
  unsigned int same_scan_count_ = 0;
};
}  // namespace cvte_lidar_slam
