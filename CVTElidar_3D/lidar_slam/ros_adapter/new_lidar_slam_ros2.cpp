#include "new_lidar_slam_ros2.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "log.hpp"

namespace cvte_lidar_slam {
using namespace std::chrono_literals;

LidarSLAM::LidarSLAM(rclcpp::Node::SharedPtr node) {
  node_ = node;
  laser_cloud_in_.reset(new laserCloud());
  global_cloud_.reset(new laserCloud());
  frame_pose_.reset(new laserCloud());
  traver_cloud_in_.reset(new laserCloud());
  traver_cloud_obstacle_.reset(new laserCloud());
  traver_cloud_out_.reset(new laserCloud());
  is_first_map_ = true;
  is_first_pose_ = true;
  is_first_odom_ = true;
  odom_count_ = 0;
  lidar_count_ = 0;
  ptr_state_machine_ = SlamStateMachine::getInstance();
  ptr_slam_system_ = System::getInstance();
  init();
}

LidarSLAM::~LidarSLAM() {
  LOG(ERROR) << "SENSOR COUNT " << odom_count_ << ' ' << lidar_count_;
  pose_result_.close();
  ptr_slam_system_->shutdown();
}

void LidarSLAM::updateParameter() {
  node_->get_parameter_or(
      "config_file", config_file_,
      std::string("./install/cvte_lidar_slam/params/velodyne.yaml"));

  std::vector<double> t_lo = {1., 0., 0., -0.32, 0., 1.,
                              0., 0., 0., 0.,    1., 0.};
  node_->get_parameter_or("t_lo", t_lo, t_lo);
  T_lo_ << t_lo[0], t_lo[1], t_lo[2], t_lo[3], t_lo[4], t_lo[5], t_lo[6],
      t_lo[7], t_lo[8], t_lo[9], t_lo[10], t_lo[11];
  ptr_slam_system_->setExtrincs(T_lo_);
  node_->get_parameter_or("is_debug", is_debug_, false);
  LOG(WARNING) << "\nT_lo: \n" << T_lo_;
  LOG(WARNING) << "config_file: " << config_file_;
  LOG(WARNING) << "is_debug: " << (is_debug_ ? "True" : "False");
}

// TODO: 初始化各参数 flag 读取参数文件,初始化node/pub
void LidarSLAM::init() {
  localization_pose_x_ = 0;
  localization_pose_y_ = 0;
  localization_pose_z_ = 0;
  localization_pose_yaw_ = 0;
  pose_result_.open("./pose_result.txt");

  node_pub_ = std::make_shared<rclcpp::Node>("lidar_slam_pub");

  updateParameter();

  pub_laser_odometry_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/laser_map_to_init", 5);
  pub_global_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/global_map_cloud_", 2);
  pub_frame_pose_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/frame_pose_cloud_", 2);
  pub_gps_pose_ = node_->create_publisher<nav_msgs::msg::Odometry>("/gps_pose");

  pub_pose_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/fusion_pose");

  pub_ref_path_ = node_->create_publisher<nav_msgs::msg::Path>("/ref_path");

  cloud_visual_hi_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_pointcloud", 5);
  if (is_debug_) {
    ptr_tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_pub_);
    ptr_tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_pub_->get_clock());
    ptr_tf2_buffer_->setUsingDedicatedThread(true);
    ptr_tf_ = std::make_shared<tf2_ros::TransformListener>(*ptr_tf2_buffer_);
    ptr_tf_camerainit_map_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(node_pub_);
    ptr_tf_baselink_camera_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(node_pub_);
  }

  ptr_config_ = std::make_shared<SystemConfig>(config_file_);
  lidar_topic_name_ = ptr_config_->lidar_topic_name;
  ptr_slam_system_->setConfig(ptr_config_);
  if (is_debug_) {
    pub_cloud_thread_ =
        std::thread(std::bind(&LidarSLAM::publishAllCloud, this));
    traver_filter_.allocateMemory();
  }
}
void LidarSLAM::clear() {}

// TODO: 注册callback
void LidarSLAM::registerDataCallback() {
  if (nullptr == sub_odom_ || nullptr == sub_laser_cloud_) {
    LOG(WARNING) << ".....................register data "
                    "call_back......................";
    is_first_odom_ = true;
    odom_count_ = 0;
    lidar_count_ = 0;
    sub_laser_cloud_ =
        node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_name_, std::bind(&LidarSLAM::pointCloudDataCallback,
                                         this, std::placeholders::_1));

    sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        std::bind(&LidarSLAM::odomDataCallback, this, std::placeholders::_1),
        50);

    // sub_gps_msg_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    //     "/gps_msg",
    //     std::bind(&LidarSLAM::gpsDataCallback, this, std::placeholders::_1));
  }
}

void LidarSLAM::resetDataCallback() {
  sub_laser_cloud_.reset();
  sub_odom_.reset();
  sub_gps_msg_.reset();
  sub_relocalization_msg_.reset();
  is_first_pose_ = true;
  is_first_map_ = true;
  LOG(INFO) << "reset all data callback.";
}

void LidarSLAM::pubRefPath(nav_msgs::msg::Path &path) {
  rclcpp::Clock::SharedPtr ptr_clock;
  ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  path.header.stamp = ptr_clock->now();
  path.header.frame_id = "map";
  pub_ref_path_->publish(path);
}

void LidarSLAM::pointCloudDataCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
  if (ptr_state_machine_->getCurrentState() == AD_STATU::UNKNOW) {
    return;
  }
  lidar_count_++;
  double time_stamp_now =
      double(laserCloudMsg->header.stamp.sec) +
      double(laserCloudMsg->header.stamp.nanosec) * (1.0 * 1e-9);
  static double last_timestamp = time_stamp_now;
  double delta_time = time_stamp_now - last_timestamp;
  last_timestamp = time_stamp_now;
  if (fabs(delta_time) < 0.001) {
    return;
  }
  if (laserCloudMsg->point_step == 26) {
    laser_cloud_in_->clear();
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*laserCloudMsg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*laserCloudMsg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*laserCloudMsg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*laserCloudMsg,
                                                                "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_line(*laserCloudMsg,
                                                             "line");
    uint32_t n_points = laserCloudMsg->width;
    for (size_t i = 0; i < n_points;
         ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_line) {
      PointType point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      point.intensity = 10 * (*iter_intensity) + *iter_line;
      laser_cloud_in_->push_back(point);
    }
  } else {
    pcl::fromROSMsg(*laserCloudMsg, *laser_cloud_in_);
  }
  last_keyframe_time_ = time_stamp_now;

  ptr_slam_system_->addCloudFrame(laser_cloud_in_, time_stamp_now);
  if (is_debug_) {
    traverFilter(laser_cloud_in_, time_stamp_now);
  }
}

void LidarSLAM::odomDataCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  if (ptr_state_machine_->getCurrentState() == AD_STATU::UNKNOW) {
    return;
  }
  odom_count_++;
  auto odom_pose = fromOdometryMsg(odom_msg);
  double time_stamp = odom_msg->header.stamp.sec +
                      odom_msg->header.stamp.nanosec * (1.0 * 1e-9);
  if (is_first_odom_) {
    is_first_odom_ = false;
     T_wo_0_ = odom_pose;
    last_odom_pose_ = odom_pose;
  }
  // 判断里程计是否出现异常值
  Mat34d delta_trans = Mathbox::deltaPose34d(last_odom_pose_, odom_pose);
  last_odom_pose_ = odom_pose;
  double delta_rot =
      Mathbox::rotationMatrixToEulerAngles(delta_trans.block<3, 3>(0, 0))
          .norm();
  double delta_translation = delta_trans.block<3, 1>(0, 3).norm();
  if (delta_translation > 3 || delta_rot > 1) {
    LOG(ERROR) << "Tool large odom delta_translation" << delta_translation
               << "," << delta_rot;
    return;
  }
  Mat34d T_wo = transformOdom(odom_pose);
  ptr_slam_system_->addOdom(OdomMeasure(T_wo, time_stamp));
  static double last_odom_time = time_stamp;
  if ((time_stamp - last_odom_time) > 0.1) {
    LOG(WARNING) << "odom frame lost  " << (time_stamp - last_odom_time)
                 << " s";
  }
  last_odom_time = time_stamp;
  if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_LOCALIZATION) {
    if (ptr_slam_system_->isLocalizationInit()) {
      if (ptr_state_machine_->getCurrentState() ==
          AD_STATU::INIT_LOCALIZATION) {
        ptr_state_machine_->sendEvent(EventName::INIT_LOCALIZATION_SUCCESS);
      }
      RobotState robot_state = ptr_slam_system_->getRobotState();
      Mat34d predict_cur_frame_pose = robot_state.pose;
      double cov = ptr_slam_system_->getPoseCov();
      if (is_debug_) {
        publishOdometryAndTF(robot_state.pose, time_stamp);
      }
      publishLocalizationResult(predict_cur_frame_pose, time_stamp, cov);
      Mat34d transform_in_base =
          Mathbox::multiplePose34d(predict_cur_frame_pose, T_lo_);
      localization_pose_x_ = transform_in_base(0, 3);
      localization_pose_y_ = transform_in_base(1, 3);
      localization_pose_z_ = transform_in_base(2, 3);
      if (is_debug_) {
        Eigen::Quaterniond q(predict_cur_frame_pose.block<3, 3>(0, 0));
        pose_result_ << std::setprecision(15) << time_stamp << ' '
                     << localization_pose_x_ << ' ' << localization_pose_y_
                     << ' ' << localization_pose_z_ << ' ' << q.x() << ' '
                     << q.y() << ' ' << q.z() << ' ' << q.w() << std::endl;
      }
      Vec3d angle = Mathbox::rotationMatrixToEulerAngles(
          predict_cur_frame_pose.block<3, 3>(0, 0));
      localization_pose_yaw_ = angle(2);
    }
  } else if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING) {
    RobotState robot_state = ptr_slam_system_->getRobotState();
    localization_pose_x_ = robot_state.pose(0, 3);
    localization_pose_y_ = robot_state.pose(1, 3);
    localization_pose_z_ = robot_state.pose(2, 3);
    Vec3d angle = Mathbox::rotationMatrixToEulerAngles(
        robot_state.pose.block<3, 3>(0, 0));
    localization_pose_yaw_ = angle(2);
    double cov = 1.;
    if (is_debug_) {
      Eigen::Quaterniond q(robot_state.pose.block<3, 3>(0, 0));
      pose_result_ << std::setprecision(15) << time_stamp << ' '
                   << localization_pose_x_ << ' ' << localization_pose_y_ << ' '
                   << localization_pose_z_ << ' ' << q.x() << ' ' << q.y()
                   << ' ' << q.z() << ' ' << q.w() << std::endl;
      publishOdometryAndTF(robot_state.pose, time_stamp);
    }
    publishLocalizationResult(robot_state.pose, time_stamp, cov);
  } else {
    localization_pose_x_ = 0;
    localization_pose_y_ = 0;
    localization_pose_z_ = 0;
    localization_pose_yaw_ = 0;
  }
}

void LidarSLAM::gpsDataCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg) {
  if (ptr_state_machine_->getCurrentState() == AD_STATU::UNKNOW) {
    return;
  }
  if (gps_msg->position_covariance_type != 4) {
    return;
  }
  GpsMsg msg;
  msg.time_stamp =
      gps_msg->header.stamp.sec + gps_msg->header.stamp.nanosec * (1.0 * 1e-9);
  msg.latitude = gps_msg->latitude;
  msg.longitude = gps_msg->longitude;
  msg.altitude = gps_msg->altitude;
  msg.cov = gps_msg->position_covariance[0];
  msg.loc_type = gps_msg->position_covariance_type;
  ptr_slam_system_->addGPS(msg);
}

void LidarSLAM::publishAllCloud() {
  while (rclcpp::ok()) {
    if (ptr_slam_system_ != nullptr) {
      if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
        if (is_first_map_) {
          this->publishMap();
        }
      } else if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
        bool flag = ptr_slam_system_->getViewCloud(global_cloud_, frame_pose_);
        if (flag) {
          publishCloud(last_keyframe_time_);
        }
        Vec3d pose;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  }
}

void LidarSLAM::publishOdometryAndTF(const Mat34d &pose, double time) {
  nav_msgs::msg::Odometry laserOdometry;
  rclcpp::Time now_time(time * 1e9);
  laserOdometry.header.frame_id = "/map";
  laserOdometry.child_frame_id = "/laser";
  laserOdometry.header.stamp = now_time;
  laserOdometry.pose.pose = toGeometryMsg(pose);
  pub_laser_odometry_->publish(laserOdometry);

  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  Eigen::Quaterniond geoQuat(pose.block<3, 3>(0, 0));
  tf2::Transform tf;
  tf2::Quaternion q(geoQuat.x(), geoQuat.y(), geoQuat.z(), geoQuat.w());
  tf = tf2::Transform(q, tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  tmp_tf_stamped.header.frame_id = "/map";
  tmp_tf_stamped.child_frame_id = "/laser";
  tmp_tf_stamped.header.stamp = laserOdometry.header.stamp;
  tmp_tf_stamped.transform = tf2::toMsg(tf);
  ptr_tfb_->sendTransform(tmp_tf_stamped);
  q.setRPY(0, 0, 0);
  tf = tf2::Transform(q, tf2::Vector3(T_lo_(0, 3), 0, 0));
  tmp_tf_stamped.header.frame_id = "/laser";
  tmp_tf_stamped.child_frame_id = "/base_link";
  tmp_tf_stamped.header.stamp = laserOdometry.header.stamp;
  tmp_tf_stamped.transform = tf2::toMsg(tf);
  ptr_tf_baselink_camera_->sendTransform(tmp_tf_stamped);
}

void LidarSLAM::publishMap() {
  if (ptr_state_machine_->getCurrentState() != AD_STATU::UNKNOW) {
    is_first_map_ = false;
    global_cloud_ = ptr_slam_system_->getWholeMap();
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*global_cloud_, cloudMsgTemp);
    rclcpp::Clock::SharedPtr ptr_clock;
    ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    cloudMsgTemp.header.stamp = ptr_clock->now();
    cloudMsgTemp.header.frame_id = "/map";
    pub_global_cloud_->publish(cloudMsgTemp);
    std::cout << "pub map.  " << global_cloud_->size() << std::endl;
  }
}

void LidarSLAM::publishCloud(double time) {
  rclcpp::Time now_time(time * 1e9);
  sensor_msgs::msg::PointCloud2 cloudMsgTemp;

  pcl::toROSMsg(*global_cloud_, cloudMsgTemp);
  cloudMsgTemp.header.stamp = now_time;
  cloudMsgTemp.header.frame_id = "/map";
  pub_global_cloud_->publish(cloudMsgTemp);

  pcl::toROSMsg(*frame_pose_, cloudMsgTemp);
  cloudMsgTemp.header.stamp = now_time;
  cloudMsgTemp.header.frame_id = "/map";
  pub_frame_pose_->publish(cloudMsgTemp);
}

bool LidarSLAM::publishLocalizationResult(const Mat34d &pose, double time,
                                          const double cov) {
  Mat34d transform_in_base = Mathbox::multiplePose34d(pose, T_lo_);
  geometry_msgs::msg::Pose base_pose = toGeometryMsg(transform_in_base);
  geometry_msgs::msg::Pose laser_pose = toGeometryMsg(pose);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msgs;
  geometry_msgs::msg::Pose robot_pose;
  rclcpp::Time now_time(time * 1e9);
  robot_transform_mutex_.lock();
  robot_transform_.header.stamp = now_time;
  robot_transform_.transform.translation.x = base_pose.position.x;
  robot_transform_.transform.translation.y = base_pose.position.y;
  robot_transform_.transform.translation.z = base_pose.position.z;
  robot_transform_.transform.rotation.x = base_pose.orientation.x;
  robot_transform_.transform.rotation.y = base_pose.orientation.y;
  robot_transform_.transform.rotation.z = base_pose.orientation.z;
  robot_transform_.transform.rotation.w = base_pose.orientation.w;
  laser_transform_.transform.translation.x = laser_pose.position.x;
  laser_transform_.transform.translation.y = laser_pose.position.y;
  laser_transform_.transform.translation.z = laser_pose.position.z;
  laser_transform_.transform.rotation.x = laser_pose.orientation.x;
  laser_transform_.transform.rotation.y = laser_pose.orientation.y;
  laser_transform_.transform.rotation.z = laser_pose.orientation.z;
  laser_transform_.transform.rotation.w = laser_pose.orientation.w;
  geometry_msgs::msg::TransformStamped robot_transform = robot_transform_;
  current_frame_pose_ = pose;
  robot_transform_mutex_.unlock();
  pose_msgs.header.stamp = now_time;
  pose_msgs.header.frame_id = "map";
  pose_msgs.pose.pose.position.x = robot_transform.transform.translation.x;
  pose_msgs.pose.pose.position.y = robot_transform.transform.translation.y;
  pose_msgs.pose.pose.position.z = robot_transform.transform.translation.z;
  pose_msgs.pose.pose.orientation.x = robot_transform.transform.rotation.x;
  pose_msgs.pose.pose.orientation.y = robot_transform.transform.rotation.y;
  pose_msgs.pose.pose.orientation.z = robot_transform.transform.rotation.z;
  pose_msgs.pose.pose.orientation.w = robot_transform.transform.rotation.w;
  pose_msgs.pose.covariance[0] = cov;
  robot_pose.position.x = robot_transform.transform.translation.x;
  robot_pose.position.y = robot_transform.transform.translation.y;
  robot_pose.position.z = robot_transform.transform.translation.z;
  robot_pose.orientation.x = robot_transform.transform.rotation.x;
  robot_pose.orientation.y = robot_transform.transform.rotation.y;
  robot_pose.orientation.z = robot_transform.transform.rotation.z;
  robot_pose.orientation.w = robot_transform.transform.rotation.w;
  pub_pose_->publish(pose_msgs);

  return true;
}

void LidarSLAM::traverFilter(const laserCloud::Ptr laser_cloud_in,
                             double time) {
  bool mapping_mode = false;
  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    mapping_mode = true;
  } else {
    mapping_mode = false;
  }
  traver_filter_.setInputCloud(*laser_cloud_in);
  traver_filter_.processFilter(traver_cloud_out_, traver_cloud_obstacle_,
                               mapping_mode);
  // Publish laserCloudOut for visualization (before downsample and BGK
  // prediction)
  sensor_msgs::msg::PointCloud2 laserCloudTemp;
  rclcpp::Clock::SharedPtr ptr_clock;
  ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  pcl::toROSMsg(*traver_cloud_obstacle_, laserCloudTemp);
  rclcpp::Time now_time(time * 1e9);
  laserCloudTemp.header.stamp = now_time;
  laserCloudTemp.header.frame_id = "laser";
  cloud_visual_hi_pub_->publish(laserCloudTemp);

  traver_filter_.resetVariables();

  traver_cloud_in_->clear();
  traver_cloud_obstacle_->clear();
  traver_cloud_out_->clear();
}

}  // namespace cvte_lidar_slam
