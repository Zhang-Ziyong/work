#include "ros2_adapter.hpp"
#include "occupancy_map/occupancy_map_common.hpp"

#include "conversion.hpp"
#include "glog/logging.h"

namespace slam2d_ros2 {
using namespace std::chrono_literals;

Ros2Adapter::Ros2Adapter(rclcpp::Node::SharedPtr node) {
  node_ = node;
  ptr_state_machine_ =
      slam2d_core::state_machine::SlamStateMachine::getInstance();
  ptr_slam_system_ = slam2d_core::slam_system::SlamSystem::getInstance();

  ptr_depth_occ_map_ =
      slam2d_core::occupancy_map::DepthOccupancyMap::getInstance();

  ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  init();
}

Ros2Adapter::~Ros2Adapter() {}

void Ros2Adapter::init() {
  loadParams();

  pub_pose_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          localization_pose_topic_, rclcpp::QoS(10).best_effort());
  loc_pf_set_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
      "particlecloud", rclcpp::QoS(10).best_effort());

  trajectory_node_list_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "node_list", rclcpp::QoS(10).best_effort());

  trajectory_constraint_list_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "constraint_list", rclcpp::QoS(10).best_effort());

  obstacle_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/obstacle_cloud", rclcpp::QoS(2).best_effort());

  scan_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>(
      "/scan_cloud", rclcpp::QoS(2).best_effort());

  laser_odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/laser_odometry", rclcpp::QoS(2).best_effort());

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  // custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  // custom_qos_profile.depth = 1;
  // custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  // custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  occupancy_grid_publisher_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "map", rclcpp::QoS(1).best_effort());
  mapping_png_publisher_ =
      node_->create_publisher<chassis_interfaces::msg::MappingPng>(
          "mapping_png", rclcpp::QoS(1).best_effort());
  sensor_state_publisher_ =
      node_->create_publisher<chassis_interfaces::msg::UploadRunLog>(
          "state_pub", rclcpp::QoS(1).best_effort());

  pub_local_map_cloud_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "local_map_cloud", rclcpp::QoS(1).best_effort());
  pub_pre_aligned_cloud_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "pre_aligned_cloud", rclcpp::QoS(1).best_effort());
  // odometry_data_update_publisher_ =
  //     node_->create_publisher<nav_msgs::msg::Odometry>(
  //         "odometry_data_update_pub", rclcpp::QoS(1).best_effort());

  ptr_tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  ptr_tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  ptr_tf2_buffer_->setUsingDedicatedThread(true);
  ptr_tfl_ = std::make_shared<tf2_ros::TransformListener>(*ptr_tf2_buffer_);

  pub_data_thread_ = std::thread(std::bind(&Ros2Adapter::publishData, this));

  // add sensor timer moniter thread
  timer_monitor_thread_ =
      std::thread(std::bind(&Ros2Adapter::caluateSeneorTime, this));

  // add sensor status
  sensor_status_ = {
      {slam2d_core::state_machine::SENSOR_STATUS::ODOM_DELAY,
       "警告： 里程计数据延时"},
      {slam2d_core::state_machine::SENSOR_STATUS::ODOM_TIME_STAMP_JUMP,
       "警告： 里程计时间戳跳变"},
      {slam2d_core::state_machine::SENSOR_STATUS::ODOM_POSE_JUMP,
       "警告： 里程计位姿跳变"},
      {slam2d_core::state_machine::SENSOR_STATUS::LASER_DELAY,
       "警告： 雷达数据延时"}};
}

void Ros2Adapter::loadParams() {
  std::vector<double> laser_tf(3, 0.0);
  node_->get_parameter_or("ros2_params.laser_tf", laser_tf, laser_tf);
  geometry_msgs::msg::Pose baselink_to_laser;
  baselink_to_laser.position.x = laser_tf[0];
  baselink_to_laser.position.y = laser_tf[1];
  baselink_to_laser.position.z = 0.0;
  auto q = slam2d_core::common::yawToQuaternion(0.0, 0.0, laser_tf[2]);
  baselink_to_laser.orientation.x = q.x();
  baselink_to_laser.orientation.y = q.y();
  baselink_to_laser.orientation.z = q.z();
  baselink_to_laser.orientation.w = q.w();
  laser_tf_ = toRigid3(baselink_to_laser);

  ptr_slam_system_->setLaserTf(laser_tf_);

  node_->get_parameter_or("ros2_params.odometry_topic", odom_topic_,
                          std::string("odom"));
  node_->get_parameter_or("ros2_params.imu_topic", imu_topic_,
                          std::string("imu"));
  node_->get_parameter_or("ros2_params.scan_topic", scan_topic_,
                          std::string("scan"));
  node_->get_parameter_or("ros2_params.depth_camera_front_up_topic",
                          depth_camera_front_up_topic_,
                          std::string("orbbec_cloud"));
  node_->get_parameter_or("ros2_params.depth_camera_front_down_topic",
                          depth_camera_front_down_topic_,
                          std::string("orbbec_cloud"));
  node_->get_parameter_or("ros2_params.step_depth_camera_front_up_topic",
                          step_depth_camera_front_up_topic_,
                          std::string("orbbec_cloud"));
  node_->get_parameter_or("ros2_params.step_depth_camera_front_down_topic",
                          step_depth_camera_front_down_topic_,
                          std::string("orbbec_cloud"));
  node_->get_parameter_or("ros2_params.elevator_status_topic",
                          elevator_status_topic_,
                          std::string("/navi_manager/lift_area"));

  node_->get_parameter_or("ros2_params.localization_pose_topic",
                          localization_pose_topic_, std::string("fusion_pose"));
  node_->get_parameter_or("ros2_params.use_imu", use_imu_, false);
  node_->get_parameter_or("ros2_params.use_depth_camera", use_depth_camera_,
                          false);

  node_->get_parameter_or("ros2_params.debug_mode", debug_mode_, true);

  slam2d_params_ptr_ = std::make_unique<Slam2dParamsRos2>(node_);
  slam2d_params_ptr_->loadLocalTrajectoryBuilderParams();
  slam2d_params_ptr_->loadPoseGraphParams();
  slam2d_params_ptr_->loadLocalizationParams();
  slam2d_params_ptr_->loadAmclParams();
  slam2d_params_ptr_->loadMSFParams();
  slam2d_params_ptr_->loadGSMParams();
  slam2d_params_ptr_->loadDepthCameraParams();

  slam2d_params_ptr_->amcl_options_.laser_pose_x_ = laser_tf[0];
  slam2d_params_ptr_->amcl_options_.laser_pose_y_ = laser_tf[1];
  slam2d_params_ptr_->amcl_options_.laser_pose_yaw_ = laser_tf[2];

  slam2d_params_ptr_->loc_options.laser_tf[0] = laser_tf[0];
  slam2d_params_ptr_->loc_options.laser_tf[1] = laser_tf[1];
  slam2d_params_ptr_->loc_options.laser_tf[2] = laser_tf[2];

  ptr_slam_system_->setParams(slam2d_params_ptr_->pose_graph_options_,
                              slam2d_params_ptr_->local_trajectory_options_);
  ptr_slam_system_->setParams(
      slam2d_params_ptr_->loc_options, slam2d_params_ptr_->amcl_options_,
      slam2d_params_ptr_->msf_options_, slam2d_params_ptr_->gsm_options_);

  ptr_depth_occ_map_->setCameraCalibrateOption(
      slam2d_params_ptr_->depth_camera_options_);
  ptr_depth_occ_map_->setDepthOccMapOption(
      slam2d_params_ptr_->depth_occupancy_map_options_);

  // 先关闭3d点云
  // ptr_depth_occ_map_->setCameraCalibrateOption(
  //     slam2d_params_ptr_->depth_camera_options_);

  // sensor status params
  node_->get_parameter_or("ros2_params.open_sensor_statu_pub",
                          open_sensor_statu_pub_, false);
  node_->get_parameter_or("ros2_params.sensor_statu_pub_delay",
                          sensor_statu_pub_delay_, 60);
}

void Ros2Adapter::registerDataCallback() {
  if (nullptr == odom_sub_ || nullptr == laser_scan_sub_) {
    LOG(INFO) << "***************register data "
                 "call_back******************";
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10).best_effort(),
        std::bind(&Ros2Adapter::odomCallback, this, std::placeholders::_1));

    if (use_imu_) {
      imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
          imu_topic_, rclcpp::QoS(10).best_effort(),
          std::bind(&Ros2Adapter::imuCallback, this, std::placeholders::_1));
    }

    if (use_depth_camera_) {
      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthCloudFrontUpCallback = std::bind(
              &Ros2Adapter::depthCloudCallback, this, std::placeholders::_1, 1);
      depth_cloud_front_up_sub_ =
          node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              depth_camera_front_up_topic_, rclcpp::QoS(10).best_effort(),
              depthCloudFrontUpCallback);
      // std::cout << " depth_camera_front_up_topic_ = "
      //           << depth_camera_front_up_topic_ << std::endl;

      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthCloudFrontDownCallback = std::bind(
              &Ros2Adapter::depthCloudCallback, this, std::placeholders::_1, 2);
      depth_cloud_front_down_sub_ =
          node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              depth_camera_front_down_topic_, rclcpp::QoS(10).best_effort(),
              depthCloudFrontDownCallback);

      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthStepCloudFrontUpCallback = std::bind(
              &Ros2Adapter::depthCloudCallback, this, std::placeholders::_1, 1);
      depth_step_cloud_front_up_sub_ =
          node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              step_depth_camera_front_up_topic_, rclcpp::QoS(10).best_effort(),
              depthStepCloudFrontUpCallback);

      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthStepCloudFrontDownCallback = std::bind(
              &Ros2Adapter::depthCloudCallback, this, std::placeholders::_1, 2);
      depth_step_cloud_front_down_sub_ =
          node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              step_depth_camera_front_down_topic_,
              rclcpp::QoS(10).best_effort(), depthStepCloudFrontDownCallback);
    }

    laser_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::QoS(10).best_effort(),
        std::bind(&Ros2Adapter::laserScanCallback, this,
                  std::placeholders::_1));
    elevator_status_sub_ =
        node_->create_subscription<chassis_interfaces::msg::TakeLiftAreaStatus>(
            elevator_status_topic_, rclcpp::QoS(10).best_effort(),
            std::bind(&Ros2Adapter::elevatorStatusCallback, this,
                      std::placeholders::_1));
  }
}

void Ros2Adapter::resetDataCallback() {
  //   sub_laser_cloud_.reset();
  //   sub_odom_.reset();
  //   sub_gps_msg_.reset();
  //   sub_relocalization_msg_.reset();
  //   is_first_pose_ = true;
  //   is_first_map_ = true;
  LOG(INFO) << "reset all data callback.";
}

void Ros2Adapter::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  if (ptr_state_machine_->getCurrentState() ==
      slam2d_core::state_machine::AD_STATU::UNKNOW) {
    return;
  }

  if (ptr_state_machine_->getCurrentMode() ==
      slam2d_core::state_machine::AD_MODE::AD_MAPPING) {
    ptr_slam_system_->mappingAddImu(slam2d_core::common::ImuData{
        fromRos(imu_msg->header.stamp), toEigen(imu_msg->linear_acceleration),
        toEigen(imu_msg->angular_velocity)});
  }
}

void Ros2Adapter::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  if (ptr_state_machine_->getCurrentState() ==
      slam2d_core::state_machine::AD_STATU::UNKNOW) {
    return;
  }

  // static int _test_count = 0;
  // _test_count++;
  // if (_test_count > 3000 && _test_count < 6000) {
  //   LOG(INFO) << "_test_count : " << _test_count;
  //   return;
  // }

  last_odom_time_ = std::chrono::system_clock::now();
  obtained_odom_time_ = true;

  if (ptr_state_machine_->getCurrentMode() ==
      slam2d_core::state_machine::AD_MODE::AD_LOCALIZATION) {
    slam2d_core::common::Time time = fromRos(odom_msg->header.stamp);
    ptr_slam_system_->localizationAddOdomData(
        slam2d_core::common::OdometryData{time, toRigid3(odom_msg->pose.pose)});

    if (ptr_slam_system_->isLocalizationInit()) {
      if (ptr_state_machine_->getCurrentState() ==
          slam2d_core::state_machine::AD_STATU::INIT_LOCALIZATION) {
        ptr_state_machine_->sendEvent(
            slam2d_core::state_machine::EventName::INIT_LOCALIZATION_SUCCESS);
      }
      Eigen::Vector3d result_cov;
      slam2d_core::common::Rigid3 result_pose;
      bool loc_status;
      ptr_slam_system_->getLocalizationResult(result_cov, result_pose,
                                              loc_status);
      publishLocalizationResult(odom_msg->header.stamp, result_pose,
                                result_cov);
      // publishOdometryDataUpdate();
    }

  } else if (ptr_state_machine_->getCurrentMode() ==
             slam2d_core::state_machine::AD_MODE::AD_MAPPING) {
    slam2d_core::common::Time time = fromRos(odom_msg->header.stamp);
    ptr_slam_system_->mappingAddOdom(
        slam2d_core::common::OdometryData{time, toRigid3(odom_msg->pose.pose)});

    Eigen::Vector3d result_cov;
    slam2d_core::common::Rigid3 result_pose;
    ptr_slam_system_->getMappingLocalizationResult(time, result_pose);
    publishLocalizationResult(odom_msg->header.stamp, result_pose, result_cov);
  }
}
void Ros2Adapter::laserScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  if (ptr_state_machine_->getCurrentState() ==
      slam2d_core::state_machine::AD_STATU::UNKNOW) {
    return;
  }

  last_laser_time_ = std::chrono::system_clock::now();
  obtained_laser_time_ = true;

  if (ptr_state_machine_->getCurrentMode() ==
      slam2d_core::state_machine::AD_MODE::AD_LOCALIZATION) {
    slam2d_core::amcl::LaserScanData laser_scan_data(scan_msg->ranges.size());
    laser_scan_data.time = fromRos(scan_msg->header.stamp);
    laser_scan_data.range_max = scan_msg->range_max;
    laser_scan_data.range_min = scan_msg->range_min;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++) {
      laser_scan_data.ranges[i] = scan_msg->ranges[i];
      laser_scan_data.ranges_angle[i] =
          scan_msg->angle_min + scan_msg->angle_increment * i;
    }
    ptr_slam_system_->localizationAddScanData(laser_scan_data);

    // 发布粒子集合
    std::vector<slam2d_core::amcl::Sample> set;
    bool pf_set_pub_flag;
    ptr_slam_system_->getCurrentPFSet(set, pf_set_pub_flag);
    if (pf_set_pub_flag && debug_mode_) {
      geometry_msgs::msg::PoseArray cloud_msg;
      cloud_msg.header.stamp = scan_msg->header.stamp;
      cloud_msg.header.frame_id = "map";
      cloud_msg.poses.resize(set.size());
      for (size_t i = 0; i < set.size(); i++) {
        cloud_msg.poses[i].position.x = set[i].pose.translation().x();
        cloud_msg.poses[i].position.y = set[i].pose.translation().y();
        cloud_msg.poses[i].position.z = 0;

        cloud_msg.poses[i].orientation.x = set[i].pose.rotation().x();
        cloud_msg.poses[i].orientation.y = set[i].pose.rotation().y();
        cloud_msg.poses[i].orientation.z = set[i].pose.rotation().z();
        cloud_msg.poses[i].orientation.w = set[i].pose.rotation().w();
      }

      loc_pf_set_pub_->publish(cloud_msg);
    }

    // 发布激光里程计
    slam2d_core::common::OdometryData laser_odom;
    nav_msgs::msg::Odometry msg;
    if (ptr_slam_system_->getLaserOdometryData(laser_odom) && debug_mode_) {
      msg.header.stamp = toRos(laser_odom.time);
      msg.header.frame_id = "odom";
      msg.pose.pose = toGeometryMsgPose(laser_odom.pose);
      laser_odom_pub->publish(msg);
    }

  } else if (ptr_state_machine_->getCurrentMode() ==
             slam2d_core::state_machine::AD_MODE::AD_MAPPING) {
    slam2d_core::common::TimedPointCloudData time_point_cloud;
    sensor_msgs::msg::PointCloud scan_cloud;

    scanToPointCloud(*scan_msg, laser_tf_, time_point_cloud, scan_cloud);

    if (debug_mode_)
      scan_cloud_pub_->publish(scan_cloud);

    ptr_slam_system_->mappingAddScan(time_point_cloud);
  }
}

void Ros2Adapter::depthCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, const int &flag) {
  if (ptr_state_machine_->getCurrentState() ==
      slam2d_core::state_machine::AD_STATU::UNKNOW) {
    return;
  }

  if (ptr_state_machine_->getCurrentMode() ==
      slam2d_core::state_machine::AD_MODE::AD_MAPPING) {
    slam2d_core::common::Time cloud_time = fromRos(cloud_msg->header.stamp);
    int last_node_id;
    slam2d_core::common::Rigid3 last_node_pose;
    bool find_node =
        ptr_slam_system_->getLastTrajectoryNode(last_node_id, last_node_pose);
    if (find_node) {
      slam2d_core::occupancy_map::Mat34d near_pose;
      auto node_trans = last_node_pose.translation();
      auto node_rotate = last_node_pose.rotation();
      node_rotate.normalize();
      near_pose.block<3, 3>(0, 0) = node_rotate.toRotationMatrix();
      near_pose.block<3, 1>(0, 3) = node_trans;

      slam2d_core::common::Rigid3 cur_rigi_pose;
      if (!ptr_slam_system_->getMappingLocalizationResult(cloud_time,
                                                          cur_rigi_pose)) {
        LOG(WARNING) << "getMappingLocalizationResult failed!";
        return;
      }

      auto cur_trans = cur_rigi_pose.translation();
      auto cur_rotate = cur_rigi_pose.rotation();
      slam2d_core::occupancy_map::Mat34d cur_pose;
      cur_rotate.normalize();
      cur_pose.block<3, 3>(0, 0) = cur_rotate.toRotationMatrix();
      cur_pose.block<3, 1>(0, 3) = cur_trans;

      slam2d_core::occupancy_map::laserCloud::Ptr laser_cloud_in(
          new slam2d_core::occupancy_map::laserCloud());
      slam2d_core::occupancy_map::laserCloud::Ptr obstacle_cloud(
          new slam2d_core::occupancy_map::laserCloud());
      pcl::fromROSMsg(*cloud_msg, *laser_cloud_in);

      if (laser_cloud_in->empty()) {
        // LOG(WARNING) << "obstacle cloud in empty!";
        return;
      }

      ptr_depth_occ_map_->transformCloud(laser_cloud_in, obstacle_cloud, flag);

      ptr_depth_occ_map_->insertObstacleCloud(obstacle_cloud, cur_pose,
                                              last_node_id, near_pose);

      // 点云可视化
      // slam2d_core::occupancy_map::laserCloud::Ptr trans_obstacle_cloud =
      //     slam2d_core::occupancy_map::Transformbox::transformPointCloud(
      //         cur_pose, obstacle_cloud);
      // sensor_msgs::msg::PointCloud2 cloudMsgTemp;
      // pcl::toROSMsg(*trans_obstacle_cloud, cloudMsgTemp);
      // rclcpp::Clock::SharedPtr ptr_clock;
      // ptr_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      // cloudMsgTemp.header.stamp = ptr_clock->now();
      // cloudMsgTemp.header.frame_id = "map";
      // obstacle_cloud_pub_->publish(cloudMsgTemp);
    }
  }
}

void Ros2Adapter::elevatorStatusCallback(
    const chassis_interfaces::msg::TakeLiftAreaStatus::SharedPtr
        elevator_status_msg) {
  LOG(INFO) << "-----elevatorStatusCallback working";
  if (ptr_state_machine_->getCurrentState() ==
      slam2d_core::state_machine::AD_STATU::UNKNOW) {
    return;
  }

  if (ptr_state_machine_->getCurrentMode() ==
      slam2d_core::state_machine::AD_MODE::AD_LOCALIZATION) {
    // judge elevator status for params change
    bool is_elevator_params = false;
    if (elevator_status_msg->elevator_state == "HalfElevator" ||
        elevator_status_msg->elevator_state == "InElevator") {
      is_elevator_params = true;
    }
    LOG(INFO) << "-----elevator_status_msg->elevator_state "
              << elevator_status_msg->elevator_state;
    if (nullptr != ptr_slam_system_) {
      ptr_slam_system_->setElevatorStatus(is_elevator_params);
    }
  }
}

void Ros2Adapter::publishLocalizationResult(
    const builtin_interfaces::msg::Time &time,
    const slam2d_core::common::Rigid3 &pose, const Eigen::Vector3d &cov) {
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msgs;
  pose_msgs.header.frame_id = "map";
  pose_msgs.header.stamp = time;

  pose_msgs.pose.pose.position.x = pose.translation().x();
  pose_msgs.pose.pose.position.y = pose.translation().y();

  pose_msgs.pose.pose.orientation.x = pose.rotation().x();
  pose_msgs.pose.pose.orientation.y = pose.rotation().y();
  pose_msgs.pose.pose.orientation.z = pose.rotation().z();
  pose_msgs.pose.pose.orientation.w = pose.rotation().w();

  pose_msgs.pose.covariance[0] = cov[0];
  pose_msgs.pose.covariance[7] = cov[1];
  pose_msgs.pose.covariance[35] = cov[2];

  pub_pose_->publish(pose_msgs);

  if (debug_mode_) {
    tf2::Transform tf;
    tf2::Quaternion q;
    q.setRPY(0, 0, slam2d_core::common::quaternionToYaw(pose.rotation()));
    tf = tf2::Transform(
        q, tf2::Vector3(pose.translation().x(), pose.translation().y(), 0.0));

    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = "map";
    tmp_tf_stamped.child_frame_id = "base_link";
    tmp_tf_stamped.header.stamp = time;

    tmp_tf_stamped.transform = tf2::toMsg(tf);
    ptr_tfb_->sendTransform(tmp_tf_stamped);

    // 发布激光雷达静态tf
    q.setRPY(0, 0, slam2d_core::common::quaternionToYaw(laser_tf_.rotation()));
    tf = tf2::Transform(q, tf2::Vector3(laser_tf_.translation().x(),
                                        laser_tf_.translation().y(), 0.0));
    tmp_tf_stamped.header.frame_id = "base_link";
    tmp_tf_stamped.child_frame_id = "laser";
    tmp_tf_stamped.header.stamp = time;
    tmp_tf_stamped.transform = tf2::toMsg(tf);
    ptr_tfb_->sendTransform(tmp_tf_stamped);

    // obtained laserodom_cloud tf
    if (!is_obtained_transfrom_) {
      is_obtained_transfrom_ = true;

      slam2d_core::common::Rigid3 transfrom_pose;
      ptr_slam_system_->getFirstMapToLaserTransfrom(transfrom_pose);
      q.setRPY(0, 0,
               slam2d_core::common::quaternionToYaw(transfrom_pose.rotation()));
      tf = tf2::Transform(q,
                          tf2::Vector3(transfrom_pose.translation().x(),
                                       transfrom_pose.translation().y(), 0.0));
      tf_laserodom_cloud_ = tf;
    }

    // 发布laserodom_cloud tf
    // tmp_tf_stamped.header.frame_id = "map";
    // tmp_tf_stamped.child_frame_id = "laserodom_cloud";
    // tmp_tf_stamped.header.stamp = time;
    // tmp_tf_stamped.transform = tf2::toMsg(tf_laserodom_cloud_);
    // ptr_tfb_->sendTransform(tmp_tf_stamped);

    // sensor_msgs::msg::PointCloud2 local_map_msg, pre_aligned_cloud_msg;
    // pcl::PointCloud<PointType>::Ptr pre_aligned_laser_cloud;
    // pcl::PointCloud<PointType>::Ptr local_laser_cloud_map;
    // ptr_slam_system_->getLaserOdomCloud(pre_aligned_laser_cloud,
    //                                     local_laser_cloud_map);

    // if (pre_aligned_laser_cloud != nullptr &&
    //     local_laser_cloud_map != nullptr) {
    //   pcl::toROSMsg(*(pre_aligned_laser_cloud), pre_aligned_cloud_msg);
    //   pcl::toROSMsg(*(local_laser_cloud_map), local_map_msg);

    //   // LOG(ERROR) << " publish local map cloud ";
    //   // rclcpp::Time now_time(curr_keyFrame_time * 1e9);
    //   local_map_msg.header.stamp = time;
    //   local_map_msg.header.frame_id = "laserodom_cloud";
    //   pub_local_map_cloud_->publish(local_map_msg);

    //   pre_aligned_cloud_msg.header.stamp = time;
    //   pre_aligned_cloud_msg.header.frame_id = "laserodom_cloud";
    //   pub_pre_aligned_cloud_->publish(pre_aligned_cloud_msg);
    // } else {
    //   LOG(WARNING) << " pre_aligned_laser_cloud != nullptr && "
    //                   "local_laser_cloud_map != nullptr ";
    // }
  }
}  // namespace slam2d_ros2

void Ros2Adapter::publishSensorStatus() {
  auto iter_statu = sensor_status_.find(cur_sensor_statu_);
  if (iter_statu == sensor_status_.end()) {
    publish_sensor_state_count_ = 0;
    return;
  }

  if (0 < publish_sensor_state_count_) {
    --publish_sensor_state_count_;
  } else {
    auto sensor_status = chassis_interfaces::msg::UploadRunLog();
    sensor_status.log_type = "warn";
    sensor_status.log_value = iter_statu->second + delay_real_time_;
    sensor_status.application_name = "localization";
    LOG(WARNING) << "----------------" << sensor_status.log_value
                 << "----------------";
    sensor_state_publisher_->publish(sensor_status);
    publish_sensor_state_count_ = sensor_statu_pub_delay_;
    delay_real_time_ = "";
  }
}

void Ros2Adapter::publishNodeList() {
  visualization_msgs::msg::MarkerArray trajectory_node_list;
  const auto node_poses = ptr_slam_system_->getTrajectoryNodePoses();
  if (0 == node_poses.size()) {
    LOG(WARNING) << "node_poses.size() = 0.";
    return;
  }

  for (const auto &node_id_data : node_poses) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ptr_clock_->now();

    if (node_id_data.first % 30 == 0) {
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.type = visualization_msgs::msg::Marker::CUBE;
    } else {
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
    }

    marker.pose.position.z = 0.1;
    std::stringstream name;
    name << "node " << node_id_data.first;
    marker.ns = name.str();
    marker.id = node_id_data.first;

    const ::geometry_msgs::msg::Point node_point =
        toGeometryMsgPoint(node_id_data.second.global_pose.translation());

    marker.color =
        colorBar(((node_id_data.first / 30) % 10) / static_cast<double>(10.0));
    marker.points.push_back(node_point);
    marker.pose.position.x = node_point.x;
    marker.pose.position.y = node_point.y;
    marker.action = visualization_msgs::msg::Marker::ADD;

    trajectory_node_list.markers.push_back(marker);
  }

  trajectory_node_list_pub_->publish(trajectory_node_list);
}

void Ros2Adapter::publishConstraintList() {
  visualization_msgs::msg::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::msg::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = ptr_clock_->now();
  constraint_intra_marker.header.frame_id = "/map";
  constraint_intra_marker.scale.x = 0.025;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::msg::Marker residual_intra_marker =
      constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses = ptr_slam_system_->getTrajectoryNodePoses();
  if (0 == trajectory_node_poses.size()) {
    LOG(WARNING) << "trajectory_node_poses.size() = 0.";
    return;
  }

  const auto submap_poses = ptr_slam_system_->getAllSubmapPoses();
  if (0 == submap_poses.size()) {
    LOG(WARNING) << "submap_poses.size() = 0.";
    return;
  }

  const auto constraints = ptr_slam_system_->constraints();
  if (0 == constraints.size()) {
    LOG(WARNING) << "constraints.size() = 0.";
    return;
  }

  for (const auto &constraint : constraints) {
    visualization_msgs::msg::Marker *constraint_marker, *residual_marker;
    std_msgs::msg::ColorRGBA color_constraint, color_residual;

    if (constraint.tag == slam2d_core::backend::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      color_constraint = getColor(constraint.submap_id + 25);
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      constraint_marker = &constraint_inter_same_trajectory_marker;
      residual_marker = &residual_inter_same_trajectory_marker;
      // Bright yellow
      color_constraint.a = 1.0;
      color_constraint.r = color_constraint.g = 1.0;
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto submap_it = submap_poses.find(constraint.submap_id);
    if (submap_it == submap_poses.end()) {
      continue;
    }
    const auto &submap_pose = submap_it->second.pose;
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    const auto &trajectory_node_pose = node_it->second.global_pose;
    const slam2d_core::common::Rigid3 constraint_pose =
        submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        toGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        toGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        toGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        toGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  trajectory_constraint_list_pub_->publish(constraint_list);
}

void Ros2Adapter::publishData() {
  size_t loc_map_pub_cnt = 0;
  while (rclcpp::ok()) {
    if (nullptr != ptr_slam_system_) {
      if (slam2d_core::state_machine::AD_MODE::AD_LOCALIZATION ==
              ptr_state_machine_->getCurrentMode() &&
          debug_mode_) {
        if (loc_map_pub_cnt < 3) {
          // 发布一次定位所用的地图
          loc_map_pub_cnt++;
          std::shared_ptr<slam2d_core::amcl::OccupancyGrid> map_ptr =
              ptr_slam_system_->getLocMap();
          if (map_ptr != nullptr) {
            nav_msgs::msg::OccupancyGrid map_msg;
            createOccupancyGridMsg(map_msg, map_ptr, ptr_clock_->now());
            LOG(INFO) << "Published the Localizaiton Map";
            occupancy_grid_publisher_->publish(map_msg);
          }
        }
      } else if (slam2d_core::state_machine::AD_MODE::AD_MAPPING ==
                     ptr_state_machine_->getCurrentMode() &&
                 !ptr_slam_system_->getJudgeSaveMap()) {
        ptr_slam_system_->mapLock();
        publishMap();
        ptr_slam_system_->mapUnLock();

        if (debug_mode_) {
          publishNodeList();
          publishConstraintList();
        }

        loc_map_pub_cnt = 0;
      } else {
        loc_map_pub_cnt = 0;
      }
    }
    std::this_thread::sleep_for(2s);
  }
}

void Ros2Adapter::publishMap() {
  ptr_depth_occ_map_->clearTmpMapWithLock();
  auto start_time = std::chrono::system_clock::now();
  auto paint_submap_slices = ptr_slam_system_->paintSubmapSlices(0.1);
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> ptr_map =
      createOccupancyGridMsg(paint_submap_slices, 0.1, "map",
                             ptr_clock_->now());

  if (debug_mode_)
    occupancy_grid_publisher_->publish(*ptr_map);

  if (use_depth_camera_) {
    if (ptr_map->data.size() <= 0) {
      LOG(ERROR) << "use ptr_map->data.size() <= 0";
      return;
    }

    global_occ_map_.info.width = ptr_map->info.width;
    global_occ_map_.info.height = ptr_map->info.height;
    global_occ_map_.info.resolution = ptr_map->info.resolution;
    global_occ_map_.data.clear();
    global_occ_map_.data.resize(ptr_map->info.width * ptr_map->info.height);
    std::fill(global_occ_map_.data.begin(), global_occ_map_.data.end(), -1);
    global_occ_map_.info.origin(0, 3) = ptr_map->info.origin.position.x;
    global_occ_map_.info.origin(1, 3) = ptr_map->info.origin.position.y;
    for (size_t i = 0; i < ptr_map->data.size(); i++) {
      global_occ_map_.data[i] = ptr_map->data[i];
    }

    if (ptr_depth_occ_map_ != nullptr) {
      const auto node_poses = ptr_slam_system_->getTrajectoryNodePoses();
      if (ptr_depth_occ_map_->updateGlobalMap(node_poses, global_occ_map_)) {
        // ptr_depth_occ_map_->saveGlobalMap();
        ptr_depth_occ_map_->saveGlobalMapYaml();
      }
    }
  }

  if (use_depth_camera_) {
    chassis_interfaces::msg::MappingPng mapping_png;
    slam2d_core::occupancy_map::UserOccupancyGrid globalmap =
        ptr_depth_occ_map_->getGlobalMap();
    if (getPngFromDepthOccupancyGrid(globalmap, mapping_png,
                                     "/tmp/map_png.png")) {
      mapping_png_publisher_->publish(mapping_png);
    }
  } else {
    chassis_interfaces::msg::MappingPng mapping_png;
    nav_msgs::msg::OccupancyGrid *p = ptr_map.get();

    if (getPngFromOccupancyGrid(p, mapping_png, "/tmp/map_png.png")) {
      mapping_png_publisher_->publish(mapping_png);
    }
  }
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> cost_time = end_time - start_time;
  LOG(INFO) << "publish map png times(ms): " << cost_time.count() * 1000;
}

// void Ros2Adapter::publishOdometryDataUpdate() {
//   slam2d_core::common::OdometryData odometry_data_update;
//   nav_msgs::msg::Odometry msg;

//   ptr_slam_system_->getOdometryDataUpdate(odometry_data_update);
//   msg.header.stamp = toRos(odometry_data_update.time);
//   msg.header.frame_id = "odom";
//   msg.pose.pose = toGeometryMsgPose(odometry_data_update.pose);
//   odometry_data_update_publisher_->publish(msg);
// }
void Ros2Adapter::caluateSeneorTime() {
  while (rclcpp::ok()) {
    if (ptr_state_machine_->getCurrentState() ==
        slam2d_core::state_machine::AD_STATU::UNKNOW) {
      std::this_thread::sleep_for(1s);
      continue;
    }

    cur_sensor_statu_ = ptr_slam_system_->getSensorStatus();

    if (obtained_laser_time_ && obtained_odom_time_) {
      auto now_time = std::chrono::system_clock::now();

      std::chrono::duration<double> cost_time_laser =
          now_time - last_laser_time_;
      std::chrono::duration<double> cost_time_odom = now_time - last_odom_time_;

      if (cost_time_laser.count() > 1.0) {
        LOG(WARNING) << " laser time delay more then 1s";
        delay_real_time_ =
            ", 延时时间为 " + std::to_string(cost_time_laser.count()) + "s";
        cur_sensor_statu_ =
            slam2d_core::state_machine::SENSOR_STATUS::LASER_DELAY;
      }

      if (cost_time_odom.count() > 1.0) {
        LOG(WARNING) << " odom time delay more then 1s";
        delay_real_time_ =
            ", 延时时间为 " + std::to_string(cost_time_odom.count()) + "s";
        cur_sensor_statu_ =
            slam2d_core::state_machine::SENSOR_STATUS::ODOM_DELAY;
      }
    }

    if (open_sensor_statu_pub_) {
      publishSensorStatus();
    }
    cur_sensor_statu_ = slam2d_core::state_machine::SENSOR_STATUS::NORMAL;
    ptr_slam_system_->setSensorStatusNormal();

    std::this_thread::sleep_for(1s);
  }
}

}  // namespace slam2d_ros2
