#include "new_lidar_slam_ros2.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "log.hpp"

namespace cvte_lidar_slam {
using namespace std::chrono_literals;

LidarSLAM::LidarSLAM(rclcpp::Node::SharedPtr sub_odom_node,
                     rclcpp::Node::SharedPtr sub_lidar_node,
                     rclcpp::Node::SharedPtr pub_node) {
  sub_odom_node_ = sub_odom_node;
  sub_lidar_node_ = sub_lidar_node;
  pub_node_ = pub_node;

  laser_cloud_in_.reset(new laserCloud());
  global_cloud_.reset(new laserCloud());
  surround_cloud_.reset(new laserCloud());
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  corner_cloud_top_.reset(new laserCloud());
  surf_cloud_top_.reset(new laserCloud());
  frame_pose_.reset(new laserCloud());
  traver_cloud_in_.reset(new laserCloud());
  traver_cloud_obstacle_.reset(new laserCloud());
  traver_cloud_out_.reset(new laserCloud());
  is_first_map_ = true;
  is_first_pose_ = true;
  is_first_odom_ = true;
  is_first_cloud_ = true;
  odom_count_ = 0;
  lidar_count_ = 0;
  imu_count_ = 0;
  slam_params_ptr_ = std::make_unique<LoadParamsRos2>(pub_node_);
  ptr_state_machine_ = SlamStateMachine::getInstance();
  ptr_slam_system_ = System::getInstance();
  ptr_depth_occ_map_ = DepthOccupancyMap::getInstance();

  resetDataCallback();

  init();
}

LidarSLAM::~LidarSLAM() {
  LOG(ERROR) << "SENSOR COUNT " << odom_count_ << ' ' << lidar_count_;

  // pose_result_.close();

  ptr_slam_system_->shutdown();
}

void LidarSLAM::updateParameter() {
  LOG(INFO) << " updata LidarSLAM params for topic name, sensors transfrom and "
               "debug model";
  std::vector<double> t_lo = {1., 0., 0., -0.48, 0., 1.,
                              0., 0., 0., 0.,    1., 0.};
  pub_node_->get_parameter_or("t_lo", t_lo, t_lo);
  T_lo_ << t_lo[0], t_lo[1], t_lo[2], t_lo[3], t_lo[4], t_lo[5], t_lo[6],
      t_lo[7], t_lo[8], t_lo[9], t_lo[10], t_lo[11];
  LOG(INFO) << "T_lo: \n" << T_lo_;

  // 判断出现nan， 使用默认值
  ptr_slam_system_->setExtrincs(T_lo_);

  slam_params_ptr_->getParameter("depth_camera_front_up_topic",
                                 depth_camera_front_up_topic_,
                                 std::string("orbbec_cloud"));
  slam_params_ptr_->getParameter("depth_camera_front_down_topic",
                                 depth_camera_front_down_topic_,
                                 std::string("orbbec_cloud"));
  slam_params_ptr_->getParameter("step_depth_camera_front_up_topic",
                                 step_depth_camera_front_up_topic_,
                                 std::string("orbbec_cloud"));
  slam_params_ptr_->getParameter("step_depth_camera_front_down_topic",
                                 step_depth_camera_front_down_topic_,
                                 std::string("orbbec_cloud"));
  slam_params_ptr_->getParameter("use_depth_camera", use_depth_camera_, false);
  slam_params_ptr_->getParameter("is_debug", is_debug_, false);
  slam_params_ptr_->getParameter("print_debug", print_debug_, false);
  slam_params_ptr_->getParameter("publish_tf", publish_tf_, false);
  slam_params_ptr_->getParameter("publish_cloud", publish_cloud_, false);
  slam_params_ptr_->getParameter("publish_global_cloud", publish_global_cloud_,
                                 false);
  slam_params_ptr_->getParameter("publish_cloud_loc", publish_cloud_loc_,
                                 false);
  slam_params_ptr_->getParameter("publish_global_cloud_loc",
                                 publish_global_cloud_loc_, false);
  slam_params_ptr_->getParameter("publish_path", publish_path_, false);

  slam_params_ptr_->getParameter("open_sensor_statu_pub",
                                 open_sensor_statu_pub_, false);
  slam_params_ptr_->getParameter("sensor_statu_pub_delay",
                                 sensor_statu_pub_delay_, 60);

  slam_params_ptr_->loadSystemConfig();
  ptr_depth_occ_map_->setCameraCalibrateOption(
      slam_params_ptr_->depth_camera_options_);
  ptr_depth_occ_map_->setDepthOccMapOption(
      slam_params_ptr_->depth_occupancy_map_options_);
  ptr_depth_occ_map_->setExtrincs(T_lo_);
}

void LidarSLAM::init() {
  localization_pose_x_ = 0;
  localization_pose_y_ = 0;
  localization_pose_z_ = 0;
  localization_pose_yaw_ = 0;
  // pose_result_.open("./pose_result.txt");
  // pose_result_ << "#format: time_stamp tx ty tz qx qy qz qw" << std::endl;

  updateParameter();

  if (publish_cloud_) {
    pub_surroud_cloud_ =
        pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/surround_map_cloud_", rclcpp::QoS(1).best_effort());
    pub_global_cloud_ =
        pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_map_cloud_", rclcpp::QoS(1).best_effort());
    pub_frame_pose_ =
        pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/frame_pose_cloud_", rclcpp::QoS(1).best_effort());
    pub_corner_cloud_ =
        pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/corner_cloud_", rclcpp::QoS(1).best_effort());
    pub_surf_cloud_ =
        pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/surf_cloud_", rclcpp::QoS(1).best_effort());
    pub_loop_constrain_edge_ =
        pub_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/loop_edge", rclcpp::QoS(1).best_effort());
    mapping_png_publisher_ =
        pub_node_->create_publisher<chassis_interfaces::msg::MappingPng>(
            "/mapping_png", rclcpp::QoS(1).best_effort());
    obstacle_cloud_pub_ =
        pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/obstacle_cloud", rclcpp::QoS(2).best_effort());
    sensor_state_publisher_ =
        pub_node_->create_publisher<chassis_interfaces::msg::UploadRunLog>(
            "state_pub", rclcpp::QoS(1).best_effort());
  }

  if (publish_path_) {
    pub_slam_odom_path_ = pub_node_->create_publisher<nav_msgs::msg::Path>(
        "/slam_odom_path", rclcpp::QoS(1).best_effort());
    pub_wheel_odom_path_ = pub_node_->create_publisher<nav_msgs::msg::Path>(
        "/wheel_odom_path", rclcpp::QoS(1).best_effort());
  }

  if (publish_tf_) {
    ptr_tfl_ = std::make_shared<tf2_ros::TransformBroadcaster>(pub_node_);
    ptr_tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(pub_node_);
    pub_laser_odometry_ = pub_node_->create_publisher<nav_msgs::msg::Odometry>(
        "/laser_odom_to_init", rclcpp::QoS(1).best_effort());
  }

  pub_pose_ =
      pub_node_
          ->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              "/fusion_pose", rclcpp::QoS(2).best_effort());

  lidar_topic_name_ = slam_params_ptr_->ptr_config_->lidar_topic_name;
  wheel_odom_topic_name_ = slam_params_ptr_->ptr_config_->wheel_odom_topic_name;
  imu_topic_name_ = slam_params_ptr_->ptr_config_->imu_topic_name;
  depthcloud_sample_frequent_ =
      slam_params_ptr_->ptr_config_->depthcloud_sample_frequent;
  ptr_config_ = slam_params_ptr_->ptr_config_;

  ptr_slam_system_->setConfig(ptr_config_);
  // add sensor status
  sensor_status_ = {
      {SENSOR_STATUS::ODOM_DELAY, "警告： 里程计数据延时"},
      {SENSOR_STATUS::ODOM_TIME_STAMP_JUMP, "警告： 里程计时间戳跳变"},
      {SENSOR_STATUS::ODOM_POSE_JUMP, "警告： 里程计位姿跳变"},
      {SENSOR_STATUS::LASER_DELAY, "警告： 雷达数据延时"}};

  if (publish_cloud_) {
    pub_cloud_thread_ =
        std::thread(std::bind(&LidarSLAM::publishAllCloud, this));
  }

  pub_map_thread_ = std::thread(std::bind(&LidarSLAM::publishMap, this));

  // add sensor timer moniter thread
  timer_monitor_thread_ =
      std::thread(std::bind(&LidarSLAM::caluateSeneorAbnormalValue, this));

  registerDataCallback();
}

void LidarSLAM::clear() {}

void LidarSLAM::registerDataCallback() {
  if (nullptr == sub_odom_ || nullptr == sub_laser_scan_) {
    LOG(INFO)
        << ".....................register data call_back......................";
    is_first_odom_ = true;
    is_first_cloud_ = true;
    is_first_imu_ = true;
    odom_count_ = 0;
    lidar_count_ = 0;
    imu_count_ = 0;

    sub_laser_scan_ =
        sub_lidar_node_->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_name_, rclcpp::QoS(10).best_effort(),
            std::bind(&LidarSLAM::laserScanDataCallback, this,
                      std::placeholders::_1));

    sub_odom_ = sub_odom_node_->create_subscription<nav_msgs::msg::Odometry>(
        wheel_odom_topic_name_, rclcpp::QoS(10).best_effort(),
        std::bind(&LidarSLAM::odomDataCallback, this, std::placeholders::_1));

    if (ptr_config_->USE_IMU) {
      // sub_imu_ =
      // node_->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name_,
      // std::bind(&LidarSLAM::IMUDataCallback, this, std::placeholders::_1),
      // 1000); sub_imu_ =
      // node_->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name_,
      // std::bind(&LidarSLAM::IMUDataCallback, this), 1000);
    }

    if (use_depth_camera_) {
      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthCloudFrontUpCallback = std::bind(&LidarSLAM::depthCloudCallback,
                                                this, std::placeholders::_1, 1);
      sub_depth_cloud_front_up_ =
          sub_lidar_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              depth_camera_front_up_topic_, rclcpp::QoS(10).best_effort(),
              depthCloudFrontUpCallback);
      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthCloudFrontDownCallback = std::bind(
              &LidarSLAM::depthCloudCallback, this, std::placeholders::_1, 2);
      sub_depth_cloud_front_down_ =
          sub_lidar_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              depth_camera_front_down_topic_, rclcpp::QoS(10).best_effort(),
              depthCloudFrontDownCallback);

      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthStepCloudFrontUpCallback = std::bind(
              &LidarSLAM::depthCloudCallback, this, std::placeholders::_1, 1);
      sub_depth_step_cloud_front_up_ =
          sub_lidar_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              step_depth_camera_front_up_topic_, rclcpp::QoS(10).best_effort(),
              depthStepCloudFrontUpCallback);

      std::function<void(std::shared_ptr<sensor_msgs::msg::PointCloud2>)>
          depthStepCloudFrontDownCallback = std::bind(
              &LidarSLAM::depthCloudCallback, this, std::placeholders::_1, 2);
      sub_depth_step_cloud_front_down_ =
          sub_lidar_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              step_depth_camera_front_down_topic_,
              rclcpp::QoS(10).best_effort(), depthStepCloudFrontDownCallback);
    }
  }
}

void LidarSLAM::resetDataCallback() {
  sub_laser_cloud_.reset();
  sub_laser_scan_.reset();
  sub_odom_.reset();
  sub_imu_.reset();
  sub_gps_msg_.reset();
  sub_relocalization_msg_.reset();
  is_first_pose_ = true;
  is_first_map_ = true;
  is_first_odom_ = true;
  is_first_cloud_ = true;
  odom_count_ = 0;
  lidar_count_ = 0;
  imu_count_ = 0;
  LOG(INFO) << "reset all data callback.";
}

void LidarSLAM::laserScanDataCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr laserScanMsg) {
  static double _first_cloud_time = -1.0;
  static double MINIMUM_RANGE = ptr_config_->occ_map_config.laser_min_range;
  static double MAXMUM_RANGE = ptr_config_->occ_map_config.laser_max_range;
  if (ptr_state_machine_->getCurrentState() == AD_STATU::UNKNOW) {
    return;
  }

  // 上次获取的雷达状态时间，用于之后的延时判断
  last_laser_status_time_ = std::chrono::system_clock::now();
  obtained_laser_time_ = true;

  lidar_count_++;

  double time_stamp_now =
      double(laserScanMsg->header.stamp.sec) +
      double(laserScanMsg->header.stamp.nanosec) * (1.0 * 1e-9);
  if (is_first_cloud_ == true) {
    _first_cloud_time = time_stamp_now;
    is_first_cloud_ = false;
  }

  PointType newPoint;
  newPoint.z = 0.0;
  double newPointAngle = 0.0;

  pcl::PointCloud<PointType>::Ptr lidar_cloud_raw(
      new pcl::PointCloud<PointType>());

  // scan ros数据类型转换到pcl类型下
  int beamNum = laserScanMsg->ranges.size();
  for (int i = 0; i < beamNum; i++) {
    if (std::isnan(laserScanMsg->ranges[i])) {
      continue;
    }
    if (laserScanMsg->ranges[i] < MINIMUM_RANGE ||
        laserScanMsg->ranges[i] > MAXMUM_RANGE) {
      continue;
    }
    newPointAngle = laserScanMsg->angle_min + laserScanMsg->angle_increment * i;
    newPoint.x = laserScanMsg->ranges[i] * std::cos(newPointAngle);
    newPoint.y = laserScanMsg->ranges[i] * std::sin(newPointAngle);
    newPoint.intensity = laserScanMsg->intensities[i];
    lidar_cloud_raw->push_back(newPoint);
  }

  curr_lidar_time_ = time_stamp_now;

  // 添加获取的激光点云
  ptr_slam_system_->addCloudFrame(lidar_cloud_raw, time_stamp_now);

  LOG(INFO) << "new cloud time relative to first:: "
            << (time_stamp_now - _first_cloud_time)
            << ", size: " << lidar_cloud_raw->size();

  last_keyframe_time_ = time_stamp_now;
}

void LidarSLAM::odomDataCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  static double last_odom_time = -1.0;

  if (ptr_state_machine_->getCurrentState() == AD_STATU::UNKNOW) {
    return;
  }

  last_odom_status_time_ = std::chrono::system_clock::now();
  obtained_odom_time_ = true;

  odom_count_++;

  Mat34d curr_raw_odom_pose = fromOdometryMsg(odom_msg);

  double linear = odom_msg->twist.twist.linear.x;
  double angular = odom_msg->twist.twist.angular.z;

  double time_stamp = odom_msg->header.stamp.sec +
                      odom_msg->header.stamp.nanosec * (1.0 * 1e-9);

  if (is_first_odom_) {
    if (odom_count_ < 50) {
      T_wo_0_ = curr_raw_odom_pose;
      Eigen::Vector3d rpy = Mathbox::rotation2rpy(T_wo_0_.block<3, 3>(0, 0));
      rpy.x() = 0.0;
      rpy.y() = 0.0;
      Mat3d rotation_g = Mathbox::rpyToRotationMatrix(rpy);
      // 只将xyz位置与yaw角转换到０点，保留里程计原始姿态，因为已经与重力对齐；
      T_wo_0_.block<3, 3>(0, 0) = rotation_g;

      last_raw_odom_pose_ = curr_raw_odom_pose;
      last_odom_pose_ = curr_raw_odom_pose;
      first_odom_time_ = time_stamp;
      last_odom_time = time_stamp;
      is_first_odom_ = false;
    }
  }

  // 判断里程计是否出现异常值
  bool time_gap_flag = false;
  // TODO:
  // 查看里程计代码中时间戳是根据什么原则打的；是否会存在连续时间发布topic的情况；
  double delta_time = std::abs(time_stamp - last_odom_time);
  if (delta_time < 0.02) {
    delta_time = 0.02;  // 50hz=0.02s;
  }

  if (delta_time > 0.1)  // 0.1
  {
    time_gap_flag = true;
    LOG(ERROR) << "big delta time ocurr for odom : d_t: "
               << (time_stamp - last_odom_time) << " s";
  }
  cur_sensor_jump_ = SENSOR_STATUS::NORMAL;
  if (delta_time > 1.0) {
    cur_sensor_jump_ = SENSOR_STATUS::ODOM_TIME_STAMP_JUMP;
    odom_time_jump_ = ", 跳变时间为 " + std::to_string(delta_time) + "秒";
  }

  Mat34d delta_odom =
      Mathbox::deltaPose34d(last_raw_odom_pose_, curr_raw_odom_pose);
  double delta_rot =
      Mathbox::rotationMatrixToEulerAngles(delta_odom.block<3, 3>(0, 0)).norm();
  double delta_dis = delta_odom.block<3, 1>(0, 3).norm();
  if (delta_dis > 0.04 || delta_rot > 0.04) {
    LOG(ERROR) << "big delta pose ocurr for odom: d_dis: " << delta_dis
               << ", d_rot: " << delta_rot;

    double linear_temp = delta_dis / delta_time;
    double angular_temp = delta_rot / delta_time;
    // 只要瞬时速度异常，或者运动增量太大，则进行异常值剔除;
    if (linear_temp > 2.0 || angular_temp > 2.0 || delta_dis > 2.0 ||
        delta_rot > 2.0) {
      delta_odom = last_delta_odom_pose_;
      LOG(ERROR) << "delta odom abnormal, using last_delta_odom_pose_: \n"
                 << last_delta_odom_pose_;
    }
  }

  if (delta_dis >= 0.1 || delta_rot >= 0.1) {
    cur_sensor_jump_ = SENSOR_STATUS::ODOM_POSE_JUMP;
    odom_pose_jump_ = ", 跳变位移为 " + std::to_string(delta_dis) + "米, " +
                      ", 跳变角度为 " + std::to_string(delta_rot * 57.3) + "度";
  }

  Mat34d new_odom_pose = Mathbox::multiplePose34d(last_odom_pose_, delta_odom);

  bool PLANE_MODE = true;
  if (PLANE_MODE == true) {
    Vec3d p_predict = new_odom_pose.block<3, 1>(0, 3);
    Mat3d r_predict = new_odom_pose.block<3, 3>(0, 0);
    Vec3d rpy_predict = Mathbox::rotation2rpy(r_predict);

    p_predict.z() = 0.0;
    rpy_predict.x() = 0.0;
    rpy_predict.y() = 0.0;
    r_predict = Mathbox::rpyToRotationMatrix(rpy_predict);

    new_odom_pose.block<3, 1>(0, 3) = p_predict;
    new_odom_pose.block<3, 3>(0, 0) = r_predict;
  }

  curr_odom_time_ = time_stamp;

  // 将里程计坐标系下的里程计结果通过 外参 转化到激光雷达坐标系；
  // 地图坐标系以激光雷达为原点
  T_wo_ = transformOdom(new_odom_pose);

  // 添加当前帧里程计结果，并利用map_to_odom_和最新的里程计结果对机器人当前状态进行预测；
  ptr_slam_system_->addOdom(OdomMeasure(T_wo_, linear, angular, time_stamp));

  LOG(INFO) << "new odom time relative to first: "
            << (time_stamp - first_odom_time_);

  last_odom_pose_ = new_odom_pose;
  last_raw_odom_pose_ = curr_raw_odom_pose;
  last_delta_odom_pose_ = delta_odom;
  last_odom_time = time_stamp;

  bool lidar_in_time_flag = true;
  if (curr_odom_time_ - curr_lidar_time_ > 0.5) {
    LOG(INFO) << "odomDataCallback: lidar data too delay: "
              << (curr_odom_time_ - curr_lidar_time_);
    lidar_in_time_flag = false;
  }

  if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_LOCALIZATION) {
    // 定位模式下，如果初始定位成功的话，则开始发布fusion_pose;
    if (ptr_slam_system_->isLocalizationInit() && lidar_in_time_flag) {
      if (ptr_state_machine_->getCurrentState() ==
          AD_STATU::INIT_LOCALIZATION) {
        ptr_state_machine_->sendEvent(EventName::INIT_LOCALIZATION_SUCCESS);
      }
      RobotState robot_state =
          ptr_slam_system_->getRobotState();  // 当前预测结果；
      Mat34d predict_cur_frame_pose = robot_state.pose;
      double cov = ptr_slam_system_->getPoseCov();

      if (publish_tf_) {
        publishTf(predict_cur_frame_pose, time_stamp);
      }

      publishLocalizationResult(predict_cur_frame_pose, time_stamp, cov);

      if (is_debug_) {
        localization_pose_x_ = robot_state.pose(0, 3);
        localization_pose_y_ = robot_state.pose(1, 3);
        localization_pose_z_ = robot_state.pose(2, 3);
        Vec3d angle = Mathbox::rotationMatrixToEulerAngles(
            robot_state.pose.block<3, 3>(0, 0));
        localization_pose_yaw_ = angle(2);
        Eigen::Quaterniond q(robot_state.pose.block<3, 3>(0, 0));
        // pose_result_ << std::setprecision(15) << time_stamp << ' '
        //              << localization_pose_x_ << ' ' << localization_pose_y_
        //              << ' ' << localization_pose_z_ << ' ' << q.x() << ' '
        //              << q.y() << ' ' << q.z() << ' ' << q.w() << std::endl;
      }
    }
  } else if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING) {
    if (lidar_in_time_flag) {
      RobotState robot_state =
          ptr_slam_system_->getRobotState();  // 当前预测结果；
      Mat34d predict_cur_frame_pose = robot_state.pose;
      double cov = ptr_slam_system_->getPoseCov();

      if (publish_tf_) {
        publishTf(predict_cur_frame_pose, time_stamp);
      }
      publishLocalizationResult(robot_state.pose, time_stamp, cov);
      if (is_debug_) {
        localization_pose_x_ = robot_state.pose(0, 3);
        localization_pose_y_ = robot_state.pose(1, 3);
        localization_pose_z_ = robot_state.pose(2, 3);
        Vec3d angle = Mathbox::rotationMatrixToEulerAngles(
            robot_state.pose.block<3, 3>(0, 0));
        localization_pose_yaw_ = angle(2);
        Eigen::Quaterniond q(robot_state.pose.block<3, 3>(0, 0));
        // pose_result_ << std::setprecision(15) << time_stamp << ' '
        //              << localization_pose_x_ << ' ' << localization_pose_y_
        //              << ' ' << localization_pose_z_ << ' ' << q.x() << ' '
        //              << q.y() << ' ' << q.z() << ' ' << q.w() << std::endl;
      }
    }
  } else {
    localization_pose_x_ = 0;
    localization_pose_y_ = 0;
    localization_pose_z_ = 0;
    localization_pose_yaw_ = 0;
  }
}

void LidarSLAM::IMUDataCallback(const sensor_msgs::msg::Imu::SharedPtr ImuMsg) {
  static double G = 1.0;
  static Eigen::Vector3d _acc;
  static Eigen::Vector3d _gyr;
  if (ptr_state_machine_->getCurrentState() == AD_STATU::UNKNOW) {
    return;
  }

  imu_count_++;

  // 丢弃前1秒的数据；
  if (imu_count_ < 100) {
    if (print_debug_) {
      printf("--IMUDataCallback: throw first 100 imu data: %d \n",
             (int) imu_count_);
    }
    return;
  }

  double time_stamp =
      ImuMsg->header.stamp.sec + ImuMsg->header.stamp.nanosec * (1.0 * 1e-9);
  _acc = Eigen::Vector3d(ImuMsg->linear_acceleration.x,
                         ImuMsg->linear_acceleration.y,
                         ImuMsg->linear_acceleration.z) *
         G;
  _gyr = Eigen::Vector3d(ImuMsg->angular_velocity.x, ImuMsg->angular_velocity.y,
                         ImuMsg->angular_velocity.z);
  ptr_slam_system_->addIMU(ImuMeasure(_acc, _gyr, imu_count_, time_stamp));
}

void LidarSLAM::depthCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, const int &flag) {
  if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_UNKNOW) {
    return;
  }

  if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING) {
    // 判断点云是否为空
    if (0 == cloud_msg->data.size()) {
      return;
    }

    // 将不为空的点云进行降频率
    if (depth_downsample_count_ <
        static_cast<size_t>(1.0 / depthcloud_sample_frequent_)) {
      depth_downsample_count_++;
      return;
    } else {
      depth_downsample_count_ = 0;
    }

    double cloud_time = double(cloud_msg->header.stamp.sec) +
                        double(cloud_msg->header.stamp.nanosec) * (1.0 * 1e-9);
    Mat34d near_pose;
    long unsigned int last_node_id = ptr_slam_system_->getLatestFrameID();
    bool find_node = ptr_slam_system_->getPoseFromID(last_node_id, near_pose);
    Mat34d base_near_pose = Mathbox::multiplePose34d(near_pose, T_lo_);
    if (find_node) {
      Mat34d cur_pose;
      auto robot_state = ptr_slam_system_->getRobotState();
      if (fabs(robot_state.time_stamp - cloud_time) > 0.06) {
        LOG(WARNING) << "depth time stamp too fast"
                     << robot_state.time_stamp - cloud_time;
        return;
      }

      cur_pose = robot_state.pose;
      Mat34d base_cur_pose = Mathbox::multiplePose34d(cur_pose, T_lo_);
      depthCloud::Ptr laser_cloud_in(new depthCloud());
      depthCloud::Ptr obstacle_cloud(new depthCloud());
      pcl::fromROSMsg(*cloud_msg, *laser_cloud_in);

      if (laser_cloud_in->empty()) {
        return;
      }
      // 需要对点云进行降采样
      LOG(INFO) << " before down sample " << laser_cloud_in->size();
      depthcloudDownSample(laser_cloud_in);
      LOG(INFO) << " after down sample " << laser_cloud_in->size();

      ptr_depth_occ_map_->transformCloud(laser_cloud_in, obstacle_cloud, flag);
      ptr_depth_occ_map_->insertObstacleCloud(obstacle_cloud, base_cur_pose,
                                              last_node_id, base_near_pose);

      // 点云可视化
      // depthCloud::Ptr trans_obstacle_cloud =
      //   Transformbox::transformDepthCloud(cur_pose, obstacle_cloud);
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

void LidarSLAM::publishMap() {
  while (rclcpp::ok()) {
    if (nullptr != ptr_slam_system_) {
      if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode() &&
          is_debug_) {
      } else if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
        if (!ptr_config_->use_depth_cloud) {
          UserOccupancyGrid realtime_global_map =
              ptr_slam_system_->getCurrOccMap();
          chassis_interfaces::msg::MappingPng realtime_mapping_png;
          if (getMappingPngMsg(realtime_global_map, realtime_mapping_png,
                               "/tmp/map_png.png")) {
            mapping_png_publisher_->publish(realtime_mapping_png);
          }
        } else {
          // 打印发布地图需要的时间
          auto start_time = std::chrono::system_clock::now();
          UserOccupancyGrid realtime_global_map =
              ptr_slam_system_->getCurrDepthOccMap();
          chassis_interfaces::msg::MappingPng realtime_mapping_png;
          if (getMappingPngMsg(realtime_global_map, realtime_mapping_png,
                               "/tmp/map_png.png")) {
            LOG(WARNING) << "publish map";
            mapping_png_publisher_->publish(realtime_mapping_png);
          }
          auto end_time = std::chrono::system_clock::now();
          std::chrono::duration<double> cost_time = end_time - start_time;
          LOG(WARNING) << "publish map cost time: " << cost_time.count();
          // publish_map_cost_all_time_ += cost_time.count();
          // LOG(WARNING) << "publish maps " << ++map_numbers_
          //              << " publish map cost all time "
          //              << publish_map_cost_all_time_ << " average time "
          //              << publish_map_cost_all_time_ / map_numbers_;
        }
      } else {
      }
    }
    std::this_thread::sleep_for(6s);
  }
}

void LidarSLAM::publishAllCloud() {
  while (rclcpp::ok()) {
    if (ptr_slam_system_ == nullptr) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (is_debug_) {
      static double _last_keyFrame_time = 0.0;

      if (ptr_slam_system_->hasKeyFrame() == false) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      double curr_keyFrame_time = ptr_slam_system_->getCurKeyFrameTimeStamp();
      if (std::abs(_last_keyFrame_time - curr_keyFrame_time) < 0.001) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      _last_keyFrame_time = curr_keyFrame_time;

      rclcpp::Time now_time(curr_keyFrame_time * 1e9);

      // Mat34d curr_key_pose = ptr_slam_system_->getKeyFramePose();
      Mat34d curr_lidar_odom_pose =
          ptr_slam_system_->getKeyFrameLidarOdomPose();

      TicToc pubMap_cost;

      if (publish_path_ &&
          ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING) {
        publishPath(curr_lidar_odom_pose, T_wo_, curr_keyFrame_time);
      }

      if (publish_cloud_loc_ ||
          (publish_cloud_ &&
           ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING)) {
        sensor_msgs::msg::PointCloud2 cornerMsg, surfMsg, cornerTopMsg,
            surfTopMsg, localMapMsg, keyPoseMsg;
        pcl::toROSMsg(*(ptr_slam_system_->corner_cloud_), cornerMsg);
        pcl::toROSMsg(*(ptr_slam_system_->surf_cloud_), surfMsg);

        cornerMsg.header.stamp = now_time;
        cornerMsg.header.frame_id = "/laser";
        pub_corner_cloud_->publish(cornerMsg);

        surfMsg.header.stamp = now_time;
        surfMsg.header.frame_id = "/laser";
        pub_surf_cloud_->publish(surfMsg);

        /////

        surround_cloud_->clear();
        surround_cloud_ = ptr_slam_system_->getCurrSurroundCloud();
        pcl::toROSMsg(*surround_cloud_, localMapMsg);
        localMapMsg.header.stamp = now_time;
        localMapMsg.header.frame_id = "/map";
        pub_surroud_cloud_->publish(localMapMsg);

        frame_pose_->clear();
        frame_pose_ = ptr_slam_system_->getCurrKeyPoses();
        pcl::toROSMsg(*frame_pose_, keyPoseMsg);
        keyPoseMsg.header.stamp = now_time;
        keyPoseMsg.header.frame_id = "/map";
        pub_frame_pose_->publish(keyPoseMsg);
      }

      if (publish_global_cloud_loc_ ||
          (publish_global_cloud_ &&
           ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING)) {
        laserCloud::Ptr global_cloud, global_pose;
        if (ptr_slam_system_->getViewCloud(global_cloud, global_pose)) {
          sensor_msgs::msg::PointCloud2 global_cloud_msg;
          pcl::toROSMsg(*global_cloud, global_cloud_msg);
          global_cloud_msg.header.stamp = now_time;
          global_cloud_msg.header.frame_id = "/map";
          pub_global_cloud_->publish(global_cloud_msg);
        }
      }

      if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING) {
        std::shared_ptr<std::list<LoopConstrain>> loop_info =
            ptr_slam_system_->getLoopInfo();
        if (!loop_info->empty() && curr_loop_pos_.size() != loop_info->size()) {
          curr_loop_pos_.clear();
          for (auto loop_constrain_iter = loop_info->begin();
               loop_constrain_iter != loop_info->end(); loop_constrain_iter++) {
            Mat34d T_curr = loop_constrain_iter->ptr_keyframe2->getPose();
            Mat34d T_loop = loop_constrain_iter->ptr_keyframe1->getPose();

            PointType curr_pos, loop_pos;
            curr_pos.x = T_curr(0, 3);
            curr_pos.y = T_curr(1, 3);
            curr_pos.z = T_curr(2, 3);
            loop_pos.x = T_loop(0, 3);
            loop_pos.y = T_loop(1, 3);
            loop_pos.z = T_loop(2, 3);
            curr_loop_pos_.push_back(std::make_pair(curr_pos, loop_pos));
          }
          visualizeLoopConstraintEdge(curr_keyFrame_time);
        }
      }

      if (frame_pose_->size() < 5) {
        slam_odom_path_.poses.clear();
        wheel_odom_path_.poses.clear();
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}

void LidarSLAM::publishPath(const Mat34d &slam_pose, const Mat34d &wheel_pose,
                            double time) {
  static int _path_count = 0;

  rclcpp::Time now_time(time * 1e9);

  _path_count++;

  if (_path_count > 7200) {
    _path_count = 0;
    slam_odom_path_.poses.clear();
    wheel_odom_path_.poses.clear();
  }

  geometry_msgs::msg::Pose laser_pose = toGeometryMsg(slam_pose);

  geometry_msgs::msg::PoseStamped slamPose;
  slamPose.pose.position = laser_pose.position;
  slamPose.pose.orientation = laser_pose.orientation;
  slam_odom_path_.header.stamp = now_time;
  slam_odom_path_.header.frame_id = "map";
  slam_odom_path_.poses.push_back(slamPose);
  pub_slam_odom_path_->publish(slam_odom_path_);

  geometry_msgs::msg::PoseStamped odomPose;
  odomPose.pose = toGeometryMsg(wheel_pose);
  wheel_odom_path_.header.stamp = now_time;
  wheel_odom_path_.header.frame_id = "map";
  wheel_odom_path_.poses.push_back(odomPose);
  pub_wheel_odom_path_->publish(wheel_odom_path_);
}

void LidarSLAM::publishTf(const Mat34d &pose, double time) {
  rclcpp::Time now_time(time * 1e9);

  geometry_msgs::msg::Pose laser_pose = toGeometryMsg(pose);

  nav_msgs::msg::Odometry laserOdometry;
  laserOdometry.header.frame_id = "map";
  laserOdometry.child_frame_id = "laser";
  laserOdometry.header.stamp = now_time;
  laserOdometry.pose.pose = laser_pose;
  pub_laser_odometry_->publish(laserOdometry);

  geometry_msgs::msg::TransformStamped tmp_tf_stamped;
  Eigen::Quaterniond geoQuat(pose.block<3, 3>(0, 0));
  tf2::Transform tf_map_laser;
  tf2::Quaternion q(geoQuat.x(), geoQuat.y(), geoQuat.z(), geoQuat.w());
  tf_map_laser =
      tf2::Transform(q, tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  tmp_tf_stamped.header.frame_id = "map";
  tmp_tf_stamped.child_frame_id = "laser";
  tmp_tf_stamped.header.stamp = now_time;
  tmp_tf_stamped.transform = tf2::toMsg(tf_map_laser);
  ptr_tfl_->sendTransform(tmp_tf_stamped);

  static tf2::Quaternion q_lo(0.0, 0.0, 0.0, 1.0);
  static tf2::Transform tf_laser_baselink(
      q_lo, tf2::Vector3(T_lo_(0, 3), T_lo_(1, 3), T_lo_(2, 3)));
  tmp_tf_stamped.header.frame_id = "laser";
  tmp_tf_stamped.child_frame_id = "base_link";
  tmp_tf_stamped.header.stamp = now_time;
  tmp_tf_stamped.transform = tf2::toMsg(tf_laser_baselink);
  ptr_tfb_->sendTransform(tmp_tf_stamped);
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

void LidarSLAM::visualizeLoopConstraintEdge(double time) {
  visualization_msgs::msg::MarkerArray markerArray;
  rclcpp::Time now_time(time * 1e9);

  // loop nodes
  visualization_msgs::msg::Marker markerNode;
  markerNode.header.frame_id = "map";
  markerNode.header.stamp = now_time;
  markerNode.action = visualization_msgs::msg::Marker::ADD;
  markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  markerNode.ns = "loop_nodes";
  markerNode.id = 0;
  markerNode.pose.orientation.w = 1;
  markerNode.scale.x = 0.2;
  markerNode.scale.y = 0.2;
  markerNode.scale.z = 0.2;
  markerNode.color.r = 0;
  markerNode.color.g = 0.8;
  markerNode.color.b = 1;
  markerNode.color.a = 1;

  // loop edges
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = "map";
  markerEdge.header.stamp = now_time;
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.ns = "loop_edges";
  markerEdge.id = 1;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.1;
  markerEdge.scale.y = 0.1;
  markerEdge.scale.z = 0.1;
  markerEdge.color.r = 0.9;
  markerEdge.color.g = 0.9;
  markerEdge.color.b = 0;
  markerEdge.color.a = 1;

  for (auto it = curr_loop_pos_.begin(); it != curr_loop_pos_.end(); ++it) {
    PointType curr_pos = it->first;
    PointType loop_pos = it->second;

    geometry_msgs::msg::Point p;
    p.x = curr_pos.x;
    p.y = curr_pos.y;
    p.z = curr_pos.z;
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
    p.x = loop_pos.x;
    p.y = loop_pos.y;
    p.z = loop_pos.z;
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
  }

  markerArray.markers.push_back(markerNode);
  markerArray.markers.push_back(markerEdge);

  pub_loop_constrain_edge_->publish(markerArray);
}

bool LidarSLAM::testReLocalization() {
  std::string data_dir_path = "/home/suyun/code_test/catkin_ws_fast_lio";
  if (access((data_dir_path + "/3d_map/keyframe_pos.pcd").c_str(), 0) != 0) {
    LOG(ERROR) << data_dir_path << "  is not exist";
    return false;
  }

  LOG(WARNING) << " data path: " << data_dir_path;

  if (!ptr_slam_system_->loadMap(data_dir_path + "/3d_map/")) {
    LOG(ERROR) << "loading map failed.";
    return false;
  }

  // TODO:开始重定位
  INIT_MODEL init_model;
  Mat34d init_predict_pose = Mathbox::Identity34();
  LOG(WARNING) << "set init model: fix_pose" << std::endl;
  init_model = INIT_MODEL::FIX_POSE;
  ptr_slam_system_->setInitModel(init_model);
  ptr_slam_system_->setInitPose(init_predict_pose);

  return true;
}

void LidarSLAM::base64Encode(const std::string &input, std::string &output) {
  //编码表
  const char encode_table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  //返回值
  size_t data_byte = input.size();
  const char *data = input.data();
  std::string str_encode;
  unsigned char Tmp[4] = {0};
  int line_length = 0;
  for (int i = 0; i < (int) (data_byte / 3); i++) {
    Tmp[1] = *data++;
    Tmp[2] = *data++;
    Tmp[3] = *data++;
    str_encode += encode_table[Tmp[1] >> 2];
    str_encode += encode_table[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
    str_encode += encode_table[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
    str_encode += encode_table[Tmp[3] & 0x3F];
    if (line_length += 4, line_length == 76) {
      str_encode += "\r\n";
      line_length = 0;
    }
  }

  //对剩余数据进行编码
  int mod = data_byte % 3;
  if (mod == 1) {
    Tmp[1] = *data++;
    str_encode += encode_table[(Tmp[1] & 0xFC) >> 2];
    str_encode += encode_table[((Tmp[1] & 0x03) << 4)];
    str_encode += "==";
  } else if (mod == 2) {
    Tmp[1] = *data++;
    Tmp[2] = *data++;
    str_encode += encode_table[(Tmp[1] & 0xFC) >> 2];
    str_encode += encode_table[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
    str_encode += encode_table[((Tmp[2] & 0x0F) << 2)];
    str_encode += "=";
  }

  output = str_encode;
}

bool LidarSLAM::getMappingPngMsg(
    const UserOccupancyGrid &globalmap,
    chassis_interfaces::msg::MappingPng &mapping_png,
    const std::string &map_temp_path) {
  const int threshold_free = 49;
  const int threshold_occupied = 55;

  if (0 == globalmap.data.size()) {
    LOG(ERROR) << "globalmap is empty";
    return false;
  }

  unsigned int x, y, i;
  cv::Mat png_grey_mat =
      cv::Mat::zeros(globalmap.info.height, globalmap.info.width,
                     CV_8UC1);  // 8位无符号灰度图像

  for (y = 0; y < globalmap.info.height; ++y) {
    for (x = 0; x < globalmap.info.width; ++x) {
      i = x + (globalmap.info.height - y - 1) * globalmap.info.width;
      if (globalmap.data[i] >= 0 && globalmap.data[i] <= threshold_free) {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 254;
      } else if (globalmap.data[i] >= threshold_occupied) {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 0;
      } else {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 205;
      }
    }
  }

  if (png_grey_mat.rows <= 0) {
    LOG(ERROR) << "image_rgb_png row <= 0";
    return false;
  }

  cv::imwrite(map_temp_path, png_grey_mat);

  std::ifstream image_fstream;
  std::stringstream map_png_ss;
  image_fstream.open(map_temp_path, std::ios::in | std::ios::binary);
  if (!image_fstream.is_open()) {
    LOG(ERROR) << " open file" << map_temp_path << " fail!";
    boost::filesystem::remove(map_temp_path);
    return false;
  }
  map_png_ss << image_fstream.rdbuf();
  image_fstream.close();
  Pose2d laser_pose = Mathbox::Mat34d2Pose2d(globalmap.info.origin);

  std::string base64_str;
  base64Encode(map_png_ss.str(), base64_str);
  mapping_png.width = globalmap.info.width;
  mapping_png.height = globalmap.info.height;
  mapping_png.origin_x = laser_pose.x();
  mapping_png.origin_y = laser_pose.y();
  mapping_png.resolution = globalmap.info.resolution;
  mapping_png.png_image = base64_str;
  mapping_png.file_size = base64_str.size();

  return true;
}

void LidarSLAM::publishSensorStatus() {
  auto iter_statu = sensor_status_.find(cur_sensor_statu_);
  if (iter_statu == sensor_status_.end()) {
    publish_sensor_state_count_ = 0;
    return;
  }

  if (last_sensor_statu_ != cur_sensor_statu_) {
    publish_sensor_state_count_ = 0;
  }
  std::string status_info = "";
  if (0 < publish_sensor_state_count_) {
    --publish_sensor_state_count_;
  } else {
    auto sensor_status = chassis_interfaces::msg::UploadRunLog();
    sensor_status.log_type = "warn";
    sensor_status.application_name = "localization";

    if (iter_statu->first == SENSOR_STATUS::ODOM_DELAY) {
      sensor_status.log_value = iter_statu->second + delay_real_time_;
      delay_real_time_ = "";
    } else if (iter_statu->first == SENSOR_STATUS::ODOM_TIME_STAMP_JUMP) {
      status_info = odom_time_jump_;
      sensor_status.log_value = iter_statu->second + status_info;
    } else if (iter_statu->first == SENSOR_STATUS::ODOM_POSE_JUMP) {
      status_info = odom_pose_jump_;
      sensor_status.log_value = iter_statu->second + status_info;
    } else {
      sensor_status.log_value = iter_statu->second;
    }

    LOG(WARNING) << "----------------" << sensor_status.log_value
                 << "----------------"
                 << "jump" << status_info;
    sensor_state_publisher_->publish(sensor_status);
    publish_sensor_state_count_ = sensor_statu_pub_delay_;
  }

  last_sensor_statu_ = cur_sensor_statu_;
}

void LidarSLAM::caluateSeneorAbnormalValue() {
  while (rclcpp::ok()) {
    if (ptr_state_machine_->getCurrentState() == AD_STATU::UNKNOW) {
      std::this_thread::sleep_for(1s);
      continue;
    }
    cur_sensor_statu_ = cur_sensor_jump_;
    if (obtained_laser_time_ && obtained_odom_time_) {
      auto now_time = std::chrono::system_clock::now();

      std::chrono::duration<double> cost_time_laser =
          now_time - last_laser_status_time_;
      std::chrono::duration<double> cost_time_odom =
          now_time - last_odom_status_time_;

      if (cost_time_laser.count() > 1.0) {
        LOG(WARNING) << " laser time delay more then 1s";
        delay_real_time_ =
            ", 延时时间为 " + std::to_string(cost_time_laser.count()) + "s";
        cur_sensor_statu_ = SENSOR_STATUS::LASER_DELAY;
      }

      if (cost_time_odom.count() > 1.0) {
        LOG(WARNING) << " odom time delay more then 1s";
        delay_real_time_ =
            ", 延时时间为 " + std::to_string(cost_time_odom.count()) + "s";
        cur_sensor_statu_ = SENSOR_STATUS::ODOM_DELAY;
      }
    }

    if (open_sensor_statu_pub_) {
      publishSensorStatus();
    }
    cur_sensor_statu_ = SENSOR_STATUS::NORMAL;

    std::this_thread::sleep_for(1s);
  }
}

}  // namespace cvte_lidar_slam