/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file navigation_arch_adapter_ros2.cpp
 *
 *@brief
 * 架构相关接口的实现文件
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev
 *@data 2020-01-08
 ************************************************************************/
#include "arch_adapter/navigation_arch_adapter_ros2.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <eigen3/Eigen/Core>

#include "arch_adapter/navigation_params_ros2.hpp"
#include "controller_ultis/move_command.hpp"
#include "logic_controller.hpp"
#include "malloc.h"
#include "manager_ros2/map_manager_ros2.hpp"
#include "narrow_recognition.hpp"
#include "navigation_mediator.hpp"
#include "navigation_params.hpp"
#include "path_tools/path_optimize.hpp"
#include "path_tools/path_tools.hpp"
#include "planner_utils.hpp"
#include "processor_utils.hpp"
// #include "speed_controller.hpp"
#include <Eigen/Geometry>

#include "speed_decision_base.hpp"

// 底盘控制灯协议定义的转向灯值
#define LEFT_LIGHT 0
#define RIGHT_LIGHT 1
#define FORWARD_LIGHT 2

namespace CVTE_BABOT {
using namespace std::chrono_literals;

NavigationArchAdapter::NavigationArchAdapter() {
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>("navigation", options);
  node_service_ = std::make_shared<rclcpp::Node>("navigation_service", options);

  node_scan_ = std::make_shared<rclcpp::Node>("nav_scan_sub", options);
  node_odom_ = std::make_shared<rclcpp::Node>("nav_odom_sub", options);
  node_point_cloud_ = std::make_shared<rclcpp::Node>("nav_point_cloud_sub",
                                                     options);  //???作用是啥

  node_infrared_ = std::make_shared<rclcpp::Node>("nav_infrared_sub", options);
  node_pose_ = std::make_shared<rclcpp::Node>("nav_pose_sub", options);

  map_area_types_[AreaType::clean_area] = "clean";
  map_area_types_[AreaType::slope_area] = "slope";
  map_area_types_[AreaType::elevator_area] = "elevator";
  map_area_types_[AreaType::carpet_area] = "carpet";
  map_area_types_[AreaType::narrow_area] = "narrow";
  map_area_types_[AreaType::black_area] = "black";
  map_area_types_[AreaType::prohibited_area] = "prohibited";
  map_area_types_[AreaType::none] = "none";
  cur_area_type_ = AreaType::clean_area;
  last_area_type_ = AreaType::clean_area;
  // node_planner_ = std::make_shared<rclcpp::Node>("navigation_planner",
  // options); node_controller_ =
  // std::make_shared<rclcpp::Node>("navigation_controller", options);

  node_->get_parameter_or("use_default_path", use_default_path_, false);
  LOG(INFO) << use_default_path_;
  node_->get_parameter_or("default_path_file", default_path_file_,
                          std::string(""));
  node_->get_parameter_or("pose_topic", s_pose_topic_,
                          std::string("fusion_pose"));
  node_->get_parameter_or("control_topic", s_control_topic_,
                          std::string("navi_vel"));
  node_->get_parameter_or("edge_front_topic", s_edge_front_topic_,
                          std::string(""));
  node_->get_parameter_or("edge_back_topic", s_edge_back_topic_,
                          std::string(""));
  node_->get_parameter_or("debug_mode", is_debug_mode_, true);
  node_->get_parameter_or("use_simulator_env", use_simulator_env_, false);
  node_->get_parameter_or("is_abs_reach", is_abs_reach_, true);
  node_->get_parameter_or("planner_frequency", planner_frequency_, 1.0);
  node_->get_parameter_or("controller_frequency", controller_frequency_, 1.0);
  node_->get_parameter_or("robot_tpye", robot_tpye_, std::string("security"));
  // double x_offset;
  node_->get_parameter_or("pose_offset", x_offset_, 1.0);
  // Pose2d offset_pose(x_offset, 0, 0);
  // RobotInfo::getPtrInstance()->setOffsetPose(offset_pose);
  LOG(INFO) << "debug_mode: " << is_debug_mode_
            << " is_abs_reach: " << is_abs_reach_ << " "
            << " planner_frequency: " << planner_frequency_
            << " controller_frequency: " << controller_frequency_;

  LOG(INFO) << "Robot type: " << robot_tpye_;
  if ("clean_c5" == robot_tpye_) {
    // ptr_logic_controller_->setRobotType(ROBOTTYPE::KAVA_CLEAN_C5);
    RobotInfo::getPtrInstance()->setRobotType(ROBOTTYPE::KAVA_CLEAN_C5);
  } else if ("clean_c3" == robot_tpye_) {
    RobotInfo::getPtrInstance()->setRobotType(ROBOTTYPE::KAVA_CLEAN_C3);
  } else if ("security" == robot_tpye_) {
    // ptr_logic_controller_->setRobotType(ROBOTTYPE::KAVA_SECURITY);
    RobotInfo::getPtrInstance()->setRobotType(ROBOTTYPE::KAVA_SECURITY);
  } else {
    RobotInfo::getPtrInstance()->setRobotType(ROBOTTYPE::KAVA_SECURITY);
    // ptr_logic_controller_->setRobotType(ROBOTTYPE::KAVA_SECURITY);
  }

  ptr_navigation_params_ = std::make_shared<NavigationParameters>();
  navigation_params_ros2_ =
      std::make_shared<NavigationParamsRos2>(node_, ptr_navigation_params_);
  SpeedDecision_Factor::createSeepDecision(node_, controller_frequency_);

  ptr_speed_decision_ = SpeedDecisionBase::getInstance();

  // ptr_range_sensors_handle_ = std::make_shared<RangeSensorsRos2>(node_);
  // ptr_range_sensors_handle_->init();

  // //ptr_range_sensors_handle_->setSonarUpdateFunc(
  //     std::bind(&SpeedDecisionBase::inputSonar, ptr_speed_decision_,
  //               std::placeholders::_1));

  // //ptr_range_sensors_handle_->setIRUpdateFunc(
  //     std::bind(&SpeedDecisionBase::inputInfrared, ptr_speed_decision_,
  //               std::placeholders::_1));

  updateParameter();

  mission_manager_server_ =
      node_service_->create_service<mission_manager_msgs::srv::MissionManager>(
          "navigation_mission_manager",
          std::bind(&NavigationArchAdapter::missionManagerCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));

  // zmq server
  params_.getParameter("zmq_service.zmq_server_addr", zmq_server_addr_,
                       std::string("127.0.0.1"));
  params_.getParameter("zmq_service.navigation_server_port",
                       navigation_server_port_, (unsigned short) 9889);
  LOG(INFO) << "zmq_server_addr_: " << zmq_server_addr_
            << ", navigation_server_port_: " << navigation_server_port_;
  zmq_server_ =
      std::make_shared<ZmqService>(zmq_server_addr_, navigation_server_port_);
  zmq_server_->setCallback(std::bind(
      &NavigationArchAdapter::missionManagerCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  zmq_server_->startHandleCommand();

  localization_sub_ =
      node_pose_
          ->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              s_pose_topic_, rclcpp::QoS(0).best_effort(),
              std::bind(&NavigationArchAdapter::localizationDataCallback, this,
                        std::placeholders::_1));

  ptr_costmap_arch_ = CostmapArchAdapter::getPtrInstance();
  ptr_costmap_arch_->updateParameter();
  ptr_costmap_arch_->systemInit();
  ptr_costmap_arch_->startCostmapTimer();

  ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  NavigationMediator::getPtrInstance()->setNavigationParameters(
      ptr_navigation_params_);

  // 注册标记地图(用于标记窄道等信息)
  std::shared_ptr<Costmap2d> marked_map = nullptr;
  NavigationMediator::getPtrInstance()
      ->registerDataType<std::shared_ptr<Costmap2d>>("marked_map", marked_map);

  ptr_optimize_path_ = std::make_shared<PathOptimize>();

  ptr_logic_controller_ =
      std::make_shared<LogicController>(ptr_costmap_arch_->getCostmap());

  ptr_clean_decision_ = std::make_shared<CleanDecision>();
  ptr_pnc_map_ = std::make_shared<PncMap>();

  // 判断代价地图是否通过配置文件加载静态地图
  std::string map_path;
  if (ptr_costmap_arch_->existInitStaticMap(map_path)) {
    resetStaticCostmap(map_path, "",
                       "88888888");  // 该部分主要是调试用，给个任意md5值
  }
  stop_thread_ = false;
  planner_thread_ = nullptr;
  controller_thread_ = nullptr;
  hdmap_thread_ = nullptr;
  global_replan_thread_ = nullptr;
  update_costmap_thread_ = nullptr;
}

NavigationArchAdapter::~NavigationArchAdapter() {
  stop_thread_ = true;
  if (planner_thread_ != nullptr && planner_thread_->joinable()) {
    planner_thread_->join();
  }
  if (controller_thread_ != nullptr && controller_thread_->joinable()) {
    controller_thread_->join();
  }
  if (hdmap_thread_ != nullptr && hdmap_thread_->joinable()) {
    hdmap_thread_->join();
  }
  if (global_replan_thread_ != nullptr && global_replan_thread_->joinable()) {
    global_replan_thread_->join();
  }
  if (update_costmap_thread_ != nullptr && update_costmap_thread_->joinable()) {
    update_costmap_thread_->join();
  }
}

bool NavigationArchAdapter::startNavigation() {
  if (getNaviState() != NavigationState::UINIT) {
    LOG(WARNING) << "navigation already started, can not restart." << std::endl;
    return true;
  }
  std::lock_guard<std::mutex> lock_hdmap(hd_map_mutex_);
  // ptr_pnc_map_->clear();
  if (is_debug_mode_) {
    if (goal_sub_ == nullptr) {
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", rclcpp::QoS(0).best_effort(),
          std::bind(&NavigationArchAdapter::goalPoseCallback, this,
                    std::placeholders::_1));
    }
  }
  if (odom_sub_ == nullptr) {
    odom_sub_ = node_odom_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(2).best_effort(),
        std::bind(&NavigationArchAdapter::OdomDataCallback, this,
                  std::placeholders::_1));
  }
  if (laser_scan_sub_ == nullptr) {
    laser_scan_sub_ =
        node_scan_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::QoS(2).best_effort(),
            std::bind(&NavigationArchAdapter::scanCallback, this,
                      std::placeholders::_1));
  }
  if (psd_sub_front_ == nullptr && !s_edge_front_topic_.empty()) {
    LOG(WARNING) << "s_edge_front_topic_ " << s_edge_front_topic_;
    psd_sub_front_ = node_->create_subscription<sensor_msgs::msg::Range>(
        s_edge_front_topic_, rclcpp::QoS(2).best_effort(),
        std::bind(&NavigationArchAdapter::psdCallbackFront, this,
                  std::placeholders::_1));
  }
  if (psd_sub_back_ == nullptr && !s_edge_back_topic_.empty()) {
    psd_sub_back_ = node_->create_subscription<sensor_msgs::msg::Range>(
        s_edge_back_topic_, rclcpp::QoS(2).best_effort(),
        std::bind(&NavigationArchAdapter::psdCallbackBack, this,
                  std::placeholders::_1));
  }
  if (camera_cloud_sub_ == nullptr) {
    camera_cloud_sub_ = node_point_cloud_->create_subscription<
        sensor_msgs::msg::PointCloud2>(
        "obstacle_cloud_front_down", rclcpp::QoS(2).best_effort(),
        std::bind(
            &NavigationArchAdapter::cameraCloudCallback, this,
            std::placeholders::
                _1));  // std::placeholders::_1占位符，用于代替回调函数中的第一个参数

    if (infrared_sub_ == nullptr) {
      infrared_sub_ =
          node_infrared_->create_subscription<sensor_msgs::msg::Range>(
              "infrared_obstacle", rclcpp::QoS(0).best_effort(),
              std::bind(&NavigationArchAdapter::infraredObstacleCallback, this,
                        std::placeholders::_1));
    }
  }
  if (update_path_sub_ == nullptr) {
    update_path_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/navi_manager/nav_path/update", rclcpp::QoS(2).best_effort(),
        std::bind(&NavigationArchAdapter::pubPathCallback, this,
                  std::placeholders::_1));
  }
  if (cmd_vel_pub_ == nullptr) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        s_control_topic_, rclcpp::QoS(0).best_effort());
  }
  if (remain_path_pub_ == nullptr) {
    remain_path_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
        "remain_path_length", rclcpp::QoS(0).best_effort());
  }
  if (is_debug_mode_) {
    if (local_path_pub_ == nullptr) {
      local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
          std::string("local_path"), rclcpp::QoS(0).best_effort());
    }
    if (global_path_pub_ == nullptr) {
      global_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
          std::string("global_path"), rclcpp::QoS(0).best_effort());
    }
    if (mission_plan_pub_ == nullptr) {
      mission_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
          std::string("mission_path"), rclcpp::QoS(0).best_effort());
    }
    if (global_pose_pub_ == nullptr) {
      global_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
          std::string("global_path_pose"), rclcpp::QoS(0).best_effort());
    }
    if (local_pose_pub_ == nullptr) {
      local_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
          std::string("local_path_pose"), rclcpp::QoS(0).best_effort());
    }
    if (ori_local_pose_pub_ == nullptr) {
      ori_local_pose_pub_ =
          node_->create_publisher<geometry_msgs::msg::PoseArray>(
              std::string("original_local_path_pose"),
              rclcpp::QoS(10).best_effort());
    }
    if (carrot_pub_ == nullptr) {
      carrot_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
          std::string("pid_carrot"), rclcpp::QoS(0).best_effort());
    }
    if (area_pub_ == nullptr) {
      area_pub_ = node_->create_publisher<std_msgs::msg::String>(
          std::string("area"), rclcpp::QoS(0).best_effort());
    }
  }
  if (planner_thread_ == nullptr) {
    planner_thread_ = new std::thread(
        std::bind(&NavigationArchAdapter::plannerTimerCallback, this));
  }
  if (controller_thread_ == nullptr) {
    controller_thread_ = new std::thread(
        std::bind(&NavigationArchAdapter::controllerTimerCallback, this));
  }
  if (hdmap_thread_ == nullptr) {
    hdmap_thread_ = new std::thread(
        std::bind(&NavigationArchAdapter::hdmapTimerCallback, this));
  }
  if (global_replan_thread_ == nullptr) {
    global_replan_thread_ = new std::thread(
        std::bind(&NavigationArchAdapter::globalReplanTimerCallback, this));
  }
  // if (update_costmap_thread_ == nullptr) {
  //   update_costmap_thread_ = new std::thread(
  //       std::bind(&NavigationArchAdapter::updateCostMapTimerCallback, this));
  // }
  setNaviState(NavigationState::IDLE);
  setErrorCode(NavigationErrorCode::NO);
  LOG(INFO) << "succeed start navigation.";
  return true;
}

bool NavigationArchAdapter::updateParameter() {
  navigation_params_ros2_->loadParams();
  return true;
}

bool NavigationArchAdapter::stopNavigation() {
  if (getNaviState() == NavigationState::UINIT) {
    LOG(WARNING) << "navigation is uninit,can not restop.";
    return true;
  }
  // MutilArea mutil_area;
  // Area area;
  // area.push_back(Eigen::Vector2d(-16.6, 0.11));
  // area.push_back(Eigen::Vector2d(-7.33, -0.235));
  // area.push_back(Eigen::Vector2d(-7.42, -7.44));
  // area.push_back(Eigen::Vector2d(-16.8, -7.19));
  // mutil_area.push_back(area);
  // SubPath path;
  // ptr_clean_decision_->AdditionalCoverPlan(
  //     "/home/hubery/Development/robot_data/map/"
  //     "d36284bf-fe8e-4e03-892e-2af96d5a1a8d/edit_map.yaml",
  //     mutil_area, path);
  // ptr_clean_decision_->saveAlreadyCleanMap("/home/hubery/Development/clean");
  // ptr_clean_decision_->reset();
  // planner_timer_.reset();
  // controller_timer_.reset();
  // odom_sub_.reset();
  // goal_sub_.reset();
  // laser_scan_sub_.reset();
  // psd_sub_.reset();
  // psd_sub_front_.reset();
  // psd_sub_back_.reset();
  // camera_cloud_sub_.reset();

  brakeRobot();  // 退出前先刹车
  std::lock_guard<std::mutex> lock_hdmap(hd_map_mutex_);
  ptr_pnc_map_->clear();

  // mission_plan_pub_.reset();
  // global_path_pub_.reset();
  // local_path_pub_.reset();
  // global_pose_pub_.reset();
  // local_pose_pub_.reset();
  // ori_local_pose_pub_.reset();
  // carrot_pub_.reset();
  // cmd_vel_pub_.reset();

  setNaviState(NavigationState::UINIT);
  setErrorCode(NavigationErrorCode::NO);
  // cur_area_type_ = AreaType::clean_area;
  // last_area_type_ = AreaType::clean_area;
  LOG(INFO) << "switch navi status";
  LOG(INFO) << "Stop Navi Finish.";
  malloc_trim(0);
  return true;
}

bool NavigationArchAdapter::systemInit() {
  return true;
}

void NavigationArchAdapter::spin() {
  auto node_costmap = ptr_costmap_arch_->getNodeHander();
  // auto node_timer_costmap = ptr_costmap_arch_->getTimerNodeHander();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_);
  exec.add_node(node_costmap);
  exec.add_node(node_service_);
  exec.add_node(node_scan_);
  exec.add_node(node_odom_);
  exec.add_node(node_point_cloud_);  //<增加节点?
  exec.add_node(node_infrared_);
  exec.add_node(node_pose_);

  exec.spin();
  exec.remove_node(node_);
  exec.remove_node(node_costmap);
  exec.remove_node(node_service_);
  exec.remove_node(node_scan_);
  exec.remove_node(node_odom_);
  exec.remove_node(node_point_cloud_);
  exec.remove_node(node_infrared_);
  exec.remove_node(node_pose_);
}

void NavigationArchAdapter::localizationDataCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        localization_msg) {
  Pose2d pose;
  ptr_costmap_arch_->setLocalizationData(localization_msg);
  pose.x = localization_msg->pose.pose.position.x;
  pose.y = localization_msg->pose.pose.position.y;

  AngleCalculate::Quaternion q;
  q.x = localization_msg->pose.pose.orientation.x;
  q.y = localization_msg->pose.pose.orientation.y;
  q.z = localization_msg->pose.pose.orientation.z;
  q.w = localization_msg->pose.pose.orientation.w;
  pose.yaw = AngleCalculate::quaternionToYaw(q);

  if (getNaviState() == NavigationState::UINIT) {
    return;
  }
  ptr_logic_controller_->setCurrentPose(pose);
  ptr_clean_decision_->updateAlreadyCleanArea(
      Eigen::Vector3d(pose.getX(), pose.getY(), pose.getYaw()));
  // SpeedController::getPtrInstance()->calcLocalizaitonSpeedLevel(
  //     localization_msg->pose.covariance[0]);
  updateLocalizationTime();

  if (getNaviState() == NavigationState::ERROR &&
      getErrorCode() == NavigationErrorCode::LOCALIZATION_TIMEOUT) {
    setNaviState(NavigationState::RUNNING);
    setErrorCode(NavigationErrorCode::NO);
  }
}

void NavigationArchAdapter::goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg) {
  if (!is_debug_mode_) {
    LOG(INFO) << "not in debug mode";
    return;
  }
  setNaviState(NavigationState::IDLE);
  Pose2d pose;
  pose.x = goal_msg->pose.position.x;
  pose.y = goal_msg->pose.position.y;

  AngleCalculate::Quaternion q;
  q.x = goal_msg->pose.orientation.x;
  q.y = goal_msg->pose.orientation.y;
  q.z = goal_msg->pose.orientation.z;
  q.w = goal_msg->pose.orientation.w;
  pose.yaw = AngleCalculate::quaternionToYaw(q);

  if (getNaviState() == NavigationState::UINIT) {
    return;
  }
  if (ptr_logic_controller_ == nullptr) {
    return;
  }
  // 调试车辆避让用, 发送车辆所在的位置触发避让,
  // 如果path_follow没有作相应配置, 这个函数不会产生什么影响
  CostmapMediator::getPtrInstance()->setDynamicCar({pose.x, pose.y});
  SubPath global_path = ptr_logic_controller_->setGoalPose(pose);
  std::vector<Eigen::Vector3d> kino_path;
  if (global_path.wps.empty()) {
    LOG(ERROR) << "set goal pose failed.global path is empty." << std::endl;
    return;
  }
  global_path_ = global_path.wps;
  updateOdomTime();
  updateLocalizationTime();

  publishPath(global_path.wps, global_path_pub_);
  publishPoseArray(global_path.wps, global_pose_pub_);

  setNaviState(NavigationState::RUNNING);
  setErrorCode(NavigationErrorCode::NO);
}

void NavigationArchAdapter::OdomDataCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
  Eigen::Quaterniond q(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  ptr_speed_decision_->inputAttitude(q.toRotationMatrix());
  Velocity velocity;
  velocity.d_x = odom_msg->twist.twist.linear.x;
  velocity.d_y = odom_msg->twist.twist.linear.y;
  velocity.d_yaw = odom_msg->twist.twist.angular.z;

  if (getNaviState() == NavigationState::UINIT) {
    return;
  }
  ptr_logic_controller_->setCurrentVelocity(velocity);

  updateOdomTime();
  if (getNaviState() == NavigationState::ERROR &&
      getErrorCode() == NavigationErrorCode::ODOM_TIMEOUT) {
    setNaviState(NavigationState::RUNNING);
    setErrorCode(NavigationErrorCode::NO);
  }
}

void NavigationArchAdapter::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
  std::vector<Pose2d> scan_data;
  std::vector<Eigen::Vector2d> scan_data_eigen;
  double angle_inc = scan_msg->angle_increment;
  double begin_angle = scan_msg->angle_min;
  for (size_t index = 0; index < scan_msg->ranges.size(); index++) {
    if (scan_msg->ranges[index] > 0.001 && scan_msg->ranges[index] < 100) {
      Pose2d data;
      double angle = index * angle_inc + begin_angle;
      data.setX(scan_msg->ranges[index] * cos(angle));
      data.setY(scan_msg->ranges[index] * sin(angle));
      scan_data.push_back(data);
      scan_data_eigen.push_back(Eigen::Vector2d(data.getX(), data.getY()));
    }
  }
  RobotInfo::getPtrInstance()->inputScan(scan_data);
  ptr_speed_decision_->inputScan(scan_data_eigen);
  // 根据雷达帧更新情况进入 updateVel
  ptr_speed_decision_->updateVel();
}

void NavigationArchAdapter::psdCallback(
    const std_msgs::msg::UInt32MultiArray::SharedPtr arrary) {
  std::vector<float> cache;
  // AD
  cache.push_back(arrary->data.at(0));
  cache.push_back(arrary->data.at(2));

  // distance
  cache.push_back(((float) arrary->data.at(1)) / 1000.0);
  cache.push_back(((float) arrary->data.at(3)) / 1000.0);

  RobotInfo::getPtrInstance()->inputSPDValue(cache);
}
void NavigationArchAdapter::psdCallbackFront(
    const sensor_msgs::msg::Range::SharedPtr psd) {
  double range = psd->range;
  if (range < psd->min_range) {
    range = psd->max_range;
  }
  if (range > psd->max_range) {
    range = psd->max_range;
  }
  RobotInfo::getPtrInstance()->inputSPDValue(0, range);
}
void NavigationArchAdapter::psdCallbackBack(
    const sensor_msgs::msg::Range::SharedPtr psd) {
  double range = psd->range;
  if (range < psd->min_range) {
    range = psd->max_range;
  }
  if (range > psd->max_range) {
    range = psd->max_range;
  }
  RobotInfo::getPtrInstance()->inputSPDValue(1, range);
}

void NavigationArchAdapter::cameraCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>());

  std::vector<Pose2d> traget_cloud_pose2d;

  // 转化为pcl格式
  pcl::fromROSMsg(*point_cloud, *laser_cloud_in);

  if (!laser_cloud_in->points.empty()) {
    for (const auto &obj : laser_cloud_in->points) {
      // 根据相机的外参变化  *这个位置省略计算
      Pose2d point(obj.z, obj.x, 0);
      if (point.getX() > 0.01) {
        // 0 为 无效值
        traget_cloud_pose2d.push_back(point);
      }
    }
  }

  RobotInfo::getPtrInstance()->inputCameraCloud(traget_cloud_pose2d);
}

void NavigationArchAdapter::pubPathCallback(
    const std_msgs::msg::Empty::SharedPtr update) {
  LOG(INFO) << "pub global path";
  publishPath(global_path_, global_path_pub_);
}

void NavigationArchAdapter::infraredObstacleCallback(
    const sensor_msgs::msg::Range::SharedPtr infrared_range) {
  //...数据转换，注意这里有多个红外，如何进行循环遍历
  //....
  std::string frame_id = infrared_range->header.frame_id;
  if (frame_id == "") {
    LOG(INFO) << "Infrared Obstacle error : frame_id not set";
  }
  RobotInfo::getPtrInstance()->inputInfraredRange(
      frame_id, double(infrared_range->range));
}

bool NavigationArchAdapter::resetHdmap(const std::string &map_path) {
  auto ptr_cvte_hdmap = cvte::hdmap::CvteHdMap::getInstance();
  return ptr_cvte_hdmap->LoadMapFromFile(map_path);
}

bool NavigationArchAdapter::resetStaticCostmap(const std::string &map_path,
                                               const std::string &osm_path,
                                               const std::string &md5) {
  // ptr_costmap_arch_->rangeTopicReceiveSet(4, true);
  // ptr_range_sensors_handle_->rangeIntputSet(4, true);
  // ptr_costmap_arch_->activeLayer(LayerName::negative_obstacle_layer);
  // ptr_costmap_arch_->activeLayer(LayerName::probabiltiy_voxel_layer);
  if (map_path.empty()) {
    LOG(ERROR) << "reset static costmap name is empty.";
    return false;
  }
  if (md5 != cur_static_map_md5_) {
    //有新的地图时清空旧的补扫地图，再设置新的地图
    cur_static_map_md5_ = md5;
    ptr_clean_decision_->reset();
    ptr_clean_decision_->setCleanMap(map_path);
  }
  if (resetHdmap(osm_path)) {
    CostmapMediator::getPtrInstance()->setClearPolygons(
        std::vector<std::vector<WorldmapPoint>>());
    LOG(ERROR) << "reset osm map failed";
  } else {
    std::vector<cvte::hdmap::Polygon2d> clear_area =
        ptr_pnc_map_->GetClearArea();
    std::vector<cvte::hdmap::Polygon2d> prohibited_area =
        ptr_pnc_map_->GetProhibitedArea();
    std::vector<std::vector<WorldmapPoint>> clear_polygons;
    for (const auto &it : clear_area) {
      std::vector<cvte::hdmap::Vec2d> points = it.points();
      std::vector<WorldmapPoint> worldmap_points;
      worldmap_points.reserve(points.size());
      for (const auto &pt : points) {
        worldmap_points.push_back(WorldmapPoint(pt(0), pt(1)));
      }
      clear_polygons.push_back(worldmap_points);
    }
    CostmapMediator::getPtrInstance()->setClearPolygons(clear_polygons);
    std::vector<std::vector<WorldmapPoint>> prohibited_polygons;
    for (const auto &it : prohibited_area) {
      std::vector<cvte::hdmap::Vec2d> points = it.points();
      std::vector<WorldmapPoint> worldmap_points;
      worldmap_points.reserve(points.size());
      for (const auto &pt : points) {
        worldmap_points.push_back(WorldmapPoint(pt(0), pt(1)));
      }
      prohibited_polygons.push_back(worldmap_points);
    }
    CostmapMediator::getPtrInstance()->setProhibitedPolygons(
        prohibited_polygons);
    LOG(INFO) << "reset osm map succeed";
  }
  // 重置代价地图的静态地图(其中对静态地图进行了膨胀)
  if (ptr_costmap_arch_->resetStaticMap(map_path)) {
    LOG(INFO) << "reset static map: " << map_path << " md5 " << md5
              << std::endl;

    // 更新逻辑控制器里面的代价地图
    ptr_logic_controller_->updateCostMap(ptr_costmap_arch_->getCostmap());

    // 更新逻辑控制器里面的全局代价地图
    ptr_logic_controller_->updatePlannerGlobalCostMap();

    return true;
  } else {
    LOG(ERROR) << "reset static map failed. mapname: " << map_path << std::endl;
    return false;
  }
}

void NavigationArchAdapter::missionManagerPauseNavi() {
  setNaviState(NavigationState::PAUSE);
  setErrorCode(NavigationErrorCode::NO);
  brakeRobot();
}

void NavigationArchAdapter::brakeRobot(bool repeat_flag) {
  // 刹车函数，若重复repeat_flag标志为true向底盘下发几次零速度（几次原因是避免通信丢失没收到）
  // 重复repeat_flag标志为false时只发送一次零速度，默认为true
  // 复位转向灯
  int repeat_time = 0;
  if (!repeat_flag) {
    repeat_time = 1;  // 无需重复发送零速度
  } else {
    repeat_time = 5;
  }
  Velocity velocity;
  velocity.d_x = 0.0;
  velocity.d_y = 0.0;
  velocity.d_yaw = 0.0;
  std_msgs::msg::UInt8 direction_data;
  direction_data.data = FORWARD_LIGHT;

  while (repeat_time-- > 0) { publishCmdVel(velocity); }
  LOG(INFO) << "Brake Robot!!";
}

void NavigationArchAdapter::missionManagerCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
        request,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
        response) {
  Json::Value json_rev_data;
  Json::Value json_ack_data;

  LOG(INFO) << "navigation receive data:\n" << request->send_data << std::endl;

  if (!stringToJson(request->send_data, json_rev_data)) {
    json_ack_data["error_msgs"] = "navigation rev_data is illegal!!";
    LOG(ERROR) << "navigation rev_data is illegal!!";
    json_ack_data["succeed"] = false;
    response->ack_data = json_ack_data.toStyledString();
    return;
  }

  std::string cmd = json_rev_data["cmd"].asString();

  LOG(INFO) << "cmd : " << cmd << std::endl;
  LOG(INFO) << request->send_data;
  if ("setPath" == cmd) {
    std::lock_guard<std::mutex> lock_hdmap(hd_map_mutex_);
    ptr_pnc_map_->clear();
    global_path_.clear();
    missionManagerSetGoalPath(json_rev_data, json_ack_data);
    LOG(INFO) << "setPath finish";
  }

  if ("setFullPathPoints" == cmd) {
    ptr_pnc_map_->clear();
    global_path_.clear();
    missionManagerSetFullGoalPath(json_rev_data, json_ack_data);
    LOG(INFO) << "setFullPathPoints finish";
  }

  if ("setDetectCarResult" == cmd) {
    missionManagerSetCarResult(json_rev_data, json_ack_data);
    LOG(INFO) << "setDetectCar finish";
  }

  if ("setNaviGoal" == cmd) {
    std::lock_guard<std::mutex> lock_hdmap(hd_map_mutex_);
    ptr_pnc_map_->clear();
    global_path_.clear();
    missionManagerSetGoalPose(json_rev_data, json_ack_data);
    LOG(INFO) << "setNaviGoal finish";
  }

  if ("makePlan" == cmd) {
    missionManagerMakePlan(json_rev_data, json_ack_data);
    LOG(INFO) << "mission make plan finish.";
  }

  if ("setAddCoverPlan" == cmd) {
    std::lock_guard<std::mutex> lock_hdmap(hd_map_mutex_);
    ptr_pnc_map_->clear();
    global_path_.clear();
    missionManagerAdditionalClean(json_rev_data, json_ack_data);
    LOG(INFO) << "mission set additional clean finish";
  }

  if ("cancleAddCoverPlan" == cmd) {
    missionManagerClearClean(json_rev_data, json_ack_data);
    LOG(INFO) << "mission cancle additional clean finish";
  }

  if ("pauseNavigation" == cmd) {
    missionManagerPauseNavi();
    json_ack_data["succeed"] = true;
    LOG(INFO) << "pauseNaviGoal finish";
  }

  if ("resumeNavigation" == cmd) {
    setNaviState(NavigationState::RUNNING);
    setErrorCode(NavigationErrorCode::NO);
    json_ack_data["succeed"] = true;
    LOG(INFO) << "resumeNaviGoal finish";
  }

  if ("stopNavigation" == cmd) {
    if (stopNavigation()) {
      json_ack_data["succeed"] = true;
    } else {
      LOG(ERROR) << "stopNavigation fail!!";
      json_ack_data["error_msgs"] = "stopNavigation fail!!";
      json_ack_data["succeed"] = false;
    }
  }

  if ("startNavigation" == cmd) {
    if (startNavigation()) {
      json_ack_data["succeed"] = true;
    } else {
      LOG(ERROR) << "startNavigation fail!!";
      json_ack_data["error_msgs"] = "startNavigation fail!!";
      json_ack_data["succeed"] = false;
    }
  }

  if ("getCurrentState" == cmd) {
    missionManagerGetCurrentState(json_rev_data, json_ack_data);
    LOG(INFO) << "getCurrentState finish";
  }

  if ("setSpeedLevel" == cmd) {
#if 0
    int level = json_rev_data["speed_level"].asInt();
    if (level < 1 || level > 4) {
      json_ack_data["error_msgs"] = "Unsupport Level, input 0 ~ 4.";
      json_ack_data["succeed"] = false;
    } else {
      // SpeedController::getPtrInstance()->calcMissionMangerSpeedLevel(level);
      ptr_speed_decision_->setMissionManagerSpeedRadio(level);
      json_ack_data["succeed"] = true;
    }
#endif
    double value = json_rev_data["speed_value"].asDouble();
    LOG(INFO) << "receive set speed srv, set speed to " << value;
    if (value <= 0.0) {
      LOG(WARNING) << "set speed value is negative " << value
                   << ", force to 0.1";
      value = 0.1;
    } else if (value >= 1.0) {
      // 当前只支持最大速度1.0，更大的速度未测试过
      LOG(WARNING) << "set speed value is " << value
                   << " greater than 1.0, force to 1.0";
      value = 1.0;
    }
    ptr_speed_decision_->setMissionManagerSpeedValue(value);
    json_ack_data["succeed"] = true;
  }

  if ("setStripMark" == cmd) {
    setStripMarker(json_rev_data, json_ack_data);
  }

  if ("getPathEstimateTime" == cmd) {
    missionManagerGetPathEstimateTime(json_rev_data, json_ack_data);
  }

  response->ack_data = json_ack_data.toStyledString();
  LOG(INFO) << "response->ack_data: " << response->ack_data;
}
void NavigationArchAdapter::missionManagerGetPathEstimateTime(
    const Json::Value &rev_data, Json::Value &ack_data) {
  std::string path_name = rev_data["path_file_name"].asString();

  SubPath path;
  // 调用时间计算函数
  // TODO: 修改未不需要地图的
  // double sec = ptr_logic_controller_->getPathEstimateTime(path);
  double sec = 100.0;
  ack_data["time"] = sec;
  ack_data["succeed"] = true;
}

void NavigationArchAdapter::missionManagerSetGoalPose(
    const Json::Value &rev_data, Json::Value &ack_data) {
  if (getNaviState() == NavigationState::UINIT) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "haven't start navigation yet";
    return;
  }
  if (getNaviState() == NavigationState::RUNNING ||
      getNaviState() == NavigationState::PAUSE) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "navigation running now";
    return;
  }

  ptr_clean_decision_->setMissionType(ActionType::NAVIGAOL);
  double x = rev_data["x"].asDouble();
  double y = rev_data["y"].asDouble();
  double yaw = rev_data["yaw"].asDouble();
  Pose2d target_pose(x, y, yaw);
  std::string map_name = rev_data["map_file"].asString();
  std::string map_md5 = rev_data["map_md5"].asString();

  LOG(INFO) << "NaviGoal: (" << x << ", " << y << ", " << yaw
            << ") at map: " << map_name;
  std::string osm_map = "";
  if (rev_data.isMember("osm_path")) {
    osm_map = rev_data["osm_path"].asString();
  }
  if (!resetStaticCostmap(map_name, osm_map, map_md5)) {
    LOG(ERROR) << "NaviGoal reset costmap failed.";
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "NaviGoal resetStaticCostmap failed!!";
    return;
  }
  RobotInfo::getPtrInstance()->setActionType(ActionType::NAVIGAOL);
  setNaviType(rev_data);

  SubPath global_path;
  global_path = ptr_logic_controller_->setGoalPose(target_pose);
  if (global_path.wps.empty()) {
    LOG(ERROR) << "logic_controller setGoalPose failed." << std::endl;
    ack_data["succeed"] = false;
    ack_data["error_code"] =
        static_cast<int>(ptr_logic_controller_->getPlanErrorMsg());
    ack_data["error_msg"] = "logic_controller setGoalPose failed.";
    return;
  } else {
    global_path_ = global_path.wps;
    ack_data["succeed"] = true;
    ack_data["result"] = Json::Value(Json::ValueType::arrayValue);
    // for (auto point : global_path.wps) {
    //   Json::Value point_json;
    //   point_json["x"] = point.getX();
    //   point_json["y"] = point.getY();
    //   point_json["yaw"] = point.getYaw();
    //   ack_data["result"].append(point_json);
    // }
    setNaviState(NavigationState::RUNNING);
    setErrorCode(NavigationErrorCode::NO);
    clearReplanCount();
    updateOdomTime();
    updateLocalizationTime();
    publishPath(global_path.wps, global_path_pub_);
  }
}

void NavigationArchAdapter::missionManagerMakePlan(const Json::Value &rev_data,
                                                   Json::Value &ack_data) {
  double a_x = rev_data["a_x"].asDouble();
  double a_y = rev_data["a_y"].asDouble();
  double a_yaw = rev_data["a_yaw"].asDouble();
  double b_x = rev_data["b_x"].asDouble();
  double b_y = rev_data["b_y"].asDouble();
  double b_yaw = rev_data["b_yaw"].asDouble();
  Pose2d pose_a(a_x, a_y, a_yaw);
  Pose2d pose_b(b_x, b_y, b_yaw);
  std::string map_name = rev_data["map_file"].asString();
  std::string map_md5 = rev_data["map_md5"].asString();

  LOG(INFO) << "Misson check plan: (" << a_x << ", " << a_y << ", " << a_yaw
            << ") to (" << b_x << ", " << b_y << ", " << b_yaw
            << ") at map: " << map_name << " md5: " << map_md5;
  std::string osm_map = "";
  if (rev_data.isMember("osm_path")) {
    osm_map = rev_data["osm_path"].asString();
  }
  if (!resetStaticCostmap(map_name, osm_map, map_md5)) {
    LOG(ERROR) << "NaviGoal reset costmap failed.";
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "NaviGoal resetStaticCostmap failed!!";
    return;
  }

  SubPath global_path =
      ptr_logic_controller_->makePlanBetweenPoints(pose_a, pose_b);

  double path_resolution =
      ptr_logic_controller_->calcPathResolution(global_path.wps);

  if (global_path.wps.empty()) {
    LOG(ERROR) << "Check two points path failed.";
    ack_data["succeed"] = false;
    ack_data["error_code"] =
        static_cast<int>(ptr_logic_controller_->getPlanErrorMsg());
    ack_data["error_msg"] = "Check two points path failed.";
    return;
  } else {
    ack_data["succeed"] = true;
    ack_data["length"] =
        path_resolution * (global_path.wps.size() - 1);  // 路径长度(米)
    ack_data["result"] = Json::Value(Json::ValueType::arrayValue);
    for (auto point : global_path.wps) {
      Json::Value point_json;
      point_json["x"] = point.getX();
      point_json["y"] = point.getY();
      point_json["yaw"] = point.getYaw();
      ack_data["result"].append(point_json);
    }
  }
}

void NavigationArchAdapter::missionManagerAdditionalClean(
    const Json::Value &rev_data, Json::Value &ack_data) {
  if (getNaviState() == NavigationState::UINIT) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "haven't start navigation yet";
    return;
  }

  std::string map_path = rev_data["map_path"].asString();
  if (access(map_path.c_str(), F_OK)) {
    LOG(ERROR) << "map: " << map_path << " didn`t exist!!";
    ack_data["error_msgs"] = "map didn`t exist!!";
    ack_data["succeed"] = false;
    return;
  }
  setNaviType(rev_data);
  MutilArea mutil_area;

  for (Json::ArrayIndex i = 0; i < rev_data["areas"].size(); i++) {
    Area area;
    for (Json::ArrayIndex j = 0; j < rev_data["areas"][i]["point"].size();
         j++) {
      Vectex vectex;
      vectex(0) = rev_data["areas"][i]["point"][j]["x"].asDouble();
      vectex(1) = rev_data["areas"][i]["point"][j]["y"].asDouble();
      area.push_back(vectex);
    }
    mutil_area.push_back(area);
  }
  SubPath clean_path;
  if (!ptr_clean_decision_->AdditionalCoverPlan(map_path, mutil_area,
                                                clean_path) ||
      clean_path.empty()) {
    // ptr_clean_decision_->reset();
    ack_data["succeed"] = false;
    return;
  }
  ack_data["succeed"] = true;
  ptr_clean_decision_->saveAlreadyCleanMap("/tmp/additional_clean");
  // ptr_clean_decision_->reset();
  SubPath navi_path = ptr_logic_controller_->makePlanToPoints(clean_path[0]);
  for (size_t index = 0; index < clean_path.size(); index++) {
    navi_path.wps.push_back(clean_path.wps[index]);
    navi_path.wpis.push_back(clean_path.wpis[index]);
  }
  global_path_ = navi_path.wps;
  bool set_path_success = ptr_logic_controller_->setGoalPath(navi_path);

  if (set_path_success) {
    ack_data["succeed"] = true;
    ack_data["result"] = Json::Value(Json::ValueType::arrayValue);
    // for (auto point : navi_path.wps) {
    //   Json::Value point_json;
    //   point_json["x"] = point.getX();
    //   point_json["y"] = point.getY();
    //   point_json["yaw"] = point.getYaw();
    //   ack_data["result"].append(point_json);
    // }
    setNaviState(NavigationState::RUNNING);
    setErrorCode(NavigationErrorCode::NO);
    updateOdomTime();
    updateLocalizationTime();
  } else {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "set goal path failed.";
  }
}

void NavigationArchAdapter::missionManagerClearClean(
    const Json::Value &rev_data, Json::Value &ack_data) {
  ptr_clean_decision_->reset();
  ack_data["succeed"] = true;
}

void NavigationArchAdapter::missionManagerSetCarResult(
    const Json::Value &rev_data, Json::Value &ack_data) {
  LOG(INFO) << "resdata: " << rev_data.toStyledString();
  LOG(INFO) << "Time: " << rev_data["time"].asDouble();
  // CameraResult
  auto ptr_detection_result = std::make_shared<CarDetectionResult>();
  CameraResult camera_result;
  camera_result.id = CameraType::FrontCamera;
  for (auto one_car : rev_data["car_pose"]) {
    DetectionBox car_box(
        one_car["lefttop_x"].asInt(), one_car["lefttop_y"].asInt(),
        one_car["rightdown_x"].asInt(), one_car["rightdown_y"].asInt());
    camera_result.boxes.push_back(car_box);
  }

  static TimeType time;
  time = tf2_ros::timeToSec(ptr_clock_->now());
  LOG(INFO) << "lolololo time: " << time;
  ptr_detection_result->push_back(camera_result);
  ptr_detection_result->d_time = time;
  CostmapMediator::getPtrInstance()->updateData("car_detection_result",
                                                ptr_detection_result);
  ack_data["succeed"] = true;
}

void NavigationArchAdapter::missionManagerSetFullGoalPath(
    const Json::Value &rev_data, Json::Value &ack_data) {
  if (getNaviState() == NavigationState::UINIT) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "haven't start navigation yet";
    return;
  }
  if (getNaviState() == NavigationState::RUNNING ||
      getNaviState() == NavigationState::PAUSE) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "navigation running now";
    return;
  }
  RobotInfo::getPtrInstance()->setActionType(ActionType::CLEAN);
  setNaviType(rev_data);

  SubPath goal_path;
  Pose2d path_point;
  for (auto point : rev_data["full_path"]) {
    path_point.setX(point["x"].asDouble());
    path_point.setY(point["y"].asDouble());
    path_point.setYaw(point["yaw"].asDouble());
    goal_path.wps.push_back(path_point);
    goal_path.wpis.push_back({0, 0, 0, point["type"].asInt(), 0});
  }

  std::string map_name = rev_data["map_file"].asString();
  std::string map_md5 = rev_data["map_md5"].asString();
  std::string osm_map = "";
  if (rev_data.isMember("osm_path")) {
    osm_map = rev_data["osm_path"].asString();
  }

  if (!resetStaticCostmap(map_name, osm_map, map_md5)) {
    LOG(ERROR) << "NaviGoal reset costmap failed.";
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "NaviGoal resetStaticCostmap failed!!";
    return;
  }
  for (int i = 0; i < 3; ++i) {
    publishPath(goal_path.wps, global_path_pub_);
    LOG(INFO) << "publish global path";
  }
  global_path_ = goal_path.wps;

  bool set_path_success = ptr_logic_controller_->setGoalPath(goal_path);
  ptr_clean_decision_->setMissionType(ActionType::CLEAN);

  if (set_path_success) {
    ack_data["succeed"] = true;
    setNaviState(NavigationState::RUNNING);
    setErrorCode(NavigationErrorCode::NO);
    updateOdomTime();
    updateLocalizationTime();
  } else {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "set goal path failed.";
  }
}

void NavigationArchAdapter::missionManagerSetGoalPath(
    const Json::Value &rev_data, Json::Value &ack_data) {
  if (getNaviState() == NavigationState::UINIT) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "haven't start navigation yet";
    return;
  }

  if (getNaviState() == NavigationState::RUNNING ||
      getNaviState() == NavigationState::PAUSE) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "navigation running now";
    return;
  }
  RobotInfo::getPtrInstance()->setActionType(ActionType::CLEAN);

  setNaviType(rev_data);

  SubPath goal_path;
  if (!PathTools::getPathFromFile(rev_data["path"].asString(), goal_path,
                                  MISSIONTYPE::EDGE)) {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "can't get path file!!";
    return;
  }
  std::string map_name =
      PathTools::getMapNameFromFile(rev_data["path"].asString());
  std::string map_md5 = rev_data["map_md5"].asString();
  std::string osm_map = "";
  if (rev_data.isMember("osm_path")) {
    osm_map = rev_data["osm_path"].asString();
  }
  if (!resetStaticCostmap(map_name, osm_map, map_md5)) {
    LOG(ERROR) << "reset costmap failed.";
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "resetStaticCostmap failed!!";
    return;
  }

  for (int i = 0; i < 3; ++i) {
    publishPath(goal_path.wps, global_path_pub_);
    LOG(INFO) << "publish global path";
  }
  global_path_ = goal_path.wps;
  bool set_path_success = false;
  auto end_point = goal_path.wps[goal_path.wps.size() - 1];
  auto end_point_info = goal_path.wpis[goal_path.wpis.size() - 1];
  goal_path.wps.push_back(end_point);
  goal_path.wpis.push_back(end_point_info);
  set_path_success = ptr_logic_controller_->setGoalPath(goal_path);
  ptr_clean_decision_->setMissionType(ActionType::CLEAN);
  // publishPoseArray(ptr_logic_controller_->getOptimizeMissionPath(),
  //                  global_pose_pub_);

  if (set_path_success) {
    ack_data["succeed"] = true;
    setNaviState(NavigationState::RUNNING);
    setErrorCode(NavigationErrorCode::NO);
    updateOdomTime();
    updateLocalizationTime();
  } else {
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "set goal path failed.";
  }
}

void NavigationArchAdapter::setNaviType(const Json::Value &rev_data) {
  if (!rev_data.isMember("is_forward") || rev_data["is_forward"].asBool()) {
    ptr_logic_controller_->setForwardStatus(true);
  } else {
    ptr_logic_controller_->setForwardStatus(false);
  }

  MISSIONTYPE path_type;  // 0-贴边，1-正常跟线，2-斜坡, 3-进出电梯
  if (!rev_data.isMember("path_type")) {
    path_type = map_int_mission_[1];
  } else {
    path_type = map_int_mission_[rev_data["path_type"].asInt()];
  }
  setPathType(path_type);

  if (!rev_data.isMember("is_final_rotate") ||
      !rev_data["is_final_rotate"].asBool()) {
    ptr_logic_controller_->setFinalRotateFlag(false);
    // LOG(INFO) << "final rotate is false";
  } else {
    ptr_logic_controller_->setFinalRotateFlag(true);
  }

  if (!rev_data.isMember("is_abs_reach")) {
    // rev_data["is_abs_reach"].asBool()) {
    if (is_abs_reach_) {
      ptr_logic_controller_->setAbsReachFlag(true);
    } else {
      ptr_logic_controller_->setAbsReachFlag(false);
    }
  } else {
    if (rev_data["is_abs_reach"].asBool()) {
      ptr_logic_controller_->setAbsReachFlag(true);
    } else {
      ptr_logic_controller_->setAbsReachFlag(false);
    }
  }
}

void NavigationArchAdapter::setPathType(MISSIONTYPE type) {
  LOG(INFO) << "path_type: " << type;
  RobotInfo::getPtrInstance()->setMissionType(type);
  ptr_logic_controller_->setPathStatus(type);
  switch (type) {
    case MISSIONTYPE::EDGE: {  //贴边
      ptr_costmap_arch_->setUpdateInShope(false);
      // ptr_costmap_arch_->rangeTopicReceiveSet(1, false);
      ptr_costmap_arch_->rangeTopicReceiveSet(4, true);
      // ptr_range_sensors_handle_->rangeIntputSet(1, false);
      Pose2d offset_pose(x_offset_, 0, 0);
      RobotInfo::getPtrInstance()->setOffsetPose(offset_pose);
    } break;
    case MISSIONTYPE::TRACKING: {  //跟线
      ptr_costmap_arch_->setUpdateInShope(false);
      ptr_costmap_arch_->rangeTopicReceiveSet(4, true);
      // ptr_range_sensors_handle_->rangeIntputSet(4, true);
      Pose2d offset_pose(0, 0, 0);
      RobotInfo::getPtrInstance()->setOffsetPose(offset_pose);
    } break;
    case MISSIONTYPE::SLOPE: {  //斜坡
      ptr_costmap_arch_->setUpdateInShope(true);
      ptr_costmap_arch_->rangeTopicReceiveSet(0, false);
      // ptr_range_sensors_handle_->rangeIntputSet(0, false);
      Pose2d offset_pose(0, 0, 0);
      RobotInfo::getPtrInstance()->setOffsetPose(offset_pose);
    } break;
    case MISSIONTYPE::ELVATOR: {  //进出电梯
      ptr_costmap_arch_->rangeTopicReceiveSet(4, false);
      // ptr_range_sensors_handle_->rangeIntputSet(4, false);
      // Pose2d offset_pose(0, 0, 0);
      // RobotInfo::getPtrInstance()->setOffsetPose(offset_pose);
    } break;
    default: {  //默认跟线
      ptr_costmap_arch_->rangeTopicReceiveSet(4, true);
      // ptr_range_sensors_handle_->rangeIntputSet(4, true);
      Pose2d offset_pose(0, 0, 0);
      RobotInfo::getPtrInstance()->setOffsetPose(offset_pose);
    } break;
  }
}

bool NavigationArchAdapter::startWithDefaultNavigation() {
  if (!startNavigation()) {
    LOG(ERROR) << "Start Navi Failed.";
    return false;
  }

  if (getNaviState() == NavigationState::UINIT) {
    LOG(INFO) << "haven't start navi.";
    return false;
  }

  ptr_logic_controller_->setForwardStatus(true);

  SubPath goal_path;
  if (!PathTools::getPathFromFile(default_path_file_, goal_path)) {
    LOG(ERROR) << "Can't read default path: " << default_path_file_;
    LOG(ERROR) << "Use point navi.";
  } else {
    for (int i = 0; i < 3; ++i) {
      publishPath(goal_path.wps, global_path_pub_);
      LOG(INFO) << "publish global path";
    }
    ptr_logic_controller_->setGoalPath(goal_path);
  }
  setNaviState(NavigationState::RUNNING);
  setErrorCode(NavigationErrorCode::NO);
  updateOdomTime();
  updateLocalizationTime();
  return true;
}

void NavigationArchAdapter::missionManagerGetCurrentState(
    const Json::Value &, Json::Value &ack_data) {
  NavigationState navi_state = getNaviState();
  std::string robot_motion_state =
      RobotInfo::getPtrInstance()->getRobotMotionState();
  ack_data["motion_state"] = robot_motion_state;
  if (elevator_state_ == ElevatorState::INELEVATOR) {
    ack_data["area_state"] = "InElevator";
  } else if (elevator_state_ == ElevatorState::OUTELEVATOR) {
    ack_data["area_state"] = "OutElevator";
  } else {
    ack_data["area_state"] = "HalfElevator";
  }
  LOG(INFO) << "Get navi_state: " << static_cast<int>(navi_state);
  switch (navi_state) {
    case NavigationState::UINIT:
      ack_data["state"] = "Unconfigure";
      LOG(INFO) << "ack_data: "
                << "Unconfigure";
      break;

    case NavigationState::IDLE:
      ack_data["state"] = "Idle";
      LOG(INFO) << "ack_data: "
                << "Idle";
      break;

    case NavigationState::ELEVATOR_DONE:
      ack_data["state"] = "ElevatorDone";
      LOG(INFO) << "ack_data: ";
      break;

    case NavigationState::PAUSE:
      ack_data["state"] = "Pause";
      LOG(INFO) << "ack_data: "
                << "Pause";
      break;

    case NavigationState::RUNNING:
      ack_data["state"] = "Run";
      ack_data["currentPathPercent"] = ptr_logic_controller_->getCompletion();
      ack_data["currentPathRemainLength"] =
          ptr_logic_controller_->getReferRemainLength();
      LOG(INFO) << "ack_data: "
                << "Run";
      break;

    case NavigationState::ERROR: {
      ack_data["state"] = "Error";
      NavigationErrorCode error_code = getErrorCode();
      if (NavigationErrorCode::NO == error_code) {
        ack_data["error_code"] = "no";
      } else if (NavigationErrorCode::LOCALIZATION_TIMEOUT == error_code) {
        ack_data["error_code"] = "localization_timeout";
      } else if (NavigationErrorCode::ODOM_TIMEOUT == error_code) {
        ack_data["error_code"] = "odom_timeout";
      } else if (NavigationErrorCode::POINT_OCCUPY == error_code) {
        ack_data["error_code"] = "goal_occupy";
      } else if (NavigationErrorCode::PATH_OCCUPY == error_code) {
        ack_data["error_code"] = "path_occupy";
      } else if (NavigationErrorCode::OBSTACLE_STOP == error_code) {
        ack_data["error_code"] = "obstacle_stop";
      } else if (NavigationErrorCode::RECOVERY_ERROR == error_code) {
        ack_data["error_code"] = "recover_error";
      } else if (NavigationErrorCode::LOCALIZATION_STOP == error_code) {
        ack_data["error_code"] = "localization_stop";
      } else if (NavigationErrorCode::PATH_FARAWAY == error_code) {
        ack_data["error_code"] = "path_faraway";
      } else if (NavigationErrorCode::OVERTIME == error_code) {
        ack_data["error_code"] = "over_time";
      }
      LOG(INFO) << "ack_data: "
                << "Error";
      LOG(INFO) << "Ack error code: " << ack_data["error_code"];
    } break;

    default:
      LOG(ERROR) << "something error, should not run to here." << std::endl;
      break;
  }
}

void NavigationArchAdapter::setStripMarker(const Json::Value &rev_data,
                                           Json::Value &ack_data) {
  // 重置地图
  std::string map_name = rev_data["map_file"].asString();
  std::string map_md5 = rev_data["map_md5"].asString();
  if (!resetStaticCostmap(map_name, "", map_md5)) {
    LOG(ERROR) << "NaviGoal reset costmap failed.";
    ack_data["succeed"] = false;
    ack_data["error_msg"] = "NaviGoal resetStaticCostmap failed!!";
    return;
  }

  // 更新控制器里面的代价地图
  ptr_logic_controller_->updateCostMap(ptr_costmap_arch_->getCostmap());

  // 更新速度决策模块中的全局代价地图
  // 获取静态代价地图
  std::shared_ptr<Costmap2d> static_costmap = nullptr;
  CostmapMediator::getPtrInstance()->getData("static_costmap", static_costmap);
  SpeedDecisionBase::getInstance()->setGlobalCostmap(static_costmap);

  // 设置减速带信息
  Json::Value strip_json = rev_data["strip"];
  // SpeedDecisionBase::getInstance()->setStripMark(strip_json);
}

void NavigationArchAdapter::publishPoseArray(
    const std::vector<Pose2d> &plan_path,
    const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr &pub) {
  if (plan_path.empty() || !is_debug_mode_) {
    return;
  }

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::PoseArray pose_array;

  AngleCalculate::Quaternion q;
  for (unsigned int i = 0; i < plan_path.size(); i++) {
    q = AngleCalculate::yawToQuaternion(0.0, 0.0, plan_path[i].getYaw());
    pose.position.x = plan_path[i].getX();
    pose.position.y = plan_path[i].getY();
    pose.position.z = 0.0;
    pose.orientation.x = q.x;
    pose.orientation.y = q.y;
    pose.orientation.z = q.z;
    pose.orientation.w = q.w;
    pose_array.header.frame_id = "map";
    pose_array.poses.push_back(pose);
  }
  if (pub != nullptr) {
    try {
      pub->publish(pose_array);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what() << endl;
    }
  }
}
void NavigationArchAdapter::publishRemainPath(double path_length) {
  if (remain_path_pub_ != nullptr) {
    std_msgs::msg::Float32 remain_path_length;
    remain_path_length.data = path_length;
    remain_path_pub_->publish(remain_path_length);
  }
}
void NavigationArchAdapter::publishCarrot(const Pose2d &carrot_pose) {
  if (!is_debug_mode_) {
    return;
  }
  visualization_msgs::msg::Marker marker;
  geometry_msgs::msg::PoseStamped carrot_local;
  carrot_local.pose.position.x = carrot_pose.getX();
  carrot_local.pose.position.y = carrot_pose.getY();

  marker.pose.position = carrot_local.pose.position;
  marker.ns = "prediction";
  marker.header.frame_id = "map";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.id = 0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.5;
  marker.type = visualization_msgs::msg::Marker::CUBE;

  if (carrot_pub_ != nullptr) {
    try {
      carrot_pub_->publish(marker);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what() << endl;
    }
  }
}

void NavigationArchAdapter::publishPath(
    const std::vector<Pose2d> &path,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pub) {
  if (!is_debug_mode_ || path.empty()) {
    return;
  }
  if (pub == nullptr) {
    LOG(ERROR) << "pub ptr is empty";
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.poses.resize(path.size());
  unsigned int plan_size = path.size();
  AngleCalculate::Quaternion quaternion;
  path_msg.header.frame_id = std::string("map");
  for (unsigned int i = 0; i < plan_size; ++i) {
    quaternion = AngleCalculate::yawToQuaternion(0.0, 0.0, path[i].getYaw());

    path_msg.poses[i].pose.position.x = path[i].getX();
    path_msg.poses[i].pose.position.y = path[i].getY();
    path_msg.poses[i].pose.orientation.x = quaternion.x;
    path_msg.poses[i].pose.orientation.y = quaternion.y;
    path_msg.poses[i].pose.orientation.z = quaternion.z;
    path_msg.poses[i].pose.orientation.w = quaternion.w;
  }
  if (pub != nullptr) {
    try {
      pub->publish(path_msg);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what() << endl;
    }
  }
}

void NavigationArchAdapter::publishCmdVel(const Velocity &velocity) {
  if (cmd_vel_pub_ == nullptr) {
    LOG(ERROR) << "cmd_vel_pub_ is null.";
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = velocity.d_x;
  cmd_vel.angular.z = velocity.d_yaw;

  LOG(INFO) << "NavigationArchAdapter::publishCmdVel: " << velocity.d_x << ", "
            << velocity.d_y << ", " << velocity.d_yaw;

  if (cmd_vel_pub_ != nullptr) {
    try {
      cmd_vel_pub_->publish(cmd_vel);
    } catch (std::exception &e) {
      LOG(ERROR) << "ros pub exception";
      LOG(ERROR) << e.what() << endl;
    }
  }
}

void NavigationArchAdapter::defaultPath(std::vector<Pose2d> &default_path) {
  nav_msgs::msg::Path path_msg;
  if (!PathTools::path_transform(default_path_file_, path_msg)) {
    LOG(ERROR) << "read default path file failed. path_file: "
               << default_path_file_ << std::endl;
    return;
  }
  LOG(INFO) << "load default path file: " << default_path_file_
            << " with size: " << path_msg.poses.size() << std::endl;
  default_path.resize(path_msg.poses.size());
  for (unsigned int i = 0; i < path_msg.poses.size(); i++) {
    default_path[i].x = path_msg.poses[i].pose.position.x;
    default_path[i].y = path_msg.poses[i].pose.position.y;

    AngleCalculate::Quaternion q;
    q.x = path_msg.poses[i].pose.orientation.x;
    q.y = path_msg.poses[i].pose.orientation.y;
    q.z = path_msg.poses[i].pose.orientation.z;
    q.w = path_msg.poses[i].pose.orientation.w;
    default_path[i].yaw = AngleCalculate::quaternionToYaw(q);
  }
}

void NavigationArchAdapter::updateCostMapTimerCallback() {
  static unsigned int costmap_id = 65531;
  while (rclcpp::ok() && !stop_thread_) {
    if (ptr_logic_controller_ == nullptr) {
      LOG(ERROR) << "ptr_logic_controller_ is nullptr";
      return;
    }
    unsigned int id = ptr_costmap_arch_->getCostmapId();
    if (id != costmap_id) {
      LOG(INFO) << "update cost map";
      costmap_id = id;
      ptr_logic_controller_->updateCostMap(ptr_costmap_arch_->getCostmap());
      ptr_logic_controller_->updatePlannerCostMap();
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(50)));
  }
}

void NavigationArchAdapter::globalReplanTimerCallback() {
  while (rclcpp::ok() && !stop_thread_) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000)));
    if (getErrorCode() != NavigationErrorCode::PATH_OCCUPY) {
      continue;
      // return;
    }
    ///< 暂时写死重规划的次数限制
    if (RobotInfo::getPtrInstance()->getActionType() == ActionType::NAVIGAOL &&
        RobotInfo::getPtrInstance()->getMissionType() != MISSIONTYPE::ELVATOR &&
        replan_count_ < 5) {
      setNaviState(NavigationState::PAUSE);
      setErrorCode(NavigationErrorCode::NO);
      brakeRobot();
      Pose2d goal_pose = ptr_logic_controller_->getCurrentGoalPose();
      SubPath global_path = ptr_logic_controller_->setGoalPose(goal_pose, true);
      if (global_path.wps.empty()) {
        LOG(ERROR) << "global replan path is empty.";
        setNaviState(NavigationState::RUNNING);
        setErrorCode(NavigationErrorCode::NO);
        return;
      }
      LOG(INFO) << "global replan succeed";
      replan_count_++;
      updateOdomTime();
      updateLocalizationTime();

      publishPath(global_path.wps, global_path_pub_);
      publishPoseArray(global_path.wps, global_pose_pub_);
      setNaviState(NavigationState::RUNNING);
      setErrorCode(NavigationErrorCode::NO);
    }
  }
}

void NavigationArchAdapter::plannerTimerCallback() {
  while (rclcpp::ok() && !stop_thread_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / planner_frequency_)));
    if (getNaviState() == NavigationState::UINIT ||
        getNaviState() == NavigationState::IDLE ||
        getNaviState() == NavigationState::ELEVATOR_DONE) {
      continue;
      // return;
    } else if (getNaviState() == NavigationState::PAUSE) {
      LOG_EVERY_N(WARNING, 10) << "Robot at Pause State.";
      // return;
      continue;
    }

    std::chrono::duration<double> odom_diff = calcOdomTimeDiff();
    if (odom_diff.count() > 1.0 && getNaviState() != NavigationState::ERROR) {
      setNaviState(NavigationState::ERROR);
      setErrorCode(NavigationErrorCode::ODOM_TIMEOUT);
      LOG(ERROR) << "odom data late more than 1.0s";
      // return;
      continue;
    }

    std::chrono::duration<double> local_diff = calcLocalizationTimeDiff();
    if (local_diff.count() > 1.0 && getNaviState() != NavigationState::ERROR) {
      setNaviState(NavigationState::ERROR);
      setErrorCode(NavigationErrorCode::LOCALIZATION_TIMEOUT);
      LOG(ERROR) << "localization data late more than 1.0s";
      // return;
      continue;
    }

    if (ptr_logic_controller_ == nullptr) {
      LOG(ERROR) << "ptr_logic_controller_ is nullptr";
      // return;
      continue;
    }
    // 调整到一个频率更高的线程进行更新，提高实时性
    ptr_logic_controller_->updateCostMap(ptr_costmap_arch_->getCostmap());
    ptr_logic_controller_->updatePlannerCostMap();

    // 根据区域信息调整导航速度
    // calcLogicSpeedLevel();

    FollowerStateRes follower_state = ptr_logic_controller_->plannerUpdate();

    switch (follower_state.states) {
      case PathFollowerStates::DONE: {
        //切换状态前先刹车
        brakeRobot();
        setNaviState(NavigationState::IDLE);
        LOG(INFO) << "Navi Done";
        // return;
        continue;
      } break;

      case PathFollowerStates::ELEVATOR_DONE: {
        brakeRobot();
        setNaviState(NavigationState::ELEVATOR_DONE);
        LOG(INFO) << "Elevator Done";
        continue;
      } break;

      case PathFollowerStates::PAUSE: {
        if (follower_state.msg == "PointOccupy") {
          setNaviState(NavigationState::ERROR);
          setErrorCode(NavigationErrorCode::POINT_OCCUPY);
          CostmapArchAdapter::getPtrInstance()->resetLayer(
              LayerName::probabiltiy_voxel_layer);
          LOG(INFO) << "PointOccupy ~~~";
        } else if (follower_state.msg == "PathFaraway") {
          setNaviState(NavigationState::ERROR);
          setErrorCode(NavigationErrorCode::PATH_FARAWAY);
          LOG(INFO) << "PathFaraway ===";
        } else if (follower_state.msg == "ObstacleAvoider") {
          setNaviState(NavigationState::ERROR);
          setErrorCode(NavigationErrorCode::PATH_OCCUPY);
          CostmapArchAdapter::getPtrInstance()->resetLayer(
              LayerName::probabiltiy_voxel_layer);
          LOG(INFO) << "PathOccupy ***";
        } else if (follower_state.msg == "OverTime") {
          setNaviState(NavigationState::ERROR);
          setErrorCode(NavigationErrorCode::OVERTIME);
          LOG(INFO) << "OverTime $$$";
        } else if (follower_state.msg == "ObstacleStop") {
          setNaviState(NavigationState::ERROR);
          setErrorCode(NavigationErrorCode::OBSTACLE_STOP);
          LOG(INFO) << "ObstacleStop @@@@";
        } else if (follower_state.msg == "PassPit") {
          setNaviState(NavigationState::ERROR);
          setErrorCode(NavigationErrorCode::NO);
          LOG(INFO) << "PassPit &&&";
        }
        // return;
        continue;
      } break;

      case PathFollowerStates::FOLLOW_LOCAL: {
      } break;

      case PathFollowerStates::FOLLOW_REFERENCE: {
      } break;

      case PathFollowerStates::PASS_PIT: {
        // setNaviState(NavigationState::RUNNING);
        // setErrorCode(NavigationErrorCode::NO);
      }

      default:
        break;
    }

    // publishPath(ptr_logic_controller_->getCtrlPath(), local_path_pub_);
    publishPoseArray(ptr_logic_controller_->getCtrlPath(), local_pose_pub_);
    publishPoseArray(ptr_logic_controller_->getOriginalLocalPath(),
                     ori_local_pose_pub_);
    publishRemainPath(ptr_logic_controller_->getReferRemainLength());
  }
}

void NavigationArchAdapter::controllerTimerCallback() {
  while (rclcpp::ok() && !stop_thread_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / controller_frequency_)));
    LOG(INFO) << "controller update";
    if (getNaviState() == NavigationState::UINIT ||
        getNaviState() == NavigationState::IDLE ||
        getNaviState() == NavigationState::ELEVATOR_DONE) {
      // return;
      continue;
    } else if (getNaviState() == NavigationState::PAUSE) {
      LOG_EVERY_N(WARNING, 10) << "Robot at Pause State.";
      // return;
      continue;
    }

    ptr_logic_controller_->updateControllerCostMap();
    MoveCommand move_command;
    MoveCommand::MoveCommandStatus move_command_status =
        MoveCommand::MoveCommandStatus::OKAY;
    ptr_logic_controller_->controllerUpdate(move_command, move_command_status);
    if (getNaviState() == NavigationState::UINIT ||
        getNaviState() == NavigationState::PAUSE) {
      LOG(INFO) << "Already change state.";
      // return;
      continue;
    }
    // 如果定位或里程超时，则不发送控制命令(仿真环境中不检查是否超时)
    if (getNaviState() == NavigationState::ERROR &&
        (getErrorCode() == NavigationErrorCode::LOCALIZATION_TIMEOUT ||
         getErrorCode() == NavigationErrorCode::ODOM_TIMEOUT)) {
      LOG(ERROR) << "navi state " << static_cast<int>(getNaviState())
                 << " or error state " << static_cast<int>(getErrorCode())
                 << " error return.";
      // return;
      continue;
    }

    switch (move_command_status) {
      case MoveCommand::MoveCommandStatus::OKAY: {
        setNaviState(NavigationState::RUNNING);
        setErrorCode(NavigationErrorCode::NO);
        LOG(INFO) << "ctrl cmd : ok!";
      } break;

      case MoveCommand::MoveCommandStatus::OBSTACLE_STOP: {
        setNaviState(NavigationState::ERROR);
        setErrorCode(NavigationErrorCode::OBSTACLE_STOP);
        LOG(INFO) << "ctrl cmd : OBSTACLE_STOP";
      } break;

      case MoveCommand::MoveCommandStatus::RECOVERY_ERROR: {
        setNaviState(NavigationState::ERROR);
        setErrorCode(NavigationErrorCode::RECOVERY_ERROR);
        LOG(INFO) << "ctrl cmd : RECOVERY_ERROR";
      } break;

      case MoveCommand::MoveCommandStatus::LOCALIZATION_STOP: {
        setNaviState(NavigationState::ERROR);
        setErrorCode(NavigationErrorCode::LOCALIZATION_STOP);
        LOG(INFO) << "ctrl cmd : LOCALIZATION_STOP";
      } break;

      case MoveCommand::MoveCommandStatus::REACHED_GOAL: {
        setNaviState(NavigationState::IDLE);
        setErrorCode(NavigationErrorCode::NO);
      } break;

      default:
        break;
    }

    Velocity velocity;
    velocity.d_x = move_command.getVelX();
    velocity.d_y = move_command.getVelY();
    velocity.d_yaw = move_command.getVelTH();
    if ((move_command_status != MoveCommand::MoveCommandStatus::ERROR) &&
        (move_command_status != MoveCommand::MoveCommandStatus::REACHED_GOAL)) {
      publishCmdVel(velocity);  // 当状态是正常运行时发布速度信息
    } else {
      brakeRobot(false);  // 计算错误时发送零速度刹车(false代表无需重复发送零)
    }

    publishCarrot(move_command.getCarrotPose());
  }
}

void NavigationArchAdapter::hdmapTimerCallback() {
  while (rclcpp::ok() && !stop_thread_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(1000.0 / planner_frequency_)));
    if (getNaviState() == NavigationState::UINIT ||
        getNaviState() == NavigationState::IDLE ||
        getNaviState() == NavigationState::ELEVATOR_DONE) {
      continue;
      // return;
    }
    std::lock_guard<std::mutex> lock_hdmap(hd_map_mutex_);
    if (ptr_pnc_map_ != nullptr) {
      if (!ptr_pnc_map_->InitSubPath()) {
        ptr_pnc_map_->SetPath(ptr_logic_controller_->getOptimizeMissionPath());
      }
      auto pose = RobotInfo::getPtrInstance()->getCurrentPose();
      std::vector<Eigen::Vector2d> stop_shape =
          SpeedDecisionBase::getInstance()->getBoundingbox("stop");
      bool in_elevator_flag = true;
      bool out_elevator_flag = true;
      for (const auto &it : stop_shape) {
        Pose2d point(it.x(), it.y(), 0.0);
        Pose2d cur_point = pose * point;
        if (!ptr_pnc_map_->InElevatorArea(
                math_utils::Vec2d(cur_point.getX(), cur_point.getY()))) {
          in_elevator_flag = false;
        } else {
          out_elevator_flag = false;
        }
      }
      if (in_elevator_flag) {
        elevator_state_ = ElevatorState::INELEVATOR;
      } else if (out_elevator_flag) {
        elevator_state_ = ElevatorState::OUTELEVATOR;
      } else {
        elevator_state_ = ElevatorState::HALFELEVATOR;
      }
      bool mark_area = false;
      if (ptr_pnc_map_->InElevatorArea(
              math_utils::Vec2d(pose.getX(), pose.getY())) ||
          ptr_pnc_map_->ForwardInElevatorArea(
              ptr_logic_controller_->getReferIndex(), 3.0)) {
        cur_area_type_ = AreaType::elevator_area;
        dealAreaType(cur_area_type_);
        mark_area = true;
      }
      if (ptr_pnc_map_->InSlopeArea(
              math_utils::Vec2d(pose.getX(), pose.getY())) ||
          ptr_pnc_map_->ForwardInSlopeArea(
              ptr_logic_controller_->getReferIndex(), 2.0)) {
        cur_area_type_ = AreaType::slope_area;
        dealAreaType(cur_area_type_);
        mark_area = true;
      }
      if (ptr_pnc_map_->InNarrowArea(
              math_utils::Vec2d(pose.getX(), pose.getY())) ||
          ptr_pnc_map_->ForwardInNarrowArea(
              ptr_logic_controller_->getReferIndex(), 2.0)) {
        cur_area_type_ = AreaType::narrow_area;
        dealAreaType(cur_area_type_);
        mark_area = true;
      }
      if (ptr_pnc_map_->InBlackArea(
              math_utils::Vec2d(pose.getX(), pose.getY())) ||
          ptr_pnc_map_->ForwardInBlackArea(
              ptr_logic_controller_->getReferIndex(), 2.0)) {
        cur_area_type_ = AreaType::black_area;
        dealAreaType(cur_area_type_);
        mark_area = true;
      }
      if (ptr_pnc_map_->InCleanArea(
              math_utils::Vec2d(pose.getX(), pose.getY())) &&
          !mark_area) {
        cur_area_type_ = AreaType::clean_area;
        dealAreaType(cur_area_type_);
      } else if (!mark_area) {
        cur_area_type_ = AreaType::none;
        dealAreaType(cur_area_type_);
      }
      // deal hdmap;
    }
  }
}

void NavigationArchAdapter::dealAreaType(AreaType area_type) {
  std_msgs::msg::String area;
  if (map_area_types_.find(area_type) != map_area_types_.end()) {
    area.data = map_area_types_[area_type];
    if (area_pub_ != nullptr) {
      area_pub_->publish(area);
    }
  }
  if (area_type != last_area_type_) {
    last_area_type_ = area_type;
    if (map_area_types_.find(area_type) != map_area_types_.end()) {
      switch (area_type) {
        case AreaType::narrow_area: {
          ptr_costmap_arch_->rangeTopicReceiveSet(4, false);
          // ptr_range_sensors_handle_->rangeIntputSet(4, false);
          LOG(INFO) << "cur pose is in narrow area";
        } break;
        case AreaType::clean_area: {
          ptr_costmap_arch_->rangeTopicReceiveSet(4, true);
          // ptr_range_sensors_handle_->rangeIntputSet(4, true);
          ptr_costmap_arch_->activeLayer(LayerName::negative_obstacle_layer);
          ptr_costmap_arch_->activeLayer(LayerName::probabiltiy_voxel_layer);
          ptr_costmap_arch_->activeLayer(LayerName::sonar_layer);
          CostmapMediator::getPtrInstance()->resetCameraConfig(
              "obstacle_cloud_front_down");
          CostmapMediator::getPtrInstance()->resetCameraConfig(
              "obstacle_cloud_front_up");
          CostmapMediator::getPtrInstance()->resetCameraConfig(
              "obstacle_cloud_back");
          LOG(INFO) << "cur pose is in clean area";
        } break;
        case AreaType::none: {
          ptr_costmap_arch_->rangeTopicReceiveSet(4, true);
          // ptr_range_sensors_handle_->rangeIntputSet(4, true);
          ptr_costmap_arch_->activeLayer(LayerName::negative_obstacle_layer);
          ptr_costmap_arch_->activeLayer(LayerName::probabiltiy_voxel_layer);
          ptr_costmap_arch_->activeLayer(LayerName::sonar_layer);
          ptr_costmap_arch_->activeLayer(LayerName::range_layer);
          CostmapMediator::getPtrInstance()->resetCameraConfig(
              "obstacle_cloud_front_down");
          CostmapMediator::getPtrInstance()->resetCameraConfig(
              "obstacle_cloud_front_up");
          CostmapMediator::getPtrInstance()->resetCameraConfig(
              "obstacle_cloud_back");
          LOG(INFO) << "cur pose is in clean area";
        } break;
        case AreaType::slope_area: {
          ptr_costmap_arch_->deactiveLayer(LayerName::negative_obstacle_layer);
          ptr_costmap_arch_->deactiveLayer(LayerName::probabiltiy_voxel_layer);
          LOG(INFO) << "cur pose is in slope area";
        } break;
        case AreaType::black_area: {
          ptr_costmap_arch_->deactiveLayer(LayerName::negative_obstacle_layer);
          LOG(INFO) << "cur pose is in black area";
        } break;
        case AreaType::elevator_area: {
          CameraConfig camera_config;
          camera_config.min_h_ = 0.10;
          camera_config.max_d_ = 2.0;
          CostmapMediator::getPtrInstance()->setCamreaConfig(
              "obstacle_cloud_front_down", camera_config);
          CostmapMediator::getPtrInstance()->setCamreaConfig(
              "obstacle_cloud_front_up", camera_config);
          CostmapMediator::getPtrInstance()->setCamreaConfig(
              "obstacle_cloud_back", camera_config);
          ptr_costmap_arch_->deactiveLayer(LayerName::negative_obstacle_layer);
          // ptr_costmap_arch_->deactiveLayer(LayerName::probabiltiy_voxel_layer);
          ptr_costmap_arch_->deactiveLayer(LayerName::sonar_layer);
          ptr_costmap_arch_->deactiveLayer(LayerName::range_layer);
          LOG(INFO) << "cur pose is in elevator";
        } break;
        default: {
          LOG(INFO) << "no need to deal with this area";
        } break;
      }
    }
  }
}

bool NavigationArchAdapter::stringToJson(const std::string &str_data,
                                         Json::Value &pose) {
  if (str_data.size() != 0) {
    Json::Reader json_reader;
    try {
      if (!json_reader.parse(str_data, pose)) {
        LOG(ERROR) << "JSON parse error!";
        return false;
      }
    } catch (...) {
      LOG(ERROR) << "JSON parse error!";
      return false;
    }
  }
  return true;
}

void NavigationArchAdapter::setNaviState(const NavigationState &state) {
  std::lock_guard<std::mutex> lock(navi_state_mutex_);
  state_ = state;
}

void NavigationArchAdapter::setErrorCode(const NavigationErrorCode &code) {
  std::lock_guard<std::mutex> lock(navi_error_mutex_);
  error_code_ = code;
}

NavigationState NavigationArchAdapter::getNaviState() {
  std::lock_guard<std::mutex> lock(navi_state_mutex_);
  return state_;
}

NavigationErrorCode NavigationArchAdapter::getErrorCode() {
  std::lock_guard<std::mutex> lock(navi_error_mutex_);
  return error_code_;
}

void NavigationArchAdapter::updateOdomTime() {
  std::lock_guard<std::mutex> lock(odom_time_mutex_);
  last_odom_time_ = std::chrono::system_clock::now();
}

void NavigationArchAdapter::updateLocalizationTime() {
  std::lock_guard<std::mutex> lock(localization_time_mutex_);
  last_localization_time_ = std::chrono::system_clock::now();
}

std::chrono::duration<double> NavigationArchAdapter::calcOdomTimeDiff() {
  odom_time_mutex_.lock();
  std::chrono::time_point<std::chrono::system_clock> last_odom =
      last_odom_time_;
  odom_time_mutex_.unlock();
  auto now = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = now - last_odom;
  return diff;
}

std::chrono::duration<double>
NavigationArchAdapter::calcLocalizationTimeDiff() {
  localization_time_mutex_.lock();
  std::chrono::time_point<std::chrono::system_clock> last_localization =
      last_localization_time_;
  localization_time_mutex_.unlock();
  auto now = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = now - last_localization;
  return diff;
}

}  // namespace CVTE_BABOT
