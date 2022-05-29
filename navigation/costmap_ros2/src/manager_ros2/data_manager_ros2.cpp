/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file data_manager_ros2.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-08-24
 ************************************************************************/
#include "manager_ros2/data_manager_ros2.hpp"

#include <angles/angles.h>
#include <tf2/utils.h>

#include "common/point_cloud_tools.hpp"
#include "costmap_cloud.hpp"
#include "costmap_range_data.hpp"
#include "data_register.hpp"
#include "obstacle_map.hpp"

namespace CVTE_BABOT {

DataManagerRos2::DataManagerRos2(
    const rclcpp::Node::SharedPtr &node,
    const std::shared_ptr<CostmapParamsRos2> &costmap_params_ros2)
    : costmap_params_ros2_(costmap_params_ros2), node_(node) {}

void DataManagerRos2::updateParameter() {
  node_->get_parameter_or("data_manager.origin_obstacle_clouds_name",
                          s_origin_obstacle_clouds_name_,
                          std::string("camera_map"));
  node_->get_parameter_or("data_manager.detach_dynamic_point_cost_threshold",
                          i_detach_dynamic_point_cost_threshold_, 128);

  // 获取点云回调参数
  node_->get_parameter_or("data_manager.subscribe_point_cloud_topic",
                          b_subscribe_point_cloud_topic_, false);
  if (b_subscribe_point_cloud_topic_) {
    costmap_params_ros2_->loadPointCloudParams();
  }

  // 获取range回调参数
  node_->get_parameter_or("data_manager.subscribe_range_topic",
                          b_subscribe_range_topic_, false);
  if (b_subscribe_range_topic_) {
    costmap_params_ros2_->loadRangeParams();
  }

  // 获取动态障碍物回调参数
  node_->get_parameter_or("data_manager.subscribe_dynamic_obs_topic",
                          b_subscribe_dynamic_obs_topic_, false);
  if (b_subscribe_dynamic_obs_topic_) {
    costmap_params_ros2_->loadDynamicObsParams();
  }

  // 获取开关层回调参数
  node_->get_parameter_or("data_manager.subscribe_switch_topic",
                          b_subscribe_switch_topic_, false);
  if (b_subscribe_switch_topic_) {
    costmap_params_ros2_->loadSwitchParams();
  }

  // 获取红外开关回调参数
  node_->get_parameter_or("data_manager.subscribe_switch_range_topic",
                          b_subscribe_switch_range_topic_, false);
  if (b_subscribe_switch_range_topic_) {
    costmap_params_ros2_->loadRangeSwitchParams();
  }

  // 获取避障红外开关量参数
  node_->get_parameter_or(
      "data_manager.subscribe_infrared_obstacle_switch_range_topic",
      b_subscribe_infrared_obstacle_switch_range_topic_, false);
  if (b_subscribe_infrared_obstacle_switch_range_topic_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(0).best_effort());
    costmap_params_ros2_->loadInfraredObstacleSwitchRangeParams();
  }

  // 获取红外对管回调参数
  node_->get_parameter_or("data_manager.subscribe_ir_topic",
                          b_subscribe_ir_topic_, false);
  if (b_subscribe_ir_topic_) {
    costmap_params_ros2_->loadRangeIRParams();
  }
}

bool DataManagerRos2::systemInit() {
  DataRegister::createSubscribers();

  // if (nullptr == dynamic_obstacle_clouds_pub_) {
  //   dynamic_obstacle_clouds_pub_ =
  //       node_->create_publisher<sensor_msgs::msg::PointCloud>(
  //           s_origin_obstacle_clouds_name_ + "_map");
  // }
  return true;
}

void DataManagerRos2::irSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr range_msg,
    const std::string &topic_name) {
  std::string frame_id = range_msg->header.frame_id;
  if (frame_id == "") {
    LOG(ERROR) << "sonar frame_id is not set.";
    return;
  }

  //判断使能
  if (!CostmapMediator::getPtrInstance()->checkSensorEnable(topic_name + "." +
                                                            frame_id)) {
    return;
  }

  // 获取tf
  std::vector<double> sensor_tf;
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".sensor_tf", sensor_tf, sensor_tf);
  if (sensor_tf.size() != 3) {
    LOG(ERROR) << "data frame_id: " << topic_name << ", the tf was not set.";
    return;
  }

  // if (range_msg->range < 0.19 &&  range_msg->range > 8) return;
  LOG(INFO) << "irSensorCallback : range_msg->range : " << range_msg->range;

  auto range_data = std::make_shared<CostmapRangeData>();
  range_data->sensor_pose_ = {sensor_tf[0], sensor_tf[1],
                              angles::normalize_angle(sensor_tf[2])};
  range_data->s_topic_name_ = topic_name;
  range_data->s_fram_id_ = frame_id;
  range_data->range_ = range_msg->range;
  range_data->max_range_ = range_msg->max_range;
  range_data->min_range_ = range_msg->min_range;
  range_data->field_of_view_ = range_msg->field_of_view;

  if (range_data->range_ < range_data->min_range_) {
    range_data->range_ = range_data->max_range_;
  }
  double time = tf2_ros::timeToSec(range_msg->header.stamp);
  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time, current_pose);
  range_data->world_pose_ = current_pose;
  CostmapMediator::getPtrInstance()->updateData(topic_name + "." + frame_id,
                                                range_data);
}

void DataManagerRos2::rangeSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr range_msg,
    const std::string &topic_name_str) {
  std::string frame_id = range_msg->header.frame_id;
  if (frame_id == "") {
    LOG(ERROR) << "sonar frame_id is not set.";
    return;
  }

  // //判断是否使能
  if (!CostmapMediator::getPtrInstance()->checkSensorEnable(topic_name_str +
                                                            "." + frame_id)) {
    return;
  }
  // 获取tf
  std::vector<double> sensor_tf;
  CostmapMediator::getPtrInstance()->getParam(
      topic_name_str + "." + frame_id + ".sensor_tf", sensor_tf, sensor_tf);
  if (sensor_tf.size() != 3) {
    LOG(ERROR) << "sonar data frame_id: " << frame_id
               << ", the tf was not set.";
    return;
  }
  // 读取法向量
  double clear_range;
  std::vector<Eigen::Vector2d> v_product;
  std::vector<Eigen::Vector2d> v_origin;
  CostmapMediator::getPtrInstance()->getParam(
      topic_name_str + "." + frame_id + ".clear_range", clear_range, 0.0);
  if (clear_range > 0.01) {
    CostmapMediator::getPtrInstance()->getParam(
        topic_name_str + "." + frame_id + ".clear_product", v_product,
        v_product);
    CostmapMediator::getPtrInstance()->getParam(
        topic_name_str + "." + frame_id + ".clear_origin", v_origin, v_origin);
  }

  auto range_data = std::make_shared<CostmapRangeData>();
  range_data->sensor_pose_ = {sensor_tf[0], sensor_tf[1],
                              angles::normalize_angle(sensor_tf[2])};
  range_data->v_clear_product = v_product;
  range_data->v_clear_origin = v_origin;
  range_data->s_topic_name_ = topic_name_str;
  range_data->s_fram_id_ = frame_id;
  range_data->range_ = range_msg->range;
  range_data->max_range_ = range_msg->max_range;
  range_data->min_range_ = range_msg->min_range;
  range_data->field_of_view_ = range_msg->field_of_view;

  if (range_data->range_ < range_data->min_range_) {
    range_data->range_ = range_data->max_range_;
  }

  // 滤波
  bool filter_enable = false;
  CostmapMediator::getPtrInstance()->getParam(
      topic_name_str + "." + frame_id + ".filter_enable", filter_enable,
      filter_enable);
  if (filter_enable) {
    std::string key = topic_name_str + "." + frame_id;
    filter_value[key].push(range_data->range_);
    range_data->range_ = filter_value[key].getFilterData();
  }

  double time = tf2_ros::timeToSec(range_msg->header.stamp);
  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time, current_pose);
  range_data->world_pose_ = current_pose;

  CostmapMediator::getPtrInstance()->updateData(topic_name_str + "." + frame_id,
                                                range_data);
}

void DataManagerRos2::laserScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg,
    const std::string &topic_name) {
  std::string frame_id = laser_scan_msg->header.frame_id;
  std::vector<double> sensor_tf;
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".sensor_tf",
                                              sensor_tf, sensor_tf);
  if (sensor_tf.size() != 6) {
    LOG(ERROR) << "laser scan data's frame id is: " << frame_id
               << ", the tf was not set.";
    return;
  }

  auto ptr_origin_cloud = std::make_shared<CostmapCloud>();
  ptr_origin_cloud->sensor_pose_ = {sensor_tf[0],
                                    sensor_tf[1],
                                    sensor_tf[2],
                                    angles::normalize_angle(sensor_tf[3]),
                                    angles::normalize_angle(sensor_tf[4]),
                                    angles::normalize_angle(sensor_tf[5])};
  ptr_origin_cloud->s_topic_name_ = topic_name;
  bool b_need_transform_to_map = true;
  CostmapMediator::getPtrInstance()->getParam(
      frame_id + ".need_transform_to_map", b_need_transform_to_map, true);
  ptr_origin_cloud->b_need_transformed_ = b_need_transform_to_map;

  double time_stamp = tf2_ros::timeToSec(laser_scan_msg->header.stamp);

  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time_stamp, current_pose);

  WorldmapPose sensor_origin_pose = {
      cos(current_pose.d_yaw) * sensor_tf[0] -
          sin(current_pose.d_yaw) * sensor_tf[1] + current_pose.d_x,
      sin(current_pose.d_yaw) * sensor_tf[0] +
          cos(current_pose.d_yaw) * sensor_tf[1] + current_pose.d_y,
      angles::normalize_angle(current_pose.d_yaw +
                              ptr_origin_cloud->sensor_pose_.yaw)};
  // 基于激光坐标系的点
  ptr_origin_cloud->world_pose_ = current_pose;  // sensor_origin_pose;

  // double sensor_height = 0.0;
  // CostmapMediator::getPtrInstance()->getParam(frame_id + ".sensor_height",
  //                                             sensor_height, sensor_height);
  ptr_origin_cloud->origin_ = {sensor_origin_pose.d_x, sensor_origin_pose.d_y,
                               ptr_origin_cloud->sensor_pose_.z};
  ptr_origin_cloud->is_used_to_mark_ = false;
  ptr_origin_cloud->is_used_to_clear_ = false;
  // PointCloud2转换成自定义数据结构
  if (!convertLaserScanToCostmapCloud(*laser_scan_msg,
                                      *ptr_origin_cloud->ptr_v_cloud_)) {
    LOG(ERROR) << "transform laserscan to costmapPointCloud failed!";
    return;
  }

  CostmapMediator::getPtrInstance()->updateData(topic_name, ptr_origin_cloud);

  // update_func_(topic_name);
}

void DataManagerRos2::pointCloud2Callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msg,
    const std::string &topic_name) {
  std::string frame_id = point_cloud2_msg->header.frame_id;
  // LOG(INFO) << "get point cloud msg from " << topic_name;
  // x, y, yaw
  std::vector<double> sensor_tf;
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".sensor_tf",
                                              sensor_tf, sensor_tf);
  if (sensor_tf.size() != 6) {
    LOG(ERROR) << "point cloud data's frame id is: " << frame_id
               << ", the tf was not set.";
    return;
  }
  double v_fov = 0;  ///<  垂直fov
  double h_fov = 0;  ///< 水平fov
  double min_d = 0;  ///<  最小测量距离
  double max_d = 0;  ///< 最大测量距离
  double negative_min_d = 0;
  double negative_max_d = 0;
  double min_h = 0;
  double max_h = 0;
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".v_fov", v_fov,
                                              v_fov);
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".h_fov", h_fov,
                                              h_fov);
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".min_d", min_d,
                                              min_d);
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".max_d", max_d,
                                              max_d);

  CostmapMediator::getPtrInstance()->getParam(frame_id + ".negative_min_d",
                                              negative_min_d, negative_min_d);
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".negative_max_d",
                                              negative_max_d, negative_max_d);

  CostmapMediator::getPtrInstance()->getParam(frame_id + ".min_h", min_h,
                                              min_h);
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".max_h", max_h,
                                              max_h);

  CameraConfig setting_config;
  if (CostmapMediator::getPtrInstance()->getCameraConfig(topic_name,
                                                         setting_config)) {
    if (!setting_config.use) {
      return;
    }
    if (setting_config.v_fov_ > 0) {
      v_fov = setting_config.v_fov_;
    }
    if (setting_config.h_fov_ > 0) {
      h_fov = setting_config.h_fov_;
    }
    if (setting_config.min_d_ > 0) {
      min_d = setting_config.min_d_;
    }
    if (setting_config.max_d_ > 0) {
      max_d = setting_config.max_d_;
    }
    if (setting_config.negative_min_d_ > 0) {
      negative_min_d = setting_config.negative_min_d_;
    }
    if (setting_config.negative_max_d_ > 0) {
      negative_max_d = setting_config.negative_max_d_;
    }
    if (setting_config.min_h_ > 0) {
      min_h = setting_config.min_h_;
    }
    if (setting_config.max_h_ > 0) {
      max_h = setting_config.max_h_;
    }
  }

  bool b_need_transform_to_map = false;
  CostmapMediator::getPtrInstance()->getParam(
      frame_id + ".need_transform_to_map", b_need_transform_to_map, false);

  auto ptr_origin_cloud = std::make_shared<CostmapCloud>();
  ptr_origin_cloud->sensor_pose_ = {sensor_tf[0],
                                    sensor_tf[1],
                                    sensor_tf[2],
                                    angles::normalize_angle(sensor_tf[3]),
                                    angles::normalize_angle(sensor_tf[4]),
                                    angles::normalize_angle(sensor_tf[5])};
  ptr_origin_cloud->s_topic_name_ = topic_name;
  ptr_origin_cloud->v_fov_ = v_fov;
  ptr_origin_cloud->h_fov_ = h_fov;
  ptr_origin_cloud->min_d_ = min_d;
  ptr_origin_cloud->max_d_ = max_d;
  ptr_origin_cloud->negative_min_d_ = negative_min_d;
  ptr_origin_cloud->negative_max_d_ = negative_max_d;
  ptr_origin_cloud->min_h_ = min_h;
  ptr_origin_cloud->max_h_ = max_h;
  ptr_origin_cloud->b_need_transformed_ = b_need_transform_to_map;

  double time_stamp = tf2_ros::timeToSec(point_cloud2_msg->header.stamp);

  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time_stamp, current_pose);

  ptr_origin_cloud->world_pose_ = current_pose;

  double sensor_height = 0.0;
  // CostmapMediator::getPtrInstance()->getParam(frame_id + ".sensor_height",
  //                                             sensor_height, sensor_height);
  // ptr_origin_cloud->origin_ = {sensor_origin_pose.d_x,
  // sensor_origin_pose.d_y,
  //                              sensor_height};
  ptr_origin_cloud->is_used_to_mark_ = false;
  ptr_origin_cloud->is_used_to_clear_ = false;
  // PointCloud2转换成自定义数据结构
  if (!convertPointCloud2ToCostmapCloud(*point_cloud2_msg,
                                        *(ptr_origin_cloud->ptr_v_cloud_))) {
    LOG(ERROR) << "transform pointcloud2 to costmapPointCloud failed!";
    return;
  }

  CostmapMediator::getPtrInstance()->updateData(topic_name, ptr_origin_cloud);
}

void DataManagerRos2::collisionSwitchRangeCallback(
    sensor_msgs::msg::Range::SharedPtr switch_msg,
    const std::string &topic_name) {
  std::string frame_id = switch_msg->header.frame_id;  // C5该值为空
  if (frame_id == "") {
    LOG(INFO) << "SwitchRange error: fram_id not set";
    return;
  }
  bool enable_flg = true;
  int rves_flg = 0;
  double min_d = 0.0;
  double max_d = 0.0;
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".enable", enable_flg, enable_flg);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".rves_flg", rves_flg, rves_flg);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".min_d", min_d, min_d);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".max_d", max_d, max_d);

  // if (!enable_flg) {
  //   LOG(INFO) << "SwitchRange " << topic_name << "---" << frame_id
  //             << " is disable";
  //   return;
  // }
  if (!CostmapMediator::getPtrInstance()->checkSensorEnable(topic_name + "." +
                                                            frame_id)) {
    return;
  }
  //将红外信号量、红外激光建模为信号量数据
  auto ptr_switch_obs = std::make_shared<SensorSwitchObs>();
  bool touch_flag = false;
  if (rves_flg == 0 &&
      (switch_msg->range > max_d || switch_msg->range < min_d)) {
    touch_flag = true;
  } else if (rves_flg == 1 &&
             (switch_msg->range < max_d && switch_msg->range > min_d)) {
    touch_flag = true;
  } else {
    LOG(INFO) << "unknow type rves_flg : " << rves_flg;
  }
  // if (！touch_flag) return;
  ptr_switch_obs->status = touch_flag;
  ptr_switch_obs->time = time(NULL);
  ptr_switch_obs->s_topic_name_ = topic_name;
  ptr_switch_obs->s_fram_id_ = frame_id;
  double time_stamp;
  CostmapMediator::getPtrInstance()->getCurrentTime(time_stamp);
  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time_stamp, current_pose);
  ptr_switch_obs->pose = current_pose;
  std::vector<double> sensor_tf(2, 0.0);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".sensor_tf", sensor_tf, sensor_tf);
  ptr_switch_obs->touched_tfs = sensor_tf;
  CostmapMediator::getPtrInstance()->updateData(topic_name + "." + frame_id,
                                                ptr_switch_obs);
}

void DataManagerRos2::pubStopVel() {
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  LOG(INFO) << "pbu infrared 0 velue";
  int count = 0;
  // int pub_t = 20 / 0.05;
  int pub_t = 50;  //发布50次
  while (count < pub_t && rclcpp::ok()) {
    LOG(INFO) << "enter the publish thread !!!!!!";
    cmd_vel_pub_->publish(cmd_vel);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(20)));  //每20ms发布一次停障，持续一秒
    count++;
  }
  LOG(INFO) << "free the cmd_vel controlling!!";

  std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(3 * 1000)));  // 3s后将速度控制权返回给navigation

  LOG(INFO) << "restart stop detect";
  not_obs_front_ = true;
}

void DataManagerRos2::infraredObstacleSwitchRangeCallback(
    sensor_msgs::msg::Range::SharedPtr switch_msg,
    const std::string &topic_name) {
  std::string frame_id = switch_msg->header.frame_id;
  if (frame_id == "") {
    LOG(INFO) << "InfraredObstacleSwitchRange error: frame_id not set";
    return;
  }

  bool enable_flg = true;
  int rves_flg = 0;
  double min_d = 0.0;
  double max_d = 0.0;
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".enable", enable_flg, enable_flg);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".rves_flg", rves_flg, rves_flg);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".min_d", min_d, min_d);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".max_d", max_d, max_d);

  if (!CostmapMediator::getPtrInstance()->checkSensorEnable(topic_name + "." +
                                                            frame_id)) {
    return;
  }

  //将避障红外信号量转化为开关量--0，1
  auto ptr_switch_obs = std::make_shared<SensorSwitchObs>();
  bool touch_flg = false;
  if (rves_flg == 0 && switch_msg->range > max_d) {
    //正常的costmap操作
    touch_flg = true;

    //发布停障线程
    if (not_obs_front_) {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      LOG(INFO) << "pbu infrared 0 velue";
      cmd_vel_pub_->publish(cmd_vel);
      LOG(INFO) << "infrared set velocity 0!! ";
      not_obs_front_ = false;
      if (pub_stop_vel_thread_.joinable()) {
        pub_stop_vel_thread_.detach();
      }
      pub_stop_vel_thread_ =
          std::thread(std::bind(&DataManagerRos2::pubStopVel, this));
    }
  } else if (rves_flg == 1 &&
             (switch_msg->range < max_d && switch_msg->range > max_d)) {
    touch_flg = true;
  } else {
    LOG(INFO) << "unknow type rves_flg : " << rves_flg;
  }

  ptr_switch_obs->status = touch_flg;
  ptr_switch_obs->time = time(NULL);
  ptr_switch_obs->s_topic_name_ = topic_name;
  ptr_switch_obs->s_fram_id_ = frame_id;

  double time_stamp;
  CostmapMediator::getPtrInstance()->getCurrentTime(time_stamp);
  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time_stamp, current_pose);
  ptr_switch_obs->pose = current_pose;
  std::vector<double> sensor_tf(2, 0.0);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".sensor_tf", sensor_tf, sensor_tf);
  ptr_switch_obs->touched_tfs = sensor_tf;
  CostmapMediator::getPtrInstance()->updateData(topic_name + "." + frame_id,
                                                ptr_switch_obs);
}

void DataManagerRos2::collisionSwitchCallback(
    const std_msgs::msg::Bool::SharedPtr switch_msg,
    const std::string &topic_name) {
  std::string frame_id = "edge";  // C5该值为空
  if (frame_id == "") {
    LOG(ERROR) << "sonar frame_id is not set.";
    return;
  }
  //判断使能
  // bool enable_flg = true;
  // CostmapMediator::getPtrInstance()->getParam(
  //     topic_name + "." + frame_id + ".enable", enable_flg, enable_flg);
  // if (!enable_flg) {
  //   LOG(INFO) << "Switch " << topic_name << "---" << frame_id << " is
  //   disable"; return;
  // }
  if (!CostmapMediator::getPtrInstance()->checkSensorEnable(topic_name + "." +
                                                            frame_id)) {
    return;
  }
  auto ptr_switch_obs = std::make_shared<SensorSwitchObs>();
  ptr_switch_obs->status = switch_msg->data;
  ptr_switch_obs->time = time(NULL);
  ptr_switch_obs->s_topic_name_ = topic_name;
  ptr_switch_obs->s_fram_id_ = frame_id;
  double time_stamp;
  CostmapMediator::getPtrInstance()->getCurrentTime(time_stamp);
  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time_stamp, current_pose);
  ptr_switch_obs->pose = current_pose;

  std::vector<double> sensor_tf(2, 0.0);
  CostmapMediator::getPtrInstance()->getParam(
      topic_name + "." + frame_id + ".sensor_tf", sensor_tf, sensor_tf);
  ptr_switch_obs->touched_tfs = sensor_tf;
  CostmapMediator::getPtrInstance()->updateData(topic_name + "." + frame_id,
                                                ptr_switch_obs);
}

void DataManagerRos2::getDynamicObsCloudMsg(
    sensor_msgs::msg::PointCloud &dynamic_obstacle_cloud) {
  auto ptr_cloud = std::make_shared<CostmapCloud>();
  if (CostmapMediator::getPtrInstance()->getData(
          s_origin_obstacle_clouds_name_ + "_map", ptr_cloud)) {
    for (const auto &cloud_point : *ptr_cloud->ptr_v_cloud_) {
      geometry_msgs::msg::Point32 point;
      point.x = cloud_point.d_x;
      point.y = cloud_point.d_y;
      point.z = cloud_point.d_z;

      dynamic_obstacle_cloud.points.push_back(point);
    }
  }
}

void DataManagerRos2::dynamicObjectCallback(
    const lidar_perception_msgs::msg::TrackingObjectArray::SharedPtr
        ptr_dynamic_object_msg,
    const std::string &topic_name) {
  std::string frame_id = ptr_dynamic_object_msg->header.frame_id;
  double sensor_height = 0.0;
  CostmapMediator::getPtrInstance()->getParam(frame_id + ".sensor_height",
                                              sensor_height, sensor_height);

  double time_stamp = tf2_ros::timeToSec(ptr_dynamic_object_msg->header.stamp);
  WorldmapPose current_pose;
  CostmapMediator::getPtrInstance()->getPose(time_stamp, current_pose);

  auto ptr_dynamic_obstacles = std::make_shared<DynamicObstacles>();
  // map坐标系下不用转换
  ptr_dynamic_obstacles->sensor_origin_ = {current_pose.d_x, current_pose.d_y,
                                           sensor_height};

  convertTrackingObjsToDynamicObs(*ptr_dynamic_object_msg,
                                  *ptr_dynamic_obstacles);
  for (const auto &dynamic_object : ptr_dynamic_obstacles->v_dynamic_object_) {
    if (dynamic_object.obstacle_type == Car) {  // cars
      double object_distance =
          distance(current_pose.d_x, current_pose.d_y,
                   dynamic_object.position.d_x, dynamic_object.position.d_y);

      LOG(INFO) << "detected Car, id: " << dynamic_object.id
                << ", distance: " << object_distance
                << ", length: " << dynamic_object.size.d_length
                << ", width: " << dynamic_object.size.d_width
                << ", height: " << dynamic_object.size.d_height;
    } else if (dynamic_object.obstacle_type == Pedestrian) {  // pedestrians
      // LOG(ERROR) << "detected Pedestrian";
    }
  }

  CostmapMediator::getPtrInstance()->updateData(topic_name + "_obj",
                                                ptr_dynamic_obstacles);

  auto ptr_dynamic_cloud = std::make_shared<CostmapCloud>();
  ptr_dynamic_cloud->origin_ = ptr_dynamic_obstacles->sensor_origin_;
  ptr_dynamic_cloud->b_need_transformed_ = false;

  for (const auto &dynamic_object : ptr_dynamic_obstacles->v_dynamic_object_) {
    ptr_dynamic_cloud->ptr_v_cloud_->insert(
        ptr_dynamic_cloud->ptr_v_cloud_->end(),
        dynamic_object.ptr_v_cloud_->begin(),
        dynamic_object.ptr_v_cloud_->end());
  }

  CostmapMediator::getPtrInstance()->updateData(topic_name, ptr_dynamic_cloud);
}

void DataManagerRos2::convertTrackingObjsToDynamicObs(
    const lidar_perception_msgs::msg::TrackingObjectArray &dynamic_object_msg,
    DynamicObstacles &dynamic_obstacles) {
  for (size_t object_it = 0; object_it < dynamic_object_msg.ids.size();
       object_it++) {
    DynamicObject dynamic_object;

    dynamic_object.id = dynamic_object_msg.ids[object_it];

    dynamic_object.size.d_length = dynamic_object_msg.sizes[object_it].x;
    dynamic_object.size.d_width = dynamic_object_msg.sizes[object_it].y;
    dynamic_object.size.d_height = dynamic_object_msg.sizes[object_it].z;

    dynamic_object.position.d_x = dynamic_object_msg.positions[object_it].x;
    dynamic_object.position.d_y = dynamic_object_msg.positions[object_it].y;
    dynamic_object.position.d_z = dynamic_object_msg.positions[object_it].z;

    dynamic_object.direction.d_x = dynamic_object_msg.directions[object_it].x;
    dynamic_object.direction.d_y = dynamic_object_msg.directions[object_it].y;
    dynamic_object.direction.d_z = dynamic_object_msg.directions[object_it].z;

    dynamic_object.velocity.d_x = dynamic_object_msg.velocities[object_it].x;
    dynamic_object.velocity.d_y = dynamic_object_msg.velocities[object_it].y;
    dynamic_object.velocity.d_z = dynamic_object_msg.velocities[object_it].z;

    if (!convertPointCloud2ToCostmapCloud(
            dynamic_object_msg.segments[object_it],
            *dynamic_object.ptr_v_cloud_)) {
      LOG(ERROR) << "transform pointcloud2 to costmapPointCloud failed!";
      continue;
    }

    if (dynamic_object_msg.type[object_it] == 2) {  // cars
      dynamic_object.obstacle_type = Car;
    } else if (dynamic_object_msg.type[object_it] == 1) {  // pedestrians
      dynamic_object.obstacle_type = Pedestrian;
    }

    dynamic_obstacles.v_dynamic_object_.push_back(dynamic_object);
  }
}

void DataManagerRos2::startProcessorTimer() {
  // 雷达
  if (b_subscribe_point_cloud_topic_) {
    std::string topics_2d;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.topics_2d"), topics_2d, std::string(""));
    if (!topics_2d.empty()) {
      std::stringstream ss(topics_2d);
      std::string source;
      while (ss >> source) {
        v_laser_scan_subs_.push_back(
            node_->create_subscription<sensor_msgs::msg::LaserScan>(
                source, rclcpp::QoS(10).best_effort(),
                [this, source](const sensor_msgs::msg::LaserScan::SharedPtr
                                   laser_scan_msg) {
                  laserScanCallback(laser_scan_msg, source);
                }));
        LOG(INFO) << "subscribe topic: " << source;
      }
    }
  }

  // 相机
  if (b_subscribe_point_cloud_topic_) {
    std::string topics_3d;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.topics_3d"), topics_3d, std::string(""));
    if (!topics_3d.empty()) {
      std::stringstream ss(topics_3d);
      std::string source;
      while (ss >> source) {
        v_point_cloud2_subs_.push_back(
            node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                source, rclcpp::QoS(10).best_effort(),
                [this, source](const sensor_msgs::msg::PointCloud2::SharedPtr
                                   point_cloud2_msg) {
                  pointCloud2Callback(point_cloud2_msg, source);
                }));
      }
    }
  }

  // 订阅动态障碍topic,汽车和行人等
  if (b_subscribe_dynamic_obs_topic_) {
    /*std::string dynamic_obs_topic;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.dynamic_obs_topic"), dynamic_obs_topic,
        std::string(""));
    if (!dynamic_obs_topic.empty()) {
      dynamic_objects_sub_ = node_->create_subscription<
          lidar_perception_msgs::msg::TrackingObjectArray>(
          dynamic_obs_topic,
          [this, dynamic_obs_topic](
              const lidar_perception_msgs::msg::TrackingObjectArray::SharedPtr
                  ptr_dynamic_object_msg) {
            dynamicObjectCallback(ptr_dynamic_object_msg, dynamic_obs_topic);
          });
    }*/
  }

  // 超声波 || tof
  if (b_subscribe_range_topic_) {
    std::string range_topics;
    std::string source;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.range_topics"), range_topics,
        std::string(""));
    if (!range_topics.empty()) {
      // 创建sub
      LOG(INFO) << "subscribe range_sensor topic : " << range_topics;
      std::stringstream sonar_topic_list(range_topics);
      std::string sonar_topic_temp;
      while (sonar_topic_list >> sonar_topic_temp) {
        v_range_sensor_sub_.push_back(
            node_->create_subscription<sensor_msgs::msg::Range>(
                sonar_topic_temp, rclcpp::QoS(10).best_effort(),
                [this, sonar_topic_temp](
                    const sensor_msgs::msg::Range::SharedPtr msg) {
                  rangeSensorCallback(msg, sonar_topic_temp);
                }));

        std::string frame_ids;
        CostmapMediator::getPtrInstance()->getParam(
            std::string(sonar_topic_temp + ".frame_id"), frame_ids,
            std::string(""));

        std::stringstream frame_id_list(frame_ids);
        std::string frame_id;
        while (frame_id_list >> frame_id) {
          bool filter_enable;
          CostmapMediator::getPtrInstance()->getParam(
              std::string(sonar_topic_temp + "." + frame_id + ".filter_enable"),
              filter_enable, false);
          if (filter_enable) {
            filter_value.insert(std::make_pair(
                sonar_topic_temp + "." + frame_id, DataFilter<double>()));
            filter_value[sonar_topic_temp + "." + frame_id].creat(5);
          }
        }
      }
    }
  }

  // 订阅碰撞开关
  if (b_subscribe_switch_topic_) {
    std::string collision_switch_topics;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.switch_topic"), collision_switch_topics,
        std::string(""));
    std::stringstream ss_topics(collision_switch_topics);
    std::string switch_topic_temp;
    if (!collision_switch_topics.empty()) {
      LOG(INFO) << "Init collision_switch callback...";
      while (ss_topics >> switch_topic_temp) {
        v_collision_switch_sub_.push_back(
            node_->create_subscription<std_msgs::msg::Bool>(
                switch_topic_temp, rclcpp::QoS(10).best_effort(),
                [this, switch_topic_temp](const std_msgs::msg::Bool::SharedPtr
                                              ptr_collision_switch_msg) {
                  collisionSwitchCallback(ptr_collision_switch_msg,
                                          switch_topic_temp);
                }));
      }
    } else {
      LOG(ERROR) << "collision_switch_topics not set!!!";
    }
  }

  // 断崖红外
  if (b_subscribe_switch_range_topic_) {
    std::string collision_switch_range_topics;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.switch_range_topic"),
        collision_switch_range_topics, std::string(""));
    std::stringstream ss_topics_range(collision_switch_range_topics);
    std::string topic_temp_range;
    if (!collision_switch_range_topics.empty()) {
      LOG(INFO) << "Init collision_range_switch callback...";
      while (ss_topics_range >> topic_temp_range) {
        v_collision_switch_range_sub_.push_back(
            node_->create_subscription<sensor_msgs::msg::Range>(
                topic_temp_range, rclcpp::QoS(10).best_effort(),
                [this,
                 topic_temp_range](const sensor_msgs::msg::Range::SharedPtr
                                       ptr_collision_switch_range_msg) {
                  collisionSwitchRangeCallback(ptr_collision_switch_range_msg,
                                               topic_temp_range);
                }));
      }
    } else {
      LOG(ERROR) << "collision_switch_range_topics not set!!!";
    }
  }

  //避障红外回调函数调用

  if (b_subscribe_infrared_obstacle_switch_range_topic_) {
    std::string infrared_obstacle_switch_range_topics;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.infrared_obstacle_switch_range_topic"),
        infrared_obstacle_switch_range_topics, std::string(""));
    std::stringstream ss_topics_range(infrared_obstacle_switch_range_topics);
    std::string topic_temp_range;
    if (!infrared_obstacle_switch_range_topics.empty()) {
      LOG(INFO) << "Init infrared_obstacle_range_switch callback...";
      while (ss_topics_range >> topic_temp_range) {
        v_infrared_obstacle_switch_range_sub_.push_back(
            node_->create_subscription<sensor_msgs::msg::Range>(
                topic_temp_range, rclcpp::QoS(10).best_effort(),
                [this,
                 topic_temp_range](const sensor_msgs::msg::Range::SharedPtr
                                       ptr_infrared_Obstacle_switch_range_msg) {
                  infraredObstacleSwitchRangeCallback(
                      ptr_infrared_Obstacle_switch_range_msg, topic_temp_range);
                }));
      }
    } else {
      LOG(ERROR) << "infrared_switch_range_topics not set!!!";
    }
  }

  //测距红外
  if (b_subscribe_ir_topic_) {
    std::string ir_topic_str;
    CostmapMediator::getPtrInstance()->getParam(
        std::string("data_manager.ir_topic"), ir_topic_str, std::string(""));
    std::stringstream ir_topic_list(ir_topic_str);
    std::string ir_topic_temp;
    if (!ir_topic_str.empty()) {
      LOG(INFO) << "subscribe ir_range_sensor topic ";
      while (ir_topic_list >> ir_topic_temp) {
        v_range_sensor_sub_.push_back(
            node_->create_subscription<sensor_msgs::msg::Range>(
                ir_topic_temp, rclcpp::QoS(10).best_effort(),
                [this,
                 ir_topic_temp](const sensor_msgs::msg::Range::SharedPtr msg) {
                  irSensorCallback(msg, ir_topic_temp);
                }));
      }
    } else {
      LOG(ERROR) << "sonar_farme_id size error";
    }
  }
}

void DataManagerRos2::stopProcessorTimer() {
  CostmapMediator::getPtrInstance()->resetSensorEnable("");
  v_laser_scan_subs_.clear();
  v_point_cloud2_subs_.clear();
  dynamic_objects_sub_.reset();
  collision_switch_sub_.reset();
}

void DataManagerRos2::startRangeIntput(std::string source) {
  if (!CostmapMediator::getPtrInstance()->checkSensorEnable(source)) {
    LOG(INFO) << "start " << source;
    CostmapMediator::getPtrInstance()->setSensorEnable(source);
  } else {
    LOG(INFO) << source << " has already begun";
  }
}

void DataManagerRos2::stopRangeIntput(std::string source) {
  LOG(INFO) << "stop " << source;
  CostmapMediator::getPtrInstance()->resetSensorEnable(source);
}

void DataManagerRos2::startDirRangeIntput(int dir) {
  std::string topics;
  CostmapMediator::getPtrInstance()->getParam(
      std::string("data_manager.range_topics"), topics, std::string(""));

  if (topics.empty()) {
    // 未订阅话题  不操作
    return;
  }

  std::string topic_name;
  std::stringstream ss(topics);
  while (ss >> topic_name) {
    std::string frame_id;
    CostmapMediator::getPtrInstance()->getParam(
        std::string(topic_name + ".frame_id"), frame_id, std::string(""));

    std::stringstream ss_f(frame_id);
    std::string frame_id_single;
    while (ss_f >> frame_id_single) {
      startRangeIntput(topic_name + "." + frame_id_single);
    }
  }
}
void DataManagerRos2::stopDirRangeIntput(int dir) {
  // sonar_fl sonar_fr sonar_lf sonar_rf sonar_lb sonar_rb sonar_bl sonar_br
  // left_tof_sensor right_tof_sensor
  // 顺时针
  switch (dir) {
    case 0: {
      // 前
      stopRangeIntput("usound.usound_1");
      stopRangeIntput("usound.usound_2");
      stopRangeIntput("usound.usound_7");
      stopRangeIntput("usound.usound_8");

      stopRangeIntput("tof_range.tof_range_1");
      stopRangeIntput("tof_range.tof_range_2");
    } break;
    case 1: {
      // 右
      stopRangeIntput("usound.usound_8");
      stopRangeIntput("usound.usound_7");
      stopRangeIntput("usound.usound_6");
      stopRangeIntput("usound.usound_5");

      // stopRangeIntput("tof_range.tof_range_2");
    } break;
    case 2: {
      // 后
      stopRangeIntput("usound.usound_3");
      stopRangeIntput("usound.usound_4");
      stopRangeIntput("usound.usound_5");
      stopRangeIntput("usound.usound_6");
    } break;
    case 3: {
      // 左
      stopRangeIntput("usound.usound_1");
      stopRangeIntput("usound.usound_2");
      stopRangeIntput("usound.usound_3");
      stopRangeIntput("usound.usound_4");

      stopRangeIntput("tof_range.tof_range_1");
    } break;
    default: {
      std::string topics;
      CostmapMediator::getPtrInstance()->getParam(
          std::string("data_manager.range_topics"), topics, std::string(""));

      if (topics.empty()) {
        // 未订阅话题  不操作
        return;
      }

      std::string topic_name;
      std::stringstream ss(topics);
      while (ss >> topic_name) {
        std::string frame_id;
        CostmapMediator::getPtrInstance()->getParam(
            std::string(topic_name + ".frame_id"), frame_id, std::string(""));

        std::stringstream ss_f(frame_id);
        std::string frame_id_single;
        while (ss_f >> frame_id_single) {
          stopRangeIntput(topic_name + "." + frame_id_single);
        }
      }
    } break;
  }
}

}  // namespace CVTE_BABOT
