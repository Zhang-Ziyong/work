/*
 * @Author: your name
 * @Date: 2020-11-02 10:29:11
 * @LastEditTime: 2021-07-02 10:47:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath:
 * /src/tracking_motion/tracking_motion_adapter/tracking_motion_adapter.cpp
 */
#include "tracking_motion_adapter.hpp"
#include <iostream>
#include "jsoncpp/json/json.h"
#include "laser_object_track.hpp"
#include "potential_controller.hpp"
#include "kalman_filter.hpp"
#include "slam_math.hpp"
#include "log.hpp"
#include <string>
#include <fstream>
namespace TRACKING_MOTION {
TrackingMotionAdapter::TrackingMotionAdapter() {
  T_lo_ << 1, 0, 0, 0.25, 0, 1, 0, 0, 0, 0, 1, 0;
  target_in_world_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
  base_link_to_laser_ = -0.25;
  first_odom_ = true;
  state_ = UNKNOW;
  tracking_state_ = IDLE;
  ptr_tracking_adapter_ros2_ = std::make_shared<TrackingAdapterRos2>();
  ptr_tracking_adapter_ros2_->registerMissionManagerCallback(
      std::bind(&TrackingMotionAdapter::missionManagerCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  ptr_tracking_adapter_ros2_->registerOdomCallback(
      std::bind(&TrackingMotionAdapter::odomCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  ptr_tracking_adapter_ros2_->registerScanCallback(std::bind(
      &TrackingMotionAdapter::scanCallback, this, std::placeholders::_1));
  ptr_tracking_adapter_ros2_->registerPointCloudCallback(std::bind(
      &TrackingMotionAdapter::pointCloudCallback, this, std::placeholders::_1));
  map_status_str_ = {
      {INIT, "init"}, {IDLE, "idle"}, {SUCCEED, "succeed"}, {LOST, "lost"}};
  // ptr_tracking_adapter_ros2_->registerTargetCallback(std::bind(&TrackingMotionAdapter::targetCallback,
  //     this, std::placeholders::_1));
}

void TrackingMotionAdapter::start() {
  tracking_state_ = INIT;
  ptr_tracking_adapter_ros2_->init();
  std::string config_file(" ");
  ptr_tracking_adapter_ros2_->updateParameter(config_file);
  LOG(ERROR) << "tracking motion config file: " << config_file;
  std::ifstream in(config_file, std::ios::binary);
  if (!in.is_open()) {
    LOG(ERROR) << "Error opening file: " << config_file << std::endl;
    LOG(ERROR) << "use default params";
  } else {
    Json::Reader reader;
    Json::Value root;
    if (reader.parse(in, root)) {
      if (root.isMember("potential_controller")) {
        LOG(INFO) << root["potential_controller"].toStyledString();
        config_.potential_config_.T_bl_ =
            root["potential_controller"]["T_bl"].asDouble();
        config_.potential_config_.k_attactive_ =
            root["potential_controller"]["k_attactive"].asDouble();
        config_.potential_config_.k_error_angle_ =
            root["potential_controller"]["k_error_angle"].asDouble();
        config_.potential_config_.k_error_dist_ =
            root["potential_controller"]["k_error_dist"].asDouble();
        config_.potential_config_.k_repulsive_ =
            root["potential_controller"]["k_repulsive"].asDouble();
        config_.potential_config_.k_vel_ =
            root["potential_controller"]["k_vel"].asDouble();
        config_.potential_config_.k_d_ =
            root["potential_controller"]["k_d"].asDouble();
        config_.potential_config_.max_rot_ =
            root["potential_controller"]["max_rot"].asDouble();
        config_.potential_config_.max_vel_ =
            root["potential_controller"]["max_vel"].asDouble();
        config_.potential_config_.scan_size_ =
            root["potential_controller"]["scan_size"].asDouble();
        config_.potential_config_.slow_dist_ =
            root["potential_controller"]["slow_dist"].asDouble();
        config_.potential_config_.stop_dist_ =
            root["potential_controller"]["stop_dist"].asDouble();
        config_.potential_config_.repulsive_top_right_(0) =
            root["potential_controller"]["repulsive_top_right_x"].asDouble();
        config_.potential_config_.repulsive_top_right_(1) =
            root["potential_controller"]["repulsive_top_right_y"].asDouble();
        config_.potential_config_.repulsive_top_left_(0) =
            root["potential_controller"]["repulsive_top_left_x"].asDouble();
        config_.potential_config_.repulsive_top_left_(1) =
            root["potential_controller"]["repulsive_top_left_y"].asDouble();
        config_.potential_config_.repulsive_down_right_(0) =
            root["potential_controller"]["repulsive_down_right_x"].asDouble();
        config_.potential_config_.repulsive_down_right_(1) =
            root["potential_controller"]["repulsive_down_right_y"].asDouble();
        config_.potential_config_.repulsive_down_left_(0) =
            root["potential_controller"]["repulsive_down_left_x"].asDouble();
        config_.potential_config_.repulsive_down_left_(1) =
            root["potential_controller"]["repulsive_down_left_y"].asDouble();
        config_.potential_config_.footprint_top_right_(0) =
            root["potential_controller"]["footprint_top_right_x"].asDouble();
        config_.potential_config_.footprint_top_right_(1) =
            root["potential_controller"]["footprint_top_right_y"].asDouble();
        config_.potential_config_.footprint_top_left_(0) =
            root["potential_controller"]["footprint_top_left_x"].asDouble();
        config_.potential_config_.footprint_top_left_(1) =
            root["potential_controller"]["footprint_top_left_y"].asDouble();
        config_.potential_config_.footprint_down_right_(0) =
            root["potential_controller"]["footprint_down_right_x"].asDouble();
        config_.potential_config_.footprint_down_right_(1) =
            root["potential_controller"]["footprint_down_right_y"].asDouble();
        config_.potential_config_.footprint_down_left_(0) =
            root["potential_controller"]["footprint_down_left_x"].asDouble();
        config_.potential_config_.footprint_down_left_(1) =
            root["potential_controller"]["footprint_down_left_y"].asDouble();
      }
      if (root.isMember("kalmanfilter")) {
        LOG(INFO) << root["kalmanfilter"].toStyledString();
        config_.kalman_config_.Q_k_ = root["kalmanfilter"]["Q_k"].asDouble() *
                                      Eigen::Matrix<double, 4, 4>::Identity();
        config_.kalman_config_.R_k_ = root["kalmanfilter"]["R_k"].asDouble() *
                                      Eigen::Matrix<double, 2, 2>::Identity();
        config_.kalman_config_.delta_t_ =
            root["kalmanfilter"]["delta_t"].asDouble();
        config_.kalman_config_.covariance_ =
            root["kalmanfilter"]["covariance"].asDouble() *
            Eigen::Matrix<double, 4, 4>::Identity();
      }
      if (root.isMember("laser_object_track")) {
        LOG(INFO) << root["laser_object_track"].toStyledString();
        config_.laser_object_track_config_.grid_resolution_ =
            root["laser_object_track"]["grid_resolution"].asDouble();
        config_.laser_object_track_config_.region_interest_ =
            root["laser_object_track"]["region_interest"].asInt();
        config_.laser_object_track_config_.sensor_height_ =
            root["laser_object_track"]["sensor_height"].asDouble();
        config_.laser_object_track_config_.th_grid_het_ =
            root["laser_object_track"]["th_grid_het"].asDouble();
        config_.laser_object_track_config_.topic_ =
            root["laser_object_track"]["topic"].toStyledString();
        config_.laser_object_track_config_.model_path_ =
            root["laser_object_track"]["model_path"].asString();
        config_.laser_object_track_config_.range_path_ =
            root["laser_object_track"]["range_path"].asString();
      }
    } else {
      LOG(ERROR) << "Error reading file: " << config_file << std::endl;
      LOG(ERROR) << "use default params";
    }
  }
  // updateOdomTime();
  // updatePointCloudTime();
  // updateScanTime();
  ptr_track_base_ =
      std::make_shared<LaserObjectTrack>(config_.laser_object_track_config_);
  ptr_controller_ =
      std::make_shared<PotentialController>(config_.potential_config_);
  ptr_kalman_tracker_ = std::make_shared<KalmanFilter>(config_.kalman_config_);
}

void TrackingMotionAdapter::spin() {
  ptr_tracking_adapter_ros2_->spin();
}

void TrackingMotionAdapter::stop() {
  LOG(ERROR) << "stop tracking";
  ptr_tracking_adapter_ros2_->stop();
  first_odom_ = true;
  state_ = UNKNOW;
  ptr_track_base_.reset();
  ptr_controller_.reset();
  ptr_kalman_tracker_.reset();
  tracking_state_ = IDLE;
}

void TrackingMotionAdapter::odomCallback(const Mat34d &pose,
                                         const CmdVel &rev_vel) {
  updateOdomTime();
  if (first_odom_) {
    first_odom_pose_ = pose;
    first_odom_ = false;
  }
  odom_pose_mutex_.lock();
  odom_pose_ = Mathbox::deltaPose34d(first_odom_pose_, pose);
  cur_rev_vel_ = rev_vel;
  ptr_controller_->setCurVel(rev_vel.vel_);
  odom_pose_mutex_.unlock();
}

void TrackingMotionAdapter::targetCallback(const Mat34d &pose) {
  if (calcuOdomTimeDiff().count() < 1.5) {
    Mat34d delta_pose = Mathbox::Mathbox::multiplePose34d(
        Mathbox::inversePose34d(odom_pose_), pose);
    // LOG(ERROR) << "delta pose: " << std::endl << delta_pose << std::endl;
    ptr_controller_->setTargetPose(delta_pose);
    CmdVel cmd_vel;
    // ptr_controller_->computeCommand(cmd_vel);
    ptr_tracking_adapter_ros2_->publishCmdVel(cmd_vel);
  } else {
    LOG(ERROR) << "odom late more than 1.5";
  }
}

void TrackingMotionAdapter::scanCallback(
    const std::vector<Eigen::Vector2d> &scan) {
  updateScanTime();
  std::vector<Vec2d> input_scan;

  for (int index = 0; index < scan.size(); index++) {
    Eigen::Vector2d point;
    point(0) = scan[index](0) + config_.potential_config_.T_bl_;
    point(1) = scan[index](1);
    input_scan.push_back(point);
  }
  ptr_controller_->InputScan(input_scan);
}

void TrackingMotionAdapter::pointCloudCallback(
    const laserCloud::Ptr ptr_point_cloud) {
  if (calcuOdomTimeDiff().count() > 1.5 || calcuScanTimeDiff().count() > 1.5) {
    LOG(ERROR) << "odom time or scan time late more than 1.5s";
    return;
  }
  static int count = 0;
  if (ptr_track_base_->isInitTrack()) {
    ptr_kalman_tracker_->propagateState();
    Eigen::Vector4d state;
    ptr_kalman_tracker_->getState(state);
    target_in_world_.block<2, 1>(0, 3) = state.block<2, 1>(0, 0);
    // std::cout << "target_in_world_: " << std::endl << target_in_world_ <<
    // std::endl;
    Mat34d T_lt = Mathbox::multiplePose34d(Mathbox::inversePose34d(odom_pose_),
                                           target_in_world_);
    // LOG(ERROR) << "predict pose: " << std::endl << T_lt;
    ptr_track_base_->setPredictPose(T_lt);
    if (ptr_track_base_->updateTracking(ptr_point_cloud)) {
      LOG(INFO) << "update tracking";
      count = 0;
      Mat34d last_object_pose;
      ptr_track_base_->getTrackingPose(last_object_pose);
      ptr_controller_->setTargetPose(last_object_pose);
      CmdVel cmd_vel;
      ptr_controller_->computeCommand(cmd_vel, state.block<2, 1>(2, 0));
      target_in_world_.block<3, 3>(0, 0) = odom_pose_.block<3, 3>(0, 0);
      target_in_world_.block<3, 1>(0, 3) =
          odom_pose_.block<3, 3>(0, 0) * last_object_pose.block<3, 1>(0, 3) +
          odom_pose_.block<3, 1>(0, 3);
      Eigen::Vector2d measure_pose(target_in_world_(0, 3),
                                   target_in_world_(1, 3));
      ptr_kalman_tracker_->updateState(measure_pose);
      Eigen::Vector4d state;
      ptr_kalman_tracker_->getState(state);
      // LOG(ERROR) << "state: " << state.transpose();
      target_in_world_.block<2, 1>(0, 3) = state.block<2, 1>(0, 0);
      ptr_tracking_adapter_ros2_->publishPose(target_in_world_);
      ptr_tracking_adapter_ros2_->publishCmdVel(cmd_vel);
      tracking_state_ = SUCCEED;
    } else {
      if (++count % 10 == 0) {
        tracking_state_ = LOST;
      }
      ptr_kalman_tracker_->resetState();
    }
  } else {
    if (ptr_track_base_->initTracking(ptr_point_cloud)) {
      tracking_state_ = SUCCEED;
      Mat34d last_object_pose;
      ptr_track_base_->getTrackingPose(last_object_pose);
      odom_pose_mutex_.lock();
      target_in_world_.block<3, 3>(0, 0) = odom_pose_.block<3, 3>(0, 0);
      target_in_world_.block<3, 1>(0, 3) =
          odom_pose_.block<3, 3>(0, 0) * last_object_pose.block<3, 1>(0, 3) +
          odom_pose_.block<3, 1>(0, 3);
      odom_pose_mutex_.unlock();
      Eigen::Vector4d init_state(0, 0, 0, 0);
      init_state.block<2, 1>(0, 0) = target_in_world_.block<2, 1>(0, 3);
      ptr_tracking_adapter_ros2_->publishPose(target_in_world_);
      ptr_kalman_tracker_->initState(init_state);
    }
  }
}

void TrackingMotionAdapter::missionManagerCallback(std::string &rev_data,
                                                   std::string &ack_data) {
  Json::Reader json_reader;
  Json::Value json_rev_data;
  Json::Value json_ack_data;
  if (!json_reader.parse(rev_data, json_rev_data)) {
    json_ack_data["error_msgs"] = "rev_data is illegal!!";
    LOG(ERROR) << "rev_data is illegal!!";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
    return;
  }
  std::string cmd = json_rev_data["cmd"].asString();
  LOG(INFO) << "receive cmd: " << cmd;
  if (cmd == std::string("startTracking")) {
    // LOG(ERROR) << "receive cmd startTracking";
    if (state_ == UNKNOW) {
      start();
      state_ = TRACKING;
      json_ack_data["succeed"] = true;
    } else if (state_ == TRACKING) {
      stop();
      start();
      json_ack_data["succeed"] = true;
    }
  } else if (cmd == std::string("stopTracking")) {
    // LOG(ERROR) << "receive cmd stopTracking";
    // if (state_ == TRACKING) {
    stop();
    state_ = UNKNOW;
    json_ack_data["succeed"] = true;
    // } else if (state_ == UNKNOW) {
    //   //   json_ack_data["error_msgs"] = "tracking motion is stop";
    //   json_ack_data["succeed"] = true;
    // }
    // tracking_state_ = LOST;
  } else if (cmd == std::string("getState")) {
    // LOG(ERROR) << "receive cmd getState";
    json_ack_data["state"] = map_status_str_[tracking_state_];
    json_ack_data["succeed"] = true;
    LOG(INFO) << "cur_state: " << map_status_str_[tracking_state_];
  } else {
    json_ack_data["error_msgs"] = "undefine cmd";
    json_ack_data["succeed"] = false;
  }
  ack_data = json_ack_data.toStyledString();
}

std::chrono::duration<double> TrackingMotionAdapter::calcuOdomTimeDiff() {
  auto now = std::chrono::system_clock::now();
  odom_time_mutex_.lock();
  std::chrono::duration<double> diff = now - last_odom_time_;
  odom_time_mutex_.unlock();
  return diff;
}
std::chrono::duration<double> TrackingMotionAdapter::calcuScanTimeDiff() {
  auto now = std::chrono::system_clock::now();
  scan_time_mutex_.lock();
  std::chrono::duration<double> diff = now - last_scan_time_;
  scan_time_mutex_.unlock();
  return diff;
}
std::chrono::duration<double> TrackingMotionAdapter::calcuPointCloudTimeDiff() {
  auto now = std::chrono::system_clock::now();
  cloud_time_mutex_.lock();
  std::chrono::duration<double> diff = now - last_cloud_time_;
  cloud_time_mutex_.unlock();
  return diff;
}

std::chrono::duration<double> TrackingMotionAdapter::calcuTargetTimeDiff() {
  auto now = std::chrono::system_clock::now();
  target_time_mutex_.lock();
  std::chrono::duration<double> diff = now - last_target_time_;
  target_time_mutex_.unlock();
  return diff;
}

void TrackingMotionAdapter::updateOdomTime() {
  odom_time_mutex_.lock();
  last_odom_time_ = std::chrono::system_clock::now();
  odom_time_mutex_.unlock();
}
void TrackingMotionAdapter::updateScanTime() {
  scan_time_mutex_.lock();
  last_scan_time_ = std::chrono::system_clock::now();
  scan_time_mutex_.unlock();
}
void TrackingMotionAdapter::updatePointCloudTime() {
  cloud_time_mutex_.lock();
  last_cloud_time_ = std::chrono::system_clock::now();
  cloud_time_mutex_.unlock();
}

void TrackingMotionAdapter::updateTargetTime() {
  target_time_mutex_.lock();
  last_target_time_ = std::chrono::system_clock::now();
  target_time_mutex_.unlock();
}
}  // namespace TRACKING_MOTION
