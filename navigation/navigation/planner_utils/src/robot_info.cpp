/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file robot_info.cpp
 *
 *@brief 机器人基本信息接口实现
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 09
 ************************************************************************/

#include "robot_info.hpp"

namespace CVTE_BABOT {

std::shared_ptr<RobotInfo> RobotInfo::rb_info_ptr_ = nullptr;

RobotInfo::RobotInfo() {
  motion_state_ = ROBOTMOTIONSTATE::STOP;
  motion_state2string_[ROBOTMOTIONSTATE::STOP] = "Stop";
  motion_state2string_[ROBOTMOTIONSTATE::FORWARD] = "Forward";
  motion_state2string_[ROBOTMOTIONSTATE::PAUSE] = "Pause";
  motion_state2string_[ROBOTMOTIONSTATE::RECOVER] = "Recover";
  motion_state2string_[ROBOTMOTIONSTATE::TURN_LEFT] = "Turn_left";
  motion_state2string_[ROBOTMOTIONSTATE::TURN_RIGHT] = "Turn_right";
}

std::shared_ptr<RobotInfo> RobotInfo::getPtrInstance() {
  if (rb_info_ptr_.get() == nullptr) {
    rb_info_ptr_.reset(new RobotInfo());
  }
  return rb_info_ptr_;
}

void RobotInfo::setCurrentPose(const Pose2d &current_pose) {
  current_pose_mutex_.lock();
  current_pose_ = current_pose;
  current_pose_mutex_.unlock();
}

void RobotInfo::setCurrentVelocity(const Velocity &current_velocity) {
  current_velocity_mutex_.lock();
  current_velocity_ = current_velocity;
  current_velocity_mutex_.unlock();
}

void RobotInfo::setRobotMotionState(
    const ROBOTMOTIONSTATE &robot_motion_state) {
  robot_motion_state_mutex_.lock();
  motion_state_ = robot_motion_state;
  robot_motion_state_mutex_.unlock();
}

Pose2d RobotInfo::getCurrentPose() {
  current_pose_mutex_.lock();
  Pose2d current_pose = current_pose_;
  current_pose_mutex_.unlock();
  return current_pose;
}

Pose2d RobotInfo::getCurrentPoseWithOffset() {
  current_pose_mutex_.lock();
  Pose2d current_pose = current_pose_;
  current_pose_mutex_.unlock();
  return current_pose * offset_pose_;
}

Velocity RobotInfo::getCurrentVel() {
  current_velocity_mutex_.lock();
  Velocity velocity = current_velocity_;
  current_velocity_mutex_.unlock();
  return velocity;
}

std::string RobotInfo::getRobotMotionState() {
  std::lock_guard<std::mutex> lock(robot_motion_state_mutex_);
  return motion_state2string_[motion_state_];
}

void RobotInfo::setRobotType(ROBOTTYPE type) {
  robot_type_mutex_.lock();
  robot_type_ = type;
  robot_type_mutex_.unlock();
}

void RobotInfo::setOffsetPose(const Pose2d &offset_pose) {
  offset_pose_ = offset_pose;
}

ROBOTTYPE RobotInfo::getRobotType() {
  robot_type_mutex_.lock();
  ROBOTTYPE type = robot_type_;
  robot_type_mutex_.unlock();
  return type;
}

}  // namespace CVTE_BABOT