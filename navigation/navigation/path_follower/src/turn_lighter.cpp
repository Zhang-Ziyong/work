/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file turn_lighter.cpp
 *
 *@brief 转向灯的接口实现
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev
 *@data 2021 - 02 - 26
 ************************************************************************/

#include "turn_lighter.hpp"
#include "robot_info.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {
  
void TurnLighter::updateTurnLightWithCtrlPath(const SubPath &ctrl_path) {
  Pose2d turn_target_point = ctrl_path.wps.back();
  Pose2d robot_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  Pose2d target_at_robot_coor = robot_pose.inverse() * turn_target_point;
  // 将当前目标点投影到机器人坐标系，根据Y轴的大小来判断接下来要往哪个方向运动
  control_direction_ = target_at_robot_coor.getY();
}

void TurnLighter::updateTurnLightWithRotate(const double &w_vel) {
  if (w_vel < -0.01) {
    control_direction_ = -1.0;
  } else if (w_vel > 0.01) {
    control_direction_ = 1.0;
  } else {
    resetForwardTurnLight();
  }
}


int TurnLighter::getCtrlDirection() {
  if (control_direction_ < -turn_direction_tolerance_) {
    return -1;  // 左转
  } else if (control_direction_ > turn_direction_tolerance_) {
    return 1;  // 右转
  } else {
    return 0;  // 前进
  }
}

int TurnLighter::getCtrlDirectionWithFilter() {
  // 方向计算滤波：当路径方向连续出现 filter_num
  // 次，才认为是当前路径朝向，避免中间跳变导致灯闪烁不连续
  // 比如： 左左左右左左的情况，中间右将会被过滤
  const int filter_num = 3;
  int dir = getCtrlDirection();
  if (last_control_direction_ == dir) {
    direction_count_++;
  } else {
    direction_count_ = 0;
  }
  last_control_direction_ = dir;
  if (direction_count_ >= filter_num) {
    final_control_direction_ = last_control_direction_;
  }
  return final_control_direction_;
}

}

