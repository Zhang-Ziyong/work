/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file rotation_check.cpp
 *
 *@brief 使机器人角度与路径小于一定值时再出发,并且不要朝有障碍的那一边转向
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.2
 *@data 2020-04-09
 ************************************************************************/
#include "dwa/rotation/rotation_check.hpp"
#include <glog/logging.h>
#include <math.h>
#include "dwa/world_model/costmap_model.hpp"

namespace CVTE_BABOT {

RotationCheck::RotationCheck() {}

bool RotationCheck::checkIfImpactAfterRot(const Pose2d &current_pose,
                                          const double &angle_error) {
  // 用于计算机器人旋转后的角度
  // 正值说明机器人将左转,加上旋转角得到旋转后的角度
  // 负值说明机器人将右转,减去旋转角得到旋转后的角度
  double rot_direction = angle_error > 0.0 ? 1.0 : -1.0;

  // 将机器人的中心线旋转45度,检测旋转过程是否会碰撞
  // 旋转前的中心线
  double middle_x = 0.00, middle_y = 0.00;
  transformBaseToWorld(current_pose, 1.2, 0.0, middle_x, middle_y);

  // 计算实际旋转的角度,小于MIN_ANGLE_ERROR时并不会继续旋转,控制权交还DWA了
  double valid_angle_error = std::max(fabs(angle_error) - MIN_ANGLE_ERROR, 0.0);

  // 旋转后的中心线
  double rot_angle = std::min(valid_angle_error, M_PI / 4);
  double robot_angle_rot_45 = AngleCalculate::normalizeAngle(
      (current_pose.getYaw() + rot_direction * rot_angle));
  Pose2d transform_rot_45(current_pose.getX(), current_pose.getY(),
                          robot_angle_rot_45);
  double middle_rot45_x = 0.00, middle_rot45_y = 0.00;
  transformBaseToWorld(transform_rot_45, 1.2, 0.0, middle_rot45_x,
                       middle_rot45_y);

  // 以左前(1.2, 0.4), 右前(1.2, -0.4),左后(0.3, 0.4),右后(0.3,-0.4)四个点
  // 做为机器人的避障区,计算机器人旋转后避障区在世界坐标的位置
  // 将机器人最大旋转90度,检测是否会碰撞
  rot_angle = std::min(valid_angle_error, M_PI / 2);
  double robot_angle_rot_90 = AngleCalculate::normalizeAngle(
      (current_pose.getYaw() + rot_direction * rot_angle));
  Pose2d transform_rot_90(current_pose.getX(), current_pose.getY(),
                          robot_angle_rot_90);

  double left_front_x = 0.00, left_front_y = 0.00;
  transformBaseToWorld(transform_rot_90, 1.2, 0.4, left_front_x, left_front_y);
  double right_front_x, right_front_y;
  transformBaseToWorld(transform_rot_90, 1.2, -0.4, right_front_x,
                       right_front_y);
  double left_back_x, left_back_y;
  // 判断范围选择比较靠前,设置较后的话范围太大,机器人比较靠近障碍物时很容易进入框内
  transformBaseToWorld(transform_rot_90, 0.45, 0.4, left_back_x, left_back_y);
  double right_back_x, right_back_y;
  transformBaseToWorld(transform_rot_90, 0.45, -0.4, right_back_x,
                       right_back_y);

  // 检查避障区内是否有障碍, 通过两点间的线段内是否有障碍来判断
  // 左前与右前, 左前与左后, 右前与右后
  // 同时检查中心线旋转过程是否有障碍
  if (isThereObsBetweenPoints(left_front_x, left_front_y, right_front_x,
                              right_front_y) ||  // 左前与右前
      isThereObsBetweenPoints(left_front_x, left_front_y, left_back_x,
                              left_back_y) ||  // 左前与左后
      isThereObsBetweenPoints(right_front_x, right_front_y, right_back_x,
                              right_back_y) ||  // 右前与右后
      isThereObsBetweenPoints(middle_x, middle_y, middle_rot45_x,
                              middle_rot45_y)  // 中心线旋转过程是否有障碍
      ) {
    return true;
  } else {
    return false;
  }
}

void RotationCheck::transformBaseToWorld(const Pose2d &transform,
                                         const double &base_x,
                                         const double &base_y, double &world_x,
                                         double &world_y) {
  world_x = cos(transform.getYaw()) * base_x -
            sin(transform.getYaw()) * base_y + transform.getX();
  world_y = sin(transform.getYaw()) * base_x +
            cos(transform.getYaw()) * base_y + transform.getY();
}

bool RotationCheck::isThereObsBetweenPoints(const double &d_x0,
                                            const double &d_y0,
                                            const double &d_x1,
                                            const double &d_y1) {
  unsigned int cp_x0, cp_y0, cp_x1, cp_y1;
  // 超出边界视为有障碍
  if (!CostmapModel::getPtrInstance()->worldToMap(d_x0, d_y0, cp_x0, cp_y0)) {
    LOG(ERROR) << "transform point: (" << d_x0 << ", " << d_y0
               << ") to map failed";
    return true;
  }

  if (!CostmapModel::getPtrInstance()->worldToMap(d_x1, d_y1, cp_x1, cp_y1)) {
    LOG(ERROR) << "transform point: (" << d_x1 << ", " << d_y1
               << ") to map failed";
    return true;
  }

  if (CostmapModel::getPtrInstance()->lineCostStrict(cp_x0, cp_y0, cp_x1,
                                                     cp_y1) < 0) {
    return true;
  }

  return false;
}

double RotationCheck::computeAngleError(const double &current_angle,
                                        const double &target_angle) {
  double angle_error = target_angle - current_angle;
  // 坐标范围为-pi到pi,直接减差值范围在-2pi到2pi,需要进行转换
  if (angle_error > M_PI) {
    angle_error = -(M_PI * 2 - angle_error);
  } else if (angle_error < -M_PI) {
    angle_error = M_PI * 2 + angle_error;
  }
  return angle_error;
}

bool RotationCheck::checkIfNeedToUpdateRotateTurn(const double &target_angle) {
  static double abs_target_angle_change;
  abs_target_angle_change =
      fabs(computeAngleError(target_angle, last_target_angle_));
  last_target_angle_ = target_angle;

  if (rotate_turn_ != ALLTURN) {
    if (abs_target_angle_change < MIN_CHANGE_ERROR) {
      std::string turn_string =
          rotate_turn_ == LEFTTURN ? "turn left." : "turn right.";
      LOG(INFO) << "keep last single turn, " << turn_string;
      return false;
    }
  }
  return true;
}

void RotationCheck::updateRotDirection(const Pose2d &current_pose,
                                       const double &target_angle) {
  double angle_error = computeAngleError(current_pose.getYaw(), target_angle);
  // LOG(ERROR) << "current_angle: " << current_pose.getYaw()
  //            << ", target_angle: " << target_angle
  //            << ", angle_error: " << angle_error;

  // 小于一定角度值直接按最小旋转角度的方向旋转,不继续判断
  if (fabs(angle_error) <= MIN_ANGLE_ERROR) {
    rotate_turn_ = ALLTURN;
    LOG(INFO) << "normal turn, angle_error: " << angle_error;
    return;
  }

  // 如果上次判断的有一边有障碍,则只有当当前路径角度与上次路径角度差
  // 大于阀值时认为路径发生了变化,才重新判断哪边有障碍,否则延续上次的结果
  if (!checkIfNeedToUpdateRotateTurn(target_angle)) {
    return;
  }

  // 角度差大于阀值应左转
  if (angle_error > MIN_ANGLE_ERROR) {
    rotate_turn_ = LEFTTURN;  // 左转
    LOG(INFO) << "prepare to turn left";
  } else if (angle_error < -MIN_ANGLE_ERROR) {
    // 角度差小于阀值说明应右转
    rotate_turn_ = RIGHTTURN;  // 右转
    LOG(INFO) << "prepare to turn right";
  } else {
    // 默认所有方向都可行
    rotate_turn_ = ALLTURN;
  }

  LOG(INFO) << "compute legalTurn";

  // 函数计算dwa可能得到的旋转方向,判断往该方向旋转会不会碰撞,如果两边都有,则保持原方向
  if (checkIfImpactAfterRot(current_pose, angle_error) &&
      !checkIfImpactAfterRot(current_pose, -angle_error)) {
    LOG(ERROR) << "there is obstacle in this direction";
    // 角度差为正说明原本打算左转,有障碍应右转
    if (rotate_turn_ == LEFTTURN) {
      rotate_turn_ = RIGHTTURN;  // 右转
      LOG(ERROR) << "use opposite direction: right";
    } else if (rotate_turn_ == RIGHTTURN) {
      // 角度差为负说明原本打算右转,有障碍应左转
      rotate_turn_ = LEFTTURN;  // 左转
      LOG(ERROR) << "use opposite direction: left";
    }
    std::string turn_string =
        rotate_turn_ == LEFTTURN ? "turn left." : "turn right.";
    LOG(ERROR) << "set single turn, " << turn_string;
  } else {
    LOG(INFO) << "normal turn";
  }
}
}  // namespace CVTE_BABOT
