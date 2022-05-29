/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file rotation_check.hpp
 *
 *@brief 使机器人角度与路径小于一定值时再出发,并且不要朝有障碍的那一边转向
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.2
 *@data 2020-04-09
 ************************************************************************/
#ifndef __ROTATION_CHECK_HPP
#define __ROTATION_CHECK_HPP

#include <pose2d/pose2d.hpp>
#include <string>
#include "trajectory.hpp"

namespace CVTE_BABOT {
// 机器人应该旋转的方向,机器人速度为正左转,速度为负右转
enum ROTATETURN { RIGHTTURN = -1, ALLTURN = 0, LEFTTURN = 1 };

// 当前角度与目标角度差大于阀值时才使用此函数判断，小于阀值由其他函数决定评分
const double MIN_ANGLE_ERROR = M_PI / 6;
// 当前路径角度与上次路径角度差大于阀值时认为路径发生了变化,才重新判断哪边有障碍
const double MIN_CHANGE_ERROR = 0.3;

/**
* RotationCheck
* @brief
*   使机器人角度与路径小于一定值时再出发,并倾向于朝无障碍的方向旋转
* */

class RotationCheck {
 public:
  RotationCheck();
  ~RotationCheck() = default;

  RotationCheck &operator=(const RotationCheck &obj) = delete;
  RotationCheck(const RotationCheck &obj) = delete;

  /**
   *setRotateTurn
   *@brief
   *调试用设置合适的角速度方向，RIGHTTURN说明应该朝右转
   *LEFTTURN说明应该朝左转
   *
   *@param[in]　rotate_turn-合法的角速度状态
  **/
  void setRotateTurn(const ROTATETURN &rotate_turn) {
    rotate_turn_ = rotate_turn;
  }

  /**
   *getRotateTurn
   *@brief
   *获取设置合适的角速度方向，RIGHTTURN说明应该朝右转
   *LEFTTURN说明应该朝左转
   *
   *@return 合适的角速度状态
  **/
  ROTATETURN getRotateTurn() { return rotate_turn_; }

  /**
   *updateRotDirection
   *@brief
   *计算并更新机器人可旋转的方向
   *
   *@param[in] current_pose-当前位置
   *@param[out] target_angle-目标角度
  **/
  void updateRotDirection(const Pose2d &current_pose,
                          const double &target_angle);

 private:
  /**
   *checkIfNeedToUpdateRotateTurn
   *@brief
   *检查是否需要更新可旋转的方向,如果上次已经计算出某个方向有障碍,不能朝那边旋转,
   *则只有当target_angle与上一次的target_angle变化较大时,认为障碍信息变化,
   *路径发生变化,才重新重新计算
   *
   *@param[in] target_angle-目标角度
   *@return true-需要更新可旋转的方向, false-不需要
  **/
  bool checkIfNeedToUpdateRotateTurn(const double &target_angle);

  /**
   *isThereObsBetweenPoints
   *@brief
   *检查两点间是否有障碍，两点任意一点不在costmap内都会返回true.
   *
   *@param[in] d_x0-起点x值
   *@param[in] d_y0-起点y值
   *@param[in] d_x1-终点x值
   *@param[in] d_y1-终点y值
   *@return true-有障碍, false-无障碍
  **/
  bool isThereObsBetweenPoints(const double &d_x0, const double &d_y0,
                               const double &d_x1, const double &d_y1);

  /**
   *transformBaseToWorld
   *@brief
   *使用传入的变换将base_link坐标系的点转换到世界坐标系,用于计算footprint在世界坐标的位置
   *
   *@param[in] transform-变换
   *@param[in] base_x-base_link坐标系下该点的x
   *@param[in] base_y-base_link坐标系下该点的y
   *@param[out] world_x-世界坐标系下该点的x
   *@param[out] world_y-世界坐标系下该点的y
  **/
  void transformBaseToWorld(const Pose2d &transform, const double &base_x,
                            const double &base_y, double &world_x,
                            double &world_y);

  /**
   *computeAngleError
   *@brief
   *计算目标角度与当前角度的差值,并转换到-PI到PI
   *
   *@param[in] current_angle-当前角度
   *@param[in] target_angle-目标角度
   *@return angle_error-目标角度与当前角度的差值
  **/
  double computeAngleError(const double &current_angle,
                           const double &target_angle);

  /**
   *checkIfImpactAfterRot
   *@brief
   *检测机器人朝某个方向旋转是否会发生碰撞
   *
   *@param[in] current_pose-机器人当前的位置
   *@param[in] angle_error-机器人目标角与当前的角度的差值,
   *                       正值说明机器人将左转,负值将右转
   *@return true-旋转过程会发生碰撞, false-旋转过程不会发生碰撞
  **/
  bool checkIfImpactAfterRot(const Pose2d &current_pose,
                             const double &angle_error);

  ROTATETURN rotate_turn_ =
      ALLTURN;  ///< 合法的角速度状态，限制机器人不要朝靠近障碍物的一侧旋转

  // 初始为较大的差值,使得可以触发启动条件
  double last_target_angle_ = 4 * M_PI;  ///< 上次目标点的位姿
};

}  // namespace CVTE_BABOT
#endif
