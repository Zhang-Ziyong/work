/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file controller_limits.hpp
 *
 *@brief 控制器的速度限制
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version navigation-v2.0
 *@data 2019-12-19
 ************************************************************************/

#ifndef __CONTROLLER_LIMITS_HPP
#define __CONTROLLER_LIMITS_HPP

#include <eigen3/Eigen/Core>

namespace CVTE_BABOT {
/**
 * ControllerLimits
 * @brief
 *   控制机器人运动时的速度类参数上限值
 * */

class ControllerLimits {
public:
  ControllerLimits() = default;
  ~ControllerLimits() = default;

  ControllerLimits(const ControllerLimits &obj) {
    max_trans_vel_ = obj.max_trans_vel_;
    min_trans_vel_ = obj.min_trans_vel_;
    max_vel_x_ = obj.max_vel_x_;
    min_vel_x_ = obj.min_vel_x_;
    max_vel_y_ = obj.max_vel_y_;
    min_vel_y_ = obj.min_vel_y_;
    max_rot_vel_ = obj.max_rot_vel_;
    min_rot_vel_ = obj.min_rot_vel_;
    acc_lim_x_ = obj.acc_lim_x_;
    acc_lim_y_ = obj.acc_lim_y_;
    acc_lim_theta_ = obj.acc_lim_theta_;
    acc_limit_trans_ = obj.acc_limit_trans_;
    xy_goal_tolerance_ = obj.xy_goal_tolerance_;
    yaw_goal_tolerance_ = obj.yaw_goal_tolerance_;
    trans_stopped_vel_ = obj.trans_stopped_vel_;
    rot_stopped_vel_ = obj.rot_stopped_vel_;
    prune_plan_ = obj.prune_plan_;
    restore_defaults_ = obj.restore_defaults_;
  }

  ControllerLimits &operator=(const ControllerLimits &obj) {
    if (this == &obj) {
      return *this;
    }
    max_trans_vel_ = obj.max_trans_vel_;
    min_trans_vel_ = obj.min_trans_vel_;
    max_vel_x_ = obj.max_vel_x_;
    min_vel_x_ = obj.min_vel_x_;
    max_vel_y_ = obj.max_vel_y_;
    min_vel_y_ = obj.min_vel_y_;
    max_rot_vel_ = obj.max_rot_vel_;
    min_rot_vel_ = obj.min_rot_vel_;
    acc_lim_x_ = obj.acc_lim_x_;
    acc_lim_y_ = obj.acc_lim_y_;
    acc_lim_theta_ = obj.acc_lim_theta_;
    acc_limit_trans_ = obj.acc_limit_trans_;
    prune_plan_ = obj.prune_plan_;
    xy_goal_tolerance_ = obj.xy_goal_tolerance_;
    yaw_goal_tolerance_ = obj.yaw_goal_tolerance_;
    trans_stopped_vel_ = obj.trans_stopped_vel_;
    rot_stopped_vel_ = obj.rot_stopped_vel_;
    restore_defaults_ = obj.restore_defaults_;

    return *this;
  }

  ControllerLimits(const double &nmax_trans_vel, const double &nmin_trans_vel,
                   const double &nmax_vel_x, const double &nmin_vel_x,
                   const double &nmax_vel_y, const double &nmin_vel_y,
                   const double &nmax_rot_vel, const double &nmin_rot_vel,
                   const double &nacc_lim_x, const double &nacc_lim_y,
                   const double &nacc_lim_theta, const double &nacc_limit_trans,
                   const double &nxy_goal_tolerance,
                   const double &nyaw_goal_tolerance,
                   const bool nprune_plan = true,
                   const double ntrans_stopped_vel = 0.1,
                   const double nrot_stopped_vel = 0.1)
      : max_trans_vel_(nmax_trans_vel), min_trans_vel_(nmin_trans_vel),
        max_vel_x_(nmax_vel_x), min_vel_x_(nmin_vel_x), max_vel_y_(nmax_vel_y),
        min_vel_y_(nmin_vel_y), max_rot_vel_(nmax_rot_vel),
        min_rot_vel_(nmin_rot_vel), acc_lim_x_(nacc_lim_x),
        acc_lim_y_(nacc_lim_y), acc_lim_theta_(nacc_lim_theta),
        acc_limit_trans_(nacc_limit_trans),
        xy_goal_tolerance_(nxy_goal_tolerance),
        yaw_goal_tolerance_(nyaw_goal_tolerance),
        trans_stopped_vel_(ntrans_stopped_vel),
        rot_stopped_vel_(nrot_stopped_vel), prune_plan_(nprune_plan) {}

  /**
   * getAccLimits
   * @brief
   *   获取加速度限制
   * */
  inline std::array<double, 3> getAccLimits() const {
    std::array<double, 3> acc_limits;
    acc_limits[0] = acc_lim_x_;
    acc_limits[1] = acc_lim_y_;
    acc_limits[2] = acc_lim_theta_;
    return acc_limits;
  }

  /**
   * getMaxVelLimits
   * @brief
   *   获取最大速度限制
   * */
  inline std::array<double, 3> getMaxVelLimits() const {
    std::array<double, 3> max_vel_limits;
    max_vel_limits[0] = max_vel_x_;
    max_vel_limits[1] = max_vel_y_;
    max_vel_limits[2] = max_rot_vel_;
    return max_vel_limits;
  }

  /**
   * getMinVelLimits
   * @brief
   *   获取最小速度限制
   * */
  inline std::array<double, 3> getMinVelLimits() const {
    std::array<double, 3> min_vel_limits;
    min_vel_limits[0] = min_vel_x_;
    min_vel_limits[1] = min_vel_y_;
    min_vel_limits[2] = min_rot_vel_;
    return min_vel_limits;
  }

  /**
   * setMaxVelLimits
   * @brief
   *   设置最大速度限制
   * */
  inline void setMaxVelLimits(const std::array<double, 3> &max_vel) {
    max_vel_x_ = max_vel[0];
    max_vel_y_ = max_vel[1];
    max_rot_vel_ = max_vel[2];
  }

  /**
   * setMinVelLimits
   * @brief
   *   设置最小速度限制
   * */
  inline void setMinVelLimits(const std::array<double, 3> &min_vel) {
    min_vel_x_ = min_vel[0];
    min_vel_y_ = min_vel[1];
    min_rot_vel_ = min_vel[2];
  }

  inline void setMaxVelX(const double &max_vel_x) { max_vel_x_ = max_vel_x; }
  inline void setMinVelX(const double &min_vel_x) { min_vel_x_ = min_vel_x; }

  inline void setMaxVelY(const double &max_vel_y) { max_vel_y_ = max_vel_y; }
  inline void setMinVelY(const double &min_vel_y) { min_vel_y_ = min_vel_y; }

  inline void setMaxRot(const double &max_rot) { max_rot_vel_ = max_rot; }
  inline void setMinRot(const double &min_rot) { min_rot_vel_ = min_rot; }

  inline void setMaxTransVel(const double &max_trans_vel) {
    max_trans_vel_ = max_trans_vel;
  }
  inline void setMinTransVel(const double &min_trans_vel) {
    min_trans_vel_ = min_trans_vel;
  }

  inline void setMinVelocityThreshold(const double &min_vel_threshold) {
    min_velocity_threshold_ = min_vel_threshold;
  }
  inline void setMinRotateThreshold(const double &min_rot_threshold) {
    min_rotate_threshold_ = min_rot_threshold;
  }

  inline void setXYTolerance(const double &xy_tolerance) {
    xy_goal_tolerance_ = xy_tolerance;
  }
  inline void setYawTolerance(const double &yaw_tolerance) {
    yaw_goal_tolerance_ = yaw_tolerance;
  }

  inline void setACCLimitX(const double &acc_limit_x) {
    acc_lim_x_ = acc_limit_x;
  }
  inline void setACCLimitY(const double &acc_limit_y) {
    acc_lim_y_ = acc_limit_y;
  }
  inline void setACCLimitTH(const double &acc_limit_th) {
    acc_lim_theta_ = acc_limit_th;
  }
  inline void setACCLimitTrans(const double &acc_limit_trans) {
    acc_limit_trans_ = acc_limit_trans;
  }

  inline double getMaxTransVel() { return max_trans_vel_; }
  inline double getMinTransVel() { return min_trans_vel_; }
  inline double getMaxVelX() { return max_vel_x_; }
  inline double getMinVelX() { return min_vel_x_; }
  inline double getMaxVelY() { return max_vel_y_; }
  inline double getMinVelY() { return min_vel_y_; }
  inline double getMaxRot() { return max_rot_vel_; }
  inline double getMinRot() { return min_rot_vel_; }
  inline double getMinVelocityThreshold() { return min_velocity_threshold_; }
  inline double getMinRotateThreshold() { return min_rotate_threshold_; }
  inline double getXYTolerance() { return xy_goal_tolerance_; }
  inline double getYawTolerance() { return yaw_goal_tolerance_; }
  inline double getACCLimitX() { return acc_lim_x_; }
  inline double getACCLimitY() { return acc_lim_y_; }
  inline double getACCLimitTH() { return acc_lim_theta_; }
  inline double getACCLimitTrans() { return acc_limit_trans_; }

private:
  double min_velocity_threshold_ = 0.1;///< 机器人底盘最小可执行线速度
  double min_rotate_threshold_ = 0.05; ///< 机器人底盘最小可转弯速度
  double max_trans_vel_ = 0.5;         ///< 最大移动速度
  double min_trans_vel_ = 0.0;         ///< 最小移动速度
  double max_vel_x_ = 1.0;             ///< x方向最大移动速度
  double min_vel_x_ = 0.0;             ///< x方向最小移动速度
  double max_vel_y_ = 0.0;             ///< y方向最大移动速度
  double min_vel_y_ = 0.0;             ///< y方向最小移动速度
  double max_rot_vel_ = 0.5;           ///< 最大旋转速度
  double min_rot_vel_ = 0.0;           ///< 最小旋转速度
  double acc_lim_x_ = 0.2;             ///< x轴最大加速度
  double acc_lim_y_ = 0.0;             ///< y轴最大加速度
  double acc_lim_theta_ = 0.5;         ///< 最大旋转角加速度
  double acc_limit_trans_ = 3.2;       ///< 最大移动加速度
  double xy_goal_tolerance_ = 0.02;     ///< 与目标点直线距离的容差值
  double yaw_goal_tolerance_ = 0.2;    ///< 目目标点角度的容差值
  double trans_stopped_vel_ = 0.01;    ///< 停止线速度
  double rot_stopped_vel_ = 0.01;      ///< 停止角速度
  bool prune_plan_ = true;             ///< 裁剪路径
  bool restore_defaults_ = false;      ///< 恢复
};

} // namespace CVTE_BABOT

#endif // __CONTROLLER_LIMITS_HPP
