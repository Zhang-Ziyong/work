/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file move_command.hpp
 *
 *@brief 用于保存规划控制指令的类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-31
 ************************************************************************/

#ifndef __MOVE_COMMAND_HPP
#define __MOVE_COMMAND_HPP

#include "trajectory.hpp"

#include <eigen3/Eigen/Core>
#include <pose2d/pose2d.hpp>

namespace CVTE_BABOT {

/**
 * MoveCommand
 * @brief
 *   用来封装不同规划控制算法的输出，统一通过该类来获取结果
 * */

class MoveCommand {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class MoveCommandStatus {
    OKAY,
    REACHED_GOAL,
    ERROR,
    OBSTACLE_STOP,
    RECOVERY_ERROR,
    LOCALIZATION_STOP,
    MISSION_STOP,
  };

  MoveCommand(bool can_rotate = false, bool torque_mode = false)
      : move_dir_(1, 0), use_rotation_(can_rotate), use_torque_(torque_mode) {}
  ~MoveCommand() = default;

  /**
   * setVelX 、setVelY 、setVelTH
   * @brief
   *   将 x、y和z轴的速度设置进来
   * */
  inline void setVelX(const double &vx) { vel_x_ = vx; }
  inline void setVelY(const double &vy) { vel_y_ = vy; }
  inline void setVelTH(const double &vth) { vel_th_ = vth; }

  /**
   * getVelX 、getVelY 、getVelTH
   * @brief
   *   获取 x、y和z轴的计算结果
   * */
  inline double getVelX() { return vel_x_; }
  inline double getVelY() { return vel_y_; }
  inline double getVelTH() { return vel_th_; }

  Trajectory getBestTraject() { return best_traject_; }
  std::vector<Trajectory> getAllTrajects() { return all_trajects_; }

  void setBestTraject(Trajectory traj) { best_traject_ = traj; }
  void setAllTrajects(std::vector<Trajectory> &traj_result) {
    all_trajects_.clear();
    all_trajects_ = traj_result;
  }

  inline Eigen::Vector2f getDirection() const { return move_dir_; }
  inline Eigen::Vector2f getVelocityVector() const {
    return vel_x_ * move_dir_;
  }

  inline float getDirectionAngle() const {
    return atan2(move_dir_[1], move_dir_[0]);
  }

  inline bool canRotate() const { return use_rotation_; }
  inline bool useTorque() const { return use_torque_; }

  inline void setDirection(const Eigen::Vector2f &dir) {
    move_dir_ = dir.normalized();
  }
  inline void setDirection(float angle) {
    move_dir_[0] = cos(angle);
    move_dir_[1] = sin(angle);
  }

  //! Check if the given value is neither NaN nor +/-infinity.
  inline bool isValid(float val) const {
    return !std::isnan(val) && !std::isinf(val);
  }

  inline void setCarrotPose(const Pose2d &carrot) { carrot_pose_ = carrot; }
  inline Pose2d getCarrotPose() { return carrot_pose_; }

  inline void setCurrentWPPose(const Pose2d &wp_pose) {
    current_wp_pose_ = wp_pose;
  }
  inline Pose2d getCurrentWPPose() { return current_wp_pose_; }

 private:
  double vel_x_ = 0.0;   ///< X轴的速度
  double vel_y_ = 0.0;   ///< Y轴的速度
  double vel_th_ = 0.0;  ///< Z轴的速度（角速度）

  //! Unit vector pointing in the direction of movement.
  Eigen::Vector2f move_dir_;
  //! If false, rot_velocity_ is undefined and must not be used.
  bool use_rotation_;
  //! If true, torque mode is used
  bool use_torque_;

  // DWA 的轨迹结果
  Trajectory best_traject_;
  std::vector<Trajectory> all_trajects_;

  // PID 的跟踪目标
  Pose2d carrot_pose_;
  Pose2d current_wp_pose_;
};

}  // namespace CVTE_BABOT

#endif  // end of __MOVE_COMMAND_HPP