/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file rotate_state.hpp
 *
 *@brief 旋转状态
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2021 - 02 - 19
 ************************************************************************/

#ifndef __ROTATE_STATE_HPP
#define __ROTATE_STATE_HPP

#include "state.hpp"
#include "type.hpp"

#include <mutex>

namespace CVTE_BABOT {

class RotateState : public State {
 public:
  ~RotateState() = default;
  static std::shared_ptr<RotateState> getPtrInstance();

  inline void setTargetAngleWithType(const double &angle,
                                     const std::string &type) {
    target_rotate_angle_ = angle;
    rotate_type_ = type;
  }

  inline void setAngleTolerance(const double &tolerance) {
    angle_tolerance_ = tolerance;
  }

  inline void setDangerValue(double dangerous_value) {
    dangerous_value_ = dangerous_value;
  }

  inline void setFootprint(double head_length, double tail_length) {
    head_length_ = head_length;
    tail_length_ = tail_length;
    calcRobotTypeWithFootprint();
  }

  inline void setRotateVelocity(const double &w) { rotate_velocity_ = w; }

  inline std::string getRotateType() { return rotate_type_; }

  FollowerStateRes dealState(PathFollower *pf_ptr) final;

 private:
  RotateState();
  RotateState(const RotateState &obj) = delete;
  RotateState &operator=(const RotateState &obj) = delete;

  bool checkIfNeedToRotate(double &angle_diff);
  double judgeRotateDirection(const double &angle_diff,
                              std::shared_ptr<Costmap2d> costmap);
  void judgeCrash(double current_angle, double target_angle,
                  bool &positive_crash, bool &negative_crash,
                  Eigen::Vector3d &robot_pose,
                  std::shared_ptr<Costmap2d> cost_map) const;
  bool judgeCrash(double current_angle, const Eigen::Vector3d &robot_pose,
                  std::shared_ptr<Costmap2d> cost_map) const;
  bool judgeCrash();

  void calcRobotTypeWithFootprint();
  double getRotateWval(const double &angle_diff);

  double target_rotate_angle_ = 0.0;
  double angle_tolerance_ = 0.1;
  double rotate_velocity_ = 0.4;
  std::string rotate_type_;
  bool is_front_dynamic_ = false;  // 是否前驱底盘，影响旋转时碰撞检测
  double head_length_ = 0.0;  // 车头长度
  double tail_length_ = 0.0;  // 车尾长度
  double dangerous_value_ = 130.0;

  bool has_been_narrow_checked_{false};  // 记录对于全局路径是否进行过窄道检测

  static std::shared_ptr<RotateState> ptr_rotate_state_;
};

}  // namespace CVTE_BABOT

#endif  // end of __ROTATE_STATE_HPP
