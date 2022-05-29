/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file avoidcar_state.hpp
 *
 *@brief 车辆避让状态
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 08
 ************************************************************************/

#ifndef __AVOIDCAR_STATE_HPP
#define __AVOIDCAR_STATE_HPP

#include <mutex>
#include "state.hpp"
#include "type.hpp"

namespace CVTE_BABOT {

class AvoidCarState : public State {
 public:
  ~AvoidCarState() = default;
  static std::shared_ptr<AvoidCarState> getPtrInstance();

  FollowerStateRes dealState(PathFollower *) final;

  void setPathAngleTolerance(const double &tolerance) {
    d_angle_tolerance_ = tolerance;
  }

  void setWaitCount(const u_int16_t &wait_count) {
    ui_wait_count_ = wait_count;
  }

 private:
  AvoidCarState() { follow_state_ = PathFollowerStates::AVOID_CAR; }
  AvoidCarState(const AvoidCarState &obj) = delete;
  AvoidCarState &operator=(const AvoidCarState &obj) = delete;

  /**
  *isCloseEnoughToGoal
  *@brief
  *   判断是否已经靠近车辆避让点
  *
  *@param[in] target - 车辆避让点
  *@return true - 代表与目标点的直线距离已经在允许范围内
  **/
  bool isCloseEnoughToGoal(PathFollower *pf_ptr);

  PathFollowerStates prepareGoToRefer(PathFollower *pf_ptr);

  double d_angle_tolerance_ = M_PI / 18.0;
  u_int16_t ui_wait_count_ = 50;
};
}  // end of namespace

#endif  // end of __AVOIDCAR_STATE_HPP