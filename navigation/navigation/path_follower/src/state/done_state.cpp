/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file done_state.cpp
 *
 *@brief 完成的状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 08
 ************************************************************************/

#include "state/done_state.hpp"
#include "robot_info.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<DoneState> DoneState::ptr_done_state_ = nullptr;

DoneState::DoneState() {
  follow_state_ = PathFollowerStates::DONE;
}

std::shared_ptr<DoneState> DoneState::getPtrInstance() {
  if (ptr_done_state_.get() == nullptr) {
    ptr_done_state_.reset(new DoneState());
  }
  return ptr_done_state_;
}

FollowerStateRes DoneState::dealState(PathFollower *) {
  LOG(INFO) << " ^^^^^^^^^^^^^^^^^^^^ Done State ^^^^^^^^^^^^^^^^^^^^ ";
  RobotInfo::getPtrInstance()->setRobotMotionState(ROBOTMOTIONSTATE::STOP);
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = "";
  return follower_state;
}

}  // namespace CVTE_BABOT