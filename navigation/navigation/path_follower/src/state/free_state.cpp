/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file free_state.cpp
 *
 *@brief 空闲的状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 08
 ************************************************************************/

#include "state/free_state.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<FreeState> FreeState::ptr_free_state_ = nullptr;

FreeState::FreeState() { follow_state_ = PathFollowerStates::FREE; }

std::shared_ptr<FreeState> FreeState::getPtrInstance() {
  if (ptr_free_state_.get() == nullptr) {
    ptr_free_state_.reset(new FreeState());
  }
  return ptr_free_state_;
}

FollowerStateRes FreeState::dealState(PathFollower *) {
  LOG(INFO) << " #################### FreeState #################### ";
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = "";

  return follower_state;
}


}  // namespace CVTE_BABOT