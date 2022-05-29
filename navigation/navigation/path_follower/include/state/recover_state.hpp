/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file recover_state.hpp
 *
 *@brief 后退状态
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2021 - 02 - 20
 ************************************************************************/

#ifndef __RECOVER_STATE_HPP
#define __RECOVER_STATE_HPP

#include "state.hpp"
#include "type.hpp"

namespace CVTE_BABOT {

class RecoverState : public State {
 public:
  ~RecoverState() = default;

  static std::shared_ptr<RecoverState> getPtrInstance();

  FollowerStateRes dealState(PathFollower *) final;
  void StateClear(PathFollower *pf_ptr);

  void setIntoState(PathFollowerStates state);
  void clearIntoState();

 private:
  RecoverState() { follow_state_ = PathFollowerStates::RECOVER; };
  RecoverState(const RecoverState &obj) = delete;
  RecoverState &operator=(const RecoverState &obj) = delete;
  bool refindLocalTarget(PathFollower *pf_ptr);

  static std::shared_ptr<RecoverState> ptr_recover_state_;
  PathFollowerStates into_state_ = PathFollowerStates::FREE;
};

}  // namespace CVTE_BABOT

#endif  // end of __RECOVER_STATE_HPP
