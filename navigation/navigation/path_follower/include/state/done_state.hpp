/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file done_state.hpp
 *
 *@brief 完成任务的状态
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 08
 ************************************************************************/


#ifndef __DONE_STATE_HPP
#define __DONE_STATE_HPP

#include "type.hpp"
#include "state.hpp"
#include <mutex>

namespace CVTE_BABOT{

class DoneState : public State {
public:
  ~DoneState() = default;
  static std::shared_ptr<DoneState> getPtrInstance();

  FollowerStateRes dealState(PathFollower* ) final;

private:
  DoneState();
  DoneState(const DoneState &obj) = delete;
  DoneState &operator= (const DoneState &obj) = delete;

  static std::shared_ptr<DoneState> ptr_done_state_;
};
} // end of namespace 

#endif // end of __DONE_STATE_HPP