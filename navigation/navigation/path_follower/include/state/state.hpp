/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file state.hpp
 *
 *@brief 状态模式各个状态的基类
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 07
 ************************************************************************/

#ifndef __BASE_STATE_HPP
#define __BASE_STATE_HPP

#include <mutex>
#include "type.hpp"

namespace CVTE_BABOT {

class PathFollower;

/*********************************************
 使用状态模式，每个状态对应一个类来处理内部不同逻辑处理
*********************************************/

class State {
 public:
  virtual ~State() {}
  /**
  *dealState
  *@brief
  *  纯虚的接口函数，每个状态下都在此函数内完成相应操作
  *
  *@param[in] pf_ptr - PathFollwer的类指针，用于状态内修改数据
  **/
  virtual FollowerStateRes dealState(PathFollower* pf_ptr) = 0;

  /**
  *getState
  *@brief
  *  获取状态指针内部目前指向的状态
  *
  *@return 状态枚举类型
  **/
  PathFollowerStates getState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return follow_state_;
  }

  void setState(const PathFollowerStates& state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    follow_state_ = state;
  }

 protected:
  std::mutex state_mutex_;
  PathFollowerStates follow_state_ = PathFollowerStates::FREE;
};

}  // end of namespace

#endif  // end of __BASE_STATE_HPP