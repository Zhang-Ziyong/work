/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file free_state.hpp
 *
 *@brief 空闲的状态，代表目前没有任务路径要执行
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 07
 ************************************************************************/


#ifndef __FREE_STATE_HPP
#define __FREE_STATE_HPP

#include "type.hpp"
#include "state.hpp"
#include <mutex>

namespace CVTE_BABOT{

class FreeState : public State {
public:
  ~FreeState() {};
  static std::shared_ptr<FreeState> getPtrInstance();

  FollowerStateRes dealState(PathFollower* ) final;

private:
  FreeState();
  FreeState(const FreeState &obj) = delete;
  FreeState &operator= (const FreeState &obj) = delete;
  static std::shared_ptr<FreeState> ptr_free_state_;
};

} // end of namespace 

#endif // end of __FREE_STATE_HPP