/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file refer_state.hpp
 *
 *@brief 跟踪任务路径的状态
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 07
 ************************************************************************/

#ifndef __REFER_STATE_HPP
#define __REFER_STATE_HPP

#include <iostream>
#include <mutex>
#include "state.hpp"
#include "path.hpp"

namespace CVTE_BABOT {

class ReferState : public State {
 public:
  ~ReferState() = default;
  static std::shared_ptr<ReferState> getPtrInstance();

  FollowerStateRes dealState(PathFollower *pf_ptr) final;

  /**
   *setPauseTime
   *@brief
   *  设置开始绕障暂停时间和触发暂停后退的时间
   *
   *@param[in] avoid_sce - 触发绕障前等待时间
   *@param[in] recover_sec - 触发恢复前的等待时间
   **/
  void setPauseTime(const int &avoid_sec, const int &recover_sec,
                    const int &unreach_wait_sec);

  void setFinalRotateFlag(bool need_final_rotate) {
    need_final_rotate_ = need_final_rotate;
  }

 private:
  /**
   *prepareLocalPath
   *@brief
   *   通过监督器查找到的目标点，更新局部路径
   *
   *@param[in] pf_ptr - PathFollower类指针
   *@param[in] start - 监督器内查找到的绕障起点
   *@param[in] target - 监督器内查找到的绕障目标点
   *@return true - 代表规划成功
   **/
  bool prepareLocalPath(PathFollower *pf_ptr, const PoseMsg &start,
                        const PoseMsg &target);
  /*重载版本是默认从机器人当前位置到目标点更新路径，不需要明确定义起点*/
  bool prepareLocalPath(PathFollower *pf_ptr, const PoseMsg &target);

  /**
   *pauseAMonent
   *@brief
   *   进入暂停状态，针对一些异常情况
   *
   *@param[in] pf_ptr - PathFollower类指针
   *@param[in] sup_name - 进入暂停状态的监督器名称
   **/
  void pauseAMonent(PathFollower *pf_ptr, const std::string &sup_name);

  ReferState() { follow_state_ = PathFollowerStates::FOLLOW_REFERENCE; }
  ReferState(const ReferState &obj) = delete;
  ReferState &operator=(const ReferState &obj) = delete;

  static std::shared_ptr<ReferState> ptr_refer_state_;

  int recover_wait_second_ = 3;  // 停障后到倒车绕障的等待时间（单位：秒）
  int avoid_wait_second_ =
      0;  // 前方有障碍物时，先等待这个时间再触发绕障（单位：秒）
  int unreach_wait_second_ = 0;  //占用被占用时,等待时长

  bool need_final_rotate_ = false;
};

}  // namespace CVTE_BABOT

#endif  // end of __REFER_STATE_HPP