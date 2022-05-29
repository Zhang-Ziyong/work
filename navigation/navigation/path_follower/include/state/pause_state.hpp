/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file pause_state.hpp
 *
 *@brief 暂停状态
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 09
 ************************************************************************/

#ifndef __PAUSE_STATE_HPP
#define __PAUSE_STATE_HPP

#include <mutex>
#include "state.hpp"
#include "type.hpp"

namespace CVTE_BABOT {

class PauseState : public State {
 public:
  ~PauseState() = default;
  static std::shared_ptr<PauseState> getPtrInstance();

  /**
   *setPauseMessage
   *@brief
   *   设置进入暂停状态前的上一个状态与此次暂停时间
   *   count倒数到0的时候，自动恢复到target_state状态
   *   msg此次进入暂停状态的原因
   *
   *@param[in] into_state - 进入暂停的状态
   *@param[in] target_state - 暂停结束后需要恢复的目标状态
   *@param[in] msg - 此次进入暂停的原因，用于告知上层
   *@param[in] count - 暂停计数器，为零则切回。频率与线程频率一致, 默认暂停2秒
   **/
  void setPauseMessage(const PathFollowerStates &into_state,
                       const PathFollowerStates &target_state,
                       const std::string &msg, const unsigned int &count = 2);

  /**
   *setPlannerFrequence
   *@brief
   *  设置状态处理的频率，用于计算暂停倒计时计数值。
   *
   *@param[in] frequency - 处理状态的频率
   **/
  void setPlannerFrequence(const int &frequency);

  void setAbsReachFlag(bool is_abs_reach) { is_abs_reach_ = is_abs_reach; }

  /**
   *reCheckReferPath
   *@brief
   *  再次检查全局任务路径是否异常消除可正常执行
   *
   *@param[in] pf_ptr - PathFollower类指针
   **/
  bool reCheckReferPath(PathFollower *pf_ptr);

  FollowerStateRes dealState(PathFollower *pf_ptr) final;

 private:
  /**
   *resume
   *@brief
   *  恢复进入暂停状态前的情况
   *
   *@param[in] pf_ptr - PathFollower类指针
   **/
  void resume(PathFollower *pf_ptr);

  /**
   *needToUpdateTarget
   *@brief
   *  检查一下当前目标点情况，是否被占用或者超出地图范围
   *  如果是则需要更新此时目标点
   *
   *@param[in] pf_ptr - PathFollower类指针
   **/
  bool needToUpdateTarget(PathFollower *pf_ptr);

  /**
   *refindLocalTarget
   *@brief
   *  从新搜索此时合适的绕障目标点
   *
   *@param[in] pf_ptr - PathFollower类指针
   **/
  bool refindLocalTarget(PathFollower *pf_ptr);

  /**
   *reCheck
   *@brief
   *  此次暂停时间结束后，再做一次检查，看是否可以恢复到上次状态
   *
   *@param[in] pf_ptr - PathFollower类指针
   **/
  void reCheck(PathFollower *pf_ptr);

  /**
   *reCheckReferPath
   *@brief
   *  再次检查过坎任务路径是否异常消除可正常执行
   *
   *@param[in] pf_ptr - PathFollower类指针
   **/
  bool reCheckPitPath(PathFollower *pf_ptr);

  PauseState() { follow_state_ = PathFollowerStates::PAUSE; }
  PauseState(const PauseState &obj) = delete;
  PauseState &operator=(const PauseState &obj) = delete;

  static std::shared_ptr<PauseState> ptr_pause_state_;
  PathFollowerStates into_state_;    ///< 保存进入暂停前的状态
  PathFollowerStates target_state_;  ///< 保存暂停后需要恢复的状态
  int every_pause_count_ = 0;  ///< 每次暂停的倒计时,按间隔做重新检查
  int once_pause_count_ = 0;       ///< 此次暂停的时间计数
  int fequency_ = 0;               ///< 此次暂停的时间计数
  std::string pause_reason_ = "";  ///< 暂停的理由

  bool is_abs_reach_ = true;
};

}  // namespace CVTE_BABOT

#endif  // end of __PAUSE_STATE_HPP