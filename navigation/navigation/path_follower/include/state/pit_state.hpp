/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file pit_planner.hpp
 *
 *@brief 过坎状态头文件
 *
 *@modified by
 *
 *@author lizhongjia(lizhongjia@cvte.com)
 *@version Navigation-v2.0
 *@data 2022-04-16
 ************************************************************************/

#ifndef __PIT_STATE_HPP
#define __PIT_STATE_HPP

#include <iostream>
#include <mutex>
#include "state.hpp"
#include "path.hpp"

namespace CVTE_BABOT {

class PitState : public State {
 public:
  ~PitState() = default;
  static std::shared_ptr<PitState> getPtrInstance();

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
  void setParams(double edge_dist);

  void setFinalRotateFlag(bool need_final_rotate) {
    need_final_rotate_ = need_final_rotate;
  }

 private:
  SupervisorResult executeSupervise(PathFollower *pf_ptr,
                                    std::shared_ptr<SubPath> path);

  bool isCloseEnoughToGoal(PathFollower *pf_ptr);

  bool isSafe(PathFollower *pf_ptr, size_t target_index);

  bool isInPit(PathFollower *pf_ptr, double forward_dist, double backward_dist);

  bool isVerticalPit(const CVTE_BABOT::Pose2d &start,
                     const CVTE_BABOT::Pose2d &target);
  /**
   *pauseAMonent
   *@brief
   *   进入暂停状态，针对一些异常情况
   *
   *@param[in] pf_ptr - PathFollower类指针
   *@param[in] sup_name - 进入暂停状态的监督器名称
   **/
  void pauseAMonent(PathFollower *pf_ptr, const std::string &sup_name);

  PitState() { follow_state_ = PathFollowerStates::PASS_PIT; }
  PitState(const PitState &obj) = delete;
  PitState &operator=(const PitState &obj) = delete;

  static std::shared_ptr<PitState> ptr_refer_state_;

  int recover_wait_second_ = 3;  // 停障后到倒车绕障的等待时间（单位：秒）
  int avoid_wait_second_ = 0;  // 前方有障碍物时等待延迟后触发绕障（单位：秒）
  int unreach_wait_second_ = 0;  //占用被占用时,等待时长
  double pit_edge_dist_ = 0;     ///< 过坎点与坎边界的间距
  bool need_final_rotate_ = false;
};

}  // namespace CVTE_BABOT

#endif  // end of __PIT_STATE_HPP