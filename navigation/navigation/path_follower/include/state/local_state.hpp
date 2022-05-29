/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file local_state.hpp
 *
 *@brief 局部路径的跟踪状态，一般用于绕障和车辆避让时候
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 07
 ************************************************************************/

#ifndef __LOCAL_STATE_HPP
#define __LOCAL_STATE_HPP

#include <mutex>
#include "state.hpp"
#include "type.hpp"

namespace CVTE_BABOT {

class LocalState : public State {
 public:
  ~LocalState() = default;
  static std::shared_ptr<LocalState> getPtrInstance();
  FollowerStateRes dealState(PathFollower *pf_ptr) final;

 private:
  /**
   *isCloseEnoughToGoal
   *@brief
   *   判断是否已经靠近此次绕障目标点，如果是则切换回Refer状态
   *   内部加了一个逻辑处理，如果此次是普通绕障，则可以提前切回
   *   如果此次目标点是任务路径最后一个点，则要靠得比较近才切回
   *
   *@param[in] pf_ptr - PathFollower类指针
   *@return true - 代表与目标点的直线距离已经在允许范围内
   **/
  bool isCloseEnoughToGoal(PathFollower *pf_ptr);

  bool prepareLocalPath(PathFollower *pf_ptr, const PoseMsg &target);

  bool findBetterTarget(PathFollower *pf_ptr);

  /**
   *pauseAMonent
   *@brief
   *   进入暂停状态，针对一些异常情况
   *
   *@param[in] pf_ptr - PathFollower类指针
   *@param[in] sup_name - 进入暂停状态的监督器名称
   **/
  void pauseAMonent(PathFollower *pf_ptr, const std::string &sup_name);

  /**
   *judgeLastPoint
   *@brief
   *   判断此次绕障目标点是否为任务路径的最后一点，如果是则
   *   允许绕障到比较接近时才能切换状态
   *
   *@param[in] temp - 当前绕障目标点
   *@param[in] target - 任务路径的最后一点
   *@return 此次绕障的允许停靠距离
   **/
  double judgeLastPoint(const Pose2d &temp, const Pose2d &target);

  /**
   *executeSupervise
   *@brief
   *   根据传入的执行一次监督检查操作
   *
   *@param[in] pf_ptr - PathFollower类指针
   *@param[in] path - 需要检查的路径
   *@return 监督结果
   **/
  SupervisorResult executeSupervise(PathFollower *pf_ptr,
                                    std::shared_ptr<SubPath> path);

  LocalState() { follow_state_ = PathFollowerStates::FOLLOW_LOCAL; }
  LocalState(const LocalState &obj) = delete;
  LocalState &operator=(const LocalState &obj) = delete;

  static std::shared_ptr<LocalState> ptr_local_state_;
};

}  // namespace CVTE_BABOT

#endif  // end of __LOCAL_STATE_HPP