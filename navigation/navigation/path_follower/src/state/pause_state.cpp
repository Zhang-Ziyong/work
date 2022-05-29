/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file pause_state.cpp
 *
 *@brief 暂停状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 09
 ************************************************************************/

#include "state/pause_state.hpp"
#include "state/recover_state.hpp"
#include "state/done_state.hpp"
#include "state/elevator_done_state.hpp"
#include "state/state_utils.hpp"
#include "path_follower.hpp"
#include "pnc_map.hpp"
#include "robot_info.hpp"
#include "planner_decision/planner_decision.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<PauseState> PauseState::ptr_pause_state_ = nullptr;

std::shared_ptr<PauseState> PauseState::getPtrInstance() {
  if (ptr_pause_state_.get() == nullptr) {
    ptr_pause_state_.reset(new PauseState());
  }
  return ptr_pause_state_;
}

FollowerStateRes PauseState::dealState(PathFollower *pf_ptr) {
  LOG(INFO) << " ******************** PAUSE State ******************** ";

  RobotInfo::getPtrInstance()->setRobotMotionState(ROBOTMOTIONSTATE::PAUSE);
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = pause_reason_;
  if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
    PncMap pnc_map;
    std::vector<cvte::hdmap::Polygon2d> elevator_polygons =
        pnc_map.GetElevatorArea();
    for (const auto &it : elevator_polygons) {
      std::vector<Eigen::Vector2d> stop_shape =
          SpeedDecisionBase::getInstance()->getBoundingbox("stop");
      Pose2d cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
      bool in_elevator_flag = true;
      for (const auto &it : stop_shape) {
        Pose2d point(it.x(), it.y(), 0.0);
        Pose2d cur_point = cur_pose * point;
        if (!pnc_map.InElevatorArea(
                math_utils::Vec2d(cur_point.getX(), cur_point.getY()))) {
          in_elevator_flag = false;
          break;
        }
      }
      if (in_elevator_flag) {
        LOG(INFO) << "elevator pause change to done";
        pf_ptr->changeState(ElevatorDoneState::getPtrInstance());
        return follower_state;
      }
    }
  }
  if (pause_reason_ == "OverTime") {
    if (!is_abs_reach_) {
      LOG(INFO) << "OverTime change to Done";
      pf_ptr->changeState(DoneState::getPtrInstance());
    }
  } else if (pause_reason_ == "PointOccupy") {
    if (is_abs_reach_) {
      if (once_pause_count_-- > 0) {
        LOG(INFO) << "Pause count: " << once_pause_count_
                  << " Reason: " << pause_reason_;
      } else {
        reCheck(pf_ptr);
      }
    } else {
      if (every_pause_count_-- > 0) {
        LOG(INFO) << "Pause count: " << every_pause_count_
                  << " Reason: " << pause_reason_;
        reCheck(pf_ptr);
      } else {
        LOG(INFO) << "point occupy change to done state";
        pf_ptr->changeState(DoneState::getPtrInstance());
      }
    }
    return follower_state;
  } else {
    if (once_pause_count_-- > 0) {
      LOG(INFO) << "Pause count: " << once_pause_count_
                << " Reason: " << pause_reason_;
    } else {
      reCheck(pf_ptr);
    }
    return follower_state;
  }
}
void PauseState::setPlannerFrequence(const int &frequency) {
  fequency_ = frequency;
  LOG(INFO) << "set pause fequency: " << fequency_;
}

void PauseState::setPauseMessage(const PathFollowerStates &into_state,
                                 const PathFollowerStates &target_state,
                                 const std::string &msg,
                                 const unsigned int &count) {
  into_state_ = into_state;
  target_state_ = target_state;
  pause_reason_ = msg;
  if (pause_reason_ == "PointOccupy" && !is_abs_reach_) {
    every_pause_count_ = count * fequency_;
    once_pause_count_ = count * fequency_;
  } else {
    every_pause_count_ = fequency_;
    once_pause_count_ = fequency_;
  }
  LOG(INFO) << "into state: " << static_cast<int>(into_state_)
            << " target_state: " << static_cast<int>(target_state)
            << " pause_reason: " << pause_reason_ << " pause count: " << count;
}

void PauseState::resume(PathFollower *pf_ptr) {
  // TODO: 贴边判断一下是否可以旋转
  // 检查完恢复前，优先判断一下前面是否有障碍物，有则进入后退，没有时才恢复
  if (pf_ptr->checkIfNeedToRecover()) {
    LOG(INFO) << "resume to RECOVER.";
    // refindLocalTarget(pf_ptr);
    pf_ptr->recoverClear();
    RecoverState::getPtrInstance()->setIntoState(into_state_);
    pf_ptr->changeState(RecoverState::getPtrInstance());
  } else {
    LOG(INFO) << "resume to target state." << static_cast<int>(target_state_);
    pf_ptr->changeState(getStateInstance(target_state_));
  }
}

void PauseState::reCheck(PathFollower *pf_ptr) {
  // 根据上个状态来检查局部路径或全局路径
  LOG(INFO) << "into state: " << static_cast<int>(into_state_);
  std::shared_ptr<SubPath> path;
  Pose2d cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  PncMap pnc_map;
  bool in_elevator = pnc_map.InElevatorArea(
      math_utils::Vec2d(cur_pose.getX(), cur_pose.getY()));
  switch (into_state_) {
    case PathFollowerStates::RECOVER:
    case PathFollowerStates::FOLLOW_LOCAL: {
      path = pf_ptr->getPmPtr()->getLocalPath();
      // if (needToUpdateTarget(pf_ptr)) {
      if (refindLocalTarget(pf_ptr)) {
        if (!pf_ptr->getPmPtr()->updateLocalPath()) {
          LOG(ERROR) << "Replan at pause state error.";
        }
        if (pause_reason_ == "NoTargetPoint") {
          path = nullptr;
        } else {
          path = pf_ptr->getPmPtr()->getLocalPath();
        }
        // 检查完恢复前，优先判断一下前面是否有障碍物，有则进入后退，没有时才恢复
        if (pf_ptr->checkIfNeedToRecover()) {
          LOG(INFO) << "goto recover before Local";
          // refindLocalTarget(pf_ptr);
          pf_ptr->recoverClear();
          pf_ptr->changeState(RecoverState::getPtrInstance());
        }
      } else if (!in_elevator) {
        // 局部路径寻找失败 进入脱困部分
        LOG(ERROR) << "can not find local";
        if (pf_ptr->checkIfNeedToRecover()) {
          LOG(INFO) << "can not find local resume to RECOVER.";
          pf_ptr->recoverClear();
          pf_ptr->changeState(RecoverState::getPtrInstance());
        }
      }
    } break;

    case PathFollowerStates::FOLLOW_REFERENCE: {
      if (reCheckReferPath(pf_ptr)) {
        // 暂停完成后如果refer路径的异常已经消除，则直接使用refer路径继续
        LOG(INFO) << "Refer is ok again!";
        path = pf_ptr->getPmPtr()->getReferPath();
      } else if (pause_reason_ == "ObstacleAvoider" ||
                 pause_reason_ == "NoTargetPoint" ||
                 pause_reason_ == "PathFaraway" ||
                 pause_reason_ == "ObstacleStop") {
        LOG(INFO) << "Refer update local target.";
        // target_state_ = PathFollowerStates::FOLLOW_LOCAL;
        if (refindLocalTarget(pf_ptr)) {
          path = pf_ptr->getPmPtr()->getLocalPath();
          target_state_ = PathFollowerStates::FOLLOW_LOCAL;
          resume(pf_ptr);
        } else if (!in_elevator) {
          path = pf_ptr->getPmPtr()->getReferPath();
          // 路径搜索失败  先切换至recover检查机器身边的环境
          LOG(ERROR) << "can not find local,into_state: refer";
          // FIXME: 不能直接调用此接口 没有找到路径进入local会导致崩溃
          // resume(pf_ptr);
          if (pf_ptr->checkIfNeedToRecover()) {
            LOG(INFO) << "resume to RECOVER.";
            // refindLocalTarget(pf_ptr);
            pf_ptr->recoverClear();
            RecoverState::getPtrInstance()->setIntoState(into_state_);
            pf_ptr->changeState(RecoverState::getPtrInstance());
            return;
          }
        }
      } else {
        path = pf_ptr->getPmPtr()->getReferPath();
      }
    } break;
    case PathFollowerStates::PASS_PIT: {
      if (reCheckPitPath(pf_ptr)) {
        // 暂停完成后如果refer路径的异常已经消除，则直接使用refer路径继续
        LOG(INFO) << "Pit is ok again!";
        resume(pf_ptr);
        return;
      }
    } break;

    default:
      break;
  }

  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  SupervisorState state(current_pose, path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
  auto res = SupervisorChain::getPtrInstance()->supervise(state);
  //超时监督单独再进行一次，如果超时，直接返回
  auto overtime_res =
      SupervisorChain::getPtrInstance()->superviseAssignOne(state, "OverTime");
  if (!overtime_res.can_continue) {
    pause_reason_ = res.supervisor_name;
    LOG(INFO) << "pause over time";
    return;
  }

  // 如果除了停障和终点导致的暂停，其它异常需要继续等待。
  // （停障除外是因为需要恢复到后退状态解除停障, 终点停止是要恢复旋转到达）
  LOG(INFO) << "can_continue " << res.can_continue << " status " << res.status;
  if (!res.can_continue &&
      ((res.status != OBSTACLESTOP && res.status != FINALGOAL) ||
       in_elevator)) {
    if (res.supervisor_name == pause_reason_) {
      LOG(INFO) << "Still at " << res.supervisor_name << " state, waitting...";
    } else {
      LOG(INFO) << "Change other " << res.supervisor_name
                << " state, waitting...";
      pause_reason_ = res.supervisor_name;
    }
    once_pause_count_ = every_pause_count_;  // 再等待一次轮回
  } else {
    LOG(INFO) << pause_reason_ << " is clear.";
    resume(pf_ptr);
  }
}

bool PauseState::reCheckReferPath(PathFollower *pf_ptr) {
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  std::shared_ptr<SubPath> refer_path = pf_ptr->getPmPtr()->getReferPath();
  SupervisorState state(current_pose, refer_path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
  auto res = SupervisorChain::getPtrInstance()->supervise(state);
  return res.can_continue;
}

bool PauseState::reCheckPitPath(PathFollower *pf_ptr) {
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  std::shared_ptr<SubPath> sub_path = pf_ptr->getPmPtr()->getLocalPath();
  SupervisorState state(current_pose, sub_path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
  auto res = SupervisorChain::getPtrInstance()->superviseAssignOne(
      state, "ObstacleAvoider");
  return res.can_continue;
}

bool PauseState::needToUpdateTarget(PathFollower *pf_ptr) {
  Pose2d current_target = pf_ptr->getPmPtr()->getLocalTarget().position;
  Pose2d last_pose = pf_ptr->getPmPtr()->getReferLastPoint();

  // 如果当前的绕障目标点已经是最后的终点，则不需要再往后搜索更新
  if (current_target == last_pose) {
    LOG(WARNING) << "Current target is last point.";
    return false;
  }

  auto costmap = pf_ptr->getPmPtr()->getCostmap();
  int value =
      costmap->getCostWithMapPose(current_target.getX(), current_target.getY());
  LOG(INFO) << "Current target value: " << value;
  return (value != 0);  // 如果此时目标点代价值不为零，则需要更新当前目标点
}

bool PauseState::refindLocalTarget(PathFollower *pf_ptr) {
  LOG(INFO) << "Pause state refind local target";
  std::shared_ptr<PlannerDecision> ptr_planner_decision =
      PlannerDecision::getInstance();
  PoseMsg target_pose;
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
    std::shared_ptr<SubPath> refer_path = pf_ptr->getPmPtr()->getReferPath();
    //取倒数第二个点作为目标点，倒数第一个点的姿态有问题
    target_pose.position = refer_path->wps[refer_path->wps.size() - 2];
    target_pose.velocity = WayPointInfo(0.0, 0.0, 0.0);
    target_pose.index = refer_path->wps.size() - 2;
    return pf_ptr->getPmPtr()->updateLocalPath(target_pose);
  }
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = pause_reason_;
  if (ptr_planner_decision->getTargetPoint(current_pose, pf_ptr, follower_state,
                                           target_pose)) {
    PncMap pnc_map;
    if (pnc_map.InElevatorArea(
            math_utils::Vec2d(current_pose.getX(), current_pose.getY())) &&
        pnc_map.InElevatorArea(math_utils::Vec2d(
            target_pose.position.getX(), target_pose.position.getY()))) {
      LOG(INFO) << "target in elevator, throw";
      return false;
    }
    return pf_ptr->getPmPtr()->updateLocalPath(target_pose);
  } else {
    return false;
  }
}

}  // namespace CVTE_BABOT