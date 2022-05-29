/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file recover_state.cpp
 *
 *@brief 后退的状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev
 *@data 2021 - 02 - 20
 ************************************************************************/

#include "robot_info.hpp"
#include "path_follower.hpp"
#include "path_manager.hpp"
#include "state/recover_state.hpp"
#include "state/local_state.hpp"
#include "state/pause_state.hpp"
#include "state/rotate_state.hpp"
#include "planner_decision/planner_decision.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<RecoverState> RecoverState::ptr_recover_state_ = nullptr;

std::shared_ptr<RecoverState> RecoverState::getPtrInstance() {
  if (ptr_recover_state_.get() == nullptr) {
    ptr_recover_state_.reset(new RecoverState());
  }
  return ptr_recover_state_;
}

FollowerStateRes RecoverState::dealState(PathFollower *pf_ptr) {
  LOG(INFO) << " //////////////// RecoverState //////////////// ";
  RobotInfo::getPtrInstance()->setRobotMotionState(ROBOTMOTIONSTATE::RECOVER);
  std::shared_ptr<SubPath> path =
      std::make_shared<SubPath>();  // 后退状态无需特意检查哪条路径
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  SupervisorState state(current_pose, path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
  SupervisorChain::getPtrInstance()->superviseCycle(state);

  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = "";

  // 一直退到前方没有障碍物才会切换状态
  if (!pf_ptr->checkIfNeedToRecover()) {
    // 在贴边时，refer没有被阻挡，切到rotate后，切换回refer
    if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::EDGE &&
        into_state_ == PathFollowerStates::FOLLOW_REFERENCE) {
      if (PauseState::getPtrInstance()->reCheckReferPath(pf_ptr)) {
        LOG(WARNING) << "recover change to refer";
        // pf_ptr->getPmPtr()->updateReferIndex();
        pf_ptr->changeState(ReferState::getPtrInstance());
        return follower_state;
      }
    }
    // 后退完成后，触发一次从当前位置到目标点的重规划
    if (refindLocalTarget(pf_ptr)) {
      // if (pf_ptr->getPmPtr()->updateLocalPath()) {
      // LOG(INFO) << "Recover finish switch to LOCAL";
      // pf_ptr->changeState(LocalState::getPtrInstance());
      // }getLocalPath
      std::shared_ptr<SubPath> sub_path = pf_ptr->getPmPtr()->getLocalPath();
      if (sub_path == nullptr || sub_path->wps.empty()) {
        LOG(ERROR) << "sub_path is empty";
      }
      double target_yaw =
          sub_path->wps[pf_ptr->getPmPtr()->getLocalIndex()].getYaw();
      double diff_yaw =
          (RobotInfo::getPtrInstance()->getCurrentPose().inverse() *
           sub_path->wps[pf_ptr->getPmPtr()->getLocalIndex()])
              .getYaw();

      if (fabs(diff_yaw) > 0.09 &&
          RobotInfo::getPtrInstance()->getRobotType() ==
              ROBOTTYPE::KAVA_CLEAN_C3) {  // 5°
        LOG(WARNING) << "Recover finish switch to Rotate yaw: " << target_yaw
                     << " diff_yaw " << diff_yaw;
        RotateState::getPtrInstance()->setTargetAngleWithType(
            target_yaw, "rotate_from_local");
        pf_ptr->changeState(RotateState::getPtrInstance());
      } else {
        LOG(INFO) << "Recover finish switch to LOCAL";
        pf_ptr->changeState(LocalState::getPtrInstance());
      }

    } else {
      PauseState::getPtrInstance()->setPauseMessage(
          getState(), PathFollowerStates::FOLLOW_LOCAL, "NoTargetPoint");
      pf_ptr->changeState(PauseState::getPtrInstance());
    }
  } else {
    LOG(INFO) << "robot continue to recover.";
  }
  return follower_state;
}

bool RecoverState::refindLocalTarget(PathFollower *pf_ptr) {
  LOG(INFO) << "Retreat state refind better local target.";
  std::shared_ptr<PlannerDecision> ptr_planner_decision =
      PlannerDecision::getInstance();
  PoseMsg target_pose;
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = "";
  if (ptr_planner_decision->getTargetPoint(current_pose, pf_ptr, follower_state,
                                           target_pose)) {
    return pf_ptr->getPmPtr()->updateLocalPath(target_pose);
  } else {
    return false;
  }
}

void RecoverState::clearIntoState() {
  into_state_ = PathFollowerStates::FREE;
}

void RecoverState::setIntoState(PathFollowerStates state) {
  into_state_ = state;
}
}  // namespace CVTE_BABOT
