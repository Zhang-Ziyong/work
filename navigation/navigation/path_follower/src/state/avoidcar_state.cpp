/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file avoidcar_state.cpp
 *
 *@brief 车辆避让的状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020-07-08
 ************************************************************************/

#include "state/avoidcar_state.hpp"
#include "path_follower.hpp"
#include "path_manager.hpp"
#include "robot_info.hpp"
#include "state/local_state.hpp"
#include "state/pause_state.hpp"
#include "state/refer_state.hpp"
#include "state/state_utils.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<AvoidCarState> AvoidCarState::getPtrInstance() {
  static std::shared_ptr<AvoidCarState> ptr_avoidcar_state =
      std::shared_ptr<AvoidCarState>(new AvoidCarState);
  return ptr_avoidcar_state;
}

FollowerStateRes AvoidCarState::dealState(PathFollower *pf_ptr) {
  LOG(INFO) << " ~~~~~~~~~~~~~~~~~~~~ Avoid car State ~~~~~~~~~~~~~~~~~~~~ ";

  FollowerStateRes follower_state;
  follower_state.states = getState();

  if (isCloseEnoughToGoal(pf_ptr)) {
    // 进行等待
    LOG(INFO) << "Finish avoid car, wait for car to leave and resume to "
                 "patrol. ***********";
    follower_state.msg = "wait for car leaving";

    auto target_state = prepareGoToRefer(pf_ptr);
    PauseState::getPtrInstance()->setPauseMessage(getState(),
        target_state, follower_state.msg, ui_wait_count_);

    pf_ptr->changeState(PauseState::getPtrInstance());
    return follower_state;
  }

  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  SubPath local_path = pf_ptr->getPmPtr()->getLocalPath();
  SupervisorState state(current_pose, local_path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);

  auto res = SupervisorChain::getPtrInstance()->superviseCycle(state);

  follower_state.msg = "avoid car";

  if (!res.can_continue) {
    LOG(ERROR) << "the path to avoid car is invalid, supervisor_name: "
               << res.supervisor_name;

    // 返回远离路径是否也重新规划？
    if (!pf_ptr->getPmPtr()->updateLocalPath()) {
      LOG(ERROR) << "Failed to replan to avoid car, recover to refer path.";

      follower_state.msg = "recover to refer path";
      auto target_state = prepareGoToRefer(pf_ptr);

      pf_ptr->changeState(getStateInstance(target_state));
    }
  }

  return follower_state;
}

PathFollowerStates AvoidCarState::prepareGoToRefer(PathFollower *pf_ptr) {
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  SubPath target_path = pf_ptr->getPmPtr()->getReferPath();
  SupervisorState state(current_pose, target_path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);

  auto res = SupervisorChain::getPtrInstance()->superviseCycle(state);

  double min_dist = 1e10;
  unsigned int index = 0;
  if (!res.can_continue) {
    LOG(INFO) << "The robot state is " << res.supervisor_name
              << ", change to recover to the refer.";

    pf_ptr->getPmPtr()->setLocalTarget(res.target_point);
  } else {
    for (size_t i = 0; i < target_path.wps.size(); ++i) {
      double dist = state.robot_pose_.distanceTo(target_path.wps[i]);
      if (dist < min_dist) {
        min_dist = dist;
        index = i;
      }
    }

    // pf_ptr->getPmPtr()->setLocalTarget(target_path.wps[index]);
  }

  if (pf_ptr->getPmPtr()->updateLocalPath()) {
    LOG(ERROR) << "Failed to recover to the refer path, remain car avoiding "
                  "state.!!";
  }

  return LocalState::getPtrInstance()->getState();
}

bool AvoidCarState::isCloseEnoughToGoal(PathFollower *pf_ptr) {
  Pose2d current = RobotInfo::getPtrInstance()->getCurrentPose();
  Pose2d target = pf_ptr->getPmPtr()->getLocalTarget().position;
  double distance = current.distanceTo(target);
  LOG(INFO) << "goal dist: " << distance << "m T(" << target.getX() << ", "
            << target.getY() << ")";

  double angle_error = fabs(current.getYaw() - target.getYaw());

  // 距离与角度同时满足,距离应与local_controller中的保持一致，这里先给定
  if (distance < 0.25 && angle_error < d_angle_tolerance_) {  // 加角度
    return true;
  } else {
    return false;
  }
}

}  // namespace CVTE_BABOT