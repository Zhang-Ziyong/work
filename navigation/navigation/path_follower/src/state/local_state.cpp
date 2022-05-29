/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file local_state.cpp
 *
 *@brief 绕障路径的状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 08
 ************************************************************************/

#include "path_follower.hpp"
#include "path_manager.hpp"
#include "robot_info.hpp"
#include "state/rotate_state.hpp"
#include "state/local_state.hpp"
#include "state/avoidcar_state.hpp"
#include "state/pause_state.hpp"
#include "state/pit_state.hpp"
#include "state/refer_state.hpp"
#include "state/recover_state.hpp"
#include "speed_decision_base.hpp"
#include "planner_decision/planner_decision.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<LocalState> LocalState::ptr_local_state_ = nullptr;

std::shared_ptr<LocalState> LocalState::getPtrInstance() {
  if (ptr_local_state_.get() == nullptr) {
    ptr_local_state_.reset(new LocalState());
  }
  return ptr_local_state_;
}

FollowerStateRes LocalState::dealState(PathFollower *pf_ptr) {
  LOG(INFO) << " ==================== Local State ==================== ";
  // 判断是否已经完成此次绕障
  if (isCloseEnoughToGoal(pf_ptr)) {
    LOG(INFO) << "finish local path, checkto REFERENCE.";
    pf_ptr->getPmPtr()->updateReferIndex();
    pf_ptr->changeState(ReferState::getPtrInstance());
  }

  auto res = executeSupervise(pf_ptr, pf_ptr->getPmPtr()->getLocalPath());
  FollowerStateRes follower_state;
  follower_state.states = getState();
  if (res.can_continue) {
    // 这里一直输入全局路径来更新此时绕障目标点，目标点被占用时不切换暂停（后续Refer状态会处理）
    // 最好还是分开两个函数做，目前相似度较高暂时合并
    findBetterTarget(pf_ptr);
    return follower_state;
  }

  switch (res.status) {
    // 远离路径和需要绕障，处理方式都是重新规划，若失败则去暂停状态
    case DISTANCETOPATH:
    case OBSTACLEAVOIDER: {
      //电梯任务规划路径前先进入暂停状态
      if (RobotInfo::getPtrInstance()->getMissionType() ==
          MISSIONTYPE::ELVATOR) {
        LOG(INFO) << "elevator Obstacle avoider switch to Pause";
        PauseState::getPtrInstance()->setPauseMessage(getState(), getState(),
                                                      res.supervisor_name, 0);
        pf_ptr->changeState(PauseState::getPtrInstance());
        return follower_state;
      }
      // 当机器人正前方停障框内有障碍物，则切换到倒退状态
      if (SpeedDecisionBase::getInstance()->checkFrontDanger()) {
        LOG(INFO) << "ObstacleAvoider switch to RETREAT";
        pf_ptr->recoverClear();
        pf_ptr->changeState(RecoverState::getPtrInstance());
      }
      // findBetterTarget(pf_ptr, /*care_unreachable*/ true);
      //当发现绕障路径上有障碍物时，则重新规划绕障路径
      if (pf_ptr->getPmPtr()->updateLocalPath()) {
        LOG(INFO) << "Replan a path to avoid obstacle.";
        // 规划完路径后，需要判断轨迹是否处于窄道，并且判断是否需要倒退
        {
          std::shared_ptr<SubPath> l_path = pf_ptr->getPmPtr()->getLocalPath();
          Pose2d current_pose =
              RobotInfo::getPtrInstance()->getCurrentPoseWithOffset();
          SupervisorState tmp_state(current_pose, l_path, getState(),
                                    pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
          auto tmp_res = SupervisorChain::getPtrInstance()->superviseAssignOne(
              tmp_state, "MarkerMap");
          if (tmp_res.can_continue) {
            return follower_state;
          }

          //当在窄道内，并且不能直接行走的时候，将窄道路径反向，使机器人退出窄道
          if (tmp_res.start_point.index < 0 ||
              tmp_res.start_point.index >= l_path->wps.size() ||
              tmp_res.target_point.index < 0 ||
              tmp_res.target_point.index >= l_path->wps.size()) {
            LOG(WARNING) << "local path size " << int(l_path->wps.size())
                         << ", res start index " << tmp_res.start_point.index
                         << " or target index " << tmp_res.target_point.index
                         << " is invalid !";
          } else {
            const auto &sp = l_path->wps[tmp_res.start_point.index];
            const auto &ep = l_path->wps[tmp_res.target_point.index];
            LOG(INFO) << "# marker state # reverse refer path idx from "
                      << tmp_res.start_point.index << " to "
                      << tmp_res.target_point.index << ", position from ["
                      << sp.getX() << "," << sp.getY() << "] to [" << ep.getX()
                      << "," << ep.getY() << "]";
            pf_ptr->getPmPtr()->reverseLocalPath(tmp_res.start_point.index,
                                                 tmp_res.target_point.index);
          }
        }

      } else {
        // 如果不能重新规划出绕障路径，则停止
        LOG(INFO) << "Replan failed, switch to Pause.";
        pauseAMonent(pf_ptr, res.supervisor_name);
      }
    } break;
    case PASSPIT: {
      std::shared_ptr<PlannerDecision> ptr_planner_decision =
          PlannerDecision::getInstance();
      Pose2d pit_in_pose = res.start_point.position;
      Pose2d pit_out_pose = res.target_point.position;
      PoseMsg target;
      follower_state.states = PathFollowerStates::PASS_PIT;
      if (!ptr_planner_decision->getTargetPoint(pit_out_pose, pf_ptr,
                                                follower_state, target)) {
        break;
      }
      std::cout << "Local state PitPlan,start=(" << pit_in_pose.getX() << ","
                << pit_in_pose.getY() << "), end=(" << pit_out_pose.getX()
                << "," << pit_out_pose.getY() << "), target=("
                << target.position.getX() << "," << target.position.getY()
                << ")\r\n";
      if (pf_ptr->getPmPtr()->updatePitPath(pit_in_pose, pit_out_pose,
                                            target)) {
        LOG(INFO) << "Pass pit switch to Pit";
        pf_ptr->changeState(PitState::getPtrInstance());
      } else {
        LOG(INFO) << "Pass pit path fail";
      }
    } break;
    // 如果绕障过程中，突然有障碍物冲入停障框，则进入后退状态
    case OBSTACLESTOP: {
      LOG(INFO) << "ObstacleStop switch to RECOVER";
      pf_ptr->recoverClear();
      pf_ptr->changeState(RecoverState::getPtrInstance());
    } break;

    // 如果执行任务路径超过限制时间，进入暂停状态（注意这里面默认是对任务路径的执行超时，不是指绕障路径）
    case OVERTIME: {
      LOG(INFO) << "Path run overtime to Pause.";
      pauseAMonent(pf_ptr, res.supervisor_name);
    } break;

    // 如果此时绕障目标点被占用，则在任务路径上继续往后找一个更优的绕障目标点
    case UNREACHABLE: {
      // 需要处理点位占用的情况
      LOG(INFO) << "local plan target occupy.";
      std::shared_ptr<SubPath> ptr_refer_path =
          pf_ptr->getPmPtr()->getReferPath();
      if (!findBetterTarget(pf_ptr)) {
        LOG(INFO) << "Can't not reach switch to PAUSE";
        PauseState::getPtrInstance()->setPauseMessage(getState(), getState(),
                                                      res.supervisor_name);
        pf_ptr->changeState(PauseState::getPtrInstance());
      }
    } break;

    // 如果机器人角度偏差太大，则切换至rotate进行矫正(原地旋转)
    case ANGLETOPATH: {
      LOG(INFO) << "angle error too lager change to ROTATE";
      RotateState::getPtrInstance()->setTargetAngleWithType(
          res.target_point.position.getYaw(), "rotate_from_local");
      pf_ptr->changeState(RotateState::getPtrInstance());
    } break;

    default:
      break;
  }

  return follower_state;
}  // namespace CVTE_BABOT

bool LocalState::isCloseEnoughToGoal(PathFollower *pf_ptr) {
  double dist = pf_ptr->getControllerPathRemainLength();
  if (dist < 0.1) {
    return true;
  } else {
    return false;
  }
}

SupervisorResult LocalState::executeSupervise(PathFollower *pf_ptr,
                                              std::shared_ptr<SubPath> path) {
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPoseWithOffset();
  SupervisorState state(current_pose, path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
  return SupervisorChain::getPtrInstance()->superviseCycle(state);
}

bool LocalState::findBetterTarget(PathFollower *pf_ptr) {
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  std::shared_ptr<PlannerDecision> ptr_planner_decision =
      PlannerDecision::getInstance();
  PoseMsg target_pose;
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = "";
  if (ptr_planner_decision->getTargetPoint(current_pose, pf_ptr, follower_state,
                                           target_pose)) {
    double dis = target_pose.position.distanceTo(
        pf_ptr->getPmPtr()->getLocalTarget().position);
    if (dis < 0.4 || target_pose.index < pf_ptr->getPmPtr()->getTargetIndex()) {
      LOG(INFO) << "no better target find";
      return false;
    } else if (pf_ptr->getPmPtr()->updateLocalPath(target_pose)) {
      LOG(INFO) << "make plan to new target";
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
        LOG(WARNING) << "Local switch to Rotate yaw: " << target_yaw
                     << " diff_yaw " << diff_yaw;
        RotateState::getPtrInstance()->setTargetAngleWithType(
            target_yaw, "rotate_from_local");
        pf_ptr->changeState(RotateState::getPtrInstance());
      }
      return true;
    } else {
      LOG(ERROR) << "failed to make plan to new target";
      return false;
    }
  }
}

double LocalState::judgeLastPoint(const Pose2d &temp, const Pose2d &target) {
  // 如果找到是任务路径的最后一点，则靠近低于0.1米才切换回REFERENCE，
  // 但并非最后一点时，则可以接近1.5米就切换回来，避免触发靠近目标点减速导致卡顿
  if (temp == target) {
    return 0.1;
  } else {
    return 0.3;
  }
}

bool LocalState::prepareLocalPath(PathFollower *pf_ptr, const PoseMsg &target) {
  LOG(INFO) << "prepare to target(" << target.position.getX() << ", "
            << target.position.getY() << ", " << target.position.getYaw()
            << ")";
  if (pf_ptr->getPmPtr()->updateLocalPath(target)) {
    return true;
  } else {
    LOG(ERROR) << "update local path while prepare failed.";
    return false;
  }
}

void LocalState::pauseAMonent(PathFollower *pf_ptr,
                              const std::string &sup_name) {
  PauseState::getPtrInstance()->setPauseMessage(getState(), getState(),
                                                sup_name);
  pf_ptr->changeState(PauseState::getPtrInstance());
}

}  // namespace CVTE_BABOT
