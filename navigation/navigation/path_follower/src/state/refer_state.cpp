/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file refer_state.cpp
 *
 *@brief 跟踪任务路径下的状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020-07-08
 ************************************************************************/

#include "path_follower.hpp"
#include "path_manager.hpp"
#include "robot_info.hpp"
#include "state/refer_state.hpp"
#include "state/avoidcar_state.hpp"
#include "state/local_state.hpp"
#include "state/pause_state.hpp"
#include "state/rotate_state.hpp"
#include "state/recover_state.hpp"
#include "state/done_state.hpp"
#include "state/pit_state.hpp"
#include "supervisor/supervisor.hpp"
#include "supervisor/supervisor_chain.hpp"
#include "type.hpp"
#include "planner_decision/planner_decision.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<ReferState> ReferState::ptr_refer_state_ = nullptr;

std::shared_ptr<ReferState> ReferState::getPtrInstance() {
  if (ptr_refer_state_.get() == nullptr) {
    ptr_refer_state_.reset(new ReferState());
  }
  return ptr_refer_state_;
}

void ReferState::setPauseTime(const int &avoid_sec, const int &recover_sec,
                              const int &unreach_wait_sec) {
  recover_wait_second_ = recover_sec;
  avoid_wait_second_ = avoid_sec;
  unreach_wait_second_ = unreach_wait_sec;
  // TODO: 修改recover暂停时间
  LOG(INFO) << "Refer get avoid sec: " << avoid_wait_second_
            << "  recover sec: " << recover_wait_second_
            << " unreach wait sec: " << unreach_wait_second_;
}

FollowerStateRes ReferState::dealState(PathFollower *pf_ptr) {
  LOG(INFO) << " -------------------- Refer State -------------------- ";
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPoseWithOffset();
  std::shared_ptr<SubPath> r_path = pf_ptr->getPmPtr()->getReferPath();
  SupervisorState state(current_pose, r_path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);

  SupervisorResult res;
  // C3 屏蔽部分监督器
  if (RobotInfo::getPtrInstance()->getRobotType() == ROBOTTYPE::KAVA_CLEAN_C3 &&
      RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::EDGE) {
    // res = SupervisorChain::getPtrInstance()->superviseCycle(state);
    res = SupervisorChain::getPtrInstance()->superviseAssignOne(
        state, "DistanceToPath FinalGoal OverTime SpeedLevel");
  } else {
    res = SupervisorChain::getPtrInstance()->superviseCycle(state);
  }

  FollowerStateRes follower_state;
  follower_state.states = getState();
  // 如果在任务路径上，监督器返回一切正常，则直接返回
  if (res.can_continue) {
    follower_state.msg = "";
    return follower_state;
  }

  switch (res.status) {
    // 如果路径上有障碍物，当不需要暂停时间且找到绕障点且规划路径成功，则进入LOCAL状态绕障。
    // 失败或需要等待则进入暂停状态等待一会
    case OBSTACLEAVOIDER: {
      std::shared_ptr<PlannerDecision> ptr_planner_decision =
          PlannerDecision::getInstance();
      PoseMsg target_pose;
      //电梯任务规划路径前先进入暂停状态
      if (RobotInfo::getPtrInstance()->getMissionType() ==
          MISSIONTYPE::ELVATOR) {
        LOG(INFO) << "elevator Obstacle avoider switch to Pause";
        PauseState::getPtrInstance()->setPauseMessage(getState(), getState(),
                                                      res.supervisor_name, 0);
        pf_ptr->changeState(PauseState::getPtrInstance());
        return follower_state;
      }
      FollowerStateRes follower_state;
      follower_state.states = getState();
      follower_state.msg = "";
      if (ptr_planner_decision->getTargetPoint(current_pose, pf_ptr,
                                               follower_state, target_pose)) {
        PoseMsg start_pose;
        start_pose.position = current_pose;
        start_pose.index = pf_ptr->getPmPtr()->getReferIndex();
        if (prepareLocalPath(pf_ptr, start_pose, target_pose)) {
          LOG(INFO) << "Obstacle avoider switch to LOCAL";
          // 规划完路径后，需要判断轨迹是否处于窄道，并且判断是否需要倒退
          {
            std::shared_ptr<SubPath> l_path =
                pf_ptr->getPmPtr()->getLocalPath();
            SupervisorState tmp_state(current_pose, l_path, getState(),
                                      pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
            auto tmp_res =
                SupervisorChain::getPtrInstance()->superviseAssignOne(
                    tmp_state, "MarkerMap");
            if (!tmp_res.can_continue) {
              //当在窄道内，并且不能直接行走的时候，将窄道路径反向，使机器人退出窄道
              if (start_pose.index < 0 ||
                  start_pose.index >= l_path->wps.size() ||
                  target_pose.index < 0 ||
                  target_pose.index >= l_path->wps.size()) {
                LOG(WARNING) << "local path size " << int(l_path->wps.size())
                             << ", res start index " << start_pose.index
                             << " or target index " << target_pose.index
                             << " is invalid !";
              } else {
                const auto &sp = l_path->wps[start_pose.index];
                const auto &ep = l_path->wps[target_pose.index];
                LOG(INFO) << "# marker state # reverse refer path idx from"
                          << start_pose.index << " to " << target_pose.index
                          << ", position from[ " << sp.getX() << ","
                          << sp.getY() << "] to [" << ep.getX() << ","
                          << ep.getY() << "]";
                pf_ptr->getPmPtr()->reverseLocalPath(start_pose.index,
                                                     target_pose.index);
              }
            }
          }
          std::shared_ptr<SubPath> sub_path =
              pf_ptr->getPmPtr()->getLocalPath();
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
            LOG(WARNING) << "Refer switch to Rotate yaw: " << target_yaw
                         << " diff_yaw " << diff_yaw;
            RotateState::getPtrInstance()->setTargetAngleWithType(
                target_yaw, "rotate_from_local");
            pf_ptr->changeState(RotateState::getPtrInstance());
          } else {
            LOG(INFO) << "Refer switch to LOCAL";
            pf_ptr->changeState(LocalState::getPtrInstance());
          }
        } else {
          LOG(INFO) << "Obstacle avoider switch to Pause";
          PauseState::getPtrInstance()->setPauseMessage(
              getState(), getState(), res.supervisor_name, avoid_wait_second_);
          pf_ptr->changeState(PauseState::getPtrInstance());
        }
      } else {
        LOG(INFO) << "Obstacle avoider switch to Pause";
        PauseState::getPtrInstance()->setPauseMessage(
            getState(), getState(), res.supervisor_name, avoid_wait_second_);
        pf_ptr->changeState(PauseState::getPtrInstance());
      }
    } break;
    case PASSPIT: {
      std::shared_ptr<PlannerDecision> ptr_planner_decision =
          PlannerDecision::getInstance();
      PoseMsg target;
      Pose2d pit_in_pose = res.start_point.position;
      Pose2d pit_out_pose = res.target_point.position;
      follower_state.states = PathFollowerStates::PASS_PIT;
      if (!ptr_planner_decision->getTargetPoint(pit_out_pose, pf_ptr,
                                                follower_state, target)) {
        break;
      }
      // std::cout << "Refer state PitPlan,start=(" << pit_in_pose.getX() << ","
      //           << pit_in_pose.getY() << "), end=(" << pit_out_pose.getX()
      //           << "," << pit_out_pose.getY() << "), target=("
      //           << target.position.getX() << "," << target.position.getY()
      //           << " yaw=" << target.position.getYaw() << ")\r\n";
      if (pf_ptr->getPmPtr()->updatePitPath(pit_in_pose, pit_out_pose,
                                            target)) {
        LOG(INFO) << "Pass pit switch to Pit";
        pf_ptr->changeState(PitState::getPtrInstance());
      } else {
        LOG(INFO) << "Pass pit path fail";
        std::cout << "pass pit path fail\r\n";
        PoseMsg target_pose;
        follower_state.states = getState();
        if (ptr_planner_decision->getTargetPoint(current_pose, pf_ptr,
                                                 follower_state, target_pose)) {
          PoseMsg start_pose;
          start_pose.position = current_pose;
          start_pose.index = pf_ptr->getPmPtr()->getReferIndex();
          if (prepareLocalPath(pf_ptr, start_pose, target_pose)) {
            std::cout << "switch Local\r\n";
            pf_ptr->changeState(LocalState::getPtrInstance());
          }
        }
      }
    } break;
    // 如果此时已经到达最后一点，则进入旋转状态来修正与终点朝向
    case FINALGOAL: {
      LOG(INFO) << "Final Goal switch to Done";
      if (need_final_rotate_) {
        LOG(INFO) << "Final Goal switch to ROTATE";
        RotateState::getPtrInstance()->setTargetAngleWithType(
            res.target_point.position.getYaw(), "goal");
        pf_ptr->changeState(RotateState::getPtrInstance());
      } else {
        pf_ptr->changeState(DoneState::getPtrInstance());
      }
    } break;

    // 如果在任务路径上有障碍物进入停障框，优先进暂停一下，暂停状态下再决定是恢复还是绕障
    case OBSTACLESTOP: {
      LOG(INFO) << "OBSTACLESTOP switch to PAUSE";
      PauseState::getPtrInstance()->setPauseMessage(
          getState(), getState(), "ObstacleStop", recover_wait_second_);
      // 切换到停障碍处理 状态切换在每个状态自己内部
      pf_ptr->changeState(PauseState::getPtrInstance());
    } break;

    // 如果任务路径的最后一点被占用，则进入暂停状态返回点位占用异常（因为无法继续往后找绕障点）
    case UNREACHABLE: {
      LOG(INFO) << "Can't not reach switch to PAUSE";
      PauseState::getPtrInstance()->setPauseMessage(
          getState(), getState(), res.supervisor_name, unreach_wait_second_);
      pf_ptr->changeState(PauseState::getPtrInstance());
      // pauseAMonent(pf_ptr, res.supervisor_name);
    } break;

    // 如果执行任务路径超过限制时间，进入暂停状态
    case OVERTIME: {
      LOG(INFO) << "Path run overtime to PAUSE";
      pauseAMonent(pf_ptr, res.supervisor_name);
    } break;

    // 如果机器人偏离任务路径，则寻找最近点做规划路径，使其自己回到任务路径上
    // 导航过程中如果人遥控机器人偏离了会出现这种情况
    case DISTANCETOPATH: {
      if (res.new_local_goal && prepareLocalPath(pf_ptr, res.target_point)) {
        LOG(INFO) << "Path faraway switch to LOCAL";
        pf_ptr->changeState(LocalState::getPtrInstance());
      } else {
        LOG(INFO) << "Path faraway replan failed switch to Pause";
        pauseAMonent(pf_ptr, res.supervisor_name);
      }
    } break;

    // 如果机器人角度偏差太大，则切换至rotate进行矫正(原地旋转)
    case ANGLETOPATH: {
      LOG(INFO) << "angle error too lager change to ROTATE";
      RotateState::getPtrInstance()->setTargetAngleWithType(
          res.target_point.position.getYaw(), "rotate_from_refer");
      pf_ptr->changeState(RotateState::getPtrInstance());
    } break;

    default:
      break;
  }
  return follower_state;
}

bool ReferState::prepareLocalPath(PathFollower *pf_ptr, const PoseMsg &start,
                                  const PoseMsg &target) {
  LOG(INFO) << "prepare to target(" << target.position.getX() << ", "
            << target.position.getY() << ", " << target.position.getYaw()
            << ") with vel (" << target.velocity.v << ", " << target.velocity.w
            << ")";
  if (pf_ptr->getPmPtr()->updateLocalPath(start, target)) {
    return true;
  } else {
    LOG(ERROR) << "update local path while prepare failed.";
    return false;
  }
}

bool ReferState::prepareLocalPath(PathFollower *pf_ptr, const PoseMsg &target) {
  LOG(INFO) << "prepare to target(" << target.position.getX() << ", "
            << target.position.getY() << ", " << target.position.getYaw()
            << ") with vel (" << target.velocity.v << ", " << target.velocity.w
            << ")";
  if (pf_ptr->getPmPtr()->updateLocalPath(target)) {
    return true;
  } else {
    LOG(ERROR) << "update local path while prepare failed.";
    return false;
  }
}

void ReferState::pauseAMonent(PathFollower *pf_ptr,
                              const std::string &sup_name) {
  PauseState::getPtrInstance()->setPauseMessage(getState(), getState(),
                                                sup_name);
  pf_ptr->changeState(PauseState::getPtrInstance());
}

}  // namespace CVTE_BABOT
