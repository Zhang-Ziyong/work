/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file pit_state.cpp
 *
 *@brief 过坎任务路径下的状态处理
 *
 *@modified by lizhongjia(lizhongjia@cvte.com)
 *
 *@author lizhongjia(lizhongjia@cvte.com)
 *@version current_algo.dev
 *@data 2022-04-06
 ************************************************************************/

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "path_follower.hpp"
#include "path_manager.hpp"
#include "robot_info.hpp"
#include "state/refer_state.hpp"
#include "state/pit_state.hpp"
#include "state/avoidcar_state.hpp"
#include "state/local_state.hpp"
#include "state/pause_state.hpp"
#include "state/rotate_state.hpp"
#include "state/recover_state.hpp"
#include "state/done_state.hpp"
#include "supervisor/supervisor.hpp"
#include "supervisor/supervisor_chain.hpp"
#include "type.hpp"
#include "planner_decision/planner_decision.hpp"
#include "pnc_map.hpp"
#include "pit_planner/pit_planner.hpp"

#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<PitState> PitState::ptr_refer_state_ = nullptr;

std::shared_ptr<PitState> PitState::getPtrInstance() {
  if (ptr_refer_state_.get() == nullptr) {
    ptr_refer_state_.reset(new PitState());
  }
  return ptr_refer_state_;
}

void PitState::setParams(double edge_dist) {
  pit_edge_dist_ = edge_dist;
}

void PitState::setPauseTime(const int &avoid_sec, const int &recover_sec,
                            const int &unreach_wait_sec) {
  recover_wait_second_ = recover_sec;
  avoid_wait_second_ = avoid_sec;
  unreach_wait_second_ = unreach_wait_sec;
  // TODO: 修改recover暂停时间
  LOG(INFO) << "Pit get avoid sec: " << avoid_wait_second_
            << "  recover sec: " << recover_wait_second_
            << " unreach wait sec: " << unreach_wait_second_;
}

FollowerStateRes PitState::dealState(PathFollower *pf_ptr) {
  LOG(INFO) << " -------------------- Pit State -------------------- ";
  // std::cout << "Pit State: Refer:" << pf_ptr->getPmPtr()->getReferIndex()
  //           << ",index_inc:"
  //           << pf_ptr->getPmPtr()->getReferIndex() - pf_ptr->index_last
  //           << ", Refer size:" << pf_ptr->getPmPtr()->getReferPath()->size()
  //           << ", local:" << pf_ptr->getPmPtr()->getLocalIndex()
  //           << ", local size:" << pf_ptr->getPmPtr()->getLocalPath()->size()
  //           << std::endl;
  if (isCloseEnoughToGoal(pf_ptr)) {
    LOG(INFO) << "finish pit path, checkto REFERENCE.";
    pf_ptr->getPmPtr()->updateReferIndex();
    pf_ptr->changeState(ReferState::getPtrInstance());
  }

  auto res = executeSupervise(pf_ptr, pf_ptr->getPmPtr()->getLocalPath());
  FollowerStateRes follower_state;
  follower_state.states = getState();

  switch (res.status) {
    case OBSTACLEAVOIDER: {
      LOG(INFO) << "obstacle avoider";
      if (isInPit(pf_ptr, pit_edge_dist_, pit_edge_dist_ * 0.6)) {
        LOG(INFO) << "obstacle avoider, but car in pit ,switch PauseState";
        res.supervisor_name = "PassPit";
        PauseState::getPtrInstance()->setPauseMessage(getState(), getState(),
                                                      res.supervisor_name, 0);
        pf_ptr->changeState(PauseState::getPtrInstance());

      } else {
        pf_ptr->changeState(LocalState::getPtrInstance());
      }
    } break;
    case PASSPIT: {
      LOG(INFO) << "pit ahead";
      if (isInPit(pf_ptr, pit_edge_dist_ * 0.6, pit_edge_dist_ * 0.8)) {
        break;
      }

      if (isSafe(pf_ptr, res.target_point.index) &&
          isVerticalPit(res.start_point.position, res.target_point.position)) {
        break;
      }

      Pose2d pit_in_pose = res.start_point.position;
      Pose2d pit_out_pose = res.target_point.position;
      PoseMsg target;
      std::shared_ptr<PlannerDecision> ptr_planner_decision =
          PlannerDecision::getInstance();
      follower_state.states = PathFollowerStates::PASS_PIT;
      if (!ptr_planner_decision->getTargetPoint(pit_out_pose, pf_ptr,
                                                follower_state, target)) {
        break;
      }
      std::cout << "Pit state PitPlan,start=(" << pit_in_pose.getX() << ","
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
    // 如果此时已经到达最后一点，则进入旋转状态来修正与终点朝向
    case FINALGOAL: {
      // LOG(INFO) << "Final Goal switch to Done";
      // if (need_final_rotate_) {
      //   LOG(INFO) << "Final Goal switch to ROTATE";
      //   RotateState::getPtrInstance()->setTargetAngleWithType(
      //       res.target_point.position.getYaw(), "goal");
      //   pf_ptr->changeState(RotateState::getPtrInstance());
      // } else {
      //   pf_ptr->changeState(DoneState::getPtrInstance());
      // }
    } break;

    // 如果在任务路径上有障碍物进入停障框，优先进暂停一下，暂停状态下再决定是恢复还是绕障
    case OBSTACLESTOP: {
      // LOG(INFO) << "OBSTACLESTOP switch to PAUSE";
      // PauseState::getPtrInstance()->setPauseMessage(
      //     getState(), getState(), "ObstacleStop", recover_wait_second_);
      // // 切换到停障碍处理 状态切换在每个状态自己内部
      // pf_ptr->changeState(PauseState::getPtrInstance());
    } break;

    // 如果任务路径的最后一点被占用，则进入暂停状态返回点位占用异常（因为无法继续往后找绕障点）
    case UNREACHABLE: {
      // LOG(INFO) << "Can't not reach switch to PAUSE";
      // PauseState::getPtrInstance()->setPauseMessage(
      //     getState(), getState(), res.supervisor_name,
      //     unreach_wait_second_);
      // pf_ptr->changeState(PauseState::getPtrInstance());
      // // pauseAMonent(pf_ptr, res.supervisor_name);
    } break;

    // 如果执行任务路径超过限制时间，进入暂停状态
    case OVERTIME: {
      // LOG(INFO) << "Path run overtime to PAUSE";
      // pauseAMonent(pf_ptr, res.supervisor_name);
    } break;

    // 如果机器人偏离任务路径，则寻找最近点做规划路径，使其自己回到任务路径上
    // 导航过程中如果人遥控机器人偏离了会出现这种情况
    case DISTANCETOPATH: {
      // if (res.new_local_goal && prepareLocalPath(pf_ptr, res.target_point))
      // {
      //   LOG(INFO) << "Path faraway switch to LOCAL";
      //   pf_ptr->changeState(LocalState::getPtrInstance());
      // } else {
      //   LOG(INFO) << "Path faraway replan failed switch to Pause";
      //   pauseAMonent(pf_ptr, res.supervisor_name);
      // }
    } break;

    default:
      break;
  }
  return follower_state;
}

void PitState::pauseAMonent(PathFollower *pf_ptr, const std::string &sup_name) {
  PauseState::getPtrInstance()->setPauseMessage(getState(), getState(),
                                                sup_name);
  pf_ptr->changeState(PauseState::getPtrInstance());
}

SupervisorResult PitState::executeSupervise(PathFollower *pf_ptr,
                                            std::shared_ptr<SubPath> path) {
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPoseWithOffset();
  SupervisorState state(current_pose, path, getState(),
                        pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
  return SupervisorChain::getPtrInstance()->superviseCycle(state);
}

bool PitState::isCloseEnoughToGoal(PathFollower *pf_ptr) {
  double dist = pf_ptr->getControllerPathRemainLength();
  if (dist < 0.15) {
    return true;
  } else {
    return false;
  }
}

bool PitState::isSafe(PathFollower *pf_ptr, size_t target_index) {
  auto path = pf_ptr->getPmPtr()->getLocalPath();
  auto current_index = pf_ptr->getPmPtr()->getLocalIndex();
  auto end_index = pf_ptr->getPmPtr()->getLocalPathSize();
  for (size_t i = target_index; i < pf_ptr->getPmPtr()->getLocalPathSize();
       i += 5) {
    if (path->wpis[i].path_length - path->wpis[target_index].path_length >
        pit_edge_dist_ * 1.8) {
      end_index = i;
      break;
    }
  }

  auto cost_map = pf_ptr->getPmPtr()->getCostmap();
  for (size_t i = current_index; i < end_index; i += 5) {
    if (cost_map->getCostWithMapPose(path->wps[i].getX(), path->wps[i].getY()) >
        252) {
      LOG(INFO) << "pit path have obstacle";
      return false;
    }
  }
  return true;
}

bool PitState::isVerticalPit(const CVTE_BABOT::Pose2d &start,
                             const CVTE_BABOT::Pose2d &target) {
  math_utils::Vec2d pit_in(start.getX(), start.getY());
  math_utils::Vec2d pit_out(target.getX(), target.getY());

  auto ptr_pnc_map = std::make_shared<PncMap>();
  cvte::hdmap::Box2d box;
  ptr_pnc_map->GetPitBox(pit_in, box);
  math_utils::Vec2d box_vec(-box.sin_heading(), box.cos_heading());
  math_utils::Vec2d pit_vec = pit_out - pit_in;
  double angle = acos(abs(pit_vec.dot(box_vec)) / pit_vec.norm());
  angle = angle / 3.14159265 * 180;
  return angle < 10 ? true : false;
}

bool PitState::isInPit(PathFollower *pf_ptr, double forward_dist,
                       double backward_dist) {
  size_t current_index = pf_ptr->getPmPtr()->getLocalIndex();
  std::shared_ptr<SubPath> l_path = pf_ptr->getPmPtr()->getLocalPath();
  auto ptr_pnc_map = std::make_shared<PncMap>();
  ptr_pnc_map->SetPath(l_path);
  return ptr_pnc_map->ForwardInPitArea(current_index, forward_dist) ||
         ptr_pnc_map->BackwardInPitArea(current_index, backward_dist);
}

}  // namespace CVTE_BABOT
