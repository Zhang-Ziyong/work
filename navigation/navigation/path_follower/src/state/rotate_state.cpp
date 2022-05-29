/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file rotate_state.cpp
 *
 *@brief 空闲的状态处理
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 08
 ************************************************************************/

#include "costmap_mediator.hpp"
#include "path_follower.hpp"
#include "state/rotate_state.hpp"
#include "state/refer_state.hpp"
#include "state/local_state.hpp"
#include "state/done_state.hpp"
#include "state/recover_state.hpp"
#include "robot_info.hpp"
#include "pose2d/pose2d.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {

std::shared_ptr<RotateState> RotateState::ptr_rotate_state_ = nullptr;

RotateState::RotateState() {
  follow_state_ = PathFollowerStates::ROTATE;
  calcRobotTypeWithFootprint();
}

std::shared_ptr<RotateState> RotateState::getPtrInstance() {
  if (ptr_rotate_state_.get() == nullptr) {
    ptr_rotate_state_.reset(new RotateState());
  }
  return ptr_rotate_state_;
}

FollowerStateRes RotateState::dealState(PathFollower *pf_ptr) {
  LOG(INFO) << " @@@@@@@@@@@@@@@@@@@@@ RotateState @@@@@@@@@@@@@@@@@@@@@ ";
  FollowerStateRes follower_state;
  follower_state.states = getState();
  follower_state.msg = "";
  bool state_changed = false;
  double output_angle_diff = 0.0;
  if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR &&
      rotate_type_ == "goal") {
    Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
    SupervisorState state(current_pose, nullptr, getState(),
                          pf_ptr->getPmPtr()->getCostmap(), pf_ptr);
    auto overtime_res = SupervisorChain::getPtrInstance()->superviseAssignOne(
        state, "OverTime");
    if (!overtime_res.can_continue) {
      LOG(INFO) << "elevator rotate over time, change to done";
      pf_ptr->changeState(DoneState::getPtrInstance());
      return follower_state;
    }
  }
  if (checkIfNeedToRotate(output_angle_diff)) {
    double w_val = 0.0;
    if (rotate_type_ == "start" || rotate_type_ == "goal") {
      w_val = judgeRotateDirection(output_angle_diff,
                                   pf_ptr->getPmPtr()->getCostmap());
      LOG(INFO) << "w_val: " << w_val;
      if (rotate_type_ == "start" && fabs(w_val) < 0.001) {
        // 起点无法旋转，进入Refer开始巡逻
        LOG(INFO) << "Rotate change to Refer";
        pf_ptr->changeState(ReferState::getPtrInstance());
        state_changed = true;
      }
    } else {
      if (judgeCrash()) {
        LOG(INFO) << "Rotate change to Recover ";
        pf_ptr->recoverClear();
        pf_ptr->changeState(RecoverState::getPtrInstance());
        state_changed = true;
      } else {
        w_val = getRotateWval(output_angle_diff);
      }
    }
    pf_ptr->setRotateVelocity(w_val);
  } else {
    // 如果已经在允许角度差范围内，则根据情况切换状态
    if (rotate_type_ == "start") {
      // 起点的旋转完成后，进入Refer开始巡逻
      LOG(INFO) << "Rotate Finish to Refer";
      pf_ptr->changeState(ReferState::getPtrInstance());
    } else if (rotate_type_ == "goal") {
      // 终点的旋转完成后，进入Done的完成状态
      LOG(INFO) << "Rotate Finish to Done";
      pf_ptr->changeState(DoneState::getPtrInstance());
      state_changed = true;
    } else if (rotate_type_ == "rotate_from_refer") {
      // 切换回原来的状态(refer)
      LOG(INFO) << "Rotate Finish to refer";
      pf_ptr->changeState(ReferState::getPtrInstance());
      state_changed = true;
    } else if (rotate_type_ == "rotate_from_local") {
      // 切换回原来的状态(local)
      LOG(INFO) << "Rotate Finish to local";
      pf_ptr->changeState(LocalState::getPtrInstance());
      state_changed = true;

    } else {
      LOG(ERROR) << "Undefine Rotate type: " << rotate_type_;
    }
  }
  SpeedDecisionResult speed_result =
      SpeedDecisionBase::getInstance()->getSpeedResult();

  if (speed_result.speed_radio == SpeedRadio::STOP && rotate_type_ == "start") {
    LOG(INFO) << "Rotate change to Refer";
    pf_ptr->changeState(ReferState::getPtrInstance());
  }

  return follower_state;
}

bool RotateState::checkIfNeedToRotate(double &angle_diff) {
  double robot_current_angle =
      RobotInfo::getPtrInstance()->getCurrentPose().getYaw();
  angle_diff =
      AngleCalculate::angle_diff(target_rotate_angle_, robot_current_angle);
  LOG(INFO) << "Angle diff: " << angle_diff << "(ca: " << robot_current_angle
            << ", ta: " << target_rotate_angle_ << ")";
  if (fabs(angle_diff) > angle_tolerance_) {
    return true;
  } else {
    return false;
  }
}

double RotateState::judgeRotateDirection(const double &angle_diff,
                                         std::shared_ptr<Costmap2d> costmap) {
  Eigen::Vector3d robot_pose(
      RobotInfo::getPtrInstance()->getCurrentPose().getX(),
      RobotInfo::getPtrInstance()->getCurrentPose().getY(),
      RobotInfo::getPtrInstance()->getCurrentPose().getYaw());
  double target_angle = robot_pose(2) + angle_diff;
  double current_angle = robot_pose(2);
  bool positive_crash = false;
  bool negative_crash = false;
  judgeCrash(current_angle, target_angle, positive_crash, negative_crash,
             robot_pose, costmap);
  double recalcu_angle_diff = angle_diff;
  if (positive_crash) {
    LOG(ERROR) << "positive rotate will crash";
  }
  if (negative_crash) {
    LOG(ERROR) << "negative rotate will crash";
  }
  if (positive_crash && negative_crash) {
    return 0.0;
  } else if (angle_diff > 0.0 && positive_crash) {
    recalcu_angle_diff = -angle_diff;
  } else if (angle_diff < 0.0 && negative_crash) {
    recalcu_angle_diff = -angle_diff;
  }

  if (recalcu_angle_diff > 0.0) {
    LOG(INFO) << "Left Turn.";
    double rotate_vel = std::min(1.0 * recalcu_angle_diff, rotate_velocity_);
    return rotate_vel;
  } else if (recalcu_angle_diff < 0.0) {
    LOG(INFO) << "Right Turn";
    double rotate_vel = std::max(1.0 * recalcu_angle_diff, -rotate_velocity_);
    return rotate_vel;
  } else {
    LOG(INFO) << "No Turn";
    return 0.0;
  }
}

void RotateState::judgeCrash(double current_angle, double target_angle,
                             bool &positive_crash, bool &negative_crash,
                             Eigen::Vector3d &robot_pose,
                             std::shared_ptr<Costmap2d> cost_map) const {
  positive_crash = negative_crash = false;
  double temp_current_angle = current_angle;
  // LOG(INFO) << "judge positive crash";
  while (!positive_crash && fabs(AngleCalculate::normalizeAngle(
                                target_angle - temp_current_angle)) > 0.1) {
    temp_current_angle =
        AngleCalculate::normalizeAngle(temp_current_angle + 0.1);
    if (judgeCrash(temp_current_angle, robot_pose, cost_map)) {
      positive_crash = true;
    }
  }
  temp_current_angle = current_angle;
  // LOG(INFO) << "judge negative crash";
  while (!negative_crash && fabs(AngleCalculate::normalizeAngle(
                                target_angle - temp_current_angle)) > 0.1) {
    temp_current_angle =
        AngleCalculate::normalizeAngle(temp_current_angle - 0.1);
    if (judgeCrash(temp_current_angle, robot_pose, cost_map)) {
      negative_crash = true;
    }
  }
}

bool RotateState::judgeCrash(double current_angle,
                             const Eigen::Vector3d &robot_pose,
                             std::shared_ptr<Costmap2d> cost_map) const {
  Eigen::Vector2d head_pt;
  Eigen::Vector2d half_head_pt;
  Eigen::Vector2d tail_pt;
  Eigen::Vector2d half_tail_pt;
  head_pt(0) = head_length_ * cos(current_angle) + robot_pose(0);
  head_pt(1) = head_length_ * sin(current_angle) + robot_pose(1);
  tail_pt(0) = tail_length_ * cos(current_angle + M_PI) + robot_pose(0);
  tail_pt(1) = tail_length_ * sin(current_angle + M_PI) + robot_pose(1);
  half_head_pt = 0.5 * (robot_pose.block<2, 1>(0, 0) + head_pt);
  half_tail_pt = 0.5 * (robot_pose.block<2, 1>(0, 0) + tail_pt);
  double head_pt_value = cost_map->getCostWithMapPose(head_pt(0), head_pt(1));
  double half_head_pt_value =
      cost_map->getCostWithMapPose(half_tail_pt(0), half_tail_pt(1));
  double tail_pt_value = cost_map->getCostWithMapPose(tail_pt(0), tail_pt(1));
  double half_tail_pt_value =
      cost_map->getCostWithMapPose(half_tail_pt(0), half_tail_pt(1));
  // LOG(INFO) << "head_pt: " << head_pt << " head_pt_value: " << head_pt_value;
  // LOG(INFO) << "half_head_pt: " << half_head_pt
  //           << " half_head_pt_value: " << half_head_pt_value;
  // LOG(INFO) << "tail_pt: " << tail_pt << " tail_pt_value: " << tail_pt_value;
  // LOG(INFO) << "half_tail_pt: " << half_tail_pt
  //           << " half_tail_pt_value: " << half_tail_pt_value;
  if (head_pt_value > dangerous_value_ ||
      half_head_pt_value > dangerous_value_ ||
      tail_pt_value > dangerous_value_ ||
      half_tail_pt_value > dangerous_value_) {
    return true;
  } else {
    return false;
  }
}

bool RotateState::judgeCrash() {
  return !(SpeedDecisionBase::getInstance()->checkOutofRecovery());
}

double RotateState::getRotateWval(const double &angle_diff) {
  Eigen::Vector3d robot_pose(
      RobotInfo::getPtrInstance()->getCurrentPose().getX(),
      RobotInfo::getPtrInstance()->getCurrentPose().getY(),
      RobotInfo::getPtrInstance()->getCurrentPose().getYaw());
  double target_angle = robot_pose(2) + angle_diff;
  double current_angle = robot_pose(2);
  bool positive_crash = false;
  bool negative_crash = false;

  double recalcu_angle_diff = angle_diff;

  if (recalcu_angle_diff > 0.0) {
    LOG(INFO) << "Left Turn.";
    double rotate_vel = std::min(1.5 * recalcu_angle_diff, rotate_velocity_);
    return rotate_vel;
  } else if (recalcu_angle_diff < 0.0) {
    LOG(INFO) << "Right Turn";
    double rotate_vel = std::max(1.5 * recalcu_angle_diff, -rotate_velocity_);
    return rotate_vel;
  } else {
    LOG(INFO) << "No Turn";
    return 0.0;
  }
}

void RotateState::calcRobotTypeWithFootprint() {
  // std::vector<WorldmapPoint> vwp_footprint;
  // CostmapMediator::getPtrInstance()->getParam("footprint", vwp_footprint,
  //                                             vwp_footprint);
  // head_length_ = 0.0;
  // tail_length_ = 0.0;
  // for (const auto &coor : vwp_footprint) {
  //   // 从四个foorprint坐标点的x轴找到对应车头和车尾长度
  //   if (coor.d_x >= head_length_) {
  //     head_length_ = coor.d_x;
  //   }

  //   if (coor.d_x < tail_length_) {
  //     tail_length_ = coor.d_x;
  //   }
  // }
  // 车头长度小于车尾长度，则认为是前驱底盘
  is_front_dynamic_ = (fabs(head_length_) <= fabs(tail_length_)) ? true : false;
  head_length_ = fabs(head_length_);
  tail_length_ = fabs(tail_length_);
  LOG(INFO) << "head length: " << head_length_
            << "   tail length: " << tail_length_
            << "  is_front_dynamic_:" << is_front_dynamic_;
}

}  // namespace CVTE_BABOT
