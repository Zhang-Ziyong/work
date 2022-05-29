/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file obstacle_avoider_supervisor.cpp
 *
 *@brief
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2020-01-06
 ************************************************************************/
#include <float.h>
#include <glog/logging.h>
#include <iostream>

#include "costmap_2d.hpp"
#include "costmap_utils.hpp"
#include "path_follower.hpp"
#include "pnc_map.hpp"
#include "supervisor/obstacle_avoider_supervisor.hpp"

namespace CVTE_BABOT {
ObstacleAvoiderSupervisor::ObstacleAvoiderSupervisor(
    double obstacle_distance_front, double obstacle_distance_behind,
    double dangerous_value) {
  obstacle_distance_front_ = obstacle_distance_front;
  // obstacle_distance_behind_ = obstacle_distance_behind;
  dangerous_value_ = dangerous_value;
}

void ObstacleAvoiderSupervisor::supervise(const SupervisorState &state,
                                          SupervisorResult &out) {
  // 这部分后续要改成碰撞检测，要包括运动估计，要根据不同的运动模型来计算
  std::shared_ptr<SubPath> sub_path = state.sub_path_;
  if (sub_path == nullptr || sub_path->wps.empty()) {
    out.can_continue = false;
    out.new_local_goal = false;
    LOG(INFO) << "empty path";
    return;
  }
  size_t begin_index = 0;
  if (sub_path->type == PathType::LOCAL) {
    begin_index = state.pf_ptr_->getPmPtr()->getLocalIndex();
  } else {
    begin_index = state.pf_ptr_->getPmPtr()->getReferIndex();
  }
  bool crash = false;
  size_t crash_index = begin_index;
  double supervisor_length = obstacle_distance_front_;
  PncMap pnc_map;
  bool in_slope = pnc_map.InSlopeArea(math_utils::Vec2d(
      sub_path->wps[begin_index].x, sub_path->wps[begin_index].y));
  if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
    supervisor_length = 4.0;
  } else if (in_slope) {
    LOG(INFO) << "in slope, supervisor_length = 1.0";
    supervisor_length = 1.0;
  }
  for (size_t index = begin_index; index < sub_path->wps.size(); index++) {
    if (sub_path->wpis[index].path_length -
            sub_path->wpis[begin_index].path_length >
        supervisor_length) {
      break;
    } else {
      if (checkPointCostValue(sub_path->wps[index], state.ptr_costmap_2d_) >
          dangerous_value_) {
        crash = true;
        crash_index = index;
        break;
      }
    }
  }
  if (!crash) {
    out.can_continue = true;
    out.new_local_goal = false;
    out.target_point.position = sub_path->wps[crash_index];
    out.target_point.velocity = sub_path->wpis[crash_index];
    out.target_point.index = crash_index;
    LOG(INFO) << "obstacle_avoider is nothing continue";
  } else {
    out.can_continue = false;
    out.new_local_goal = false;
    out.target_point.position = sub_path->wps[crash_index];
    out.target_point.velocity = sub_path->wpis[crash_index];
    out.target_point.index = crash_index;
    LOG(INFO) << "obstacle supervise crash index: " << crash_index;
    // if(crash_index == sub_path->wps.size() - 1)
    std::shared_ptr<SubPath> ptr_refer_path =
        state.pf_ptr_->getPmPtr()->getReferPath();
    double point_occ_length = 2.0;
    double dangle_cost = dangerous_value_;
    if (RobotInfo::getPtrInstance()->getMissionType() == MISSIONTYPE::ELVATOR) {
      point_occ_length = 4.0;
      dangle_cost = 80;
    }
    if (ptr_refer_path->wpis.back().path_length -
            ptr_refer_path->wpis[state.pf_ptr_->getPmPtr()->getReferIndex()]
                .path_length <
        point_occ_length) {
      int cost = checkPointCostValue(ptr_refer_path->wps.back(),
                                     state.ptr_costmap_2d_);
      if (cost > dangle_cost && cost <= 254) {
        out.status = UNREACHABLE;
        out.supervisor_name = "PointOccupy";
      } else {
        out.status = OBSTACLEAVOIDER;
        out.supervisor_name = "ObstacleAvoider";
      }
    } else {
      out.status = OBSTACLEAVOIDER;
      out.supervisor_name = "ObstacleAvoider";
    }
  }
}
int ObstacleAvoiderSupervisor::checkPointCostValue(
    const Pose2d &check_point, std::shared_ptr<Costmap2d> ptr_costmap_2d) {
  return ptr_costmap_2d->getCostWithMapPose(check_point.getX(),
                                            check_point.getY());
}

}  // namespace CVTE_BABOT
