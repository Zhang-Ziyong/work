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
 *@modified by lizhongjia(lizhongjia@cvte.com)
 *
 *@author lizhongjia(lizhongjia@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2022-04-06
 ************************************************************************/
#include <float.h>
#include <glog/logging.h>
#include <iostream>

#include "costmap_2d.hpp"
#include "costmap_utils.hpp"
#include "navigation_mediator.hpp"
#include "path_follower.hpp"
#include "pnc_map.hpp"
#include "supervisor/pass_pit_supervisor.hpp"

namespace CVTE_BABOT {
PassPitSupervisor::PassPitSupervisor(double pit_distance_front,
                                     bool enable_pit) {
  pit_distance_front_ = pit_distance_front;
  enable_pit_ = enable_pit;
}

void PassPitSupervisor::supervise(const SupervisorState &state,
                                  SupervisorResult &out) {
  if (!enable_pit_) {
    return;
  }

  std::shared_ptr<SubPath> sub_path = state.sub_path_;
  if (sub_path == nullptr || sub_path->wps.empty()) {
    out.can_continue = false;
    out.new_local_goal = false;
    LOG(INFO) << "empty path";
    return;
  }

  auto ptr_pnc_map = std::make_shared<PncMap>();

  size_t begin_index = 0;
  if (sub_path->type == PathType::LOCAL) {
    begin_index = state.pf_ptr_->getPmPtr()->getLocalIndex();
    ptr_pnc_map->SetPath(state.pf_ptr_->getPmPtr()->getLocalPath());
  } else {
    begin_index = state.pf_ptr_->getPmPtr()->getReferIndex();
    ptr_pnc_map->SetPath(state.pf_ptr_->getPmPtr()->getReferPath());
  }

  bool pit = false;
  size_t pit_index = begin_index;
  size_t pit_end_index = sub_path->wps.size() - 1;
  for (size_t index = begin_index; index < sub_path->wps.size(); index++) {
    if (sub_path->wpis[index].path_length -
            sub_path->wpis[begin_index].path_length >
        pit_distance_front_) {
      break;
    } else {
      if (ptr_pnc_map->InPitArea(math_utils::Vec2d(
              sub_path->wps[index].getX(), sub_path->wps[index].getY()))) {
        pit = true;
        pit_index = index;
        break;
      }
    }
  }
  if (pit) {
    for (size_t index = pit_index; index < sub_path->wps.size(); index++) {
      if (!ptr_pnc_map->InPitArea(math_utils::Vec2d(
              sub_path->wps[index].getX(), sub_path->wps[index].getY()))) {
        pit_end_index = index;
        break;
      }
    }
  }

  if (!pit) {
    out.can_continue = true;
    out.new_local_goal = false;
    out.target_point.position = sub_path->wps[pit_index];
    out.target_point.velocity = sub_path->wpis[pit_index];
    out.target_point.index = pit_index;
    LOG(INFO) << "pass pit is nothing continue";
  } else {
    out.can_continue = false;
    out.new_local_goal = false;
    out.start_point.position = sub_path->wps[pit_index];
    out.start_point.velocity = sub_path->wpis[pit_index];
    out.start_point.index = pit_index;
    out.target_point.position = sub_path->wps[pit_end_index];
    out.target_point.velocity = sub_path->wpis[pit_end_index];
    out.target_point.index = pit_end_index;
    out.status = PASSPIT;
    out.supervisor_name = "PassPit";
    LOG(INFO) << "pit supervise index: " << pit_index;
  }
}

}  // namespace CVTE_BABOT
