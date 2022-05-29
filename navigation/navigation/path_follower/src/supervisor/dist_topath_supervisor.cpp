/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file dist_topath_supervisor.cpp
 *
 *@brief
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-30
 ************************************************************************/

#include "path_follower.hpp"
#include "supervisor/dist_topath_supervisor.hpp"

#include <float.h>
#include <glog/logging.h>

namespace CVTE_BABOT {

DistanceToPathSupervisor::DistanceToPathSupervisor(double max_distance_to_path)
    : max_dist_(max_distance_to_path) {}

void DistanceToPathSupervisor::supervise(const SupervisorState &state,
                                         SupervisorResult &out) {
  double dist = calculateDistanceToCurrentPathSegment(state, out);
  LOG(INFO) << "DistanceToPathSupervisor distance: " << dist;
  if (dist > max_dist_) {
    out.can_continue = false;
    out.new_local_goal = true;
    out.status = DISTANCETOPATH;
    out.supervisor_name = "PathFaraway";
  }
}

double DistanceToPathSupervisor::calculateDistanceToCurrentPathSegment(
    const SupervisorState &state, SupervisorResult &out) {
  std::shared_ptr<SubPath> sub_path = state.sub_path_;

  if (sub_path == nullptr || sub_path->wps.empty()) {
    return 0.0;
  }
  int index = 0;
  double dist = 0.0;
  double sum_dist = 0.0;
  double min_dist = DBL_MAX;

  // 如果当前检查的是任务路径，则只检查当前索引到往后5米的范围，如果是局部路径则检查整段的最小值
  size_t current_refer_index = state.pf_ptr_->getPmPtr()->getReferIndex();
  size_t index_start =
      (sub_path->type == PathType::REFER) ? current_refer_index : 0;
  size_t index_end =
      (sub_path->type == PathType::REFER)
          ? state.pf_ptr_->getPmPtr()->calcReferIndexWithDistance(5.0)
          : sub_path->wps.size();

  for (size_t i = index_start; i < index_end; ++i) {
    dist = state.robot_pose_.distanceTo(sub_path->wps[i]);
    if (dist < min_dist) {
      min_dist = dist;
      index = i;
    }
  }

  if (min_dist <= max_dist_) {
    return min_dist;
  }

  // 选择比最近点远1m的点作为局部路径的目标点
  size_t j;
  for (j = index; j < sub_path->wps.size() - 2; ++j) {
    sum_dist += sub_path->wps[j].distanceTo(sub_path->wps[j + 1]);
    if (sum_dist > 1.0) {
      break;
    }
  }

  if (j < sub_path->wps.size() - 1) {
    out.target_point.position = sub_path->wps[j];
    out.target_point.velocity = sub_path->wpis[j];
    out.target_point.index = j;
  } else {
    out.target_point.position = sub_path->wps[index];
    out.target_point.velocity = sub_path->wpis[index];
    out.target_point.index = index;
  }

  return min_dist;
}

}  // namespace CVTE_BABOT
