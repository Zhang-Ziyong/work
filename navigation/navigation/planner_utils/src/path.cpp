/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file path.hpp
 *
 *@brief 导航路径类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-16
 ************************************************************************/
#include "path.hpp"
#include <glog/logging.h>
#include <iostream>

namespace CVTE_BABOT {

void Path::clear() {
  path_.clear();
  current_sub_path_ = path_.begin();
  next_waypoint_idx_ = 0;
  wp_distance_to_end_.clear();
}

void Path::reset() {
  current_sub_path_ = path_.begin();
  next_waypoint_idx_ = 0;
  computeWayPointToEndDistances();
}

bool Path::empty() const {
  if (path_.empty()) {
    return true;
  }
  for (const auto &sp : path_) {
    if (!sp.empty()) {
      return false;
    }
  }
  return true;
}

bool Path::isDone() const {
  return current_sub_path_ == path_.end();
}

bool Path::isSubPathDone() const {
  return next_waypoint_idx_ >= current_sub_path_->size();
}

bool Path::isNewPath() const {
  return new_path_flag_;
}

void Path::clearNewPathFlag() {
  new_path_flag_ = false;
}

size_t Path::subPathCount() const {
  return path_.size();
}

void Path::setPath(const std::vector<SubPath> &path) {
  path_ = path;
  current_sub_path_ = path_.begin();
  next_waypoint_idx_ = 0;
  new_path_flag_ = true;
  computeWayPointToEndDistances();
}

void Path::registerNextWayPointCallback(NextWayPointCallback_t func) {
  next_wp_callback_ = func;
  has_callback_ = true;
}

void Path::switchToNextSubPath() {
  std::vector<SubPath>::iterator endIter = path_.end();
  if (current_sub_path_ != endIter) {
    ++current_sub_path_;
    next_waypoint_idx_ = 0;
    fireNextWayPointCallback();
    computeWayPointToEndDistances();
  }
}

void Path::switchToNextWayPoint() {
  if (!isSubPathDone()) {
    ++next_waypoint_idx_;
    fireNextWayPointCallback();
  }
}

void Path::setPathTargetPoint(const Pose2d &point) {
  target_point = point;
  LOG(INFO) << "Set target point: " << target_point.getX() << ", "
            << target_point.getY();
}

bool Path::isLastWayPoint() const {
  return next_waypoint_idx_ + 1 == current_sub_path_->size();
}

std::shared_ptr<SubPath> Path::getCurrentSubPath() const {
  return std::make_shared<SubPath>((*current_sub_path_));
}

std::shared_ptr<SubPath> Path::getSubPath(size_t idx) const {
  return std::make_shared<SubPath>(path_.at(idx));
}

Pose2d Path::getWayPoint(size_t idx) const {
  return (*current_sub_path_)[idx];
}

Pose2d Path::getCurrentWayPoint() const {
  return (*current_sub_path_).at(next_waypoint_idx_);
}

Pose2d Path::getLastWayPoint() const {
  return current_sub_path_->back();
}

Pose2d Path::getPathTargetPoint() const {
  LOG(INFO) << "get target point !@! " << target_point.getX() << ", "
            << target_point.getY();
  return target_point;
}

size_t Path::getWayPointIndex() const {
  return next_waypoint_idx_;
}

float Path::getRemainingSubPathDistance() const {
  return wp_distance_to_end_[next_waypoint_idx_];
}

void Path::fireNextWayPointCallback() {
  if (has_callback_) {
    next_wp_callback_();
  }
}

void Path::computeWayPointToEndDistances() {
  if (current_sub_path_ == path_.end()) {
    wp_distance_to_end_.clear();
    return;
  }

  // TODO: resize initializes every value. This is not necessary as they are
  // overwritten anyway.
  //      Is there a more efficient way to do this?
  wp_distance_to_end_.resize(current_sub_path_->size());

  // Distance form last waypoint to end is zero (last wp == end of path)
  wp_distance_to_end_.back() = 0;
  // iterate subpath in reversed order starting with the penultimate waypoint
  for (int i = current_sub_path_->size() - 2; i >= 0; --i) {
    float dist_to_next_waypoint =
        (*current_sub_path_)[i].distanceTo((*current_sub_path_)[i + 1]);
    wp_distance_to_end_.at(i) =
        dist_to_next_waypoint + wp_distance_to_end_.at(i + 1);
  }
}

double Path::getCurrentPathAngle() const {
  std::vector<Pose2d> sub_path;
  if (current_sub_path_ == path_.end()) {
    LOG(ERROR) << "iterator point to empty.";
    return 0.0;
  }

  sub_path.reserve((*current_sub_path_).wps.size());
  sub_path = (*current_sub_path_).wps;

  if (sub_path.empty()) {
    LOG(ERROR) << "get an empty path angle,return 0.0";
    return 0.0;
  }

  // 如果小于4个点，则以路径最后一点的角度为路径角度
  if (sub_path.size() < 4) {
    LOG(INFO)
        << "Plan only three point, use the point's angle as plan's angle: "
        << sub_path.back().getYaw();
    return sub_path.back().getYaw();
  } else {
    return sub_path.front().getYaw();
  }

  // // 截取路径前面半米的长度来计算路径朝向角度
  // int calc_size = 0;
  // double sum_dist = 0.0;
  // for (size_t i = 0; i < sub_path.size(); ++i) {
  //   sum_dist += sub_path[i].distanceTo(sub_path[i + 1]);
  //   if (sum_dist >= 0.5) {
  //     calc_size = i;
  //     break;
  //   }
  // }
  // // 如果路径总长小于半米，则直接计算整条
  // if (sum_dist < 0.5) {
  //   calc_size = static_cast<int>(sub_path.size());
  // }

  // double temp_angle = 0.0;
  // double sum_angle = 0.0;
  // double plan_angle = 0.0;
  // std::vector<double> angle_vec;
  // angle_vec.reserve(calc_size);
  // for (int i = 1; i < calc_size; ++i) {
  //   temp_angle = atan2((sub_path[i].getY() - sub_path[0].getY()),
  //                      (sub_path[i].getX() - sub_path[0].getX()));
  //   sum_angle += temp_angle;
  //   angle_vec.push_back(temp_angle);
  // }

  // sort(angle_vec.begin(), angle_vec.end());
  // plan_angle = angle_vec.at(angle_vec.size() / 2);

  // // 统一转化到[-pi, pi]的范围
  // if (plan_angle > M_PI) {
  //   plan_angle -= 2 * M_PI;
  // }
  // return plan_angle;
}

// void Path::reverseCurrentPath(int sidx, int eidx) {
//   // 提取轨迹位姿和信息
//   auto &poses = current_sub_path_->wps;
//   auto &infos = current_sub_path_->wpis;
//   if (poses.size() != infos.size() || sidx < 0 || sidx >= poses.size() ||
//       eidx < 0 || eidx >= poses.size()) {
//     LOG(WARNING) << "input idx: [" << sidx << ", " << eidx << "] or poses
//     size "
//                  << int(poses.size()) << " isnt valid !";
//     return;
//   }

//   // 取反给定区间内的位姿点
//   for (int i = sidx; i <= eidx; i++) {
//     // 取反姿态
//     poses[i].yaw += M_PI;
//     if (poses[i].yaw > M_PI)
//       poses[i].yaw -= 2 * M_PI;
//     // 取反速度
//     infos[i].v = -infos[i].v;
//   }
// }

}  // namespace CVTE_BABOT
