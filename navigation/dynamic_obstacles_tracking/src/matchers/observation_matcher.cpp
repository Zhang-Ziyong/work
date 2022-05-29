/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file observation_matcher.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#include <glog/logging.h>

#include "matchers/observation_matcher.hpp"
namespace CVTE_BABOT {
double ObservationMatcher::d_match_distance_maximum_ = 0.7;

// 如果有过分割分割好的,可以不用再匹配
void ObservationMatcher::match(
    const std::vector<ObjectPtr> &objects_obsved,
    const std::vector<ConstObjectTrackerPtr> &v_ptr_tracker,
    const std::vector<Eigen::VectorXd> &trackers_predict,
    std::vector<TrackerObsvPair> &assignments,
    std::vector<int> &unassigned_objects_obsved,
    std::vector<int> &unassigned_trackers) {
  std::vector<bool> if_objects_obsved_assigned(objects_obsved.size(), false);
  std::vector<bool> if_tracker_assigned(trackers_predict.size(), false);
  // LOG(ERROR) << "2. objects_obsved.size(): " << objects_obsved.size()
  //            << ", trackers_predict.size(): " << trackers_predict.size();

  for (size_t obsved_it = 0; obsved_it < objects_obsved.size(); obsved_it++) {
    // 如果之前已经匹配上了, 则不用继续匹配
    if (objects_obsved[obsved_it]->ifAssignedId()) {
      for (size_t tracker_it = 0; tracker_it < v_ptr_tracker.size();
           tracker_it++) {
        if (v_ptr_tracker[tracker_it]->getId() ==
            objects_obsved[obsved_it]->id) {
          assignments.push_back(TrackerObsvPair(tracker_it, obsved_it));
          if_objects_obsved_assigned[obsved_it] = true;
          if_tracker_assigned[tracker_it] = true;
          break;
        }
      }
    }
  }

  for (size_t obsved_it = 0; obsved_it < objects_obsved.size(); obsved_it++) {
    if (if_objects_obsved_assigned[obsved_it]) {
      continue;
    }

    double object_radius = std::min(objects_obsved[obsved_it]->width,
                                    objects_obsved[obsved_it]->length) /
                           2.0;
    double min_location_distance = std::numeric_limits<double>::max();
    int min_location_distance_it = -1;

    for (size_t predict_it = 0; predict_it < trackers_predict.size();
         predict_it++) {
      // 已经分配了则跳过
      if (if_tracker_assigned[predict_it]) {
        continue;
      }

      double location_distance = computeLocationDistance(
          objects_obsved[obsved_it], trackers_predict[predict_it]);

      // 只要两个物体相交了, 即认为是同一物体.
      // 以当前和预测的点云簇的中心匹配.
      // 计算两个物体内切圆半径的和, 物体间的距离小这个值则认为两物体相交
      double tracked_obj_radius = trackers_predict[predict_it](2);
      double tolerance_distance =
          object_radius + tracked_obj_radius + d_match_distance_maximum_;

      // LOG(ERROR) << "3. tolerance_distance: " << tolerance_distance;
      // 匹配取最近的目标
      if (location_distance < tolerance_distance &&
          location_distance < min_location_distance) {
        min_location_distance = location_distance;
        min_location_distance_it = predict_it;
      }
    }

    // LOG(ERROR) << "4. min_location_distance: " << min_location_distance;
    if (min_location_distance_it != -1) {
      assignments.push_back(
          TrackerObsvPair(min_location_distance_it, obsved_it));
      if_tracker_assigned[min_location_distance_it] = true;
    } else {
      unassigned_objects_obsved.push_back(obsved_it);
    }
    if_objects_obsved_assigned[obsved_it] = true;
  }

  for (size_t tracker_it = 0; tracker_it < if_tracker_assigned.size();
       tracker_it++) {
    if (!if_tracker_assigned[tracker_it]) {
      unassigned_trackers.push_back(tracker_it);
    }
  }
}

double ObservationMatcher::computeLocationDistance(
    const ObjectPtr &object_obsved, const Eigen::VectorXd &tracker_predict) {
  Eigen::Vector2d obsved_point = object_obsved->object_center.head(2);
  Eigen::Vector2d predicted_point = tracker_predict.head(2);
  Eigen::Vector2d measurement_predict_diff = obsved_point - predicted_point;
  return measurement_predict_diff.norm();
}
}  // namespace CVTE_BABOT
