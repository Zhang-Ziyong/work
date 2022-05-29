/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file observation_matcher.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#ifndef __OBSERVATION_MATCHER_HPP
#define __OBSERVATION_MATCHER_HPP
#include "common/tracking_utils.hpp"
#include "trackers/object_tracker.hpp"

namespace CVTE_BABOT {
class ObservationMatcher {
 private:
  // threshold of matching
  static double d_match_distance_maximum_;

 public:
  ObservationMatcher() = default;
  ~ObservationMatcher() = default;

  ObservationMatcher(const ObservationMatcher &) = delete;
  ObservationMatcher &operator=(const ObservationMatcher &) = delete;

  /**
   *setMatchDistanceMaximum
   *@brief
   *简介
   *
   *@param[in] match_distance_maximum-两个类关联的最大距离
  **/
  static void setMatchDistanceMaximum(const double &match_distance_maximum) {
    d_match_distance_maximum_ = match_distance_maximum;
  }

  void match(const std::vector<ObjectPtr> &objects_obsved,
             const std::vector<ConstObjectTrackerPtr> &v_ptr_tracker,
             const std::vector<Eigen::VectorXd> &trackers_predict,
             std::vector<TrackerObsvPair> &assignments,
             std::vector<int> &unassigned_objects_obsved,
             std::vector<int> &unassigned_trackers);

 private:
  double computeLocationDistance(const ObjectPtr &object_obsved,
                                 const Eigen::VectorXd &tracker_predict);
};
}  // namespace CVTE_BABOT
#endif