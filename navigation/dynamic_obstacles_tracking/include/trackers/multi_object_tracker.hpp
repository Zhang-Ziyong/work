/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file multi_object_tracker.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#ifndef __MULTI_OBJECT_TRACKER_HPP
#define __MULTI_OBJECT_TRACKER_HPP
#include "trackers/object_tracker.hpp"

namespace CVTE_BABOT {
class MultiObjectTracker {
 public:
  MultiObjectTracker();
  ~MultiObjectTracker() = default;

  MultiObjectTracker(const MultiObjectTracker &) = delete;
  MultiObjectTracker &operator=(const MultiObjectTracker &) = delete;

  void execPredict(std::vector<Eigen::VectorXd> &tracker_predicts);

  void updateAssignedTrackers(const std::vector<ObjectPtr> &objects_obsved,
                              const std::vector<TrackerObsvPair> &assignments);

  void updateUnassignedTrackers(const std::vector<int> &unassigned_trackers);

  void createNewTrackers(const std::vector<ObjectPtr> &objects_obsved,
                         const std::vector<int> &unassigned_ids);

  void removeLostTrackers();

  void removeInvalidTrackers();

  std::vector<ConstObjectTrackerPtr> getConstTrackers() const {
    std::vector<ConstObjectTrackerPtr> v_const_trackers;
    v_const_trackers.reserve(v_trackers_.size());

    for (const auto &ptr_tracker : v_trackers_) {
      v_const_trackers.push_back(ptr_tracker);
    }

    return v_const_trackers;
  }

  std::vector<ObjectTrackerPtr> getTrackers() { return v_trackers_; }

  void collectExpectedObjects(std::vector<ConstObjectPtr> &objects_expected);

  void collectTrackingObjsTraj(ObjectsTrajactory &obs_trajactory);

 private:
  // @brief get next avaiable track id
  // 每个追踪器分配一个唯一的id
  // @return next avaiable track id
  bool getNextTrackerId(IdType &new_id) {
    int count = 0;
    while (vb_id_[s_tracker_idx_]) {
      s_tracker_idx_++;

      if (s_tracker_idx_ == MAX_LABELS_NUM) {
        s_tracker_idx_ = 1;
      }

      count++;

      // 所有标签都被分配了
      if (count >= MAX_LABELS_NUM) {
        return false;
      }
    }

    vb_id_[s_tracker_idx_] = true;

    new_id = s_tracker_idx_;
    return true;
  }

  IdType s_tracker_idx_ = 1;

  std::vector<bool> vb_id_;

  std::vector<ObjectTrackerPtr> v_trackers_;
};

}  // namespace CVTE_BABOT
#endif
