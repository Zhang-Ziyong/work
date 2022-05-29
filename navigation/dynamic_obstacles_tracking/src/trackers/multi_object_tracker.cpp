/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file multi_object_tracker.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#include <glog/logging.h>

#include "trackers/multi_object_tracker.hpp"

namespace CVTE_BABOT {
MultiObjectTracker::MultiObjectTracker() {
  vb_id_.resize(MAX_LABELS_NUM, false);
}

void MultiObjectTracker::execPredict(
    std::vector<Eigen::VectorXd> &tracker_predicts) {
  tracker_predicts.resize(v_trackers_.size());
  for (size_t i = 0u; i < v_trackers_.size(); i++) {
    // 每一个物体追踪器->predict
    v_trackers_[i]->execPredict(tracker_predicts[i]);
  }
}

void MultiObjectTracker::updateAssignedTrackers(
    const std::vector<ObjectPtr> &objects_obsved,
    const std::vector<TrackerObsvPair> &assignments) {
  for (size_t i = 0u; i < assignments.size(); i++) {
    int tracker_id = assignments[i].first;
    int obsv_id = assignments[i].second;
    v_trackers_[tracker_id]->updateWithObservation(objects_obsved[obsv_id]);
  }
}

void MultiObjectTracker::updateUnassignedTrackers(
    const std::vector<int> &unassigned_trackers) {
  for (size_t i = 0u; i < unassigned_trackers.size(); ++i) {
    int tracker_idx = unassigned_trackers[i];
    v_trackers_[tracker_idx]->updateWithoutObservation(0.1);
  }
}

void MultiObjectTracker::createNewTrackers(
    const std::vector<ObjectPtr> &objects_obsved,
    const std::vector<int> &unassigned_ids) {
  for (size_t i = 0u; i < unassigned_ids.size(); i++) {
    // 为每一个追踪障碍物绑定障碍物追踪器
    IdType new_id = 0;
    if (!getNextTrackerId(new_id)) {
      LOG(ERROR) << "Id was run out of, try to remove some useless trackers.";
      removeInvalidTrackers();
      if (!getNextTrackerId(new_id)) {
        LOG(ERROR) << "Id was run out of!!!!!!.";
        v_trackers_.clear();
        return;
      }
    }

    int obsv_id = unassigned_ids[i];
    ObjectTrackerPtr ptr_tracker(
        new ObjectTracker(objects_obsved[obsv_id], new_id));
    v_trackers_.push_back(ptr_tracker);
  }
}

void MultiObjectTracker::removeInvalidTrackers() {
  for (auto it = v_trackers_.begin(); it != v_trackers_.end();) {
    if (!(*it)->isValid()) {
      it = v_trackers_.erase(it);
    } else {
      it++;
    }
  }
}

void MultiObjectTracker::removeLostTrackers() {
  // 1.统计没有跟丢的跟踪器
  std::vector<int> v_alive_trackers;
  for (size_t tracker_it = 0; tracker_it < v_trackers_.size(); tracker_it++) {
    if (!v_trackers_[tracker_it]->isLost()) {
      v_alive_trackers.push_back(tracker_it);
    } else {
      vb_id_[v_trackers_[tracker_it]->getId()] = false;
    }
  }

  // 如果所有都跟踪丢失, 全部清除
  if (v_alive_trackers.empty()) {
    v_trackers_.clear();
    return;
  }

  // 2.清除丢失的跟踪器, 因为使用了vector存储, 直接erase索引会错乱,
  // 所以保存没跟踪丢失的跟踪器, 来达到去除跟踪丢失的跟踪器的目的
  std::vector<ObjectTrackerPtr> v_trackers_last;
  v_trackers_last.swap(v_trackers_);

  v_trackers_.reserve(v_alive_trackers.size());

  for (size_t i = 0; i < v_alive_trackers.size(); i++) {
    int tracker_index = v_alive_trackers[i];
    v_trackers_.push_back(v_trackers_last[tracker_index]);
  }
}

void MultiObjectTracker::collectExpectedObjects(
    std::vector<ConstObjectPtr> &objects_expected) {
  for (size_t i = 0; i < v_trackers_.size(); i++) {
    if (v_trackers_[i]->isValid()) {
      objects_expected.push_back(v_trackers_[i]->getConstObject());
    }
  }
}

void MultiObjectTracker::collectTrackingObjsTraj(
    ObjectsTrajactory &obs_trajactory) {
  assert(obs_trajactory.empty());

  for (size_t i = 0; i < v_trackers_.size(); i++) {
    if (v_trackers_[i]->isTracked()) {
      obs_trajactory.emplace_back(v_trackers_[i]->getId(),
                                  v_trackers_[i]->getTrajectory());
    }
  }
}

}  // namespace CVTE_BABOT
