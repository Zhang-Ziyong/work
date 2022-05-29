/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file tracking_worker.cpp
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
#include "trackers/multi_object_tracker.hpp"
#include "tracking_worker.hpp"

namespace CVTE_BABOT {
TrackingWorker::TrackingWorker()
    : ptr_matcher_(std::make_shared<ObservationMatcher>()),
      ptr_multi_obj_tracker_(std::make_shared<MultiObjectTracker>()) {}

void TrackingWorker::track(const std::vector<ObjectPtr> &objects_obsved,
                           std::vector<ObjectPtr> &objects_tracked) {
  // 1.获取之前对象的预测位置及大小: x, y, radius
  std::vector<Eigen::VectorXd> tracker_predicts;
  ptr_multi_obj_tracker_->execPredict(tracker_predicts);

  // 2.将当前对象与之前的对象关联, 关联上的继续跟踪,
  // 当前对象关联不上的建立新的跟踪器
  // 重要面积大的类在前面,匹配时以面积大的类的标签作为新类的标签
  // std::vector<ObjectPtr> objects_sorted = objects_obsved;
  // sort(objects_sorted.begin(), objects_sorted.end(), areaCmp);

  std::vector<TrackerObsvPair> assignments;
  std::vector<int> unassigned_objects_obsved;
  std::vector<int> unassigned_trackers;

  auto v_trackers = ptr_multi_obj_tracker_->getConstTrackers();
  ptr_matcher_->match(objects_obsved, v_trackers, tracker_predicts, assignments,
                      unassigned_objects_obsved, unassigned_trackers);

  // 3.更新关联上的跟踪器
  ptr_multi_obj_tracker_->updateAssignedTrackers(objects_obsved, assignments);

  // 4.旧的对象关联不上的保留一段时间
  ptr_multi_obj_tracker_->updateUnassignedTrackers(unassigned_trackers);

  // 5.移除过时的跟踪器
  ptr_multi_obj_tracker_->removeLostTrackers();

  // 6.为未关联到的观测建立新的跟踪器
  ptr_multi_obj_tracker_->createNewTrackers(objects_obsved,
                                            unassigned_objects_obsved);

  // 7.获取当前有效的对象
  collectTrackingObjects(objects_tracked);
}

void TrackingWorker::collectTrackingObjects(
    std::vector<ObjectPtr> &objects_tracked) {
  auto v_trackers = ptr_multi_obj_tracker_->getTrackers();

  for (size_t i = 0; i < v_trackers.size(); i++) {
    if (v_trackers[i]->isValid()) {
      objects_tracked.push_back(v_trackers[i]->getObject());
    }
  }
}

std::vector<ConstObjectPtr> TrackingWorker::collectExpectedObjects() {
  std::vector<ConstObjectPtr> objects_expected;
  ptr_multi_obj_tracker_->collectExpectedObjects(objects_expected);
  return objects_expected;
}

}  // namespace CVTE_BABOT
