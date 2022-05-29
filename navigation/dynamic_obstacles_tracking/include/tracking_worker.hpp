/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file tracking_worker.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#ifndef __TRACKING_WORKER_HPP
#define __TRACKING_WORKER_HPP
#include "common/tracking_utils.hpp"

namespace CVTE_BABOT {
class ObservationMatcher;
class MultiObjectTracker;

class TrackingWorker {
 public:
  TrackingWorker();
  ~TrackingWorker() = default;

  TrackingWorker(const TrackingWorker &) = delete;
  TrackingWorker &operator=(const TrackingWorker &) = delete;

  void track(const std::vector<ObjectPtr> &objects_obsved,
             std::vector<ObjectPtr> &objects_tracked);

  std::vector<ConstObjectPtr> collectExpectedObjects();

  void collectTrackingObjects(std::vector<ObjectPtr> &objects_tracked);

 private:
  std::shared_ptr<ObservationMatcher> ptr_matcher_;
  std::shared_ptr<MultiObjectTracker> ptr_multi_obj_tracker_;
};
}  // namespace CVTE_BABOT
#endif
