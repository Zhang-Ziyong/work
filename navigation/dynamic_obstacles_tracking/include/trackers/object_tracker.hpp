/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file object_tracker.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#ifndef __OBJECT_TRACKER_HPP
#define __OBJECT_TRACKER_HPP
#include "common/tracking_utils.hpp"
#include "filters/mean_filter.hpp"

namespace CVTE_BABOT {

class ObjectTracker {
 public:
  ObjectTracker(const ObjectPtr &object_obsved, const IdType &tracker_id);

  /**
   *execPredict
   *@brief
   *预测该类下一帧的位置
   *
   *@param[out] tracker_predict-下一帧的位置
  **/
  void execPredict(Eigen::VectorXd &tracker_predict) const;

  void updateWithObservation(const ObjectPtr &object_obsved);

  void updateWithoutObservation(const double &time_diff);

  bool isLost() const {
    return ui_consecutive_invisible_count_ >
           sui_tracker_consecutive_invisible_maximum_;
  }

  bool isValid() const {
    return ptr_object_->tracking_state != TEMPORARY_OBJECT;
  }

  bool isTracked() const {
    return ptr_object_->tracking_state == TRACKED_OBJECT;
  }

  ConstObjectPtr getConstObject() const { return ptr_object_; }

  ObjectPtr getObject() { return ptr_object_; }

  Trajectory getTrajectory() const { return trajectory_; }

  IdType getId() const { return idx_; }

 private:
  void removeDatedTrajectory();

  void updateTrajectory(const double &origin_x, const double &origin_y);

  void updateVelocity(const Trajectory &trajectory, double &velocity_x,
                      double &velocity_y);

  void updateMotionStatus(const ObjectPtr &object_obsved);

  inline double distance2D(const Eigen::Vector2d &p1,
                           const Eigen::Vector2d &p2) {
    double x_error = fabs(p1(0) - p2(0));
    double y_error = fabs(p1(1) - p2(1));
    return sqrt(x_error * x_error + y_error * y_error);
  }

  ObjectPtr ptr_object_;

  MeanFilter mean_fitler_;

  IdType idx_;

  Trajectory trajectory_;

  double d_velocity_x_ = 0.0;
  double d_velocity_y_ = 0.0;

  uint32_t ui_consecutive_invisible_count_ = 0;

  uint32_t mobile_count_ = 0;

  Eigen::Vector2d origin_point_;  // x, y

  static uint32_t sui_filter_num_;

  static uint32_t sui_tracker_consecutive_invisible_maximum_;

  static double sd_distance_threshold_;  ///<判断障碍是否移动的距离阀值
  static uint32_t
      sui_mobile_count_threshold_;  ///< 障碍连续移动几帧后认为是运动的
};

typedef std::shared_ptr<ObjectTracker> ObjectTrackerPtr;
typedef std::shared_ptr<const ObjectTracker> ConstObjectTrackerPtr;
}

#endif  // GROUND_HPP_
