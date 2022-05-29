/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file object_tracker.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#include <glog/logging.h>
#include <Eigen/Core>

#include "trackers/object_tracker.hpp"

namespace CVTE_BABOT {
uint32_t ObjectTracker::sui_filter_num_ = 5;
uint32_t ObjectTracker::sui_tracker_consecutive_invisible_maximum_ = 10;

double ObjectTracker::sd_distance_threshold_ = 2.5;
uint32_t ObjectTracker::sui_mobile_count_threshold_ = 5;

ObjectTracker::ObjectTracker(const ObjectPtr &object_obsved,
                             const IdType &tracker_id)
    : mean_fitler_(sui_filter_num_),
      idx_(tracker_id),
      ui_consecutive_invisible_count_(0) {
  // 用均值滤波做了轨迹平滑
  updateTrajectory(object_obsved->object_center(0),
                   object_obsved->object_center(1));

  // 初始化对象
  ptr_object_ = std::make_shared<Object>();
  ptr_object_->id = idx_;
  ptr_object_->tracking_state = NEW_OBJECT;

  ptr_object_->copyPointsFeature(*object_obsved);  // 浅拷贝

  ptr_object_->setMotionFeature(trajectory_, 0.00, 0.00);

  // 记录该类的原始点
  origin_point_ = object_obsved->object_center.head(2);
}

void ObjectTracker::execPredict(Eigen::VectorXd &tracker_predict) const {
  // x, y, radius
  tracker_predict.resize(3);

  const double time_diff = 0.1;  // 0.1s
  tracker_predict(0) = trajectory_.back()(0) + d_velocity_x_ * time_diff;
  tracker_predict(1) = trajectory_.back()(1) + d_velocity_y_ * time_diff;

  // 内切圆半径
  tracker_predict(2) = std::min(ptr_object_->length, ptr_object_->width) / 2.0;
}

void ObjectTracker::updateMotionStatus(const ObjectPtr &object_obsved) {
  // 低矮障碍认为是不移动的
  const double low_obj_height_threshold = 0.5;
  if (object_obsved->height < low_obj_height_threshold) {
    return;
  } else if (ptr_object_->height < low_obj_height_threshold) {
    // 先前是低矮障碍，重新计算原始点
    origin_point_ = object_obsved->object_center.head(2);
    return;
  }

  // 边长变化太大的话,重新记录该类的原始点
  const double edge_change_threshold = 0.7;
  double length_error = fabs(ptr_object_->length - object_obsved->length);
  double width_error = fabs(ptr_object_->width - object_obsved->width);

  if (length_error > edge_change_threshold ||
      width_error > edge_change_threshold) {
    mobile_count_ = 0;
    origin_point_ = object_obsved->object_center.head(2);
    return;
  }

  double distance_to_last = distance2D(object_obsved->object_center.head(2),
                                       ptr_object_->object_center.head(2));

  double threshold = sd_distance_threshold_;
  if (object_obsved->width < 1.0 && object_obsved->length < 1.0) {
    threshold = 1.0;
  }

  // 计算障碍是否移动
  double distance_to_origin =
      distance2D(object_obsved->object_center.head(2), origin_point_);
  if (distance_to_origin > threshold) {
    mobile_count_++;
  } else {
    mobile_count_ = 0;
  }

  if (mobile_count_ > sui_mobile_count_threshold_ && distance_to_last > 0.05) {
    ptr_object_->motion_state = MOBILE_OBJECT;
  }
}

void ObjectTracker::updateWithObservation(const ObjectPtr &object_obsved) {
  ui_consecutive_invisible_count_ = 0;

  // 更新轨迹
  // 用均值滤波做了轨迹平滑
  updateTrajectory(object_obsved->object_center(0),
                   object_obsved->object_center(1));

  updateVelocity(trajectory_, d_velocity_x_, d_velocity_y_);

  updateMotionStatus(object_obsved);

  if (fabs(d_velocity_x_) > 3.0 || fabs(d_velocity_y_) > 3.0) {
    // LOG(ERROR) << "Observation object id: " << idx_
    //            << ", current_x: " << trajectory_.back()(0)
    //            << ", current_y: " << trajectory_.back()(1)
    //            << ", velocity_x: " << d_velocity_x_
    //            << ", velocity_y: " << d_velocity_y_;
  } else {
    // LOG(INFO) << "Observation object id: " << idx_
    //           << ", current_x: " << trajectory_.back()(0)
    //           << ", current_y: " << trajectory_.back()(1)
    //           << ", velocity_x: " << d_velocity_x_
    //           << ", velocity_y: " << d_velocity_y_;
  }

  ptr_object_->copyPointsFeature(*object_obsved);
  ptr_object_->setMotionFeature(trajectory_, d_velocity_x_, d_velocity_y_);

  ptr_object_->tracking_state = TRACKED_OBJECT;
}

void ObjectTracker::updateWithoutObservation(const double &time_diff) {
  ui_consecutive_invisible_count_++;

  // 1.计算位移
  Eigen::Vector2d predicted_shift;
  predicted_shift(0) = d_velocity_x_ * time_diff;
  predicted_shift(1) = d_velocity_y_ * time_diff;

  // 2.更新轨迹和速度
  Eigen::Vector2d object_center = trajectory_.back();
  Eigen::Vector2d predicted_center = object_center + predicted_shift;

  updateTrajectory(predicted_center(0), predicted_center(1));

  updateVelocity(trajectory_, d_velocity_x_, d_velocity_y_);
  // LOG(INFO) << "outObservation object id: " << idx_
  //           << ", current_x: " << trajectory_.back()(0)
  //           << ", current_y: " << trajectory_.back()(1)
  //           << ", velocity_x: " << d_velocity_x_
  //           << ", velocity_y: " << d_velocity_y_;

  // 3.更新对象信息,是否需要深拷贝?
  ptr_object_->setMotionFeature(trajectory_, d_velocity_x_,
                                d_velocity_y_);  //浅拷贝

  ptr_object_->object_center(0) = predicted_center(0);
  ptr_object_->object_center(1) = predicted_center(1);

  ptr_object_->tracking_state = TEMPORARY_OBJECT;
}

void ObjectTracker::removeDatedTrajectory() {
  // remain 5
  if (trajectory_.size() > 30) {
    trajectory_.erase(trajectory_.begin(), trajectory_.begin() + 26);
    assert(trajectory_.size() == 5);
  }
}

void ObjectTracker::updateTrajectory(const double &origin_x,
                                     const double &origin_y) {
  TrajectoryPoint filtered_point;
  filtered_point.setZero();

  mean_fitler_.filter(origin_x, origin_y, filtered_point(0), filtered_point(1));

  trajectory_.push_back(filtered_point);

  removeDatedTrajectory();
}

void ObjectTracker::updateVelocity(const Trajectory &trajectory,
                                   double &velocity_x, double &velocity_y) {
  auto size = trajectory.size();
  // 至少两个点才能计算速度
  if (size < 2) {
    velocity_x = 0.00;
    velocity_y = 0.00;
    return;
  }

  // 小于5个点使用最近的两帧计算速度
  if (size < 5) {
    velocity_x = trajectory[size - 1](0) - trajectory[size - 2](0);
    velocity_y = trajectory[size - 1](1) - trajectory[size - 2](1);
    return;
  }

  TrajectoryPoint velocity0 = trajectory[size - 1] - trajectory[size - 2];

  TrajectoryPoint velocity1 = trajectory[size - 2] - trajectory[size - 3];

  TrajectoryPoint velocity2 = trajectory[size - 3] - trajectory[size - 4];

  TrajectoryPoint velocity3 = trajectory[size - 4] - trajectory[size - 5];

  // 低通滤波.计算得到时间间隔0.1ｓ的速度，所以要乗上10得到秒速
  velocity_x = (velocity0(0) * 0.4 + velocity1(0) * 0.3 + velocity2(0) * 0.2 +
                velocity3(0) * 0.1) *
               10.0;
  velocity_y = (velocity0(1) * 0.4 + velocity1(1) * 0.3 + velocity2(1) * 0.2 +
                velocity3(1) * 0.1) *
               10.0;
}

}  // namespace CVTE_BABOT