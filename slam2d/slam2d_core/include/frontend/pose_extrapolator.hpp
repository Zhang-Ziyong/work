
#ifndef _POSE_EXTRAPOLATOR_H_
#define _POSE_EXTRAPOLATOR_H_

#include "common/math.hpp"
#include "common/odometry_data.hpp"
#include "common/time.hpp"
#include "glog/logging.h"
#include <Eigen/Core>
#include <deque>
#include <memory>

namespace slam2d_core {
namespace frontend {

class PoseExtrapolator {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration);

  PoseExtrapolator(const PoseExtrapolator &) = delete;
  PoseExtrapolator &operator=(const PoseExtrapolator &) = delete;

  //获取最后更新位置的时间
  common::Time getLastPoseTime() const;
  //   common::Time getLastExtrapolatedTime() const; //是否一定需要

  void addPose(common::Time time, const common::Rigid3 &pose);

  //添加里程
  void addOdometryData(const common::OdometryData &odometry_data);

  // 根据输入的时间推断姿态
  bool extrapolatePose(const common::Time &time, common::Rigid3 &pose);

 private:
  void updateVelocitiesFromPoses();
  void trimOdometryData();

  Eigen::Quaterniond extrapolateRotation(const common::Time time) const;
  Eigen::Vector3d extrapolateTranslation(const common::Time time);

  const common::Duration pose_queue_duration_;

  struct TimedPose {
    common::Time time;
    common::Rigid3 pose;
  };

  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  std::deque<common::OdometryData> odometry_data_;
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace frontend
}  // namespace slam2d_core

#endif
