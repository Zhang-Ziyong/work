
#include "frontend/pose_extrapolator.hpp"

#include "glog/logging.h"
#include <algorithm>

namespace slam2d_core {
namespace frontend {

PoseExtrapolator::PoseExtrapolator(common::Duration pose_queue_duration)
    : pose_queue_duration_(pose_queue_duration) {}

common::Time PoseExtrapolator::getLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

// common::Time PoseExtrapolator::getLastExtrapolatedTime() const {
//   if (!extrapolation_imu_tracker_) {
//     return common::Time::min();
//   }
//   return extrapolation_imu_tracker_->time();
// }

void PoseExtrapolator::addPose(common::Time time, const common::Rigid3 &pose) {
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  updateVelocitiesFromPoses();

  trimOdometryData();
}

void PoseExtrapolator::addOdometryData(
    const common::OdometryData &odometry_data) {
  if (odometry_data.time < timed_pose_queue_.back().time) {
    return;
  }

  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  trimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }

  const common::OdometryData &odometry_data_oldest = odometry_data_.front();
  const common::OdometryData &odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::toSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const common::Rigid3 odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  angular_velocity_from_odometry_ = common::rotationQuaternionToAngleAxisVector(
                                        odometry_pose_delta.rotation()) /
                                    odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;

  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      extrapolateRotation(odometry_data_newest.time);

  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

bool PoseExtrapolator::extrapolatePose(const common::Time &time,
                                       common::Rigid3 &pose) {
  const TimedPose &newest_timed_pose = timed_pose_queue_.back();
  // CHECK_GE(time, newest_timed_pose.time);
  if (time < newest_timed_pose.time) {
    LOG(WARNING) << "time is < newest_timed_pose.time.";
    return false;
  }

  pose = common::Rigid3::Translation(extrapolateTranslation(time)) *
         newest_timed_pose.pose *
         common::Rigid3::Rotation(extrapolateRotation(time));
  return true;
}

void PoseExtrapolator::updateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose &newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose &oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::toSeconds(newest_time - oldest_time);

  if (queue_delta < 0.001) {  // 1 ms
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration : "
                 << queue_delta << " ms";
    return;
  }

  const common::Rigid3 &newest_pose = newest_timed_pose.pose;
  const common::Rigid3 &oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      common::rotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;

  // LOG(INFO) << "linear_velocity_from_poses_: " <<
  // linear_velocity_from_poses_[0]
  //           << " " << linear_velocity_from_poses_[1];
  // LOG(INFO) << "angular_velocity_from_poses_: "
  //           << angular_velocity_from_poses_[2];
}

void PoseExtrapolator::trimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

Eigen::Quaterniond PoseExtrapolator::extrapolateRotation(
    const common::Time time) const {
  const TimedPose &newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::toSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return common::angleAxisVectorToRotationQuaternion(
        Eigen::Vector3d(angular_velocity_from_poses_ * extrapolation_delta));
  }
  return common::angleAxisVectorToRotationQuaternion(
      Eigen::Vector3d(angular_velocity_from_odometry_ * extrapolation_delta));
}

Eigen::Vector3d PoseExtrapolator::extrapolateTranslation(common::Time time) {
  const TimedPose &newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::toSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace frontend
}  // namespace slam2d_core
