#include "frontend/motion_filter.hpp"
#include "glog/logging.h"

namespace slam2d_core {
namespace frontend {

MotionFilter::MotionFilter(const MotionFilterOptions &options)
    : options_(options) {}

bool MotionFilter::isSimilar(const common::Time time,
                             const common::Rigid3 &pose) {
  LOG_IF_EVERY_N(INFO, num_total_ >= 500, 500)
      << "Motion filter reduced the number of nodes to "
      << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  if (num_total_ > 1 &&
      time - last_time_ <= common::fromSeconds(options_.max_time_seconds) &&
      (pose.translation() - last_pose_.translation()).norm() <=
          options_.max_distance_meters &&
      (pose.inverse() * last_pose_).getAngle() <= options_.max_angle_radians) {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

}  // namespace frontend
}  // namespace slam2d_core
