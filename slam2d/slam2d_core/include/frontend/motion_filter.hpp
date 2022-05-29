#ifndef _MOTION_FILTER_H_
#define _MOTION_FILTER_H_

#include <limits>

#include "common/math.hpp"
#include "common/rigid_transform.hpp"
#include "common/time.hpp"

namespace slam2d_core {
namespace frontend {

class MotionFilterOptions {
 public:
  double max_time_seconds;
  double max_distance_meters;
  double max_angle_radians;
};

class MotionFilter {
 public:
  explicit MotionFilter(const MotionFilterOptions &options);
  bool isSimilar(common::Time time, const common::Rigid3 &pose);

 private:
  int num_total_ = 0;
  int num_different_ = 0;
  common::Time last_time_;
  common::Rigid3 last_pose_;
  MotionFilterOptions options_;
};

}  // namespace frontend
}  // namespace slam2d_core

#endif
