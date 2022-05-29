/*******************************************************************************
 * @file       : gyro_pre_integration.h                                        *
 * @author     : PG.Wang (leatherwang@foxmail.com)                             *
 * @version    : v1.0.0                                                        *
 * @date       : 2021/02/10                                                    *
 * @brief      :                                                               *
 *******************************************************************************
 **-History---------------------------------------------------------------------
 * Version   Date        Name         Changes and comments
 * v1.0.0    2021/02/10  PG.Wang      initial version
 ******************************************************************************/
#pragma once

#include <algorithm>
#include <deque>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "glog/logging.h"
#include "common/time.hpp"

namespace slam2d_core {
namespace backend {
class GyroYawPreintegrator {
 public:
  GyroYawPreintegrator(const double linearized_bg_z, const double linearized_td)
      : linearized_bg_z_(linearized_bg_z), linearized_td_(linearized_td) {}

  GyroYawPreintegrator(const GyroYawPreintegrator &) = delete;
  GyroYawPreintegrator &operator=(const GyroYawPreintegrator &) = delete;

  template <typename RangeType, typename IteratorType>
  void computePreint(const RangeType &imu_data, const common::Time start_time,
                     const common::Time end_time, IteratorType *const it) {
    CHECK_LE(start_time, end_time);
    CHECK(*it != imu_data.end());
    CHECK_LE((*it)->first, start_time);
    if (std::next(*it) != imu_data.end()) {
      CHECK_GT(std::next(*it)->first, start_time);
    }

    common::Time current_time = start_time;
    while (current_time < end_time) {
      common::Time next_imu_data = common::Time::max();
      if (std::next(*it) != imu_data.end()) {
        next_imu_data = std::next(*it)->first;
      }
      common::Time next_time = std::min(next_imu_data, end_time);
      const double delta_t(common::toSeconds(next_time - current_time));

      delta_t_ += delta_t;
      delta_yaw_ +=
          delta_t * ((*it)->second.angular_velocity.template cast<double>()[2] -
                     linearized_bg_z_);

      current_time = next_time;
      if (current_time == next_imu_data) {
        ++(*it);
      }
    }
  }

  void repropagate(const double linearized_bg_z) {
    double corrected_delta_yaw =
        delta_yaw_ - (linearized_bg_z - linearized_bg_z_) * delta_t_;

    delta_yaw_ = corrected_delta_yaw;
    linearized_bg_z_ = linearized_bg_z;
  }

  double getDeltaYaw() const { return delta_yaw_; }
  double getDeltaTime() const { return delta_t_; }
  double getLinearizedTd() const { return linearized_td_; }
  double getLinearizedBgz() const { return linearized_bg_z_; }

  template <typename T>
  T getCorrectedDeltaYaw(const T bg_z) const {
    T corrected_delta_yaw = delta_yaw_ - (bg_z - linearized_bg_z_) * delta_t_;

    return corrected_delta_yaw;
  }

  template <typename T>
  T getCorrectedDeltaTd(const T td) const {
    return td - linearized_td_;
  }

 private:
  double delta_yaw_ = 0.;
  double delta_t_ = 0.;
  double linearized_bg_z_;
  double linearized_td_;
};

}  // namespace backend
}  // namespace slam2d_core
