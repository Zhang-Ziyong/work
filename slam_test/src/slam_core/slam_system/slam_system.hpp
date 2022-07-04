#ifndef _SLAM_SYSTEM_HPP_
#define _SLAM_SYSTEM_HPP_

#include <iostream>
#include <deque>

#include "common/sensors_type.hpp"

namespace EIOLIDAR_SLAM {
class System {
 public:
  explicit System();
  //   System() = delete;
  ~System();
  System(const System &obj) = delete;
  System &operator=(const System &obj) = delete;

  // 生成单例，用于不同类之间进行调用
  static System *getInstance();

  inline void addOdomData(const OdomMeasure &odom_measure) {
    while (odom_data_buffer_.size() > 100) { odom_data_buffer_.pop_front(); }
    odom_data_buffer_.push_back(odom_measure);
  }

  inline void addImuData(const ImuMeasure &imu_measure) {
    while (imu_data_buffer_.size() > 200) { imu_data_buffer_.pop_front(); }
    imu_data_buffer_.push_back(imu_measure);
  }

 private:
  std::deque<OdomMeasure> odom_data_buffer_;
  std::deque<ImuMeasure> imu_data_buffer_;

 private:
  static System *ptr_system_;
};
}  // namespace EIOLIDAR_SLAM

#endif