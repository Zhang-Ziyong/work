#ifndef _COMMON_IMU_DATA_H_
#define _COMMON_IMU_DATA_H_

#include "rigid_transform.hpp"
#include "time.hpp"

namespace slam2d_core {
namespace common {

struct ImuData {
  common::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
};

}  // namespace common
}  // namespace slam2d_core

#endif
