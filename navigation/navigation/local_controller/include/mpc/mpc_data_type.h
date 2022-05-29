/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath:
 /src/navigation/navigation/local_controller/include/mpc/mpc_data_type.h
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2021-01-12 11:07:31
 ************************************************************************/
#pragma once

namespace CVTE_BABOT {

enum TrajectoryType { Circles = 0, Square, SinCurve };

struct Waypoint {
  Waypoint(const double x, const double y, const double theta,
           const double curve = 0.)
      : x(x), y(y), theta(theta), curve(curve) {}
  double x;
  double y;
  double theta;
  double curve;

};  // class Waypoint

// struct Velocity {
//   Velocity() {}
//   Velocity(const double x, const double y, const double d_yaw)
//       : d_x(x), d_y(y), d_yaw(d_yaw) {}
//   double d_x;
//   double d_y;
//   double d_yaw;
// };

struct MpcState {
  MpcState() {}
  MpcState(const double x, const double y, const double theta)
      : x(x), y(y), theta(theta) {}
  MpcState(const double x, const double y, const double theta,
           const double linear_speed, const double angular_speed = 0.)
      : x(x),
        y(y),
        theta(theta),
        linear_speed(linear_speed),
        angular_speed(angular_speed) {}
  double x;
  double y;
  double theta;
  double linear_speed;
  double angular_speed;
  double time_stamp = -1.0;
};

}  // namespace CVTE_BABOT