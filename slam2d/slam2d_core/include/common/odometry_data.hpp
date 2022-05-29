#ifndef _COMMON_ODOMETRY_DATA_H_
#define _COMMON_ODOMETRY_DATA_H_

#include "rigid_transform.hpp"
#include "time.hpp"

#include "proto/sensor.pb.h"

namespace slam2d_core {
namespace common {

struct OdometryData {
  Time time;
  Rigid3 pose;
};

inline slam2d::mapping::proto::OdometryData toProto(
    const OdometryData &odometry_data) {
  slam2d::mapping::proto::OdometryData proto;
  proto.set_timestamp(common::toUniversal(odometry_data.time));
  *proto.mutable_pose() = common::toProto(odometry_data.pose);
  return proto;
}

inline OdometryData fromProto(
    const slam2d::mapping::proto::OdometryData &proto) {
  return OdometryData{common::fromUniversal(proto.timestamp()),
                      common::toRigid3(proto.pose())};
}

}  // namespace common
}  // namespace slam2d_core

#endif
