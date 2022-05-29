#ifndef _TRAJECTORY_NODE_H_
#define _TRAJECTORY_NODE_H_

#include <memory>
#include <optional>
#include <vector>

#include "Eigen/Core"
#include "point_cloud.hpp"
#include "rigid_transform.hpp"
#include "time.hpp"

#include "proto/trajectory_node_data.pb.h"

namespace slam2d_core {
namespace common {

struct TrajectoryNodePose {
  struct ConstantPoseData {
    common::Time time;
    common::Rigid3 local_pose;
  };

  common::Rigid3 global_pose;
  std::optional<ConstantPoseData> constant_pose_data;
};

struct TrajectoryNode {
  struct Data {
    common::Time time;
    common::PointCloud point_cloud;
    common::Rigid3 local_pose;
    double move_distance;
  };

  common::Time time() const { return constant_data->time; }
  std::shared_ptr<const Data> constant_data;
  common::Rigid3 global_pose;
};

inline slam2d::mapping::proto::TrajectoryNodeData toProto(
    const TrajectoryNode::Data &constant_data) {
  slam2d::mapping::proto::TrajectoryNodeData proto;
  proto.set_timestamp(common::toUniversal(constant_data.time));
  *proto.mutable_point_cloud() = constant_data.point_cloud.toProto();
  *proto.mutable_local_pose() = common::toProto(constant_data.local_pose);
  proto.set_move_distance(constant_data.move_distance);
  return proto;
}

inline TrajectoryNode::Data fromProto(
    const slam2d::mapping::proto::TrajectoryNodeData &proto) {
  return TrajectoryNode::Data{common::fromUniversal(proto.timestamp()),
                              common::PointCloud(proto.point_cloud()),
                              common::toRigid3(proto.local_pose()),
                              proto.move_distance()};
}

}  // namespace common
}  // namespace slam2d_core

#endif
