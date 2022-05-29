#include "common/state_serialization.hpp"
#include "proto/serialization.pb.h"

namespace slam2d_core {
namespace common {

namespace {

using slam2d::mapping::proto::SerializedData;

slam2d::mapping::proto::SerializationHeader createHeader() {
  slam2d::mapping::proto::SerializationHeader header;
  header.set_format_version(kMappingStateSerializationFormatVersion);
  return header;
}

SerializedData serializePoseGraph(const backend::PoseGraph &pose_graph,
                                  bool include_unfinished_submaps) {
  SerializedData proto;
  *proto.mutable_pose_graph() = pose_graph.toProto(include_unfinished_submaps);
  return proto;
}

void serializeSubmaps(const std::map<int, backend::SubmapData> &submap_data,
                      bool include_unfinished_submaps,
                      ProtoStreamWriter *const writer) {
  for (const auto &submap_id_data : submap_data) {
    if (!include_unfinished_submaps &&
        !submap_id_data.second.submap->insertion_finished()) {
      continue;
    }
    SerializedData proto;
    auto *const submap_proto = proto.mutable_submap();
    *submap_proto->mutable_submap_data() =
        submap_id_data.second.submap->toProto(
            /*include_probability_grid_data=*/true);
    submap_proto->set_submap_id(submap_id_data.first);
    writer->writeProto(proto);
  }
}

void serializeTrajectoryNodes(
    const std::map<int, common::TrajectoryNode> &trajectory_nodes,
    ProtoStreamWriter *const writer) {
  for (const auto &node_id_data : trajectory_nodes) {
    SerializedData proto;
    auto *const node_proto = proto.mutable_node();

    node_proto->set_node_id(node_id_data.first);

    *node_proto->mutable_node_data() =
        common::toProto(*node_id_data.second.constant_data);
    writer->writeProto(proto);
  }
}

void serializeOdometryData(
    const std::map<common::Time, common::OdometryData> &all_odometry_data,
    ProtoStreamWriter *const writer) {
  for (const auto &odometry_data : all_odometry_data) {
    SerializedData proto;
    *proto.mutable_odometry_data() = common::toProto(odometry_data.second);
    writer->writeProto(proto);
  }
}

}  // namespace

void writeStream(const backend::PoseGraph &pose_graph,
                 ProtoStreamWriter *const writer,
                 bool include_unfinished_submaps) {
  writer->writeProto(createHeader());
  writer->writeProto(
      serializePoseGraph(pose_graph, include_unfinished_submaps));
  serializeSubmaps(pose_graph.getAllSubmapData(), include_unfinished_submaps,
                   writer);
  serializeTrajectoryNodes(pose_graph.getTrajectoryNodes(), writer);
  serializeOdometryData(pose_graph.getOdometryData(), writer);
}

}  // namespace common
}  // namespace slam2d_core