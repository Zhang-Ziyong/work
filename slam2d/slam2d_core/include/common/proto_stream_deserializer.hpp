#ifndef _PROTO_STREAM_DESERIALIZER_H_
#define _PROTO_STREAM_DESERIALIZER_H_

#include <string>

#include "common/proto_stream.hpp"
#include "common/state_serialization.hpp"

#include "proto/serialization.pb.h"

namespace slam2d_core {
namespace common {

slam2d::mapping::proto::PoseGraph deserializePoseGraphFromFile(
    const std::string &file_name);

class ProtoStreamDeserializer {
 public:
  explicit ProtoStreamDeserializer(ProtoStreamReader *const reader);

  ProtoStreamDeserializer(const ProtoStreamDeserializer &) = delete;
  ProtoStreamDeserializer &operator=(const ProtoStreamDeserializer &) = delete;
  ProtoStreamDeserializer(ProtoStreamDeserializer &&) = delete;

  slam2d::mapping::proto::SerializationHeader &header() { return header_; }

  slam2d::mapping::proto::PoseGraph &pose_graph() {
    return *pose_graph_.mutable_pose_graph();
  }

  const slam2d::mapping::proto::PoseGraph &pose_graph() const {
    return pose_graph_.pose_graph();
  }

  bool readNextSerializedData(slam2d::mapping::proto::SerializedData *data);

 private:
  ProtoStreamReader *reader_;

  slam2d::mapping::proto::SerializationHeader header_;
  slam2d::mapping::proto::SerializedData pose_graph_;
};
}  // namespace common
}  // namespace slam2d_core

#endif