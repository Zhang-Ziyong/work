#include "common/proto_stream_deserializer.hpp"

namespace slam2d_core {
namespace common {

namespace {
slam2d::mapping::proto::SerializationHeader readHeaderOrDie(
    ProtoStreamReader *const reader) {
  slam2d::mapping::proto::SerializationHeader header;
  CHECK(reader->readProto(&header)) << "Failed to read SerializationHeader.";
  return header;
}

bool isVersionSupported(
    const slam2d::mapping::proto::SerializationHeader &header) {
  return header.format_version() ==
             common::kMappingStateSerializationFormatVersion ||
         header.format_version() ==
             common::kFormatVersionWithoutSubmapHistograms;
}

}  // namespace

slam2d::mapping::proto::PoseGraph deserializePoseGraphFromFile(
    const std::string &file_name) {
  ProtoStreamReader reader(file_name);
  ProtoStreamDeserializer deserializer(&reader);
  return deserializer.pose_graph();
}

ProtoStreamDeserializer::ProtoStreamDeserializer(
    ProtoStreamReader *const reader)
    : reader_(reader), header_(readHeaderOrDie(reader)) {
  CHECK(isVersionSupported(header_)) << "Unsupported serialization format \""
                                     << header_.format_version() << "\"";

  CHECK(readNextSerializedData(&pose_graph_))
      << "Serialized stream misses PoseGraph.";
  CHECK(pose_graph_.has_pose_graph())
      << "Serialized stream order corrupt. Expecting `PoseGraph` after "
         "`SerializationHeader`, but got field tag "
      << pose_graph_.data_case();
}

bool ProtoStreamDeserializer::readNextSerializedData(
    slam2d::mapping::proto::SerializedData *data) {
  return reader_->readProto(data);
}

}  // namespace common
}  // namespace slam2d_core