#ifndef _STATE_SERIALIZATION_H_
#define _STATE_SERIALIZATION_H_
#include "backend/pose_graph.hpp"
#include "proto_stream.hpp"

namespace slam2d_core {
namespace common {
static constexpr int kMappingStateSerializationFormatVersion = 2;
static constexpr int kFormatVersionWithoutSubmapHistograms = 1;

void writeStream(const backend::PoseGraph &pose_graph,
                 ProtoStreamWriter *const writer,
                 bool include_unfinished_submaps);

}  // namespace common
}  // namespace slam2d_core
#endif