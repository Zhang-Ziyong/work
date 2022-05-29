#include "backend/global_trajectory_builder.hpp"
#include "common/proto_stream.hpp"
#include "common/state_serialization.hpp"

#include "proto/serialization.pb.h"

namespace slam2d_core {
namespace backend {

using slam2d::mapping::proto::SerializedData;

GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(
    const frontend::LocalTrajectoryBuilderOptions
        &local_trajectory_builder_options,
    const PoseGraphOptions &pose_graph_options)
    : thread_pool_(pose_graph_options.num_background_threads) {
  local_trajectory_builder_ptr_ =
      std::make_unique<frontend::LocalTrajectoryBuilder>(
          local_trajectory_builder_options);

  pose_graph_ = std::make_unique<PoseGraph>(
      pose_graph_options,
      std::make_unique<OptimizationProblem>(
          pose_graph_options.optimization_problem_options),
      &thread_pool_);
}

void GlobalTrajectoryBuilder::addRangeData(
    const common::TimedPointCloudData &timed_point_cloud_data) {
  auto matching_result =
      local_trajectory_builder_ptr_->addRangeData(timed_point_cloud_data);
  if (matching_result == nullptr) {
    return;
  }

  std::unique_ptr<InsertionResult> insertion_result;
  if (matching_result->insertion_result != nullptr) {
    auto node_id = pose_graph_->addNode(
        matching_result->insertion_result->constant_data,
        matching_result->insertion_result->insertion_submaps);
    (void) node_id;

    // insertion_result = std::make_unique<InsertionResult>(InsertionResult{
    //     node_id, matching_result->insertion_result->constant_data,
    //     std::vector<std::shared_ptr<const frontend::Submap2D>>(
    //         matching_result->insertion_result->insertion_submaps.begin(),
    //         matching_result->insertion_result->insertion_submaps.end())});
  }

  //   if (local_slam_result_callback_) {
  //     local_slam_result_callback_(matching_result->time,
  //                                 matching_result->local_pose,
  //                                 std::move(matching_result->range_data_in_local),
  //                                 std::move(insertion_result));
  //   }
}

bool GlobalTrajectoryBuilder::getLocalPose(const common::Time &time,
                                           common::Rigid3 &pose) {
  return local_trajectory_builder_ptr_->getLocalPose(time, pose);
}

void GlobalTrajectoryBuilder::addOdomData(
    const common::OdometryData &odometry_data) {
  local_trajectory_builder_ptr_->addOdometryData(odometry_data);
  pose_graph_->addOdometryData(odometry_data);
}

void GlobalTrajectoryBuilder::addImuData(const common::ImuData &imu_data) {
  //!@todo
  // local_trajectory_builder_ptr_->addImuData(imu_data);
  pose_graph_->addImuData(imu_data);
}

std::map<int, common::TrajectoryNodePose>
GlobalTrajectoryBuilder::getTrajectoryNodePoses() const {
  return pose_graph_->getTrajectoryNodePoses();
}

std::vector<Constraint> GlobalTrajectoryBuilder::constraints() const {
  return pose_graph_->constraints();
}

std::map<int, backend::SubmapPose> GlobalTrajectoryBuilder::getAllSubmapPoses()
    const {
  return pose_graph_->getAllSubmapPoses();
}

std::map<int, backend::SubmapData> GlobalTrajectoryBuilder::getAllSubmapData()
    const {
  return pose_graph_->getAllSubmapData();
}

bool GlobalTrajectoryBuilder::submapToTexture(
    const int &submap_id, common::SubmapTexture *const texture) {
  const auto submap_data = pose_graph_->getSubmapData(submap_id);
  if (nullptr == submap_data.submap) {
    LOG(WARNING) << "submap_data.submap is nullptr.";
    return false;
  }
  submap_data.submap->toSubmapTexture(submap_data.pose, texture);
  return true;
}

void GlobalTrajectoryBuilder::runFinalOptimization() {
  pose_graph_->runFinalOptimization();
}

bool GlobalTrajectoryBuilder::getGlobalPose(const common::Time &time,
                                            common::Rigid3 &pose) {
  std::lock_guard<std::mutex> guard(get_global_pose_mutex_);
  if (!local_trajectory_builder_ptr_->getLocalPose(time, pose)) {
    return false;
  }
  pose = pose_graph_->getLocalToGlobalTransform() * pose;
  return true;
}

void GlobalTrajectoryBuilder::setReturnMappingPose(
    const common::Rigid3 &global_pose) {
  pose_graph_->setReturnMappingPose(global_pose);
}

bool GlobalTrajectoryBuilder::serializeStateToFile(
    bool include_unfinished_submaps, const std::string &filename) {
  common::ProtoStreamWriter writer(filename);
  common::writeStream(*pose_graph_, &writer, include_unfinished_submaps);
  return (writer.close());
}

void GlobalTrajectoryBuilder::loadSerializestate(
    common::ProtoStreamReader *const reader) {
  common::ProtoStreamDeserializer deserializer(reader);
  slam2d::mapping::proto::PoseGraph pose_graph_proto =
      deserializer.pose_graph();

  std::map<int, common::Rigid3> submap_poses;
  const slam2d::mapping::proto::Trajectory &trajectory_proto =
      pose_graph_proto.trajectory();
  for (const slam2d::mapping::proto::Trajectory::Submap &submap_proto :
       trajectory_proto.submap()) {
    submap_poses.emplace(submap_proto.submap_index(),
                         common::toRigid3(submap_proto.pose()));
  }

  std::map<int, common::Rigid3> node_poses;
  for (const slam2d::mapping::proto::Trajectory::Node &node_proto :
       trajectory_proto.node()) {
    node_poses.emplace(node_proto.node_index(),
                       common::toRigid3(node_proto.pose()));
  }

  SerializedData proto;
  while (deserializer.readNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;

      case SerializedData::kSubmap: {
        int submap_id = proto.submap().submap_id();
        pose_graph_->addSubmapFromProto(submap_poses.at(submap_id),
                                        proto.submap());
        break;
      }

      case SerializedData::kNode: {
        int node_id = proto.node().node_id();
        pose_graph_->addNodeFromProto(node_poses.at(node_id), proto.node());
        break;
      }

      case SerializedData::kOdometryData: {
        pose_graph_->addOdometryData(common::fromProto(proto.odometry_data()));
        break;
      }

      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  pose_graph_->addSerializedConstraints(
      backend::fromProto(pose_graph_proto.constraint()));
}

void GlobalTrajectoryBuilder::loadSerializeStateFromFile(
    const std::string &state_filename) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }

  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  common::ProtoStreamReader stream(state_filename);
  loadSerializestate(&stream);
}

}  // namespace backend

}  // namespace slam2d_core
