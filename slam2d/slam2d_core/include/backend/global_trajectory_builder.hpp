#ifndef _GLOBAL_TRAJECTORY_BUILDER_H_
#define _GLOBAL_TRAJECTORY_BUILDER_H_

#include "backend/global_trajectory_builder.hpp"
#include "backend/optimization_problem.hpp"
#include "backend/pose_graph.hpp"
#include "common/trajectory_node.hpp"
#include "frontend/local_trajectory_builder.hpp"
#include "frontend/submap_2d.hpp"
#include "common/proto_stream.hpp"
#include "common/proto_stream_deserializer.hpp"

#include <memory>
#include <mutex>
#include <string>

namespace slam2d_core {
namespace backend {
class GlobalTrajectoryBuilder {
 public:
  struct InsertionResult {
    int node_id;
    std::shared_ptr<const common::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const frontend::Submap2D>> insertion_submaps;
  };

  using LocalSlamResultCallback =
      std::function<void(common::Time, common::Rigid3 /* local pose estimate */,
                         common::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;

  GlobalTrajectoryBuilder(const frontend::LocalTrajectoryBuilderOptions
                              &local_trajectory_builder_options,
                          const PoseGraphOptions &pose_graph_options);
  ~GlobalTrajectoryBuilder() {}

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder &) = delete;
  GlobalTrajectoryBuilder &operator=(const GlobalTrajectoryBuilder &) = delete;

  void runFinalOptimization();
  bool getLocalPose(const common::Time &time, common::Rigid3 &pose);
  bool getGlobalPose(const common::Time &time, common::Rigid3 &pose);

  void addOdomData(const common::OdometryData &odometry_data);
  void addRangeData(const common::TimedPointCloudData &timed_point_cloud_data);
  void addImuData(const common::ImuData &imu_data);

  std::map<int, common::TrajectoryNodePose> getTrajectoryNodePoses()
      const;  // int -> node id.

  std::map<int, backend::SubmapPose> getAllSubmapPoses() const;
  std::vector<Constraint> constraints() const;
  std::map<int, backend::SubmapData> getAllSubmapData() const;

  bool submapToTexture(const int &submap_id,
                       common::SubmapTexture *const texture);

  void setReturnMappingPose(const common::Rigid3 &global_pose);

  bool serializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename);

  void loadSerializestate(common::ProtoStreamReader *const reader);

  void loadSerializeStateFromFile(const std::string &state_filename);

  void setTrimData() { pose_graph_->setTrimData(); }

 private:
  std::mutex get_global_pose_mutex_;

  common::ThreadPool thread_pool_;
  std::unique_ptr<PoseGraph> pose_graph_;

  LocalSlamResultCallback local_slam_result_callback_;
  std::unique_ptr<frontend::LocalTrajectoryBuilder>
      local_trajectory_builder_ptr_;
};

}  // namespace backend

}  // namespace slam2d_core

#endif
