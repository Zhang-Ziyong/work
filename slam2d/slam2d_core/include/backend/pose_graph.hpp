#ifndef _POSE_GRAPH_H_
#define _POSE_GRAPH_H_

#include <optional>
#include <vector>
#include <map>

#include "backend/constraint_builder.hpp"
#include "backend/optimization_problem.hpp"
#include "backend/pose_graph_data.hpp"
#include "common/mutex.hpp"
#include "common/rigid_transform.hpp"
#include "common/task.hpp"
#include "common/value_conversion_tables.hpp"

#include "proto/pose_graph.pb.h"
#include "proto/serialization.pb.h"
namespace slam2d_core {
namespace backend {

class PoseGraphOptions {
 public:
  double scan_sampling_ratio = 0.3;      ///< scan数据采样率
  double odometry_sampling_ratio = 0.3;  ///< 里程数据采样率
  double imu_sampling_ratio = 1.0;       ///< IMU数据采样率

  int num_background_threads = 4;
  bool log_residual_histograms = true;
  int32_t optimize_every_n_nodes = 20;
  double global_sampling_ratio = 0.003;
  int32_t max_num_final_iterations = 200;
  double matcher_rotation_weight = 1.6e3;
  double matcher_translation_weight = 5e2;
  double global_constraint_search_after_n_seconds = 10;

  int32_t pure_localization_submaps_keep = 3;

  ConstraintBuilderOptions constraint_builder_options;
  OptimizationProblemOptions optimization_problem_options;

  class OverlappingSubmapsTrimmerOptions2D {
   public:
    int32_t fresh_submaps_count;
    double min_covered_area;
    int32_t min_added_submaps_count;
  };

  OverlappingSubmapsTrimmerOptions2D overlapping_submaps_trimmer_2d;
};

class PoseGraph {
 public:
  PoseGraph(const PoseGraphOptions &options,
            std::unique_ptr<OptimizationProblem> optimization_problem,
            common::ThreadPool *thread_pool);
  ~PoseGraph();
  PoseGraph(const PoseGraph &) = delete;
  PoseGraph &operator=(const PoseGraph &) = delete;

  int addNode(std::shared_ptr<const common::TrajectoryNode::Data> constant_data,
              const std::vector<std::shared_ptr<const frontend::Submap2D>>
                  &insertion_submaps);
  void addOdometryData(const common::OdometryData &odometry_data);
  void addImuData(const common::ImuData &imu_data);

  void finishTrajectory();
  bool isTrajectoryFinished() const;

  void addNodeToSubmap(const int &node_id, const int &submap_id);
  void runFinalOptimization();

  SubmapData getSubmapData(const int &submap_id) const;
  std::map<int, SubmapData> getAllSubmapData() const;   // int -> submap id.
  std::map<int, SubmapPose> getAllSubmapPoses() const;  // int -> submap id.
  common::Rigid3 getLocalToGlobalTransform() const;
  std::map<int, common::TrajectoryNode> getTrajectoryNodes()
      const;  // int -> node id.
  std::map<int, common::TrajectoryNodePose> getTrajectoryNodePoses()
      const;  // int -> node id.
  std::vector<Constraint> constraints() const;

  common::Rigid3 getInterpolatedGlobalTrajectoryPose(
      const common::Time time) const;

  InternalTrajectoryState getTrajectoryStates() const;

  std::map<common::Time, common::OdometryData> getOdometryData() const;

  void addSerializedConstraints(const std::vector<Constraint> &constraints);

  slam2d::mapping::proto::PoseGraph toProto(
      bool include_unfinished_submaps) const;

  void addSubmapFromProto(const common::Rigid3 &global_submap_pose,
                          const slam2d::mapping::proto::SubmapData &submap);

  void addNodeFromProto(const common::Rigid3 &global_pose,
                        const slam2d::mapping::proto::Node &node);

  void setReturnMappingPose(const common::Rigid3 &global_pose);

  void setTrimData() { is_trim_data_ = true; }

 private:
  std::map<int, SubmapData> getSubmapDataUnderLock() const;
  void addWorkItem(const std::function<common::WorkItem::Result()> &work_item);

  int appendNode(
      std::shared_ptr<const common::TrajectoryNode::Data> constant_data,
      const std::vector<std::shared_ptr<const frontend::Submap2D>>
          &insertion_submaps,
      const common::Rigid3 &optimized_pose);  // int -> node id.

  std::vector<int> initializeGlobalSubmapPoses(
      const common::Time time,
      const std::vector<std::shared_ptr<const frontend::Submap2D>>
          &insertion_submaps);

  common::WorkItem::Result computeConstraintsForNode(
      const int &node_id,
      std::vector<std::shared_ptr<const frontend::Submap2D>> insertion_submaps,
      bool newly_finished_submap);

  void drainWorkQueue();
  void runOptimization();
  void waitForAllComputations();
  bool canAddWorkItemModifying();
  void handleWorkQueue(const ConstraintBuilder::Result &result);
  SubmapData getSubmapDataUnderLock(const int &submap_id) const;
  void computeConstraint(const int &node_id, const int &submap_id);

  common::Rigid3 computeLocalToGlobalTransform(
      const std::map<int, SubmapSpec2D> &global_submap_poses) const;

  common::Time getLatestNodeTime(const int &node_id,
                                 const int &submap_id) const;

  void trimData();
  void trimSubmap(const int &submap_id);

  bool is_trim_data_ = false;
  bool return_mapping_ = false;
  int return_mapping_nodeid_ = 0;
  int return_mapping_submapid_ = 0;
  common::Rigid3 return_mapping_pose_;

  PoseGraphData data_;
  mutable common::Mutex mutex_;
  common::Mutex work_queue_mutex_;
  const PoseGraphOptions options_;
  ConstraintBuilder constraint_builder_;
  common::ThreadPool *const thread_pool_;
  std::unique_ptr<common::WorkQueue> work_queue_;
  common::ValueConversionTables conversion_tables_;
  std::unique_ptr<OptimizationProblem> optimization_problem_;
  GlobalSlamOptimizationCallback global_slam_optimization_callback_;
  std::unique_ptr<common::FixedRatioSampler> global_localization_sampler_;

  int num_nodes_since_last_loop_closure_ = 0;
};

std::vector<Constraint> fromProto(
    const ::google::protobuf::RepeatedPtrField<
        ::slam2d::mapping::proto::PoseGraph::Constraint> &constraint_protos);

slam2d::mapping::proto::PoseGraph::Constraint toProto(
    const Constraint &constraint);

}  // namespace backend
}  // namespace slam2d_core

#endif
