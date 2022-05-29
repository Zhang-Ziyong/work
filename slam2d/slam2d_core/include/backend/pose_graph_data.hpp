#ifndef _POSE_GRAPH_DATA_H_
#define _POSE_GRAPH_DATA_H_

#include "common/rigid_transform.hpp"
#include "common/trajectory_node.hpp"
#include "frontend/submap_2d.hpp"
#include <map>
#include <set>
#include <vector>

namespace slam2d_core {
namespace backend {

struct NodeSpec2D {
  common::Time time;
  common::Rigid2 local_pose_2d;
  common::Rigid2 global_pose_2d;
};

struct SubmapSpec2D {
  common::Rigid2 global_pose;
};

struct Constraint {
  struct Pose {
    common::Rigid3 zbar_ij;
    double translation_weight;
    double rotation_weight;
  };

  int submap_id;  // 'i' in the paper.
  int node_id;    // 'j' in the paper.

  Pose pose;
  enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
};

struct SubmapPose {
  int version;
  common::Rigid3 pose;
};

struct SubmapData {
  std::shared_ptr<const frontend::Submap2D> submap;
  common::Rigid3 pose;
};

enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };

enum class SubmapState { kNoConstraintSearch, kFinished };

struct InitialTrajectoryPose {
  common::Time time;
  common::Rigid3 relative_pose;
};

struct InternalTrajectoryState {
  enum class DeletionState {
    NORMAL,
    SCHEDULED_FOR_DELETION,
    WAIT_FOR_DELETION
  };

  TrajectoryState state = TrajectoryState::ACTIVE;
  DeletionState deletion_state = DeletionState::NORMAL;
};

struct InternalSubmapData {
  std::shared_ptr<const frontend::Submap2D> submap;
  SubmapState state = SubmapState::kNoConstraintSearch;

  std::set<int> node_ids;
};

struct PoseGraphData {
  std::map<int, InternalSubmapData> submap_data;           // int -> submap id.
  std::map<int, SubmapSpec2D> global_submap_poses_2d;      // int ->submap id.
  std::map<int, common::TrajectoryNode> trajectory_nodes;  // int -> node id.

  int num_trajectory_nodes = 0;
  InternalTrajectoryState trajectories_state;
  InitialTrajectoryPose initial_trajectory_pose;

  std::vector<Constraint> constraints;
};

using GlobalSlamOptimizationCallback =
    std::function<void(const int /* SubmapId */ &, const int /* NodeId */ &)>;
}  // namespace backend
}  // namespace slam2d_core

#endif
