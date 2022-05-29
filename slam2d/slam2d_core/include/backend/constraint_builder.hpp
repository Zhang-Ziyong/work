#ifndef _CONSTRAINT_BUILDER_H_
#define _CONSTRAINT_BUILDER_H_

#include "backend/histogram.hpp"
#include "backend/pose_graph_data.hpp"
#include "common/fixed_ratio_sampler.hpp"
// #include "common/grid_map.hpp"
#include "common/probability_grid.hpp"
#include "common/mutex.hpp"
#include "common/rigid_transform.hpp"
#include "common/thread_pool.hpp"
#include "common/trajectory_node.hpp"
#include "frontend/submap_2d.hpp"
#include "scan_matcher/ceres_scan_matcher_2d.hpp"
#include "scan_matcher/fast_correlative_scan_matcher_2d.hpp"

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace slam2d_core {
namespace backend {

common::Rigid2 computeSubmapPose(const frontend::Submap2D &submap);

class ConstraintBuilderOptions {
 public:
  double min_score = 0.5;
  bool log_matches = true;
  double sampling_ratio = 0.1;
  double max_constraint_distance = 15;
  double global_localization_min_score = 0.5;
  double loop_closure_rotation_weight = 1e5;
  double loop_closure_translation_weight = 1.1e4;
  double loop_closure_error_ratio = 0.1;

  scan_matcher::FastCorrelativeScanMatcherOptions2D
      fast_correlative_scan_matcher_options;
  scan_matcher::CeresScanMatcherOptions2D ceres_scan_matcher_options;
};

class ConstraintBuilder {
 public:
  using Result = std::vector<Constraint>;

  ConstraintBuilder(const ConstraintBuilderOptions &options,
                    common::ThreadPool *thread_pool);
  ~ConstraintBuilder();

  ConstraintBuilder(const ConstraintBuilder &) = delete;
  ConstraintBuilder &operator=(const ConstraintBuilder &) = delete;

  void maybeAddConstraint(
      const int &submap_id, const frontend::Submap2D *submap,
      const int &node_id,
      const common::TrajectoryNode::Data *const constant_data,
      const common::Rigid2 &initial_relative_pose, const double &distance);

  void maybeAddGlobalConstraint(
      const int &submap_id, const frontend::Submap2D *submap,
      const int &node_id,
      const common::TrajectoryNode::Data *const constant_data);

  void notifyEndOfNode();
  void whenDone(const std::function<void(const Result &)> &callback);

  int getNumFinishedNodes();
  void deleteScanMatcher(const int &submap_id);

 private:
  struct SubmapScanMatcher {
    const common::ProbabilityGrid *grid = nullptr;
    std::unique_ptr<scan_matcher::FastCorrelativeScanMatcher2D>
        fast_correlative_scan_matcher;
    std::weak_ptr<common::Task> creation_task_handle;
  };

  void runWhenDoneCallback();

  const SubmapScanMatcher *dispatchScanMatcherConstruction(
      const int &submap_id, const common::ProbabilityGrid *grid);
  void computeConstraint(
      const int &submap_id, const frontend::Submap2D *submap,
      const int &node_id, bool match_full_submap,
      const common::TrajectoryNode::Data *const constant_data,
      const common::Rigid2 &initial_relative_pose,
      const SubmapScanMatcher &submap_scan_matcher,
      std::unique_ptr<Constraint> *constraint, const double &distance);

  common::Mutex mutex_;
  common::ThreadPool *thread_pool_;
  const ConstraintBuilderOptions options_;
  std::unique_ptr<std::function<void(const Result &)>> when_done_;

  int num_started_nodes_ = 0;
  int num_finished_nodes_ = 0;

  Histogram score_histogram_;
  std::unique_ptr<common::Task> when_done_task_;
  std::unique_ptr<common::Task> finish_node_task_;

  std::deque<std::unique_ptr<Constraint>> constraints_;

  scan_matcher::CeresScanMatcher2D ceres_scan_matcher_;

  std::map<int, SubmapScanMatcher> submap_scan_matchers_;  // int -> submap id.

  std::map<int, common::FixedRatioSampler>
      per_submap_sampler_;  // int -> submap id.
};

}  // namespace backend
}  // namespace slam2d_core

#endif
