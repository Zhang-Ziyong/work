#ifndef _LOCAL_TRAJECTORY_BUILDER_H_
#define _LOCAL_TRAJECTORY_BUILDER_H_

#include "common/odometry_data.hpp"
#include "common/point_cloud.hpp"
#include "common/rigid_transform.hpp"
#include "common/time.hpp"
#include "common/trajectory_node.hpp"
#include "frontend/voxel_filter.hpp"
#include "motion_filter.hpp"
#include "pose_extrapolator.hpp"
#include "scan_matcher/ceres_scan_matcher_2d.hpp"
#include "scan_matcher/real_time_correlative_scan_matcher_2d.hpp"
#include "submap_2d.hpp"

#include <chrono>
#include <memory>

namespace slam2d_core {
namespace frontend {

class LocalTrajectoryBuilderOptions {
 public:
  float min_range;
  float max_range;
  float voxel_filter_size = 0.025;

  float missing_data_ray_length;
  bool use_online_correlative_scan_matching = false;
  scan_matcher::RealTimeCorrelativeScanMatcherOptions
      real_time_correlative_scan_matcher_options;
  scan_matcher::CeresScanMatcherOptions2D ceres_scan_matcher_options;
  MotionFilterOptions motion_filter_options;
  SubmapsOptions submaps_options;
  AdaptiveVoxelFilterOptions adaptive_voxel_filter_options;
};

class LocalTrajectoryBuilder {
 public:
  struct InsertionResult {
    std::shared_ptr<const common::TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  };

  struct MatchingResult {
    common::Time time;
    common::Rigid3 local_pose;
    common::RangeData range_data_in_local;
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit LocalTrajectoryBuilder(const LocalTrajectoryBuilderOptions &options);
  ~LocalTrajectoryBuilder();

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder &) = delete;
  LocalTrajectoryBuilder &operator=(const LocalTrajectoryBuilder &) = delete;

  std::unique_ptr<MatchingResult> addRangeData(
      const common::TimedPointCloudData &time_point_cloud);

  void addOdometryData(const common::OdometryData &odometry_data);
  bool getLocalPose(const common::Time &time, common::Rigid3 &pose);

 private:
  std::unique_ptr<MatchingResult> addAccumulatedRangeData(
      common::Time time, const common::RangeData &range_data);

  std::unique_ptr<InsertionResult> insertIntoSubmap(
      common::Time time, const common::RangeData &range_data_in_local,
      const common::PointCloud &point_cloud,
      const common::Rigid3 &pose_estimate);

  std::unique_ptr<common::Rigid2> scanMatch(
      common::Time time, const common::Rigid2 &pose_prediction,
      const common::PointCloud &point_cloud);

  void initializeExtrapolator(common::Time time);

  const LocalTrajectoryBuilderOptions options_;
  ActiveSubmaps2D active_submaps_;

  MotionFilter motion_filter_;

  scan_matcher::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  scan_matcher::CeresScanMatcher2D ceres_scan_matcher_;

  std::unique_ptr<PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  common::RangeData accumulated_range_data_;
};

}  // namespace frontend
}  // namespace slam2d_core

#endif
