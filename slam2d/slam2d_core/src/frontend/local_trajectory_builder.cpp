#include "frontend/local_trajectory_builder.hpp"

#include <limits>
#include <memory>

namespace slam2d_core {
namespace frontend {

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const LocalTrajectoryBuilderOptions &options)
    : options_(options),
      active_submaps_(options.submaps_options),
      motion_filter_(options.motion_filter_options),
      real_time_correlative_scan_matcher_(
          options.real_time_correlative_scan_matcher_options),
      ceres_scan_matcher_(options.ceres_scan_matcher_options) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

std::unique_ptr<common::Rigid2> LocalTrajectoryBuilder::scanMatch(
    const common::Time time, const common::Rigid2 &pose_prediction,
    const common::PointCloud &point_cloud) {
  (void) time;
  if (active_submaps_.submaps().empty()) {
    return std::make_unique<common::Rigid2>(pose_prediction);
  }
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();

  common::Rigid2 initial_ceres_pose = pose_prediction;

  //搜索框内暴力匹配
  if (options_.use_online_correlative_scan_matching) {
    const double score = real_time_correlative_scan_matcher_.match(
        pose_prediction, point_cloud, *matching_submap->grid(),
        &initial_ceres_pose);
    (void) score;
  }

  auto pose_observation = std::make_unique<common::Rigid2>();
  ceres::Solver::Summary summary;
  // 以pose_exptrapolator最为约束
  ceres_scan_matcher_.match(pose_prediction.translation(), initial_ceres_pose,
                            point_cloud, *matching_submap->grid(),
                            pose_observation.get(), &summary);
  return pose_observation;
}

bool LocalTrajectoryBuilder::getLocalPose(const common::Time &time,
                                          common::Rigid3 &pose) {
  if (extrapolator_ == nullptr) {
    LOG(INFO) << "Extrapolator not yet initialized.";
    return false;
  }

  return extrapolator_->extrapolatePose(time, pose);

  // LOG(INFO) << "local pose: " << pose.translation().x() << " ,"
  //           << pose.translation().y();
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>
LocalTrajectoryBuilder::addRangeData(
    const common::TimedPointCloudData &time_point_cloud) {
  const common::Time &time = time_point_cloud.time;
  initializeExtrapolator(time);
  if (extrapolator_ == nullptr) {
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  accumulated_range_data_ = common::RangeData{{0.0, 0.0, 0.0}, {}, {}};

  for (size_t i = 0; i < time_point_cloud.ranges.size(); ++i) {
    common::RangePoint hit_in_local = time_point_cloud.ranges[i];
    const Eigen::Vector3d delta =
        hit_in_local.position - time_point_cloud.origin;
    if (time_point_cloud.lengths[i] >= options_.min_range) {
      if (time_point_cloud.lengths[i] <= options_.max_range) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        hit_in_local.position =
            time_point_cloud.origin + options_.missing_data_ray_length /
                                          time_point_cloud.lengths[i] * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }

  accumulated_range_data_.origin = time_point_cloud.origin;

  return addAccumulatedRangeData(
      time, common::RangeData{accumulated_range_data_.origin,
                              voxelFilter(accumulated_range_data_.returns,
                                          options_.voxel_filter_size),
                              voxelFilter(accumulated_range_data_.misses,
                                          options_.voxel_filter_size)});
}

std::unique_ptr<LocalTrajectoryBuilder::MatchingResult>
LocalTrajectoryBuilder::addAccumulatedRangeData(
    const common::Time time, const common::RangeData &range_data) {
  common::Rigid3 pose;
  extrapolator_->extrapolatePose(time, pose);
  const common::Rigid2 pose_prediction = common::project2D(pose);

  const common::PointCloud &point_clound = adaptiveVoxelFilter(
      range_data.returns, options_.adaptive_voxel_filter_options);

  if (point_clound.empty()) {
    return nullptr;
  }

  std::unique_ptr<common::Rigid2> pose_estimate_2d =
      scanMatch(time, pose_prediction, point_clound);

  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  const common::Rigid3 pose_estimate = common::embed3D(*pose_estimate_2d);
  extrapolator_->addPose(time, pose_estimate);

  common::RangeData range_data_in_local =
      transformRangeData(range_data, common::embed3D(pose_estimate_2d->cast()));

  std::unique_ptr<InsertionResult> insertion_result =
      insertIntoSubmap(time, range_data_in_local, point_clound, pose_estimate);

  return std::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::insertIntoSubmap(
    const common::Time time, const common::RangeData &range_data_in_local,
    const common::PointCloud &point_cloud,
    const common::Rigid3 &pose_estimate) {
  if (motion_filter_.isSimilar(time, pose_estimate)) {
    return nullptr;
  }

  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.insertRangeData(range_data_in_local);

  return std::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const common::TrajectoryNode::Data>(
          common::TrajectoryNode::Data{time, point_cloud, pose_estimate, 0}),
      std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder::addOdometryData(
    const common::OdometryData &odometry_data) {
  if (extrapolator_ == nullptr) {
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->addOdometryData(odometry_data);
}

void LocalTrajectoryBuilder::initializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }

  extrapolator_ =
      std::make_unique<PoseExtrapolator>(common::fromSeconds(0.001));
  extrapolator_->addPose(time, common::Rigid3::Identity());
}

}  // namespace frontend
}  // namespace slam2d_core
