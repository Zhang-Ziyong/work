#include "backend/constraint_builder.hpp"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Eigen/Eigenvalues"

#include "glog/logging.h"

namespace slam2d_core {
namespace backend {

common::Rigid2 computeSubmapPose(const frontend::Submap2D &submap) {
  return common::project2D(submap.local_pose());
}

ConstraintBuilder::ConstraintBuilder(const ConstraintBuilderOptions &options,
                                     common::ThreadPool *const thread_pool)
    : thread_pool_(thread_pool),
      options_(options),
      when_done_task_(std::make_unique<common::Task>()),
      finish_node_task_(std::make_unique<common::Task>()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options) {}

ConstraintBuilder::~ConstraintBuilder() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(finish_node_task_->getState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->getState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder::maybeAddConstraint(
    const int &submap_id, const frontend::Submap2D *const submap,
    const int &node_id, const common::TrajectoryNode::Data *const constant_data,
    const common::Rigid2 &initial_relative_pose, const double &distance) {
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance) {
    return;
  }

  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio))
           .first->second.Pulse()) {
    return;
  }

  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  //   kQueueLengthMetric->Set(constraints_.size());
  auto *const constraint = &constraints_.back();
  const auto *scan_matcher =
      dispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = std::make_unique<common::Task>();
  constraint_task->setWorkItem([=]() EXCLUDES(mutex_) {
    computeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                      constant_data, initial_relative_pose, *scan_matcher,
                      constraint, distance);
  });
  constraint_task->addDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->schedule(std::move(constraint_task));
  finish_node_task_->addDependency(constraint_task_handle);
}

void ConstraintBuilder::maybeAddGlobalConstraint(
    const int &submap_id, const frontend::Submap2D *const submap,
    const int &node_id,
    const common::TrajectoryNode::Data *const constant_data) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();

  //   kQueueLengthMetric->Set(constraints_.size());
  auto *const constraint = &constraints_.back();
  const auto *scan_matcher =
      dispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = std::make_unique<common::Task>();
  constraint_task->setWorkItem([=]() EXCLUDES(mutex_) {
    computeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                      constant_data, common::Rigid2::Identity(), *scan_matcher,
                      constraint, 100.0);
  });
  constraint_task->addDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->schedule(std::move(constraint_task));
  finish_node_task_->addDependency(constraint_task_handle);
}

void ConstraintBuilder::notifyEndOfNode() {
  common::MutexLocker locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->setWorkItem([this] {
    common::MutexLocker locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->schedule(std::move(finish_node_task_));
  finish_node_task_ = std::make_unique<common::Task>();
  when_done_task_->addDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder::whenDone(
    const std::function<void(const ConstraintBuilder::Result &)> &callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ = std::make_unique<std::function<void(const Result &)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->setWorkItem([this] { runWhenDoneCallback(); });
  thread_pool_->schedule(std::move(when_done_task_));
  when_done_task_ = std::make_unique<common::Task>();
}

const ConstraintBuilder::SubmapScanMatcher *
ConstraintBuilder::dispatchScanMatcherConstruction(
    const int &submap_id, const common::ProbabilityGrid *const grid) {
  CHECK(grid);
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto &submap_scan_matcher = submap_scan_matchers_[submap_id];
  submap_scan_matcher.grid = grid;
  auto &scan_matcher_options = options_.fast_correlative_scan_matcher_options;
  auto scan_matcher_task = std::make_unique<common::Task>();
  scan_matcher_task->setWorkItem(
      [&submap_scan_matcher, &scan_matcher_options]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            std::make_unique<scan_matcher::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder::computeConstraint(
    const int &submap_id, const frontend::Submap2D *const submap,
    const int &node_id, bool match_full_submap,
    const common::TrajectoryNode::Data *const constant_data,
    const common::Rigid2 &initial_relative_pose,
    const SubmapScanMatcher &submap_scan_matcher,
    std::unique_ptr<Constraint> *constraint, const double &distance) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const common::Rigid2 initial_pose =
      computeSubmapPose(*submap) * initial_relative_pose;

  float score = 0.;
  common::Rigid2 pose_estimate = common::Rigid2::Identity();

  if (match_full_submap) {
    // 该部分仍然是暴力匹配，不过加上了分支定界搜索，和前端的方法是有一定的相似性，传进来的都是重力对齐的点云数据，搜索框大小为：线：1e6*0.05，角度为pi
    // 分数是global_localization_min_score
    if (submap_scan_matcher.fast_correlative_scan_matcher->matchFullSubmap(
            constant_data->point_cloud, options_.global_localization_min_score,
            &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score);
    } else {
      return;
    }
  } else {
    if (submap_scan_matcher.fast_correlative_scan_matcher->match(
            initial_pose, constant_data->point_cloud, options_.min_score,
            &score, &pose_estimate)) {
      CHECK_GT(score, options_.min_score);
    } else {
      // LOG(INFO) << "fast_correlative_scan_matcher false"<< score << "  "
      //           << options_.min_score;
      return;
    }
  }

  const common::Rigid2 delta = initial_pose.inverse() * pose_estimate;
  LOG(INFO) << "fast correlative scan matcher differ by translation "
            << std::setprecision(3) << delta.translation().norm()
            << " rotation " << std::setprecision(3)
            << std::abs(delta.normalizedAngle());

  // 给定位使用避免长走廊跳动
  // if (delta.translation().norm() > 1.2) {
  //   float score_t = 0.;
  //   common::Rigid2 pose_estimate_t = common::Rigid2::Identity();
  //   if (submap_scan_matcher.fast_correlative_scan_matcher->match(
  //           initial_pose, constant_data->point_cloud, 0.2,
  //           options_.min_score, &score_t, &pose_estimate_t)) {
  //     CHECK_GT(score_t, options_.min_score);
  //   }
  //   LOG(INFO) << "score_t: " << score_t << "  score: " << score;
  //   if (fabs(score_t - score) < 0.05) {
  //     pose_estimate = initial_pose;
  //   }
  // }

  double ratio = fabs(delta.translation().norm() / distance);
  LOG(INFO) << "distance: " << distance << ", rate: " << ratio;

  if (ratio > options_.loop_closure_error_ratio && distance > 0.01) {
    LOG(WARNING) << "compute Constraint error.";
    return;
  }

  {
    common::MutexLocker locker(&mutex_);
    score_histogram_.add(score);
    // LOG(INFO) << "score_histogram_ add. -----------------";
  }

  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.match(
      pose_estimate.translation(), pose_estimate, constant_data->point_cloud,
      *submap_scan_matcher.grid, &pose_estimate, &unused_summary);

  const common::Rigid2 constraint_transform =
      computeSubmapPose(*submap).inverse() * pose_estimate;
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {common::embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight,
                                    options_.loop_closure_rotation_weight},
                                   Constraint::INTER_SUBMAP});

  if (options_.log_matches) {
    std::ostringstream info;
    info << "Node " << node_id << " with " << constant_data->point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const common::Rigid2 difference = initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalizedAngle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder::runWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result &)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint> &constraint : constraints_) {
      if (constraint == nullptr) {
        continue;
      }
      result.push_back(*constraint);
    }
    if (options_.log_matches) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.toString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    // kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}

int ConstraintBuilder::getNumFinishedNodes() {
  common::MutexLocker locker(&mutex_);
  return num_finished_nodes_;
}

void ConstraintBuilder::deleteScanMatcher(const int &submap_id) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
  //   kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
}

}  // namespace backend
}  // namespace slam2d_core
