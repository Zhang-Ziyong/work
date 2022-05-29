#include "scan_matcher/ceres_scan_matcher_2d.hpp"

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <utility>
#include <vector>

namespace slam2d_core {
namespace scan_matcher {

CeresScanMatcher2D::CeresScanMatcher2D(const CeresScanMatcherOptions2D &options)
    : options_(options),
      ceres_solver_options_(
          common::createCeresSolverOptions(options.ceres_solver_options)) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::match(const Eigen::Vector2d &target_translation,
                               const common::Rigid2 &initial_pose_estimate,
                               const common::PointCloud &point_cloud,
                               const common::ProbabilityGrid &grid,
                               common::Rigid2 *const pose_estimate,
                               ceres::Solver::Summary *const summary) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_weight, 0.);

  // 点云匹配残差块
  problem.AddResidualBlock(
      createOccupiedSpaceCostFunction2D(
          options_.occupied_space_weight /
              std::sqrt(static_cast<double>(point_cloud.size())),
          point_cloud, grid),
      nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(options_.translation_weight, 0.);

  // 平移残差
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::createAutoDiffCostFunction(
          options_.translation_weight, target_translation),
      nullptr /* loss function */, ceres_pose_estimate);

  // 旋转残差
  CHECK_GT(options_.rotation_weight, 0.);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::createAutoDiffCostFunction(
          options_.rotation_weight, ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = common::Rigid2(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matcher
}  // namespace slam2d_core
