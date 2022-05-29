#ifndef _CERES_SCAN_MATCHER_2D_H_
#define _CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "common/ceres_solver_options.hpp"
// #include "common/grid_map.hpp"
#include "common/probability_grid.hpp"
#include "common/point_cloud.hpp"
#include "occupied_space_cost_function_2d.hpp"
#include "rotation_delta_cost_functor_2d.hpp"
#include "translation_delta_cost_functor_2d.hpp"

namespace slam2d_core {
namespace scan_matcher {

class CeresScanMatcherOptions2D {
 public:
  double occupied_space_weight = 20.0;
  double translation_weight = 10.0;
  double rotation_weight = 1.0;
  common::CeresSolverOptions ceres_solver_options;
};

class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const CeresScanMatcherOptions2D &options);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D &) = delete;
  CeresScanMatcher2D &operator=(const CeresScanMatcher2D &) = delete;

  void match(const Eigen::Vector2d &target_translation,
             const common::Rigid2 &initial_pose_estimate,
             const common::PointCloud &point_cloud,
             const common::ProbabilityGrid &grid, common::Rigid2 *pose_estimate,
             ceres::Solver::Summary *summary) const;

 private:
  const CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif  //
