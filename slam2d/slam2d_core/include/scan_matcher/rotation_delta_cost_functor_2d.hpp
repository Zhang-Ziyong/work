#ifndef _ROTATION_DELTA_COST_FUNCTOR_2D_H_
#define _ROTATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace slam2d_core {
namespace scan_matcher {

class RotationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction *createAutoDiffCostFunction(
      const double scaling_factor, const double target_angle) {
    return new ceres::AutoDiffCostFunction<
        RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
        new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
  }

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                      const double target_angle)
      : scaling_factor_(scaling_factor), angle_(target_angle) {}

  RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D &) = delete;
  RotationDeltaCostFunctor2D &operator=(const RotationDeltaCostFunctor2D &) =
      delete;

  const double scaling_factor_;
  const double angle_;
};

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif
