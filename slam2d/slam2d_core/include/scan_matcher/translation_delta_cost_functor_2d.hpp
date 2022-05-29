#ifndef _TRANSLATION_DELTA_COST_FUNCTOR_2D_H_
#define _TRANSLATION_DELTA_COST_FUNCTOR_2D_H_

#include "Eigen/Core"
#include "ceres/ceres.h"

namespace slam2d_core {
namespace scan_matcher {

class TranslationDeltaCostFunctor2D {
 public:
  static ceres::CostFunction *createAutoDiffCostFunction(
      const double scaling_factor, const Eigen::Vector2d &target_translation) {
    return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                           2 /* residuals */,
                                           3 /* pose variables */>(
        new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
  }

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  explicit TranslationDeltaCostFunctor2D(
      const double scaling_factor, const Eigen::Vector2d &target_translation)
      : scaling_factor_(scaling_factor),
        x_(target_translation.x()),
        y_(target_translation.y()) {}

  TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D &) = delete;
  TranslationDeltaCostFunctor2D &operator=(
      const TranslationDeltaCostFunctor2D &) = delete;

  const double scaling_factor_;
  const double x_;
  const double y_;
};

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif
