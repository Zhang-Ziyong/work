#include "backend/spa_cost_function_2d.hpp"
#include "common/math.hpp"
#include "common/rigid_transform.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/jet.h"
#include <array>

namespace slam2d_core {
namespace backend {
namespace {

class SpaCostFunction2D {
 public:
  explicit SpaCostFunction2D(const Constraint::Pose &observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T *const start_pose, const T *const end_pose,
                  T *e) const {
    const std::array<T, 3> error = ScaleError(
        ComputeUnscaledError(common::project2D(observed_relative_pose_.zbar_ij),
                             start_pose, end_pose),
        observed_relative_pose_.translation_weight,
        observed_relative_pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  const Constraint::Pose observed_relative_pose_;
};

class AnalyticalSpaCostFunction2D
    : public ceres::SizedCostFunction<3 /* number of residuals */,
                                      3 /* size of start pose */,
                                      3 /* size of end pose */> {
 public:
  explicit AnalyticalSpaCostFunction2D(const Constraint::Pose &constraint_pose)
      : observed_relative_pose_(common::project2D(constraint_pose.zbar_ij)),
        translation_weight_(constraint_pose.translation_weight),
        rotation_weight_(constraint_pose.rotation_weight) {}
  virtual ~AnalyticalSpaCostFunction2D() {}

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    double const *start = parameters[0];
    double const *end = parameters[1];

    const double cos_start_rotation = cos(start[2]);
    const double sin_start_rotation = sin(start[2]);
    const double delta_x = end[0] - start[0];
    const double delta_y = end[1] - start[1];

    residuals[0] =
        translation_weight_ *
        (observed_relative_pose_.translation().x() -
         (cos_start_rotation * delta_x + sin_start_rotation * delta_y));
    residuals[1] =
        translation_weight_ *
        (observed_relative_pose_.translation().y() -
         (-sin_start_rotation * delta_x + cos_start_rotation * delta_y));
    residuals[2] =
        rotation_weight_ *
        common::normalizeAngleDifference(
            observed_relative_pose_.rotation().angle() - (end[2] - start[2]));
    if (jacobians == NULL)
      return true;

    const double weighted_cos_start_rotation =
        translation_weight_ * cos_start_rotation;
    const double weighted_sin_start_rotation =
        translation_weight_ * sin_start_rotation;

    // Jacobians in Ceres are ordered by the parameter blocks:
    // jacobian[i] = [(dr_0 / dx_i)^T, ..., (dr_n / dx_i)^T].
    if (jacobians[0] != NULL) {
      jacobians[0][0] = weighted_cos_start_rotation;
      jacobians[0][1] = weighted_sin_start_rotation;
      jacobians[0][2] = weighted_sin_start_rotation * delta_x -
                        weighted_cos_start_rotation * delta_y;
      jacobians[0][3] = -weighted_sin_start_rotation;
      jacobians[0][4] = weighted_cos_start_rotation;
      jacobians[0][5] = weighted_cos_start_rotation * delta_x +
                        weighted_sin_start_rotation * delta_y;
      jacobians[0][6] = 0;
      jacobians[0][7] = 0;
      jacobians[0][8] = rotation_weight_;
    }
    if (jacobians[1] != NULL) {
      jacobians[1][0] = -weighted_cos_start_rotation;
      jacobians[1][1] = -weighted_sin_start_rotation;
      jacobians[1][2] = 0;
      jacobians[1][3] = weighted_sin_start_rotation;
      jacobians[1][4] = -weighted_cos_start_rotation;
      jacobians[1][5] = 0;
      jacobians[1][6] = 0;
      jacobians[1][7] = 0;
      jacobians[1][8] = -rotation_weight_;
    }
    return true;
  }

 private:
  const common::Rigid2 observed_relative_pose_;
  const double translation_weight_;
  const double rotation_weight_;
};

}  // namespace

ceres::CostFunction *createAutoDiffSpaCostFunction(
    const Constraint::Pose &observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3 /* residuals */,
                                         3 /* start pose variables */,
                                         3 /* end pose variables */>(
      new SpaCostFunction2D(observed_relative_pose));
}

ceres::CostFunction *createAnalyticalSpaCostFunction(
    const Constraint::Pose &observed_relative_pose) {
  return new AnalyticalSpaCostFunction2D(observed_relative_pose);
}

}  // namespace backend
}  // namespace slam2d_core
