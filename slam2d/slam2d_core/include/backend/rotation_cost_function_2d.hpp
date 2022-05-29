/*******************************************************************************
 * @file       : rotation_cost_function_2d.h                                   *
 * @author     : PG.Wang (leatherwang@foxmail.com)                             *
 * @version    : v1.0.0                                                        *
 * @date       : 2021/02/10                                                    *
 * @brief      :                                                               *
 *******************************************************************************
 **-History---------------------------------------------------------------------
 * Version   Date        Name         Changes and comments
 * v1.0.0    2021/02/10  PG.Wang      initial version
 ******************************************************************************/

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "common/math.hpp"
#include "common/rigid_transform.hpp"
#include "gyro_pre_integration.hpp"

namespace slam2d_core {
namespace backend {
/// \brief rotation factor(analytical derivation).
///        vertexes: 3DoF pose(ony 1DoF rotation used) + 1DoF bias + Td
class AnalyticalP3DAndB1DAndTdCostFunction2D
    : public ceres::SizedCostFunction<
          1 /* number of residuals */, 3 /* size of start pose */,
          3 /* size of end pose */, 1 /* size of bias of gyro z */,
          1 /* size of td */> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit AnalyticalP3DAndB1DAndTdCostFunction2D(
      const GyroYawPreintegrator *pre_integration,
      const Eigen::Quaterniond &gravity_alignment_i,
      const Eigen::Quaterniond &gravity_alignment_j, const double vel_yaw_i,
      const double vel_yaw_j, const double scaling_factor)
      : pre_integration_(pre_integration),
        gravity_alignment_i_(gravity_alignment_i),
        gravity_alignment_j_(gravity_alignment_j),
        vel_yaw_i_(vel_yaw_i),
        vel_yaw_j_(vel_yaw_j),
        scaling_factor_(scaling_factor) {}

  virtual ~AnalyticalP3DAndB1DAndTdCostFunction2D() {}

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    const double yi = parameters[0][2];
    const double yj = parameters[1][2];
    const double bgi_z = parameters[2][0];
    const double td = parameters[3][0];

    double delta_yaw_corrected = pre_integration_->getCorrectedDeltaYaw(bgi_z);
    double delta_td = pre_integration_->getCorrectedDeltaTd(td);

    // yaw_w_gj + yaw_gj_bj - vel_j*td = yaw_w_gi + yaw_gi_bi - vel_i*td +
    // yaw_bi_bj
    residuals[0] = common::getYaw(gravity_alignment_i_) + delta_yaw_corrected -
                   common::getYaw(gravity_alignment_j_) + yi - yj +
                   delta_td * (vel_yaw_j_ - vel_yaw_i_);
    residuals[0] =
        scaling_factor_ * common::normalizeAngleDifference(residuals[0]);

    if (jacobians == NULL)
      return true;

    if (jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_pose_i(
          jacobians[0]);
      jacobian_pose_i.setZero();
      jacobian_pose_i(2) = scaling_factor_;
    }
    if (jacobians[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_pose_j(
          jacobians[1]);
      jacobian_pose_j.setZero();
      jacobian_pose_j(2) = -scaling_factor_;
    }
    if (jacobians[2] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_bg_z(
          jacobians[2]);
      jacobian_bg_z(0) = -scaling_factor_ * pre_integration_->getDeltaTime();
    }
    if (jacobians[3] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> jacobian_td(
          jacobians[3]);
      jacobian_td(0) = scaling_factor_ * (vel_yaw_j_ - vel_yaw_i_);
    }

    return true;
  }

 private:
  const GyroYawPreintegrator *pre_integration_;
  const Eigen::Quaterniond gravity_alignment_i_, gravity_alignment_j_;
  const double vel_yaw_i_, vel_yaw_j_;
  const double scaling_factor_;
};

/// \brief 1DoF bias vary model factor(automatic derivation).
///
class BiasZCostFunction2D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction *CreateAutoDiffCostFunction(
      const double scaling_factor) {
    return new ceres::AutoDiffCostFunction<
        BiasZCostFunction2D, 1 /* residuals */, 1 /* size of gyro bias z i*/,
        1 /* size of gyro bias z j*/
        >(new BiasZCostFunction2D(scaling_factor));
  }

  template <typename T>
  bool operator()(const T *const bg_z_i, const T *const bg_z_j,
                  T *residual) const {
    residual[0] = scaling_factor_ * (bg_z_j[0] - bg_z_i[0]);

    return true;
  }

 private:
  BiasZCostFunction2D(const double scaling_factor)
      : scaling_factor_(scaling_factor) {}

  BiasZCostFunction2D(const BiasZCostFunction2D &) = delete;
  BiasZCostFunction2D &operator=(const BiasZCostFunction2D &) = delete;

  const double scaling_factor_;
};

/// \brief 1DoF bias prior factor(automatic derivation).
///
class BiasZPriorCostFunction2D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static ceres::CostFunction *CreateAutoDiffCostFunction(
      const double bias_gyro_z_prior, const double scaling_factor) {
    return new ceres::AutoDiffCostFunction<BiasZPriorCostFunction2D,
                                           1 /* residuals */,
                                           1 /* size of gyro bias z i*/
                                           >(
        new BiasZPriorCostFunction2D(bias_gyro_z_prior, scaling_factor));
  }

  template <typename T>
  bool operator()(const T *const bg_z, T *residual) const {
    residual[0] = scaling_factor_ * (bg_z[0] - bias_gyro_z_prior_);

    return true;
  }

 private:
  BiasZPriorCostFunction2D(const double bias_gyro_z_prior,
                           const double scaling_factor)
      : bias_gyro_z_prior_(bias_gyro_z_prior),
        scaling_factor_(scaling_factor) {}

  BiasZPriorCostFunction2D(const BiasZPriorCostFunction2D &) = delete;
  BiasZPriorCostFunction2D &operator=(const BiasZPriorCostFunction2D &) =
      delete;

  const double bias_gyro_z_prior_;
  const double scaling_factor_;
};

}  // namespace backend
}  // namespace slam2d_core
