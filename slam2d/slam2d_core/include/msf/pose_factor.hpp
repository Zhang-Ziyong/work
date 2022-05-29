#ifndef _POSE_FACTOR_H_
#define _POSE_FACTOR_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "Eigen/Core"

#include <ceres/ceres.h>

#include "common/math.hpp"

namespace slam2d_core {
namespace msf {

inline Eigen::Matrix<double, 2, 2> RotationMatrix2D(const double &yaw_radians) {
  const double cos_yaw = ceres::cos(yaw_radians);
  const double sin_yaw = ceres::sin(yaw_radians);

  Eigen::Matrix<double, 2, 2> rotation;
  rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
  return rotation;
}

class RelativePoseFactor
    : public ceres::SizedCostFunction<3, 1, 1, 1, 1, 1, 1> {
 public:
  /**
   * RelativePoseFactor
   * @brief 二元边的雅克比求导过程
   * @param[in] x_ab-b点到a点的x坐标差
   * @param[in] y_ab-b点到a点的y坐标差
   * @param[in] yaw_ab_radians-b点到a点的角度差
   * @param[in] sqrt_information-信息矩阵
   **/
  RelativePoseFactor(const double &x_ab, const double &y_ab,
                     const double &yaw_ab_radians,
                     const Eigen::Matrix3d &sqrt_information)
      : p_ab_(x_ab, y_ab),
        yaw_ab_radians_(yaw_ab_radians),
        sqrt_information_(sqrt_information) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Matrix<double, 2, 1> p_a(parameters[0][0], parameters[1][0]);
    Eigen::Matrix<double, 2, 1> p_b(parameters[3][0], parameters[4][0]);
    double yaw_a = parameters[2][0];
    double yaw_b = parameters[5][0];

    Eigen::Map<Eigen::Matrix<double, 3, 1>> residuals_map(residuals);

    residuals_map.head<2>() =
        RotationMatrix2D(yaw_a).transpose() * (p_b - p_a) - p_ab_;
    residuals_map(2) =
        common::normalize_angle((yaw_b - yaw_a) - yaw_ab_radians_);

    residuals_map = sqrt_information_ * residuals_map;
    Eigen::Matrix2d Const_Rot;  // Const_Rot = -Rot(-PI/2)
    Const_Rot << 0, -1, 1, 0;
    Eigen::Matrix3d H1 = -Eigen::Matrix3d::Identity();
    Eigen::Matrix3d H2 = Eigen::Matrix3d::Identity();
    H1.block<2, 2>(0, 0) = -RotationMatrix2D(-yaw_a);
    H1.block<2, 1>(0, 2) =
        Const_Rot * RotationMatrix2D(yaw_a).transpose() * (p_a - p_b);
    H2.block<2, 2>(0, 0) = RotationMatrix2D(-yaw_a);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_pose_ax(jacobians[0]);
        jacobian_pose_ax.setZero();
        jacobian_pose_ax = H1.block<3, 1>(0, 0);
        jacobian_pose_ax = sqrt_information_ * jacobian_pose_ax;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_pose_ay(jacobians[1]);
        jacobian_pose_ay.setZero();
        jacobian_pose_ay = H1.block<3, 1>(0, 1);
        jacobian_pose_ay = sqrt_information_ * jacobian_pose_ay;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_yaw_a(jacobians[2]);
        jacobian_yaw_a.setZero();
        jacobian_yaw_a = H1.block<3, 1>(0, 2);
        jacobian_yaw_a = sqrt_information_ * jacobian_yaw_a;
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_p_bx(jacobians[3]);
        jacobian_p_bx.setZero();
        jacobian_p_bx = H2.block<3, 1>(0, 0);
        jacobian_p_bx = sqrt_information_ * jacobian_p_bx;
      }
      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_p_by(jacobians[4]);
        jacobian_p_by.setZero();
        jacobian_p_by = H2.block<3, 1>(0, 1);
        jacobian_p_by = sqrt_information_ * jacobian_p_by;
      }
      if (jacobians[5]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_yaw_b(jacobians[5]);
        jacobian_yaw_b.setZero();
        jacobian_yaw_b = H2.block<3, 1>(0, 2);
        jacobian_yaw_b = sqrt_information_ * jacobian_yaw_b;
      }
    }

    return true;
  }

 private:
  // The position of B relative to A in the A frame.
  const Eigen::Vector2d p_ab_;  ///< T_ba
  // The orientation of frame B relative to frame A.
  const double yaw_ab_radians_;  ///< R_ba
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;  ///< 信息矩阵
};

class GlobalPoseFactor : public ceres::SizedCostFunction<3, 1, 1, 1> {
 public:
  /**
   * GlobalPoseFactor
   * @brief 一元边的雅克比求导过程
   * @param[in] m_x-一元边的x坐标
   * @param[in] m_y-一元边的y坐标
   * @param[in] m_yaw_radians-一元边的角度
   * @param[in] sqrt_information-信息矩阵
   **/
  GlobalPoseFactor(const double &m_x, const double &m_y,
                   const double &m_yaw_radians,
                   const Eigen::Matrix3d &sqrt_information)
      : m_p_(m_x, m_y),
        m_yaw_radians_(m_yaw_radians),
        sqrt_information_(sqrt_information) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Matrix<double, 2, 1> p_a(parameters[0][0], parameters[1][0]);
    double yaw_a = parameters[2][0];

    Eigen::Map<Eigen::Matrix<double, 3, 1>> residuals_map(residuals);

    residuals_map.head<2>() = p_a - m_p_;
    residuals_map(2) = common::normalize_angle(yaw_a - m_yaw_radians_);

    residuals_map = sqrt_information_ * residuals_map;
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_pose_ax(jacobians[0]);
        jacobian_pose_ax.setZero();
        jacobian_pose_ax = H.block<3, 1>(0, 0);
        jacobian_pose_ax = sqrt_information_ * jacobian_pose_ax;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_pose_ay(jacobians[1]);
        jacobian_pose_ay.setZero();
        jacobian_pose_ay = H.block<3, 1>(0, 1);
        jacobian_pose_ay = sqrt_information_ * jacobian_pose_ay;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 1>> jacobian_yaw_a(jacobians[2]);
        jacobian_yaw_a.setZero();
        jacobian_yaw_a = H.block<3, 1>(0, 2);
        jacobian_yaw_a = sqrt_information_ * jacobian_yaw_a;
      }
    }

    return true;
  }

 private:
  // The position of  A in the A frame.
  const Eigen::Vector2d m_p_;  ///< 一元边的位置状态
  // The orientation of  frame A.
  const double m_yaw_radians_;  ///< 一元边的角度状态
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix3d sqrt_information_;  ///< 信息矩阵
};

template <typename T>
inline T NormalizeAngle(const T &angle_radians) {
  // Use ceres::floor because it is specialized for double and Jet types.
  T two_pi(2.0 * M_PI);
  return angle_radians -
         two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

class AngleLocalParameterization {
 public:
  /**
   * operator()
   * @brief 角度加法
   * @param[in] theta_radians-角度1
   * @param[in] delta_theta_radians-角度2
   * @param[in] theta_radians_plus_delta-角度1和角度2的加法结果
   * @return true
   **/
  template <typename T>
  bool operator()(const T *theta_radians, const T *delta_theta_radians,
                  T *theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }
  /**
   * Create()
   * @brief 创建一个对象
   * @return 创建的对象
   **/
  static ceres::LocalParameterization *Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

}  // namespace msf
}  // namespace slam2d_core

#endif