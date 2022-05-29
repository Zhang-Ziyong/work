/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file pose_graph_error_term.hpp
 *
 *@brief
 * 1.误差优化类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/
#ifndef POSE_GRAPH_ERROR_TERM_HPP
#define POSE_GRAPH_ERROR_TERM_HPP

#include <ceres/ceres.h>
#include "common/math_base/slam_math.hpp"

namespace cvte_lidar_slam {
/**
 * PosePara
 * @brief pose本地参数化类
 **/
class PosePara : public ceres::LocalParameterization {
 public:
  PosePara() noexcept = default;
  virtual ~PosePara() = default;

  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const override {
    Eigen::Map<const Mat3d, 0, Eigen::OuterStride<4>> R(x, 3, 3);
    Eigen::Map<const Vec3d, 0, Eigen::InnerStride<4>> p(x + 3, 3);
    Eigen::Map<const Vec3d> dr(delta, 3);
    Eigen::Map<const Vec3d> dp(delta + 3, 3);
    Eigen::Map<Mat34d> T_se3(x_plus_delta);
    if (isTraditional) {
      Mat3d dR = Mathbox::Exp3d(dr);
      T_se3.block<3, 3>(0, 0) = dR * R;
      T_se3.block<3, 1>(0, 3) = p + dp;
    } else {
      // Mat3d dR, Jl;
      // Exp_SO3(dr, dR, Jl  );
      // T_se3.block<3,3>(0,0)  =  dR * R;
      // T_se3.block<3,1>(0,3) =   dR * p + Jl*dp;
    }
    return true;
  }
  virtual bool ComputeJacobian(const double *x,
                               double *jacobian) const override {
    Eigen::Map<Mat126d> Jacobian(jacobian);
    Jacobian.setZero();
    Jacobian.block<6, 6>(0, 0) = Mat6d::Identity();
    if (this->isTraditional) {
      Jacobian.block<6, 6>(0, 0) = Mat6d::Identity();
    } else {
      Jacobian.block<6, 6>(0, 0) = Mat6d::Identity();
      Eigen::Map<const Vec3d, 0, Eigen::InnerStride<4>> p(x + 3, 3);
      Jacobian.block<3, 3>(3, 0) = -Mathbox::skew(p);
    }
    return true;
  }
  virtual int GlobalSize() const override { return 12; }
  virtual int LocalSize() const override { return 6; }

  bool isTraditional = true;
};

/**
 * RelativeFactor
 * @brief 相对2d pose的factor类
 **/
class OdomRelativeFactor
    : public ceres::SizedCostFunction<6,   // number of residuals
                                      12,  // pose1
                                      12>  // pose2
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OdomRelativeFactor() = delete;
  OdomRelativeFactor(const Mat34d dT, const Mat6d SqrtInf) noexcept
      : mdT(dT), mSqrtInf(SqrtInf) {}
  virtual ~OdomRelativeFactor() = default;
  virtual size_t parameterBlocks() const { return 2; }
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const override {
    Eigen::Map<const Mat34d> pose1(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> pose2(parameters[1], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &dR = mdT.block<3, 3>(0, 0);
    const Mat3d &R1 = pose1.block<3, 3>(0, 0);
    const Mat3d &R2 = pose2.block<3, 3>(0, 0);
    const Vec3d &dt = mdT.block<3, 1>(0, 3);
    const Vec3d &t1 = pose1.block<3, 1>(0, 3);
    const Vec3d &t2 = pose2.block<3, 1>(0, 3);

    Vec3d rotion_err = Mathbox::Log(R2.transpose() * R1 * dR);
    Vec3d translation_err = R2.transpose() * (R1 * dt - (t2 - t1));
    Vec6d all_err;
    all_err << rotion_err, translation_err;

    resVec = weighting * mSqrtInf * all_err;

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) =
            Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J0.block<3, 3>(3, 0) = -R2.transpose() * Mathbox::skew(R1 * dt);
        J0.block<3, 3>(3, 3) = R2.transpose();
        J0 = weighting * mSqrtInf * J0;
        // std::cout << "J0：  " << J0 << std::endl;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J1(
            jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) =
            -Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J1.block<3, 3>(3, 0) =
            R2.transpose() * Mathbox::skew(R1 * dt - (t2 - t1));
        J1.block<3, 3>(3, 3) = -R2.transpose();
        J1 = weighting * mSqrtInf * J1;
        // std::cout << "J1：  " << J1 << std::endl;
      }
    }

    return true;
  }

  Mat34d mdT;
  Mat6d mSqrtInf;
  double weighting = 1.0;
};

class LoopRelativeFactor
    : public ceres::SizedCostFunction<6,   // number of residuals
                                      12,  // pose1
                                      12>  // pose2
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LoopRelativeFactor() = delete;
  LoopRelativeFactor(const Mat34d dT, const Mat6d SqrtInf) noexcept
      : mdT(dT), mSqrtInf(SqrtInf) {}
  virtual ~LoopRelativeFactor() = default;
  virtual size_t parameterBlocks() const { return 2; }
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const override {
    Eigen::Map<const Mat34d> pose1(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> pose2(parameters[1], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &dR = mdT.block<3, 3>(0, 0);
    const Mat3d &R1 = pose1.block<3, 3>(0, 0);
    const Mat3d &R2 = pose2.block<3, 3>(0, 0);
    const Vec3d &dt = mdT.block<3, 1>(0, 3);
    const Vec3d &t1 = pose1.block<3, 1>(0, 3);
    const Vec3d &t2 = pose2.block<3, 1>(0, 3);

    Vec3d rotion_err = Mathbox::Log(R2.transpose() * R1 * dR);
    Vec3d translation_err = R2.transpose() * (R1 * dt - (t2 - t1));
    Vec6d all_err;
    all_err << rotion_err, translation_err;

    Mat6d s_mat;
    // s_mat.setZero();
    // for(int i = 0;i < 3;i++){
    //   double s = 1 / (0.5 + all_err(i)*all_err(i));
    //   s_mat(i,i) = s < 1.0 ? s : 1;
    // }
    for (int i = 0; i < 6; i++) {
      double s = 3 / (3 + std::fabs(all_err(i)));
      s_mat(i, i) = s < 1.0 ? s : 1;
    }

    resVec = weighting * mSqrtInf * all_err;

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) =
            Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J0.block<3, 3>(3, 0) = -R2.transpose() * Mathbox::skew(R1 * dt);
        J0.block<3, 3>(3, 3) = R2.transpose();
        J0 = weighting * mSqrtInf * J0;
        // std::cout << "J0：  " << J0 << std::endl;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J1(
            jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) =
            -Mathbox::Jr_inv(rotion_err) * (R1 * dR).transpose();
        J1.block<3, 3>(3, 0) =
            R2.transpose() * Mathbox::skew(R1 * dt - (t2 - t1));
        J1.block<3, 3>(3, 3) = -R2.transpose();
        J1 = weighting * mSqrtInf * J1;
        // std::cout << "J1：  " << J1 << std::endl;
      }
    }

    return true;
  }

  Mat34d mdT;
  Mat6d mSqrtInf;
  double weighting = 1.0;
};

/**
 * RelativeWithExFactor
 * @brief 相对pose的factor类(增加外参估计)
 **/
class RelativeWithExFactor : public ceres::SizedCostFunction<6, 12, 12, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RelativeWithExFactor(const Mat34d &T1, const Mat6d &information)
      : T_odom(T1), information(information) {}
  virtual ~RelativeWithExFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ex(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> T_ci(parameters[1], 3, 4);
    Eigen::Map<const Mat34d> T_cj(parameters[2], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &R_ci = T_ci.block<3, 3>(0, 0);
    const Mat3d &R_cj = T_cj.block<3, 3>(0, 0);
    const Mat3d &dR_o = T_odom.block<3, 3>(0, 0);
    const Mat3d &R_co = T_ex.block<3, 3>(0, 0);
    const Vec3d &t_ci = T_ci.block<3, 1>(0, 3);
    const Vec3d &t_cj = T_cj.block<3, 1>(0, 3);
    const Vec3d &dt_o = T_odom.block<3, 1>(0, 3);
    const Vec3d &t_co = T_ex.block<3, 1>(0, 3);
    const Mat3d &dR_x = R_co * dR_o * R_co.transpose();

    Vec3d rotion_err = Mathbox::Log(R_cj.transpose() * R_ci * dR_x);
    Vec3d translation_err =
        R_cj.transpose() *
        (R_ci * (-dR_x * t_co + R_co * dt_o + t_co) - (t_cj - t_ci));
    Vec6d all_err;
    all_err << rotion_err, translation_err;
    all_err = information * all_err;
    resVec = all_err;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = Mathbox::Jr_inv(rotion_err) *
                               (dR_x.transpose() - Eigen::Matrix3d::Identity());
        J0.block<3, 3>(3, 0) =
            R_cj.transpose() * R_ci *
            (Mathbox::skew(dR_x * t_co) - dR_x * Mathbox::skew(t_co) -
             Mathbox::skew(R_co * dt_o));
        J0.block<3, 3>(3, 3) =
            R_cj.transpose() * R_ci * (-dR_x + Eigen::Matrix3d::Identity());
        J0 = information * J0;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J1(
            jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) =
            Mathbox::Jr_inv(rotion_err) * (R_ci * dR_x).transpose();
        J1.block<3, 3>(3, 0) =
            -R_cj.transpose() *
            Mathbox::skew(R_ci * (-dR_x * t_co + R_co * dt_o + t_co));
        J1.block<3, 3>(3, 3) = R_cj.transpose();
        J1 = information * J1;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J2(
            jacobians[2]);
        J2.setZero();
        J2.block<3, 3>(0, 0) =
            -Mathbox::Jr_inv(rotion_err) * (R_ci * dR_x).transpose();
        J2.block<3, 3>(3, 0) =
            R_cj.transpose() *
            Mathbox::skew(R_ci * (-dR_x * t_co + R_co * dt_o + t_co) -
                          (t_cj - t_ci));
        J2.block<3, 3>(3, 3) = -R_cj.transpose();
        J2 = information * J2;
      }
    }

    return true;
  }

 private:
  Mat34d T_odom;
  Mat6d information;

};  // class

/**
 * MapFactor
 * @brief 地图pose的factor类
 **/
class MapFactor : public ceres::SizedCostFunction<6, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapFactor(const Mat34d &T1, const Mat6d &information)
      : T_map(T1), information(information) {}
  virtual ~MapFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ci(parameters[0], 3, 4);
    Eigen::Map<Vec6d> resVec(residuals);

    const Mat3d &R_ci = T_ci.block<3, 3>(0, 0);
    const Mat3d &R_mi = T_map.block<3, 3>(0, 0);
    const Vec3d &t_ci = T_ci.block<3, 1>(0, 3);
    const Vec3d &t_mi = T_map.block<3, 1>(0, 3);

    Vec3d rotion_err = Mathbox::Log(R_ci.transpose() * R_mi);
    Vec3d translation_err = R_ci.transpose() * (t_mi - t_ci);
    Vec6d all_err;
    all_err << rotion_err, translation_err;
    all_err = information * all_err;
    resVec = all_err;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = -Mathbox::Jr_inv(rotion_err) * R_mi.transpose();
        J0.block<3, 3>(3, 0) = R_ci.transpose() * Mathbox::skew(t_mi - t_ci);
        J0.block<3, 3>(3, 3) = -R_ci.transpose();
        J0 = information * J0;
      }
    }

    return true;
  }

 private:
  Mat34d T_map;
  Mat6d information;

};  // class

/**
 * GPSFactor
 * @brief GPS坐标的factor类,包含外参优化
 **/
class GPSFactor : public ceres::SizedCostFunction<3, 12, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSFactor(const Vec3d &position, const Mat3d &information)
      : position(position), information(information) {}
  virtual ~GPSFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ci(parameters[0], 3, 4);
    Eigen::Map<const Mat34d> T_ex(parameters[1], 3, 4);
    const Mat3d &R_ex = T_ex.block<3, 3>(0, 0);
    const Vec3d &t_ex = T_ex.block<3, 1>(0, 3);

    Eigen::Map<Vec3d> resVec(residuals);

    resVec = T_ci.block<3, 1>(0, 3) - R_ex * position - t_ex;
    resVec = information * resVec;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 3) = information * Mat3d::Identity();
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J1(
            jacobians[1]);
        J1.setZero();
        J1.block<3, 3>(0, 0) = information * Mathbox::skew(R_ex * position);
        J1.block<3, 3>(0, 3) = -information * Mat3d::Identity();
      }
    }

    return true;
  }

 private:
  Vec3d position;
  Mat3d information;

};  // class

/**
 * GPSExtrincsFactor
 * @brief GPS外参的factor类
 **/
class GPSExtrincsFactor : public ceres::SizedCostFunction<3, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSExtrincsFactor(const Vec3d &position, const Vec3d &lidar_position,
                    const Mat3d &information)
      : position(position),
        lidar_position(lidar_position),
        information(information) {}
  virtual ~GPSExtrincsFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> T_ex(parameters[0], 3, 4);
    const Mat3d &R_ex = T_ex.block<3, 3>(0, 0);
    const Vec3d &t_ex = T_ex.block<3, 1>(0, 3);

    Eigen::Map<Vec3d> resVec(residuals);

    resVec = lidar_position - R_ex * position - t_ex;
    resVec = information * resVec;

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 0) = Mathbox::skew(R_ex * position);
        J0.block<3, 3>(0, 3) = -Mat3d::Identity();
        J0 = information * J0;
      }
    }

    return true;
  }

 private:
  Vec3d position;
  Vec3d lidar_position;
  Mat3d information;

};  // class

/**
 * GPSFactor
 * @brief GPS坐标的factor类
 **/
class GPS2dFactor : public ceres::SizedCostFunction<3, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPS2dFactor(const Vec3d &position, const Mat3d &information)
      : position(position), information(information) {}
  virtual ~GPS2dFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> P_ci(parameters[0], 3, 4);
    Eigen::Map<Vec3d> resVec(residuals);

    resVec = P_ci.block<3, 1>(0, 3) - position;
    resVec = information * resVec;
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<3, 3>(0, 3) = information * Mat3d::Identity();
      }
    }

    return true;
  }

 private:
  Vec3d position;
  Mat3d information;

};  // end of class

/**
 * PlaneFactor
 * @brief 激光面点的factor类
 **/
class PlaneFactor : public ceres::SizedCostFunction<1, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PlaneFactor(const Vec3d &origin_point, const Vec3d &plane_norm,
              const double scale, double const const_value)
      : origin_point(origin_point),
        plane_norm(plane_norm),
        scale(scale),
        const_value(const_value) {}
  virtual ~PlaneFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> pose(parameters[0], 3, 4);
    Vec3d map_point = Mathbox::multiplePoint(pose, origin_point);

    residuals[0] = scale * (plane_norm.transpose() * map_point + const_value);

    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<1, 3>(0, 0) =
            -scale * plane_norm.transpose() *
            Mathbox::skew(map_point - pose.block<3, 1>(0, 3));
        J0.block<1, 3>(0, 3) = scale * plane_norm.transpose();
      }
    }

    return true;
  }

 private:
  Vec3d origin_point;
  Vec3d plane_norm;
  double scale;
  double const_value;
};  // end of class

/**
 * EdgeFactor
 * @brief 激光线点的factor类
 **/
class EdgeFactor : public ceres::SizedCostFunction<1, 12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeFactor(const Vec3d &origin_point, const Vec3d &point_a,
             const Vec3d &point_b, const double scale)
      : origin_point(origin_point),
        point_a(point_a),
        point_b(point_b),
        scale(scale) {}
  virtual ~EdgeFactor() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const {
    Eigen::Map<const Mat34d> pose(parameters[0], 3, 4);
    Vec3d map_point = Mathbox::multiplePoint(pose, origin_point);

    residuals[0] =
        scale * ((map_point - point_a).cross(map_point - point_b)).norm();

    double k =
        (point_b - point_a).transpose() *
        (map_point -
         point_a);  ///((point_b - point_a).transpose()*(point_b - point_a));
    k = k / ((point_b - point_a).transpose() * (point_b - point_a));
    Vec3d point_vetical = point_a + k * (point_b - point_a);
    Vec3d vetical_vec = map_point - point_vetical;
    vetical_vec.normalize();
    if (jacobians != nullptr) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> J0(
            jacobians[0]);
        J0.setZero();
        J0.block<1, 3>(0, 0) =
            -scale * (vetical_vec.transpose()) *
            Mathbox::skew(map_point - pose.block<3, 1>(0, 3));
        J0.block<1, 3>(0, 3) = scale * (vetical_vec.transpose());
      }
    }

    return true;
  }

 private:
  Vec3d origin_point;
  Vec3d point_a;
  Vec3d point_b;
  double scale;
};  // end of class

}  // namespace cvte_lidar_slam
#endif