/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: /lidar_slam/slam_core/map_track/pose_solver.hpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-12-23 10:11:28
 ************************************************************************/
#pragma once

#include <vector>
#include <memory>
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace cvte_lidar_slam {

class LossFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~LossFunction() {}

  //    virtual double Compute(double error) const = 0;
  virtual void Compute(double err2, Eigen::Vector3d &rho) const = 0;
};

/**
 * Huber loss
 *
 * Huber(e) = e^2                      if e <= delta
 * huber(e) = delta*(2*e - delta)      if e > delta
 */
class HuberLoss : public LossFunction {
 public:
  explicit HuberLoss(double delta) : delta_(delta) {}

  virtual void Compute(double err2, Eigen::Vector3d &rho) const override {
    double dsqr = delta_ * delta_;
    if (err2 <= dsqr) {  // inlier
      rho[0] = err2;
      rho[1] = 1.;
      rho[2] = 0.;
    } else {                      // outlier
      double sqrte = sqrt(err2);  // absolut value of the error
      rho[0] = 2 * sqrte * delta_ -
               dsqr;            // rho(e)   = 2 * delta * e^(1/2) - delta^2
      rho[1] = delta_ / sqrte;  // rho'(e)  = delta / sqrt(e)
      rho[2] = -0.5 * rho[1] /
               err2;  // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
    }
  }

 private:
  double delta_;
};

class Node {};

class Factor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * 构造函数，会自动化配雅可比的空间
   * @param residual_dimension 残差维度
   */
  ~Factor() {}
  /// 计算残差,雅可比，由子类实现
  virtual void ComputeResidualAndJacobian() = 0;
  virtual void ComputeResidualOnly() = 0;

  void updatePose(const Mat34d &pose) { pose_ = pose; }

  /// 计算平方误差，会乘以信息矩阵
  double Chi2() const { return (residual_ * residual_); }
  double RobustChi2() const {
    double e2 = this->Chi2();
    if (lossfunction_) {
      Vec3d rho;
      lossfunction_->Compute(e2, rho);
      e2 = rho[0];
    }
    return e2;
  }

  /// 返回残差
  double Residual() const { return residual_; }

  /// 返回雅可比
  Eigen::Matrix<double, 1, 6, Eigen::RowMajor> Jacobian() const {
    return jacobian_;
  }

  void SetLossFunction(LossFunction *ptr) { lossfunction_ = ptr; }
  LossFunction *GetLossFunction() { return lossfunction_; }
  void RobustInfo(double &drho, double &info) const {
    if (nullptr != lossfunction_) {
      double e2 = this->Chi2();
      Eigen::Vector3d rho;
      lossfunction_->Compute(e2, rho);

      double robust_info = 0.0;
      robust_info *= rho[1];
      if (rho[1] + 2 * rho[2] * e2 > 0.) {
        robust_info = rho[1] + 2 * rho[2] * e2;
      }

      info = robust_info;
      drho = rho[1];
    } else {
      drho = 1.0;
      info = 1.;
    }
  }

 protected:
  Mat34d pose_;      // 对应pose
  double residual_;  // 残差
  double information_ = 1.;
  Eigen::Matrix<double, 1, 6, Eigen::RowMajor>
      jacobian_;  // 雅可比，每个雅可比维度是 residual x vertex_dim
  LossFunction *lossfunction_;
};

class PlanePointFactor : public Factor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PlanePointFactor(const Mat34d &pose, const Vec3d &origin_point,
                   const Vec3d &plane_norm, const double scale,
                   double const const_value)
      : origin_point_(origin_point),
        plane_norm_(plane_norm),
        scale_(scale),
        const_value_(const_value) {
    pose_ = pose;
  }
  ~PlanePointFactor() {}
  virtual void ComputeResidualAndJacobian();
  virtual void ComputeResidualOnly();  // 记得先更新pose

 protected:
  Vec3d origin_point_;
  Vec3d plane_norm_;
  double scale_;
  double const_value_;
};

class EdgePointFactor : public Factor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePointFactor(const Mat34d &pose, const Vec3d &origin_point,
                  const Vec3d &point_a, const Vec3d &point_b,
                  const double scale)
      : origin_point_(origin_point),
        point_a_(point_a),
        point_b_(point_b),
        scale_(scale) {
    pose_ = pose;
  }
  ~EdgePointFactor() {}
  virtual void ComputeResidualAndJacobian();
  virtual void ComputeResidualOnly();

 protected:
  Vec3d origin_point_;
  Vec3d point_a_;
  Vec3d point_b_;
  double scale_;
};

class PoseSolver {
 public:
  bool Solve(int iterations);
  void addFactor(const std::shared_ptr<Factor> &pfactor) {
    vptr_factors_.emplace_back(pfactor);
  }
  bool makeHessian();
  void backupPose() { backup_pose_ = pose_; }
  void setPose(const Mat34d &pose) { pose_ = pose; }
  Mat34d getPose() const { return pose_; }
  void updatePose();
  void rollbackStates() {
    pose_ = backup_pose_;
  }  // 有时候 update 后残差会变大，需要退回去，重来
  bool isGoodStepInLM();
  void initLamda();
  bool solverLinearEquation();
  void clearFactors() { vptr_factors_.clear(); }

 private:
  Mat34d pose_;
  Mat34d backup_pose_;
  Mat6d hessian_;
  Vec6d b_;
  Vec6d delta_x_;
  double cur_lambda_;
  double cur_chi_;
  double stop_threshold_;  // LM 迭代退出阈值条件
  double ni_;              //控制 Lambda 缩放大小
  std::vector<std::shared_ptr<Factor> > vptr_factors_;
};

}  // namespace cvte_lidar_slam