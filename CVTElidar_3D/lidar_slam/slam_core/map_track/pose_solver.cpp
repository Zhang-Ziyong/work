/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: /lidar_slam/slam_core/map_track/pose_solver.cpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-12-23 14:36:28
 ************************************************************************/
#include "map_track/pose_solver.hpp"

#include <glog/logging.h>
#include "common/debug_tools/tic_toc.h"
namespace cvte_lidar_slam {

void PlanePointFactor::ComputeResidualAndJacobian() {
  Vec3d map_point = Mathbox::multiplePoint(pose_, origin_point_);
  residual_ = scale_ * (plane_norm_.transpose() * map_point + const_value_);
  jacobian_.setZero();
  jacobian_.block<1, 3>(0, 0) =
      -scale_ * plane_norm_.transpose() *
      Mathbox::skew(map_point - pose_.block<3, 1>(0, 3));
  jacobian_.block<1, 3>(0, 3) = scale_ * plane_norm_.transpose();
}

void PlanePointFactor::ComputeResidualOnly() {
  Vec3d map_point = Mathbox::multiplePoint(pose_, origin_point_);
  residual_ = scale_ * (plane_norm_.transpose() * map_point + const_value_);
}

void EdgePointFactor::ComputeResidualAndJacobian() {
  Vec3d map_point = Mathbox::multiplePoint(pose_, origin_point_);
  const Vec3d &v_am = map_point - point_a_;
  const Vec3d &v_ab = point_b_ - point_a_;  // v_ab.norm() = 0.2
  const Vec3d &v_bm = map_point - point_b_;
  residual_ = 5 * scale_ * (v_am.cross(v_bm)).norm();
  // residual_ = scale_ * (v_am.cross(v_bm)).norm()/v_ab.norm()

  double k = 25 * v_ab.transpose() * v_am;
  const Vec3d &point_vetical = point_a_ + k * v_ab;
  const Vec3d &vetical_vec = map_point - point_vetical;
  jacobian_.setZero();
  jacobian_.block<1, 3>(0, 0) =
      -scale_ * (vetical_vec.transpose()) *
      Mathbox::skew(map_point - pose_.block<3, 1>(0, 3));
  jacobian_.block<1, 3>(0, 3) = scale_ * (vetical_vec.transpose());
}

void EdgePointFactor::ComputeResidualOnly() {
  Vec3d map_point = Mathbox::multiplePoint(pose_, origin_point_);
  const Vec3d &v_am = map_point - point_a_;
  const Vec3d &v_ab = point_b_ - point_a_;
  const Vec3d &v_bm = map_point - point_b_;
  residual_ = 5 * scale_ * (v_am.cross(v_bm)).norm();
}

bool PoseSolver::Solve(int iterations) {
  TicToc t_solve;
  if (pose_.hasNaN()) {
    LOG(ERROR) << "Wrong pose data";
    return false;
  }
  makeHessian();
  initLamda();
  // LM 算法迭代求解
  bool stop = false;
  int iter = 0;
  double last_chi = 1e20;
  while (!stop && (iter < iterations)) {
    // std::cout << "lamda:  " << cur_lambda_ << " " << iter << " " << cur_chi_
    //           << std::endl;
    bool oneStepSuccess = false;
    int false_cnt = 0;
    while (!oneStepSuccess && false_cnt < 3) {
      solverLinearEquation();
      backupPose();
      updatePose();
      oneStepSuccess = isGoodStepInLM();

      if (oneStepSuccess) {
        // 在新线性化点 构建 hessian
        makeHessian();
        false_cnt = 0;
      } else {
        false_cnt++;
        rollbackStates();  // 误差没下降，回滚
      }
    }
    iter++;
    // std::cout << "lamda:  " << cur_lambda_ << " " << iter << std::endl;
    // 优化退出条件3： cur_chi_ 跟第一次的 chi2 相比，下降了 1e6 倍则退出
    if (last_chi - cur_chi_ < 1e-5) {
      // LOG(INFO) << "sqrt(currentChi_) <= stopThresholdLM_" << std::endl;
      stop = true;
    }
    if (oneStepSuccess) {
      last_chi = cur_chi_;
    }
  }
  return true;
}

bool PoseSolver::makeHessian() {
  if (vptr_factors_.empty()) {
    LOG(ERROR) << "Empty factor container";
    return false;
  }
  hessian_.setZero();
  b_.setZero();
  for (const auto &it : vptr_factors_) {
    it->updatePose(pose_);
    it->ComputeResidualAndJacobian();
    const Eigen::Matrix<double, 1, 6, Eigen::RowMajor> &jac = it->Jacobian();
    double drho, robustInfo;
    it->RobustInfo(drho, robustInfo);
    hessian_.noalias() += robustInfo * jac.transpose() * jac;
    b_.noalias() -= drho * jac.transpose() * it->Residual();
  }
  return true;
}

bool PoseSolver::isGoodStepInLM() {
  const double &scale =
      0.5 * delta_x_.transpose() * (cur_lambda_ * delta_x_ + b_) + 1e-6;

  // recompute residuals after update state
  double tempChi = 0.0;
  for (const auto &it : vptr_factors_) {
    it->updatePose(pose_);
    it->ComputeResidualOnly();
    tempChi += it->RobustChi2();
  }
  tempChi *= 0.5;  // 1/2 * err^2

  double rho = (cur_chi_ - tempChi) / scale;
  // std::cout << cur_chi_ << " " << tempChi << " " << scale << std::endl;
  if (rho > 0 && std::isfinite(tempChi))  // last step was good, 误差在下降
  {
    double alpha = 1. - pow((2 * rho - 1), 3);
    alpha = std::min(alpha, 2. / 3.);
    double scaleFactor = (std::max)(1. / 3., alpha);
    cur_lambda_ *= scaleFactor;
    ni_ = 2;
    cur_chi_ = tempChi;
    return true;
  } else {
    // std::cout << "Error " << cur_chi_ << " " << tempChi << std::endl;
    cur_lambda_ *= ni_;
    ni_ *= 2;
    return false;
  }
}

// This function must be used after MakeHessian
void PoseSolver::initLamda() {
  ni_ = 2.;
  cur_lambda_ = -1.;
  cur_chi_ = 0.0;
  for (auto it : vptr_factors_) { cur_chi_ += it->RobustChi2(); }
  cur_chi_ *= 0.5;

  stop_threshold_ = 1e-9 * cur_chi_;

  // std::cout << "hessian_  \n " << hessian_ << std::endl;

  double maxDiagonal = 0;
  for (ulong i = 0; i < 6; ++i) {
    maxDiagonal = std::max(fabs(hessian_(i, i)), maxDiagonal);
  }
  maxDiagonal = std::min(5e10, maxDiagonal);
  const double &tau = 1e-5;  // 1e-5
  cur_lambda_ = tau * maxDiagonal;
  // std::cout << "init lambda  " << cur_lambda_ << std::endl;
}

bool PoseSolver::solverLinearEquation() {
  Mat6d H = hessian_;
  for (size_t i = 0; i < 6; ++i) { H(i, i) += cur_lambda_; }
  delta_x_ = H.ldlt().solve(b_);
}

void PoseSolver::updatePose() {
  const Mat3d &dR = Mathbox::Exp3d(delta_x_.head<3>());
  const Mat3d &R = pose_.block<3, 3>(0, 0);
  const Vec3d &p = pose_.block<3, 1>(0, 3);
  pose_.block<3, 3>(0, 0) = dR * R;
  pose_.block<3, 1>(0, 3) = p + delta_x_.tail<3>();
}

}  // namespace cvte_lidar_slam