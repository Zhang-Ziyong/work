/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file non_uniform_bspline.cpp
 *
 *@brief B样条库
 *
 *@modified by linyanlong(linyanlong@cvte.com)
 *
 *@version Navigation-v2.0
 *@data 2021-01-06
 ************************************************************************/

#include "bsline/bsline_base/non_uniform_bspline.h"
#include <glog/logging.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/SparseCholesky"
#include "eigen3/Eigen/SparseQR"
#include "pose2d/pose2d.hpp"
#include "chrono"

namespace CVTE_BABOT {

NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd &points,
                                     const int &order, const double &interval) {
  setUniformBspline(points, order, interval);
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd &points,
                                          const int &order,
                                          const double &interval) {
  control_points_ = points;
  p_ = order;
  interval_ = interval;

  n_ = points.rows() - 1;
  m_ = n_ + p_ + 1;

  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) {
    if (i <= p_) {
      u_(i) = double(-p_ + i) * interval_;
    } else if (i > p_ && i <= m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    } else if (i > m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    }
  }
}

void NonUniformBspline::setKnot(const Eigen::VectorXd &knot) {
  this->u_ = knot;
}

Eigen::VectorXd NonUniformBspline::getKnot() {
  return this->u_;
}

void NonUniformBspline::getTimeSpan(double &um, double &um_p) {
  um = u_(p_);
  um_p = u_(m_ - p_);
}

Eigen::MatrixXd NonUniformBspline::getControlPoint() {
  return control_points_;
}

pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double &u) {
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // determine which [ui,ui+1] lay in
  int k = p_;
  int end = m_ - p_;
  //二分查找确定区间
  while (u_(k + 1) < ub) {
    int mid = (k + end) / 2;
    if (u_(mid) > ub) {
      end = mid;
    } else {
      k = mid;
    }
  }
  // while (true) {
  //   if (u_(k + 1) >= ub)
  //     break;
  //   ++k;
  // }

  /* deBoor's alg */
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
    // cout << d[i].transpose() << endl;
  }

  for (int r = 1; r <= p_; ++r) {
    for (int i = p_; i >= r; --i) {
      double alpha =
          (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p_];
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double &t) {
  return evaluateDeBoor(t + u_(p_));
}

Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // The derivative of a b-spline is also a b-spline, its order become p_-1
  // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
  Eigen::MatrixXd ctp =
      Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    ctp.row(i) = p_ * (control_points_.row(i + 1) - control_points_.row(i)) /
                 (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

NonUniformBspline NonUniformBspline::getDerivative() {
  Eigen::MatrixXd ctp = getDerivativeControlPoints();
  NonUniformBspline derivative(ctp, p_ - 1, interval_);

  /* cut the first and last knot */
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

double NonUniformBspline::getInterval() {
  return interval_;
}

void NonUniformBspline::setPhysicalLimits(const double vel,
                                          const double angle_vel,
                                          const double acc) {
  limit_vel_ = vel;
  limit_angle_vel_ = angle_vel;
  limit_acc_ = acc;
  limit_ratio_ = 1.1;
}

bool NonUniformBspline::checkFeasibility(bool show) {
  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P = control_points_;
  int dimension = control_points_.cols();

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel =
        p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {
      if (show)
        LOG(ERROR) << "[Check]: Infeasible vel " << i << " :" << vel.transpose()
                   << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc =
        p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {
      if (show)
        LOG(ERROR) << "[Check]: Infeasible acc " << i << " :" << acc.transpose()
                   << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }
    }
  }

  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea;
}

double NonUniformBspline::checkRatio() {
  Eigen::MatrixXd P = control_points_;
  int dimension = control_points_.cols();

  // find max vel
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel =
        p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));
    }
  }
  // find max acc
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc =
        p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));
    }
  }
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  LOG(INFO) << "ratio: " << ratio << " "
            << "max vel: " << max_vel << " "
            << "max acc: " << max_acc;
  return ratio;
}

bool NonUniformBspline::reallocateTime(bool show) {
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  // cout << "origin knots:\n" << u_.transpose() << endl;
  bool fea = true;

  Eigen::MatrixXd P = control_points_;
  int dimension = control_points_.cols();

  double max_vel, max_acc;

  /* check vel feasibility and insert points */
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel =
        p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    double vel_norm = vel.norm();
    if (fabs(vel_norm) > limit_vel_ + 1e-4) {
      fea = false;
      if (show)
        LOG(INFO) << "[Realloc]: Infeasible vel " << i << " :"
                  << vel.transpose() << endl;

      max_vel = vel_norm;
      // for (int j = 0; j < dimension; ++j) {
      //   max_vel = max(max_vel, fabs(vel(j)));
      // }

      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_)
        ratio = limit_ratio_;

      double time_ori = u_(i + p_ + 1) - u_(i + 1);
      double time_new = ratio * time_ori;
      double delta_t = time_new - time_ori;
      double t_inc = delta_t / double(p_);

      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

      for (int j = i + p_ + 2; j < u_.rows(); ++j) { u_(j) += delta_t; }
    }
  }

  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd last_vel =
        (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + p_));
    Eigen::VectorXd vel =
        (P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + p_ + 1));
    double last_yaw = atan2(last_vel(1), last_vel(0));
    double yaw = atan2(vel(1), vel(0));
    double angle_vel = AngleCalculate::angle_diff(last_yaw, yaw) /
                       (u_(i + p_ + 1) - u_(i + p_));
    if (fabs(angle_vel) > limit_angle_vel_) {
      double radio = fabs(angle_vel) / limit_angle_vel_;
      double delta_t = p_ * radio * (u_(i + p_ + 1) - u_(i + p_)) -
                       (u_(i + p_ + 1) - u_(i + p_));
      double t_inc = delta_t / p_;
      // LOG(ERROR) << "[Realloc]: Infeasible angle_vel " << i << " :" <<
      // angle_vel << " "
      //   << t_inc;
      for (int j = i + p_; j <= i + p_ + 2; j++) {
        u_[j] += double(j - i - 2) * t_inc;
      }
      for (int j = i + p_ + 3; j <= m_; j++) { u_[j] += delta_t; }
    }
  }

  /* acc feasibility */
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc =
        p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {
      fea = false;
      if (show)
        LOG(INFO) << "[Realloc]: Infeasible acc " << i << " :"
                  << acc.transpose() << endl;

      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }

      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_)
        ratio = limit_ratio_;
      // cout << "ratio: " << ratio << endl;

      double time_ori = u_(i + p_ + 1) - u_(i + 2);
      double time_new = ratio * time_ori;
      double delta_t = time_new - time_ori;
      double t_inc = delta_t / double(p_ - 1);

      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        for (int j = 2; j <= 5; ++j) { u_(j) += double(j - 1) * t_inc; }

        for (int j = 6; j < u_.rows(); ++j) { u_(j) += 4.0 * t_inc; }

      } else {
        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) { u_(j) += delta_t; }
      }
    }
  }

  return fea;
}

void NonUniformBspline::lengthenTime(const double &ratio) {
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

void NonUniformBspline::recomputeInit() {}

void NonUniformBspline::parameterizeToBspline(
    const double &ts, const vector<Eigen::Vector3d> &point_set,
    const vector<Eigen::Vector3d> &start_end_derivative,
    Eigen::MatrixXd &ctrl_pts) {
  if (ts <= 0) {
    LOG(ERROR) << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    LOG(ERROR) << "[B-spline]:point set have only " << point_set.size()
               << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    LOG(ERROR) << "[B-spline]:derivatives error." << endl;
  }
  Eigen::Vector3d add_point = point_set.back() + start_end_derivative[2] * ts;
  vector<Eigen::Vector3d> add_point_set = point_set;
  // add_point_set[add_point_set.size() - 1] = add_point;
  // add_point_set.push_back(add_point);

  int K = add_point_set.size();
  // Eigen::Matrix3d A_begin;
  // A_begin << 1.0/6.0, 4.0/6.0, 1.0/6.0,
  //      -1.0/2.0/ts, 0, 1.0/2.0/ts,
  //      1.0/ts/ts, -2.0/ts/ts, 1.0/ts/ts;

  // Eigen::Vector3d Q0 = add_point_set[0];
  // Q0(2) = 0;
  // Eigen::Vector3d v0 = start_end_derivative[0];
  // Eigen::Vector3d a0 = start_end_derivative[2];
  // Eigen::Vector3d bx(Q0(0), v0(0), a0(0));
  // Eigen::Vector3d by(Q0(1), v0(1), a0(1));
  // Eigen::Vector3d bz(Q0(2), v0(2), a0(2));
  // // Eigen::Vector3d px = A_begin.colPivHouseholderQr().solve(bx);
  // // Eigen::Vector3d py = A_begin.colPivHouseholderQr().solve(by);
  // // Eigen::Vector3d pz = A_begin.colPivHouseholderQr().solve(bz);
  // Eigen::Vector3d px = A_begin.inverse()*bx;
  // Eigen::Vector3d py = A_begin.inverse()*by;
  // Eigen::Vector3d pz = A_begin.inverse()*bz;
  // Eigen::Matrix3d A_end;
  // A_end << 1.0/6.0, 4.0/6.0, 1.0/6.0,
  //      -1.0/2.0/ts, 0, 1.0/2.0/ts,
  //      1.0/ts/ts, -2.0/ts/ts, 1.0/ts/ts;

  // Eigen::Vector3d Qk = add_point_set[add_point_set.size() - 1];
  // Qk(2) = 0;
  // Eigen::Vector3d vk = start_end_derivative[1];
  // Eigen::Vector3d ak = start_end_derivative[3];
  // Eigen::Vector3d bkx(Qk(0), vk(0), ak(0));
  // Eigen::Vector3d bky(Qk(1), vk(1), ak(1));
  // Eigen::Vector3d bkz(Qk(2), vk(2), ak(2));
  // // Eigen::Vector3d pkx = A_end.colPivHouseholderQr().solve(bkx);
  // // Eigen::Vector3d pky = A_end.colPivHouseholderQr().solve(bky);
  // // Eigen::Vector3d pkz = A_end.colPivHouseholderQr().solve(bkz);
  // Eigen::Vector3d pkx = A_end.inverse()*bkx;
  // Eigen::Vector3d pky = A_end.inverse()*bky;
  // Eigen::Vector3d pkz = A_end.inverse()*bkz;
  // Eigen::Matrix3d temp_A;
  // temp_A.block<3,1>(0,0) = A_end*pkx;
  // temp_A.block<3,1>(0,1) = A_end*pky;
  // temp_A.block<3,1>(0,2) = A_end*pkz;
  // LOG(INFO) << "temp_A: " << std::endl << temp_A;
  // LOG(INFO) << "end_derivative: " << std::endl <<
  // start_end_derivative[1].transpose()
  //   << " " << start_end_derivative[3].transpose();

  // ctrl_pts.resize(K + 2, 3);
  // ctrl_pts.block<3,1>(0,0) = px;
  // ctrl_pts.block<3,1>(0,1) = py;
  // ctrl_pts.block<3,1>(0,2) = pz;
  // ctrl_pts.block<3,1>(K - 1, 0) = pkx;
  // ctrl_pts.block<3,1>(K - 1, 1) = pky;
  // ctrl_pts.block<3,1>(K - 1, 2) = pkz;
  // for(size_t index = 2; index < add_point_set.size() - 2; index++) {
  //   ctrl_pts.block<1,3>(index+1,0) = add_point_set[index+1].transpose();
  // }

  // write A
  Eigen::Vector3d prow(3), vrow_begin(3), vrow_end(3), arow(3);
  double lambda_pose = 3.0;
  prow << 1, 4, 1;
  vrow_begin << -1, 0, 1;
  vrow_end << -1, 0, 1;
  arow << 1, -2, 1;

  // Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);
  Eigen::SparseMatrix<double> A(K + 4, K + 2);

  for (int i = 0; i < K; ++i) {
    // A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();
    A.insert(i, i) = (1 / 6.0) * prow(0);
    A.insert(i, i + 1) = (1 / 6.0) * prow(1);
    A.insert(i, i + 2) = (1 / 6.0) * prow(2);
  }
  // A.block(0, 0, 1, 3) = 5*A.block(0, 0, 1, 3);
  // A.block(K - 1, K - 1, 1, 3) = 5*A.block(K - 1, K - 1, 1, 3);
  A.coeffRef(0, 0) = lambda_pose * A.coeffRef(0, 0);
  A.coeffRef(0, 1) = lambda_pose * A.coeffRef(0, 1);
  A.coeffRef(0, 2) = lambda_pose * A.coeffRef(0, 2);

  A.coeffRef(K - 1, K - 1) = lambda_pose * A.coeffRef(K - 1, K - 1);
  A.coeffRef(K - 1, K) = lambda_pose * A.coeffRef(K - 1, K);
  A.coeffRef(K - 1, K + 1) = lambda_pose * A.coeffRef(K - 1, K + 1);
  A.insert(K, 0) = (1 / 2.0 / ts) * vrow_begin(0);
  A.insert(K, 1) = (1 / 2.0 / ts) * vrow_begin(1);
  A.insert(K, 2) = (1 / 2.0 / ts) * vrow_begin(2);
  A.insert(K + 1, K - 1) = (1 / 2.0 / ts) * vrow_end(0);
  A.insert(K + 1, K) = (1 / 2.0 / ts) * vrow_end(1);
  A.insert(K + 1, K + 1) = (1 / 2.0 / ts) * vrow_end(2);
  A.insert(K + 2, 0) = lambda_pose * (1 / ts / ts) * arow(0);
  A.insert(K + 2, 1) = lambda_pose * (1 / ts / ts) * arow(1);
  A.insert(K + 2, 2) = lambda_pose * (1 / ts / ts) * arow(2);
  A.insert(K + 3, K - 1) = lambda_pose * (1 / ts / ts) * arow(0);
  A.insert(K + 3, K) = lambda_pose * (1 / ts / ts) * arow(1);
  A.insert(K + 3, K + 1) = lambda_pose * (1 / ts / ts) * arow(2);
  // A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
  // A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  // A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
  // A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
  // cout << "A:\n" << A << endl;

  // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
  // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
  // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // write b
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i) {
    bx(i) = add_point_set[i](0);
    by(i) = add_point_set[i](1);
    bz(i) = 0;
  }
  bx(0) = lambda_pose * bx(0);
  by(0) = lambda_pose * by(0);
  bz(0) = lambda_pose * bz(0);

  bx(K - 1) = lambda_pose * bx(K - 1);
  by(K - 1) = lambda_pose * by(K - 1);
  bz(K - 1) = lambda_pose * bz(K - 1);

  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }
  Eigen::SparseMatrix<double> At = A.transpose();
  Eigen::SparseMatrix<double> AtA(K + 2, K + 2);
  AtA = At * A;
  // Eigen::MatrixXd temp_AtA(K + 2, K + 2);
  // temp_AtA = (A).transpose() * (A);
  // for(size_t i = 0;i < K + 2;i++) {
  //   for(size_t j = 0;j < K + 2;j++) {
  //     // if(fabs(temp_AtA(i,j) > 0.0000001)) {
  //       AtA.insert(i,j) = temp_AtA(i,j);
  //     // }
  //   }
  // }
  Eigen::VectorXd Atbx(K + 2), Atby(K + 2), Atbz(K + 2);
  Atbx = At * bx;
  Atby = At * by;
  Atbz = At * bz;
  // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::AMDOrdering<int>> qr;
  AtA.makeCompressed();
  // typedef Eigen::SparseMatrix<double> sparseMatrix;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> sparse_ltl;
  // qr.compute(AtA);
  sparse_ltl.compute(AtA);
  auto befort_solve = std::chrono::system_clock::now();
  Eigen::VectorXd px = sparse_ltl.solve(Atbx);
  Eigen::VectorXd py = sparse_ltl.solve(Atby);
  Eigen::VectorXd pz = sparse_ltl.solve(Atbz);
  auto after_solve = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = after_solve - befort_solve;
  LOG(INFO) << "sparse matrix solve time: " << diff.count();

  // solve Ax = b
  // Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  // Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  // Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;

  // cout << "[B-spline]: parameterization ok." << endl;
}

double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength(const double &res) {
  double length = 0.0;
  double dur = getTimeSum();
  Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double NonUniformBspline::getJerk() {
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
  int dimension = ctrl_pts.cols();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v) {
  NonUniformBspline vel = getDerivative();
  double tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double vn = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v = mean_vel;
  max_v = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a) {
  NonUniformBspline acc = getDerivative().getDerivative();
  double tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double an = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a = mean_acc;
  max_a = max_acc;
}
}  // namespace CVTE_BABOT
