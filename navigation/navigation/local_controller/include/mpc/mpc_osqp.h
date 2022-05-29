/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: /src/navigation/navigation/local_controller/include/mpc/mpc_osqp.h
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2021-01-12 11:07:31
 ************************************************************************/

#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Core>
#include <osqp/osqp.h>

#include "glog/logging.h"

#include "mpc_data_type.h"

namespace CVTE_BABOT {
class MpcOsqp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcOsqp(const Eigen::MatrixXd &matrix_q, const Eigen::MatrixXd &matrix_r,
          const Eigen::MatrixXd &matrix_initial_x,
          const Eigen::MatrixXd &matrix_u_lower,
          const Eigen::MatrixXd &matrix_u_upper,
          const Eigen::MatrixXd &matrix_x_lower,
          const Eigen::MatrixXd &matrix_x_upper, const int state_dim,
          const int control_dim, const double dt, const int max_iter,
          const int horizon, const double eps_abs);

  inline void UpdateState(const std::vector<MpcState> &ref_states) {
    ref_states_ = ref_states;
  }

  // control vector and state
  bool Solve(std::vector<double> *states, std::vector<double> *control_cmd);

 private:
  void CalculateKernel(std::vector<c_float> *P_data,
                       std::vector<c_int> *P_indices,
                       std::vector<c_int> *P_indptr);
  void CalculateEqualityConstraint(std::vector<c_float> *A_data,
                                   std::vector<c_int> *A_indices,
                                   std::vector<c_int> *A_indptr);
  void CalculateGradient();
  void CalculateConstraintVectors();
  void ComputeA_B(const MpcState &state, Eigen::MatrixXd &matrix_a,
                  Eigen::MatrixXd &matrix_b);
  OSQPSettings *Settings();
  OSQPData *Data();
  void FreeData(OSQPData *data);

  template <typename T>
  T *CopyData(const std::vector<T> &vec) {
    T *data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }

 private:
  Eigen::MatrixXd matrix_a_;
  Eigen::MatrixXd matrix_b_;
  Eigen::MatrixXd matrix_q_;
  Eigen::MatrixXd matrix_r_;
  Eigen::MatrixXd matrix_initial_x_;
  const Eigen::MatrixXd matrix_u_lower_;
  const Eigen::MatrixXd matrix_u_upper_;
  const Eigen::MatrixXd matrix_x_lower_;
  const Eigen::MatrixXd matrix_x_upper_;
  const Eigen::MatrixXd matrix_x_ref_;
  Eigen::MatrixXd matrix_u_ref_;
  std::vector<MpcState> ref_states_;
  int max_iteration_;
  size_t horizon_;
  double eps_abs_;
  double dt_ = 0.1;
  size_t state_dim_;
  size_t control_dim_;
  size_t num_param_;
  int num_constraint_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd lowerBound_;
  Eigen::VectorXd upperBound_;
};
}  // namespace CVTE_BABOT
