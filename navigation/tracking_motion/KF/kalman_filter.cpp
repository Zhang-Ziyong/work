/*
 * @Author: your name
 * @Date: 2020-10-26 11:25:50
 * @LastEditTime: 2020-11-14 11:07:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/object_track/KF/kalman_filter.cpp
 */
#include "kalman_filter.hpp"
#include <Eigen/Dense>
#include <iostream>
#include "log.hpp"

namespace TRACKING_MOTION {

KalmanFilter::KalmanFilter() {
  Q_k_ = 0.1 * Eigen::Matrix<double, 4, 4>::Identity();
  R_k_ = 10 * Eigen::Matrix<double, 2, 2>::Identity();
  covariance_ = 0.1 * Eigen::Matrix<double, 4, 4>::Identity();
  state_.setZero();
  last_state_.setZero();
  F_.setIdentity();
  F_.block<2, 2>(0, 2) = 0.1 * Eigen::Matrix<double, 2, 2>::Identity();
  H_.setZero();
  H_.block<2, 2>(0, 0) = Eigen::Matrix<double, 2, 2>::Identity();
  LOG(INFO) << "F: " << std::endl << F_ << std::endl;
  LOG(INFO) << "H: " << std::endl << H_ << std::endl;
}

KalmanFilter::KalmanFilter(const KalmanFilterConfig kalman_config) {
  Q_k_ = kalman_config.Q_k_;
  R_k_ = kalman_config.R_k_;
  covariance_ = kalman_config.covariance_;
  state_.setZero();
  F_.setIdentity();
  F_.block<2, 2>(0, 2) =
      kalman_config.delta_t_ * Eigen::Matrix<double, 2, 2>::Identity();
  H_.setZero();
  H_.block<2, 2>(0, 0) = Eigen::Matrix<double, 2, 2>::Identity();
  LOG(INFO) << "F: " << std::endl << F_ << std::endl;
  LOG(INFO) << "H: " << std::endl << H_ << std::endl;
}

void KalmanFilter::propagateState() {
  state_ = F_ * state_;
  covariance_ = F_ * covariance_ * F_.transpose() + Q_k_;
}

void KalmanFilter::resetState() {
  state_ = last_state_;
}

void KalmanFilter::getState(Eigen::Vector4d &state) const {
  state = state_;
}

void KalmanFilter::initState(const Eigen::Vector4d &state) {
  state_ = state;
  last_state_ = state;
}

void KalmanFilter::updateState(const Eigen::Vector2d &measures_pose) {
  Eigen::Matrix<double, 4, 2> kalman_gain;
  kalman_gain = covariance_ * H_.transpose() *
                (H_ * covariance_ * H_.transpose() + R_k_).inverse();
  //   std::cout << "measure error: " << std::endl << (measures_pose - H_ *
  //   state_).transpose() << std::endl;
  state_ = state_ + kalman_gain * (measures_pose - H_ * state_);
  covariance_ = (Eigen::Matrix<double, 4, 4>::Identity() - kalman_gain * H_) *
                covariance_;
  last_state_ = state_;
}

}  // namespace TRACKING_MOTION