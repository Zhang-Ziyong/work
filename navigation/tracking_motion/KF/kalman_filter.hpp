/*
 * @Author: your name
 * @Date: 2020-10-26 11:25:34
 * @LastEditTime: 2020-11-04 16:42:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/object_track/KF/kalman_filter.hpp
 */
#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_
#include "eigen3/Eigen/Core"
#include <vector>
#include "tracking_motion_config.hpp"
namespace TRACKING_MOTION {

class KalmanFilter {
 public:
  KalmanFilter();
  KalmanFilter(const KalmanFilterConfig kalman_config);
  ~KalmanFilter() = default;
  KalmanFilter(const KalmanFilter &obj) = delete;
  const KalmanFilter &operator=(const KalmanFilter &obj) = delete;
  void propagateState();
  void updateState(const Eigen::Vector2d &measures_pose);
  void resetState();
  void getState(Eigen::Vector4d &state) const;
  void initState(const Eigen::Vector4d &state);

 private:
  Eigen::Matrix<double, 4, 4> Q_k_;
  Eigen::Matrix<double, 2, 2> R_k_;
  Eigen::Matrix<double, 4, 4> covariance_;
  Eigen::Matrix<double, 4, 4> F_;
  Eigen::Matrix<double, 2, 4> H_;
  Eigen::Vector4d state_;
  Eigen::Vector4d last_state_;
};

}  // namespace TRACKING_MOTION

#endif