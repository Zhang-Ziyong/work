/*
 * @Author: your name
 * @Date: 2020-10-30 15:46:46
 * @LastEditTime: 2021-06-24 15:00:01
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/common/tracking_motion_config.hpp
 */
#ifndef TRACKING_MOTION_CONFIG
#define TRACKING_MOTION_CONFIG
#include "slam_math.hpp"
namespace TRACKING_MOTION {
struct PotentialControllerConfig {
  PotentialControllerConfig()
      : repulsive_top_right_(0.25, -0.45),
        repulsive_top_left_(0.25, 0.45),
        repulsive_down_right_(-0.75, -0.45),
        repulsive_down_left_(-0.75, 0.45),
        footprint_top_right_(0.15, -0.35),
        footprint_top_left_(0.15, 0.35),
        footprint_down_right_(-0.65, -0.35),
        footprint_down_left_(-0.65, 0.35) {}
  int scan_size_ = 360;
  double k_attactive_ = 2.0;
  double k_repulsive_ = 1.0;
  double k_error_angle_ = 0.8;
  double k_error_dist_ = 0.35;
  double k_vel_ = 0.3;
  double k_d_ = 1;
  double slow_dist_ = 2;
  double stop_dist_ = 1.5;
  double max_vel_ = 1.0;
  double max_rot_ = 0.6;
  Vec2d repulsive_top_right_;
  Vec2d repulsive_top_left_;
  Vec2d repulsive_down_right_;
  Vec2d repulsive_down_left_;
  Vec2d footprint_top_right_;
  Vec2d footprint_top_left_;
  Vec2d footprint_down_right_;
  Vec2d footprint_down_left_;
  double T_bl_ = 0.25;
};

struct KalmanFilterConfig {
  Eigen::Matrix<double, 4, 4> Q_k_ =
      0.1 * Eigen::Matrix<double, 4, 4>::Identity();
  Eigen::Matrix<double, 2, 2> R_k_ =
      5 * Eigen::Matrix<double, 2, 2>::Identity();
  Eigen::Matrix<double, 4, 4> covariance_ =
      0.1 * Eigen::Matrix<double, 4, 4>::Identity();
  double delta_t_ = 0.1;
};

struct LaserObjectTrackConfig {
  int region_interest_ = 20;
  double grid_resolution_ = 0.2;
  double th_grid_het_ = 0.4;
  double sensor_height_ = 0.5;
  std::string topic_ = "/velodyne_points";
  std::string model_path_ = "./install/tracking_motion/model/LSC_1.model";
  std::string range_path_ = "./install/tracking_motion/model/LSC_1.range";
};

struct TrackingMotionConfig {
  PotentialControllerConfig potential_config_;
  KalmanFilterConfig kalman_config_;
  LaserObjectTrackConfig laser_object_track_config_;
};
}  // namespace TRACKING_MOTION

#endif
