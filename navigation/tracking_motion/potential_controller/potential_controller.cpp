/*
 * @Author: your name
 * @Date: 2020-10-30 16:25:51
 * @LastEditTime: 2021-07-19 09:44:15
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/potential_controller/potential_controller.cpp
 */
#include "potential_controller.hpp"
#include "log.hpp"
namespace TRACKING_MOTION {
PotentialController::PotentialController(
    const PotentialControllerConfig &config) {
  config_ = config;
  // check_obstacle_stop_thread_ =
  // std::thread(std::bind(&PotentialController::obstackeStopThreadCallback,
  // this));
  is_stop_ = false;
  is_obstacle_stop_ = false;
}
PotentialController::~PotentialController() {
  is_stop_ = true;
  if (check_obstacle_stop_thread_.joinable()) {
    check_obstacle_stop_thread_.join();
  }
}

void PotentialController::obstackeStopThreadCallback() {
  while (!is_stop_) {
    scan_mutex_.lock();
    bool in_obstacle_field = false;
    for (size_t index = 0; index < scan_.size(); index++) {
      if (isPointInObstacleStop(scan_[index])) {
        // std::cout << "point in obstacle stop:" << scan_[index].transpose() <<
        // std::endl;
        in_obstacle_field = true;
      }
    }
    scan_mutex_.unlock();
    if (in_obstacle_field) {
      LOG(ERROR) << "obstacle stop";
      is_obstacle_stop_ = true;
    } else {
      is_obstacle_stop_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool PotentialController::isPointInObstacleStop(const Vec2d &point) {
  double obstacle_stop_length;
  if (std::fabs(cur_vel_ > 0.3)) {
    obstacle_stop_length = 0.5;
  } else {
    obstacle_stop_length = 0.0;
  }
  if (point(0) < config_.footprint_top_right_(0) + obstacle_stop_length &&
      point(0) > config_.footprint_down_right_(0) &&
      point(1) > config_.footprint_top_right_(1) &&
      point(1) < config_.footprint_down_left_(1)) {
    return true;
  }
}

bool PotentialController::isPointInRepulsive(const Vec2d &point) {
  if (point(0) < config_.repulsive_top_right_(0) &&
      point(0) > config_.repulsive_down_right_(0) &&
      point(1) > config_.repulsive_top_right_(1) &&
      point(1) < config_.repulsive_top_left_(0)) {
    return true;
  }
}

bool PotentialController::pointInStopPosLinear(const Vec2d &point) {
  double obstacle_stop_length;
  if (std::fabs(cur_vel_ > 0.3)) {
    obstacle_stop_length = 0.4;
  } else {
    obstacle_stop_length = 0.0;
  }
  if (point(0) < config_.repulsive_top_right_(0) + obstacle_stop_length &&
      point(0) > 0 && point(1) > config_.repulsive_top_right_(1) &&
      point(1) < config_.repulsive_top_left_(0)) {
    return true;
  } else {
    return false;
  }
}
bool PotentialController::pointInStopNegtiveLinear(const Vec2d &point) {
  if (point(0) < 0 && point(0) > config_.repulsive_down_right_(0) &&
      point(1) > config_.repulsive_top_right_(1) &&
      point(1) < config_.repulsive_top_left_(0)) {
    return true;
  } else {
    return false;
  }
}
bool PotentialController::pointInStopNegtiveRot(const Vec2d &point) {
  if (point(0) < config_.repulsive_top_right_(0) &&
      point(0) > config_.repulsive_down_right_(0) && point(1) > 0 &&
      point(1) < config_.repulsive_top_left_(0)) {
    return true;
  } else if (point(0) < config_.repulsive_top_right_(0) && point(0) > 0 &&
             point(1) < 0 && point(1) > config_.repulsive_top_right_(1)) {
    return true;
  } else {
    return false;
  }
}
bool PotentialController::pointInStopPosRot(const Vec2d &point) {
  if (point(0) < config_.repulsive_top_right_(0) &&
      point(0) > config_.repulsive_down_right_(0) &&
      point(1) > config_.repulsive_top_right_(1) && point(1) < 0) {
    return true;
  } else if (point(0) < config_.repulsive_top_right_(0) && point(0) > 0 &&
             point(1) > 0 && point(1) < config_.repulsive_top_left_(1)) {
    return true;
  } else {
    return false;
  }
}

void PotentialController::setTargetPose(const Mat34d &target_pose) {
  target_pose_mutex_.lock();
  target_pose_ = target_pose;
  target_pose_mutex_.unlock();
}

void PotentialController::setCurVel(double vel) {
  cur_vel_ = vel;
}

void PotentialController::InputScan(const std::vector<Vec2d> &scan) {
  scan_mutex_.lock();
  scan_ = scan;
  scan_mutex_.unlock();
}

Vec3d PotentialController::computeAttractiveForce() {
  target_pose_mutex_.lock();
  Eigen::Vector3d attractive_force =
      config_.k_attactive_ * target_pose_.block<3, 1>(0, 3);
  target_pose_mutex_.unlock();
  return attractive_force;
}

Vec3d PotentialController::computeRepulsiveForce() {
  scan_mutex_.lock();
  Eigen::Vector3d repulsive_force;
  repulsive_force.setZero();
  for (int index = 0; index < scan_.size(); index++) {
    Vec2d point = scan_[index];
    // LOG(ERROR) << "point: " << point.transpose();
    if (isPointInRepulsive(point)) {
      double dist = fabs(point(1));
      Eigen::Vector3d direct(-point(0), -point(1), 0.0);
      repulsive_force += config_.k_repulsive_ * direct.normalized();
    }
  }
  scan_mutex_.unlock();
  return repulsive_force;
}

void PotentialController::computeCommand(CmdVel &cmd_vel,
                                         const Vec2d &predict_vel) {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  for (size_t index = 0; index < scan_.size(); index++) {
    if (isPointInObstacleStop(scan_[index])) {
      LOG(INFO) << "obstacle stop : " << scan_[index].transpose();
      cmd_vel.vel_ = 0.0;
      cmd_vel.w_ = 0.0;
      return;
    }
  }
  Eigen::Vector3d attractive_force = computeAttractiveForce();
  // Eigen::Vector3d repulsive_force = computeRepulsiveForce();
  Eigen::Vector3d total_force = attractive_force;
  LOG(INFO) << "attractive_force: " << attractive_force.transpose();
  // LOG(INFO) << "repulsive_force: " << repulsive_force.transpose();
  // LOG(INFO) << "total_force: " << total_force.transpose();
  double force_yaw = atan2(total_force(1), total_force(0));
  LOG(INFO) << "force_yaw: " << force_yaw;
  double w;
  double v;
  Eigen::Vector3d relative_pose = attractive_force / config_.k_attactive_;
  double dist = relative_pose.norm();
  static double last_dist = dist;
  LOG(INFO) << "dist:" << dist;
  if (dist < config_.stop_dist_) {
    // w = 0;
    w = config_.k_error_angle_ * force_yaw;
    v = 0;
  } else {
    if (dist < config_.slow_dist_) {
      v = (dist - config_.stop_dist_) * config_.k_error_dist_ *
          config_.slow_dist_ / (config_.slow_dist_ - config_.stop_dist_);
    } else {
      v = config_.k_error_dist_ * dist + config_.k_vel_ * predict_vel.norm() +
          config_.k_d_ * (dist - last_dist);
    }
    w = config_.k_error_angle_ * force_yaw;
    double v_limit =
        config_.max_vel_ -
        w * w * config_.max_vel_ / (config_.max_rot_ * config_.max_rot_);
    if (v > v_limit) {
      v = v_limit;
    }
    if (v < 0) {
      v = 0;
    }
  }
  bool stop_pose_linear = false;
  bool stop_negative_linear = false;
  bool stop_pose_rot = false;
  bool stop_negative_rot = false;
  last_dist = dist;
  // scan_mutex_.lock();
  // for (int index = 0; index < scan_.size(); index++) {
  //   Vec2d point = scan_[index];
  //   if (!stop_negative_linear) {
  //     if (pointInStopNegtiveLinear(point)) {
  //       stop_negative_linear = true;
  //     }
  //   }
  //   if (!stop_pose_linear) {
  //     if (pointInStopPosLinear(point)) {
  //       stop_pose_linear = true;
  //     }
  //   }
  //   if (!stop_pose_rot) {
  //     if (pointInStopPosRot(point)) {
  //       stop_pose_rot = true;
  //     }
  //   }
  //   if (!stop_negative_rot) {
  //     if (pointInStopNegtiveRot(point)) {
  //       stop_negative_rot = true;
  //     }
  //   }
  // }
  // scan_mutex_.unlock();
  // if (stop_negative_rot) {
  //   LOG(ERROR) << "stop negative rot";
  //   if (w < 0) {
  //     w = 0;
  //   }
  // }
  // if (stop_pose_rot) {
  //   LOG(ERROR) << "stop positive rot";
  //   if (w > 0) {
  //     w = 0;
  //   }
  // }
  // if (stop_pose_linear) {
  //   LOG(ERROR) << "stop positive vel";
  //   if (v > 0) {
  //     v = 0;
  //   }
  // }
  cmd_vel.w_ = w;
  cmd_vel.vel_ = v;
  // if (!is_obstacle_stop_) {
  //   cmd_vel.w_ = w;
  //   cmd_vel.vel_ = v;
  // } else {
  //   cmd_vel.w_ = 0;
  //   cmd_vel.vel_ = 0;
  // }
}
}  // namespace TRACKING_MOTION
