/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file pid_planner.cpp
 *
 *@brief PID算法实现文件
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author "GeRoNa OpenSource"
 *@version Navigation-v2.0
 *@data 2020-01-07
 ************************************************************************/

#include "pid/pid_planner.hpp"
#include "controller_ultis/Line2d.h"
#include "dwa/world_model/costmap_model.hpp"

namespace CVTE_BABOT {

/**
 * \brief Signum function
 * \author user79758@stackoverflow
 *
 * \see http://stackoverflow.com/a/4609795/2095383
 */
template <typename T>
int sign(T val) {
  return (T(0) < val) - (val < T(0));
}

PIDLocalController::PIDLocalController(std::shared_ptr<Costmap2d> costmap)
    : behaviour_(ON_PATH) {
  getParams();
  steer_pid_ = PidController<1>(pid_kp_, pid_ki_, pid_kd_, pid_t_);
  CostmapModel::getPtrInstance()->setPtrCostmap(costmap);
}

void PIDLocalController::getParams() {
  if (!ptr_navigation_mediator_->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return;
  }

  double max_x = 0.0;
  double min_x = 0.0;
  double max_y = 0.0;
  double min_y = 0.0;
  double max_th = 0.0;
  double min_th = 0.0;
  double min_velocity_threshold = 0.0;
  double min_rotate_threshold = 0.0;

  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.kp", pid_kp_, 1.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.ki", pid_ki_, 0.001);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.kd", pid_kd_, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.kt", pid_t_, 0.03);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.wp_tolerance", tolerance_,
      1.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.max_steer",
      max_steer_param_, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.car_length",
      car_length_params_, 0.38);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.steer_slow_threshold",
      steer_slow_threshold_param_, 0.25);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.back_multiple",
      back_multiple_, 1.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.max_x", max_x, 1.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.min_x", min_x, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.max_y", max_y, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.min_y", min_y, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.max_th", max_th, 0.5);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.min_th", min_th, 0.0);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.min_velocity_threshold",
      min_velocity_threshold, 0.1);
  ptr_navigation_mediator_->getParam(
      "path_follower.controller_algorithm_params.pid.min_rotate_threshold",
      min_rotate_threshold, 0.05);
  setMoveSpeedRange(min_x, max_x, min_y, max_y, min_th, max_th);
  limits_->setMinVelocityThreshold(min_velocity_threshold);
  limits_->setMinRotateThreshold(min_rotate_threshold);
}

void PIDLocalController::updateCostMap(std::shared_ptr<Costmap2d> ptr_costmap) {
  CostmapModel::getPtrInstance()->setPtrCostmap(ptr_costmap);
}

MoveCommand::MoveCommandStatus PIDLocalController::computeMoveComand(
    MoveCommand &cmd) {
  /* This is a reimplemented, simplified version of the old behaviour based
   * Ackermann
   * controller. There is still a internal state called "behaviour", but this is
   * not a strict
   * state machine and there are no behaviour classes anymore.
   *
   * The path is processed as follows:
   *  - At every time, only the current subpath is taken into account.
   *  - While driving on a subpath, the controller tries to minimize the
   * distance to the line
   *    of the current path segment (defined by the next two waypoints).
   * behaviour_ = ON_PATH.
   *  - When reaching the last waypoint of the current sub path, the behaviour
   * switches to
   *    APPROACH_SUBPATH_END and tries to hit the waypoint as close as possible.
   *  - When this last waypoint is reached, the robot is stopped and the
   * controller waits,
   *    until there is no more movement (behaviour_ = WAIT_FOR_STOP)
   *  - When the robot really has stopped, the whole procedure starts again with
   * the next sub
   *    path, until the end of the last sub path (= the goal) is reached
   */
  if (path_.empty()) {
    return MoveCommand::MoveCommandStatus::ERROR;
  }

  // If wait_for_stop_ is set, do nothing, until the actual velocity fell below
  // a given
  // threshold.
  // When the robot has finally stopped, go to the next subpath or quit, if goal
  // is reached.
  if (behaviour_ == WAIT_FOR_STOP) {
    // stopMotion(); //< probably not necessary to repeat this, but be on the
    // save side.
    Velocity cur_vel = getCurrentVelocity();

    // do nothing until robot has realy stopped.
    if ((std::abs(cur_vel.d_x) > 0.01) || (std::abs(cur_vel.d_y) > 0.01) ||
        (std::abs(cur_vel.d_yaw) > 0.01)) {
      LOG(INFO) << "WAITING until no more motion";
      return MoveCommand::MoveCommandStatus::OKAY;
    } else {
      LOG(INFO) << "Done at waypoint -> reset";
      // path_.switchToNextSubPath();
      if (path_.isDone()) {
        behaviour_ = ON_PATH;  // not necessary to set this explicitly, but it
                               // is more clear.
        path_.clear();
        return MoveCommand::MoveCommandStatus::REACHED_GOAL;
      }
      behaviour_ = ON_PATH;  // not necessary to set this explicitly, but it is
                             // more clear.
    }
  }

  // choose waypoint for this step
  selectWaypoint();

  // check if done (if last step was ATP and the direction sign flipped)
  double dir_sign = sign<double>(next_wp_local_.x());
  if (behaviour_ == APPROACH_SUBPATH_END && dir_sign != getDirSign()) {
    behaviour_ = WAIT_FOR_STOP;
    return MoveCommand::MoveCommandStatus::OKAY;
  }
  LOG(INFO) << "setDirSign = " << dir_sign;
  setDirSign(dir_sign);

  double error;
  if (path_.isLastWayPoint()) {
    behaviour_ = APPROACH_SUBPATH_END;
    error = getErrorApproachSubpathEnd();
  } else {
    behaviour_ = ON_PATH;
    error = getErrorOnPath();
  }

  updateCommand(error);
  cmd = cmd_;

  return MoveCommand::MoveCommandStatus::OKAY;
}

void PIDLocalController::updateCommand(double error) {
  // call PID controller for steering.
  double u = 0;
  if (!steer_pid_.execute(error, &u)) {
    return;  // Nothing to do
  }

  double steer = std::max(-max_steer_param_, std::min(u, max_steer_param_));

  LOG(INFO) << "direction = " << dir_sign_ << ", steer = " << steer;
  LOG(INFO) << "PID: error = " << error << "   u = " << u;

  // Control velocity
  double velocity = controlVelocity(steer);

  cmd_.setDirection(dir_sign_ * steer);
  cmd_.setVelX(dir_sign_ * velocity);
  cmd_.setVelTH(dir_sign_ * velocity * tan(cmd_.getDirectionAngle()) / 0.38);
  last_velocity_ = dir_sign_ * velocity;

  LOG(INFO) << "velocity = " << velocity << " with direction: " << dir_sign_;
  LOG(INFO) << "setDirection : " << dir_sign_ * steer;
  LOG(INFO) << "setVelX : " << dir_sign_ * velocity;
  LOG(INFO) << "getDirection : " << cmd_.getDirectionAngle();
}

double PIDLocalController::controlVelocity(const double &steer_angle) {
  double velocity = limits_->getMaxTransVel();
  LOG(INFO) << "controlVelocity velocity = " << velocity;

  if (abs(steer_angle) > steer_slow_threshold_param_) {
    LOG(INFO) << "Slowing Down.";
    velocity *= 0.75;
  }
  // Reduce maximal velocity, when driving backwards.
  if (dir_sign_ < 0) {
    velocity = std::min(velocity, 0.4f * limits_->getMaxTransVel());
  }
  LOG(INFO) << "controlVelocity velocity = " << velocity;

  // linearly reduce velocity, if the goal is within 2s*velocity (e.g. when
  // driving with
  // 2 m/s, start to slow down 4m in front of the goal)
  // path_.getRemainingSubPathDistance() only returns the distance starting
  // from the next
  // waypoint, so add the distance of the robot to this waypoint to get a more
  // precise result.
  double distance_to_next_wp = std::sqrt(next_wp_local_.dot(next_wp_local_));
  double dist_to_path_end =
      path_.getRemainingSubPathDistance() + distance_to_next_wp;

  LOG(INFO) << "dist_to_path_end = " << dist_to_path_end;
  LOG(INFO) << "distance_to_next_wp = " << distance_to_next_wp;
  LOG(INFO) << "getWayPointIndex = " << path_.getWayPointIndex();

  if (dist_to_path_end < 2 * velocity) {
    velocity = std::max(0.1f + dist_to_path_end / 2.0f,
                        limits_->getMinVelocityThreshold());
  }

  LOG(INFO) << "calc velocity = " << velocity;
  // 保证计算的速度在机器人可执行范围内
  velocity = std::min(velocity, limits_->getMaxTransVel());
  velocity = std::max(velocity, limits_->getMinVelocityThreshold());
  LOG(INFO) << "return velocity = " << velocity;
  return velocity;
}

void PIDLocalController::selectWaypoint() {
  // increase tolerance, when driving backwards
  double distance_tolerance = tolerance_;
  if (getDirSign() < 0) {
    distance_tolerance *= back_multiple_;
  }
  // switch to the nearest waypoint, that is at least 'tolerance' far away.
  while (!path_.isLastWayPoint() &&
         distanceToWaypoint(path_.getCurrentWayPoint()) < distance_tolerance) {
    path_.switchToNextWayPoint();
  }

  cmd_.setCurrentWPPose(path_.getCurrentWayPoint());

  // convert waypoint to local frame. NOTE: This has to be done, even if the
  // waypoint did not
  // change, as its position in the local frame changes while the robot moves.
  Pose2d current_wp = path_.getCurrentWayPoint();
  eigenTransFormToLocal(current_wp, next_wp_local_);
}

double PIDLocalController::distanceToWaypoint(const Pose2d &wp) {
  Pose2d cur_pose = getCurrentPose();
  return cur_pose.distanceTo(wp);
}

void PIDLocalController::predictPose(Eigen::Vector2d &front_pred,
                                     Eigen::Vector2d &rear_pred) {
  // NOTE: This is an ancient relict of the days of the `motion_control`
  // package.
  // I am not absolutely sure, what it is doing. I think it has the purpose to
  // predict the
  // positon of the robot (more exactly its front and rear axes) in the next
  // time step.
  // This is how it is used at least, though I do not know if this is what it
  // really does...
  // ~Felix

  double dt =
      0.1;  // opt_.dead_time(); //TODO: could opt_.pid_ta() be used instead?
  double deltaf = cmd_.getDirectionAngle();
  double deltar = 0.0;            // currently not supported
  double v = 2 * last_velocity_;  // why '2*'?

  double beta = std::atan(0.5 * (std::tan(deltaf) + std::tan(deltar)));
  double ds = v * dt;
  double dtheta = ds * std::cos(beta) * (std::tan(deltaf) - std::tan(deltar)) /
                  car_length_params_;
  double thetan = dtheta;  // <- why this ???
  double yn = ds * std::sin(dtheta * 0.5 + beta * 0.5);
  double xn = ds * std::cos(dtheta * 0.5 + beta * 0.5);

  front_pred[0] = xn + cos(thetan) * car_length_params_ / 2.0;
  front_pred[1] = yn + sin(thetan) * car_length_params_ / 2.0;
  rear_pred[0] = xn - cos(thetan) * car_length_params_ / 2.0;
  rear_pred[1] = yn - sin(thetan) * car_length_params_ / 2.0;

  LOG(INFO) << "Predict pose. Front: (" << front_pred[0] << ", "
            << front_pred[1] << ")  Rear: (" << rear_pred[0] << ", "
            << rear_pred[1] << ")";
}

double PIDLocalController::getErrorOnPath() {
  /* The error is the sum of the orientation angle error and the distance to the
   * line that
   * goes through the next waypoints.
   * The velocity is set to the maximum.
   */

  // Calculate target line from current to next waypoint (if there is any)
  double e_distance = calculateLineError();
  double e_angle = calculateAngleError();

  // TODO: summing entities with different units (metric and angular) is
  // probably bad.
  double error = e_distance + e_angle;
  LOG(INFO) << "OnLine: e_dist = " << e_distance << "  e_angle = " << e_angle
            << "  e_comb = " << error;

  return error;
}

double PIDLocalController::getErrorApproachSubpathEnd() {
  /* The error is the sum of the orientation angle error and the sideways
   * distance to the
   * waypoint (that is the distance on the y-axis in the robot frame)
   * The velocity is decreasing with the distance to the waypoint.
   */

  // Calculate target line from current to next waypoint (if there is any)
  double e_distance = calculateSidewaysDistanceError();
  double e_angle = calculateAngleError();

  // TODO: summing entities with different units (metric and angular) is
  // probably bad.
  double error = e_distance + e_angle;
  LOG(INFO) << "Approach: e_dist = " << e_distance << "  e_angle = " << e_angle
            << "  e_comb = " << error;

  return error;
}

double PIDLocalController::calculateLineError() {
  Pose2d followup_next_wp_map;

  if (path_.getWayPointIndex() + 1 == path_.getCurrentSubPath()->size()) {
    followup_next_wp_map = path_.getWayPoint(path_.getWayPointIndex() - 1);
  } else {
    followup_next_wp_map = path_.getWayPoint(path_.getWayPointIndex() + 1);
  }

  Line2d target_line;
  Eigen::Vector3d followup_next_wp_local;
  eigenTransFormToLocal(followup_next_wp_map, followup_next_wp_local);

  target_line =
      Line2d(next_wp_local_.head<2>(), followup_next_wp_local.head<2>());

  Eigen::Vector2d main_carrot, alt_carrot, front_pred, rear_pred;

  predictPose(front_pred, rear_pred);
  if (dir_sign_ >= 0) {
    main_carrot = front_pred;
    alt_carrot = rear_pred;
  } else {
    main_carrot = rear_pred;
    alt_carrot = front_pred;
  }

  // 将此时计算的跟踪点，从局部坐标系转换到世界坐标系，用于rviz显示使用
  Pose2d local_carrot(main_carrot.x(), main_carrot.y(), 0.0);
  Pose2d global_carrot;
  transformToGlobal(local_carrot, global_carrot);
  cmd_.setCarrotPose(global_carrot);

  return -target_line.GetSignedDistance(main_carrot) -
         0.25 * target_line.GetSignedDistance(alt_carrot);
}

double PIDLocalController::calculateSidewaysDistanceError() {
  const double tolerance = 0.1;
  Eigen::Vector2d main_carrot, alt_carrot, front_pred, rear_pred;

  predictPose(front_pred, rear_pred);
  if (dir_sign_ >= 0) {
    main_carrot = front_pred;
    alt_carrot = rear_pred;
  } else {
    main_carrot = rear_pred;
    alt_carrot = front_pred;
  }

  // 将此时计算的跟踪点，从局部坐标系转换到世界坐标系，用于rviz显示使用
  Pose2d local_carrot(main_carrot.x(), main_carrot.y(), 0.0);
  Pose2d global_carrot;
  transformToGlobal(local_carrot, global_carrot);
  cmd_.setCarrotPose(global_carrot);

  double dist_on_y_axis = next_wp_local_[1] - main_carrot[1];
  if (std::abs(dist_on_y_axis) < tolerance) {
    return 0;
  } else {
    return dist_on_y_axis;
  }
}

double PIDLocalController::calculateAngleError() {
  Pose2d cur_pose = getCurrentPose();
  Pose2d wp = path_.getCurrentWayPoint();
  return AngleCalculate::angle_diff(wp.getYaw(), cur_pose.getYaw());
}

void PIDLocalController::eigenTransFormToLocal(const Pose2d &global_pose,
                                               Eigen::Vector3d &local_pose) {
  Pose2d local;
  transformToLocal(global_pose, local);
  local_pose.x() = local.getX();
  local_pose.y() = local.getY();
  local_pose.z() = local.getYaw();
}

}  // namespace CVTE_BABOT
