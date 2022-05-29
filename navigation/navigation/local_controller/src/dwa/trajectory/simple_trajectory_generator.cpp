/************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2018, CVTE.
* All rights reserved.
*
*@file simple_trajectory_generator.hpp
*
*@author TKruse

*@modified caoyong (caoyong@cvte.com)
*@current_algo.dev.1.0
*@data 2019-04-11
************************************************************************/

#include "dwa/trajectory/simple_trajectory_generator.hpp"
#include "controller_ultis/controller_limits.hpp"
#include "dwa/iterator/velocity_iterator.hpp"
#include "poly5_trajectory.hpp"
#include "trajectory.hpp"

#include <glog/logging.h>
#include <cmath>
#include <memory>

namespace CVTE_BABOT {
void SimpleTrajectoryGenerator::initialise(
    const Pose2d &pos, const std::array<double, 3> &vel,
    const std::shared_ptr<ControllerLimits> limits,
    const std::array<int, 3> &vsamples, bool is_close_goal,
    bool discretize_by_time) {
  vel_ = vel;
  next_sample_index_ = 0;
  sample_params_.clear();
  discretize_by_time_ = discretize_by_time;
  pos_.setPose(pos.getX(), pos.getY(), pos.getYaw());

  acc_limit_ = limits->getAccLimits();

  max_vel_x_ = limits->getMaxVelX();
  min_vel_x_ = limits->getMinVelX();
  
  max_vel_y_ = limits->getMaxVelY();
  min_vel_y_ = limits->getMinVelY();
  
  max_vel_rot_ = limits->getMaxRot();
  min_vel_rot_ = limits->getMinRot();

  max_trans_vel_ = limits->getMaxTransVel();
  min_trans_vel_ = limits->getMinTransVel();

  // 采样长度需要根据最大速度来动态调整，保证前向采样路径在 sim_distance_ 米左右, t = S / max_v
  sim_time_ = sim_distance_ / max_vel_x_;
  LOG(INFO) << "sim_time_ : " << sim_time_;

  // 如果已经靠近目标点，则采取降速(slow_down_v_ m/s)和降低采样路径长度(0.3m/s × 2.0s)，提高停靠精度
  if (is_close_goal) {
    max_vel_x_ = slow_down_v_;
    min_vel_x_ = -slow_down_v_;
    max_vel_rot_ = slow_down_w_;
    sim_time_ = 2.0;
  }
  
  // if sampling number is zero in any dimension, we don't generate samples
  // generically
  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) {
    // compute the feasible velocity space based on the rate at which we run
    std::array<double, 3> max_vel;
    std::array<double, 3> min_vel;

    // with dwa do not accelerate beyond the first step, we only sample within
    // velocities we reach in sim_period
    max_vel[0] = std::min(max_vel_x_, vel[0] + acc_limit_[0] * sim_period_ * 5);
    max_vel[1] = std::min(max_vel_y_, vel[1] + acc_limit_[1] * sim_period_ * 5);
    max_vel[2] =
        std::min(max_vel_rot_, vel[2] + acc_limit_[2] * sim_period_ * 5);

    min_vel[0] = std::max(min_vel_x_, vel[0] - acc_limit_[0] * sim_period_ * 5);
    min_vel[1] = std::max(min_vel_y_, vel[1] - acc_limit_[1] * sim_period_ * 5);
    min_vel[2] =
        std::max(-1.0 * max_vel_rot_, vel[2] - acc_limit_[2] * sim_period_ * 5);

    std::array<double, 3> vel_samp;
    Pose2d samples;
    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
    for (; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for (; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();
        for (; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          samples.setPose(vel_samp[0], vel_samp[1], vel_samp[2]);
          sample_params_.push_back(samples);
        }
        th_it.reset();
      }
      y_it.reset();
    }
    x_it.reset();
  }
}

void SimpleTrajectoryGenerator::setParameters(
    const double &sim_time, const double &sim_granularity,
    const double &angular_sim_granularity, const double &sim_period,
    const double &sim_dis, const double &slow_v, const double &slow_w) {
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  sim_period_ = sim_period;
  sim_distance_ = sim_dis;
  slow_down_v_ = slow_v;
  slow_down_w_ = slow_w;
  continued_acceleration_ = true;
}

/**
 * Whether this generator can create more trajectories
 */
bool SimpleTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool SimpleTrajectoryGenerator::generateTrajectory(
    const Pose2d &pos, const std::array<double, 3> &vel,
    const std::array<double, 3> &sample_target_vel, Trajectory &traj) {
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
  bool use_p5 = false;
  double eps = 1e-4;
  traj.cost_ = -1.0;

  // trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();
  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  if ((min_trans_vel_ >= 0 && vmag + eps < min_trans_vel_) &&
      (min_vel_rot_ >= 0 && fabs(sample_target_vel[2]) + eps < min_vel_rot_)) {
    return false;
  }
  // 看是否要对整个trans速度加限制
  // if (max_trans_vel_ >= 0 && vmag - eps > max_trans_vel_) {
  //   return false;
  // }
  int num_steps = 0;
  if (discretize_by_time_) {
    num_steps = ceil(sim_time_ / sim_granularity_);
  } else {
    // compute the number of steps we must take along this trajectory to be
    // "safe"
    double sim_time_distance = vmag * sim_time_;
    // the distance the robot would
    // travel in sim_time if it did
    // not change velocity
    double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_;
    // the angle the robot would rotate in sim_time
    num_steps = ceil(std::max(sim_time_distance / sim_granularity_,
                              sim_time_angle / angular_sim_granularity_));
  }

  if (num_steps == 0) {
    LOG(ERROR) << "num_steps is zero" << std::endl;
    return false;
  }
  double dt = 0.0;
  std::array<double, 3> loop_vel;
  std::array<double, 3> pos_temp = {pos.getX(), pos.getY(), pos.getYaw()};

  if (use_p5) {
    dt = TIME_DELTA;
    traj.time_delta_ = TIME_DELTA;
    num_steps = ceil(sim_time_ / TIME_DELTA);
    loop_vel = sample_target_vel;
    traj.xv_ = loop_vel[0];
    traj.yv_ = loop_vel[1];
    traj.thetav_ = loop_vel[2];

    for (int i = 0; i < num_steps; ++i) {
      // 前五个速度值是匀加速，后面的值就平速
      if (i < 5) {
        loop_vel[0] = vel[0] + (sample_target_vel[0] - vel[0]) * (i + 1) / 5.0;
        loop_vel[2] = vel[2] + (sample_target_vel[2] - vel[2]) * (i + 1) / 5.0;
      } else {
        loop_vel[0] = sample_target_vel[0];
        loop_vel[2] = sample_target_vel[2];
      }
      pos_temp = computeNewPositions(pos_temp, loop_vel, dt);
      traj.addPoint(pos_temp[0], pos_temp[1], pos_temp[2], loop_vel[0],
                    loop_vel[1], loop_vel[2]);
    }
  } else {
    // assuming sample_vel is our target velocity within acc limits for one
    // timestep
    dt = sim_time_ / num_steps;
    traj.time_delta_ = dt;
    loop_vel = sample_target_vel;
    traj.xv_ = sample_target_vel[0];
    traj.yv_ = sample_target_vel[1];
    traj.thetav_ = sample_target_vel[2];
    for (int i = 0; i < num_steps; ++i) {
      if (continued_acceleration_) {
        loop_vel =
            computeNewVelocities(sample_target_vel, loop_vel, acc_limit_, dt);
      }
      pos_temp = computeNewPositions(pos_temp, loop_vel, dt);
      traj.addPoint(pos_temp[0], pos_temp[1], pos_temp[2], loop_vel[0],
                    loop_vel[1], loop_vel[2]);
    }
  }
  return true;
}

/**
*change vel using acceleration limits to converge towards sample_target-vel
*/
std::array<double, 3> SimpleTrajectoryGenerator::computeNewVelocities(
    const std::array<double, 3> &sample_target_vel,
    const std::array<double, 3> &vel, const std::array<double, 3> &acclimits,
    const double &dt) {
  std::array<double, 3> new_vel;
  for (int i = 0; i < 3; ++i) {
    if (vel[i] < sample_target_vel[i]) {
      new_vel[i] =
          std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
    } else {
      new_vel[i] =
          std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);
    }
  }
  return new_vel;
}

std::array<double, 3> SimpleTrajectoryGenerator::computeNewPositions(
    const std::array<double, 3> &pos, const std::array<double, 3> &vel,
    const double &dt) {
  double eps = 1e-2;
  double pose_x = pos[0];
  double pose_y = pos[1];
  double pose_theta = pos[2];
  double vel_x = vel[0];
  double vel_theta = vel[2];
  std::array<double, 3> new_pos;

  if (fabs(vel_theta) > eps) {
    new_pos[0] = pose_x - vel_x / vel_theta * sin(pose_theta) +
                 vel_x / vel_theta * sin(pose_theta + vel_theta * dt);
    new_pos[1] = pose_y + vel_x / vel_theta * cos(pose_theta) -
                 vel_x / vel_theta * cos(pose_theta + vel_theta * dt);
    new_pos[2] = pose_theta + vel_theta * dt;
  } else {
    new_pos[0] = pose_x + cos(pose_theta) * vel_x * dt;
    new_pos[1] = pose_y + sin(pose_theta) * vel_x * dt;
    new_pos[2] = pose_theta;
  }
  return new_pos;
}

/**
 * Create and return the next sample trajectory
 */
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
    std::array<double, 3> sample_params = {
        sample_params_[next_sample_index_].getX(),
        sample_params_[next_sample_index_].getY(),
        sample_params_[next_sample_index_].getYaw()};
    if (generateTrajectory(pos_, vel_, sample_params, comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

}  // namespace CVTE_BABOT
