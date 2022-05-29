/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file simple_trajectory_generator.hpp
 *
 *@brief 生成轨迹样本的类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __SIMPLE_TRAJECTORY_GENERATOR_HPP
#define __SIMPLE_TRAJECTORY_GENERATOR_HPP

#include <eigen3/Eigen/Core>
#include <memory>
#include <pose2d/pose2d.hpp>

namespace CVTE_BABOT {
static const double TIME_DELTA = 0.1;

class ControllerLimits;
class Trajectory;

/**
* SimpleTrajectoryGenerator
* @brief
*   生成轨迹样本的类
* */

class SimpleTrajectoryGenerator {
 public:
  friend class SimpleTrajectoryGeneratorTest;
  friend class SimpleScoredSamplingTest;
  SimpleTrajectoryGenerator() = default;
  ~SimpleTrajectoryGenerator() = default;

  SimpleTrajectoryGenerator(const SimpleTrajectoryGenerator &obj) = delete;
  SimpleTrajectoryGenerator &operator=(const SimpleTrajectoryGenerator &obj) =
      delete;

  /**
    * initialise
    * @brief
    *   初始化函数
    *
    * @param[in] pos-当前机器人位置
    * @param[in] vel-当前机器人速度
    * @param[in] limits-当前速度的限制值
    * @param[in] vsamples-在给定的维度的采样数量
    * @param[in] is_close_goal-是否已经到达目标点
    * @param[in] discretize_by_time-true的时候轨迹按等时间隔采样
    * */
  void initialise(const Pose2d &pos, const std::array<double, 3> &vel,
                  const std::shared_ptr<ControllerLimits> limits,
                  const std::array<int, 3> &vsamples, bool is_close_goal,
                  bool discretize_by_time = false);

  /**
    * setParameters
    * @brief
    *   设置参数
    *
    * @param[in] sim_time-采样时间
    * @param[in] sim_granularity-碰撞检测粒度
    * @param[in] angular_sim_granularity-角度碰撞检测粒度
    * @param[in] sim_period-轨迹内点的间隔
    * */
  void setParameters(const double &sim_time, const double &sim_granularity,
                     const double &angular_sim_granularity,
                     const double &sim_period, const double &sim_dis,
                     const double &slow_v, const double &slow_w);

  /**
    * hasMoreTrajectories
    * @brief
    *   生成器是否可以生成更多路径
    * @return true-代表能生成更多路径
    * */
  bool hasMoreTrajectories();

  /**
    * nextTrajectory
    * @brief
    *   创建并返回下一条路径
    *
    * @param[out] traj-生成的路径
    * @return true-代表成功生成一条路径
    * */
  bool nextTrajectory(Trajectory &traj);

  /**
    * computeNewPositions
    * @brief
    *   计算新的位置
    *
    * @param[in] pos-当前位置
    * @param[in] vel-当前速度
    * @param[in] dt-时间间隔
    * */
  std::array<double, 3> computeNewPositions(const std::array<double, 3> &pos,
                                            const std::array<double, 3> &vel,
                                            const double &dt);

  /**
    * computeNewVelocities
    * @brief
    *   计算新的速度
    *
    * @param[in] sample_target_vel-速度期望值
    * @param[in] vel-当前速度
    * @param[in] acclimits-加速度限制
    * @param[in] dt-时间间隔
    * */
  std::array<double, 3> computeNewVelocities(
      const std::array<double, 3> &sample_target_vel,
      const std::array<double, 3> &vel, const std::array<double, 3> &acclimits,
      const double &dt);

  /**
    * generateTrajectory
    * @brief
    *   计算新的速度
    *
    * @param[in] pos-当前位置
    * @param[in] vel-当前速度
    * @param[in] sample_target_vel-采样的期望速度
    * @param[in] dt-时间间隔
    * @param[out] traj-生成得到的轨迹
    * */
  bool generateTrajectory(const Pose2d &pos, const std::array<double, 3> &vel,
                          const std::array<double, 3> &sample_target_vel,
                          Trajectory &traj);

  /**
    * setGeneratorParams
    * @brief
    *   设置参数
    * */
  void setGeneratorParams(const double &sim_time, const double &sim_dis,
                          const double &slow_v, const double &slow_w);

 private:
  unsigned int next_sample_index_ = 0;  ///< 样本点的索引
  std::vector<Pose2d> sample_params_;  ///< 存储从初始到采样的每个样本值参数

  Pose2d pos_;  ///< 位置

  std::array<double, 3> vel_;        ///< 速度
  std::array<double, 3> acc_limit_;  ///< 加速度
  double min_vel_x_ = 0.0;
  double max_vel_x_ = 0.0;
  double min_vel_y_ = 0.0;
  double max_vel_y_ = 0.0;
  double min_vel_rot_ = 0.0;
  double max_vel_rot_ = 0.0;
  double min_trans_vel_ = 0.0;
  double max_trans_vel_ = 0.0;
  double slow_down_v_ = 0.0;
  double slow_down_w_ = 0.0;

  ///< whether velocity of trajectory changes over time or not
  bool continued_acceleration_;
  bool discretize_by_time_;

  double sim_time_ = 0.0;                 ///< 采样时间
  double sim_distance_ = 0.0;             ///< 采样距离
  double sim_granularity_ = 0.0;          ///< 采样间隔
  double angular_sim_granularity_ = 0.0;  ///< 角度间隔

  double sim_period_ = 0.0;  ///< only for dwa
};

}  // namespace CVTE_BABOT

#endif  // __SIMPLE_TRAJECTORY_GENERATOR_HPP