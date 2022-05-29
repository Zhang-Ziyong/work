/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath:
 /src/navigation/navigation/local_controller/include/mpc/mpc_controller.hpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2021-01-12 09:42:57
 ************************************************************************/
#pragma once

#include "local_controller_base.hpp"
#include "pose2d/pose2d.hpp"
#include "mpc_data_type.h"
#include "mpc_osqp.h"

namespace CVTE_BABOT {

class MPCLocalController : public LocalController {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  MPCLocalController(const MPCLocalController &obj) = delete;
  MPCLocalController &operator=(const MPCLocalController &obj) = delete;
  ~MPCLocalController() = default;

  MPCLocalController();

  /**
   * computeMoveComand
   * @brief
   *   控制器根据当前位置和实时速度反馈，计算跟踪路径过程中需要执行的线速度和角速度
   *   计算之前确保已经将当前位置和速度通过setCurrentPose 和 setCurrentVelocity
   * 输入。
   *
   * @param[out] cmd_ptr-计算指令的集合
   * */
  virtual MoveCommand::MoveCommandStatus computeMoveComand(
      MoveCommand &cmd) override final;

  /**
   * setPath
   * @brief
   *   设置任务路径
   *
   * */
  virtual bool setPath(Path path) override final;
  /**
   * isGoalReached
   * @brief
   *   计算机器人当前位置与目标的距离，判断是否已经到达
   * @param[in] current_pose-机器人的当前位置
   * */
  bool isGoalReached(const Pose2d &current_pose);

  void initParams();

  void getParams();

 private:
  std::vector<MpcState> getTargetWaypoints(const Pose2d &pose,
                                           const Velocity &velocity);

 private:
  //状态相关变量
  MpcState current_state_;
  // mpcc::State mpcc_state_;
  std::vector<Pose2d> sim_waypoints_;

  Pose2d goal_;  ///< 目标点（设置任务路径中的最后一点）>
  // x , y , heading
  size_t state_dim_ = 3;

  size_t controls_dim_ = 2;

  int horizon_ = 15;

  size_t last_target_index_ = 0;

  double controller_frequency_ = 10.;

  bool is_state_updated_ = false;

  size_t sim_counter = 0;
  std::mutex goal_mutex_;  ///< 获取与设置目标点锁
  std::mutex path_mutex_;  ///< 路径更新锁

  std::vector<double> matrix_q_prams_;
  std::vector<double> matrix_r_prams_;
  std::vector<double> u_lower_prams_;
  std::vector<double> u_upper_prams_;
  std::vector<double> u_bound_prams_;
  double kappa_limit_cofficient_ = 0.2;
  double min_velocity_ = 0.2;

  // mpc相关变量
  Eigen::MatrixXd matrix_q_;
  Eigen::MatrixXd matrix_r_;
  Eigen::MatrixXd matrix_initial_x_;
  Eigen::MatrixXd matrix_u_lower_;
  Eigen::MatrixXd matrix_u_upper_;
  Eigen::MatrixXd matrix_x_lower_;
  Eigen::MatrixXd matrix_x_upper_;
};  // end class

}  // namespace CVTE_BABOT
