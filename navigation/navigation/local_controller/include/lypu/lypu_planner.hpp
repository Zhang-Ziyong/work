/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file lypu_planner.hpp
 *
 *@brief LYPU 算法的头文件
 *
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2020-07-16
 ************************************************************************/

#ifndef __LYPU_PLANNER_HPP
#define __LYPU_PLANNER_HPP

#include <fstream>
#include "local_controller_base.hpp"
#include "pose2d/pose2d.hpp"

#include "navigation_mediator.hpp"
namespace CVTE_BABOT {

class LYPULocalController : public LocalController {
 public:
  LYPULocalController(const LYPULocalController &obj) = delete;
  LYPULocalController &operator=(const LYPULocalController &obj) = delete;
  ~LYPULocalController() = default;

  LYPULocalController();

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
  virtual double getRemainCtrlPathLength() override final;

  virtual size_t getReferIndex() override final;
  /**
   * clearCache
   * @brief
   *   清空缓存数据
   *
   * */
  void clearCache();
  /**
   * trackingLYPU
   * @brief
   *   lypu位置渐进跟踪算法
   *
   * @param[in] traget_pos-需要跟踪的目标点
   * @param[in] current_pos-当前机器人的姿态
   * @param[in] target_v-跟踪期望的线速度
   * @param[in] target_w-跟踪期望的角速度
   * @param[out] result-计算得到的控制速度结果
   * @return true-计算成功
   * */
  bool trackingLYPU(const Pose2d &traget_pos, const Pose2d &current_pos,
                    const double &target_v, const double &target_w,
                    TrackCtrl &result);

 private:
  /**
   * findTrackTarget
   * @brief
   *   从任务路径上查找离机器人最近的目标点，查找方式：
   *       寻找路径上 与机器人位置直线距离最近点 的下一点
   *
   * @param[in] c_pose-机器人当前位置
   * @param[in] path-需要跟踪的任务路径
   * @param[out] next_target-计算得到的跟踪目标点
   * @return true-正常找到下一个跟踪点
   * */
  bool findTrackTarget(const Pose2d &c_pose, size_t &path_point_index);

  /**
   * checkMaxVel
   * @brief
   *   检查计算得到的结果，保证在安全运动范围内，避免计算错误直接下发导致意外
   *
   * @param[out] result-计算得到的速度结构体
   * */
  void checkMaxVel(double &v_vel, double &w_vel);

  /**
   * checkMinVel
   * @brief
   *   检查计算得到的结果，保证最小可执行速度
   *
   * @param[out] result-计算得到的速度结构体
   * */
  void checkMinVel(double &v_vel, double &w_vel);

  /**
   * setPath
   * @brief
   *   设置新的全局路径
   *
   * @param[in] orig_global_plan-需要传入给DWA的新全局路径
   * @return true-代表设置成功
   * */
  bool setPath(std::shared_ptr<SubPath> path, size_t begin_index) override;

  void getParams();

 private:
  Pose2d next_target_;  ///< 下一个目标跟踪点
  Pose2d goal_;         ///< 目标点（设置任务路径中的最后一点）

  // double dKx_ = 1;  ///< 控制器参数, X增益
  // double dKy_ = 3;  ///< 控制器参数, Y增益
  // double dKq_ = 1;  ///< 控制器参数, 角度增益
  double dKp_ = 1;
  double dKth_ = 0.5;
  double dKx_ = 1;  ///< 控制器参数, X增益
  double dKy_ = 3;  ///< 控制器参数, Y增益
  double dKq_ = 2;  ///< 控制器参数, 角度增益
  double dKd_ = 1.0;

  // FIXME: 角度大于90°时会有严重超调
  std::mutex goal_mutex_;  ///< 获取与设置目标点锁
  std::mutex path_mutex_;  ///< 获取与设置路径锁

  double max_target_v_ = 0.5;
  double max_target_w_ = 1.0;
  double min_target_v_ = 0.1;
  double min_target_w_ = 0.1;
  double max_v_ = 0.5;
  double max_y_ = 0.0;
  double max_w_ = 1.0;
  double min_v_ = 0.05;
  double min_w_ = 0.05;
  double acc_ = 0.5;
  double a_acc_ = 1.0;
  double inverse_dist_ = 0.3;

  double last_dQ_ = M_PI;
  bool init_flag_ = false;

  std::shared_ptr<SubPath> local_path_;
  size_t last_target_index_ = 0;
  bool compute_vel = true;
  std::vector<double> path_points_curve;
  std::shared_ptr<NavigationMediator> ptr_navigation_mediator_ =
      nullptr;  ///< 传参
};
}  // namespace CVTE_BABOT

#endif