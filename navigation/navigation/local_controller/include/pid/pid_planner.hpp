/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file pid_planner.hpp
 *
 *@brief PID 算法的头文件
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2020-01-06
 ************************************************************************/

#ifndef __PID_PLANNER_HPP
#define __PID_PLANNER_HPP

#include "local_controller_base.hpp"
#include "pidcontroller.hpp"

namespace CVTE_BABOT {
class Costmap2d;
class PIDLocalController : public LocalController {
 public:
  PIDLocalController(const PIDLocalController &obj) = delete;
  PIDLocalController &operator=(const PIDLocalController &obj) = delete;
  ~PIDLocalController() = default;

  PIDLocalController(std::shared_ptr<Costmap2d> costmap);

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
   * updateCostMap
   * @brief
   *   设置costmap
   *
   * @param[in] ptr_costmap-cost_map指针
   * */
  void updateCostMap(std::shared_ptr<Costmap2d> ptr_costmap) override;

 private:
  /**
   * getParams
   * @brief
   *   读取参数
   * */
  void getParams();

  /**
   * selectWaypoint
   * @brief
   *   根据速度、角度和寻找策略等因素，从任务路径中选择需要跟踪的点
   * */
  void selectWaypoint();

  /**
   * getErrorOnPath
   * @brief
   *   计算路径相关的误差函数
   *
   * @return 返回的误差 包含朝向角偏差 与 通过下一个跟踪点的直线距离
   * */
  double getErrorOnPath();

  /**
   * getErrorApproachSubpathEnd
   * @brief
   *   计算快到子路经终点的误差函数
   *
   * @return 返回的误差 包含朝向角偏差 与 机器人坐标系下y轴的偏移
   * */
  double getErrorApproachSubpathEnd();

  /**
   * updateCommand
   * @brief
   *   根据当前误差值，通过计算后（controlVelocity函数）结果更新控制明林
   *
   * @param[in] error-路径误差
   * */
  void updateCommand(double error);

  /**
   * controlVelocity
   * @brief
   *   根据当前误差角度，计算接下来执行的速度指令
   *   里面包含对最终速度值的判断，需要在机器人可执行的安全范围内
   *
   * @param[in] steer_angle-PID计算的角度偏差
   * @return 返回控制的速度值，但不包含方向
   * */
  double controlVelocity(const double &steer_angle);

  /**
   * distanceToWaypoint
   * @brief
   *   计算机器人与路径点的直线距离
   *
   * @param[in] wp-需要计算的路径点坐标
   * @return 返回两点之间的直线距离
   * */
  double distanceToWaypoint(const Pose2d &wp);

  /**
   * predictPose
   * @brief
   *   预测下一时刻，需要跟踪的前后两个点
   *
   * @param[out] front_pred-机器人前方的一个跟踪点
   * @param[out] rear_pred-机器人后方的一个跟踪点
   * */
  void predictPose(Eigen::Vector2d &front_pred, Eigen::Vector2d &rear_pred);

  /**
   * calculateLineError
   * @brief
   *   预测的前后两个点之间，可以得到一条直线。
   *   然后计算机器人位置与该直线之间的偏差
   *
   * @return 直线之间的偏差
   * */
  double calculateLineError();

  /**
   * calculateSidewaysDistanceError
   * @brief
   *   计算左右两侧偏移的误差
   *
   * @return 偏移误差
   * */
  double calculateSidewaysDistanceError();

  /**
   * calculateAngleError
   * @brief
   *   计算角度误差
   *
   * @return 角度误差
   * */
  double calculateAngleError();

  /**
   * eigenTransFormToLocal
   * @brief
   *   全局到局部的坐标转化，输出是Eigen的Vector
   *
   * */
  void eigenTransFormToLocal(const Pose2d &global_pose,
                             Eigen::Vector3d &local_pose);

  /**
   * \brief The current behaviour of the controller.
   *
   * ON_PATH: The robot is driving on a sub path. This is the default case.
   * APPROACH_SUBPATH_END: The robot is approaching the last waypoint of a sub
   * path.
   * WAIT_FOR_STOP: The robot reached the end of a sub path and is now waiting
   * until motion
   *     stopped (i.e. measured velocity is below some tolerance threshold).
   */
  enum { ON_PATH, APPROACH_SUBPATH_END, WAIT_FOR_STOP } behaviour_;

  //! The current move command.
  MoveCommand cmd_;

  //! PID controller for the steering angle
  PidController<1> steer_pid_;
  double last_velocity_ = 0.0;

  double pid_kp_ = 1.5;    ///< pid参数： p值
  double pid_ki_ = 0.001;  ///< pid参数： i值
  double pid_kd_ = 0.0;    ///< pid参数： d值
  double pid_t_ = 0.03;    ///< pid参数： 单位时间
  double tolerance_ = 1.5;  ///< 选择跟踪点的直线容忍值，倒车时会翻倍
  double back_multiple_ = 1.0;       ///< 倒车选择目标点的倍数
  double max_steer_param_ = 1.0;     ///< 最大允许转弯角度（弧度）
  double car_length_params_ = 0.38;  ///< 前轮和后轮之间的长度
  double steer_slow_threshold_param_ =
      0.25;  ///< 当偏差角度大于这个值时（弧度），降低运行速度

};  // end of class

}  // end of namespace

#endif  // end of __PID_PLANNER_HPP
