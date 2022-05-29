/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file edge_controller.hpp
 *
 *@brief 贴边控制算法的头文件   控制器 --> lypu
 *
 *@modified by
 *
 *@author chenweijian(chenweijian@cvte.com)
 *@version Navigation-v2.0
 *@data 2021-12-02
 ************************************************************************/
#ifndef EDGE_CONTROLLER_HPP_
#define EDGE_CONTROLLER_HPP_
#include <string>
#include <mutex>
#include <time.h>
#include "local_controller_base.hpp"
#include "pose2d/pose2d.hpp"
#include "lypu/lypu_planner.hpp"
#include "stanley/stanley_controller.hpp"

namespace CVTE_BABOT {
// struct TrackCtrl {
//   double v_velocity;
//   double w_velocity;
//   std::string msg;
// };
class EdgeController : public LocalController {
 public:
  EdgeController();
  EdgeController(const EdgeController &obj) = delete;
  EdgeController &operator=(const EdgeController &obj) = delete;
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
      MoveCommand &cmd) final;
  /**
   * setPath
   * @brief
   *   设置新的全局路径
   *
   * @param[in] orig_global_plan-需要传入给DWA的新全局路径
   * @return true-代表设置成功
   * */
  virtual bool setPath(std::shared_ptr<SubPath> path,
                       size_t begin_index) override final;

  virtual double getRemainCtrlPathLength() override final;

  virtual size_t getReferIndex() override final;

 private:
  /**
   * @brief 读取控制参数
   *
   */
  void getParams();

  void trackingStanley(const Pose2d &traget_pos, const Pose2d &current_pos,
                       const double &target_v, const double &target_w,
                       TrackCtrl &result);
  /**
   * findTrackTarget
   * @brief
   *   从任务路径上查找离机器人最近的目标点，查找方式：
   *       寻找路径上 与机器人位置直线距离最近点 的下一点
   *
   * @param[in] c_pose-机器人当前位置
   * @param[out] path_point_index-计算得到的跟踪目标点
   * @param[in] last_point_index-上一次的起点
   * @param[in] front_distance-前瞻距离
   * @return true-正常找到下一个跟踪点
   * */
  bool findTrackTarget(const Pose2d &c_pose, size_t &path_point_index,
                       size_t &last_point_index, double front_distance = 0.10,
                       double sum_dist = 1.0);
  /**
   * changeCtrl
   * @brief
   *   把控制量从速度,转向角度转为角速度,速度
   *
   *
   * @param[in] 输入线速度
   * @param[in] 输入转角
   * @param[out] TrackCtrl 类型的输出
   * */
  TrackCtrl changeCtrlOutput(double vel, double steer_angle_);
  /**
   * changeCtrl
   * @brief
   *   把控制量从角速度转为转角
   *
   * @param[in] 输入线速度
   * @param[out] 转角
   * */
  double changeToSteerAngle(double vel, double w);

  double checkLimitSteerAngle(double steer_angle) const;

  void checkMaxVel(double &v_vel, double &w_vel) const;

  void checkMinVel(double &v_vel, double &w_vel) const;

  double last_steer_angle_;
  double u_steer_angle_;
  double l_steer_angle_;
  double kd_yaw_;
  double kp_e_;
  double kp_soft_;
  double kd_steer_;
  double kag_;
  double kp_v_;

  double car_length_;
  double max_vel_;
  double max_w_;
  double min_vel_;
  double min_w_;
  std::string edge_sensor_ = "";

  CtrlMode last_ctrl_mode_;

  std::shared_ptr<SubPath> local_path_ = nullptr;
  std::shared_ptr<LYPULocalController> lypu_ptr_ = nullptr;

  std::mutex path_mutex_;
  std::mutex steer_angle_mutex_;
  size_t last_target_index_;

  time_t last_close_time_ = 0;
  bool very_close_flag_;
};
}  // namespace CVTE_BABOT
#endif