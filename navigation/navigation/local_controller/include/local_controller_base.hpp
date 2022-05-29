/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file local_controller_base.hpp
 *
 *@brief 控制算法的基类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#ifndef __LOCAL_CONTROLLER_BASE_HPP
#define __LOCAL_CONTROLLER_BASE_HPP

#include "controller_ultis/controller_limits.hpp"
#include "controller_ultis/move_command.hpp"
#include "navigation_mediator.hpp"
#include "path.hpp"
#include "planner_utils.hpp"

#include <glog/logging.h>
#include <eigen3/Eigen/Core>
#include <memory>
#include <mutex>
#include <pose2d/pose2d.hpp>
#include <vector>

namespace CVTE_BABOT {

class Costmap2d;
class MoveCommand;
struct TrackCtrl {
  double v_velocity;
  double w_velocity;
  std::string msg;
};
/**
 * Abstract
 * @brief
 *   控制算法的基类
 * */
class LocalController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalController(const LocalController &obj) = delete;
  LocalController &operator=(const LocalController &obj) = delete;
  virtual ~LocalController(){};

  LocalController() {
    limits_ = std::make_shared<ControllerLimits>();
    // path_ = std::make_shared<Path>();
    ptr_navigation_mediator_ = NavigationMediator::getPtrInstance();
  }

  /**
   * setCurrentPose
   * @brief
   *   输入当前机器人全局位置，更新给局部控制器
   *
   * @param[in] pose-机器人当前位置
   * */
  inline void setCurrentPose(const Pose2d &pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    robot_pose_ = pose;
  }

  /**
   * getCurrentPose
   * @brief
   *   获取机器人当前全局位置
   *
   * @return 机器人当前全局位置（地图坐标系下）
   * */
  inline Pose2d getCurrentPose() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    Pose2d pose = robot_pose_;
    return pose;
  }

  /**
   * setCurrentVelocity
   * @brief
   *   输入当前机器人速度值，更新给局部控制器
   *
   * @param[in] current_velocity-机器人当前速度信息结构体
   * */
  inline void setCurrentVelocity(const Velocity &current_velocity) {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    current_velocity_ = current_velocity;
  }

  /**
   * setCurrentVelocity
   * @brief
   *   输入当前机器人速度值，更新给局部控制器
   *
   * @param[in] current_velocity-机器人当前速度信息结构体
   * */
  inline Velocity getCurrentVelocity() {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    Velocity current_vel = current_velocity_;
    return current_vel;
  }

  /**
   * setPath
   * @brief
   *   设置任务路径
   *
   * */
  virtual bool setPath(std::shared_ptr<SubPath> path, size_t begin_index) {
    // if (path.wps.size() == 0) {
    //   LOG(ERROR) << "No sub Paths.";
    //   return false;
    // }
    // path_.clear();
    // path_ = path;
    // LOG(INFO) << "set subPath size = " << path_.subPathCount();

    // Pose2d current_wp = path_.getCurrentWayPoint();
    // Pose2d next_wp_pose2d_lc_;
    // transformToLocal(current_wp, next_wp_pose2d_lc_);
    // next_wp_local_.x() = next_wp_pose2d_lc_.getX();
    // next_wp_local_.y() = next_wp_pose2d_lc_.getY();
    // next_wp_local_.z() = next_wp_pose2d_lc_.getYaw();
    return true;
  }

  virtual size_t getReferIndex() { return 0; }

  /**
   * transformToLocal
   * @brief
   *   坐标转化，全局坐标系转机器人标系
   *
   * @param[in] map_point_pose-世界坐标系下的点
   * @param[out] local-机器人坐标系下的位置结果
   * */
  bool transformToLocal(const Pose2d &global_pose, Pose2d &local_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    // 将路径点从世界坐标系转到机器人坐标系，通过Pose2d 的 减法方法
    local_pose = global_pose - robot_pose_;
    return true;
  }

  /**
   * transformToGlobal
   * @brief
   *   坐标转化，机器人坐标系转全局坐标系
   *
   * @param[in] local_pose-机器人坐标系下的点
   * @param[out] global_pose-世界坐标系下的位置结果
   * */
  bool transformToGlobal(const Pose2d &local_pose, Pose2d &global_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    global_pose = robot_pose_ * local_pose;
    return true;
  }

  /**
   * setDirSign
   * @brief
   *   设置方向运动方向。
   *
   * @param[in] s = (+1) 代表前进； (-1) 代表后退
   * */
  virtual void setDirSign(float s) { dir_sign_ = s; }

  /**
   * getDirSign
   * @brief
   *   获取方向运动方向。
   *
   * @return (+1) 代表前进； (-1) 代表后退
   * */
  virtual float getDirSign() const { return dir_sign_; }

  /**
   * startController
   * @brief
   *   打开控制器
   * */
  virtual inline void startController() {}

  /**
   * stopController
   * @brief
   *   关闭控制器
   * */
  virtual inline void stopController() {}

  /**
   * updateCostMap
   * @brief
   *   设置代价地图
   *
   * @param[in] costmap_ptr-costmap指针
   * */
  virtual void updateCostMap(std::shared_ptr<Costmap2d>){};

  /**
   * setLimits
   * @brief
   *   设置速度限制的参数
   *
   * @param[in] config-速度参数
   * */
  virtual inline void setLimits(std::shared_ptr<ControllerLimits> config) {
    std::lock_guard<std::mutex> lock(limit_mutex_);
    limits_ = config;
  }

  /**
   * setMoveSpeedRange
   * @brief
   *   设置控制速度的范围
   *
   * @param[in] min_xv-最小X线速度
   * @param[in] max_xv-最大X线速度
   * @param[in] min_yv-最小Y线速度
   * @param[in] max_yv-最大Y线速度
   * @param[in] min_thv-最小角速度
   * @param[in] max_thv-最大角速度
   * */
  virtual void setMoveSpeedRange(const double &min_xv, const double &max_xv,
                                 const double &min_yv, const double &max_yv,
                                 const double &min_thv, const double &max_thv) {
    std::lock_guard<std::mutex> lock(limit_mutex_);
    limits_->setMaxTransVel(max_xv);
    limits_->setMaxVelX(max_xv);
    limits_->setMaxVelY(max_yv);
    limits_->setMaxRot(max_thv);

    limits_->setMinTransVel(min_xv);
    limits_->setMinVelX(min_xv);
    limits_->setMinVelY(min_yv);
    limits_->setMinRot(min_thv);
  }

  /**
   * getMaxSpeedLimit
   * @brief
   *   获取当前允许最大速度
   *
   * @param[out] max_xv-最大X线速度
   * @param[out] max_yv-最大Y线速度
   * @param[out] max_thv-最大角速度
   * */
  void getMaxSpeedLimit(double &max_xv, double &max_yv, double &max_th) {
    std::lock_guard<std::mutex> lock(limit_mutex_);
    max_xv = limits_->getMaxVelX();
    max_yv = limits_->getMaxVelY();
    max_th = limits_->getMaxRot();
  }

  /**
   * setMoveAccLimit
   * @brief
   *   设置控制的加速度
   *
   * @param[in] acc_x - X方向线加速度
   * @param[in] acc_y - y方向线加速度
   * @param[in] acc_th - 角加速度
   * */
  virtual void setMoveAccLimit(const double &acc_x, const double &acc_y,
                               const double &acc_th) {
    std::lock_guard<std::mutex> lock(limit_mutex_);
    limits_->setACCLimitX(acc_x);
    limits_->setACCLimitY(acc_y);
    limits_->setACCLimitTH(acc_th);
    LOG(INFO) << "acc x" << acc_x << "   " << acc_y << "      " << acc_th;
  }

  /**
   * computeMoveComand
   * @brief
   *   控制器根据当前位置和实时速度反馈，计算跟踪路径过程中需要执行的线速度和角速度
   *   计算之前确保已经将当前位置和速度通过setCurrentPose 和 setCurrentVelocity
   * 输入。
   *
   * @param[out] result_v-计算得到需要执行的线速度
   * @param[out] result_w-计算得到需要执行的角速度
   * */
  virtual MoveCommand::MoveCommandStatus computeMoveComand(
      MoveCommand &cmd) = 0;

  virtual void setGlobalGoal(const Pose2d &goal) { global_goal_ = goal; }
  virtual Pose2d getGlobalGoal() { return global_goal_; }
  virtual double getRemainCtrlPathLength() {}

 protected:
  std::shared_ptr<SubPath> path_;  ///< 任务路径
  Pose2d robot_pose_;              ///< 机器人在地图的位置
  Velocity current_velocity_;      ///< 机器人当前速度
  std::shared_ptr<ControllerLimits> limits_ = nullptr;  ///< 速度类的上限类
  std::shared_ptr<NavigationMediator> ptr_navigation_mediator_ =
      nullptr;  ///< 传参

  std::mutex vel_mutex_;    ///< 速度变量锁
  std::mutex pose_mutex_;   ///< 位置变量锁
  std::mutex limit_mutex_;  ///< 速度上限类变量锁

  ///< GeRoNa 框架里面算法的公用变量
  //! Indicates the direction of movement (>0 -> forward, <0 -> backward)
  float dir_sign_ = 0.0;
  //! The next waypoint in the robot frame (set by setPath).
  Eigen::Vector3d next_wp_local_;

  Pose2d global_goal_;
};
}  // namespace CVTE_BABOT

#endif  // end of __LOCAL_CONTROLLER_BASE_HPP
