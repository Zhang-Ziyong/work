/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file path_recovery.hpp
 *
 *@brief 处理机器人停障恢复时的后退逻辑
 *
 *@author linyanlong
 *@modified
 *@version
 *@data
 ************************************************************************/
#ifndef FRONT_PATH_RECOVERY_HPP_
#define FRONT_PATH_RECOVERY_HPP_
#include <math.h>
#include <string>
#include "base_path_recovery.hpp"
#include <type.hpp>
#include "pose2d/pose2d.hpp"

namespace CVTE_BABOT {
class FrontPathRecovery : public BasePathRecovery {
 public:
  FrontPathRecovery();
  ~FrontPathRecovery() = default;

  FrontPathRecovery &operator=(const FrontPathRecovery &obj) = delete;
  FrontPathRecovery(const FrontPathRecovery &obj) = delete;

 public:
  /**
   *recoverClear
   *@brief
   *清空临时变量
   *
   **/
  void recoverClear() final;
  /**
   *checkIfNeedToRecover
   *@brief
   *判断是否需要恢复，脱离障碍物
   *
   *@return true-是, false-否
   **/
  bool checkIfNeedToRecover() final;

  /**
   *changeStateTo
   *@brief
   *切换状态
   *
   *@param[in] traget_state 目标状态
   **/
  void changeStateTo(RecoveryState traget_state) final;

  /**
   *calculateRecoverVelocity
   *@brief
   *计算恢复速度
   *
   *@param[out] recover_v-恢复用的线速度
   *@param[out] recover_w-恢复用的角速度
   *@return 计算结果
   **/
  RecoveryState calculateRecoverVelocity(double &recover_v,
                                         double &recover_w) final;

  /**
   *setRecoverVelocity
   *@brief
   *设置恢复速度大小
   *
   *@param[in] recover_v-恢复用的线速度
   *@param[in] recover_w-恢复用的角速度
   **/
  void setRecoverVelocity(const double &recover_v,
                          const double &recover_w) final {
    recover_v_ = recover_v;
    recover_w_ = recover_w;
  }

  void setMaxRecoverDistance(const double &max_recover_dis) final {
    max_recover_distance_ = max_recover_dis;
  }

 private:
  double recover_v_ = 0.2;             ///< 后退速度
  double recover_w_ = 0.15;            ///< 旋转速度
  Pose2d current_pose_;                ///<当前的位置
  Pose2d recover_start_trans_pose_;    ///<开始移动恢复时的位置
  Pose2d recover_start_rot_pose_;      ///<开始旋转恢复时的位置
  double max_recover_distance_ = 1.0;  ///<一次恢复的最大移动距离
  double max_recover_angle_ = 0.5236;  ///<一次恢复的最大旋转角度(30度)
  bool start_trans_recover_ = false;  ///<开始后退标志位
  bool start_rot_recover_ = false;    ///<开始旋转标志位
  RecoveryState state_ = RecoveryState::GO_DEFAULT;
};

}  // namespace CVTE_BABOT
#endif
