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
#ifndef ROUND_PATH_RECOVERY_HPP_
#define ROUND_PATH_RECOVERY_HPP_
#include <math.h>
#include <string>
#include "base_path_recovery.hpp"
#include <type.hpp>
#include "pose2d/pose2d.hpp"

namespace CVTE_BABOT {
class RoundPathRecovery : public BasePathRecovery {
 public:
  RoundPathRecovery();
  ~RoundPathRecovery() = default;

  RoundPathRecovery &operator=(const RoundPathRecovery &obj) = delete;
  RoundPathRecovery(const RoundPathRecovery &obj) = delete;

 public:
  /**
   *recoverClear
   *@brief
   *清空临时变量
   *
   **/
  void recoverClear() final;

  /**
   *changeStateTo
   *@brief
   *切换状态
   *
   *@param[in] traget_state 目标状态
   **/
  void changeStateTo(RecoveryState traget_state) final;

  /**
   *checkIfNeedToRecover
   *@brief
   *判断是否需要恢复，脱离障碍物
   *
   *@return true-是, false-否
   **/
  bool checkIfNeedToRecover() final;

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
    max_recover_distance_config_ = max_recover_dis;
  }

 private:
  double recover_v_ = 0.4;             // < 后退速度
  double recover_w_ = 0.6;             // < 旋转速度
  Pose2d recover_start_pose_;          // < 开始移动恢复时的位置
  Pose2d recover_start_temp_pose_;     // < 开始移动恢复时的位置
  double max_recover_distance_ = 1.0;  // < 一次恢复的最大移动距离
  double max_recover_distance_config_ = 1.0;  // < 一次恢复的最大移动距离

  bool easy_recover_flag = false;

  RecoveryState last_state_ = RecoveryState::GO_DEFAULT;
  RecoveryState state_ = RecoveryState::GO_DEFAULT;
  RecoveryState last_rotate_state_ = RecoveryState::GO_DEFAULT;
};

}  // namespace CVTE_BABOT
#endif
