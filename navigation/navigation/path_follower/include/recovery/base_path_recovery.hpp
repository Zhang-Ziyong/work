/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file path_recovery.hpp
 *
 *@brief 机器人停障恢复基类
 *
 *@author linyanlong
 *@modified
 *@version
 *@data
 ************************************************************************/
#ifndef BASE_PATH_RECOVERY_HPP_
#define BASE_PATH_RECOVERY_HPP_
#include <type.hpp>
namespace CVTE_BABOT {
class BasePathRecovery {
 public:
  virtual ~BasePathRecovery() {}
  /**
   *recoverClear
   *@brief
   *清空临时变量
   *
   **/
  virtual void recoverClear() = 0;

  /**
   *changeStateTo
   *@brief
   *切换状态
   *
   *@param[int] traget_state 目标状态
   **/
  virtual void changeStateTo(RecoveryState traget_state) = 0;

  /**
   *checkIfNeedToRecover
   *@brief
   *判断是否需要恢复，脱离障碍物
   *
   *@return true-是, false-否
   **/
  virtual bool checkIfNeedToRecover() = 0;

  /**
   *calculateRecoverVelocity
   *@brief
   *计算恢复速度
   *
   *@param[out] recover_v-恢复用的线速度
   *@param[out] recover_w-恢复用的角速度
   *@return 计算结果
   **/
  virtual RecoveryState calculateRecoverVelocity(double &recover_v,
                                                 double &recover_w) = 0;

  /**
   *setRecoverVelocity
   *@brief
   *设置恢复速度大小
   *
   *@param[in] recover_v-恢复用的线速度
   *@param[in] recover_w-恢复用的角速度
   **/
  virtual void setRecoverVelocity(const double &recover_v,
                                  const double &recover_w) = 0;

  virtual void setMaxRecoverDistance(const double &max_recover_dis) = 0;
};
}  // namespace CVTE_BABOT

#endif