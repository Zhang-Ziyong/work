/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file angle_topath_supervisor.hpp
 *
 *@brief 实现获取车辆避让避让点的功能
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-27
 ************************************************************************/
#ifndef __ANGLE_TOPATH_SUPERVISOR_HPP
#define __ANGLE_TOPATH_SUPERVISOR_HPP

#include "supervisor.hpp"

namespace CVTE_BABOT {
/**
 * AngleTopathSupervisor
 * @brief
 * 1.实现车辆主动避让停靠点检测
 * 2.先采样,再依据制定的规则评分得到最优点
 **/
class AngleTopathSupervisor : public Supervisor {
 public:
  AngleTopathSupervisor(double max_angle_to_path);

  virtual std::string getName() const { return "AngleTopathSupervisor"; }

  /**
   *supervise
   *@brief
   *输入机器人状态,输出状态及目标点等
   *
   *@param[in] state-机器人的一些状态,作为数据传递
   *@param[in] out-包含状态,目标点等
   **/
  virtual void supervise(const SupervisorState &state, SupervisorResult &out);

 private:
  /**
   * @brief 计算当前路径偏移角度 (寻找前方 3米 内最小角度)
   *
   * @param state
   * @param out
   * @return double
   */
  double calculateAngleToCurrentPathSegment(const SupervisorState &state,
                                            SupervisorResult &out);

  double max_angel_ = 1.0;
  double traget_yaw_ = 0;
};
}  // namespace CVTE_BABOT
#endif
