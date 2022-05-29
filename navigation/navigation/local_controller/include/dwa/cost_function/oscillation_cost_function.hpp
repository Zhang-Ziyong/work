/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file oscillation_cost_function.hpp
 *
 *@brief DWA中计算震荡的评价函数
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/

#ifndef __OSCILLATION_COST_FUNCTION_HPP
#define __OSCILLATION_COST_FUNCTION_HPP

#include "cost_function.hpp"
#include "trajectory.hpp"

#include <eigen3/Eigen/Core>

namespace CVTE_BABOT {
/**
* OscillationCostFunction
* @brief
*   DWA中计算震荡的评价函数
* */

class OscillationCostFunction : public CostFunction {
 public:
  OscillationCostFunction();
  OscillationCostFunction(const OscillationCostFunction &obj);
  OscillationCostFunction &operator=(const OscillationCostFunction &obj);
  ~OscillationCostFunction();

  /**
  * scoreTrajectory
  * @brief
  *   计算一条路径的得分
  *
  * @param[in] traj-需要计算分数的路径
  * */
  double scoreTrajectory(const Trajectory &traj) override;

  /**
  * resetOscillationFlags
  * @brief
  *   将类里面的标志位复位
  * */
  inline void resetOscillationFlags() {
    strafe_pos_only_ = false;
    strafe_neg_only_ = false;
    strafing_pos_ = false;
    strafing_neg_ = false;

    rot_pos_only_ = false;
    rot_neg_only_ = false;
    rotating_pos_ = false;
    rotating_neg_ = false;

    forward_pos_only_ = false;
    forward_neg_only_ = false;
    forward_pos_ = false;
    forward_neg_ = false;
  }

  /**
  * updateOscillationFlags
  * @brief
  *   更新震荡的标志位
  *
  * @param[in] pos-上一时刻的固定位置
  * @param[in] traj-输入的路径
  * @param[in] min_vel_trans-最小移动速度
  * */
  void updateOscillationFlags(const Eigen::Vector3f &pos, Trajectory &traj,
                              const double &min_vel_trans);

  /**
  * setOscillationResetDist
  * @brief
  *   设置复位的距离阈值
  *
  * @param[in] dist-直线距离的阈值
  * @param[in] angle-角度复位的阈值
  * */
  inline void setOscillationResetDist(const double &dist, const double &angle) {
    oscillation_reset_dist_ = dist;
    oscillation_reset_angle_ = angle;
  }

  /**
  * resetOscillationFlagsIfPossible
  * @brief
  *   复位震荡的标志位
  *
  * @param[in] pos-当前机器人位置
  * @param[in] prev-上一时刻机器人位置
  * */
  void resetOscillationFlagsIfPossible(const Eigen::Vector3f &pos,
                                       const Eigen::Vector3f &prev);

  /**
  * setOscillationFlags
  * @brief
  *   设置震荡的标志位
  *
  * @param[in] t-路径
  * @param[in] min_vel_trans-最小移动速度
  * */
  bool setOscillationFlags(Trajectory &t, const double &min_vel_trans);

 private:
  ///< 一些判断标志位
  bool strafe_pos_only_, strafe_neg_only_, strafing_pos_, strafing_neg_;
  bool rot_pos_only_, rot_neg_only_, rotating_pos_, rotating_neg_;
  bool forward_pos_only_, forward_neg_only_, forward_pos_, forward_neg_;

  double oscillation_reset_dist_ = 0.0;   ///< 震荡距离复位值
  double oscillation_reset_angle_ = 0.0;  ///< 震荡角度复位值

  Eigen::Vector3f prev_stationary_pos_;  ///< 上一时刻固定位置
};

}  // namespace CVTE_BABOT

#endif  //__OSCILLATION_COST_FUNCTION_HPP