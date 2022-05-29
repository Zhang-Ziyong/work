/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file speed_decision_C5.hpp
 *
 *@brief 速度决策C5派生类
 *
 *@author chenweijian chenweijian@cvte.com
 *@modified
 *@version
 *@data
 ************************************************************************/
#ifndef SPEED_DECISION_C3_HPP_
#define SPEED_DECISION_C3_HPP_
#include "speed_decision_base.hpp"

namespace CVTE_BABOT {
class SpeedDecisionC3 : public SpeedDecisionBase {
 public:
  SpeedDecisionC3() = default;
  SpeedDecisionC3(const std::shared_ptr<SpeedDecisionBaseConfig> &ptr_config)
      : SpeedDecisionBase() {
    ptr_config_ = ptr_config;
    setConfig();
    LOG(INFO) << "creat C3 speed_decision ";
  }

  double getRecoverFactor(
      const RecoveDir dir_key,
      Eigen::Matrix<double, 2, 3> *T_wb_ptr = nullptr) override final;

  bool checkFrontDanger() override final;

  bool checkOutofRecovery() override final;

  double getCurSpeedFactor() override final;

  bool isObstacleInStopBound() override final;

 private:

  int getCurPoseCost();  // 路径

  double getCurPoseCostFactor();
};
}  // namespace CVTE_BABOT

#endif