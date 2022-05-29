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
#ifndef SPEED_DECISION_C5_HPP_
#define SPEED_DECISION_C5_HPP_
#include "speed_decision_base.hpp"

namespace CVTE_BABOT {
class SpeedDecisionC5 : public SpeedDecisionBase {
 public:
  SpeedDecisionC5() = default;
  SpeedDecisionC5(const std::shared_ptr<SpeedDecisionBaseConfig> &ptr_config)
      : SpeedDecisionBase() {
    ptr_config_ = ptr_config;
    setConfig();
    LOG(INFO) << "creat C5 speed_decision ";
  }
};
}  // namespace CVTE_BABOT

#endif