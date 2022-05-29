/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file speed_level_supervisor.cpp
 *
 *@brief 速度等级的监督器
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev.1.0
 *@data 2021-02-20
 ************************************************************************/

#include "speed_controller.hpp"
#include "path_follower.hpp"
#include "supervisor/speed_level_supervisor.hpp"
#include <glog/logging.h>
#include "speed_decision_base.hpp"

namespace CVTE_BABOT {

void SpeedLevelSupervisor::supervise(const SupervisorState &state,
                                     SupervisorResult &out) {
  SpeedCtrlRes speed_res;  ///< 速度控制器的结果
  // bool analyze_flag =
  //     SpeedController::getPtrInstance()->adjustSpeedLevel(speed_res);
  SpeedDecisionResult speed_result =
      SpeedDecisionBase::getInstance()->getSpeedResult();

  if (speed_result.speed_radio == SpeedRadio::STOP) {
    // 发现进入停障碍处理
    out.can_continue = false;
    out.new_local_goal = false;
    out.status = OBSTACLESTOP;
    out.supervisor_name = speed_result.real_speed_controller;
    LOG(INFO) << "speed_controller_supervisor : "
              << " with " << speed_result.real_speed_controller;
  } else {
    LOG(INFO) << "Adjust speed level.";
  }
}
}  // namespace CVTE_BABOT