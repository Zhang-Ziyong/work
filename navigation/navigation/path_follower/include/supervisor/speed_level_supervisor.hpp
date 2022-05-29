/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file speed_level_supervisor.hpp
 *
 *@brief 速度等级的监督器
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev.1.0
 *@data 2021-02-20
 ************************************************************************/

#ifndef __SPEED_LEVEL_SUPERVISOR_HPP
#define __SPEED_LEVEL_SUPERVISOR_HPP

#include "supervisor.hpp"

namespace CVTE_BABOT {

class SpeedLevelSupervisor : public Supervisor {
public:
  SpeedLevelSupervisor() = default;

  std::string getName() const override { return "SpeedLevel"; }

  void supervise(const SupervisorState &state, SupervisorResult &out) override;

private:


};

}

#endif // end of __SPEED_LEVEL_SUPERVISOR_HPP