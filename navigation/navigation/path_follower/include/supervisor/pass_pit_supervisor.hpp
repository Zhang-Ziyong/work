/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file pass_pit_supervisor.hpp
 *
 *@brief
 *
 *@modified by lizhongjia(lizhongjia@cvte.com)
 *
 *@author lizhongjia(lizhongjia@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2022-04-06
 ************************************************************************/

#ifndef __PASS_PIT_SUPERVISOR_HPP
#define __PASS_PIT_SUPERVISOR_HPP

#include "supervisor.hpp"

namespace CVTE_BABOT {
class PassPitSupervisor : public Supervisor {
 public:
  PassPitSupervisor(double pit_distance_front, bool enable_pit);

  std::string getName() const override { return "PassPit"; }

  void supervise(const SupervisorState &state, SupervisorResult &out) override;

 private:
  int checkPointCostValue(const Pose2d &check_point,
                          std::shared_ptr<Costmap2d> ptr_costmap_2d);

 private:
  double pit_distance_front_ = 1.0;  // 触发过坎距离
  bool enable_pit_ = false;
};
}  // namespace CVTE_BABOT

#endif