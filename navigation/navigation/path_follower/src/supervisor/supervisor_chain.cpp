/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file supervisor_chain.cpp
 *
 *@brief
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-30
 ************************************************************************/

#include "supervisor/supervisor_chain.hpp"

namespace CVTE_BABOT {

std::shared_ptr<SupervisorChain> SupervisorChain::vis_chain_ptr_ = nullptr;

SupervisorChain::SupervisorChain() {}

void SupervisorChain::addSupervisor(Supervisor::Ptr supervisor) {
  supervisors_.push_back(supervisor);
}

SupervisorResult SupervisorChain::supervise(SupervisorState &state) {
  for (const auto &supervisor : supervisors_) {
    SupervisorResult res;
    supervisor->supervise(state, res);

    if (!res.can_continue) {
      return res;
    }
  }

  return SupervisorResult();  // Constructor sets to can_continue = true.
}

SupervisorResult SupervisorChain::superviseCycle(SupervisorState &state) {
  for (const auto &supervisor : supervisors_) {
    SupervisorResult res;
    supervisor->superviseCycle(state, res);

    if (!res.can_continue) {
      return res;
    }
  }

  return SupervisorResult();  // Constructor sets to can_continue = true.
}

SupervisorResult SupervisorChain::superviseAssignOne(SupervisorState &state,
                                                     const std::string &names) {
  for (const auto &supervisor : supervisors_) {
    std::stringstream ss(names);
    std::string name;
    while (ss >> name) {
      // LOG(INFO) << "name : " << name;
      if (supervisor->getName() == name) {
        SupervisorResult res;
        supervisor->supervise(state, res);

        if (!res.can_continue) {
          return res;
        }
      }
    }
  }

  return SupervisorResult();  // Constructor sets to can_continue = true.
}

}  // namespace CVTE_BABOT
