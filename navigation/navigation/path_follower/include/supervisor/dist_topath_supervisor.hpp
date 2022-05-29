/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file dist_topath_supervisor.hpp
 *
 *@brief
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-30
 ************************************************************************/

#ifndef __DIST_TOPATH_SUPERVISOR_HPP
#define __DIST_TOPATH_SUPERVISOR_HPP

#include "supervisor.hpp"

namespace CVTE_BABOT {

class DistanceToPathSupervisor : public Supervisor {
 public:
  DistanceToPathSupervisor(double max_distance_to_path);

  virtual std::string getName() const { return "DistanceToPath"; }

  virtual void supervise(const SupervisorState &state, SupervisorResult &out);

 private:
  double max_dist_ = 1.0;
  double calculateDistanceToCurrentPathSegment(const SupervisorState &state,
                                               SupervisorResult &out);
};

}  // namespace CVTE_BABOT

#endif  //__DIST_TOPATH_SUPERVISOR_HPP