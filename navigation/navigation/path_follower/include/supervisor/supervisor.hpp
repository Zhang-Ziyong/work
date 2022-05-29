/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file supervisor.hpp
 *
 *@brief 用于监测导航过程中的各种事件.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-25
 ************************************************************************/
#ifndef __SUPERVISOR_HPP
#define __SUPERVISOR_HPP

#include "type.hpp"
#include <glog/logging.h>

namespace CVTE_BABOT {

class Supervisor {
 public:
  virtual ~Supervisor() {}

  typedef std::shared_ptr<Supervisor> Ptr;

  virtual void supervise(const SupervisorState &state,
                         SupervisorResult &out) = 0;

  virtual void superviseCycle(const SupervisorState &state,
                              SupervisorResult &out) {
    if (!isTimeToSupvise()) {
      return;
    }
    supervise(state, out);
  }

  virtual std::string getName() const = 0;

  void setSuperviseFrquence(const double &main_freq, const double &sub_freq) {
    frequence_ = static_cast<int>(main_freq / sub_freq);
    LOG(INFO) << getName() << " Frequence: " << frequence_;
  }

  bool isTimeToSupvise() {
    if (++current_count_ % frequence_ == 0) {
      return true;
    }
    return false;
  }

 private:
  int frequence_ = 1;
  unsigned long int current_count_ = 0;
};

}  // namespace CVTE_BABOT

#endif