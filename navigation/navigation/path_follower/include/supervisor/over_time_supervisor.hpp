/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file over_time_supervisor.hpp
 *
 *@brief 监督是否超过允许执行时间
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version dev.1.0
 *@data 2021-03-22
 ************************************************************************/

#ifndef __OVER_TIME_SUPERVISOR_HPP
#define __OVER_TIME_SUPERVISOR_HPP

#include "supervisor.hpp"
#include <chrono>

namespace CVTE_BABOT {

class OverTimeSupervisor : public Supervisor {
 public:
  OverTimeSupervisor() : short_dist_(3.0), wait_sec_(60), pause_wait_sec_(60){};

  std::string getName() const override { return "OverTime"; }

  void supervise(const SupervisorState &state, SupervisorResult &out) override;

  void setSuperviseReferPath(const std::vector<Pose2d> &refer_path);

  void setOverTimeDist(double dist) { short_dist_ = dist; };

  void setOverTimeSec(int sec) { wait_sec_ = sec; }

  void setPauseTimeSec(int sec) { pause_wait_sec_ = sec; }

  void setRotateTimeSec(double sec) { rotate_wait_sec_ = sec; }

  void setAbsReach(bool is_abs_reach) { is_abs_reach_ = is_abs_reach; }

 private:
  Pose2d target_goal_;
  size_t path_size_ = 0;
  long int decount_sec_ = 0;  // 超时倒计时计数（秒），此监督器默认是1Hz运行
  bool close_target_update_flag_ =
      false;  // 比较靠近终点时，可以更新一次倒计时时间，去除之前冗余量
  bool pause_state_flag_ = false;
  bool rotate_flag_ = false;
  double short_dist_;
  int wait_sec_;
  double pause_wait_sec_;
  double rotate_wait_sec_;
  bool is_abs_reach_ = false;
  std::chrono::time_point<std::chrono::system_clock> overtime_start_;
  std::chrono::time_point<std::chrono::system_clock> pause_state_start_;
  std::chrono::time_point<std::chrono::system_clock> rotate_start_;
};
}  // namespace CVTE_BABOT

#endif  // end of __OVER_TIME_SUPERVISOR_HPP