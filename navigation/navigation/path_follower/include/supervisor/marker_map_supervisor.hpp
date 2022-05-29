/*
 * @Author: chennuo@cvte.com
 * @Date: 2021-07-19 17:52:17
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-07-21 20:20:12
 */

#pragma once

#include "supervisor.hpp"

namespace CVTE_BABOT {

class MarkerMapSupervisor : public Supervisor {
 public:
  MarkerMapSupervisor(bool enabled, double path_dir_length,
                      double neighbor_expand_length);

  std::string getName() const override { return "MarkerMap"; }

  void supervise(const SupervisorState &state, SupervisorResult &out) override;

 private:
  bool enabled_{false};
  double path_dir_length_{1.0};         // 确定轨迹方向的轨迹长度
  double neighbor_expand_length_{1.0};  // 窄道轨迹延长距离
};

}  // namespace CVTE_BABOT