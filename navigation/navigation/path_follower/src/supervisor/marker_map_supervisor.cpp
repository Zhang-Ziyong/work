/*
 * @Author: chennuo@cvte.com
 * @Date: 2021-07-19 17:58:38
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-07-22 08:35:43
 */

#include "costmap_utils.hpp"
#include "costmap_2d.hpp"
#include "supervisor/marker_map_supervisor.hpp"

#include "path_follower.hpp"
#include "narrow_recognition.hpp"
#include "navigation_mediator.hpp"

namespace CVTE_BABOT {

MarkerMapSupervisor::MarkerMapSupervisor(bool enabled, double path_dir_length,
                                         double neighbor_expand_length)
    : enabled_(enabled),
      path_dir_length_(path_dir_length),
      neighbor_expand_length_(neighbor_expand_length) {
  LOG(INFO) << "marker surpervisor param: enabled->" << enabled_
            << " path_dir_length->" << path_dir_length_
            << " neighbor_expand_length->" << neighbor_expand_length_;
}

void MarkerMapSupervisor::supervise(const SupervisorState &state,
                                    SupervisorResult &out) {
  if (!enabled_)
    return;

  // 只在reference和local这两种状态下执行窄道策略
  if (state.state_ != PathFollowerStates::FOLLOW_REFERENCE &&
      state.state_ != PathFollowerStates::FOLLOW_LOCAL &&
      state.state_ != PathFollowerStates::ROTATE) {
    return;
  }

  // 获取标记地图
  std::shared_ptr<Costmap2d> marked_map = nullptr;
  NavigationMediator::getPtrInstance()->getData("marked_map", marked_map);
  if (!marked_map) {
    LOG_EVERY_N(INFO, 10) << "cant get marked map !";
    return;
  }

  // 获取当前执行轨迹
  // const SubPath &sub_path = state.sub_path_;
  std::shared_ptr<SubPath> sub_path = state.sub_path_;
  if (sub_path == nullptr || sub_path->wps.size() <= 1) {
    LOG(INFO) << "path size less than 1, no need check";
    return;
  }

  // 获取机器人当前的标记状态
  auto robot_pose = state.robot_pose_;
  WorldmapPoint wp(robot_pose.getX(), robot_pose.getY());
  CostmapPoint cp;
  if (!marked_map->worldToMap(wp, cp)) {
    LOG(WARNING) << "cant map [" << wp.d_x << "," << wp.d_y
                 << "] to map frame !";
    return;
  }
  unsigned char marked_state = marked_map->getCost(cp.ui_x, cp.ui_y);

  // 如果是窄道
  if (NARROW_GRAY_VALUE == marked_state) {
    LOG(INFO) << "robot locate on narrow ";

    // 计算机器人轨迹朝向
    double path_yaw = 0.0;
    double sum_dist = 0.0;
    int sidx = 0;
    if (state.state_ == PathFollowerStates::ROTATE)
      sidx = state.pf_ptr_->getPmPtr()->getReferIndex();

    for (int idx = sidx; idx < sub_path->wps.size() - 1; idx++) {
      sum_dist += sub_path->wps[idx].distanceTo(sub_path->wps[idx + 1]);
      if (sum_dist >= path_dir_length_ || idx + 1 == sub_path->wps.size() - 1) {
        path_yaw =
            atan2(sub_path->wps[idx].getY() - sub_path->wps[sidx].getY(),
                  sub_path->wps[idx].getX() - sub_path->wps[sidx].getX());
      }
    }

    // 计算机器人朝向与轨迹朝向的差值
    double diff_yaw = path_yaw - robot_pose.getYaw();
    if (diff_yaw < -M_PI)
      diff_yaw += 2 * M_PI;
    else if (diff_yaw > M_PI)
      diff_yaw -= 2 * M_PI;
    if (std::fabs(diff_yaw) < M_PI / 2) {
      out.can_continue = true;
      LOG(INFO) << "robot direction and path direction are near same, no need "
                   "retreat";
      return;
    }
    LOG(INFO) << "robot direction " << robot_pose.getYaw()
              << " and path direction " << path_yaw << " arent same !";

    // 此处开始处理机器人朝向与轨迹朝向不同的问题
    // 主要是记录哪段轨迹处于窄道中，然后交由对应的状态机处理
    bool out_narrow = false;  // 标记轨迹点是否出窄道
    int out_narrow_idx = sidx;
    int narrow_end_idx = sidx;
    for (; narrow_end_idx < sub_path->wps.size(); narrow_end_idx++) {
      // 获取当前轨迹点标记状态
      const auto &p = sub_path->wps[narrow_end_idx];
      WorldmapPoint twp(p.getX(), p.getY());
      CostmapPoint tcp;
      if (!marked_map->worldToMap(twp, tcp)) {
        LOG(WARNING) << "cant map [" << twp.d_x << "," << twp.d_y
                     << "] to map frame !";
        return;
      }
      unsigned char t_marked_state = marked_map->getCost(tcp.ui_x, tcp.ui_y);

      // 如果还在窄道内则继续向前推进
      if (NARROW_GRAY_VALUE == t_marked_state) {
        out_narrow = false;
        continue;
      }

      // 记录出窄道轨迹索引
      if (!out_narrow) {
        out_narrow_idx = narrow_end_idx;
        out_narrow = true;
      }

      // 添加窄道冗余
      double expand_dis = sub_path->wps[out_narrow_idx].distanceTo(
          sub_path->wps[narrow_end_idx]);
      if (expand_dis >= neighbor_expand_length_ ||
          narrow_end_idx + 1 == sub_path->wps.size()) {
        break;
      }
    }
    narrow_end_idx = std::min(narrow_end_idx, int(sub_path->wps.size() - 1));

    // 返回窄道起始点轨迹索引
    out.can_continue = false;
    out.start_point.index = sidx;
    out.target_point.index = narrow_end_idx;
    out.status = SupervisorStatus::MARKERMAP;
  }
}
}  // namespace CVTE_BABOT