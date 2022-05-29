/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file path_manager.cpp
 *
 *@brief 路径管理类的接口具体实现
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2020 - 07 - 08
 ************************************************************************/

#include "path_manager.hpp"
#include "bsline/path_optimizer.hpp"
#include "costmap_2d.hpp"
#include "pit_planner/pit_planner.hpp"
#include "local_planner.hpp"
#include "local_planner_factory.hpp"
#include "robot_info.hpp"
#include "speed_decision_base.hpp"
#include "pnc_map.hpp"
#include <float.h>
#include <glog/logging.h>

#define DYNAMIC_WINDOW_SIZE 100

namespace CVTE_BABOT {

PathManager::PathManager(std::shared_ptr<Costmap2d> costmap,
                         const std::string &local_algo) {
  ptr_local_costmap_ = costmap;
  ptr_costmap_2d_ = costmap;
  local_planner_algorithm_ = local_algo;
  // 工厂构造局部路径规划算法
  LCPlannerFactory local_planner_factory;
  ptr_local_planner_ = local_planner_factory.createLocalPlanner(
      local_algo, costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  ptr_dijk_planner_ = local_planner_factory.createLocalPlanner(
      "dijkstra", costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

  ptr_local_path_ = std::make_shared<SubPath>();
  ptr_refer_path_ = std::make_shared<SubPath>();
  ptr_ctrol_path_ = std::make_shared<SubPath>();
}

void PathManager::setOptimizeLocalPathFlag(bool optimize_flag,
                                           bool vel_limit_ctrl,
                                           bool optimize_ctrl) {
  optimize_local_path_ = optimize_flag;
  calc_vel_limit_ctrl_ = vel_limit_ctrl;
  calc_optimize_ctrl_ = optimize_ctrl;
  LOG(INFO) << optimize_local_path_ << ", " << calc_vel_limit_ctrl_ << ", "
            << calc_optimize_ctrl_;
}

void PathManager::setOptimizeLocalParams(double v_limit, double w_limit,
                                         double acc_limit,
                                         double sample_delta_time,
                                         double bsp_delta_time) {
  v_limit_ = v_limit;
  w_limit_ = w_limit;
  acc_limit_ = acc_limit;
  sample_delta_time_ = sample_delta_time;
  bsp_delta_time_ = bsp_delta_time;
}

void PathManager::setOptCtrlPointsParams(double lambda1, double lambda2,
                                         double lambda3, double lambda4,
                                         bool use_endpt_opt, double dist,
                                         double min_rot_radiu, int max_iter_num,
                                         double max_iter_time) {
  lambda1_ = lambda1;
  lambda2_ = lambda2;
  lambda3_ = lambda3;
  lambda4_ = lambda4;
  use_endpt_opt_ = use_endpt_opt;
  dist_ = dist;
  min_rot_radiu_ = min_rot_radiu;
  max_iter_nums_ = max_iter_num;
  max_iter_time_ = max_iter_time;
}

void PathManager::setReferPath(std::shared_ptr<SubPath> refer_path) {
  resetLocalTarget();
  setReferIndex(0);
  std::lock_guard<std::mutex> lock(refer_path_mutex_);
  refer_path_size_ = refer_path->wps.size();
  ptr_refer_path_ = refer_path;
  ptr_refer_path_->type = PathType::REFER;
  refer_path_update_flag_ = true;
  // updateCtrolPath("refer");
}

bool PathManager::updateLocalPath(const PoseMsg &start, const PoseMsg &target) {
  SubPath path;
  std::shared_ptr<SubPath> local_path = nullptr;
  if (!makeLocalPlan(start.position, target.position, start.velocity.v,
                     target.velocity.v, path)) {
    LOG(ERROR) << "update local planner failed.";
    return false;
  } else {
    // 测试可视化使用
    original_local_path_.clear();
    original_local_path_.resize(path.wps.size());
    original_local_path_ = path.wps;

    if (optimize_local_path_) {
      local_path = std::make_shared<SubPath>(
          optimizeLocalPath(path, start.velocity.v, target.velocity.v));
      // LOG(INFO) << "Optimize local path:";
      // for (auto pp : path->wpis) { LOG(INFO) << " " << pp.v << ", " << pp.w;
      // }
    }
    setLocalPath(local_path);
    setLocalTarget(
        target);  // 先保证此时找到的目标点能规划成功，再更新绕障目标点
    return true;
  }
}

bool PathManager::updatePitPath(const Pose2d &pit_in_pose,
                                const Pose2d &pit_out_pose,
                                const PoseMsg &target) {
  LOG(INFO) << "updatePitPath, pit_in_pose(" << pit_in_pose.getX() << ","
            << pit_in_pose.getY() << "), pit_out_pose(" << pit_out_pose.getX()
            << "," << pit_out_pose.getY() << "), target("
            << target.position.getX() << "," << target.position.getY()
            << ",--index=" << target.index << ")";

  /// 1.输出包罗框
  auto ptr_pnc_map = std::make_shared<PncMap>();
  cvte::hdmap::Box2d box;
  math_utils::Vec2d pit_in(pit_in_pose.getX(), pit_in_pose.getY());
  ptr_pnc_map->GetPitBox(pit_in, box);

  /// 2.寻找最优过坎路径
  PitPlanner pit_planner(pit_edge_dist_, pit_access_interval_);
  pit_planner.SetBox(box.center(), box.length(), box.width(), box.heading());
  pit_planner.SetWayPoint(pit_in_pose, pit_out_pose);
  pit_planner.SetCostMap(getCostmap());
  Eigen::Vector2d start_point;
  Eigen::Vector2d end_point;
  if (!pit_planner.GetOptimalAccessPoint(start_point, end_point)) {
    return false;
  }

  /// 3.生成过坎路径
  Pose2d current_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  Eigen::Vector2d pit_vec = end_point - start_point;

  PoseMsg pit_start;
  PoseMsg pit_end;
  PoseMsg start;
  start.position = current_pose;

  pit_start.position.setX(start_point.x());
  pit_start.position.setY(start_point.y());
  pit_start.position.setYaw(atan2(pit_vec.y(), pit_vec.x()));

  pit_end.position.setX(end_point.x());
  pit_end.position.setY(end_point.y());
  pit_end.position.setYaw(atan2(pit_vec.y(), pit_vec.x()));

  SubPath pit_path;
  SubPath path1;
  SubPath path2;
  SubPath path3;
  pit_start.velocity.v = start.velocity.v / 2;
  if (!makeDiskPlan(start.position, pit_start.position, start.velocity.v,
                    pit_start.velocity.v, path1)) {
    LOG(ERROR) << "Pit make plan1 failure\r\n";
    return false;
  }

  pit_end.velocity.v = pit_start.velocity.v;
  if (!makeStraightLinePath(pit_start.position, pit_end.position, path2)) {
    LOG(WARNING) << "make straight line faild.";
    return false;
  }

  if (!makeDiskPlan(pit_end.position, target.position, pit_end.velocity.v,
                    target.velocity.v, path3)) {
    LOG(ERROR) << "Pit make plan3 failure\r\n";
    return false;
  }

  if (path1.size() > 1) {
    path1.wps.erase(path1.wps.end() - 2);
    path1.wpis.erase(path1.wpis.end() - 2);
  }

  if (path3.size() > 1) {
    path3.wps.erase(path3.wps.end() - 2);
    path3.wpis.erase(path3.wpis.end() - 2);
  }

  pit_path = path1 + path2 + path3;

  // 测试可视化使用
  original_local_path_.clear();
  original_local_path_.resize(pit_path.wps.size());
  original_local_path_ = pit_path.wps;

  std::shared_ptr<SubPath> local_path = nullptr;
  if (optimize_local_path_) {
    local_path = std::make_shared<SubPath>(
        optimizeLocalPath(pit_path, start.velocity.v, 0));
  }

  setLocalPath(local_path);
  setLocalTarget(target);

  return true;
}

bool PathManager::updateLocalPath(const PoseMsg &target) {
  SubPath path;
  Pose2d start = RobotInfo::getPtrInstance()->getCurrentPose();
  Velocity start_vel = RobotInfo::getPtrInstance()->getCurrentVel();
  std::shared_ptr<SubPath> sub_path = nullptr;
  if (!makeLocalPlan(start, target.position, start_vel.d_x, target.velocity.v,
                     path)) {
    LOG(ERROR) << "update local planner with expect target failed.";
    return false;
  } else {
    if (optimize_local_path_) {
      sub_path = std::make_shared<SubPath>(
          optimizeLocalPath(path, start_vel.d_x, target.velocity.v));
    }
    setLocalPath(sub_path);
    setLocalTarget(
        target);  // 先保证此时找到的目标点能规划成功，再更新绕障目标点
    LOG(INFO) << "Succeed update local path and target.";
    return true;
  }
}

bool PathManager::updateLocalPath() {
  SubPath path;
  Pose2d start = RobotInfo::getPtrInstance()->getCurrentPose();
  Velocity start_vel = RobotInfo::getPtrInstance()->getCurrentVel();
  PoseMsg target = getLocalTarget();
  std::shared_ptr<SubPath> sub_path = nullptr;
  if (!makeLocalPlan(start, target.position, start_vel.d_x, target.velocity.v,
                     path)) {
    LOG(ERROR) << "update local planner failed.";
    return false;
  } else {
    if (optimize_local_path_) {
      sub_path = std::make_shared<SubPath>(
          optimizeLocalPath(path, start_vel.d_x, target.velocity.v));
    }
    setLocalPath(sub_path);
    return true;
  }
}

void PathManager::reverseLocalPath(int sidx, int eidx) {
  ptr_local_path_->reverseCurrentPath(sidx, eidx);
}

void PathManager::reverseReferPath(int sidx, int eidx) {
  ptr_refer_path_->reverseCurrentPath(sidx, eidx);
}

bool PathManager::cycleUpdateLocalPath() {
  SubPath path;
  Pose2d start = RobotInfo::getPtrInstance()->getCurrentPose();
  Velocity start_vel = RobotInfo::getPtrInstance()->getCurrentVel();
  PoseMsg target = getLocalTarget();
  LOG(INFO) << "cycle start: " << start.getX() << ", " << start.getY() << ", "
            << start.getYaw() << "     target: " << target.position.getX()
            << ", " << target.position.getY() << ", "
            << target.position.getYaw();

  // 条件一：如果当前局部路径的目标点已经改变（外部监督或搜索策略），则需要再触发规划
  // 条件二：障碍物和机器人直线距离之间存在障碍物，则不再重规划
  Pose2d path_target = getLocalPath()->wps.back();
  LOG(INFO) << "path target: " << path_target.getX() << ", "
            << path_target.getY() << ", " << path_target.getYaw();
  if ((path_target.distanceTo(target.position) < 0.1) &&
      checkObstacleBetweenTwoPoints(start, target.position)) {
    return true;
  }
  std::shared_ptr<SubPath> sub_path;
  if (!makeLocalPlan(start, target.position, start_vel.d_x, target.velocity.v,
                     path)) {
    LOG(ERROR) << "Cycle update local planner failed.";
    return false;
  } else {
    if (optimize_local_path_) {
      sub_path = std::make_shared<SubPath>(
          optimizeLocalPath(path, start_vel.d_x, target.velocity.v));
    }
    setLocalPath(sub_path);
    return true;
  }
}

bool PathManager::checkObstacleBetweenTwoPoints(const Pose2d start,
                                                const Pose2d target) {
  SubPath path;

  // 在两点之间规划一条直线
  if (!makeStraightLinePath(start, target, path)) {
    LOG(WARNING) << "make straight line faild.";
  }

  // 两点之间
  if (path.wps.empty() || start.distanceTo(target) < 1.0) {
    LOG(INFO) << "Too short Need not check.";
    return true;
  }

  for (auto pose : path.wps) {
    if (checkPointCostValue(pose)) {
      LOG(WARNING) << "straghit has obstacle, don't replan.";
      return true;
    }
  }
  return false;
}

bool PathManager::makeStraightLinePath(const Pose2d start, const Pose2d end,
                                       SubPath &path) {
  if (start == end) {
    LOG(ERROR) << "Start point and end point is the same.";
    return false;
  }

  path.wps.clear();
  path.wpis.clear();
  double length = start.distanceTo(end);
  double path_resolution = 0.2;  // 等距直线插值的距离（米）
  double delta_t = 1 * path_resolution / length;
  Eigen::Vector3d p1(start.getX(), start.getY(), start.getYaw());
  Eigen::Vector3d p0(end.getX(), end.getY(), end.getYaw());
  WayPointInfo wpi;

  for (double t = 0; t < 1.0; t += delta_t) {
    Eigen::Vector3d p_tmp = p0 * t + p1 * (1 - t);
    Pose2d point(p_tmp(0), p_tmp(1), p_tmp(2));
    path.wps.emplace_back(point);
    path.wpis.emplace_back(wpi);
    wpi.path_length += path_resolution;
  }
  return true;
}

bool PathManager::checkPointCostValue(const Pose2d &check_point) {
  WorldmapPoint wm_point = {check_point.getX(), check_point.getY()};
  CostmapPoint cp_point;
  int cost_value = 0;
  if (ptr_costmap_2d_->worldToMap(wm_point, cp_point)) {
    cost_value = static_cast<int>(
        ptr_costmap_2d_->getCost(cp_point.ui_x, cp_point.ui_y));
  } else {
    LOG(WARNING) << "checkPointCost getCost ERROR !";
    cost_value = 256;
  }

  if (cost_value > 100 && cost_value <= 254) {
    LOG(INFO) << "check cost value: " << cost_value << " at "
              << check_point.getX() << ", " << check_point.getY();
    return true;
  }
  return false;
}

bool PathManager::makeDiskPlan(const Pose2d &start, const Pose2d &end,
                               const double &start_v, const double &end_v,
                               SubPath &result) {
  MakePlanInfo make_plan_info;
  make_plan_info.start_point = start;
  make_plan_info.target_point = end;

  Eigen::Vector3d start_vel(start_v * cos(start.getYaw()),
                            start_v * sin(start.getYaw()), 0);
  Eigen::Vector3d end_vel(end_v * cos(end.getYaw()), end_v * sin(end.getYaw()),
                          0.0);
  make_plan_info.start_vel = start_vel;
  make_plan_info.target_vel = end_vel;
  LOG(INFO) << "make dijkstra plan from: (" << start.getX() << ", "
            << start.getY() << ", " << start.getYaw() << ") to (" << end.getX()
            << ", " << end.getY() << ", " << end.getYaw() << ")";

  return ptr_dijk_planner_->makePlan(make_plan_info, result);
}

bool PathManager::makeLocalPlan(const Pose2d &start, const Pose2d &end,
                                const double &start_v, const double &end_v,
                                SubPath &result) {
  MakePlanInfo make_plan_info;
  make_plan_info.start_point = start;
  make_plan_info.target_point = end;

  Eigen::Vector3d start_vel(start_v * cos(start.getYaw()),
                            start_v * sin(start.getYaw()), 0);
  Eigen::Vector3d end_vel(end_v * cos(end.getYaw()), end_v * sin(end.getYaw()),
                          0.0);
  make_plan_info.start_vel = start_vel;
  make_plan_info.target_vel = end_vel;
  LOG(INFO) << "make local plan from: (" << start.getX() << ", " << start.getY()
            << ", " << start.getYaw() << ") to (" << end.getX() << ", "
            << end.getY() << ", " << end.getYaw() << ")";
  // TODO: 如过没有速度信息需要加入

  // 规划前先判断局部代价地图大小足够支持轨迹规划，如果太小，则需要把全局代价地图嵌进去再规划
  if (ptr_local_planner_->needExpandCostMap(make_plan_info)) {
    LOG(INFO) << "need expand costmap for make local plan";

    if (!ptr_global_costmap_ || !ptr_local_costmap_) {
      LOG(WARNING) << "no global or local costmap for expand !";
      return false;
    }

    // 确定嵌入范围
    double min_x = std::max(ptr_local_costmap_->getOriginX(),
                            ptr_global_costmap_->getOriginX());
    double min_y = std::max(ptr_local_costmap_->getOriginY(),
                            ptr_global_costmap_->getOriginY());
    double max_x = std::min(ptr_local_costmap_->getOriginX() +
                                ptr_local_costmap_->getSizeInMetersX(),
                            ptr_global_costmap_->getOriginX() +
                                ptr_global_costmap_->getSizeInMetersX());
    double max_y = std::min(ptr_local_costmap_->getOriginY() +
                                ptr_local_costmap_->getSizeInMetersY(),
                            ptr_global_costmap_->getOriginY() +
                                ptr_global_costmap_->getSizeInMetersY());
    WorldmapPoint minp(min_x, min_y);
    WorldmapPoint maxp(max_x, max_y);

    // 确定copy的地图起始点索引
    CostmapPoint s_min_cp, d_min_cp, s_max_cp, d_max_cp;
    if (!ptr_local_costmap_->worldToMap(minp, s_min_cp) ||
        !ptr_local_costmap_->worldToMap(maxp, s_max_cp) ||
        !ptr_global_costmap_->worldToMap(minp, d_min_cp) ||
        !ptr_global_costmap_->worldToMap(maxp, d_max_cp)) {
      LOG(WARNING) << "minp [" << minp.d_x << "," << minp.d_y << "] or maxp ["
                   << maxp.d_x << "," << maxp.d_y
                   << "] is out of global or local map !";
    }
    const uint32_t sm_lower_left_x = s_min_cp.ui_x;
    const uint32_t sm_lower_left_y = s_min_cp.ui_y;
    const uint32_t dm_lower_left_x = d_min_cp.ui_x;
    const uint32_t dm_lower_left_y = d_min_cp.ui_y;
    const uint32_t region_size_x = s_max_cp.ui_x - s_min_cp.ui_x;
    const uint32_t region_size_y = s_max_cp.ui_y - s_min_cp.ui_y;
    const uint32_t sm_size_x = ptr_local_costmap_->getSizeInCellsX();
    const uint32_t dm_size_x = ptr_global_costmap_->getSizeInCellsX();

    // 将局部代价地图嵌入到全局代价地图中，形成新的用于规划的地图
    std::shared_ptr<Costmap2d> ptr_new_costmap =
        std::make_shared<Costmap2d>(*ptr_global_costmap_);
    boost::shared_array<uint8_t> new_costmap_array =
        ptr_new_costmap->getCharMap();
    boost::shared_array<uint8_t> local_costmap_array =
        ptr_local_costmap_->getCharMap();

    uint8_t *sm_index = local_costmap_array.get() +
                        (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    uint8_t *dm_index = new_costmap_array.get() +
                        (dm_lower_left_y * dm_size_x + dm_lower_left_x);
    for (uint32_t i = 0; i < region_size_y; ++i) {
      memcpy(dm_index, sm_index, region_size_x * sizeof(uint8_t));
      sm_index += sm_size_x;
      dm_index += dm_size_x;
    }

    // 工厂构造局部路径规划算法
    LCPlannerFactory local_planner_factory;
    ptr_local_planner_ = local_planner_factory.createLocalPlanner(
        local_planner_algorithm_, ptr_new_costmap->getSizeInCellsX(),
        ptr_new_costmap->getSizeInCellsY());
    ptr_local_planner_->updateCostMap(ptr_new_costmap);

    // 规划路径
    bool plan_ok = ptr_local_planner_->makePlan(make_plan_info, result);

    // 恢复默认配置
    ptr_local_planner_ = local_planner_factory.createLocalPlanner(
        local_planner_algorithm_, ptr_local_costmap_->getSizeInCellsX(),
        ptr_local_costmap_->getSizeInCellsY());
    ptr_local_planner_->updateCostMap(ptr_local_costmap_);

    return plan_ok;
  }

  if (RobotInfo::getPtrInstance()->getMissionType() != MISSIONTYPE::ELVATOR) {
    return ptr_local_planner_->makePlan(make_plan_info, result);
  } else {
    return ptr_dijk_planner_->makePlan(make_plan_info, result);
  }
}

SubPath PathManager::optimizeLocalPath(const SubPath &path_pose,
                                       const double &start_vel,
                                       const double &end_vel) {
  if (path_pose.empty()) {
    LOG(WARNING) << "Can't optimize an empty path.";
    return path_pose;
  }
  PathOptimizer path_opter;
  double speed_level =
      SpeedDecisionBase::getInstance()->getMissionManagerSpeedValue();
  CtrlPointOptParams ctrl_pt_opt_params(lambda1_, lambda2_, lambda3_, lambda4_,
                                        dist_, min_rot_radiu_, max_iter_nums_,
                                        max_iter_time_, use_endpt_opt_);
  if (local_planner_algorithm_ == "kinodynamic") {
    // Kinodynamic 全局规划的时候可能有前进后退的情况，优化时需要分开处理
    std::vector<SubPath> segment_result;
    if (path_opter.judgeSegmentPath(path_pose, segment_result)) {
      OptPathParams back_opt_params(
          v_limit_, w_limit_, acc_limit_, bsp_delta_time_ / speed_level,
          sample_delta_time_, 0.0, 0.0, 0, true, true, ctrl_pt_opt_params);
      OptPathParams forward_opt_params(
          v_limit_, w_limit_, acc_limit_, bsp_delta_time_ / speed_level,
          sample_delta_time_, 0.1, end_vel, 1, true, true, ctrl_pt_opt_params);
      SubPath total_path;
      for (size_t index = 0; index < segment_result.size(); index++) {
        SubPath sub_path;
        if (segment_result[index].wpis[1].v >= 0) {
          LOG(INFO) << "forward segment";
          sub_path = path_opter.curveFitPath(
              segment_result[index], forward_opt_params, ptr_costmap_2d_);
          for (size_t point_index = 0; point_index < sub_path.wps.size();
               point_index++) {
            total_path.wps.emplace_back(sub_path.wps[point_index]);
            total_path.wpis.emplace_back(sub_path.wpis[point_index]);
          }
        } else {
          LOG(INFO) << "backward segment";
          sub_path = path_opter.curveFitPath(segment_result[index],
                                             back_opt_params, ptr_costmap_2d_);
          for (size_t point_index = 0; point_index < sub_path.wps.size();
               point_index++) {
            Pose2d back_point(sub_path.wps[point_index].getX(),
                              sub_path.wps[point_index].getY(),
                              AngleCalculate::normalizeAngle(
                                  sub_path.wps[point_index].getYaw() + M_PI));
            WayPointInfo back_vel(-fabs(sub_path.wpis[point_index].v),
                                  sub_path.wpis[point_index].w,
                                  sub_path.wpis[point_index].curve);
            total_path.wps.emplace_back(back_point);
            total_path.wpis.emplace_back(back_vel);
          }
        }
      }
      return total_path;
    }
  }
  double input_start_vel = fabs(start_vel < 0.1) ? 0.1 : start_vel;
  // double input_start_vel = start_vel;
  OptPathParams opt_params(v_limit_, w_limit_, acc_limit_,
                           bsp_delta_time_ / speed_level, sample_delta_time_,
                           input_start_vel, end_vel, 1, calc_vel_limit_ctrl_,
                           calc_optimize_ctrl_, ctrl_pt_opt_params);
  SubPath opt_path =
      path_opter.curveFitPath(path_pose, opt_params, ptr_costmap_2d_);
  return opt_path;
}

void PathManager::setLocalPath(std::shared_ptr<SubPath> sub_path) {
  resetLocalIndex();
  setAvoiderReferIndex(getReferIndex());
  if (!forward_) {
    for (unsigned int i = 0; i < sub_path->wps.size() - 1; i++) {
      sub_path->wps[i + 1].yaw =
          std::atan2(sub_path->wps[i].y - sub_path->wps[i + 1].y,
                     sub_path->wps[i].x - sub_path->wps[i + 1].x);
    }
    sub_path->wps[0].yaw = sub_path->wps[1].yaw;

    for (unsigned int i = 0; i < sub_path->wps.size() - 1; i++) {
      sub_path->wpis[i].v = -sub_path->wpis[i].v;
      sub_path->wpis[i].w = -sub_path->wpis[i].w;
    }
  }
  std::lock_guard<std::mutex> lock(local_path_mutex_);
  local_path_size_ = sub_path->wps.size();
  ptr_local_path_ = sub_path;
  ptr_local_path_->type = PathType::LOCAL;
  local_path_update_flag_ = true;
  // updateCtrolPath("local");
}

bool PathManager::isNeedUpdateLocalCtrlPath() {
  if (local_path_update_flag_) {
    local_path_update_flag_ = false;
    return true;
  } else {
    return false;
  }
}

bool PathManager::isNeedUpdateReferCtrlPath() {
  if (refer_path_update_flag_) {
    refer_path_update_flag_ = false;
    return true;
  } else {
    return false;
  }
}

bool PathManager::updateCtrolPath(const std::string &from) {
  std::shared_ptr<SubPath> source_path;
  double sort_cut_size = 5.0;

  // 从局部路径或者任务路径中截取路径点，复制到控制路径中
  if ("local" == from) {
    source_path = getLocalPath();
  } else if ("refer" == from) {
    source_path = getReferPath();
  } else {
    LOG(ERROR) << from << " path isn't exist!";
    return false;
  }

  if (source_path->wps.empty()) {
    LOG(ERROR) << "PathFollower : source_path is empty.";
    return false;
  }

  LOG(INFO) << "update ctrol path from " << from
            << " with size = " << source_path->wps.size();
  std::shared_ptr<SubPath> control_path = std::make_shared<SubPath>();
  auto current_pose = RobotInfo::getPtrInstance()->getCurrentPoseWithOffset();

  // 查找轨迹上距当前机器人位姿最近的轨迹点
  int index = 0;
  double dist = 0.0;
  double sum = 0.0;
  double check_sum = 0.0;
  double check_sum_size = 0.4;
  double min_dist = DBL_MAX;
  size_t begin_index = 0;
  if ("refer" == from) {
    begin_index = getReferIndex();
  } else {
    begin_index = getLocalIndex();
  }
  LOG(INFO) << from << " at begin_index: " << begin_index;
  for (size_t i = begin_index; i < source_path->wps.size(); ++i) {
    if (i != 0) {
      // check_sum += source_path->wps[i - 1].distanceTo(source_path->wps[i]);
      check_sum += source_path->wpis[i].path_length -
                   source_path->wpis[i - 1].path_length;
    }
    dist = current_pose.distanceTo(source_path->wps[i]);
    if (dist > max_distance_) {
      continue;
    } else if (dist < min_dist) {
      min_dist = dist;
      index = i;
      check_sum = 0;
    }
    if (check_sum > check_sum_size) {
      check_sum = 0;
      break;
    }
  }

  // if ("refer" == from && index > begin_index) {
  //   // setReferIndex(index);
  //   LOG(INFO) << "update index: " << index;
  // } else if ("local" == from && index > begin_index) {
  //   setLocalIndex(index);
  // }

  if (min_dist > max_distance_) {
    LOG(ERROR) << "dist to path is too far of: " << min_dist
               << " max :" << max_distance_ << ".";
    return false;
  }

  double path_angle_diff = 0;
  double cal_angle_length = 0.4;
  double direct_change_thresh = 0.3;

  for (size_t j = index; j < source_path->wps.size(); ++j) {
    control_path->wps.emplace_back(source_path->wps[j]);

    if (!source_path->wpis.empty()) {
      control_path->wpis.emplace_back(source_path->wpis[j]);
    }

    if (j != 0) {
      sum += source_path->wps[j].distanceTo(source_path->wps[j - 1]);
    }
    if (sum > cal_angle_length && fabs(path_angle_diff) < 0.000001) {
      path_angle_diff = source_path->wps[j].getYaw() - current_pose.getYaw();
    }
    // 需要改成参数设置
    if (sum > sort_cut_size) {
      break;
    }
  }
  setCtrolPath(control_path);
  std::lock_guard<std::mutex> lock_direct(path_direct_mutex_);
  if (path_angle_diff > direct_change_thresh) {
    RobotInfo::getPtrInstance()->setRobotMotionState(
        ROBOTMOTIONSTATE::TURN_LEFT);
  } else if (path_angle_diff < -direct_change_thresh) {
    RobotInfo::getPtrInstance()->setRobotMotionState(
        ROBOTMOTIONSTATE::TURN_RIGHT);
  } else {
    RobotInfo::getPtrInstance()->setRobotMotionState(ROBOTMOTIONSTATE::FORWARD);
  }
  return true;
}

size_t PathManager::calcReferIndexWithDistance(const double &dis) {
  // 计算在任务路径中，从当前索引往后搜索n米后对应的索引值是多少
  // 如果查询长度超出路径剩余长度，则返回路径总长度
  std::shared_ptr<SubPath> refer_path = getReferPath();
  size_t current_index = getReferIndex();
  size_t total_size = refer_path->wps.size();
  size_t check_index = refer_path->wps.size();
  double sum_dis = 0.0;
  for (size_t i = current_index; i < total_size; ++i) {
    if (i == 0) {
      continue;
    }
    // sum_dis += refer_path->wps[i - 1].distanceTo(refer_path->wps[i]);
    sum_dis = refer_path->wpis[i].path_length -
              refer_path->wpis[current_index].path_length;
    if (sum_dis >= dis) {
      check_index = i;
      break;
    }
  }
  return check_index;
}

double PathManager::getCompletion() {
  size_t refer_length = getReferPath()->wps.size();
  size_t current_index = getReferIndex();
  return (double) current_index / refer_length;
}

// double PathManager::getReferRemainLength() {
//   std::shared_ptr<SubPath> refer_path = getReferPath();
//   return refer_path->wpis.back().path_length -
//          refer_path->wpis[getReferIndex()].path_length;
//   // std::vector<Pose2d> refer_path = getReferPath().wps;
//   // size_t refer_length = refer_path.size();
//   // size_t current_index = getReferIndex();
//   // double sum_dist = 0.0;
//   // for (size_t i = current_index; i < refer_length - 1; ++i) {
//   //   sum_dist += refer_path[i].distanceTo(refer_path[i + 1]);
//   // }
//   // LOG(INFO) << "Remain refer length: " << sum_dist;
//   // return sum_dist;
// }

void PathManager::setReferIndex(size_t index) {
  std::lock_guard<std::mutex> lock(refer_target_index_mutex_);
  refer_path_index_ = index;
  // updateCtrolPath("refer");
}

size_t PathManager::getReferIndex() {
  std::lock_guard<std::mutex> lock(refer_target_index_mutex_);
  return refer_path_index_;
}

size_t PathManager::getTargetIndex() {
  return getLocalTarget().index;
}

size_t PathManager::getReferPathSize() {
  return refer_path_size_;
}
size_t PathManager::getLocalPathSize() {
  return local_path_size_;
}

void PathManager::updateReferIndex() {
  LOG(INFO) << "update refer index: " << getLocalTarget().index;
  setReferIndex(getLocalTarget().index);
  refer_path_update_flag_ = true;
}

void PathManager::setLocalTarget(const PoseMsg &point) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  local_target_point_ = point;
  LOG(INFO) << "set local target: " << local_target_point_.position.getX()
            << ", " << local_target_point_.position.getY() << ", "
            << local_target_point_.position.getYaw()
            << " at index: " << local_target_point_.index;
}

void PathManager::resetLocalTarget() {
  LOG(INFO) << "resetLocalTarget!";
  PoseMsg init_pose;
  init_pose.position.setPose(DBL_MAX, DBL_MAX, DBL_MAX);
  init_pose.velocity = WayPointInfo(0.0, 0.0, 0.0);
  init_pose.index = 0;
  setLocalTarget(init_pose);
}

void PathManager::resetLocalIndex() {
  local_path_index_ = 0;
}

void PathManager::setLocalIndex(size_t index) {
  local_path_index_ = index;
}

void PathManager::setAvoiderReferIndex(size_t index) {
  std::lock_guard<std::mutex> lock(avoider_refer_index_mutex_);
  avoider_refer_path_index_ = index;
}

size_t PathManager::getLocalIndex() {
  return local_path_index_;
}

size_t PathManager::getAvoiderReferIndex() {
  std::lock_guard<std::mutex> lock(avoider_refer_index_mutex_);
  return avoider_refer_path_index_;
}

PoseMsg PathManager::getLocalTarget() {
  std::lock_guard<std::mutex> lock(target_mutex_);
  return local_target_point_;
}

Pose2d PathManager::getReferLastPoint() {
  return getReferPath()->wps.back();
}

std::shared_ptr<SubPath> PathManager::getLocalPath() {
  std::lock_guard<std::mutex> lock(local_path_mutex_);
  return ptr_local_path_;
}

std::shared_ptr<SubPath> PathManager::getReferPath() {
  std::lock_guard<std::mutex> lock(refer_path_mutex_);
  return ptr_refer_path_;
}

std::shared_ptr<SubPath> PathManager::getCtrolPath() {
  std::lock_guard<std::mutex> lock(ctrol_path_mutex_);
  return ptr_ctrol_path_;
}

// void PathManager::setLocalPath(std::shared_ptr<SubPath> sub_path) {
//   std::lock_guard<std::mutex> lock(local_path_mutex_);
//   ptr_local_path_ = sub_path;
//   ptr_local_path_->type = PathType::LOCAL;
//   updateCtrolPath("local");
// }

// void PathManager::setReferPath(const SubPath &path) {
//   std::lock_guard<std::mutex> lock(refer_path_mutex_);
//   ptr_refer_path_ = std::make_shared<SubPath>(path);
//   ptr_refer_path_->type = PathType::REFER;
//   updateCtrolPath("refer");
// }

void PathManager::setCtrolPath(std::shared_ptr<SubPath> ctrl_path) {
  std::lock_guard<std::mutex> lock(ctrol_path_mutex_);
  ptr_ctrol_path_ = ctrl_path;
  ptr_ctrol_path_->type = PathType::CTROL;
}

bool PathManager::isLocalPathEmpty() {
  std::lock_guard<std::mutex> lock(local_path_mutex_);
  return ptr_local_path_->wps.empty();
}

bool PathManager::isReferPathEmpty() {
  std::lock_guard<std::mutex> lock(refer_path_mutex_);
  return ptr_refer_path_->wps.empty();
}

bool PathManager::isCtrolPathEmpty() {
  std::lock_guard<std::mutex> lock(ctrol_path_mutex_);
  return ptr_ctrol_path_->wps.empty();
}

// bool PathManager::isNewReferPath() {
//   std::lock_guard<std::mutex> lock(refer_path_mutex_);
//   return ptr_refer_path_->isNewPath();
// }

// void PathManager::clearNewReferPathFlag() {
//   std::lock_guard<std::mutex> lock(refer_path_mutex_);
//   ptr_refer_path_->clearNewPathFlag();
// }

double PathManager::getReferPathAngle() {
  std::lock_guard<std::mutex> lock(refer_path_mutex_);
  if (!ptr_refer_path_->wps.empty()) {
    return ptr_refer_path_->wps.front().getYaw();
  }
}

double PathManager::getCtrolPathAngle() {
  std::lock_guard<std::mutex> lock(ctrol_path_mutex_);
  if (!ptr_ctrol_path_->wps.empty()) {
    return ptr_ctrol_path_->wps.front().getYaw();
  }
}

SubPath PathManager::getFullCtrolPath() {
  std::lock_guard<std::mutex> lock(ctrol_path_mutex_);
  return *ptr_ctrol_path_;
}

}  // namespace CVTE_BABOT
