/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file global_planner.cpp
 *
 *@brief cpp模板
 *
 *@author liangjiajun (liangjiajun@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2021-01-05
 ************************************************************************/
#include "global_planner.hpp"

namespace CVTE_BABOT {

bool GlobalPlanner::makePlan(const MakePlanInfo &info, SubPath &path_result) {
  path_result.wps.clear();
  path_result.wpis.clear();
  // 如果起点和终点的直线距离小于4个栅格分辨率，则直接以起点和终点插值出4个点作为此次规划结果
  // Dijkstra目前无法在两个栅格内做规划
  if (info.start_point.distanceTo(info.target_point) <=
      gb_planner_costmap_ptr_->getResolution() * 4) {
    LOG(INFO) << "global plan: too close, use inter points";
    path_result.wps.push_back(info.start_point);
    path_result.wpis.push_back(WayPointInfo(0.0, 0.0, 0.0));
    size_t inter_point = 3;
    double inter_proportion = 1.0 / (inter_point + 1);
    for (size_t i = 1; i <= inter_point; i++) {
      Pose2d point;
      point.setX(info.start_point.getX() +
                 inter_proportion * i *
                     (info.target_point.getX() - info.start_point.getX()));
      point.setY(info.start_point.getY() +
                 inter_proportion * i *
                     (info.target_point.getY() - info.start_point.getY()));
      path_result.wps.push_back(point);
      path_result.wpis.push_back(WayPointInfo(0.0, 0.0, 0.0));
    }
    path_result.wps.push_back(info.target_point);
    path_result.wpis.push_back(WayPointInfo(0.0, 0.0, 0.0));
    return true;
  }

  WorldmapPoint start_wm_point = {info.start_point.getX(),
                                  info.start_point.getY()};
  CostmapPoint start_cp_point;
  double dStart_x = 0.0, dStart_y = 0.0, dGoal_x = 0.0, dGoal_y = 0.0;
  if (!gb_planner_costmap_ptr_->worldToMap(start_wm_point, start_cp_point)) {
    LOG(WARNING) << "Start pos is:" << info.start_point.getX() << ", "
                 << info.start_point.getY()
                 << ". The robot's start position is off the global costmap. "
                    "Planning will always fail, are you sure the robot has "
                    "been properly localized?";
    return false;
  }
  dStart_x = (double) start_cp_point.ui_x;
  dStart_y = (double) start_cp_point.ui_y;

  WorldmapPoint target_wm_point = {info.target_point.getX(),
                                   info.target_point.getY()};
  CostmapPoint target_cp_point;
  if (!gb_planner_costmap_ptr_->worldToMap(target_wm_point, target_cp_point)) {
    LOG(WARNING) << "Target pos is: " << info.target_point.getX() << ", "
                 << info.target_point.getY()
                 << ", The goal sent to the global planner is off the global "
                    "costmap. Planning will always fail to this goal.";
    return false;
  }
  LOG(INFO) << "Ax from (" << info.start_point.getX() << ", "
            << info.start_point.getY() << ", " << info.start_point.getYaw()
            << ") to (" << info.target_point.getX() << ", "
            << info.target_point.getY() << ", " << info.target_point.getYaw()
            << ")";

  dGoal_x = (double) target_cp_point.ui_x;
  dGoal_y = (double) target_cp_point.ui_y;

  Pose2d path_point;
  SubPath temp_path_result;
  double path_point_x = 0.0;
  double path_point_y = 0.0;
  Pose2d start_point(dStart_x, dStart_y, 0.0);
  Pose2d target_point(dGoal_x, dGoal_y, 0.0);
  // 设置路径采样密度
  auto char_map = gb_planner_costmap_ptr_->getCharMap();
  if (!(makePlan(char_map, start_point, target_point, temp_path_result))) {
    int start_cost = gb_planner_costmap_ptr_->getCostWithMapPose(
        start_point.getX(), start_point.getY());
    int target_cost = gb_planner_costmap_ptr_->getCostWithMapPose(
        target_point.getX(), target_point.getY());
    LOG(ERROR) << "MakePlan Failed. Start cost: " << start_cost
               << "  target cost: " << target_cost;
  } else {
    SubPath dijk_path;
    // 注意，D*在势场中搜索路径是从后往前找的，所以push进去时要倒着放
    dijk_path.wps.push_back(info.start_point);
    // dijk_path.wpis.push_back(WayPointInfo(0.0, 0.0, 0.0));
    for (int i = temp_path_result.wps.size() - 1; i >= 0; --i) {
      // 注意，由于路径是经过势场平滑，这里直接用整数表示路径可能会丢失精度，为了接口一致
      pathPointToWorld(temp_path_result.wps[i].getX(),
                       temp_path_result.wps[i].getY(), path_point_x,
                       path_point_y);
      path_point.setPose(path_point_x, path_point_y, 0.0);
      dijk_path.wps.push_back(path_point);
      // dijk_path.wpis.push_back(
      //     WayPointInfo(0.0, 0.0, 0.0));  // 默认先将速度数值置为零
    }
    // 是否需要将规划的目标点在最后插入
    dijk_path.wps.push_back(info.target_point);
    // dijk_path.wpis.push_back(WayPointInfo(0.0, 0.0, 0.0));

    smoothPath(dijk_path.wps, path_result.wps);
    path_result.wpis.resize(path_result.wps.size());
    // 计算路径点中的角度信息
    calcPathPointAngle(path_result.wps, info.target_point);

    LOG(INFO) << "Original d paht:";
    for (auto pp : path_result.wps) {
      LOG(INFO) << pp.getX() << ", " << pp.getY() << ", " << pp.getYaw();
    }
    return (path_result.wps.size() > 0);
  }
  return false;
}

void GlobalPlanner::calcPathPointAngle(std::vector<Pose2d> &path_result,
                                       const Pose2d &target) {
  if (path_result.empty()) {
    return;
  }
  double yaw = 0.0;
  int path_size = path_result.size() - 1;
  // 角度的计算，从零开始是会修改起点朝向（而不是当前位置朝向）
  for (int i = 0; i <= path_size; ++i) {
    // 如果是最后一个点，则将目标点的角度设置为目最后一个点的朝向
    // 0 到 n - 1点的朝向，通过两两之间朝向计算
    if (i == path_size) {
      path_result[i].setYaw(target.getYaw());
    } else {
      // atan2(a,b)的取值范围介于 -pi 到 pi 之间（不包括 -pi）
      // 而atan(a/b)的取值范围介于-pi/2到pi/2之间（不包括±pi/2)
      yaw = atan2((path_result[i + 1].getY() - path_result[i].getY()),
                  (path_result[i + 1].getX() - path_result[i].getX()));
      path_result[i].setYaw(yaw);
    }
  }
}

void GlobalPlanner::smoothPath(const std::vector<Pose2d> &origin_path,
                               std::vector<Pose2d> &smooth_path) {
  if (origin_path.empty()) {
    LOG(ERROR) << "origin_path is empty";
    return;
  }
  Pose2d last_point = origin_path.front();
  smooth_path.emplace_back(last_point);  // 起点
  double limit_distance = 0.4;
  double inter_distance = 0.2;

  for (int i = 0; i < origin_path.size(); i++) {
    const Pose2d &cur_point = origin_path[i];
    double distance = last_point.distanceTo(cur_point);

    // 加大差值范围
    if (distance >= limit_distance) {
      // 计算极坐标角度
      double theta =
          atan2((cur_point.y - last_point.y), (cur_point.x - last_point.x));
      last_point = Pose2d(inter_distance * cos(theta) + last_point.x,
                          inter_distance * sin(theta) + last_point.y, 0);
      smooth_path.emplace_back(last_point);
    }
  }
  if (origin_path.back().distanceTo(smooth_path.back()) > 0.2) {
    double theta = atan2((origin_path.back().y - smooth_path.back().y),
                         (origin_path.back().x - smooth_path.back().x));
    last_point = Pose2d(inter_distance * cos(theta) + smooth_path.back().x,
                        inter_distance * sin(theta) + smooth_path.back().y, 0);
    smooth_path.emplace_back(last_point);
  }
  smooth_path.emplace_back(origin_path.back());
}

void GlobalPlanner::pathPointToWorld(const double &d_mx, const double &d_my,
                                     double &d_wx, double &d_wy) {
  double d_convert_offset = 0.0;  // 偏移量
  d_wx = gb_planner_costmap_ptr_->getOriginX() +
         (d_mx + d_convert_offset) * gb_planner_costmap_ptr_->getResolution();
  d_wy = gb_planner_costmap_ptr_->getOriginY() +
         (d_my + d_convert_offset) * gb_planner_costmap_ptr_->getResolution();
}

}  // namespace CVTE_BABOT
