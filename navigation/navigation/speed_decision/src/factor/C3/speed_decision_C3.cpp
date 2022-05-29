/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file speed_decision_C3.cpp
 *
 *@brief 速度决策类(根据C3特性重写部分函数)
 *
 *@author chenweijian chenweijian@cvte.com
 *@modified
 *@version
 *@data
 ************************************************************************/
#include "C3/speed_decision_C3.hpp"
#include "glog/logging.h"
#include "pnc_map.hpp"
#include "robot_info.hpp"

#define LOG_HZ_ 20

namespace CVTE_BABOT {

double SpeedDecisionC3::getCurSpeedFactor() {
  if (isObstacleInStopBound()) {
    LOG(ERROR) << "point in obstacle bound.......";
    return 0;
  }

  double cur_pose_factor = getCurPoseCostFactor();
  double path_speed_factor = getPathSlowFactor();  // 路径上机器与障碍物的距离
  double marker_factor = getMarkerFactor();  // 部署时设定的减速带
  // double camera_factor = getCameraFactor();  // 相机点云判断的减速

  double infrared_factor = getInfraredObstacleFactor();  //红外减速因子
  double slope_factor = getSlopeFactor();  // 机器倾斜(斜坡上的减速)

  LOG(INFO) << "infrared factor c3 " << infrared_factor;
  LOG_EVERY_N(INFO, LOG_HZ_) << "path_speed_factor: " << path_speed_factor;
  LOG_EVERY_N(INFO, LOG_HZ_) << "cur_pose_factor: " << cur_pose_factor;
  LOG_EVERY_N(INFO, LOG_HZ_) << "marker factor: " << marker_factor;
  // LOG_EVERY_N(INFO, LOG_HZ_) << "camera factor: " << camera_factor;
  LOG_EVERY_N(INFO, LOG_HZ_) << "slpoe factor: " << slope_factor;
  LOG_EVERY_N(INFO, LOG_HZ_) << "infrared factor: " << infrared_factor;

  double min_factor = std::min({path_speed_factor, cur_pose_factor,
                                marker_factor, slope_factor, infrared_factor});

  return min_factor;
}

bool SpeedDecisionC3::isObstacleInStopBound() {
  // 清空停障框增量
  stop_shape_.clearInc();

  // 获取贴边标志
  getEdgeFlag();

  // 贴边特殊处理
  if (edge_flag_) {
    if (refer_path_->wpis[refer_index_].edge_dist > 0) {
      stop_shape_.setInc(SHAPE_LEFT, -0.03);
    } else {
      stop_shape_.setInc(SHAPE_RIGHT, -0.03);
    }
  }

  // scan
  std::vector<Eigen::Vector2d> show_crash_point;
  std::lock_guard<std::mutex> lock_scan(scan_mutex_);
  if (stop_shape_.checkPonitInside(scan_, "all", &show_crash_point)) {
    LOG(WARNING) << "scan stop x " << show_crash_point[0](0) << " y "
                 << show_crash_point[0](1);
    return true;
  }

  // 代价地图
  auto ptr_pnc_map = std::make_shared<PncMap>();
  int cur_cost = getCurPoseCost();
  if (cur_cost > 252) {
    // 获取旋转矩阵
    double x_max = INT_MIN;
    double x_min = INT_MAX;
    double y_max = INT_MIN;
    double y_min = INT_MAX;
    auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
    Eigen::Matrix<double, 2, 3> T_wb;
    T_wb << cos(cur_pose.getYaw()), -sin(cur_pose.getYaw()), cur_pose.getX(),
        sin(cur_pose.getYaw()), cos(cur_pose.getYaw()), cur_pose.getY();

    //贴边特殊处理
    if (edge_flag_) {
      if (refer_path_->wpis[refer_index_].edge_dist > 0) {
        stop_shape_.setInc(SHAPE_LEFT, -0.07);
      } else {
        stop_shape_.setInc(SHAPE_RIGHT, -0.07);
      }
      stop_shape_.getBoundingbox(x_max, x_min, y_max, y_min);

      for (double x = x_min; x < x_max;
           x += ptr_config_->costmap_traverse_inc) {
        for (double y = y_min; y < y_max;
             y += ptr_config_->costmap_traverse_inc) {
          Eigen::Vector2d point_in_world =
              T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
              T_wb.block<2, 1>(0, 2);
          if (checkPointCostValue(point_in_world) == 254) {
            if (stop_shape_.checkPonitInside(x, y)) {
              LOG(WARNING) << "costmap stop point: " << x << ", " << y;
              return true;
            }
          }
        }
      }
    }
    // 窄通道特殊处理
    else if (ptr_pnc_map->InNarrowArea(
                 math_utils::Vec2d(cur_pose.getX(), cur_pose.getY()))) {
      stop_shape_.setInc(SHAPE_LEFT, -0.05);
      stop_shape_.setInc(SHAPE_RIGHT, -0.05);
      stop_shape_.getBoundingbox(x_max, x_min, y_max, y_min);

      for (double x = x_min; x < x_max;
           x += ptr_config_->costmap_traverse_inc) {
        for (double y = y_min; y < y_max;
             y += ptr_config_->costmap_traverse_inc) {
          Eigen::Vector2d point_in_world =
              T_wb.block<2, 2>(0, 0) * Eigen::Vector2d(x, y) +
              T_wb.block<2, 1>(0, 2);
          if (checkPointCostValue(point_in_world) == 254) {
            if (stop_shape_.checkPonitInside(x, y)) {
              LOG(WARNING) << "costmap stop point: " << x << ", " << y;
              return true;
            }
          }
        }
      }
    }
    // 正常通道时
    else {
      LOG(WARNING) << "cur cost " << cur_cost;
      return true;
    }
  }
  return false;
}

double SpeedDecisionC3::getCurPoseCostFactor() {
  if (edge_flag_) {
    return 1.0;
  }
  int cur_cost = getCurPoseCost();
  return 0.5 + ((double) (255 - cur_cost) / 255) * 0.5;
};

int SpeedDecisionC3::getCurPoseCost() {
  auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
  Eigen::Vector2d point_in_world(cur_pose.x, cur_pose.y);
  int cost = checkPointCostValue(point_in_world);
  cost = cost > 255 ? 0 : cost;
  return cost;
}

double SpeedDecisionC3::getRecoverFactor(
    const RecoveDir dir_key, Eigen::Matrix<double, 2, 3> *T_wb_ptr) {
  // 获取并校验图层
  if (m_recover_coverage_.find(dir_key) == m_recover_coverage_.end()) {
    LOG(ERROR) << "can`t find recover coverage";
    return 255.0;
  }
  std::string dir = m_recover_coverage_[dir_key];
  // get T
  Eigen::Matrix<double, 2, 3> T_wb;
  if (T_wb_ptr == nullptr) {
    auto cur_pose = RobotInfo::getPtrInstance()->getCurrentPose();
    T_wb << cos(cur_pose.getYaw()), -sin(cur_pose.getYaw()), cur_pose.getX(),
        sin(cur_pose.getYaw()), cos(cur_pose.getYaw()), cur_pose.getY();
    T_wb_ptr = &T_wb;
  }

  // danger
  danger_shape_.clearInc();
  // scan
  {
    std::lock_guard<std::mutex> lock_scan(scan_mutex_);
    std::vector<Eigen::Vector2d> crash_point;
    if (scan_.empty()) {
      LOG(ERROR) << "scan size error";
    }
    if (danger_shape_.checkPonitInside(scan_, dir, &crash_point)) {
      LOG(WARNING) << dir << " scan can`t go " << dir << " x "
                   << crash_point[0](0) << " y " << crash_point[0](1);
      return 254.0;
    }
  }

  // costmap
  {
    // get boundingbox
    double x_max = INT_MIN;
    double x_min = INT_MAX;
    double y_max = INT_MIN;
    double y_min = INT_MAX;
    danger_shape_.getBoundingbox(x_max, x_min, y_max, y_min);

    auto v_danger_point = danger_shape_.getStopShape(dir);
    for (int i = 0; i < v_danger_point.size(); i++) {
      if (v_danger_point[i](0) < x_max && v_danger_point[i](0) > x_min &&
          v_danger_point[i](1) < y_max && v_danger_point[i](1) > y_min) {
        Eigen::Vector2d point_in_world =
            T_wb_ptr->block<2, 2>(0, 0) * v_danger_point[i] +
            T_wb_ptr->block<2, 1>(0, 2);

        if (checkPointCostValue(point_in_world) == 254) {
          LOG(WARNING) << dir << " costmap stop point: " << v_danger_point[i](0)
                       << " " << v_danger_point[i](1);
          return 254.0;
        }
      }
    }
  }

  // recover
  // costmap
  {
    double cost_sum = 0;
    std::vector<Eigen::Vector2d> *v_recover_point_ptr = nullptr;
    recover_shape_.getStopShapeOriginal(&v_recover_point_ptr, dir);
    size_t v_recover_point_size = v_recover_point_ptr->size();

    // 计算因子
    for (int i = 0; i < v_recover_point_size; i++) {
      Eigen::Vector2d point_in_world =
          T_wb_ptr->block<2, 2>(0, 0) * (*v_recover_point_ptr)[i] +
          T_wb_ptr->block<2, 1>(0, 2);
      int cost = checkPointCostValue(point_in_world);
      cost_sum += cost;
    }
    if (v_recover_point_size == 0) {
      LOG(ERROR) << dir << " v_recover_point_size error";
    }
    return cost_sum / v_recover_point_size;
  }
  return 0.0;
}

bool SpeedDecisionC3::checkFrontDanger() {
  return isObstacleInStopBound();
}

bool SpeedDecisionC3::checkOutofRecovery() {
  // TODO: 修改完成脱困判断
  if (!isObstacleInStopBound()) {
    return (getCurPoseCost() < 252 &&
            getRecoverFactor(RecoveDir::RECOVER_FRONT) < 251);
  }
  return false;
}

}  // namespace CVTE_BABOT