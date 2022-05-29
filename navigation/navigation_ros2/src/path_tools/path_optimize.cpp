/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file path_optimize.hpp
 *
 *@brief
 * 对录制路径进行优化的具体实现
 *
 *@modified by jiajun liang(linjiajun@cvte.com)
 *
 *@author jiajun liang(linjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2019-07-15
 ************************************************************************/

#include "path_tools/path_optimize.hpp"
#include <glog/logging.h>

const int MAX_ITERATOR_TIME = 300; // 迭代优化最大次数

namespace CVTE_BABOT {

bool PathOptimize::optimizeMissionPath(const std::vector<Pose2d> &origin_path,
                                       std::vector<Pose2d> &optimize_path) {
  if (origin_path.empty()) {
    LOG(ERROR) << "Can't optimize empty mission path.";
    return false;
  }
  optimize_path.clear();
  // 检查递归迭代次数，避免死循环一直迭代
  if (iterator_times_++ > MAX_ITERATOR_TIME) {
    LOG(ERROR) << "Couldn't Optimize this path.";
    iterator_times_ = 0;
    return false;
  }
  LOG(INFO) << "------------";
  for (auto point : origin_path) {
    LOG(INFO) << "(" << point.getX() << ", " << point.getY() << ", "
              << point.getYaw() << ")";
  }
  LOG(INFO) << "************";

  CrossPoints cross_points_result;
  if (findUglyPose(origin_path, cross_points_result)) {
    std::vector<Pose2d> opt_result;
    std::vector<Pose2d> temp;
    createOptimizePath(cross_points_result, opt_result);
    // 直线插值出来的直线，由于可能头尾与原来是相反的，所以要判断一下头尾（用直线距离判断）
    double head_distance =
        twoPointDistance(cross_points_result.origin_pose, opt_result.front());
    double tail_distance =
        twoPointDistance(cross_points_result.origin_pose, opt_result.back());
    LOG(INFO) << "head_distance = " << head_distance
              << "  tail_distance = " << tail_distance;
    // 原始路径中没有出现折线的前半部分，直接保留到优化结果中
    for (int i = 0; i < cross_points_result.origin_pose_index; ++i) {
      optimize_path.push_back(origin_path[i]);
    }
    // 将原始路径中折线部分的地方通过直线插值的结果进行替换
    for (size_t i = 0; i < opt_result.size(); ++i) {
      if (head_distance > tail_distance) {
        optimize_path.push_back(opt_result[opt_result.size() - i - 1]);
      } else {
        optimize_path.push_back(opt_result[i]);
      }
      LOG(INFO) << "(" << opt_result[i].getX() << ", " << opt_result[i].getY()
                << ")";
    }
    // 原始路径中出现折线后的部分，不管是否还存在折线，都保留在优化结果中
    for (size_t i = cross_points_result.last_pose_index; i < origin_path.size();
         ++i) {
      optimize_path.push_back(origin_path[i]);
    }
    // 通过递归的方法，继续检查剩下的路径，直到没有折线为止
    optimizeMissionPath(optimize_path, temp);
    // 递归结果保存至优化结果中
    optimize_path = temp;
  } else {
    optimize_path = origin_path;
    LOG(INFO) << "Optimize Finish. From " << origin_path.size() << " to "
              << optimize_path.size() << " with " << iterator_times_
              << " times";
    iterator_times_ = 0;
  }
  return true;
}

bool PathOptimize::findUglyPose(const std::vector<Pose2d> &origin_path,
                                CrossPoints &cross_points) {
  // int step_size = 0.3 / path_resolution_; //
  // 检查原始路径时，每隔0.3米检查一个点
  int next_check_size =
      0.3 / path_resolution_; // 发现一个相交点之后，往后0.3米的距离继续查找

  for (size_t i = 0; i < origin_path.size(); i++) {
    cross_points.origin_pose = origin_path[i]; // 以此时检查点作为圆心记录
    cross_points.origin_pose_index = i;        // 将检查点的索引记录
    for (size_t j = i; j < origin_path.size(); j++) {
      if (isOnArcEdge(origin_path[i], origin_path[j])) {
        cross_points.cross_points.push_back(origin_path[j]);
        LOG(INFO) << "Cross point at " << j << " is " << origin_path[j].getX()
                  << ", " << origin_path[j].getY();
        // 记录最后一个相交点，后续插值会以最后一个交点作为终点
        cross_points.last_pose_index = j;
        // 往后拉一定距离继续检查，避免找到多个尖锐点
        j += next_check_size;
      }
    }
    // 如果路径与圆弧的相交点超过3个，代表存在折型路径，正常情况只会有一个相交点
    if (cross_points.cross_points.size() >= 3) {
      LOG(INFO) << "Ok, Find " << cross_points.cross_points.size()
                << " cross points.";
      return true;
    } else {
      LOG(INFO) << "Clear!";
      cross_points.cross_points.clear();
    }
  }
  return false;
}

double PathOptimize::twoPointDistance(const Pose2d &point_a,
                                      const Pose2d &point_b) {
  return hypot(point_a.getX() - point_b.getX(),
               point_a.getY() - point_b.getY());
}

bool PathOptimize::isOnArcEdge(const Pose2d &origin, const Pose2d &side_point) {
  double range = twoPointDistance(origin, side_point);
  // 如果边点与圆心的直线距离约等于半径的话，则认为该点与圆弧边相交
  if ((range <= optimize_radius_ + path_resolution_) &&
      (range >= optimize_radius_ - path_resolution_)) {
    LOG(INFO) << "range = " << range;
    return true;
  }
  return false;
}

void PathOptimize::setCrycleRadius(const double &r) {
  if (r <= 0.001) {
    return;
  }
  optimize_radius_ = r;
}

void PathOptimize::setPathResolution(const double &l) {
  if (l <= 0.001) {
    return;
  }
  path_resolution_ = l;
}

bool PathOptimize::createOptimizePath(const CrossPoints &points,
                                      std::vector<Pose2d> &result) {
  // TODO：现在优化路径是直接用直线来连接，后续优化可以选用贝塞尔之类的平滑
  Pose2d start = points.origin_pose;
  Pose2d end = points.cross_points.back();
  LOG(INFO) << "Optimize From (" << start.getX() << ", " << start.getY()
            << ") to (" << end.getX() << ", " << end.getY() << ")";
  if (start == end) {
    LOG(ERROR) << "Start point and end point is the same.";
    return false;
  }
  result.clear();
  // y = kx + b
  double k = (start.getY() - end.getY()) / (start.getX() - end.getX());
  double b = start.getY() - k * start.getX();
  double insert_start = 0.0;
  double insert_end = 0.0;
  Pose2d optimize_point;
  bool is_insert_at_x = false;
  if (abs(start.getY() - end.getY()) > abs(start.getX() - end.getX())) {
    // Y 轴开始插值
    if (start.getY() < end.getY()) {
      insert_start = start.getY();
      insert_end = end.getY();
    } else {
      insert_start = end.getY();
      insert_end = start.getY();
    }
    is_insert_at_x = false;
  } else {
    // X 轴开始插值
    if (start.getX() < end.getX()) {
      insert_start = start.getX();
      insert_end = end.getX();
    } else {
      insert_start = end.getX();
      insert_end = start.getX();
    }
    is_insert_at_x = true;
  }
  // 根据标志位，从X或Y值小的那端，以路径分辨率大小进行逐步插值，直至最后一个点
  // TODO： 目前插值出来的路径没有角度
  while (insert_start < insert_end) {
    if (is_insert_at_x) {
      // 根据x插值，求y，则： y = kx + b
      double opt_y = k * insert_start + b;
      optimize_point.setPose(insert_start, opt_y, start.getYaw());
      insert_start += path_resolution_;
      result.push_back(optimize_point);
    } else {
      // 根据y插值，求x，则 x = (y - b) / k;
      double opt_x = (insert_start - b) / k;
      optimize_point.setPose(opt_x, insert_start, start.getYaw());
      insert_start += path_resolution_;
      result.push_back(optimize_point);
    }
  }
  return true;
}
} // namespace CVTE_BABOT