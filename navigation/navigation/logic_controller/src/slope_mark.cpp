/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file slope_mark.hpp
 *
 *@brief 用于判断机器人是否在某个区域内
 *       如果是在某一区域内时，导航需要相应做出减速等操作
 *
 *@modified by liangjiajun(liangjiajun@cvte.com)
 *
 *@author xiesongde(xiesongde@cvte.com)
 *@version current_algo.dev
 *@data 2020-07-20
 ************************************************************************/

#include "slope_mark.hpp"

#include <float.h>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <glog/logging.h>
#define EPSILON 0.000001

namespace CVTE_BABOT {

bool SlopeMark::isPointOnLine(double px0, double py0, double px1, double py1,
                              double px2, double py2) {
  bool flag = false;
  double d1 = (px1 - px0) * (py2 - py0) - (px2 - px0) * (py1 - py0);
  if ((abs(d1) < EPSILON) && ((px0 - px1) * (px0 - px2) <= 0) &&
      ((py0 - py1) * (py0 - py2) <= 0)) {
    flag = true;
  }
  return flag;
}

bool SlopeMark::isIntersect(double px1, double py1, double px2, double py2,
                            double px3, double py3, double px4, double py4) {
  bool flag = false;
  double d = (px2 - px1) * (py4 - py3) - (py2 - py1) * (px4 - px3);
  if (d != 0) {
    double r = ((py1 - py3) * (px4 - px3) - (px1 - px3) * (py4 - py3)) / d;
    double s = ((py1 - py3) * (px2 - px1) - (px1 - px3) * (py2 - py1)) / d;
    if ((r >= 0) && (r <= 1) && (s >= 0) && (s <= 1)) {
      flag = true;
    }
  }
  return flag;
}

bool SlopeMark::pointInPolygon2D(double x, double y,
                                 const std::vector<Pose2d> &pol) {
  bool is_inside = false;
  int count = 0;

  //
  double min_x = DBL_MAX;
  for (uint32_t i = 0; i < pol.size(); i++) {
    min_x = std::min(min_x, pol[i].x);
  }

  //
  double px = x;
  double py = y;
  double line_point1_x = x;
  double line_point1_y = y;
  double line_point2_x = min_x - 10;  //取最小的X值还小的值作为射线的终点
  double line_point2_y = y;

  //遍历每一条边
  for (uint32_t i = 0; i < pol.size() - 1; i++) {
    double cx1 = pol[i].x;
    double cy1 = pol[i].y;
    double cx2 = pol[i + 1].x;
    double cy2 = pol[i + 1].y;

    if (isPointOnLine(px, py, cx1, cy1, cx2, cy2)) {
      return true;
    }

    //平行则不相交
    if (fabs(cy2 - cy1) < EPSILON) {
      continue;
    }

    if (isPointOnLine(cx1, cy1, line_point1_x, line_point1_y, line_point2_x,
                      line_point2_y)) {
      //只保证上端点+1
      if (cy1 > cy2) {
        count++;
      }
    } else if (isPointOnLine(cx2, cy2, line_point1_x, line_point1_y,
                             line_point2_x, line_point2_y)) {
      //只保证上端点+1
      if (cy2 > cy1) {
        count++;
      }
    } else if (isIntersect(cx1, cy1, cx2, cy2, line_point1_x, line_point1_y,
                           line_point2_x, line_point2_y)) {
      //已经排除平行的情况
      count++;
    }
  }

  if (count % 2 == 1) {
    is_inside = true;
  }

  return is_inside;
}

bool SlopeMark::getSlope(Json::Value slope_json) {
  std::lock_guard<std::mutex> lock(slopes_mutex_);
  slopes_vec_.clear();
  // 添加第一个点让首尾相连
  if (slope_json.size() != 0) {
    slope_json.append(slope_json[0]);
  }

  for (uint32_t i = 0; i < slope_json.size(); i++) {
    Pose2d pose;
    std::vector<Pose2d> poses_vec;

    for (auto point_json : slope_json[i]["points"]) {
      LOG(INFO) << "x:" << point_json["x"].asDouble()
                << ",y:" << point_json["y"].asDouble();
      pose.x = point_json["x"].asDouble();
      pose.y = point_json["y"].asDouble();
      poses_vec.push_back(pose);
    }
    slopes_vec_.push_back(poses_vec);
  }
  return true;
}

bool SlopeMark::isInSlope(double x, double y) {
  std::lock_guard<std::mutex> lock(slopes_mutex_);
  for (uint32_t i = 0; i < slopes_vec_.size(); i++) {
    if (pointInPolygon2D(x, y, slopes_vec_[i])) {
      return true;
    }
  }
  return false;
}
}  // namespace CVTE_BABOT