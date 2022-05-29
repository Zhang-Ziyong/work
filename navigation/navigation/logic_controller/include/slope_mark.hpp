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

#ifndef __SLOPE_MARK_HPP
#define __SLOPE_MARK_HPP
#include <string>
#include <vector>
#include <mutex>
#include <jsoncpp/json/json.h>

#include "pose2d/pose2d.hpp"
namespace CVTE_BABOT {
class SlopeMark {
 public:
  bool isInSlope(double x, double y);
  bool getSlope(Json::Value slope_json);

 private:
  //判断点在线段上
  bool isPointOnLine(double px0,
                     double py0,
                     double px1,
                     double py1,
                     double px2,
                     double py2);
  //判断两线段相交
  bool isIntersect(double px1,
                   double py1,
                   double px2,
                   double py2,
                   double px3,
                   double py3,
                   double px4,
                   double py4);
  //判断点在多边形内
  bool pointInPolygon2D(double x, double y, const std::vector<Pose2d>& pol);
  std::vector<std::vector<Pose2d>> slopes_vec_;
  std::mutex slopes_mutex_;
};

}  // namespace CVTE_BABOT
#endif  // __SLOPE_MARK_HPP
