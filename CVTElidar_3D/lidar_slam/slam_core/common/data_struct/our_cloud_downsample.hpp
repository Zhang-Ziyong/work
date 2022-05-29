/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file our_cloud_downsample.hpp
 *
 *@brief
 * 1.点云下采样类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/
#ifndef OUR_CLOUD_DOWNSAMPLE_HPP_
#define OUR_CLOUD_DOWNSAMPLE_HPP_

#include <unordered_map>
#include <vector>

#include "common/debug_tools/debug_color.h"
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace cvte_lidar_slam {

class Point {
 public:
  Point(const long int x, const long int y, const long int z)
      : x(x), y(y), z(z) {}
  inline bool operator<(const Point &b) const {
    return this->x < b.x || (this->x == b.x && this->y < b.y) ||
           (this->x == b.x && this->y == b.y && this->z < b.z);
  }
  long int x;
  long int y;
  long int z;
};

class CloudDownsample {
 public:
  CloudDownsample();

  ~CloudDownsample();

  void InsertCloud(const laserCloud::Ptr &cloud_in);

  void setLeafSize(const double x, const double y, const double z);

  void InsertPoint(const PointType &point);

  bool setOutputCloud(laserCloud::Ptr &cloud_out);

  void Clear();

 private:
  std::map<Point, PointType> frame_index_map_;
  double leaf_x_;
  double leaf_y_;
  double leaf_z_;
};

}  // namespace cvte_lidar_slam

#endif  // OUR_CLOUD_DOWNSAMPLE_HPP_