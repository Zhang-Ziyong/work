/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file map_limits.hpp
 *
 *@brief
 * 与slam2d_core与ros2的一些数据类型转换函数
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version v1.0
 *@data 2021-04-01
 ************************************************************************/
#ifndef _MAP_LIMITS_H_
#define _MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "math.hpp"
#include "glog/logging.h"
#include "common/rigid_transform.hpp"

#include "proto/cell_limits.pb.h"
#include "proto/map_limits.pb.h"

namespace slam2d_core {
namespace common {

struct CellLimits {
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
      : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  explicit CellLimits(const slam2d::mapping::proto::CellLimits &cell_limits)
      : num_x_cells(cell_limits.num_x_cells()),
        num_y_cells(cell_limits.num_y_cells()) {}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

inline slam2d::mapping::proto::CellLimits toProto(
    const CellLimits &cell_limits) {
  slam2d::mapping::proto::CellLimits result;
  result.set_num_x_cells(cell_limits.num_x_cells);
  result.set_num_y_cells(cell_limits.num_y_cells);
  return result;
}

/**
 * MapLimits
 * @brief 栅格地图相关属性类，以及有关操作; 比如：栅格尺寸，最大姿态，分辨率等.
 **/
class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d &max,
            const CellLimits &cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  explicit MapLimits(const slam2d::mapping::proto::MapLimits &map_limits)
      : resolution_(map_limits.resolution()),
        max_(common::toEigen(map_limits.max())),
        cell_limits_(map_limits.cell_limits()) {}

  /**
   * resolution
   * @brief 获取栅格分辨率
   * @return double-分辨率大小
   **/
  double resolution() const { return resolution_; }

  /**
   * max
   * @brief 获取最大的姿态
   * @return Eigen::Vector2d-最大的姿态
   **/
  const Eigen::Vector2d &max() const { return max_; }

  /**
   * cell_limits
   * @brief 获取栅格大小
   * @return CellLimits-栅格大小
   **/
  const CellLimits &cell_limits() const { return cell_limits_; }

  /**
   * getCellIndex
   * @brief 获取2d姿态，在栅格中的下标
   * @param[in] point-2d姿态
   * @return Eigen::Array2i-栅格地图下标
   **/
  Eigen::Array2i getCellIndex(const Eigen::Vector2d &point) const {
    return Eigen::Array2i(
        common::roundToInt((max_.y() - point.y()) / resolution_ - 0.5),
        common::roundToInt((max_.x() - point.x()) / resolution_ - 0.5));
  }

  /**
   * contains
   * @brief 判断某栅格，是否在栅格地图里面
   * @param[in] cell_index-2d姿态
   * @return bool- true在里面，false不在里面
   **/
  bool contains(const Eigen::Array2i &cell_index) const {
    return (Eigen::Array2i(0, 0) <= cell_index).all() &&
           (cell_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

 private:
  double resolution_;       ///< 栅格分辨率
  Eigen::Vector2d max_;     ///< 最大的姿态
  CellLimits cell_limits_;  ///< 栅格x,y大小尺寸
};

inline slam2d::mapping::proto::MapLimits toProto(const MapLimits &map_limits) {
  slam2d::mapping::proto::MapLimits result;
  result.set_resolution(map_limits.resolution());
  *result.mutable_max() = common::toProto(map_limits.max());
  *result.mutable_cell_limits() = toProto(map_limits.cell_limits());
  return result;
}

class XYIndexRangeIterator
    : public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
 public:
  XYIndexRangeIterator(const Eigen::Array2i &min_xy_index,
                       const Eigen::Array2i &max_xy_index)
      : min_xy_index_(min_xy_index),
        max_xy_index_(max_xy_index),
        xy_index_(min_xy_index) {}

  explicit XYIndexRangeIterator(const CellLimits &cell_limits)
      : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                             Eigen::Array2i(cell_limits.num_x_cells - 1,
                                            cell_limits.num_y_cells - 1)) {}

  XYIndexRangeIterator &operator++() {
    DCHECK(*this != end());
    if (xy_index_.x() < max_xy_index_.x()) {
      ++xy_index_.x();
    } else {
      xy_index_.x() = min_xy_index_.x();
      ++xy_index_.y();
    }
    return *this;
  }

  Eigen::Array2i &operator*() { return xy_index_; }

  bool operator==(const XYIndexRangeIterator &other) const {
    return (xy_index_ == other.xy_index_).all();
  }

  bool operator!=(const XYIndexRangeIterator &other) const {
    return !operator==(other);
  }

  XYIndexRangeIterator begin() {
    return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
  }

  XYIndexRangeIterator end() {
    XYIndexRangeIterator it = begin();
    it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
    return it;
  }

 private:
  Eigen::Array2i min_xy_index_;
  Eigen::Array2i max_xy_index_;
  Eigen::Array2i xy_index_;
};

}  // namespace common
}  // namespace slam2d_core

#endif
