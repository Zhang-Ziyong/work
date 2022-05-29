/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file map_cell.hpp
 *
 *@brief 表征地图单元格的类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-10
 ************************************************************************/

#ifndef __MAP_CELL_HPP
#define __MAP_CELL_HPP

namespace CVTE_BABOT {

#include <limits>

#ifndef DBL_MAX /* Max decimal value of a double */
#define DBL_MAX \
  std::numeric_limits<double>::max() /* 返回编译器允许的 double 型数 最大值 */
#endif

#ifndef DBL_MIN /* Min decimal value of a double*/
#define DBL_MIN \
  std::numeric_limits<double>::min() /* 返回编译器允许的 double 型数 最小值 */
#endif

/**
* MapCell
* @brief
*   整张栅格地图中的一点表示
* */

class MapCell {
 public:
  MapCell() = default;
  MapCell(const MapCell &mc) {
    cx_ = mc.cx_;
    cy_ = mc.cy_;
    target_dist_ = mc.target_dist_;
    target_mark_ = mc.target_mark_;
    within_robot_ = mc.within_robot_;
  }

  MapCell &operator=(const MapCell &mc) {
    if (this == &mc) {
      return *this;
    }
    cx_ = mc.cx_;
    cy_ = mc.cy_;
    target_dist_ = mc.target_dist_;
    target_mark_ = mc.target_mark_;
    within_robot_ = mc.within_robot_;
    return *this;
  }

  void resetCell(double reset_dist) {
    target_dist_ = reset_dist;
    target_mark_ = false;
    within_robot_ = false;
  }

 public:
  unsigned int cx_ = 0;  ///< 在栅格地图中的x坐标
  unsigned int cy_ = 0;  ///< 在栅格地图中的y坐标

  double target_dist_ = DBL_MAX;  ///< 与规划路径的距离

  bool target_mark_ = false;   ///< Marks for computing path/goal distances
  bool within_robot_ = false;  ///< Mark for cells within the robot footprint
};

}  // namespace CVTE_BABOT

#endif  // __MAP_CELL_HPP