/*
 * @Author: chennuo@cvte.com
 * @Date: 2021-06-21 18:31:56
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-06-22 18:22:39
 */

#pragma once

#include <vector>
#include <jsoncpp/json/json.h>
#include <memory>

#include "costmap_2d.hpp"
#include "polygon.hpp"

namespace CVTE_BABOT {

enum MarkerType { DEFAULT, STRIP };

// 定义膨胀过程中需要的体素栅格数据结构
struct GridData {
  size_t idx;     //当前体素hash值
  size_t x, y;    // 当前体素索引
  size_t sx, sy;  // 当前体素源膨胀体素索引
  GridData() : idx(0), x(0), y(0), sx(0), sy(0) {}
  GridData(size_t idx, size_t x, size_t y, size_t sx, size_t sy) {
    this->idx = idx;
    this->x = x;
    this->y = y;
    this->sx = sx;
    this->sy = sy;
  }
};

// 定义不同标记的代价值
static const uint8_t DEFAULT_VALUE = 0;
static const uint8_t STRIP_VALUE = 10;

class MarkerMap {
 public:
  MarkerMap();
  ~MarkerMap();

  // 设置地图参数来初始化地图
  void resetMap(const size_t &size_x, const size_t &size_y,
                const double &resolution, const double &origin_x,
                const double &origin_y);

  // 根据代价地图参数初始化地图
  void resetMap(const std::shared_ptr<Costmap2d> &costmap);

  // bresenham算法确定线段上的栅格点
  void bresenham(const int &sx, const int &sy, const int &tx, const int &ty,
                 std::vector<std::pair<int, int>> &indexs);

  // 膨胀多边形边界
  void inflateBorder(const std::vector<Point2d<size_t>> &edge_points,
                     uint8_t value);

  // 设置减速带标记
  void setStripsMarker(const Json::Value &json_file);

  // 根据地图索引获取当前状态
  MarkerType getMarkerType(size_t x, size_t y);

  // 根据实际机器人坐标获取当前状态
  MarkerType getMarkerType(double x, double y);

  // 获取标记地图
  std::shared_ptr<Costmap2d> getMarkerMap() { return ptr_map_; }

  // 判断地图是否已经设置好
  inline bool mapIsOk() { return has_set_map_; }

 private:
  std::vector<std::vector<Point2d<double>>> polygons_;  // 多边形顶点数组
  std::shared_ptr<Costmap2d> ptr_map_{
      nullptr};  // 标记地图数据(直接用代价地图数据结构)
  double inflation_radius_{0.8};  // 膨胀半径
  bool has_set_map_{false};       // 是否已经更新地图的标志
};

}  // namespace CVTE_BABOT