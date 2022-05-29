/*
 * @Author: chennuo@cvte.com
 * @Date: 2021-07-15 21:26:05
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-07-16 15:59:11
 */

#pragma once

#include <iostream>
#include <memory>
#include <queue>

#include "costmap_2d.hpp"

namespace CVTE_BABOT {

static const unsigned char DEFAULT_GRAY_VALUE = 0;  // 表示不带有任何标记信息
static const unsigned char NARROW_GRAY_VALUE = 100;  // 表示标记为窄道

class NarrowRecognition {
 public:
  NarrowRecognition();
  NarrowRecognition(double width, double scale, double inscribe_radius);
  ~NarrowRecognition();

  // 重置参数
  void resetParam(double width, double scale, double inscribe_radius);

  // 识别窄道，并返回标记了窄道的地图
  void recognition(const std::shared_ptr<Costmap2d> &static_costmap,
                   std::shared_ptr<Costmap2d> &marked_map);

 private:
  double narrow_width_{1.2};     // 窄道宽度
  double inflate_scale_{3.0};    // 膨胀地图的梯度因子
  double inscribe_radius_{0.2};  // 机器人的内切半径
};

}  // namespace CVTE_BABOT