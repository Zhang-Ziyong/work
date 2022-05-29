/*
 * @Author: chennuo@cvte.com
 * @Date: 2021-07-16 10:12:32
 * @Last Modified by: chennuo@cvte.com
 * @Last Modified time: 2021-07-16 21:07:17
 */

#include <glog/logging.h>

#include "costmap_utils.hpp"
#include "narrow_recognition.hpp"

namespace CVTE_BABOT {
NarrowRecognition::NarrowRecognition() {}
NarrowRecognition::NarrowRecognition(double width, double scale,
                                     double inscribe_radius)
    : narrow_width_(width),
      inflate_scale_(scale),
      inscribe_radius_(inscribe_radius) {
  LOG(INFO) << "narrow recognition param: narrow_width->" << narrow_width_
            << " inflate_scale->" << inflate_scale_ << " inscribe_radius->"
            << inscribe_radius_;
}

NarrowRecognition::~NarrowRecognition() {}

void NarrowRecognition::resetParam(double width, double scale,
                                   double inscribe_radius) {
  narrow_width_ = width;
  inflate_scale_ = scale;
  inscribe_radius_ = inscribe_radius;
}

void NarrowRecognition::recognition(
    const std::shared_ptr<Costmap2d> &static_costmap,
    std::shared_ptr<Costmap2d> &marked_map) {
  if (!static_costmap) {
    LOG(WARNING) << "no static costmap, cant recognition narrow !";
    return;
  }

  // 重置标记地图为默认值
  marked_map.reset(new Costmap2d(
      static_costmap->getSizeInCellsX(), static_costmap->getSizeInCellsY(),
      static_costmap->getResolution(), static_costmap->getOriginX(),
      static_costmap->getOriginY(), DEFAULT_GRAY_VALUE));

  // 计算窄道分界点代价值
  uint8_t narrow_cost = 0;
  double narrow_width_2 = narrow_width_ / 2.0;
  if (narrow_width_2 <= inscribe_radius_) {
    narrow_cost = INSCRIBED_INFLATED_OBSTACLE;
  } else {
    double factor =
        exp(-1.0 * inflate_scale_ * (narrow_width_2 - inscribe_radius_));
    narrow_cost =
        static_cast<uint8_t>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
  }
  LOG(INFO) << "narrow cost: " << int(narrow_cost);

  // 提取窄道分界点
  std::vector<std::pair<size_t, size_t>> break_points;
  for (size_t mx = 0; mx < static_costmap->getSizeInCellsX(); mx++) {
    for (size_t my = 0; my < static_costmap->getSizeInCellsY(); my++) {
      uint8_t cur_cost = static_costmap->getCost(mx, my);
      if (cur_cost >= narrow_cost && cur_cost < INSCRIBED_INFLATED_OBSTACLE) {
        // 检查上下左右八个临近栅格的代价值，确认是否不存在梯度下降，如果是的，则当前访问点为窄道临界点
        if (my + 1 < static_costmap->getSizeInCellsY()) {
          // 左上
          if (mx > 1 && static_costmap->getCost(mx - 1, my + 1) < cur_cost)
            continue;
          // 中上
          if (static_costmap->getCost(mx, my + 1) < cur_cost)
            continue;
          // 右上
          if (mx + 1 < static_costmap->getSizeInCellsX() &&
              static_costmap->getCost(mx + 1, my + 1) < cur_cost)
            continue;
        }
        if (my > 1) {
          // 左下
          if (mx > 1 && static_costmap->getCost(mx - 1, my - 1) < cur_cost)
            continue;
          // 中下
          if (static_costmap->getCost(mx, my - 1) < cur_cost)
            continue;
          // 右下
          if (mx + 1 < static_costmap->getSizeInCellsX() &&
              static_costmap->getCost(mx + 1, my - 1) < cur_cost)
            continue;
        }
        // 左中
        if (mx > 1 && static_costmap->getCost(mx - 1, my) < cur_cost)
          continue;
        // 右中
        if (mx + 1 < static_costmap->getSizeInCellsX() &&
            static_costmap->getCost(mx + 1, my) < cur_cost)
          continue;

        // 至此说明当前访问点处不存在梯度，可认为是一个窄道边界点
        break_points.push_back(std::move(std::pair<size_t, size_t>(mx, my)));
      }
    }
  }

  // 以窄道边界点为起点，沿着梯度上升的方向膨胀来确定窄道区域
  std::queue<std::pair<size_t, size_t>> visit_grids;
  for (const auto &grid : break_points) { visit_grids.push(grid); }

  auto enqueueNeighbor = [&](size_t mx, size_t my, uint8_t cur_cost) {
    if (marked_map->getCost(mx, my) == DEFAULT_GRAY_VALUE &&
        static_costmap->getCost(mx, my) > cur_cost) {
      marked_map->setCost(mx, my, NARROW_GRAY_VALUE);
      visit_grids.push(std::pair<size_t, size_t>(mx, my));
    }
  };

  while (visit_grids.size() != 0) {
    size_t mx = visit_grids.front().first;
    size_t my = visit_grids.front().second;
    uint8_t cur_cost = static_costmap->getCost(mx, my);
    marked_map->setCost(mx, my, NARROW_GRAY_VALUE);

    // 检查上下左右八个临近栅格的代价值，将梯度为正的栅格作为窄道栅格
    if (my + 1 < static_costmap->getSizeInCellsY()) {
      // 左上
      if (mx > 1)
        enqueueNeighbor(mx - 1, my + 1, cur_cost);
      // 中上
      enqueueNeighbor(mx, my + 1, cur_cost);
      // 右上
      if (mx + 1 < static_costmap->getSizeInCellsX())
        enqueueNeighbor(mx + 1, my + 1, cur_cost);
    }
    if (my > 1) {
      // 左下
      if (mx > 1)
        enqueueNeighbor(mx - 1, my - 1, cur_cost);
      // 中下
      enqueueNeighbor(mx, my - 1, cur_cost);
      // 右下
      if (mx + 1 < static_costmap->getSizeInCellsX())
        enqueueNeighbor(mx + 1, my - 1, cur_cost);
    }
    // 左中
    if (mx > 1)
      enqueueNeighbor(mx - 1, my, cur_cost);
    // 右中
    if (mx + 1 < static_costmap->getSizeInCellsX())
      enqueueNeighbor(mx + 1, my, cur_cost);

    visit_grids.pop();
  }
}
}  // namespace CVTE_BABOT