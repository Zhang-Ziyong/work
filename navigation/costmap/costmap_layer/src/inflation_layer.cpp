/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file inflation_layer.cpp
 *
 *@brief inflation layer 实现.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/
#include "inflation_layer.hpp"
#include <glog/logging.h>
#include <costmap_mediator.hpp>

namespace CVTE_BABOT {
InflationLayer::InflationLayer()
    : inflation_radius_(0.6),
      weight_(0),
      cell_inflation_radius_(0),
      cached_cell_inflation_radius_(0) {
  last_bound_.d_min_x = last_bound_.d_min_y =
      -std::numeric_limits<float>::max();
  last_bound_.d_max_x = last_bound_.d_max_y = std::numeric_limits<float>::max();

  inflation_access_ = std::make_shared<std::recursive_mutex>();
}

InflationLayer::~InflationLayer() {
  deleteKernels();
}

bool InflationLayer::onInitialize() {
  std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
  if (!CostmapMediator::getPtrInstance()->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return false;
  }

  if (!CostmapMediator::getPtrInstance()->isLayeredCostmapReady()) {
    LOG(ERROR) << "Is mediator's LayeredCostmap initialized? ";
    return false;
  }

  getParams();

  if (!b_enabled_) {
    LOG(ERROR) << s_name_ << " disabled";
    return false;
  }

  need_reinflation_ = true;

  matchSize();

  return true;
}

void InflationLayer::matchSize() {
  std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
  auto costmap = CostmapMediator::getPtrInstance()->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  // 从这里设置
  computeCaches();
}

void InflationLayer::getParams() {
  double cost_scaling_factor, inflation_radius;
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "enabled",
                                              b_enabled_, true);
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "inflation_radius", inflation_radius, 0.6);
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "cost_scaling_factor", cost_scaling_factor, 5.0);

  // 内切圆 只用了内接圆
  inscribed_radius_ = CostmapMediator::getPtrInstance()->getInscribedRadius();

  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius) {
    std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
    // 计算内接圆的栅格距离
    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    // 转存权重
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }
}

void InflationLayer::printCachesCost(const unsigned int &cache_size) {
  // 通过直接调用glog内部的打印流,把内部封裝的自动换行去掉.
  // 因为glog实例化的对象设置成了无法拷贝,所以使用右值引用
  auto &&log = COMPACT_GOOGLE_LOG_INFO;

  // 先换行
  log.stream() << std::endl;

  for (unsigned int i = 0; i < cache_size; ++i) {
    for (unsigned int j = 0; j < cache_size; ++j) {
      // 缓存代价
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
      log.stream() << static_cast<int>(cached_costs_[i][j]) << " ";
    }
    log.stream() << std::endl;
  }
  log.stream() << std::endl;
}

bool InflationLayer::computeCaches() {
  // 设置缓存代价
  if (cell_inflation_radius_ == 0) {
    return false;
  }

  // 判断是否需要重新计算缓存
  const unsigned int cache_size = cell_inflation_radius_ + 2;
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    deleteKernels();

    cached_costs_ = new unsigned char *[cache_size];
    cached_distances_ = new double *[cache_size];

    for (unsigned int i = 0; i < cache_size; ++i) {
      cached_costs_[i] = new unsigned char[cache_size];
      cached_distances_[i] = new double[cache_size];

      for (unsigned int j = 0; j < cache_size; ++j) {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  // 衰减系数可能发生变化,所以即使膨胀半径没变,也需要重新计算
  LOG(INFO) << "printCachesCost";
  printCachesCost(cache_size);
  return true;
}

void InflationLayer::deleteKernels() {
  const unsigned int cache_size = cached_cell_inflation_radius_ + 2;
  if (cached_distances_ != nullptr) {
    for (unsigned int i = 0; i < cache_size; ++i) {
      if (cached_distances_[i]) {
        delete[] cached_distances_[i];
      }
    }

    if (cached_distances_) {
      delete[] cached_distances_;
    }
    cached_distances_ = nullptr;
  }

  if (cached_costs_ != nullptr) {
    for (unsigned int i = 0; i < cache_size; ++i) {
      if (cached_costs_[i]) {
        delete[] cached_costs_[i];
      }
    }

    delete[] cached_costs_;
    cached_costs_ = nullptr;
  }
}

unsigned char InflationLayer::computeCost(const double &distance) const {
  unsigned char cost = 0;

  // 膨胀代价值计算
  if (distance == 0) {
    cost = LETHAL_OBSTACLE;
  } else if (distance * resolution_ <= inscribed_radius_) {
    cost = INSCRIBED_INFLATED_OBSTACLE;
  } else {
    double euclidean_distance = distance * resolution_;
    double factor =
        exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
    cost =
        static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
  }

  return cost;
}

void InflationLayer::onFootprintChanged() {
  inscribed_radius_ = CostmapMediator::getPtrInstance()->getInscribedRadius();

  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  need_reinflation_ = true;
}

unsigned int InflationLayer::cellDistance(const double &world_dist) {
  auto costmap = CostmapMediator::getPtrInstance()->getCostmap();
  return costmap->cellDistance(world_dist);
}

bool InflationLayer::updateBounds(const WorldmapPose &,
                                  CostmapBound &cb_costmap_bound) {
  if (need_reinflation_) {
    last_bound_ = cb_costmap_bound;

    cb_costmap_bound.d_min_x = cb_costmap_bound.d_min_y =
        -std::numeric_limits<float>::max();
    cb_costmap_bound.d_max_x = cb_costmap_bound.d_max_y =
        std::numeric_limits<float>::max();

    need_reinflation_ = false;
  } else {
    CostmapBound tmp_bound = last_bound_;
    last_bound_ = cb_costmap_bound;

    cb_costmap_bound.d_min_x =
        std::min(tmp_bound.d_min_x, cb_costmap_bound.d_min_x) -
        inflation_radius_;
    cb_costmap_bound.d_min_y =
        std::min(tmp_bound.d_min_y, cb_costmap_bound.d_min_y) -
        inflation_radius_;
    cb_costmap_bound.d_max_x =
        std::max(tmp_bound.d_max_x, cb_costmap_bound.d_max_x) +
        inflation_radius_;
    cb_costmap_bound.d_max_y =
        std::max(tmp_bound.d_max_y, cb_costmap_bound.d_max_y) +
        inflation_radius_;
  }

  return true;
}

bool InflationLayer::updateCosts(
    const std::shared_ptr<Costmap2d> ptr_master_grid, const int &i_min_i,
    const int &i_min_j, const int &i_max_i, const int &i_max_j) {
  std::unique_lock<std::recursive_mutex> lock(*inflation_access_);
  if (!b_enabled_ || (cell_inflation_radius_ == 0)) {
    LOG(ERROR) << s_name_ << " disabled or cell_inflation_radius_=0 ?"
               << std::endl;
    return false;
  }

  int min_i = i_min_i - cell_inflation_radius_;
  int min_j = i_min_j - cell_inflation_radius_;
  int max_i = i_max_i + cell_inflation_radius_;
  int max_j = i_max_j + cell_inflation_radius_;

  unsigned int size_x = ptr_master_grid->getSizeInCellsX(),
               size_y = ptr_master_grid->getSizeInCellsY();
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // Todo(hubery) : 使用opencv膨胀
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      unsigned char cost = ptr_master_grid->getCost(i, j);
      if (cost == LETHAL_OBSTACLE) {
        // 将254的障碍点进行膨胀
        inflate(i, j, *ptr_master_grid);
      }
    }
  }

  // for (int j = min_j; j < max_j; j++) {
  //   for (int i = min_i; i < max_i; i++) {
  //     unsigned char cost = ptr_master_grid->getCost(i, j);
  //     if (cost == 1) {
  //       ptr_master_grid->setCost(i, j, LETHAL_OBSTACLE);
  //     }
  //   }
  // }
  return true;
}

void InflationLayer::inflate(const int &x, const int &y,
                             Costmap2d &master_grid) {
  unsigned int size_x = master_grid.getSizeInCellsX(),
               size_y = master_grid.getSizeInCellsY();
  int ix_min = std::max(0, static_cast<int>(x - cell_inflation_radius_));
  int iy_min = std::max(0, static_cast<int>(y - cell_inflation_radius_));
  int ix_max = std::min(static_cast<int>(size_x - 1),
                        static_cast<int>(x + cell_inflation_radius_));
  int iy_max = std::min(static_cast<int>(size_y - 1),
                        static_cast<int>(y + cell_inflation_radius_));

  // 为节省计算成本,遍历时使用方形,但实际上在膨胀时要判断是否在膨胀半径内.
  for (int ix = ix_min; ix <= ix_max; ++ix) {
    for (int iy = iy_min; iy <= iy_max; ++iy) {
      // 如果距离大于膨胀半径,不膨胀
      unsigned int distance_x = abs(ix - x);
      unsigned int distance_y = abs(iy - y);
      if (distanceLookup(distance_x, distance_y) > cell_inflation_radius_) {
        continue;
      }

      auto old_cost = master_grid.getCost(ix, iy);
      if (old_cost == LETHAL_OBSTACLE) {
        continue;
      }

      // 从缓存数组中获取带价值 并比较大小
      auto new_cost = costLookup(distance_x, distance_y);
      if (old_cost == NO_INFORMATION &&
          new_cost >= INSCRIBED_INFLATED_OBSTACLE) {
        // 不需要比较
        master_grid.setCost(ix, iy, new_cost);
      } else {
        // 比较大小然后设置带价值
        master_grid.setCost(ix, iy, std::max(old_cost, new_cost));
      }
    }
  }
}
}  // namespace CVTE_BABOT
