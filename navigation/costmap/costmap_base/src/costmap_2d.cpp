/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_2d.cpp
 *
 *@brief 实现代价地图存储类
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified huabo wu(wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#include "costmap_2d.hpp"
#include <glog/logging.h>
#include <iostream>

namespace CVTE_BABOT {
// just initialize everything to NULL by default
Costmap2d::Costmap2d()
    : d_origin_x_(0.0),
      d_origin_y_(0.0),
      d_resolution_(0.0),
      ui_size_x_(0),
      ui_size_y_(0) {
  ptr_mutex_access_ = std::make_shared<std::recursive_mutex>();
}

Costmap2d::Costmap2d(const unsigned int &ui_cells_size_x,
                     const unsigned int &ui_cells_size_y,
                     const double &d_resolution, const double &d_origin_x,
                     const double &d_origin_y,
                     const unsigned char &uc_default_value)
    : d_origin_x_(d_origin_x),
      d_origin_y_(d_origin_y),
      d_resolution_(fabs(d_resolution)),
      ui_size_x_(ui_cells_size_x),
      ui_size_y_(ui_cells_size_y),
      uc_default_value_(uc_default_value) {
  ptr_mutex_access_ = std::make_shared<std::recursive_mutex>();
  initMaps(ui_size_x_, ui_size_y_);
  resetMaps();
}

Costmap2d::Costmap2d(const Costmap2d &map) {
  ptr_mutex_access_ = std::make_shared<std::recursive_mutex>();
  std::unique_lock<std::recursive_mutex> lock(*(map.ptr_mutex_access_));
  ui_size_x_ = map.ui_size_x_;
  ui_size_y_ = map.ui_size_y_;
  d_resolution_ = map.d_resolution_;
  d_origin_x_ = map.d_origin_x_;
  d_origin_y_ = map.d_origin_y_;

  ptr_uc_costmap_ = boost::shared_array<unsigned char>(
      new unsigned char[ui_size_x_ * ui_size_y_]);
  if (ptr_uc_costmap_.get() == NULL) {
    LOG(ERROR) << "ptr_uc_costmap_, memory allocation failed." << std::endl;
    return;
  }

  memcpy(ptr_uc_costmap_.get(), map.ptr_uc_costmap_.get(),
         ui_size_x_ * ui_size_y_ * sizeof(unsigned char));
}

Costmap2d &Costmap2d::operator=(const Costmap2d &map) {
  if (this == &map) {
    return *this;
  }
  std::unique_lock<std::recursive_mutex> lock(*ptr_mutex_access_);
  ui_size_x_ = map.ui_size_x_;
  ui_size_y_ = map.ui_size_y_;
  d_resolution_ = map.d_resolution_;
  d_origin_x_ = map.d_origin_x_;
  d_origin_y_ = map.d_origin_y_;

  initMaps(ui_size_x_, ui_size_y_);

  memcpy(ptr_uc_costmap_.get(), map.ptr_uc_costmap_.get(),
         ui_size_x_ * ui_size_y_ * sizeof(unsigned char));

  return *this;
}

void Costmap2d::initMaps(const unsigned int &ui_size_x,
                         const unsigned int &ui_size_y) {
  std::unique_lock<std::recursive_mutex> lock(*ptr_mutex_access_);
  ptr_uc_costmap_.reset();
  ptr_uc_costmap_ = boost::shared_array<unsigned char>(
      new unsigned char[ui_size_x * ui_size_y]);
}

void Costmap2d::resizeMap(const unsigned int &ui_size_x,
                          const unsigned int &ui_size_y,
                          const double &d_resolution, const double &d_origin_x,
                          const double &d_origin_y) {
  std::unique_lock<std::recursive_mutex> lock(*ptr_mutex_access_);
  ui_size_x_ = ui_size_x;
  ui_size_y_ = ui_size_y;
  d_resolution_ = fabs(d_resolution);
  d_origin_x_ = d_origin_x;
  d_origin_y_ = d_origin_y;

  initMaps(ui_size_x, ui_size_y);
  resetMaps();
}

void Costmap2d::resetMaps() {
  std::unique_lock<std::recursive_mutex> lock(*ptr_mutex_access_);
  memset(ptr_uc_costmap_.get(), uc_default_value_,
         ui_size_x_ * ui_size_y_ * sizeof(unsigned char));
}

void Costmap2d::resetMap(const unsigned int &ui_x0, const unsigned int &ui_y0,
                         const unsigned int &ui_xn, const unsigned int &ui_yn) {
  std::unique_lock<std::recursive_mutex> lock(*ptr_mutex_access_);
  unsigned int len = ui_xn - ui_x0;
  for (unsigned int y = ui_y0 * ui_size_x_ + ui_x0;
       y < ui_yn * ui_size_x_ + ui_x0; y += ui_size_x_)
    memset(ptr_uc_costmap_.get() + y, uc_default_value_,
           len * sizeof(unsigned char));
}

void Costmap2d::mapToWorld(const CostmapPoint &cp_point,
                           WorldmapPoint &wm_point) const {
  wm_point.d_x = d_origin_x_ + (cp_point.ui_x + 0.5) * d_resolution_;
  wm_point.d_y = d_origin_y_ + (cp_point.ui_y + 0.5) * d_resolution_;
}

bool Costmap2d::worldToMap(const WorldmapPoint &wm_point,
                           CostmapPoint &cp_point) const {
  if (wm_point.d_x < d_origin_x_ || wm_point.d_y < d_origin_y_) {
    return false;
  }

  cp_point.ui_x =
      static_cast<int>((wm_point.d_x - d_origin_x_) / d_resolution_);
  cp_point.ui_y =
      static_cast<int>((wm_point.d_y - d_origin_y_) / d_resolution_);

  if (cp_point.ui_x < (ui_size_x_) && cp_point.ui_y < (ui_size_y_)) {
    return true;
  }

  return false;
}

void Costmap2d::worldToMapNoBounds(const double &d_wx, const double &d_wy,
                                   int &i_mx, int &i_my) const {
  i_mx = static_cast<int>((d_wx - d_origin_x_) / d_resolution_);
  i_my = static_cast<int>((d_wy - d_origin_y_) / d_resolution_);
}

void Costmap2d::worldToMapEnforceBounds(const double &d_wx, const double &d_wy,
                                        int &i_mx, int &i_my) const {
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (d_wx < d_origin_x_) {
    i_mx = 0;
  } else if (d_wx > d_resolution_ * ui_size_x_ + d_origin_x_) {
    i_mx = ui_size_x_ - 1;
  } else {
    i_mx = static_cast<int>((d_wx - d_origin_x_) / d_resolution_);
  }

  if (d_wy < d_origin_y_) {
    i_my = 0;
  } else if (d_wy > d_resolution_ * ui_size_y_ + d_origin_y_) {
    i_my = ui_size_y_ - 1;
  } else {
    i_my = static_cast<int>((d_wy - d_origin_y_) / d_resolution_);
  }
}

void Costmap2d::updateOrigin(const double &d_new_origin_x,
                             const double &d_new_origin_y) {
  int cell_ox =
      static_cast<int>((d_new_origin_x - d_origin_x_) / d_resolution_);
  int cell_oy =
      static_cast<int>((d_new_origin_y - d_origin_y_) / d_resolution_);

  double new_grid_ox = d_origin_x_ + cell_ox * d_resolution_;
  double new_grid_oy = d_origin_y_ + cell_oy * d_resolution_;

  int size_x = ui_size_x_;
  int size_y = ui_size_y_;

  int lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  int lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  int upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  int upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  boost::shared_array<unsigned char> local_map =
      boost::shared_array<unsigned char>(
          new unsigned char[cell_size_x * cell_size_y]);

  copyMapRegion(ptr_uc_costmap_, lower_left_x, lower_left_y, ui_size_x_,
                local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  resetMaps();

  d_origin_x_ = new_grid_ox;
  d_origin_y_ = new_grid_oy;

  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  copyMapRegion(local_map, 0, 0, cell_size_x, ptr_uc_costmap_, start_x, start_y,
                ui_size_x_, cell_size_x, cell_size_y);
}

bool Costmap2d::setConvexPolygonCost(const std::vector<WorldmapPoint> &polygon,
                                     const unsigned char &cost_value) {
  std::vector<CostmapPoint> map_polygon;
  map_polygon.reserve(polygon.size());
  CostmapPoint loc;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    if (!worldToMap(polygon[i], loc)) {
      return false;
    }
    map_polygon.push_back(loc);
  }

  std::vector<CostmapPoint> polygon_cells;

  if (!convexFillCells(map_polygon, polygon_cells)) {
    return false;
  }
  unsigned int index;
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    index = getIndex(polygon_cells[i].ui_x, polygon_cells[i].ui_y);
    ptr_uc_costmap_[index] = cost_value;
  }
  return true;
}

bool Costmap2d::polygonOutlineCells(const std::vector<CostmapPoint> &polygon,
                                    std::vector<CostmapPoint> &polygon_cells) {
  PolygonOutlineCells cell_gatherer(*this, ptr_uc_costmap_, polygon_cells);
  for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
    raytraceLine(cell_gatherer, polygon[i].ui_x, polygon[i].ui_y,
                 polygon[i + 1].ui_x, polygon[i + 1].ui_y);
  }
  if (!polygon.empty()) {
    unsigned int last_index = polygon.size() - 1;
    raytraceLine(cell_gatherer, polygon[last_index].ui_x,
                 polygon[last_index].ui_y, polygon[0].ui_x, polygon[0].ui_y);
  } else {
    return false;
  }
  return true;
}

void Costmap2d::combineCostmap(std::shared_ptr<Costmap2d> ptr_local_costmap) {
  if (ptr_local_costmap == nullptr) {
    LOG(ERROR) << "combine costmap is nullptr";
    return;
  }
  for (size_t ix = 0; ix < ptr_local_costmap->getSizeInCellsX(); ix++) {
    for (size_t iy = 0; iy < ptr_local_costmap->getSizeInCellsY(); iy++) {
      WorldmapPoint world_point;
      ptr_local_costmap->mapToWorld(CostmapPoint(ix, iy), world_point);
      CostmapPoint cur_map_point;
      if (worldToMap(world_point, cur_map_point)) {
        int local_cost = ptr_local_costmap->getCost(CostmapPoint(ix, iy));
        if (getCost(cur_map_point) < local_cost) {
          setCost(cur_map_point.ui_x, cur_map_point.ui_y, local_cost);
        }
      }
    }
  }
}

bool Costmap2d::convexFillCells(const std::vector<CostmapPoint> &polygon,
                                std::vector<CostmapPoint> &polygon_cells) {
  if (polygon.size() < 3)
    return false;

  if (!polygonOutlineCells(polygon, polygon_cells)) {
    return false;
  }

  CostmapPoint swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1) {
    if (polygon_cells[i].ui_x > polygon_cells[i + 1].ui_x) {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0)
        --i;
    } else {
      ++i;
    }
  }

  i = 0;
  CostmapPoint min_pt;
  CostmapPoint max_pt;
  unsigned int min_x = polygon_cells[0].ui_x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].ui_x;

  for (unsigned int x = min_x; x <= max_x; ++x) {
    if (i >= polygon_cells.size() - 1)
      break;

    if (polygon_cells[i].ui_y < polygon_cells[i + 1].ui_y) {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    } else {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].ui_x == x) {
      if (polygon_cells[i].ui_y < min_pt.ui_y)
        min_pt = polygon_cells[i];
      else if (polygon_cells[i].ui_y > max_pt.ui_y)
        max_pt = polygon_cells[i];
      ++i;
    }

    CostmapPoint pt;
    for (unsigned int y = min_pt.ui_y; y < max_pt.ui_y; ++y) {
      pt.ui_x = x;
      pt.ui_y = y;
      polygon_cells.push_back(pt);
    }
  }

  return true;
}

void Costmap2d::computeLineCells(const unsigned int &x0, const unsigned int &y0,
                                 const unsigned int &x1, const unsigned int &y1,
                                 std::vector<CostmapPoint> &v_line_points) {
  PolygonOutlineCells cell_gatherer(*this, ptr_uc_costmap_, v_line_points);

  raytraceLine(cell_gatherer, x0, y0, x1, y1);
}

int Costmap2d::getCostWithMapPose(const double &point_x,
                                  const double &point_y) {
  WorldmapPoint wm_point = {point_x, point_y};
  CostmapPoint cp_point;
  if (worldToMap(wm_point, cp_point)) {
    return static_cast<int>(getCost(cp_point.ui_x, cp_point.ui_y));
  } else {
    // 超出地图坐标范围，返回256（代价值最大为255）
    return 256;
  }
}

}  // namespace CVTE_BABOT