#include <glog/logging.h>

#include "costmap_layer.hpp"
#include "costmap_mediator.hpp"
#include "footprint_utils.hpp"
#include "layered_costmap.hpp"
namespace CVTE_BABOT {
void CostmapLayer::loadAvandanceBound() {
  // 使用padded扩展之后的footprint作为原始的减速和停障框
  std::vector<WorldmapPoint> vwp_footprint;
  CostmapMediator::getPtrInstance()->getParam("footprint_padded", vwp_footprint,
                                              vwp_footprint);
  CostmapMediator::getPtrInstance()->getParam("extra_area_after_stop_size_x",
                                              extra_area_after_stop_size_x_,
                                              extra_area_after_stop_size_x_);

  LOG(INFO) << "extra_area_after_stop_size_x_ "
            << extra_area_after_stop_size_x_;

  std::vector<WorldmapPoint> stop_bound(2);
  std::vector<WorldmapPoint> slow_down_bound(2);

  if (vwp_footprint.size() == 4) {
    double min_x = 0.00, min_y = 0.00, max_x = 0.00, max_y = 0.00;
    for (const auto &wp_point : vwp_footprint) {
      min_x = std::min(min_x, wp_point.d_x);
      max_x = std::max(max_x, wp_point.d_x);
      min_y = std::min(min_y, wp_point.d_y);
      max_y = std::max(max_y, wp_point.d_y);
    }
    LOG(INFO) << "vwp_footprint " << min_x << "," << max_x << "," << min_y
              << "," << max_y;
    if (fabs(min_x) - fabs(max_x) >= 0.01) {
      robot_type_ = FRONT_DRIVE;
    } else if (fabs(min_x) - fabs(max_x) <= -0.01) {
      robot_type_ = BACK_DRIVE;
    } else {
      robot_type_ = CENTER_DRIVE;
    }
    LOG(INFO) << "robot type: " << robot_type_;

    origin_stop_bound_[LOW] = WorldmapPoint{min_x, min_y};
    origin_stop_bound_[HIGH] = WorldmapPoint{max_x, max_y};

    LOG(INFO) << "stop_bound: x: " << origin_stop_bound_[LOW].d_x << " "
              << origin_stop_bound_[HIGH].d_x;
    LOG(INFO) << "stop_bound: y: " << origin_stop_bound_[LOW].d_y << " "
              << origin_stop_bound_[HIGH].d_y;
    origin_slow_bound_[LOW] = WorldmapPoint{min_x, min_y};
    origin_slow_bound_[HIGH] = WorldmapPoint{max_x, max_y};
    LOG(INFO) << "origin_slow_bound_: x: " << origin_slow_bound_[LOW].d_x << " "
              << origin_slow_bound_[HIGH].d_x;
    LOG(INFO) << "origin_slow_bound_: y: " << origin_slow_bound_[LOW].d_y << " "
              << origin_slow_bound_[HIGH].d_y;
  } else {
    LOG(ERROR) << "Invalid footprint";
  }
}  // namespace CVTE_BABOT

bool CostmapLayer::isConvexPolygonFill(
    const std::vector<WorldmapPoint> &polygon) {
  std::vector<CostmapPoint> map_polygon;
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

  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    unsigned int index = getIndex(polygon_cells[i].ui_x, polygon_cells[i].ui_y);
    if (ptr_uc_costmap_[index] == LETHAL_OBSTACLE) {
      return true;
    }
  }
  return false;
}

void CostmapLayer::touch(const WorldmapPoint &wm_point,
                         CostmapBound &cb_bound) {
  cb_bound.d_min_x = std::min(wm_point.d_x, cb_bound.d_min_x);
  cb_bound.d_min_y = std::min(wm_point.d_y, cb_bound.d_min_y);
  cb_bound.d_max_x = std::max(wm_point.d_x, cb_bound.d_max_x);
  cb_bound.d_max_y = std::max(wm_point.d_y, cb_bound.d_max_y);
}

void CostmapLayer::matchSize() {
  auto master = CostmapMediator::getPtrInstance()->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
            master->getResolution(), master->getOriginX(),
            master->getOriginY());
}

void CostmapLayer::addExtraBounds(double &d_mx0, double &d_my0, double &d_mx1,
                                  double &d_my1) {
  d_extra_min_x_ = std::min(d_mx0, d_extra_min_x_);
  d_extra_max_x_ = std::max(d_mx1, d_extra_max_x_);
  d_extra_min_y_ = std::min(d_my0, d_extra_min_y_);
  d_extra_max_y_ = std::max(d_my1, d_extra_max_y_);
  b_has_extra_bounds_ = true;
}

void CostmapLayer::useExtraBounds(CostmapBound &costmap_bound) {
  if (!b_has_extra_bounds_)
    return;

  costmap_bound.d_min_x = std::min(d_extra_min_x_, costmap_bound.d_min_x);
  costmap_bound.d_min_y = std::min(d_extra_min_y_, costmap_bound.d_min_y);
  costmap_bound.d_max_x = std::max(d_extra_max_x_, costmap_bound.d_max_x);
  costmap_bound.d_max_y = std::max(d_extra_max_y_, costmap_bound.d_max_y);
  d_extra_min_x_ = 1e6;
  d_extra_min_y_ = 1e6;
  d_extra_max_x_ = -1e6;
  d_extra_max_y_ = -1e6;
  b_has_extra_bounds_ = false;
}

void CostmapLayer::updateWithMax(
    const std::shared_ptr<Costmap2d> ptr_master_grid, const int &i_min_i,
    const int &i_min_j, const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_)
    return;
  boost::shared_array<unsigned char> master_array =
      ptr_master_grid->getCharMap();

  unsigned int span = ptr_master_grid->getSizeInCellsX();

  for (int j = i_min_j; j < i_max_j; j++) {
    unsigned int it = j * span + i_min_i;
    for (int i = i_min_i; i < i_max_i; i++) {
      // if (ptr_uc_costmap_[it] == NO_INFORMATION || ptr_uc_costmap_[it] == 1)
      // {
      if (ptr_uc_costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION || old_cost < ptr_uc_costmap_[it]) {
        master_array[it] = ptr_uc_costmap_[it];
      }

      it++;
    }
  }
}

void CostmapLayer::updateWithTrueOverwrite(
    const std::shared_ptr<Costmap2d> ptr_master_grid, const int &i_min_i,
    const int &i_min_j, const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_)
    return;
  boost::shared_array<unsigned char> master = ptr_master_grid->getCharMap();
  unsigned int span = ptr_master_grid->getSizeInCellsX();

  for (int j = i_min_j; j < i_max_j; j++) {
    unsigned int it = j * span + i_min_i;
    for (int i = i_min_i; i < i_max_i; i++) {
      master[it] = ptr_uc_costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::updateWithOverwrite(
    const std::shared_ptr<Costmap2d> ptr_master_grid, const int &i_min_i,
    const int &i_min_j, const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_)
    return;
  boost::shared_array<unsigned char> master = ptr_master_grid->getCharMap();
  unsigned int span = ptr_master_grid->getSizeInCellsX();

  for (int j = i_min_j; j < i_max_j; j++) {
    unsigned int it = j * span + i_min_i;
    for (int i = i_min_i; i < i_max_i; i++) {
      if (ptr_uc_costmap_[it] != NO_INFORMATION)
        master[it] = ptr_uc_costmap_[it];
      it++;
    }
  }
}

void CostmapLayer::updateWithAddition(
    const std::shared_ptr<Costmap2d> ptr_master_grid, const int &i_min_i,
    const int &i_min_j, const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_)
    return;
  boost::shared_array<unsigned char> master_array =
      ptr_master_grid->getCharMap();
  unsigned int span = ptr_master_grid->getSizeInCellsX();

  for (int j = i_min_j; j < i_max_j; j++) {
    unsigned int it = j * span + i_min_i;
    for (int i = i_min_i; i < i_max_i; i++) {
      if (ptr_uc_costmap_[it] == NO_INFORMATION) {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION) {
        master_array[it] = ptr_uc_costmap_[it];
      } else {
        int sum = old_cost + ptr_uc_costmap_[it];
        if (sum >= CVTE_BABOT::INSCRIBED_INFLATED_OBSTACLE)
          master_array[it] = CVTE_BABOT::INSCRIBED_INFLATED_OBSTACLE - 1;
        else
          master_array[it] = sum;
      }
      it++;
    }
  }
}
}  // namespace CVTE_BABOT
