/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file static_layer.cpp
 *
 *@brief static layer 实现.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/

#include "static_layer.hpp"

#include <glog/logging.h>

#include "costmap_mediator.hpp"
#include "layered_costmap.hpp"

#define INFLATE_GRIDIENT true

namespace CVTE_BABOT {
bool StaticLayer::onInitialize() {
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
    LOG(INFO) << s_name_ << " disabled";
    return false;
  }

  has_updated_data_ = false;

  // 保存静态地图到mediator
  std::shared_ptr<Costmap2d> ptr_costmap = std::make_shared<Costmap2d>();
  CostmapMediator::getPtrInstance()
      ->registerUpdateFunc<std::shared_ptr<Costmap2d>>("static_costmap",
                                                       ptr_costmap);

  // 获取地图
  if (!resetMap()) {
    LOG(ERROR) << "load static map failed. ";
    return false;
  }

  // 初始化膨胀器
  inflater_.initialize("inflation_layer");
  inflater_.onInitialize();

  return true;
}

void StaticLayer::getParams() {
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "enabled",
                                              b_enabled_, true);
  if (!b_enabled_) {
    return;
  }
  CostmapMediator::getPtrInstance()->getParam("mark_unknown_space",
                                              b_mark_unknown_space_, true);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "use_maximum",
                                              use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "lethal_cost_threshold", temp_lethal_threshold, int(100));
  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);

  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "unknown_cost_value", temp_unknown_cost_value, int(-1));
  unknown_cost_value_ = temp_unknown_cost_value;

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "trinary_costmap",
                                              trinary_costmap_, true);

  int tmp_inflate_cost;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "inflate_cost", tmp_inflate_cost,
      (int) INSCRIBED_INFLATED_OBSTACLE);
  LOG(INFO) << "tmp_inflate_cost: " << tmp_inflate_cost;
  uc_inflate_cost_ = tmp_inflate_cost;
}

bool StaticLayer::resetMap() {
  if (CostmapMediator::getPtrInstance()->ifResetStaticMap()) {
    ptr_worldmap_data_.reset();

    // 获取地图
    CostmapMediator::getPtrInstance()->getData("static_map",
                                               ptr_worldmap_data_);

    if (ptr_worldmap_data_.get() == NULL) {
      LOG(ERROR) << "Could not get the static_map. ";
      return false;
    }
    loadMap();

    CostmapMediator::getPtrInstance()->setResetStaticMapFlag(false);
    LOG(INFO) << "Reset static map. ";
  }
  return true;
}

void StaticLayer::loadMap() {
  unsigned int size_x = ptr_worldmap_data_->ui_width_;
  unsigned int size_y = ptr_worldmap_data_->ui_height_;
  LOG(INFO) << "loadMap";

  auto master = CostmapMediator::getPtrInstance()->getCostmap();

  double ratio = 1.0;
  double resolution = ptr_worldmap_data_->d_resolution_;

  if (!CostmapMediator::getPtrInstance()->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != ptr_worldmap_data_->d_resolution_ ||
       master->getOriginX() != ptr_worldmap_data_->origin_.d_x ||
       master->getOriginY() != ptr_worldmap_data_->origin_.d_y)) {
    //    !ptr_layered_costmap_->isSizeLocked())) {
    CostmapMediator::getPtrInstance()->resizeMap(
        size_x, size_y, ptr_worldmap_data_->d_resolution_,
        ptr_worldmap_data_->origin_.d_x, ptr_worldmap_data_->origin_.d_y, true);
    inflater_.onInitialize();
  } else if (ui_size_x_ != size_x || ui_size_y_ != size_y ||
             d_resolution_ != ptr_worldmap_data_->d_resolution_ ||
             d_origin_x_ != ptr_worldmap_data_->origin_.d_x ||
             d_origin_y_ != ptr_worldmap_data_->origin_.d_y) {
    // only update the size of the costmap stored locally in this layer
    ratio = master->getResolution() / ptr_worldmap_data_->d_resolution_;
    resolution = ptr_worldmap_data_->d_resolution_;
    if (ratio >= 1.0) {
      size_x = static_cast<unsigned int>(static_cast<double>(size_x) / ratio);
      size_y = static_cast<unsigned int>(static_cast<double>(size_y) / ratio);
      resolution = master->getResolution();
    }

    resizeMap(size_x, size_y, resolution, ptr_worldmap_data_->origin_.d_x,
              ptr_worldmap_data_->origin_.d_y);
  }

  unsigned int qz;
  unsigned int qy;

  unsigned char value;
  unsigned int index = 0;
  unsigned int index_1, index_2;

  for (index = 0; index < size_x * size_y; ++index) {
    qz = index / size_x;
    qy = index % size_x;
    index_1 = static_cast<unsigned int>(
        qz * ratio * ptr_worldmap_data_->ui_width_ + qy * ratio);

    value = (*(ptr_worldmap_data_->ptr_data_))[index_1];
    ptr_uc_costmap_[index] = interpretValue(value);

    for (unsigned int k = 0; k < static_cast<unsigned int>(ratio); ++k) {
      for (unsigned int f = 0; f < static_cast<unsigned int>(ratio); ++f) {
        index_2 = index_1 + f + ptr_worldmap_data_->ui_width_ * k;
        value = (*(ptr_worldmap_data_->ptr_data_))[index_2];
        if (LETHAL_OBSTACLE == interpretValue(value)) {
          ptr_uc_costmap_[index] = LETHAL_OBSTACLE;
          break;
        }
      }
    }
  }

  width_ = ui_size_x_;
  height_ = ui_size_y_;
  has_updated_data_ = true;
  cp_update_origin_.ui_x = 0;
  cp_update_origin_.ui_y = 0;

  auto ptr_costmap = std::make_shared<Costmap2d>(*this);

  auto clear_polygons = CostmapMediator::getPtrInstance()->getClearPolygons();
  LOG(INFO) << "clear polygons size: " << clear_polygons.size();
  for (const auto &it : clear_polygons) {
    LOG(INFO) << "clear polygon";
    if (setConvexPolygonCost(it, 0)) {
      LOG(INFO) << "clear polygon succeed";
    } else {
      LOG(ERROR) << "clear polygon failed";
    }
  }
  auto prohibited_polygons =
      CostmapMediator::getPtrInstance()->getProhibitedPolygons();
  for (const auto &it : prohibited_polygons) {
    LOG(INFO) << "prohibited polygon";
    if (setConvexPolygonCost(it, 254)) {
      LOG(INFO) << "set prohibited polygon succeed";
    } else {
      LOG(ERROR) << "set prohibited polygon failed";
    }
  }

#if INFLATE_GRIDIENT
  inflateStaticMapGridient(ptr_costmap);
#else
  // TODO:
  // 目前暂时将静态层拉分成两次膨胀，第一次膨胀用uc_inflate_cost_值，第二次用uc_inflate_cost_的一半
  inflateStaticMap(3, uc_inflate_cost_, ptr_costmap);
  inflateStaticMap(6, uc_inflate_cost_ / 2, ptr_costmap);
#endif
  CostmapMediator::getPtrInstance()->updateData("static_costmap", ptr_costmap);
}

bool StaticLayer::updateBounds(const WorldmapPose &wp_robot_pose,
                               CostmapBound &cb_costmap_bound) {
  // TODO(icanchen): 下面三行有问题，需要修改
  if (wp_robot_pose.d_x == 0 && wp_robot_pose.d_y == 0 &&
      wp_robot_pose.d_yaw == 0) {
    // no warning
  }

  if (!b_enabled_) {
    return false;
  }

  if (!resetMap()) {
    LOG(ERROR) << "Reset static map failed. ";
    return false;
  }
  /*if not roll window ,the StaticLayer' bound will only update once
    until there are new incoming map.*/
  if (!CostmapMediator::getPtrInstance()->isRolling()) {
    if (!(has_updated_data_ || b_has_extra_bounds_)) {
      return false;
    } else {
      has_updated_data_ = false;
    }
  } else {
    if (!has_updated_data_) {
      // LOG(INFO) << "Static map has not been loaded. ";
      return false;
    }
  }

  useExtraBounds(cb_costmap_bound);

  WorldmapPoint wm_point;
  mapToWorld(cp_update_origin_, wm_point);
  cb_costmap_bound.d_min_x = std::min(wm_point.d_x, cb_costmap_bound.d_min_x);
  cb_costmap_bound.d_min_y = std::min(wm_point.d_y, cb_costmap_bound.d_min_y);

  mapToWorld(
      {cp_update_origin_.ui_x + width_, cp_update_origin_.ui_y + height_},
      wm_point);
  cb_costmap_bound.d_max_x = std::max(wm_point.d_x, cb_costmap_bound.d_max_x);
  cb_costmap_bound.d_max_y = std::max(wm_point.d_y, cb_costmap_bound.d_max_y);

  return true;
}

bool StaticLayer::updateCosts(const std::shared_ptr<Costmap2d> ptr_master_grid,
                              const int &i_min_i, const int &i_min_j,
                              const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_ || !has_updated_data_) {
    return false;
  }

  if (!CostmapMediator::getPtrInstance()->isRolling()) {
    if (!use_maximum_) {
      updateWithTrueOverwrite(ptr_master_grid, i_min_i, i_min_j, i_max_i,
                              i_max_j);
    } else {
      updateWithMax(ptr_master_grid, i_min_i, i_min_j, i_max_i, i_max_j);
    }
  } else {
    WorldmapPoint wm_point;
    CostmapPoint cm_point;
    for (int i = i_min_i; i < i_max_i; ++i) {
      for (int j = i_min_j; j < i_max_j; ++j) {
        cm_point.ui_x = i;
        cm_point.ui_y = j;

        // 实际地图可能超出静态地图范围，检查防止越界
        ptr_master_grid->mapToWorld(cm_point, wm_point);

        if (worldToMap(wm_point, cm_point)) {
          if (!use_maximum_) {
            // auto cost = getCost(cm_point.ui_x, cm_point.ui_y);
            // if (cost == LETHAL_OBSTACLE) {
            //   ptr_master_grid->setCost(i, j, 1);
            // } else {
            //   ptr_master_grid->setCost(i, j, cost);
            // }
            ptr_master_grid->setCost(i, j,
                                     getCost(cm_point.ui_x, cm_point.ui_y));
          } else {
            ptr_master_grid->setCost(
                i, j,
                std::max(getCost(cm_point.ui_x, cm_point.ui_y),
                         ptr_master_grid->getCost(i, j)));
          }
        }
      }
    }
  }
  return true;
}

void StaticLayer::inflateStaticMap(
    const unsigned int &ui_inflate_radius, const unsigned char &inflate_cost,
    const std::shared_ptr<Costmap2d> &master_grid) {
  int size_x = static_cast<int>(master_grid->getSizeInCellsX()),
      size_y = static_cast<int>(master_grid->getSizeInCellsY());

  // TODO(hubery) :循环嵌套太多,不易理解
  for (int j = 0; j < size_y; j++) {
    for (int i = 0; i < size_x; i++) {
      unsigned char cost = master_grid->getCost(i, j);

      if (cost == LETHAL_OBSTACLE) {
        // 计算膨胀的栅格范围
        int ix_min = std::max(0, static_cast<int>(i - ui_inflate_radius));
        int iy_min = std::max(0, static_cast<int>(j - ui_inflate_radius));
        int ix_max = std::min(static_cast<int>(size_x - 1),
                              static_cast<int>(i + ui_inflate_radius));
        int iy_max = std::min(static_cast<int>(size_y - 1),
                              static_cast<int>(j + ui_inflate_radius));

        for (int ix = ix_min; ix <= ix_max; ix++) {
          for (int iy = iy_min; iy <= iy_max; iy++) {
            unsigned int distance_x = abs(ix - i);
            unsigned int distance_y = abs(iy - j);
            if (sqrt(distance_x * distance_x + distance_y * distance_y) >
                ui_inflate_radius) {
              continue;
            }

            unsigned char old_cost = master_grid->getCost(ix, iy);

            if (old_cost == LETHAL_OBSTACLE) {
              continue;
            }

            if (old_cost == NO_INFORMATION &&
                inflate_cost >= INSCRIBED_INFLATED_OBSTACLE) {
              master_grid->setCost(ix, iy, inflate_cost);
              // setCost(ix, iy, inflate_cost);
            } else {
              master_grid->setCost(ix, iy, std::max(old_cost, inflate_cost));
              // setCost(ix, iy, std::max(old_cost, inflate_cost));
            }
          }
        }
      }
    }
  }
}

void StaticLayer::inflateStaticMapGridient(
    const std::shared_ptr<Costmap2d> &master_grid) {
  inflater_.updateCosts(master_grid, 0, 0, master_grid->getSizeInCellsX(),
                        master_grid->getSizeInCellsY());
}

void StaticLayer::matchSize() {
  if (!CostmapMediator::getPtrInstance()->isRolling()) {
    auto master = CostmapMediator::getPtrInstance()->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
              master->getResolution(), master->getOriginX(),
              master->getOriginY());
  }
}

unsigned char StaticLayer::interpretValue(const unsigned char &value) {
  // check if the static value is above the unknown or lethal thresholds
  // if (b_mark_unknown_space_ && value == unknown_cost_value_) {
  //   return NO_INFORMATION;
  // } else if (!b_mark_unknown_space_ && value == unknown_cost_value_) {
  //   return FREE_SPACE;
  // } else if (value >= lethal_threshold_) {
  //   return LETHAL_OBSTACLE;
  // } else if (trinary_costmap_) {
  //   return FREE_SPACE;
  // }
  if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (value == 0) {
    return FREE_SPACE;
  } else if (0 < value < lethal_threshold_) {
    return INSCRIBED_INFLATED_OBSTACLE;
  }
  double scale = static_cast<double>(value) / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::activate() {
  onInitialize();
}
void StaticLayer::deactivate() {}
void StaticLayer::reset() {}
}  // namespace CVTE_BABOT
