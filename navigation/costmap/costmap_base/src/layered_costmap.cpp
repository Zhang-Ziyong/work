/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap.cpp
 *
 *@brief costmap.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#include "layered_costmap.hpp"

#include <glog/logging.h>
#include <time.h>

#include <iostream>
namespace CVTE_BABOT {

LayeredCostmap::LayeredCostmap(const bool &rolling_window,
                               const bool &mark_unknown_space)
    : initialized_(false),
      size_locked_(false),
      rolling_window_(rolling_window) {
  layers_ = std::make_shared<std::vector<std::shared_ptr<Layer>>>();
  unsigned char uc_default_value;
  if (mark_unknown_space) {
    uc_default_value = 255;
  } else {
    uc_default_value = 0;
  }
  costmap_ =
      std::make_shared<Costmap2d>(200, 200, 0.05, -5.0, -5.0, uc_default_value);

  ptr_field_ = makeFiled();
  resetFiled(ptr_field_);

  stop_obstacle_area_.resize(OBSTACLE_DIREC_AREA);
  map_layer_names_[laser_layer] = "laser_layer";
  map_layer_names_[probabiltiy_voxel_layer] = "probabiltiy_voxel_layer";
  map_layer_names_[negative_obstacle_layer] = "negative_obstacle_layer";
  map_layer_names_[sonar_layer] = "sonar_layer";
  map_layer_names_[range_layer] = "range_layer";
  map_layer_names_[collision_layer] = "collision_layer";

  // std::vector<std::vector<WorldmapPose>> slope_areas;
  // WorldmapPose pose;
  // std::vector<WorldmapPose> poses_vec;
  // pose.d_x = -3.0;
  // pose.d_y = -3.0;
  // poses_vec.push_back(pose);

  // pose.d_x = -3.0;
  // pose.d_y = 3.0;
  // poses_vec.push_back(pose);

  // pose.d_x = 3.0;
  // pose.d_y = 3.0;
  // poses_vec.push_back(pose);

  // pose.d_x = 3.0;
  // pose.d_y = -3.0;
  // poses_vec.push_back(pose);

  // slope_areas.push_back(poses_vec);
  // setCleanAreas(slope_areas);
}

void LayeredCostmap::getFootprint(std::vector<WorldmapPoint> &footprint_spec) {
  footprint_spec.clear();

  footprint_spec.assign(footprint_.begin(), footprint_.end());
}

void LayeredCostmap::setFootprint(
    const std::vector<WorldmapPoint> &footprint_spec) {
  footprint_ = footprint_spec;
  CVTE_BABOT::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_,
                                          circumscribed_radius_);
  LOG(INFO) << "inscribed_radius: " << inscribed_radius_
            << ", circumscribed_radius: " << circumscribed_radius_;

  for (std::vector<std::shared_ptr<Layer>>::iterator layer = layers_->begin();
       layer != layers_->end(); ++layer) {
    (*layer)->onFootprintChanged();
  }
}

bool LayeredCostmap::isPointInPolygon(
    const int x, const int y, const std::vector<std::vector<int>> &points) {
  // vector<Point> points:表示闭合区域由这些点围成
  // int minX = points[0][0];
  // int maxX = points[0][0];
  // int minY = points[0][1];
  // int maxY = points[0][1];
  // for (unsigned int i = 1; i < points.size(); i++) {
  //   minX = std::min(points[i][0], minX);
  //   maxX = std::max(points[i][0], maxX);
  //   minY = std::min(points[i][1], minY);
  //   maxY = std::max(points[i][1], maxY);
  // }

  // if (x < minX || x > maxX || y < minY || y > maxY) {
  //   return false;
  // }

  bool inside = false;
  for (unsigned int i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    if ((points[i][1] > y) != (points[j][1] > y) &&
        x < (points[j][0] - points[i][0]) * (y - points[i][1]) /
                    (points[j][1] - points[i][1]) +
                points[i][0]) {
      inside = !inside;
    }
  }

  return inside;
}

bool LayeredCostmap::isPointInPolygon(const double x, const double y,
                                      const std::vector<WorldmapPose> &points) {
  // vector<Point> points:表示闭合区域由这些点围成
  double minX = points[0].d_x;
  double maxX = points[0].d_x;
  double minY = points[0].d_y;
  double maxY = points[0].d_y;
  for (unsigned int i = 1; i < points.size(); i++) {
    minX = std::min(points[i].d_x, minX);
    maxX = std::max(points[i].d_x, maxX);
    minY = std::min(points[i].d_y, minY);
    maxY = std::max(points[i].d_y, maxY);
  }

  if (x < minX || x > maxX || y < minY || y > maxY) {
    return false;
  }

  bool inside = false;
  for (unsigned int i = 0, j = points.size() - 1; i < points.size(); j = i++) {
    if ((points[i].d_y > y) != (points[j].d_y > y) &&
        x < (points[j].d_x - points[i].d_x) * (y - points[i].d_y) /
                    (points[j].d_y - points[i].d_y) +
                points[i].d_x) {
      inside = !inside;
    }
  }

  return inside;
}

bool LayeredCostmap::setCleanAreas(
    const std::vector<std::vector<WorldmapPose>> &clear_areas) {
  clear_areas_ = clear_areas;
}

// TODO: 当区域有部分在地图外时会不生效
bool LayeredCostmap::cleanMapByAreas(
    const WorldmapPose &robot_pose, const std::shared_ptr<Costmap2d> cost_map) {
  LOG(INFO) << "clean map by areas";
  // 1. 先判断区域是否在局部地图内
  // 2. 区域部分在局部地图内的处理？或者直接忽略？基本不影响导航？
  // 3. 逐个区域去除
  double origin_x = robot_pose.d_x - cost_map->getSizeInMetersX() / 2;
  double origin_y = robot_pose.d_y - cost_map->getSizeInMetersY() / 2;
  double res = cost_map->getResolution();
  for (unsigned int i = 0; i < clear_areas_.size(); i++) {
    if (clear_areas_[i].size() >= 3) {
      std::vector<std::vector<int>> map_clear_areas_points;
      int mx, my;
      int min_x = cost_map->getSizeInCellsX();
      int max_x = 0;
      int min_y = cost_map->getSizeInCellsY();
      int max_y = 0;

      for (unsigned int j = 0; j < clear_areas_[i].size(); j++) {
        mx = (clear_areas_[i][j].d_x - origin_x) / res;
        my = (clear_areas_[i][j].d_y - origin_y) / res;

        std::vector<int> mp;
        mp.push_back(mx);
        mp.push_back(my);
        map_clear_areas_points.push_back(mp);

        min_x = std::min(mx, min_x);
        max_x = std::max(mx, max_x);
        min_y = std::min(my, min_y);
        max_y = std::max(my, max_y);
      }

      int mmax_x = cost_map->getSizeInCellsX();
      int mmax_y = cost_map->getSizeInCellsY();
      min_x = std::max(0, min_x);
      max_x = std::min(mmax_x, max_x);
      min_y = std::max(0, min_y);
      max_y = std::min(mmax_y, max_y);

      for (unsigned int iy = min_y; iy < max_y; iy++) {
        for (unsigned int ix = min_x; ix < max_x; ix++) {
          if (isPointInPolygon(ix, iy, map_clear_areas_points)) {
            cost_map->setCost(ix, iy, 0);
          }
        }
      }
    }
  }
}

bool LayeredCostmap::isPointInAreas(const WorldmapPose &point) {
  for (unsigned int i = 0; i < clear_areas_.size(); i++) {
    if (clear_areas_[i].size() >= 3) {
      if (isPointInPolygon(point.d_x, point.d_y, clear_areas_[i])) {
        return true;
      }
    }
  }
  return false;
}

bool LayeredCostmap::setUpdateInShope(const bool update_in_shope) {
  if (update_in_shope_ == true && update_in_shope == false) {
    reset_shope_layer_ = true;
  }
  update_in_shope_ = update_in_shope;
}

void LayeredCostmap::activeLayer(LayerName layer) {
  for (auto layer_it = layers_->begin(); layer_it != layers_->end();
       ++layer_it) {
    if ((*layer_it)->getName() == map_layer_names_[layer]) {
      (*layer_it)->activate();
    }
  }
}

void LayeredCostmap::deactiveLayer(LayerName layer) {
  for (auto layer_it = layers_->begin(); layer_it != layers_->end();
       ++layer_it) {
    if ((*layer_it)->getName() == map_layer_names_[layer]) {
      (*layer_it)->deactivate();
    }
  }
}

void LayeredCostmap::resetLayer(LayerName layer) {
  for (auto layer_it = layers_->begin(); layer_it != layers_->end();
       ++layer_it) {
    if ((*layer_it)->getName() == map_layer_names_[layer]) {
      (*layer_it)->reset();
    }
  }
}

bool LayeredCostmap::updateMap(const WorldmapPose &robot_pose) {
  std::unique_lock<std::recursive_mutex> lock(*(costmap_->getMutx()));

  // LOG(INFO) << "updateMap ";
  if (rolling_window_) {
    double new_origin_x = robot_pose.d_x - costmap_->getSizeInMetersX() / 2;
    double new_origin_y = robot_pose.d_y - costmap_->getSizeInMetersY() / 2;
    costmap_->updateOrigin(new_origin_x, new_origin_y);
  }

  if (layers_->size() == 0) {
    LOG(ERROR) << "layers_->size() = 0." << std::endl;
    return false;
  }

  // 用于判断更新后的bound数据是否合法
  cb_extremum_bound_.d_min_x = cb_extremum_bound_.d_min_y = 1e30;
  cb_extremum_bound_.d_max_x = cb_extremum_bound_.d_max_y = -1e30;
  // bool is_in_clean_areas = false;
  // if (clear_by_areas_) {
  //   is_in_clean_areas = isPointInAreas(robot_pose);
  // }
  std::vector<std::shared_ptr<Layer>>::iterator layer;
  for (layer = layers_->begin(); layer != layers_->end(); ++layer) {
    if (!((*layer)->useForNavigation()) || !(*layer)->isActivate()) {
      continue;
    }

    // TODO: 参数化配置？
    // (*layer)->setLayerUpdate(true);
    // if (is_in_clean_areas) {
    //   if ((*layer)->getName() == "probabiltiy_voxel_layer" ||
    //       (*layer)->getName() == "negative_obstacle_layer") {
    //     (*layer)->setLayerUpdate(false);
    //   }
    // }

    // 在斜坡区域不更新概率体素层和负向障碍物层
    if (update_in_shope_) {
      if ((*layer)->getName() == "probabiltiy_voxel_layer" ||
          (*layer)->getName() == "negative_obstacle_layer") {
        continue;
      }
    } else {
      //通过斜坡区域后重置负向障碍物层
      if (reset_shope_layer_) {
        reset_shope_layer_ = true;
        if ((*layer)->getName() == "probabiltiy_voxel_layer" ||
            (*layer)->getName() == "negative_obstacle_layer") {
          (*layer)->reset();
        }
      }
    }
    CostmapBound prev_bound = cb_extremum_bound_;

    if (!(*layer)->updateBounds(robot_pose, cb_extremum_bound_)) {
      // LOG(INFO) << (*layer)->getName() << " updateBounds failed." <<
      // std::endl;
      return false;
    }

    if (cb_extremum_bound_.d_min_x > prev_bound.d_min_x ||
        cb_extremum_bound_.d_min_y > prev_bound.d_min_y ||
        cb_extremum_bound_.d_max_x < prev_bound.d_max_x ||
        cb_extremum_bound_.d_max_y < prev_bound.d_max_y) {
      LOG(ERROR) << "illegal bounds change." << std::endl;
      return false;
    }
  }

  int x0, xn, y0, yn;

  costmap_->worldToMapEnforceBounds(cb_extremum_bound_.d_min_x,
                                    cb_extremum_bound_.d_min_y, x0, y0);
  costmap_->worldToMapEnforceBounds(cb_extremum_bound_.d_max_x,
                                    cb_extremum_bound_.d_max_y, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(static_cast<int>(costmap_->getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(static_cast<int>(costmap_->getSizeInCellsY()), yn + 1);

  if (xn < x0 || yn < y0) {
    // maybe no data updated for there is no localization.
    LOG(ERROR) << "illegal enforcebounds change: min_x "
               << cb_extremum_bound_.d_min_x << ", max_x "
               << cb_extremum_bound_.d_max_x << ", min_y "
               << cb_extremum_bound_.d_min_y << ", max_y "
               << cb_extremum_bound_.d_max_y;
    return false;
  }

  // LOG(INFO) << "bound x0: " << x0 << ", y0: " << y0 << ", xn: " << xn
  //            << ", yn: " << yn;
  costmap_->resetMap(x0, y0, xn, yn);
  for (layer = layers_->begin(); layer != layers_->end(); ++layer) {
    if (!((*layer)->useForNavigation()) || !(*layer)->isActivate()) {
      continue;
    }
    (*layer)->updateCosts(costmap_, x0, y0, xn, yn);
  }
  // if (clear_by_areas_) {
  //   cleanMapByAreas(robot_pose, costmap_);
  // }

  cb_bound_.d_min_x = x0;
  cb_bound_.d_max_x = xn;
  cb_bound_.d_min_y = y0;
  cb_bound_.d_max_y = yn;

  initialized_ = true;

  return true;
}

bool LayeredCostmap::resizeMap(const unsigned int &size_x,
                               const unsigned int &size_y,
                               const double &resolution, const double &origin_x,
                               const double &origin_y,
                               const bool &size_locked) {
  size_locked_ = size_locked;
  costmap_->resizeMap(size_x, size_y, resolution, origin_x, origin_y);

  for (std::vector<std::shared_ptr<Layer>>::iterator layer = layers_->begin();
       layer != layers_->end(); ++layer) {
    (*layer)->matchSize();
  }
  return true;
}

bool LayeredCostmap::addLayer(const std::shared_ptr<Layer> layer) {
  if (nullptr != layer) {
    layers_->push_back(layer);
    return true;
  } else {
    return false;
  }
}

void LayeredCostmap::setLayerCurrent(const std::string &layer_name,
                                     const bool &current) {
  for (auto &ptr_layer : *layers_) {
    if (ptr_layer->getName() == layer_name) {
      ptr_layer->setCurrent(current);
      break;
    }
  }
}

std::shared_ptr<Costmap2d> LayeredCostmap::getLayerCostmapByname(
    const std::string layer_name) {
  for (auto &ptr_layer : *layers_) {
    if (ptr_layer->getName() == layer_name) {
      return ptr_layer->getLayerCostMap();
    }
  }
  LOG(ERROR) << "get layer_cost '" << layer_name << " ' failed.";
  return nullptr;
}
}  // namespace CVTE_BABOT
