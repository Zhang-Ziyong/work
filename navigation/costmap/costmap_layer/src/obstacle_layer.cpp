/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file obstacle_layer.cpp
 *
 *@brief obstacle layer 具体实现.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-09
 ************************************************************************/
#include "obstacle_layer.hpp"

#include <sstream>

#include "costmap_mediator.hpp"
#include "footprint_utils.hpp"

namespace CVTE_BABOT {

ObstacleLayer::ObstacleLayer() { b_can_use_for_avoidance_ = true; }

bool ObstacleLayer::onInitialize() {
  LOG(INFO) << "ObstacleLayer::onInitialize ";

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

  b_current_ = true;

  matchSize();

  createSubscribers();
  return true;
}

void ObstacleLayer::createSubscribers() {
  for (const auto &topic_name : v_marking_cloud_names_) {
    auto ptr_cloud = std::make_shared<CostmapCloud>();
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>(topic_name,
                                                            ptr_cloud);
    LOG(INFO) << "register marking topic " << topic_name;
  }

  for (const auto &topic_name : v_clearing_cloud_names_) {
    auto ptr_cloud = std::make_shared<CostmapCloud>();
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>(topic_name,
                                                            ptr_cloud);
  }  // TODO(hubery) : 创建相同类型的订阅会重复赋值

  for (const auto &topic_name : v_avoidance_cloud_names_) {
    auto ptr_cloud = std::make_shared<CostmapCloud>();
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>(topic_name,
                                                            ptr_cloud);

    // match with FIELD_NUM to avoid reallocate
    // memory when executing push_back.
    m_v_field_index_[topic_name].reserve(FIELD_NUM);

    LOG(INFO) << "register avoidance topic " << topic_name;
  }
}

void ObstacleLayer::getParams() {
  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".enabled", b_enabled_,
                                              true);
  if (!b_enabled_) {
    return;
  }
  if (b_can_use_for_avoidance_) {
    CostmapMediator::getPtrInstance()->getParam(s_name_ + ".use_for_avoidance",
                                                b_use_for_avoidance_, true);
  }
  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".use_for_navigation",
                                              b_use_for_navigation_, true);

  b_rolling_window_ = CostmapMediator::getPtrInstance()->isRolling();
  bool b_mark_unknown_space =
      CostmapMediator::getPtrInstance()->isMarkingUnknown();
  if (b_mark_unknown_space) {
    uc_default_value_ = NO_INFORMATION;
  } else {
    uc_default_value_ = FREE_SPACE;
  }

  loadAvandanceBound();

  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + ".footprint_clearing_enabled", b_footprint_clearing_enabled_,
      true);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".max_obstacle_height",
                                              d_max_obstacle_height_, 2.0);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".min_obstacle_height",
                                              d_min_obstacle_height_, -1.0);

  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".obstacle_range",
                                              d_obstacle_range_, 6.0);
  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".raytrace_range",
                                              d_raytrace_range_, 6.0);

  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".combination_method",
                                              i_combination_method_, 0);

  std::string marking_topics;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + ".marking_cloud_sources", marking_topics, std::string(""));
  {
    std::stringstream ss(marking_topics);
    std::string source;
    while (ss >> source) {
      LOG(INFO) << "marking_cloud_source : " << source;
      std::string frame_id;
      CostmapMediator::getPtrInstance()->getParam(source + ".frame_id",
                                                  frame_id, std::string(""));

      v_marking_cloud_names_.push_back(source);
    }
  }

  std::string clearing_topics;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + ".clearing_cloud_sources", clearing_topics, std::string(""));
  {
    std::stringstream ss(clearing_topics);
    std::string source;
    while (ss >> source) {
      LOG(INFO) << "clearing_cloud_sources : " << source;
      std::string frame_id;
      CostmapMediator::getPtrInstance()->getParam(source + ".frame_id",
                                                  frame_id, std::string(""));

      v_clearing_cloud_names_.push_back(source);
    }
  }

  std::string avoidance_topics;
  // 默认为空，不订阅避障消息
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + ".avoidance_cloud_sources", avoidance_topics, std::string(""));
  {
    std::stringstream ss(avoidance_topics);
    std::string source;
    while (ss >> source) {
      LOG(INFO) << "avoidance_cloud_sources : " << source;
      v_avoidance_cloud_names_.push_back(source);
    }
  }
}

bool ObstacleLayer::getAvoidanceClouds(
    std::vector<std::shared_ptr<const CostmapCloud>> &v_ptr_clouds) const {
  bool b_current = true;

  std::shared_ptr<CostmapCloud> ptr_cloud = nullptr;
  for (const auto &topic_name : v_avoidance_cloud_names_) {
    bool get_data_status =
        CostmapMediator::getPtrInstance()->getData(topic_name, ptr_cloud);
    if (get_data_status) {
      v_ptr_clouds.push_back(ptr_cloud->transformToBaseLink());
    } else {
      LOG(ERROR) << topic_name << " not regitstered.";
      b_current = false;
    }
  }

  return b_current;
}

bool ObstacleLayer::getMarkingClouds(
    std::vector<std::shared_ptr<const CostmapCloud>> &v_ptr_clouds) const {
  bool b_current = true;

  std::shared_ptr<CostmapCloud> ptr_cloud = nullptr;
  for (const auto &topic_name : v_marking_cloud_names_) {
    if (!CostmapMediator::getPtrInstance()->getData(topic_name, ptr_cloud)) {
      LOG(ERROR) << topic_name << " not regitstered.";
      b_current = false;
      continue;
    }
    if (ptr_cloud->is_used_to_mark_) {
      continue;
    }
    ptr_cloud->is_used_to_mark_ = true;
    if (ptr_cloud->b_need_transformed_) {
      v_ptr_clouds.push_back(ptr_cloud->transformToMap());
    } else {
      v_ptr_clouds.push_back(ptr_cloud);
    }
  }

  return b_current;
}

bool ObstacleLayer::getClearingClouds(
    std::vector<std::shared_ptr<const CostmapCloud>> &v_ptr_clouds) const {
  bool b_current = true;

  std::shared_ptr<CostmapCloud> ptr_cloud = nullptr;
  for (const auto &topic_name : v_clearing_cloud_names_) {
    if (!CostmapMediator::getPtrInstance()->getData(topic_name, ptr_cloud)) {
      LOG(ERROR) << topic_name << " not regitstered.";
      b_current = false;
      continue;
    }
    if (ptr_cloud->is_used_to_clear_) {
      continue;
    }
    ptr_cloud->is_used_to_clear_ = true;
    if (ptr_cloud->b_need_transformed_) {
      v_ptr_clouds.push_back(ptr_cloud->transformToMap());
    } else {
      v_ptr_clouds.push_back(ptr_cloud);
    }
  }

  return b_current;
}

void ObstacleLayer::raytraceFreespace(const CostmapCloud &clearing_clouds,
                                      CostmapBound &costmap_bound) {
  if (clearing_clouds.ptr_v_cloud_->empty()) {
    return;
  }

  WorldmapPoint wm_cloud_origin{clearing_clouds.origin_.d_x,
                                clearing_clouds.origin_.d_y};

  CostmapPoint cp_point_begin;

  if (!worldToMap(wm_cloud_origin, cp_point_begin)) {
    LOG(ERROR) << "The origin of the topic: " << clearing_clouds.s_topic_name_
               << ", x: " << clearing_clouds.origin_.d_x
               << ", y: " << clearing_clouds.origin_.d_y
               << " is out of map bounds. "
               << "Please check if the scan or the localization is updated.";
    return;
  }

  WorldmapPoint wm_map_origin = {d_origin_x_, d_origin_y_};
  WorldmapPoint wm_map_end = {d_origin_x_ + ui_size_x_ * d_resolution_,
                              d_origin_y_ + +ui_size_y_ * d_resolution_};

  touch(wm_cloud_origin, costmap_bound);

  std::vector<CostmapPointXYZ> v_cp_xyz = *(clearing_clouds.ptr_v_cloud_);
  for (const auto &cp_xyz : *clearing_clouds.ptr_v_cloud_) {
    WorldmapPoint wm_point = {cp_xyz.d_x, cp_xyz.d_y};

    WorldmapPoint delta_distance = wm_point - wm_cloud_origin;

    //等比平移
    if (wm_point.d_x < wm_map_origin.d_x) {
      double scale =
          (wm_map_origin.d_x - wm_cloud_origin.d_x) / delta_distance.d_x;
      wm_point = {wm_map_origin.d_x,
                  wm_cloud_origin.d_y + delta_distance.d_y * scale};
    }

    if (wm_point.d_y < wm_map_origin.d_y) {
      double scale =
          (wm_map_origin.d_y - wm_cloud_origin.d_y) / delta_distance.d_y;
      wm_point = {wm_cloud_origin.d_x + delta_distance.d_x * scale,
                  wm_map_origin.d_y};
    }

    if (wm_point.d_x > wm_map_end.d_x) {
      double scale =
          (wm_map_end.d_x - wm_cloud_origin.d_x) / delta_distance.d_x;
      wm_point = {wm_map_end.d_x - .001,
                  wm_cloud_origin.d_y + delta_distance.d_y * scale};
    }

    if (wm_point.d_y > wm_map_end.d_y) {
      double scale =
          (wm_map_end.d_y - wm_cloud_origin.d_y) / delta_distance.d_y;
      wm_point = {wm_cloud_origin.d_x + delta_distance.d_x * scale,
                  wm_map_end.d_y - .001};
    }

    CostmapPoint cp_point_end;
    if (!worldToMap(wm_point, cp_point_end)) {
      continue;
    }

    unsigned int cell_raytrace_range = cellDistance(d_raytrace_range_);
    MarkCell marker(ptr_uc_costmap_, FREE_SPACE);

    raytraceLine(marker, cp_point_begin.ui_x, cp_point_begin.ui_y,
                 cp_point_end.ui_x, cp_point_end.ui_y, cell_raytrace_range);

    updateRaytraceBounds(wm_cloud_origin, wm_point, d_raytrace_range_,
                         costmap_bound);
  }
}

void ObstacleLayer::updateRaytraceBounds(const WorldmapPoint &wp_origin,
                                         const WorldmapPoint &wp_point,
                                         const double &range,
                                         CostmapBound &costmap_bound) {
  static WorldmapPoint distance_delta, expected_point;
  distance_delta = wp_point - wp_origin;

  static double full_distance, scale;
  full_distance = hypot(distance_delta.d_x, distance_delta.d_y);
  scale = std::min(1.0, range / full_distance);

  expected_point = {wp_origin.d_x + distance_delta.d_x * scale,
                    wp_origin.d_y + distance_delta.d_y * scale};
  touch(expected_point, costmap_bound);
}

bool ObstacleLayer::resetMap() {
  if (CostmapMediator::getPtrInstance()->ifResetCostmap()) {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    resetMaps();
    return true;
  } else {
    return false;
  }
}

bool ObstacleLayer::updateBounds(const WorldmapPose &wp_robot_pose,
                                 CostmapBound &cb_costmap_bound) {
  if (!b_enabled_) {
    return false;
  }
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (resetMap()) {
    LOG(INFO) << "Reset obstacle layer map. ";
  }

  if (b_rolling_window_) {
    updateOrigin(wp_robot_pose.d_x - getSizeInMetersX() / 2,
                 wp_robot_pose.d_y - getSizeInMetersY() / 2);
  }

  useExtraBounds(cb_costmap_bound);

  std::vector<std::shared_ptr<const CostmapCloud>> clouds, clearing_clouds;

  // get the marking observations
  getMarkingClouds(clouds);

  // get the clearing observations
  if (v_marking_cloud_names_ == v_clearing_cloud_names_) {
    clearing_clouds = clouds;
  } else {
    getClearingClouds(clearing_clouds);
  }

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_clouds.size(); ++i) {
    raytraceFreespace(*clearing_clouds[i], cb_costmap_bound);
  }

  double sq_obstacle_range_threshold = d_obstacle_range_ * d_obstacle_range_;
  for (const auto &cloud : clouds) {
    for (const auto &point : *cloud->ptr_v_cloud_) {
      if (point.d_z > d_max_obstacle_height_) {
        continue;
      }

      // double sq_dist =
      //     (point.d_x - cloud->origin_.d_x) * (point.d_x - cloud->origin_.d_x)
      //     + (point.d_y - cloud->origin_.d_y) * (point.d_y -
      //     cloud->origin_.d_y) + (point.d_z - cloud->origin_.d_z) * (point.d_z
      //     - cloud->origin_.d_z);

      // if (sq_dist >= sq_obstacle_range_threshold) {
      //   continue;
      // }

      WorldmapPoint wm_point = {point.d_x, point.d_y};
      CostmapPoint cp_point;
      if (!worldToMap(wm_point, cp_point)) {
        continue;
      }

      unsigned int ui_index = getIndex(cp_point.ui_x, cp_point.ui_y);

      ptr_uc_costmap_[ui_index] = LETHAL_OBSTACLE;
      touch(wm_point, cb_costmap_bound);
    }
  }

  updateFootprint(wp_robot_pose, cb_costmap_bound);
  return true;
}

void ObstacleLayer::updateFootprint(const WorldmapPose &, CostmapBound &) {
  // 本函数未实现
  // TODO(hubery) : 实现本函数
  if (!b_footprint_clearing_enabled_) {
    return;
  }
}

bool ObstacleLayer::updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                                const int &i_min_i, const int &i_min_j,
                                const int &i_max_i, const int &i_max_j) {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (!b_enabled_) {
    return false;
  }

  if (b_footprint_clearing_enabled_) {
    setConvexPolygonCost(v_transformed_footprint_, FREE_SPACE);
  }

  switch (i_combination_method_) {
    case 0:
      updateWithOverwrite(master_grid, i_min_i, i_min_j, i_max_i, i_max_j);
      break;
    case 1:
      updateWithMax(master_grid, i_min_i, i_min_j, i_max_i, i_max_j);
    default:
      break;
  }
  return true;
}

//预留接口
void ObstacleLayer::activate() { b_enabled_ = true; }
void ObstacleLayer::deactivate() {
  b_enabled_ = false;
  reset();
}
void ObstacleLayer::reset() {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  resetMaps();
}
}  // namespace CVTE_BABOT