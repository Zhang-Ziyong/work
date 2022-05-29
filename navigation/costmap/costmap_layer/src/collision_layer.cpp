/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file collision_layer.cpp
 *
 *@brief collision_layer 具体实现.
 *
 *@modified by chennuo(chennuo@cvte.com)
 *
 *@author chennuo(chennuo@cvte.com)
 *@data 2021-08-24
 ************************************************************************/
#include "collision_layer.hpp"

#include <time.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sstream>

#include "costmap_mediator.hpp"
#include "processor_utils.hpp"

namespace CVTE_BABOT {

CollisionLayer::CollisionLayer() { b_can_use_for_avoidance_ = true; }

bool CollisionLayer::onInitialize() {
  LOG(INFO) << "CollisionLayer::onInitialize ";

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

  return true;
}

void CollisionLayer::matchSize() {
  auto master = CostmapMediator::getPtrInstance()->getCostmap();

  double origin_x = master->getOriginX() + master->getSizeInMetersX() / 2 -
                    getSizeInMetersX() / 2;
  double origin_y = master->getOriginY() + master->getSizeInMetersY() / 2 -
                    getSizeInMetersY() / 2;

  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
            master->getResolution(), origin_x, origin_y);

  ptr_obs_time_.reset(
      new time_t[master->getSizeInCellsX() * master->getSizeInCellsY()]);
  time_t time_now = time(NULL);
  for (size_t i = 0; i < master->getSizeInCellsX() * master->getSizeInCellsY();
       i++) {
    ptr_obs_time_[i] = time_now;
  }
}

void CollisionLayer::getParams() {
  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".enabled", b_enabled_,
                                              true);
  if (!b_enabled_) {
    return;
  }

  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + ".topics", switch_topic_, std::string("collision_switch"));

  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".keep_time",
                                              obs_keep_duration_, 120.0);
  LOG(INFO) << "collision_layer topics: " << switch_topic_;
  LOG(INFO) << "keep time: " << obs_keep_duration_;

  b_rolling_window_ = CostmapMediator::getPtrInstance()->isRolling();
  bool b_mark_unknown_space =
      CostmapMediator::getPtrInstance()->isMarkingUnknown();
  if (b_mark_unknown_space) {
    uc_default_value_ = NO_INFORMATION;
  } else {
    uc_default_value_ = FREE_SPACE;
  }
}

bool CollisionLayer::resetMap() {
  if (CostmapMediator::getPtrInstance()->ifResetCostmap()) {
    resetMaps();
    return true;
  } else {
    return false;
  }
}

bool CollisionLayer::updateBounds(const WorldmapPose &wp_robot_pose,
                                  CostmapBound &cb_costmap_bound) {
  if (!b_enabled_) {
    return false;
  }
  return true;
}

bool CollisionLayer::updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                                 const int &i_min_i, const int &i_min_j,
                                 const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_) {
    return false;
  }
  std::shared_ptr<SensorSwitchObs> ptr_data = nullptr;
  std::stringstream ss_topics_range(switch_topic_);
  std::string topic_temp_range;
  if (!switch_topic_.empty()) {
    while (ss_topics_range >> topic_temp_range) {
      std::string ids_;
      CostmapMediator::getPtrInstance()->getParam(
          std::string(topic_temp_range + ".frame_id"), ids_, std::string(""));
      std::stringstream id_ss_(ids_);
      std::string id_temp_;
      while (id_ss_ >> id_temp_) {
        ptr_data = nullptr;
        if (!CostmapMediator::getPtrInstance()->getData(
                topic_temp_range + "." + id_temp_, ptr_data) ||
            !ptr_data)
          return true;
        // // 判断开关是否被触发，0表示未触发,
        // 如果数据时间和当前时间差超过10s则不处理
        if (ptr_data->status == true && time(NULL) - ptr_data->time < 10) {
          LOG(INFO) << "collision switch is crashed !!!!!!!!!!!!!!!!!!!!!!!";
          LOG(INFO) << "time sub:" << (time(NULL) - ptr_data->time);
          // 计算碰撞开关特征点在地图坐标系下的表达
          Eigen::Matrix3d R(Eigen::AngleAxisd(ptr_data->pose.d_yaw,
                                              Eigen::Vector3d::UnitZ()));
          Eigen::Vector3d t(ptr_data->pose.d_x, ptr_data->pose.d_y, 0.0);
          std::vector<double> touched_tfs = ptr_data->touched_tfs;
          for (int i = 0; i < (touched_tfs.size() / 2); i++) {
            double tf_x = touched_tfs[2 * i];      // 0  2  4
            double tf_y = touched_tfs[2 * i + 1];  // 1  3  5
            Eigen::Vector3d touched_switch_tf(tf_x, tf_y, 0.0);
            Eigen::Vector3d wp_touched_switch = R * touched_switch_tf + t;
            q_collision_data_.push_back(
                CollisionData(time(NULL), wp_touched_switch));
          }
        } else {
          // LOG(INFO) << "no new collision data";
        }
      }
    }
  } else {
    LOG(ERROR) << "collision_switch_range_topics not set!!!";
  }

  boost::shared_array<unsigned char> master = master_grid->getCharMap();
  int count_erase = 0;
  for (auto it = q_collision_data_.begin(); it != q_collision_data_.end();
       it++) {
    WorldmapPoint wm_point(it->pose(0), it->pose(1));
    CostmapPoint cp_point;
    if (!master_grid->worldToMap(wm_point, cp_point)) {
      continue;
    }
    unsigned int ui_index = master_grid->getIndex(cp_point.ui_x, cp_point.ui_y);
    master[ui_index] = LETHAL_OBSTACLE;
    time_t time_now = time(NULL);
    if (time_now - it->time > obs_keep_duration_) {
      count_erase++;
    }
  }

  while (!q_collision_data_.empty() && count_erase > 0) {
    --count_erase;
    q_collision_data_.pop_front();
  }
  return true;
}

void CollisionLayer::updateOrigin(const double &d_new_origin_x,
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

  boost::shared_array<time_t> ptr_local_obs_time =
      boost::shared_array<time_t>(new time_t[cell_size_x * cell_size_y]);

  copyMapRegion(ptr_uc_costmap_, lower_left_x, lower_left_y, ui_size_x_,
                local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  copyMapRegion(ptr_obs_time_, lower_left_x, lower_left_y, ui_size_x_,
                ptr_local_obs_time, 0, 0, cell_size_x, cell_size_x,
                cell_size_y);

  resetMaps();

  d_origin_x_ = new_grid_ox;
  d_origin_y_ = new_grid_oy;

  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  copyMapRegion(local_map, 0, 0, cell_size_x, ptr_uc_costmap_, start_x, start_y,
                ui_size_x_, cell_size_x, cell_size_y);

  copyMapRegion(ptr_local_obs_time, 0, 0, cell_size_x, ptr_obs_time_, start_x,
                start_y, ui_size_x_, cell_size_x, cell_size_y);
}

//预留接口
void CollisionLayer::activate() { b_enabled_ = true; }
void CollisionLayer::deactivate() {
  b_enabled_ = false;
  reset();
}
void CollisionLayer::reset() { q_collision_data_.clear(); }
}  // namespace CVTE_BABOT