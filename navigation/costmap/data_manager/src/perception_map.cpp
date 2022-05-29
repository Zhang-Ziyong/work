/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 20220, CVTE.
 * All rights reserved.
 *
 *@file perception_map.cpp
 *
 *@brief 感知地图类, 保存栅格地图及动态障碍信息
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.3
 *@data 2020-04-15
 ************************************************************************/
#include <glog/logging.h>

#include "costmap_mediator.hpp"
#include "perception_map.hpp"

namespace CVTE_BABOT {
PerceptionMap::PerceptionMap(const unsigned int& ui_size_x,
                             const unsigned int& ui_size_y)
    : ui_size_x_(ui_size_x), ui_size_y_(ui_size_y) {
  perception_mat_ = cv::Mat::zeros(ui_size_x_, ui_size_y_, CV_8UC3);
  ptr_grid_data_ =
      boost::shared_array<GridStatus>(new GridStatus[ui_size_x_ * ui_size_y_]);

  // 提前分配内存
  vv_ui_correspond_grids_.resize(MAX_ID_NUM);
  for (size_t i = 0; i < MAX_ID_NUM; i++) {
    vv_ui_correspond_grids_[i].reserve(BUFFER_SIZE);
  }

  // 先做默认初始化,防止在数据未来时被调用出错.
  ptr_costmap_ = std::make_shared<Costmap2d>();
  ptr_dynamic_objects_ = std::make_shared<DynamicObstacles>();
}

PerceptionMap::PerceptionMap(const PerceptionMap& perception_map) {
  copyPerceptionMap(perception_map);
}

PerceptionMap& PerceptionMap::operator=(const PerceptionMap& perception_map) {
  if (this == &perception_map) {
    return *this;
  }

  copyPerceptionMap(perception_map);

  return *this;
}

void PerceptionMap::copyPerceptionMap(const PerceptionMap& perception_map) {
  // 必须复制所有变量, 不然没被显著拷贝的变量会被赋默认值而不会被拷贝
  std::lock_guard<std::mutex> lock(perception_map.mutex_);

  // 浅拷贝
  ui_size_x_ = perception_map.ui_size_x_;
  ui_size_y_ = perception_map.ui_size_y_;
  vv_ui_correspond_grids_ = perception_map.vv_ui_correspond_grids_;
  ptr_dynamic_objects_ = perception_map.ptr_dynamic_objects_;
  ptr_costmap_ = perception_map.ptr_costmap_;

  // 深拷贝
  perception_mat_ = perception_map.perception_mat_.clone();
  // 注意需要先拷贝ui_size_x_和ui_size_y_
  ptr_grid_data_ =
      boost::shared_array<GridStatus>(new GridStatus[ui_size_x_ * ui_size_y_]);
  memcpy(ptr_grid_data_.get(), perception_map.ptr_grid_data_.get(),
         ui_size_x_ * ui_size_y_ * sizeof(GridStatus));
}

bool PerceptionMap::worldToMap(const double& d_x, const double& d_y,
                               unsigned int& ui_x, unsigned int& ui_y) const {
  CostmapPoint cp_point;
  if (ptr_costmap_->worldToMap({d_x, d_y}, cp_point)) {
    ui_x = cp_point.ui_x;
    ui_y = cp_point.ui_y;
    return true;
  } else {
    return false;
  }
}

bool PerceptionMap::getGridStatus(const double& d_x, const double& d_y,
                                  GridStatus& grid_status) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (ptr_costmap_ == nullptr) {
    LOG(ERROR) << "ptr_costmap_ has not been updated";
    return false;
  }

  unsigned int ui_x, ui_y;
  if (worldToMap(d_x, d_y, ui_x, ui_y)) {
    grid_status = getGrid(ui_x, ui_y);
    return true;
  } else {
    return false;
  }
}

bool PerceptionMap::setGridStatus(const double& d_x, const double& d_y,
                                  const GridStatus& target_status) {
  unsigned int ui_x = 0, ui_y = 0;
  if (!worldToMap(d_x, d_y, ui_x, ui_y)) {
    return false;
  }
  GridStatus current_status = getGrid(ui_x, ui_y);

  // 要更新的目标值比原先的高才更新
  if (target_status.value > current_status.value) {
    setGrid(ui_x, ui_y, target_status);
    return true;
  } else {
    return false;
  }
}

bool PerceptionMap::getObject(const unsigned int& id,
                              DynamicObject& dynamic_object) {
  std::lock_guard<std::mutex> lock(mutex_);

  for (size_t i = 0; i < ptr_dynamic_objects_->v_dynamic_object_.size(); i++) {
    const DynamicObject& object = ptr_dynamic_objects_->v_dynamic_object_[i];
    if (id == object.id) {
      dynamic_object = object;
      return true;
    }
  }
  return false;
}

std::vector<unsigned int> PerceptionMap::getCorrespondGrids(
    const unsigned int& id) const {
  assert(id < MAX_ID_NUM);
  return vv_ui_correspond_grids_.at(id);
}

void PerceptionMap::updateDynamicGrids(const DynamicObject& dynamic_object) {
  GridStatus grid_status;
  // LOG(ERROR) << "obstacle_type: " << dynamic_object.obstacle_type;

  for (size_t i = 0; i < dynamic_object.ptr_v_cloud_->size(); i++) {
    grid_status.obstacle_type = dynamic_object.obstacle_type;
    grid_status.value = 255 + grid_status.obstacle_type * 10;
    grid_status.id = dynamic_object.id;

    const auto& point = (*dynamic_object.ptr_v_cloud_)[i];

    // 如果超出地图或已存在更高级的障碍则不会更新
    if (!setGridStatus(point.d_x, point.d_y, grid_status)) {
      // LOG(ERROR) << "setGridStatus false.";
    }
  }
}

void PerceptionMap::updateCorrespondGrids() {
  // 先清除对应关系
  for (size_t i = 0; i < MAX_ID_NUM; i++) {
    vv_ui_correspond_grids_.at(i).clear();
  }

  for (size_t i = 0; i < ui_size_x_ * ui_size_y_; i++) {
    unsigned int id = ptr_grid_data_[i].id;
    if (id != 0) {
      vv_ui_correspond_grids_.at(id).push_back(i);
    }
  }
}

std::shared_ptr<Costmap2d> PerceptionMap::getCostmapResetCorGrids(
    const unsigned int& id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::shared_ptr<Costmap2d> ptr_costmap =
      std::make_shared<Costmap2d>(*ptr_costmap_);

  std::vector<unsigned int> v_correspond_index = getCorrespondGrids(id);
  LOG(ERROR) << "id: " << id
             << ", correspondgrids size: " << v_correspond_index.size();

  for (size_t i = 0; i < v_correspond_index.size(); i++) {
    unsigned int index = v_correspond_index[i];

    unsigned int ui_x = 0, ui_y = 0;
    ptr_costmap->indexToCells(index, ui_x, ui_y);
    ptr_costmap->setCost(ui_x, ui_y, FREE_SPACE);
  }

  return ptr_costmap;
}

void PerceptionMap::inflateMap(const unsigned int& ui_inflate_grid_radius) {
  // TODO(hubery) :循环嵌套太多,不易理解
  for (size_t j = 0; j < ui_size_y_; j++) {
    for (size_t i = 0; i < ui_size_x_; i++) {
      GridStatus grid_status = getGrid(i, j);

      if (grid_status.value == 255 + 2 * 10 ||
          grid_status.value == 255 + 1 * 10) {
        // 计算膨胀的栅格范围
        int ix_min = std::max(0, static_cast<int>(i - ui_inflate_grid_radius));
        int iy_min = std::max(0, static_cast<int>(j - ui_inflate_grid_radius));
        int ix_max = std::min(static_cast<int>(ui_size_x_ - 1),
                              static_cast<int>(i + ui_inflate_grid_radius));
        int iy_max = std::min(static_cast<int>(ui_size_y_ - 1),
                              static_cast<int>(j + ui_inflate_grid_radius));

        GridStatus new_grid_status;
        new_grid_status.value = 255 + grid_status.obstacle_type * 10 - 1;
        new_grid_status.id = grid_status.id;
        new_grid_status.obstacle_type = grid_status.obstacle_type;

        for (int ix = ix_min; ix <= ix_max; ix++) {
          for (int iy = iy_min; iy <= iy_max; iy++) {
            GridStatus old_grid_status = getGrid(ix, iy);

            if (new_grid_status.value > old_grid_status.value) {
              setGrid(ix, iy, new_grid_status);
            }
          }
        }
      }
    }
  }
}

void PerceptionMap::updateVisMat() {
  using namespace cv;
  perception_mat_ = Mat::zeros(ui_size_x_, ui_size_y_, CV_8UC3);

  for (size_t j = 0; j < ui_size_x_; j++) {
    for (size_t i = 0; i < ui_size_y_; i++) {
      GridStatus grid_status = getGrid(j, i);
      if (grid_status.value != 0) {
        if (grid_status.value <= LETHAL_OBSTACLE) {
          circle(perception_mat_, Point(ui_size_y_ - i, ui_size_x_ - j), 1,
                 Scalar(255, 0, 0), 1);
        } else if (grid_status.value <= 255 + 1 * 10) {
          circle(perception_mat_, Point(ui_size_y_ - i, ui_size_x_ - j), 1,
                 Scalar(0, 255, 0), 1);
        } else if (grid_status.value <= 255 + 2 * 10) {
          circle(perception_mat_, Point(ui_size_y_ - i, ui_size_x_ - j), 1,
                 Scalar(0, 0, 255), 1);
        }
      }
    }
  }
}

void PerceptionMap::updateData() {
  // update dynamic_object
  std::string dynamic_obs_topic;
  CostmapMediator::getPtrInstance()->getParam(
      std::string("data_manager.dynamic_obs_topic"), dynamic_obs_topic,
      std::string("dynamic_obstacles"));

  auto ptr_dynamic_obstacles = std::make_shared<DynamicObstacles>();
  CostmapMediator::getPtrInstance()->getData(dynamic_obs_topic + "_obj",
                                             ptr_dynamic_obstacles);
  assert(ptr_dynamic_obstacles != nullptr);
  ptr_dynamic_objects_ = ptr_dynamic_obstacles;
}

void PerceptionMap::updateMap() {
  std::lock_guard<std::mutex> lock(mutex_);

  // 更新地图前先更新所需的数据
  updateData();

  resetGridData();
  // 更新过程没对costmap加锁, 仍让其同步更新
  // TODO(hubery): 拷贝一份costmap或对其加锁, 评估哪种方式合适
  assert(ui_size_x_ = ptr_costmap_->getSizeInCellsX());
  assert(ui_size_y_ = ptr_costmap_->getSizeInCellsY());

  GridStatus grid_status;
  // 拷贝costmap的值, TODO(hubery): 写成一个函数
  for (size_t i = 0; i < ui_size_x_; i++) {
    for (size_t j = 0; j < ui_size_y_; j++) {
      unsigned char cost = ptr_costmap_->getCost(i, j);
      if (cost != FREE_SPACE && cost != NO_INFORMATION) {
        grid_status.value = static_cast<unsigned int>(cost);
        setGrid(i, j, grid_status);
      }
    }
  }

  for (size_t i = 0; i < ptr_dynamic_objects_->v_dynamic_object_.size(); i++) {
    updateDynamicGrids(ptr_dynamic_objects_->v_dynamic_object_[i]);
  }

  inflateMap(8);

  updateCorrespondGrids();

  updateVisMat();
}

void PerceptionMap::resetGridData() {
  memset(ptr_grid_data_.get(), 0, ui_size_x_ * ui_size_y_ * sizeof(GridStatus));
}

}  // namespace CVTE_BABOT