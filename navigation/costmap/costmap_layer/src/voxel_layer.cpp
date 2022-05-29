/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file voxel_layer.cpp
 *
 *@brief voxel layer 具体实现.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.2.1
 *@data 2019-08-26
 ************************************************************************/
#include "voxel_layer.hpp"

#include <sstream>

#include "costmap_mediator.hpp"

const int VOXEL_BITS = 16;

namespace CVTE_BABOT {
bool VoxelLayer::onInitialize() {
  if (!ObstacleLayer::onInitialize()) {
    LOG(ERROR) << "onInitialize failed";
    return false;
  }

  if (b_publish_clearing_points_) {
    auto ptr_cloud = std::make_shared<CostmapCloud>();
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>("clearing_points",
                                                            ptr_cloud);
  }

  if (b_publish_voxel_) {
    auto ptr_cloud = std::make_shared<CostmapCloud>();
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>("voxel_points",
                                                            ptr_cloud);
  }
  return true;
}

void VoxelLayer::getParams() {
  ObstacleLayer::getParams();

  if (!b_enabled_) {
    return;
  }

  int unknown_threshold, mark_threshold, size_z;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "footprint_clearing_enabled",
      b_footprint_clearing_enabled_, false);

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "origin_z",
                                              d_origin_z_, 0.0);

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "z_resolution",
                                              d_resolution_z_, 0.1);

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "z_voxels",
                                              size_z, 16);
  ui_size_z_ = size_z;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "unknown_threshold", unknown_threshold, 10);
  unknown_threshold_ = unknown_threshold;
  unknown_threshold_ = unknown_threshold_ + (VOXEL_BITS - ui_size_z_);

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "mark_threshold",
                                              mark_threshold, 0);
  mark_threshold_ = mark_threshold;
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "clearing_time_threshold", d_clearing_time_threshold_,
      5.0);

  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "quick_clearing_bit_threshold",
      i_quick_clearing_bit_threshold_, 8);
  CostmapMediator::getPtrInstance()->getParam(
      s_name_ + "." + "quick_clearing_time_threshold",
      d_quick_clearing_time_threshold_, 1.0);

  CostmapMediator::getPtrInstance()->getParam("publish_voxel", b_publish_voxel_,
                                              false);
  CostmapMediator::getPtrInstance()->getParam(
      "publish_clearing_points", b_publish_clearing_points_, false);
}

void VoxelLayer::matchSize() {
  LOG(INFO) << "VoxelLayer matchSize";
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  auto ptr_costmap = CostmapMediator::getPtrInstance()->getCostmap();
  ptr_d_obstacle_time_.reset(new timeType[ptr_costmap->getSizeInCellsX() *
                                          ptr_costmap->getSizeInCellsX()]);
  ObstacleLayer::matchSize();
  voxel_grid_.resize({ui_size_x_, ui_size_y_, ui_size_z_});
  assert(voxel_grid_.sizeX() == ui_size_x_ &&
         voxel_grid_.sizeY() == ui_size_y_);
  LOG(INFO) << "VoxelLayer matchSize finished ";
}

void VoxelLayer::resetMaps() {
  Costmap2d::resetMaps();
  timeType time_now = time(NULL);
  for (size_t i = 0; i < ui_size_x_ * ui_size_y_; i++) {
    ptr_d_obstacle_time_[i] = time_now;
  }

  voxel_grid_.reset();
}

bool VoxelLayer::updateBounds(const WorldmapPose &wp_robot_pose,
                              CostmapBound &cb_costmap_bound) {
  // LOG(ERROR) << "VoxelLayer updateBounds";
  if (!b_enabled_) {
    return false;
  }
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (b_rolling_window_) {
    updateOrigin(wp_robot_pose.d_x - getSizeInMetersX() / 2,
                 wp_robot_pose.d_y - getSizeInMetersY() / 2);
  }

  useExtraBounds(cb_costmap_bound);

  std::vector<std::shared_ptr<const CostmapCloud>> clouds, clearing_clouds;

  // get the marking observations
  getMarkingClouds(clouds);

  // get the clearing observations
  getClearingClouds(clearing_clouds);

  // 存储清除射线的末端点
  if (b_publish_clearing_points_) {
    v_clearing_endpoints_.clear();
  }
  // raytrace freespace
  for (unsigned int i = 0; i < clearing_clouds.size(); ++i) {
    raytraceFreespace(*clearing_clouds[i], cb_costmap_bound);
  }

  if (b_publish_clearing_points_) {
    auto ptr_clearing_cloud = std::make_shared<CostmapCloud>();
    *ptr_clearing_cloud->ptr_v_cloud_ = v_clearing_endpoints_;

    CostmapMediator::getPtrInstance()->updateData("clearing_points",
                                                  ptr_clearing_cloud);
  }

  static double sq_dist;
  static WorldmapPoint wm_point;
  static double sq_obstacle_range_threshold;
  static unsigned int ui_index;

  timeType time_now = time(NULL);

  for (size_t i = 0; i < ui_size_x_ * ui_size_y_; i++) {
    if (ptr_uc_costmap_[i] == LETHAL_OBSTACLE) {
      if (time_now - ptr_d_obstacle_time_[i] > d_clearing_time_threshold_) {
        ptr_uc_costmap_[i] = FREE_SPACE;
        voxel_grid_.clearVoxelColumn(i);
      } else if (time_now - ptr_d_obstacle_time_[i] >
                 d_quick_clearing_time_threshold_) {
        // 一定高度以下的栅格为空,则清除该栅格,不管超过该高度
        // 的栅格状态,因为超过一定高度的栅格激光容易扫描不到
        // FREE是体素状态,不要与FREE_SPACE搞混了
        if (voxel_grid_.getVoxelsBelowBit(i, i_quick_clearing_bit_threshold_,
                                          unknown_threshold_,
                                          mark_threshold_) == FREE) {
          ptr_uc_costmap_[i] = FREE_SPACE;
        }
      }
    }
  }

  sq_obstacle_range_threshold = d_obstacle_range_ * d_obstacle_range_;
  for (const auto &cloud : clouds) {
    for (const auto &point : *cloud->ptr_v_cloud_) {
      if (point.d_z > d_max_obstacle_height_ ||
          point.d_z < d_min_obstacle_height_) {
        continue;
      }

      sq_dist =
          (point.d_x - cloud->origin_.d_x) * (point.d_x - cloud->origin_.d_x) +
          (point.d_y - cloud->origin_.d_y) * (point.d_y - cloud->origin_.d_y) +
          (point.d_z - cloud->origin_.d_z) * (point.d_z - cloud->origin_.d_z);
      if (sq_dist >= sq_obstacle_range_threshold) {
        continue;
      }

      VoxelPoint vp_map;
      if (point.d_z < d_origin_z_) {
        if (!worldToMap3D({point.d_x, point.d_y, d_origin_z_}, vp_map)) {
          continue;
        }
      } else if (!worldToMap3D(point, vp_map)) {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in
      // the costmap
      if (voxel_grid_.markVoxelInMap(vp_map, mark_threshold_)) {
        ui_index = getIndex(vp_map.ui_x, vp_map.ui_y);
        ptr_uc_costmap_[ui_index] = LETHAL_OBSTACLE;

        ptr_d_obstacle_time_[ui_index] = time_now;
        touch(wm_point, cb_costmap_bound);
      }
    }
  }

  updateFootprint(wp_robot_pose, cb_costmap_bound);

  if (b_publish_voxel_) {
    auto ptr_voxel_points_cloud = std::make_shared<CostmapCloud>();

    const uint32_t *data = voxel_grid_.getData();
    const double x_origin = d_origin_x_;
    const double y_origin = d_origin_y_;
    const double z_origin = d_origin_z_;
    const double x_res = d_resolution_;
    const double y_res = d_resolution_;
    const double z_res = d_resolution_z_;
    const uint32_t x_size = voxel_grid_.sizeX();
    const uint32_t y_size = voxel_grid_.sizeY();
    const uint32_t z_size = voxel_grid_.sizeZ();

    CostmapPointXYZ point;
    ptr_voxel_points_cloud->ptr_v_cloud_->reserve(x_size * y_size * z_size);
    for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid) {
      for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid) {
        for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid) {
          VoxelStatus status = VoxelGrid::getVoxel(
              {x_grid, y_grid, z_grid}, {x_size, y_size, z_size}, data);

          if (status == MARKED) {
            point.d_x = x_origin + (x_grid + 0.5) * x_res;
            point.d_y = y_origin + (y_grid + 0.5) * y_res;
            point.d_z = z_origin + (z_grid + 0.5) * z_res;
            ptr_voxel_points_cloud->ptr_v_cloud_->push_back(point);
          }
        }
      }
    }

    CostmapMediator::getPtrInstance()->updateData("voxel_points",
                                                  ptr_voxel_points_cloud);
  }

  return true;
}

void VoxelLayer::clearNonLethal(const WorldmapPoint &wp_point,
                                const WorldmapPoint &wp_size,
                                const bool &clear_no_info) {
  // get the cell coordinates of the center point of the window
  CostmapPoint cp_point;
  if (!worldToMap(wp_point, cp_point)) {
    return;
  }

  // compute the bounds of the window
  double start_x = wp_point.d_x - wp_size.d_x / 2;
  double start_y = wp_point.d_y - wp_size.d_y / 2;
  double end_x = start_x + wp_size.d_x;
  double end_y = start_y + wp_size.d_y;

  // scale the window based on the bounds of the costmap
  start_x = std::max(d_origin_x_, start_x);
  start_y = std::max(d_origin_y_, start_y);

  end_x = std::min(d_origin_x_ + getSizeInMetersX(), end_x);
  end_y = std::min(d_origin_y_ + getSizeInMetersY(), end_y);

  CostmapPoint cp_point_s, cp_point_e;

  // check for legality just in case
  if (!worldToMap({start_x, start_y}, cp_point_s) ||
      !worldToMap({end_x, end_y}, cp_point_e))
    return;

  // we know that we want to clear all non-lethal obstacles in this window to
  // get it ready for inflation
  unsigned int index = getIndex(cp_point_s.ui_x, cp_point_s.ui_y);
  unsigned char *current = &ptr_uc_costmap_[index];
  for (unsigned int j = cp_point_s.ui_y; j <= cp_point_e.ui_y; ++j) {
    for (unsigned int i = cp_point_s.ui_x; i <= cp_point_e.ui_y; ++i) {
      // if the cell is a lethal obstacle... we'll keep it and queue it,
      // otherwise... we'll clear it
      if (*current != LETHAL_OBSTACLE) {
        if (clear_no_info || *current != NO_INFORMATION) {
          *current = FREE_SPACE;
          voxel_grid_.clearVoxelColumn(index);
        }
      }
      current++;
      index++;
    }
    current += ui_size_x_ - (cp_point_e.ui_x - cp_point_s.ui_x) - 1;
    index += ui_size_x_ - (cp_point_e.ui_x - cp_point_s.ui_x) - 1;
  }
}

void VoxelLayer::raytraceFreespace(const CostmapCloud &clearing_clouds,
                                   CostmapBound &costmap_bound) {
  if (clearing_clouds.ptr_v_cloud_->empty()) {
    return;
  }

  const CostmapPointXYZ &cp_origin = clearing_clouds.origin_;
  CostmapPointXYZ cp_sensor;
  if (!worldToMap3DFloat(cp_origin, cp_sensor)) {
    LOG(ERROR)
        << "The origin for the sensor at (" << cp_origin.d_x << ", "
        << cp_origin.d_y << ", " << cp_origin.d_z
        << ") is out of map bounds. So, the costmap cannot raytrace for it.";
    return;
  }

  // need it ?
  touch({clearing_clouds.origin_.d_x, clearing_clouds.origin_.d_y},
        costmap_bound);

  // we can pre-compute the enpoints of the map outside of the inner loop...
  // we'll need these later
  double map_end_x = d_origin_x_ + getSizeInMetersX();
  double map_end_y = d_origin_y_ + getSizeInMetersY();

  for (auto cp_world : *(clearing_clouds.ptr_v_cloud_)) {
    double scaling_fact = 1.0;
    // scaling_fact = std::max(
    //     std::min(scaling_fact, (distance - 2 * resolution_) / distance),
    //     0.0);
    cp_world.d_x =
        scaling_fact * (cp_world.d_x - cp_origin.d_x) + cp_origin.d_x;
    cp_world.d_y =
        scaling_fact * (cp_world.d_y - cp_origin.d_y) + cp_origin.d_y;
    cp_world.d_z =
        scaling_fact * (cp_world.d_z - cp_origin.d_z) + cp_origin.d_z;

    double a = cp_world.d_x - cp_origin.d_x;
    double b = cp_world.d_y - cp_origin.d_y;
    double c = cp_world.d_z - cp_origin.d_z;
    double t = 1.0;

    // we can only raytrace to a maximum z height
    if (cp_world.d_z > d_max_obstacle_height_) {
      // we know we want the vector's z value to be max_z
      t = std::max(
          0.0,
          std::min(t, (d_max_obstacle_height_ - 0.01 - cp_origin.d_z) / c));
    } else if (cp_world.d_z <
               d_origin_z_) {  // and we can only raytrace down to the floor
      // we know we want the vector's z value to be 0.0
      t = std::min(t, (d_origin_z_ - cp_origin.d_z) / c);
    }

    // the minimum value to raytrace from is the origin
    if (cp_world.d_x < d_origin_x_) {
      t = std::min(t, (d_origin_x_ - cp_origin.d_x) / a);
    }
    if (cp_world.d_y < d_origin_y_) {
      t = std::min(t, (d_origin_y_ - cp_origin.d_y) / b);
    }

    // the maximum value to raytrace to is the end of the map
    if (cp_world.d_x > map_end_x) {
      t = std::min(t, (map_end_x - cp_origin.d_x) / a);
    }
    if (cp_world.d_y > map_end_y) {
      t = std::min(t, (map_end_y - cp_origin.d_y) / b);
    }

    cp_world.d_x = cp_origin.d_x + a * t;
    cp_world.d_y = cp_origin.d_y + b * t;
    cp_world.d_z = cp_origin.d_z + c * t;

    CostmapPointXYZ cp_point;
    if (worldToMap3DFloat(cp_world, cp_point)) {
      unsigned int cell_raytrace_range = cellDistance(d_raytrace_range_);
      // voxel_grid_.markVoxelLine(sensor_x, sensor_y, sensor_z, point_x,
      // point_y, point_z);
      voxel_grid_.clearVoxelLineInMap(
          cp_sensor, cp_point, ptr_uc_costmap_.get(), unknown_threshold_,
          mark_threshold_, FREE_SPACE, NO_INFORMATION, cell_raytrace_range);

      updateRaytraceBounds({cp_origin.d_x, cp_origin.d_y},
                           {cp_world.d_x, cp_world.d_y}, d_raytrace_range_,
                           costmap_bound);

      if (b_publish_clearing_points_) {
        CostmapPointXYZ point;
        point.d_x = cp_world.d_x;
        point.d_y = cp_world.d_y;
        point.d_z = cp_world.d_z;
        v_clearing_endpoints_.push_back(point);
      }
    }
  }
}

void VoxelLayer::updateOrigin(const double &d_new_origin_x,
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
  unsigned int *local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int *voxel_map = voxel_grid_.getData();
  boost::shared_array<timeType> local_obstacle_time =
      boost::shared_array<timeType>(new timeType[cell_size_x * cell_size_y]);

  copyMapRegion(ptr_uc_costmap_, lower_left_x, lower_left_y, ui_size_x_,
                local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
  copyMapRegion(ptr_d_obstacle_time_, lower_left_x, lower_left_y, ui_size_x_,
                local_obstacle_time, 0, 0, cell_size_x, cell_size_x,
                cell_size_y);
  copyMapRegionRaw(voxel_map, lower_left_x, lower_left_y, ui_size_x_,
                   local_voxel_map, 0, 0, cell_size_x, cell_size_x,
                   cell_size_y);

  resetMaps();

  d_origin_x_ = new_grid_ox;
  d_origin_y_ = new_grid_oy;

  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  copyMapRegion(local_map, 0, 0, cell_size_x, ptr_uc_costmap_, start_x, start_y,
                ui_size_x_, cell_size_x, cell_size_y);
  copyMapRegion(local_obstacle_time, 0, 0, cell_size_x, ptr_d_obstacle_time_,
                start_x, start_y, ui_size_x_, cell_size_x, cell_size_y);
  copyMapRegionRaw(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x,
                   start_y, ui_size_x_, cell_size_x, cell_size_y);

  delete[] local_voxel_map;
}

//预留接口
void VoxelLayer::activate() { b_enabled_ = true; }
void VoxelLayer::deactivate() { b_enabled_ = false; }
void VoxelLayer::reset() {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  resetMaps();
}

}  // namespace CVTE_BABOT
