/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file voxel_grid_filter.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-08-12
 ************************************************************************/
#include "voxel_grid_filter.hpp"

namespace CVTE_BABOT {
VoxelGridFilter::VoxelGridFilter(const double &size_x, const double &size_y,
                                 const double &size_z)
    : d_size_x_(size_x),
      d_size_y_(size_y),
      d_size_z_(size_z),
      ptr_cloud_(new pcl::PointCloud<VPoint>) {}

void VoxelGridFilter::setInputCloud(const VPointsPtr &ptr_input_cloud) {
  *ptr_cloud_ = *ptr_input_cloud;
}

void VoxelGridFilter::setLeafSize(const double &res_x, const double &res_y,
                                  const double &res_z) {
  if (double_equal(res_x, d_res_x_) && double_equal(res_y, d_res_y_) &&
      double_equal(res_z, d_res_z_)) {
    return;
  }

  d_res_x_ = res_x;
  d_res_y_ = res_y;
  d_res_z_ = res_z;
  grid_pts_.clear();

  int grid_size_x = 2 * d_size_x_ / d_res_x_;
  int grid_size_y = 2 * d_size_y_ / d_res_y_;
  int grid_size_z = 2 * d_size_z_ / d_res_z_;
  vp_grid.reserve(grid_size_x * grid_size_y * grid_size_z);

  grid_pts_.resize(grid_size_x);
  for (int x = 0; x < grid_size_x; ++x) {
    grid_pts_[x].resize(grid_size_y);
    for (int y = 0; y < grid_size_y; ++y) {
      grid_pts_[x][y].resize(grid_size_z);
    }
  }
}

void VoxelGridFilter::filter(pcl::PointCloud<VPoint> &output_cloud) {
  output_cloud.points.clear();

  for (auto &grid : vp_grid) { memset(grid, 0, sizeof(VoxelGrid)); }

  // memset(&grid_pts_[0], 0,
  //        grid_size_x * grid_size_y * grid_size_z * sizeof(voxel_grid));

  vp_grid.clear();

  for (const auto &point : ptr_cloud_->points) {
    int grid_x = (d_size_x_ - point.x) / d_res_x_;
    int grid_y = (d_size_y_ - point.y) / d_res_y_;
    int grid_z = (d_size_z_ - point.z) / d_res_z_;

    VoxelGrid *grid = &grid_pts_[grid_x][grid_y][grid_z];

    // 先前没有被填充点
    if (grid->ui_count == 0) {
      vp_grid.push_back(grid);
    }

    grid->f_sum_x_ += point.x;
    grid->f_sum_y_ += point.y;
    grid->f_sum_z_ += point.z;
    grid->f_sum_i_ += point.intensity;
    grid->ui_count++;
  }

  // // calculate the grid height
  for (const auto &grid : vp_grid) {
    VPoint point;
    point.x = grid->f_sum_x_ / grid->ui_count;
    point.y = grid->f_sum_y_ / grid->ui_count;
    point.z = grid->f_sum_z_ / grid->ui_count;
    point.intensity = grid->f_sum_i_ / grid->ui_count;
    output_cloud.points.emplace_back(point);
  }
}

}  // namespace CVTE_BABOT
