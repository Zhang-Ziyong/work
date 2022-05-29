/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file voxel_grid_filter.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-08-12
 ************************************************************************/
#ifndef __VOXEL_GRID_FILTER_HPP
#define __VOXEL_GRID_FILTER_HPP
#include <vector>
#include "ground_utils.hpp"

namespace CVTE_BABOT {
class VoxelGridFilter {
 public:
  VoxelGridFilter(const double &size_x, const double &size_y,
                  const double &size_z);

  ~VoxelGridFilter() = default;

  VoxelGridFilter(const VoxelGridFilter &) = delete;
  VoxelGridFilter &operator=(const VoxelGridFilter &) = delete;

  void setInputCloud(const VPointsPtr &ptr_input_cloud);

  void setLeafSize(const double &res_x, const double &res_y,
                   const double &res_z);

  void filter(pcl::PointCloud<VPoint> &output_cloud);

 private:
  struct VoxelGrid {
    float f_sum_x_ = 0.00;
    float f_sum_y_ = 0.00;
    float f_sum_z_ = 0.00;
    float f_sum_i_ = 0.00;
    unsigned int ui_count = 0;
  };

  inline bool double_equal(const double &value1, const double &value2) {
    return fabs(value1 - value2) < 0.0001;
  }

  double d_size_x_ = 0.00;
  double d_size_y_ = 0.00;
  double d_size_z_ = 0.00;

  double d_res_x_ = 0.00;
  double d_res_y_ = 0.00;
  double d_res_z_ = 0.00;

  VPointsPtr ptr_cloud_ = nullptr;
  std::vector<std::vector<std::vector<VoxelGrid>>> grid_pts_;
  std::vector<VoxelGrid *> vp_grid;
};
}  // namespace CVTE_BABOT
#endif
