#ifndef _VOXEL_FILTER_H_
#define _VOXEL_FILTER_H_

#include "common/point_cloud.hpp"
#include <bitset>
#include <vector>

namespace slam2d_core {
namespace frontend {

class AdaptiveVoxelFilterOptions {
 public:
  float max_length = 1.0;      // 一个栅格的最大分辨率
  float min_num_points = 100;  // 一帧激光雷达最少点数
  float max_range = 30;        // 激光雷达最大距离
};

common::PointCloud voxelFilter(const common::PointCloud &point_cloud,
                               const float resolution);

common::PointCloud adaptiveVoxelFilter(
    const common::PointCloud &point_cloud,
    const AdaptiveVoxelFilterOptions &options);
}  // namespace frontend
}  // namespace slam2d_core

#endif
