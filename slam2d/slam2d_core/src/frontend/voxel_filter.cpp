#include "frontend/voxel_filter.hpp"

#include <cmath>
#include <random>
#include <unordered_map>
#include <utility>

namespace slam2d_core {
namespace frontend {
namespace {
// 过滤掉大于最大距离的点
common::PointCloud filterByMaxRange(const common::PointCloud &point_cloud,
                                    const float max_range) {
  return point_cloud.copy_if([max_range](const common::RangePoint &point) {
    return point.position.norm() <= max_range;
  });
}

common::PointCloud adaptivelyVoxelFiltered(
    const AdaptiveVoxelFilterOptions &options,
    const common::PointCloud &point_cloud) {
  if (point_cloud.size() <= options.min_num_points) {
    return point_cloud;  // 点数少于最少点数时直接返回
  }
  //按照最大栅格分辨率稀疏点云
  common::PointCloud result = voxelFilter(point_cloud, options.max_length);
  if (result.size() >= options.min_num_points) {
    return result;  // 稀疏后的点数大于最小点数时直接返回
  }

  //对最大分辨率使用二分法，直到满足稀疏后的点数大于最小点数
  for (float high_length = options.max_length;
       high_length > 1e-2f * options.max_length; high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = voxelFilter(point_cloud, low_length);
    if (result.size() >= options.min_num_points) {
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const common::PointCloud candidate =
            voxelFilter(point_cloud, mid_length);
        if (candidate.size() >= options.min_num_points) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

using VoxelKeyType = uint64_t;

VoxelKeyType getVoxelCellIndex(const Eigen::Vector3d &point,
                               const float resolution) {
  const Eigen::Array3d index = point.array() / resolution;
  const uint64_t x = common::roundToInt(index.x());
  const uint64_t y = common::roundToInt(index.y());
  const uint64_t z = common::roundToInt(index.z());
  return (x << 42) + (y << 21) + z;
}

template <class T, class PointFunction>
std::vector<bool> randomizedVoxelFilterIndices(
    const std::vector<T> &point_cloud, const float resolution,
    PointFunction &&point_function) {
  // According to https://en.wikipedia.org/wiki/Reservoir_sampling
  std::minstd_rand0 generator;
  std::unordered_map<VoxelKeyType, std::pair<int, int>>
      voxel_count_and_point_index;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto &voxel = voxel_count_and_point_index[getVoxelCellIndex(
        point_function(point_cloud[i]), resolution)];
    voxel.first++;  // 计数相同intex的点云数量
    if (voxel.first == 1) {
      voxel.second = i;  // 记录进入栅格的第一个点
    } else {             // 均匀随机采样算法
      std::uniform_int_distribution<> distribution(1, voxel.first);
      if (distribution(generator) == voxel.first) {
        voxel.second = i;  // 均匀随机采样进入栅格的n个点
      }
    }
  }
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto &voxel_and_index : voxel_count_and_point_index) {
    points_used[voxel_and_index.second.second] = true;  //记录采样到的点
  }
  return points_used;
}
}  // namespace

common::PointCloud voxelFilter(const common::PointCloud &point_cloud,
                               const float resolution) {
  const std::vector<bool> points_used = randomizedVoxelFilterIndices(
      point_cloud.points(), resolution,
      [](const common::RangePoint &point) { return point.position; });

  std::vector<common::RangePoint> filtered_points;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      filtered_points.push_back(point_cloud[i]);
    }
  }
  //返回采样后的点云
  return common::PointCloud(std::move(filtered_points));
}

common::PointCloud adaptiveVoxelFilter(
    const common::PointCloud &point_cloud,
    const AdaptiveVoxelFilterOptions &options) {
  return adaptivelyVoxelFiltered(
      options, filterByMaxRange(point_cloud, options.max_range));
}

}  // namespace frontend
}  // namespace slam2d_core
