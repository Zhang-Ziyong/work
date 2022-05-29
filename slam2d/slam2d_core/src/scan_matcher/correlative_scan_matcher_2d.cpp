#include "scan_matcher/correlative_scan_matcher_2d.hpp"

#include "common/math.hpp"
#include "glog/logging.h"
#include <cmath>

namespace slam2d_core {
namespace scan_matcher {

SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const common::PointCloud &point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  float max_scan_range = 3.f * resolution;
  //计算当前点云数据的最大范围，最远点的距离
  for (const common::RangePoint &point : point_cloud) {
    const float range = point.position.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }

  //计算角度搜索步长,根据最远点距离和分辨率计算
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                         (2. * common::Pow2(max_scan_range)));
  //计算角度搜索空间的个数
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  num_scans = 2 * num_angular_perturbations + 1;

  //计算 xy方向的搜索个数
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);

  //确定每个点云的最大最小的边界
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

// 尽量收缩搜索框，减少计算量
// scans即：被角度搜索框内部的一系列角度旋转后并初始化平移的一系列点云的容器
// 记住，现在点云已经位于map坐标系了，不在以节点自身为参考
// 这个是非常重要的一个信息
void SearchParameters::shrinkToFit(const std::vector<DiscreteScan2D> &scans,
                                   const common::CellLimits &cell_limits) {
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);
  // 遍历生成的旋转后的很多scan
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();
    // 对每一帧scan进行遍历，确定每一帧的最大最小的坐标索引
    for (const Eigen::Array2i &xy_index : scans[i]) {
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }
    // 每一帧scan的最大最小的坐标索引
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

//生成按照不同角度旋转后的点云
std::vector<common::PointCloud> generateRotatedScans(
    const common::PointCloud &point_cloud,
    const SearchParameters &search_parameters) {
  std::vector<common::PointCloud> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);

  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;

  // 遍历360度
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    // 将 point_cloud 绕Z轴旋转 delta_theta
    rotated_scans.push_back(common::transformPointCloud(
        point_cloud, common::Rigid3::Rotation(Eigen::AngleAxisd(
                         delta_theta, Eigen::Vector3d::UnitZ()))));
  }
  return rotated_scans;
}

std::vector<DiscreteScan2D> discretizeScans(
    const common::MapLimits &map_limits,
    const std::vector<common::PointCloud> &scans,
    const Eigen::Translation2d &initial_translation) {
  std::vector<DiscreteScan2D> discrete_scans;
  // discrete_scans的size 为 旋转的点云的个数
  discrete_scans.reserve(scans.size());
  for (const common::PointCloud &scan : scans) {
    // discrete_scans中的每一个 DiscreteScan2D 的size 设置为
    // 当前这一帧scan的个数
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    for (const common::RangePoint &point : scan) {
      // 对scan中的每个点进行坐标变换
      const Eigen::Vector2d translated_point =
          Eigen::Affine2d(initial_translation) * point.position.head<2>();
      // 将旋转后的对应的栅格的索引放入discrete_scans
      discrete_scans.back().push_back(
          map_limits.getCellIndex(translated_point));
    }
  }
  return discrete_scans;
}

}  // namespace scan_matcher
}  // namespace slam2d_core
