#include "scan_matcher/real_time_correlative_scan_matcher_2d.hpp"
#include "common/probability_grid.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "glog/logging.h"

namespace slam2d_core {
namespace scan_matcher {
namespace {

float computeCandidateScore(const common::ProbabilityGrid &probability_grid,
                            const DiscreteScan2D &discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  for (const Eigen::Array2i &xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    // 计算累积占据概率
    const float probability =
        probability_grid.getProbability(proposed_xy_index);
    candidate_score += probability;
  }
  // 计算平均概率
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const RealTimeCorrelativeScanMatcherOptions &options)
    : options_(options) {}

std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::generateExhaustiveSearchCandidates(
    const SearchParameters &search_parameters) const {
  int num_candidates = 0;
  // 遍历所有的点云，计算候选的点数总量
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    // 获取x方向offset数量
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    // 获取y方向offset数量
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    // x*y为offset总量
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  // 遍历所有的点云
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    // 生成candidates，一个offset（一个新的位姿）对应一个点云，所以有点云数量*x方向offset数量*y方向offset数量个可能解
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

double RealTimeCorrelativeScanMatcher2D::match(
    const common::Rigid2 &initial_pose_estimate,
    const common::PointCloud &point_cloud, const common::ProbabilityGrid &grid,
    common::Rigid2 *pose_estimate) const {
  CHECK(pose_estimate != nullptr);

  // 点云旋转
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const common::PointCloud rotated_point_cloud = common::transformPointCloud(
      point_cloud,
      common::Rigid3::Rotation(Eigen::AngleAxisd(
          initial_rotation.cast<double>().angle(), Eigen::Vector3d::UnitZ())));

  //构建搜索参数
  const SearchParameters search_parameters(
      options_.linear_search_window, options_.angular_search_window,
      rotated_point_cloud, grid.limits().resolution());
  // 根据搜索框参数生成多个旋转后的点云
  const std::vector<common::PointCloud> rotated_scans =
      generateRotatedScans(rotated_point_cloud, search_parameters);

  // 将每个点云加上平移后投影到网格中
  const std::vector<DiscreteScan2D> discrete_scans = discretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2d(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));

  // 根据搜索框，生成candidates，即为候选解
  std::vector<Candidate2D> candidates =
      generateExhaustiveSearchCandidates(search_parameters);
  // 对candidates打分
  scoreCandidates(grid, discrete_scans, search_parameters, &candidates);

  //找到最大得分后续点
  const Candidate2D &best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = common::Rigid2(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

// 运用暴力匹配的方法
void RealTimeCorrelativeScanMatcher2D::scoreCandidates(
    const common::ProbabilityGrid &grid,
    const std::vector<DiscreteScan2D> &discrete_scans,
    const SearchParameters &search_parameters,
    std::vector<Candidate2D> *const candidates) const {
  (void) search_parameters;
  // 遍历所有的candidates可能解
  for (Candidate2D &candidate : *candidates) {
    candidate.score = computeCandidateScore(
        static_cast<const common::ProbabilityGrid &>(grid),
        discrete_scans[candidate.scan_index], candidate.x_index_offset,
        candidate.y_index_offset);

    // 最终的score=平均score*exp^(-x^2):    x=距离*权重+角度*权重
    candidate.score *= std::exp(-common::Pow2(
        std::hypot(candidate.x, candidate.y) *
            options_.translation_delta_cost_weight +
        std::abs(candidate.orientation) * options_.rotation_delta_cost_weight));
  }
}

}  // namespace scan_matcher
}  // namespace slam2d_core
