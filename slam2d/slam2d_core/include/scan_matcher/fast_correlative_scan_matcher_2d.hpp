#ifndef _FAST_CORRELATIVE_SCAN_MATCHER_2D_H_
#define _FAST_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"

// #include "common/grid_map.hpp"
#include "common/probability_grid.hpp"
#include "common/map_limits.hpp"
#include "common/point_cloud.hpp"
#include "scan_matcher/correlative_scan_matcher_2d.hpp"

namespace slam2d_core {
namespace scan_matcher {

// 单个分辨率的网格地图
class PrecomputationGrid2D {
 public:
  // width是新的网格的
  // 参数grid为submap的原始网格数据，limits是原始网格数据中xy方向上的网格数量
  PrecomputationGrid2D(const common::ProbabilityGrid &grid,
                       const common::CellLimits &limits, int width,
                       std::vector<float> *reusable_intermediate_grid);

  // Returns a value between 0 and 255 to represent probabilities between
  // min_score and max_score.
  // 根据xy坐标，获取该(xy)坐标下width*width网格内的最大值
  int getValue(const Eigen::Array2i &xy_index) const {
    const Eigen::Array2i local_xy_index = xy_index - offset_;
    // The static_cast<unsigned> is for performance to check with 2 comparisons
    // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
    // local_xy_index.x() >= wide_limits_.num_x_cells ||
    // local_xy_index.y() >= wide_limits_.num_y_cells
    // instead of using 4 comparisons.
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.num_x_cells) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.num_y_cells)) {
      return 0;
    }
    const int stride = wide_limits_.num_x_cells;
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
  }

  // Maps values from [0, 255] to [min_score, max_score].
  float toScore(float value) const {
    return min_score_ + value * ((max_score_ - min_score_) / 255.f);
  }

 private:
  uint8_t computeCellValue(float probability) const;

  const Eigen::Array2i offset_;

  const common::CellLimits wide_limits_;

  const float min_score_;
  const float max_score_;

  // Probabilites mapped to 0 to 255.
  std::vector<uint8_t> cells_;
};

class FastCorrelativeScanMatcherOptions2D {
 public:
  double linear_search_window = 10.0;
  double angular_search_window = M_PI / 6.0;
  int32_t branch_and_bound_depth = 7;
};

// 管理分支定界所需的不同分辨率的网格
class PrecomputationGridStack2D {
 public:
  PrecomputationGridStack2D(const common::ProbabilityGrid &grid,
                            const FastCorrelativeScanMatcherOptions2D &options);

  const PrecomputationGrid2D &get(int index) {
    return precomputation_grids_[index];
  }

  int max_depth() const { return precomputation_grids_.size() - 1; }

 private:
  std::vector<PrecomputationGrid2D> precomputation_grids_;
};

// 分支定界的接口类
class FastCorrelativeScanMatcher2D {
 public:
  FastCorrelativeScanMatcher2D(
      const common::ProbabilityGrid &grid,
      const FastCorrelativeScanMatcherOptions2D &options);
  ~FastCorrelativeScanMatcher2D();

  FastCorrelativeScanMatcher2D(const FastCorrelativeScanMatcher2D &) = delete;
  FastCorrelativeScanMatcher2D &operator=(
      const FastCorrelativeScanMatcher2D &) = delete;

  // 有初始位姿的情况下的匹配
  bool match(const common::Rigid2 &initial_pose_estimate,
             const common::PointCloud &point_cloud, float min_score,
             float *score, common::Rigid2 *pose_estimate) const;

  // 有初始位姿的情况下的匹配
  bool match(const common::Rigid2 &initial_pose_estimate,
             const common::PointCloud &point_cloud,
             const double &linear_search_window, float min_score, float *score,
             common::Rigid2 *pose_estimate) const;

  //无初始位姿的接口全局匹配
  bool matchFullSubmap(const common::PointCloud &point_cloud, float min_score,
                       float *score, common::Rigid2 *pose_estimate) const;

 private:
  bool matchWithSearchParameters(SearchParameters search_parameters,
                                 const common::Rigid2 &initial_pose_estimate,
                                 const common::PointCloud &point_cloud,
                                 float min_score, float *score,
                                 common::Rigid2 *pose_estimate) const;
  // 计算低分辨率的可能解
  std::vector<Candidate2D> computeLowestResolutionCandidates(
      const std::vector<DiscreteScan2D> &discrete_scans,
      const SearchParameters &search_parameters) const;

  // 生成低分辨率可能解
  std::vector<Candidate2D> generateLowestResolutionCandidates(
      const SearchParameters &search_parameters) const;

  // 打分函数
  void scoreCandidates(const PrecomputationGrid2D &precomputation_grid,
                       const std::vector<DiscreteScan2D> &discrete_scans,
                       const SearchParameters &search_parameters,
                       std::vector<Candidate2D> *const candidates) const;

  // 分支定界
  Candidate2D branchAndBound(const std::vector<DiscreteScan2D> &discrete_scans,
                             const SearchParameters &search_parameters,
                             const std::vector<Candidate2D> &candidates,
                             int candidate_depth, float min_score) const;

  const FastCorrelativeScanMatcherOptions2D options_;
  common::MapLimits limits_;
  std::unique_ptr<PrecomputationGridStack2D> precomputation_grid_stack_;
};

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif
