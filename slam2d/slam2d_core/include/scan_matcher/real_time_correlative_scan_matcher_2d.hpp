#ifndef _REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
#define _REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_

#include "Eigen/Core"
#include <iostream>
#include <memory>
#include <vector>

// #include "common/grid_map.hpp"
#include "common/probability_grid.hpp"
#include "scan_matcher/correlative_scan_matcher_2d.hpp"

namespace slam2d_core {
namespace scan_matcher {

class RealTimeCorrelativeScanMatcherOptions {
 public:
  double linear_search_window = 0.1;
  double angular_search_window = M_PI / 9.0;
  double translation_delta_cost_weight = 10;
  double rotation_delta_cost_weight = 1e-1;
};

//搜索框内暴力匹配
class RealTimeCorrelativeScanMatcher2D {
 public:
  explicit RealTimeCorrelativeScanMatcher2D(
      const RealTimeCorrelativeScanMatcherOptions &options);

  RealTimeCorrelativeScanMatcher2D(const RealTimeCorrelativeScanMatcher2D &) =
      delete;
  RealTimeCorrelativeScanMatcher2D &operator=(
      const RealTimeCorrelativeScanMatcher2D &) = delete;

  double match(const common::Rigid2 &initial_pose_estimate,
               const common::PointCloud &point_cloud,
               const common::ProbabilityGrid &grid,
               common::Rigid2 *pose_estimate) const;

  void scoreCandidates(const common::ProbabilityGrid &grid,
                       const std::vector<DiscreteScan2D> &discrete_scans,
                       const SearchParameters &search_parameters,
                       std::vector<Candidate2D> *candidates) const;

 private:
  std::vector<Candidate2D> generateExhaustiveSearchCandidates(
      const SearchParameters &search_parameters) const;

  const RealTimeCorrelativeScanMatcherOptions options_;
};

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif
