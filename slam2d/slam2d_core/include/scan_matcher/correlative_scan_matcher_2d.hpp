#ifndef _CORRELATIVE_SCAN_MATCHER_2D_H_
#define _CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "Eigen/Core"
#include "common/map_limits.hpp"
#include "common/point_cloud.hpp"

namespace slam2d_core {
namespace scan_matcher {

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

struct SearchParameters {
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const common::PointCloud &point_cloud, double resolution);

  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // 根据目前的数据缩小搜索框
  void shrinkToFit(const std::vector<DiscreteScan2D> &scans,
                   const common::CellLimits &cell_limits);

  int num_angular_perturbations;          // 角度搜索空间的个数
  double angular_perturbation_step_size;  //角度搜索步长
  double resolution;                      //地图分辨率
  int num_scans;
  std::vector<LinearBounds> linear_bounds;
};

std::vector<common::PointCloud> generateRotatedScans(
    const common::PointCloud &point_cloud,
    const SearchParameters &search_parameters);

std::vector<DiscreteScan2D> discretizeScans(
    const common::MapLimits &map_limits,
    const std::vector<common::PointCloud> &scans,
    const Eigen::Translation2d &initial_translation);

// A possible solution.
struct Candidate2D {
  Candidate2D(const int init_scan_index, const int init_x_index_offset,
              const int init_y_index_offset,
              const SearchParameters &search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate2D relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate2D &other) const { return score < other.score; }
  bool operator>(const Candidate2D &other) const { return score > other.score; }
};

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif
