#ifndef _GLOBAL_SCAN_MATCHER_2D_H_
#define _GLOBAL_SCAN_MATCHER_2D_H_

#include "amcl/occupancy_grid.hpp"
#include "common/map_limits.hpp"
#include "common/point_cloud.hpp"

namespace slam2d_core {
namespace scan_matcher {

struct GlobalScanMatcher2DOptions {
  bool do_global_search = true;                 //是否进行全局搜索匹配
  bool do_window_search = true;                 //是否进行窗口搜索匹配
  int num_map_depth = 6;                        //地图层数
  float min_score = 0.6;                        //匹配得分阈值
  float search_angle_resolution = M_PI / 90.0;  //角度分辨率
  float search_windows_x_width = 10.0;          //搜索窗口x方向大小
  float search_windows_y_width = 10.0;          //搜索窗口y方向大小
  float search_windows_angle = 180.0;           //搜索角度的缩放
};

class PrecomputeOccupancyGrid2D {
 public:
  PrecomputeOccupancyGrid2D(
      const std::shared_ptr<slam2d_core::amcl::OccupancyGrid> occ_grid_ptr,
      const common::CellLimits &limits, int width,
      std::vector<float> *reusable_intermediate_grid);

  int getValue(const Eigen::Array2i &xy_index) const {
    const Eigen::Array2i local_xy_index = xy_index - offset_;
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.num_x_cells) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.num_y_cells)) {
      return 0;
    }
    const int stride = wide_limits_.num_x_cells;
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
  }

  int getOriginValue(const Eigen::Array2i &xy_index) const {
    if (static_cast<unsigned>(xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.num_x_cells) ||
        static_cast<unsigned>(xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.num_y_cells)) {
      return 0;
    }
    const int stride = wide_limits_.num_x_cells;
    return cells_[xy_index.x() + xy_index.y() * stride];
  }

  common::CellLimits getCellLimits() const { return wide_limits_; };

  float toScore(float value) const {
    return min_score_ + value * ((max_score_ - min_score_) / 255.f);
  }

 private:
  uint8_t computeCellValue(float probability) const;

  float cellStateToProbability(slam2d_core::amcl::Cell cell);

  const Eigen::Array2i offset_;
  const common::CellLimits wide_limits_;

  const float min_score_;
  const float max_score_;

  // Probabilites mapped to 0 to 255.
  std::vector<uint8_t> cells_;
};

typedef std::vector<Eigen::Array2i> DiscreteScan2D;

struct GlobalSearchParameters {
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  void shrinkToFit(const std::vector<DiscreteScan2D> &scans,
                   const int &num_x_cells, const int &num_y_cells) {
    for (int i = 0; i != num_scans; ++i) {
      Eigen::Array2i min_bound = Eigen::Array2i::Zero();
      Eigen::Array2i max_bound = Eigen::Array2i::Zero();
      for (const Eigen::Array2i &xy_index : scans[i]) {
        min_bound = min_bound.min(-xy_index);
        max_bound = max_bound.max(
            Eigen::Array2i(num_x_cells - 1, num_y_cells - 1) - xy_index);
      }
      linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
      linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
      linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
      linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
    }
  }
  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;
};

// A possible solution.
struct GlobalCandidate2D {
  GlobalCandidate2D(const int init_scan_index, const int init_x_index_offset,
                    const int init_y_index_offset,
                    const GlobalSearchParameters &search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(x_index_offset * search_parameters.resolution),
        y(y_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}
  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this GlobalCandidate2D relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const GlobalCandidate2D &other) const {
    return score < other.score;
  }
  bool operator>(const GlobalCandidate2D &other) const {
    return score > other.score;
  }
};

//整个全局定位的实现过程分为以下几部
// 1、生成多层的分辨率地图
// 2、根据地图范围和确定搜索框（确定每一个分辨率地图和旋转角下动态确定下来）
// 3、根据搜索框确定所有的candicates
// 4、确定得分函数
// 5、分枝定界递归回调
// 6、结果处理及返回
class GlobalScanMatcher2D {
 public:
  explicit GlobalScanMatcher2D(const GlobalScanMatcher2DOptions &options);
  ~GlobalScanMatcher2D();

  GlobalScanMatcher2D(const GlobalScanMatcher2D &) = delete;
  GlobalScanMatcher2D &operator=(const GlobalScanMatcher2D &) = delete;

  void setMap(const std::shared_ptr<slam2d_core::amcl::OccupancyGrid>
                  ptr_occupancy_map);
  //在整张地图上面进行搜索
  bool scanMatchInFullMap(const slam2d_core::common::PointCloud &pointcloud,
                          common::Rigid2 *pose_estimate);
  //在初始位置附近窗口搜索
  bool scanMatchNearPose(const slam2d_core::common::PointCloud &pointcloud,
                         common::Rigid2 *pose_estimate,
                         const common::Rigid2 &init_pose);

  inline const std::vector<PrecomputeOccupancyGrid2D> &getMultiresolutionMap() {
    return precomputation_Occupancy_grids_;
  };

 private:
  bool matchWithSearchParameters(GlobalSearchParameters search_parameters,
                                 const common::Rigid2 &initial_pose_estimate,
                                 const common::PointCloud &point_cloud,
                                 common::Rigid2 *pose_estimate);

  GlobalCandidate2D branchAndBound(
      const std::vector<DiscreteScan2D> &discrete_scans,
      const GlobalSearchParameters &search_parameters,
      const std::vector<GlobalCandidate2D> &candidates,
      const int candidate_depth, float min_score) const;

  void scoreCandidates(const PrecomputeOccupancyGrid2D &precomputation_grid,
                       const std::vector<DiscreteScan2D> &discrete_scans,
                       std::vector<GlobalCandidate2D> *const candidates) const;

  GlobalScanMatcher2DOptions options_;
  std::shared_ptr<slam2d_core::amcl::OccupancyGrid> ptr_occupancy_map_ =
      nullptr;
  std::vector<PrecomputeOccupancyGrid2D> precomputation_Occupancy_grids_;
};

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif