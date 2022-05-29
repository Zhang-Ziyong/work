#include "scan_matcher/occupied_space_cost_function_2d.hpp"

#include "ceres/cubic_interpolation.h"
#include "common/probability_values.hpp"

namespace slam2d_core {
namespace scan_matcher {
namespace {
class OccupiedSpaceCostFunction2D {
 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const common::PointCloud &point_cloud,
                              const common::ProbabilityGrid &grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(
        adapter);  // 双三次插值，构造二阶线性插值类
    const common::MapLimits &limits = grid_.limits();

    // 针对每个点，计算该点匹配残差：1-Smooth(Tp)
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));
      // Tp：将点转换到全局坐标系下
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      // 计算1-Smooth(Tp)
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  // 注意，该kPadding是为了解决有些点可能跑到地图外面去的情况，所以加了一个超大的值，即将地图上下左右边界分别扩大kPadding
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const common::ProbabilityGrid &grid)
        : grid_(grid) {}

    void GetValue(const int row, const int column, double *const value) const {
      // 跑到了地图外面，在扩大的地方，直接赋值最大cost
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = common::kMaxCorrespondenceCost;
      } else {
        // 在地图里，直接取概率值，在这里需要减掉kPadding，因为在传进来的时候，已经加了kPadding
        *value = static_cast<double>(grid_.getCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const common::ProbabilityGrid &grid_;
  };

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D &) = delete;
  OccupiedSpaceCostFunction2D &operator=(const OccupiedSpaceCostFunction2D &) =
      delete;

  const double scaling_factor_;
  const common::PointCloud &point_cloud_;
  const common::ProbabilityGrid &grid_;
};

}  // namespace

ceres::CostFunction *createOccupiedSpaceCostFunction2D(
    const double scaling_factor, const common::PointCloud &point_cloud,
    const common::ProbabilityGrid &grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
      point_cloud.size());
}

}  // namespace scan_matcher
}  // namespace slam2d_core
