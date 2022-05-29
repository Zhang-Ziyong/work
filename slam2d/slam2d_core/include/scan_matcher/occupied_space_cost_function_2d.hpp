#ifndef _OCCUPIED_SPACE_COST_FUNCTION_2D_H_
#define _OCCUPIED_SPACE_COST_FUNCTION_2D_H_

#include "ceres/ceres.h"
// #include "common/grid_map.hpp"
#include "common/probability_grid.hpp"
#include "common/point_cloud.hpp"

namespace slam2d_core {
namespace scan_matcher {

ceres::CostFunction *createOccupiedSpaceCostFunction2D(
    const double scaling_factor, const common::PointCloud &point_cloud,
    const common::ProbabilityGrid &grid);

}  // namespace scan_matcher
}  // namespace slam2d_core

#endif  //
