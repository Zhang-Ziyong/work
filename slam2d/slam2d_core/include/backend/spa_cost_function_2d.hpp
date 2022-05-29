#ifndef _SPA_COST_FUNCTION_2D_H_
#define _SPA_COST_FUNCTION_2D_H_

#include "ceres/ceres.h"
#include "cost_helpers.hpp"
#include "pose_graph.hpp"

namespace slam2d_core {
namespace backend {

ceres::CostFunction *createAutoDiffSpaCostFunction(
    const Constraint::Pose &pose);

ceres::CostFunction *createAnalyticalSpaCostFunction(
    const Constraint::Pose &pose);

}  // namespace backend
}  // namespace slam2d_core

#endif
