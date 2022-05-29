#ifndef _CERES_SOLVER_OPTIONS_H_
#define _CERES_SOLVER_OPTIONS_H_

#include "ceres/ceres.h"

namespace slam2d_core {
namespace common {

class CeresSolverOptions {
 public:
  bool use_nonmonotonic_steps = true;
  int32_t max_num_iterations = 10;
  int32_t num_threads = 1;
};

ceres::Solver::Options createCeresSolverOptions(const CeresSolverOptions &opts);

}  // namespace common
}  // namespace slam2d_core

#endif
