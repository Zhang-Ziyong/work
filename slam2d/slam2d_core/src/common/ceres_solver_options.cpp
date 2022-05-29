#include "common/ceres_solver_options.hpp"

namespace slam2d_core {
namespace common {

ceres::Solver::Options createCeresSolverOptions(
    const CeresSolverOptions &opts) {
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = opts.use_nonmonotonic_steps;
  options.max_num_iterations = opts.max_num_iterations;
  options.num_threads = opts.num_threads;
  return options;
}

}  // namespace common
}  // namespace slam2d_core
