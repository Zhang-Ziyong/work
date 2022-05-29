
#ifndef _RANGE_DATA_INSERTER_PROBABILITY_GRID_H_
#define _RANGE_DATA_INSERTER_PROBABILITY_GRID_H_

#include <utility>
#include <vector>

#include "point_cloud.hpp"
#include "probability_grid.hpp"
#include "ray_to_pixel_mask.hpp"

namespace slam2d_core {
namespace common {

class ProbabilityGridRangeDataInserterOptions {
 public:
  double hit_probability;
  double miss_probability;
  bool insert_free_space;
};

class ProbabilityGridRangeDataInserter {
 public:
  explicit ProbabilityGridRangeDataInserter(
      const ProbabilityGridRangeDataInserterOptions &options);

  ProbabilityGridRangeDataInserter(const ProbabilityGridRangeDataInserter &) =
      delete;
  ProbabilityGridRangeDataInserter &operator=(
      const ProbabilityGridRangeDataInserter &) = delete;

  void insert(const common::RangeData &range_data, ProbabilityGrid *grid) const;

 private:
  ProbabilityGridRangeDataInserterOptions options_;
  const std::vector<uint16_t> hit_table_;
  const std::vector<uint16_t> miss_table_;
};

}  // namespace common
}  // namespace slam2d_core

#endif
