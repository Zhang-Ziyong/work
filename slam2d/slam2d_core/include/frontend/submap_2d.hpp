#ifndef _SUBMAP_2D_H_
#define _SUBMAP_2D_H_

#include <memory>
#include <vector>
#include <mutex>

#include "Eigen/Core"

// #include "common/grid_map.hpp"
#include "common/point_cloud.hpp"
#include "common/probability_grid.hpp"
#include "common/probability_values.hpp"
#include "common/probability_grid_range_data_inserter.hpp"
#include "common/rigid_transform.hpp"
#include "common/value_conversion_tables.hpp"

#include "proto/submap.pb.h"

namespace slam2d_core {
namespace frontend {
class Submap2D {
 public:
  Submap2D(const Eigen::Vector2d &origin,
           std::unique_ptr<common::ProbabilityGrid> grid,
           common::ValueConversionTables *conversion_tables);

  explicit Submap2D(const slam2d::mapping::proto::Submap &proto,
                    common::ValueConversionTables *conversion_tables);

  const common::ProbabilityGrid *grid() const { return grid_.get(); }

  void insertRangeData(
      const common::RangeData &range_data,
      const common::ProbabilityGridRangeDataInserter *range_data_inserter);
  void finish();

  common::Rigid3 local_pose() const { return local_pose_; }

  int num_range_data() const { return num_range_data_; }
  void set_num_range_data(const int num_range_data) {
    num_range_data_ = num_range_data;
  }

  bool insertion_finished() const { return insertion_finished_; }
  void set_insertion_finished(bool insertion_finished) {
    insertion_finished_ = insertion_finished;
  }

  void toSubmapTexture(const common::Rigid3 &global_submap_pose,
                       common::SubmapTexture *const texture) const;

  slam2d::mapping::proto::Submap toProto(bool include_grid_data) const;

 private:
  std::unique_ptr<common::ProbabilityGrid> grid_;
  common::ValueConversionTables *conversion_tables_;

  mutable std::mutex submap_mutex_;

  const common::Rigid3 local_pose_;
  int num_range_data_ = 0;
  bool insertion_finished_ = false;
};

class SubmapsOptions {
 public:
  int num_range_data;
  float resolution;
  common::ProbabilityGridRangeDataInserterOptions
      probability_grid_range_data_inserter_options;
};

class ActiveSubmaps2D {
 public:
  explicit ActiveSubmaps2D(const SubmapsOptions &options);
  ActiveSubmaps2D(const ActiveSubmaps2D &) = delete;
  ActiveSubmaps2D &operator=(const ActiveSubmaps2D &) = delete;

  std::vector<std::shared_ptr<const Submap2D>> insertRangeData(
      const common::RangeData &range_data);

  std::vector<std::shared_ptr<const Submap2D>> submaps() const;

 private:
  // std::unique_ptr<RangeDataInserterInterface> CreateRangeDataInserter();
  std::unique_ptr<common::ProbabilityGrid> createGrid(
      const Eigen::Vector2d &origin);
  void finishSubmap();
  void addSubmap(const Eigen::Vector2d &origin);

  const SubmapsOptions options_;
  std::vector<std::shared_ptr<Submap2D>> submaps_;
  std::unique_ptr<common::ProbabilityGridRangeDataInserter>
      range_data_inserter_;
  common::ValueConversionTables conversion_tables_;
};

}  // namespace frontend
}  // namespace slam2d_core

#endif
