#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "frontend/submap_2d.hpp"
#include "glog/logging.h"

namespace slam2d_core {
namespace frontend {

Submap2D::Submap2D(const Eigen::Vector2d &origin,
                   std::unique_ptr<common::ProbabilityGrid> grid,
                   common::ValueConversionTables *conversion_tables)
    : conversion_tables_(conversion_tables),
      local_pose_(common::Rigid3::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.0))) {
  grid_ = std::move(grid);
}

Submap2D::Submap2D(const slam2d::mapping::proto::Submap &proto,
                   common::ValueConversionTables *conversion_tables)
    : conversion_tables_(conversion_tables),
      local_pose_(common::toRigid3(proto.local_pose())) {
  if (proto.has_grid()) {
    grid_ = std::make_unique<common::ProbabilityGrid>(proto.grid(),
                                                      conversion_tables_);
  }
  set_num_range_data(proto.num_range_data());
  // set_insertion_finished(proto.finished());
  set_insertion_finished(true);
}

void Submap2D::insertRangeData(
    const common::RangeData &range_data,
    const common::ProbabilityGridRangeDataInserter *range_data_inserter) {
  std::lock_guard<std::mutex> guard(submap_mutex_);

  CHECK(grid_);
  CHECK(!insertion_finished());

  range_data_inserter->insert(range_data, grid_.get());

  set_num_range_data(num_range_data() + 1);
}

void Submap2D::toSubmapTexture(const common::Rigid3 &global_submap_pose,
                               common::SubmapTexture *const texture) const {
  std::lock_guard<std::mutex> guard(submap_mutex_);

  (void) global_submap_pose;
  if (!grid_) {
    LOG(WARNING) << "grid_ is nullptr.";
    return;
  }

  texture->submap_version = num_range_data();
  grid()->drawToSubmapTexture(texture, local_pose());
}

void Submap2D::finish() {
  std::lock_guard<std::mutex> guard(submap_mutex_);

  CHECK(grid_);
  CHECK(!insertion_finished());
  grid_ = grid_->computeCroppedGrid();
  set_insertion_finished(true);
}

slam2d::mapping::proto::Submap Submap2D::toProto(bool include_grid_data) const {
  slam2d::mapping::proto::Submap proto;
  *proto.mutable_local_pose() = common::toProto(local_pose());
  proto.set_num_range_data(num_range_data());
  proto.set_finished(insertion_finished());

  if (include_grid_data) {
    CHECK(grid_);
    *proto.mutable_grid() = grid_->toProto();
  }
  return proto;
}

ActiveSubmaps2D::ActiveSubmaps2D(const SubmapsOptions &options)
    : options_(options) {
  range_data_inserter_ =
      std::make_unique<common::ProbabilityGridRangeDataInserter>(
          options_.probability_grid_range_data_inserter_options);
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::insertRangeData(
    const common::RangeData &range_data) {
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data) {
    // LOG(INFO) << "add submap origin: " << range_data.origin[0] << ", "
    //           << range_data.origin[1];
    addSubmap(range_data.origin.head<2>());
  }

  for (auto &submap : submaps_) {
    submap->insertRangeData(range_data, range_data_inserter_.get());
  }

  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data) {
    submaps_.front()->finish();
  }
  return submaps();
}

std::unique_ptr<common::ProbabilityGrid> ActiveSubmaps2D::createGrid(
    const Eigen::Vector2d &origin) {
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.resolution;

  return std::make_unique<common::ProbabilityGrid>(
      common::MapLimits(
          resolution,
          origin.cast<double>() +
              0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),
          common::CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
      common::kMinCorrespondenceCost, common::kMaxCorrespondenceCost,
      &conversion_tables_);
}

void ActiveSubmaps2D::addSubmap(const Eigen::Vector2d &origin) {
  if (submaps_.size() >= 2) {
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }
  submaps_.push_back(std::make_unique<Submap2D>(
      origin,
      std::unique_ptr<common::ProbabilityGrid>(
          static_cast<common::ProbabilityGrid *>(createGrid(origin).release())),
      &conversion_tables_));
}

}  // namespace frontend
}  // namespace slam2d_core
