#include "common/probability_grid.hpp"
#include <limits>
#include <memory>

namespace slam2d_core {
namespace common {
namespace {

float minCorrespondenceCostFromProto(
    const slam2d::mapping::proto::GridMap &proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING) << "proto::GridMap: min_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMinCorrespondenceCost;
  } else {
    return proto.min_correspondence_cost();
  }
}

float maxCorrespondenceCostFromProto(
    const slam2d::mapping::proto::GridMap &proto) {
  if (proto.min_correspondence_cost() == 0.f &&
      proto.max_correspondence_cost() == 0.f) {
    LOG(WARNING) << "proto::GridMap: max_correspondence_cost "
                    "is initialized with 0 indicating an older version of the "
                    "protobuf format. Loading default values.";
    return kMaxCorrespondenceCost;
  } else {
    return proto.max_correspondence_cost();
  }
}

inline float logit(float probability) {
  return std::log(probability / (1.f - probability));
}

const float kMaxLogOdds = logit(kMaxProbability);
const float kMinLogOdds = logit(kMinProbability);

// 将概率[0.1,0.9]映射为0-255之间的数
inline uint8_t probabilityToLogOddsInteger(const float probability) {
  const int value = common::roundToInt((logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

}  // namespace

ProbabilityGrid::ProbabilityGrid(const MapLimits &limits,
                                 const float &min_correspondence_cost,
                                 const float &max_correspondence_cost,
                                 ValueConversionTables *conversion_tables)
    : conversion_tables_(conversion_tables),
      limits_(limits),
      min_correspondence_cost_(min_correspondence_cost),
      max_correspondence_cost_(max_correspondence_cost),
      correspondence_cost_cells_(
          limits_.cell_limits().num_x_cells * limits_.cell_limits().num_y_cells,
          kUnknownCorrespondenceValue),
      value_to_correspondence_cost_table_(conversion_tables->getConversionTable(
          max_correspondence_cost, min_correspondence_cost,
          max_correspondence_cost)) {
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);
}

ProbabilityGrid::ProbabilityGrid(const slam2d::mapping::proto::GridMap &proto,
                                 ValueConversionTables *conversion_tables)
    : conversion_tables_(conversion_tables),
      limits_(proto.limits()),
      min_correspondence_cost_(minCorrespondenceCostFromProto(proto)),
      max_correspondence_cost_(maxCorrespondenceCostFromProto(proto)),
      correspondence_cost_cells_(),
      value_to_correspondence_cost_table_(conversion_tables->getConversionTable(
          max_correspondence_cost_, min_correspondence_cost_,
          max_correspondence_cost_)) {
  CHECK_LT(min_correspondence_cost_, max_correspondence_cost_);

  if (proto.has_known_cells_box()) {
    const auto &box = proto.known_cells_box();
    known_cells_box_ =
        Eigen::AlignedBox2i(Eigen::Vector2i(box.min_x(), box.min_y()),
                            Eigen::Vector2i(box.max_x(), box.max_y()));
  }

  correspondence_cost_cells_.reserve(proto.cells_size());
  for (const auto &cell : proto.cells()) {
    CHECK_LE(cell, std::numeric_limits<uint16_t>::max());
    correspondence_cost_cells_.push_back(cell);
  }
}

void ProbabilityGrid::setProbability(const Eigen::Array2i &cell_index,
                                     const float probability) {
  uint16_t &cell =
      (*mutable_correspondence_cost_cells())[toFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
      correspondenceCostToValue(probabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}

bool ProbabilityGrid::applyLookupTable(const Eigen::Array2i &cell_index,
                                       const std::vector<uint16_t> &table) {
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = toFlatIndex(cell_index);
  uint16_t *cell = &(*mutable_correspondence_cost_cells())[flat_index];
  if (*cell >= kUpdateMarker) {
    return false;
  }
  mutable_update_indices()->push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

float ProbabilityGrid::getProbability(const Eigen::Array2i &cell_index) const {
  if (!limits().contains(cell_index))
    return kMinProbability;

  int index = toFlatIndex(cell_index);
  int size = correspondence_cost_cells().size();
  if (index >= size || index < 0) {
    LOG(ERROR) << "getProbability. index is error. " << index << " >= || <=0 "
               << size;
    return kMinProbability;
  }
  uint16_t cell_cost = correspondence_cost_cells()[toFlatIndex(cell_index)];

  return correspondenceCostToProbability(valueToCorrespondenceCost(cell_cost));
}

std::unique_ptr<ProbabilityGrid> ProbabilityGrid::computeCroppedGrid() const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  computeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
      limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<ProbabilityGrid> cropped_grid =
      std::make_unique<ProbabilityGrid>(
          MapLimits(resolution, max, cell_limits), kMinCorrespondenceCost,
          kMaxCorrespondenceCost, conversion_tables_);
  for (const Eigen::Array2i &xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!isKnown(xy_index + offset))
      continue;
    cropped_grid->setProbability(xy_index, getProbability(xy_index + offset));
  }

  return std::unique_ptr<ProbabilityGrid>(cropped_grid.release());
}

bool ProbabilityGrid::drawToSubmapTexture(
    SubmapTexture *const texture, const common::Rigid3 &local_pose) const {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  computeCroppedLimits(&offset, &cell_limits);

  std::string cells;
  for (const Eigen::Array2i &xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!isKnown(xy_index + offset)) {
      cells.push_back(0 /* unknown log odds value */);
      cells.push_back(0 /* alpha */);
      continue;
    }

    const int delta =
        128 - probabilityToLogOddsInteger(getProbability(xy_index + offset));
    const uint8_t alpha = delta > 0 ? 0 : -delta;
    const uint8_t value = delta > 0 ? delta : 0;
    cells.push_back(value);
    cells.push_back((value || alpha) ? alpha : 1);
  }

  texture->width = cell_limits.num_x_cells;
  texture->height = cell_limits.num_y_cells;
  const double resolution = limits().resolution();
  texture->resolution = resolution;
  const double max_x = limits().max().x() - resolution * offset.y();
  const double max_y = limits().max().y() - resolution * offset.x();
  texture->slice_pose =
      local_pose.inverse() *
      common::Rigid3::Translation(Eigen::Vector3d(max_x, max_y, 0.));

  const int num_pixels = texture->width * texture->height;
  CHECK_EQ(cells.size(), 2 * num_pixels);
  texture->pixels.intensity.reserve(num_pixels);
  texture->pixels.alpha.reserve(num_pixels);
  for (int i = 0; i < texture->height; ++i) {
    for (int j = 0; j < texture->width; ++j) {
      texture->pixels.intensity.push_back(cells[(i * texture->width + j) * 2]);
      texture->pixels.alpha.push_back(cells[(i * texture->width + j) * 2 + 1]);
    }
  }

  return true;
}

/* slam2d::mapping::proto::GridMap ProbabilityGrid::toProto() const {
  slam2d::mapping::proto::GridMap result;
  result = GridMap::toProto();
  return result;
} */

void ProbabilityGrid::finishUpdate() {
  while (!update_indices_.empty()) {
    DCHECK_GE(correspondence_cost_cells_[update_indices_.back()],
              kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}

void ProbabilityGrid::computeCroppedLimits(Eigen::Array2i *const offset,
                                           CellLimits *const limits) const {
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();  // 返回边界最小的角点
  *limits = CellLimits(known_cells_box_.sizes().x() + 1,
                       known_cells_box_.sizes().y() + 1);
}

void ProbabilityGrid::growLimits(const Eigen::Vector2d &point) {
  growLimits(point, {mutable_correspondence_cost_cells()},
             {kUnknownCorrespondenceValue});
}

void ProbabilityGrid::growLimits(
    const Eigen::Vector2d &point,
    const std::vector<std::vector<uint16_t> *> &grids,
    const std::vector<uint16_t> &grids_unknown_cell_values) {
  CHECK(update_indices_.empty());
  //每次翻倍扩展
  while (!limits_.contains(limits_.getCellIndex(point))) {
    const int x_offset = limits_.cell_limits().num_x_cells / 2;
    const int y_offset = limits_.cell_limits().num_y_cells / 2;
    const MapLimits new_limits(
        limits_.resolution(),
        limits_.max() +
            limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
        CellLimits(2 * limits_.cell_limits().num_x_cells,
                   2 * limits_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
                         new_limits.cell_limits().num_y_cells;

    for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) {
      std::vector<uint16_t> new_cells(new_size,
                                      grids_unknown_cell_values[grid_index]);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              (*grids[grid_index])[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      *grids[grid_index] = new_cells;
    }
    limits_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

slam2d::mapping::proto::GridMap ProbabilityGrid::toProto() const {
  slam2d::mapping::proto::GridMap result;

  *result.mutable_limits() = common::toProto(limits_);
  *result.mutable_cells() = {correspondence_cost_cells_.begin(),
                             correspondence_cost_cells_.end()};
  CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                    "not supported. Finish the update first.";

  if (!known_cells_box_.isEmpty()) {
    auto *const box = result.mutable_known_cells_box();
    box->set_max_x(known_cells_box_.max().x());
    box->set_max_y(known_cells_box_.max().y());
    box->set_min_x(known_cells_box_.min().x());
    box->set_min_y(known_cells_box_.min().y());
  }
  result.set_min_correspondence_cost(min_correspondence_cost_);
  result.set_max_correspondence_cost(max_correspondence_cost_);
  return result;
}

}  // namespace common
}  // namespace slam2d_core
