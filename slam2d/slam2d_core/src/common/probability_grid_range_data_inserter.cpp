#include "common/probability_grid_range_data_inserter.hpp"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "common/probability_values.hpp"
#include "glog/logging.h"
#include <cstdlib>

namespace slam2d_core {
namespace common {
namespace {

constexpr int kSubpixelScale = 1;

void growAsNeeded(const common::RangeData &range_data,
                  ProbabilityGrid *const probability_grid) {
  Eigen::AlignedBox2d bounding_box(range_data.origin.head<2>());
  constexpr float kPadding = 1e-6f;
  for (const common::RangePoint &hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>());
  }
  for (const common::RangePoint &miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  probability_grid->growLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2d::Ones());
  probability_grid->growLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2d::Ones());
}

// 计算hit 栅格， 同时计算传感器到到hit栅格经过的 miss栅格
// input :insert_free_space配置项，默认为true，表示需要更新miss情况的概率

// 1.GrowAsNeeded实现当前grid
// map边界的扩展，即由于新的scan加入，可能会导致地图变大；
// 2.将地图分辨提高kSubpixelScale=1000倍，目的是为后面画直线精度更加精确；
// 3.获取高分辨率地图下的激光原点range_origin坐标索引;
// 4.获取高分率地图下所有有效激光点云的坐标索引；
// 5.获取还原原始地图分辨率坐标cell的value，然后查询hit_table表格进行更新，即z=hit条件下的更新；
// 6.采用RayToPixelMask画线的方法，获取激光原点到点云之间直线的所有点坐标；
// 7.通过还原原始地图分辨率获取value，查询miss_table表格进行更新，即z=miss条件下的更新；
void castRays(const common::RangeData &range_data,
              const std::vector<uint16_t> &hit_table,
              const std::vector<uint16_t> &miss_table,
              const bool insert_free_space, ProbabilityGrid *probability_grid) {
  // 根据 新的range 更新grid的边界大小
  growAsNeeded(range_data, probability_grid);
  // 获取边界
  const MapLimits &limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  // 重新定义地图边界
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  // 获取激光点云起点坐标
  const Eigen::Array2i begin =
      superscaled_limits.getCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  // 获取激光端点，即有效反射点，同时为hit点
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const common::RangePoint &hit : range_data.returns) {
    ends.push_back(superscaled_limits.getCellIndex(hit.position.head<2>()));
    // 针对每个hit端点进行更新栅格概率，通过hit_table表格查询,如当前为p
    // 则，新的p = hit_table[p]
    probability_grid->applyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  // 若无需更新miss栅格单元，可直接退出
  if (!insert_free_space) {
    return;
  }
  // Now add the misses.
  // origin 到 hit之间均为miss
  for (const Eigen::Array2i &end : ends) {
    // breshman 画线法获取两点间的数据，最后一个参数用于还原原分辨率
    std::vector<Eigen::Array2i> ray =
        rayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i &cell_index : ray) {
      // 对所有点进行miss 更新
      probability_grid->applyLookupTable(cell_index, miss_table);
    }
  }
  // 更新所有 range中miss的点， 则整条光速直线均为miss更新
  for (const common::RangePoint &missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = rayToPixelMask(
        begin, superscaled_limits.getCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i &cell_index : ray) {
      probability_grid->applyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

// 构造函数，计算出hit和miss占用栅格率更新表格
// 栅格更新可认为是，当前栅格的hit和miss概率值，
// 经过新的观测（即是否miss还是hit），进行更新 表格可认为是查表，即加快更新速度
// 注意：由于真正栅格内存储的数据均转换为0~32767的整数，可认为是概率对应值。同理概率更新后也同样为整数
// 表格查询方式： 当前概率为p， 若观测为hit，则新的p = hit_table_[p]
ProbabilityGridRangeDataInserter::ProbabilityGridRangeDataInserter(
    const ProbabilityGridRangeDataInserterOptions &options)
    : options_(options),
      hit_table_(computeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability))),
      miss_table_(computeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability))) {}

// submap插入新帧scan 刷新submap
// input : range_data,  grid
// output : grid
void ProbabilityGridRangeDataInserter::insert(
    const common::RangeData &range_data, ProbabilityGrid *const grid) const {
  // 强性转换为概率地图
  ProbabilityGrid *const probability_grid =
      static_cast<ProbabilityGrid *>(grid);

  CHECK(probability_grid != nullptr);

  // 采用画线法更新地图
  castRays(range_data, hit_table_, miss_table_, options_.insert_free_space,
           probability_grid);

  probability_grid->finishUpdate();
}

}  // namespace common
}  // namespace slam2d_core
