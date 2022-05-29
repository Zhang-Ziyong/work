#ifndef _PROBABILITY_GRID_H_
#define _PROBABILITY_GRID_H_

#include <vector>

// #include "grid_map.hpp"
#include "map_limits.hpp"
#include "common/value_conversion_tables.hpp"
#include "common/probability_values.hpp"
#include "common/rigid_transform.hpp"

#include "proto/grid_map.pb.h"

namespace slam2d_core {
namespace common {

struct SubmapTexture {
  int submap_version;
  struct Pixels {
    std::vector<char> intensity;
    std::vector<char> alpha;
  };
  Pixels pixels;
  int width;
  int height;
  double resolution;
  common::Rigid3 slice_pose;
};

class ProbabilityGrid {
 public:
  explicit ProbabilityGrid(const MapLimits &limits,
                           const float &min_correspondence_cost,
                           const float &max_correspondence_cost,
                           ValueConversionTables *conversion_tables);

  explicit ProbabilityGrid(const slam2d::mapping::proto::GridMap &proto,
                           ValueConversionTables *conversion_tables);

  /**
   * limits
   * @brief 获取栅格地图的限制属性
   * @return MapLimits-栅格地图的限制属性
   **/
  const MapLimits &limits() const { return limits_; }

  void finishUpdate();

  /**
   * getCorrespondenceCost
   * @brief 获取某个栅格的代价
   * @param[in] cell_index-某栅格
   * @return float-代价
   **/
  float getCorrespondenceCost(const Eigen::Array2i &cell_index) const {
    if (!limits().contains(cell_index))
      return max_correspondence_cost_;
    return (*value_to_correspondence_cost_table_)
        [correspondence_cost_cells()[toFlatIndex(cell_index)]];
  }

  /**
   * getMinCorrespondenceCost
   * @brief 获取最小相关代价
   * @return float-代价
   **/
  float getMinCorrespondenceCost() const { return min_correspondence_cost_; }

  /**
   * getMaxCorrespondenceCost
   * @brief 获取最大相关代价
   * @return float-代价
   **/
  float getMaxCorrespondenceCost() const { return max_correspondence_cost_; }

  /**
   * isKnown
   * @brief 判断某栅格的代价是否知道
   * @return bool-true知道，false不知道
   **/
  bool isKnown(const Eigen::Array2i &cell_index) const {
    if (!limits_.contains(cell_index)) {
      LOG(ERROR) << "limits_.contains false.";
      return false;
    }

    int index = toFlatIndex(cell_index);
    if (correspondence_cost_cells_.empty()) {
      LOG(ERROR) << "correspondence_cost_cells_ is empty.";
      return false;
    }

    int size = correspondence_cost_cells_.size();

    if (index >= size || index < 0) {
      LOG(ERROR) << "index > size" << index << ", "
                 << static_cast<uint16_t>(index) << " >= || < 0" << size;
      return false;
    }

    if (correspondence_cost_cells_[index] != kUnknownCorrespondenceValue) {
      return true;
    }

    return false;
  }

  /**
   * computeCroppedLimits
   * @brief 计算裁剪后的栅格限制属性
   * @param[in] offset-偏移
   * @param[in] limits-裁剪后的栅格限制属性
   **/
  void computeCroppedLimits(Eigen::Array2i *const offset,
                            CellLimits *const limits) const;

  void setProbability(const Eigen::Array2i &cell_index,
                      const float probability);
  bool applyLookupTable(const Eigen::Array2i &cell_index,
                        const std::vector<uint16_t> &table);
  float getProbability(const Eigen::Array2i &cell_index) const;

  std::unique_ptr<ProbabilityGrid> computeCroppedGrid() const;

  bool drawToSubmapTexture(SubmapTexture *const texture,
                           const common::Rigid3 &local_pose) const;

  slam2d::mapping::proto::GridMap toProto() const;

  void growLimits(const Eigen::Vector2d &point);

 protected:
  /**
   * growLimits
   * @brief 扩展栅格限制属性，为了去包含某个点
   * @param[in] point-输出的texture
   * @param[in] grids-栅格的姿态
   * @param[in] grids_unknown_cell_values-栅格的姿态
   **/
  void growLimits(const Eigen::Vector2d &point,
                  const std::vector<std::vector<uint16_t> *> &grids,
                  const std::vector<uint16_t> &grids_unknown_cell_values);

  /**
   * correspondence_cost_cells
   * @brief 获取所有栅格的代价
   * @return correspondence_cost_cells_-所有栅格代价
   **/
  const std::vector<uint16_t> &correspondence_cost_cells() const {
    return correspondence_cost_cells_;
  }

  /**
   * update_indices
   * @brief 获取更新的的栅格
   * @return update_indices_-要更新栅格
   **/
  const std::vector<int> &update_indices() const { return update_indices_; }

  /**
   * known_cells_box
   * @brief 获取已知的栅格
   * @return known_cells_box_-已知的栅格
   **/
  const Eigen::AlignedBox2i &known_cells_box() const {
    return known_cells_box_;
  }

  /**
   * correspondence_cost_cells
   * @brief 获取所有栅格的代价
   * @return correspondence_cost_cells_-所有栅格代价
   **/
  std::vector<uint16_t> *mutable_correspondence_cost_cells() {
    return &correspondence_cost_cells_;
  }

  /**
   * update_indices
   * @brief 获取更新的的栅格
   * @return update_indices_-要更新栅格
   **/
  std::vector<int> *mutable_update_indices() { return &update_indices_; }

  /**
   * known_cells_box
   * @brief 获取已知的栅格
   * @return known_cells_box_-已知的栅格
   **/
  Eigen::AlignedBox2i *mutable_known_cells_box() { return &known_cells_box_; }

  /**
   * toFlatIndex
   * @brief 将栅格转换成vector的下标
   * @param[in] cell_index-栅格坐标
   * @return int-下标
   **/
  int toFlatIndex(const Eigen::Array2i &cell_index) const {
    CHECK(limits_.contains(cell_index)) << cell_index;
    return limits_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

 private:
  ValueConversionTables *conversion_tables_;

  MapLimits limits_;                 ///< 栅格地图相关限制属性
  float min_correspondence_cost_;    ///< 最小相关代价
  float max_correspondence_cost_;    ///< 最大相关代价
  std::vector<int> update_indices_;  ///< 更新下标

  Eigen::AlignedBox2i known_cells_box_;              ///< 已知的栅格
  std::vector<uint16_t> correspondence_cost_cells_;  ///< 相关栅格对应的代价
  const std::vector<float>
      *value_to_correspondence_cost_table_;  ///< uint_16对应的cost查询表
};

}  // namespace common
}  // namespace slam2d_core

#endif
