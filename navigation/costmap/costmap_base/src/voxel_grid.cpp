/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file voxel_grid.hpp
 *
 *@brief costmap obstacle layer.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.1.3
 *@data 2019-08-22
 ************************************************************************/
#include "voxel_grid.hpp"
#include <sys/time.h>

namespace CVTE_BABOT {
VoxelGrid::VoxelGrid(const unsigned int& ui_size_x,
                     const unsigned int& ui_size_y,
                     const unsigned int& ui_size_z)
    : ui_size_x_(ui_size_x), ui_size_y_(ui_size_y), ui_size_z_(ui_size_z) {
  if (ui_size_z_ > 16) {
    LOG(ERROR)
        << "Error, this implementation can only support up to 16 z values ("
        << ui_size_z_ << ")";
    ui_size_z_ = 16;
  }

  p_ui_data_ = new uint32_t[ui_size_x_ * ui_size_y_];
  uint32_t unknown_col = ~((uint32_t)0) >> 16;
  uint32_t* col = p_ui_data_;
  // 全部置为unknown
  for (unsigned int i = 0; i < ui_size_x_ * ui_size_y_; ++i) {
    *col = unknown_col;
    ++col;
  }
}

void VoxelGrid::resize(const VoxelPoint& vp_size) {
  // if we're not actually changing the size, we can just reset things
  if (vp_size.ui_x == ui_size_x_ && vp_size.ui_y == ui_size_y_ &&
      vp_size.ui_z == ui_size_z_) {
    reset();
    return;
  }

  delete[] p_ui_data_;
  ui_size_x_ = vp_size.ui_x;
  ui_size_y_ = vp_size.ui_y;
  ui_size_z_ = vp_size.ui_z;

  if (ui_size_z_ > 16) {
    LOG(INFO)
        << "Error, this implementation can only support up to 16 z values ("
        << ui_size_z_ << ")";
    ui_size_z_ = 16;
  }

  p_ui_data_ = new uint32_t[ui_size_x_ * ui_size_y_];
  uint32_t unknown_col = ~((uint32_t)0) >> 16;
  uint32_t* col = p_ui_data_;
  for (unsigned int i = 0; i < ui_size_x_ * ui_size_y_; ++i) {
    *col = unknown_col;
    ++col;
  }
}

VoxelGrid::~VoxelGrid() { delete[] p_ui_data_; }

void VoxelGrid::reset() {
  uint32_t unknown_col = ~((uint32_t)0) >> 16;  // 低16位置1, 高16位为0
  uint32_t* col = p_ui_data_;
  for (unsigned int i = 0; i < ui_size_x_ * ui_size_y_; ++i) {
    *col = unknown_col;
    ++col;
  }
}

void VoxelGrid::markVoxelLine(const CostmapPointXYZ& cp0,
                              const CostmapPointXYZ& cp1,
                              const unsigned int& max_length) {
  if (cp0.d_x >= ui_size_x_ || cp0.d_y >= ui_size_y_ || cp0.d_z >= ui_size_z_ ||
      cp1.d_x >= ui_size_x_ || cp1.d_y >= ui_size_y_ || cp1.d_z >= ui_size_z_) {
    LOG(WARNING) << "Error, line endpoint out of bounds. "
                 << "(" << cp0.d_x << ", " << cp0.d_y << ", " << cp0.d_z
                 << " ) to (" << cp1.d_x << ", " << cp1.d_y << ", " << cp1.d_z
                 << " ), size: (" << ui_size_x_ << ", " << ui_size_y_ << ", "
                 << ui_size_z_ << " )";
    return;
  }

  MarkVoxel mv(p_ui_data_);
  raytraceLine(mv, cp0.d_x, cp0.d_y, cp0.d_z, cp1.d_x, cp1.d_y, cp1.d_z,
               max_length);
}

void VoxelGrid::clearVoxelLine(const CostmapPointXYZ& cp0,
                               const CostmapPointXYZ& cp1,
                               const unsigned int& max_length) {
  if (cp0.d_x >= ui_size_x_ || cp0.d_y >= ui_size_y_ || cp0.d_z >= ui_size_z_ ||
      cp1.d_x >= ui_size_x_ || cp1.d_y >= ui_size_y_ || cp1.d_z >= ui_size_z_) {
    LOG(WARNING) << "Error, line endpoint out of bounds. "
                 << "(" << cp0.d_x << ", " << cp0.d_y << ", " << cp0.d_z
                 << " ) to (" << cp1.d_x << ", " << cp1.d_y << ", " << cp1.d_z
                 << " ), size: (" << ui_size_x_ << ", " << ui_size_y_ << ", "
                 << ui_size_z_ << " )";
    return;
  }

  ClearVoxel cv(p_ui_data_);
  raytraceLine(cv, cp0.d_x, cp0.d_y, cp0.d_z, cp1.d_x, cp1.d_y, cp1.d_z,
               max_length);
}

void VoxelGrid::clearVoxelLineInMap(
    const CostmapPointXYZ& cp0, const CostmapPointXYZ& cp1,
    unsigned char* map_2d, const unsigned int& unknown_threshold,
    const unsigned int& mark_threshold, unsigned char free_cost,
    unsigned char unknown_cost, const unsigned int& max_length) {
  p_uc_costmap = map_2d;
  if (map_2d == NULL) {
    clearVoxelLine(cp0, cp1, max_length);
    return;
  }

  if (cp0.d_x >= ui_size_x_ || cp0.d_y >= ui_size_y_ || cp0.d_z >= ui_size_z_ ||
      cp1.d_x >= ui_size_x_ || cp1.d_y >= ui_size_y_ || cp1.d_z >= ui_size_z_) {
    LOG(WARNING) << "Error, line endpoint out of bounds. "
                 << "(" << cp0.d_x << ", " << cp0.d_y << ", " << cp0.d_z
                 << " ) to (" << cp1.d_x << ", " << cp1.d_y << ", " << cp1.d_z
                 << " ), size: (" << ui_size_x_ << ", " << ui_size_y_ << ", "
                 << ui_size_z_ << " )";
    return;
  }

  ClearVoxelInMap cvm(p_ui_data_, p_uc_costmap, unknown_threshold,
                      mark_threshold, free_cost, unknown_cost);
  raytraceLine(cvm, cp0.d_x, cp0.d_y, cp0.d_z, cp1.d_x, cp1.d_y, cp1.d_z,
               max_length);
}

VoxelStatus VoxelGrid::getVoxel(const VoxelPoint& vp) {
  if (vp.ui_x >= ui_size_x_ || vp.ui_y >= ui_size_y_ || vp.ui_z >= ui_size_z_) {
    LOG(ERROR) << "Error, voxel out of bounds. "
               << "(" << vp.ui_x << ", " << vp.ui_y << " )";
    return UNKNOWN;
  }
  uint32_t full_mask = ((uint32_t)1 << vp.ui_z << 16) | (1 << vp.ui_z);
  uint32_t result = p_ui_data_[vp.ui_y * ui_size_x_ + vp.ui_x] & full_mask;
  unsigned int bits = numBits(result);

  // known marked: 10 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
  if (bits < 2) {
    if (bits < 1) return FREE;

    return UNKNOWN;
  }

  return MARKED;
}

VoxelStatus VoxelGrid::getVoxelColumn(const CostmapPoint& cp_point,
                                      unsigned int unknown_threshold,
                                      unsigned int marked_threshold) {
  if (cp_point.ui_x >= ui_size_x_ || cp_point.ui_y >= ui_size_y_) {
    LOG(ERROR) << "Error, voxel out of bounds. "
               << "(" << cp_point.ui_x << ", " << cp_point.ui_y << " )";
    return UNKNOWN;
  }

  uint32_t* col = &p_ui_data_[cp_point.ui_y * ui_size_x_ + cp_point.ui_x];

  unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
  unsigned int marked_bits = *col >> 16;

  // check if the number of marked bits qualifies the col as marked
  if (!bitsBelowThreshold(marked_bits, marked_threshold)) {
    return MARKED;
  }

  // check if the number of unkown bits qualifies the col as unknown
  if (!bitsBelowThreshold(unknown_bits, unknown_threshold)) return UNKNOWN;

  return FREE;
}

VoxelStatus VoxelGrid::getVoxelsBelowBit(const unsigned int& index,
                                        const unsigned int& bit,
                                        unsigned int unknown_threshold,
                                        unsigned int marked_threshold) {
  if (index >= ui_size_x_ * ui_size_y_) {
    LOG(ERROR) << "Error, voxel out of bounds. index: " << index;
    return UNKNOWN;
  }

  uint32_t* col = &p_ui_data_[index];

  uint16_t unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
  unknown_bits <<= bit;
  uint16_t marked_bits = *col >> 16;
  marked_bits <<= bit;

  // check if the number of marked bits qualifies the col as marked
  if (!bitsBelowThreshold(marked_bits, marked_threshold)) {
    return MARKED;
  }

  // check if the number of unkown bits qualifies the col as unknown
  if (!bitsBelowThreshold(unknown_bits, unknown_threshold)) return UNKNOWN;

  return FREE;
}

void VoxelGrid::printVoxelGrid() {
  for (unsigned int z = 0; z < ui_size_z_; z++) {
    printf("Layer z = %u:\n", z);
    for (unsigned int y = 0; y < ui_size_y_; y++) {
      for (unsigned int x = 0; x < ui_size_x_; x++) {
        printf((getVoxel({x, y, z})) == CVTE_BABOT::MARKED ? "#" : " ");
      }
      printf("|\n");
    }
  }
}

void VoxelGrid::printColumnGrid() {
  printf("Column view:\n");
  for (unsigned int y = 0; y < ui_size_y_; y++) {
    for (unsigned int x = 0; x < ui_size_x_; x++) {
      printf((getVoxelColumn({x, y}, 16, 0) == CVTE_BABOT::MARKED) ? "#" : " ");
    }
    printf("|\n");
  }
}
}  // namespace CVTE_BABOT
