/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file voxel_grid.hpp
 *
 *@brief costmap voxel grid.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.1.3
 *@data 2019-08-22
 ************************************************************************/
#ifndef __VOXEL_GRID_HPP
#define __VOXEL_GRID_HPP

#include <assert.h>
#include <glog/logging.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <costmap_utils.hpp>
namespace CVTE_BABOT {

enum VoxelStatus {
  FREE = 0,
  UNKNOWN = 1,
  MARKED = 2,
};

/**
 * @class VoxelGrid
 * @brief 将3D点存储在整型数组的结构，通过x和y在数组中索引，
 *        z指定该索引对应的整型的哪个位，不同的位代表的不同
 *        高度，限制了垂直方向只能有16个cells，第一位跟第十
 *        七位标记了一个高度，第二位跟第十八位标记了一个高度
 *        以此类推
 */
class VoxelGrid {
 public:
  /**
   * @brief  构造函数
   * @param[in] ui_size_x 体素的x方向大小
   * @param[in] ui_size_y 体素的x方向大小
   * @param[in] ui_size_z 体素的x方向大小，只支持小于等于16！
   */
  VoxelGrid(const unsigned int& ui_size_x, const unsigned int& ui_size_y,
            const unsigned int& ui_size_z);

  ~VoxelGrid();

  VoxelGrid(const VoxelGrid& obj) = delete;
  VoxelGrid& operator=(const VoxelGrid& obj) = delete;

  /**
   * resize
   * @brief  调整体素的大小
   * @param[in] vp_size-体素的x,y,z大小
   */
  void resize(const VoxelPoint& vp_size);

  /**
   * reset
   * @brief 将所有grid置为UNKNOWN
   */
  void reset();

  /**
   * getData
   * @brief 获取存储voxel_grid的数据指针
   * return 存储voxel_grid的数据指针
   */
  uint32_t* getData() { return p_ui_data_; }

  /**
   * markVoxel
   * @brief
   * 标记体素
   *
   * @param[in] vp-voxel的三维坐标
   * */
  inline void markVoxel(const VoxelPoint& vp) {
    if (vp.ui_x >= ui_size_x_ || vp.ui_y >= ui_size_y_ ||
        vp.ui_z >= ui_size_z_) {
      LOG(ERROR) << "Error, voxel out of bounds.";
      return;
    }
    uint32_t full_mask = ((uint32_t)1 << vp.ui_z << 16) | (1 << vp.ui_z);
    p_ui_data_[vp.ui_y * ui_size_x_ + vp.ui_x] |=
        full_mask;  // 位或操作，清除unknown,标记为占用
  }

  /**
   * markVoxelInMap
   * @brief
   * 标记坐标相应体素
   *
   * @param[in] vp-voxel的三维坐标
   * @param[in] marked_threshold-标记位的阙值
   * retrun true-该坐标对应z中marked的位数大于marked_threshold，false-不大于
   * */
  inline bool markVoxelInMap(const VoxelPoint& vp,
                             const unsigned int& marked_threshold) {
    if (vp.ui_x >= ui_size_x_ || vp.ui_y >= ui_size_y_ ||
        vp.ui_z >= ui_size_z_) {
      LOG(ERROR) << "Error, voxel out of bounds.";
      return false;
    }

    int index = vp.ui_y * ui_size_x_ + vp.ui_x;
    uint32_t* col = &p_ui_data_[index];
    uint32_t full_mask = ((uint32_t)1 << vp.ui_z << 16) | (1 << vp.ui_z);
    *col |= full_mask;  // 位或操作，清除unknown,标记为占用

    unsigned int marked_bits = *col >> 16;

    // z中marked的位数是否大于marked_threshold
    return !bitsBelowThreshold(marked_bits, marked_threshold);
  }

  /**
   * markVoxelLine
   * @brief
   * 标记两点间的体素
   *
   * @param[in] cp0-起始点的x,y,z坐标
   * @param[in] cp1-目标点的x,y,z坐标
   * @param[in] max_length-标记的最大距离
   * */
  void markVoxelLine(const CostmapPointXYZ& cp0, const CostmapPointXYZ& cp1,
                     const unsigned int& max_length = UINT_MAX);

  /**
   * clearVoxel
   * @brief
   * 清除坐标相应体素
   *
   * @param[in] vp-voxel的三维坐标
   * @param[in] marked_threshold-标记位的阙值
   * */
  inline void clearVoxel(const VoxelPoint& vp) {
    if (vp.ui_x >= ui_size_x_ || vp.ui_y >= ui_size_y_ ||
        vp.ui_z >= ui_size_z_) {
      LOG(ERROR) << "Error, voxel out of bounds.";

      return;
    }
    uint32_t full_mask = ((uint32_t)1 << vp.ui_z << 16) | (1 << vp.ui_z);
    p_ui_data_[vp.ui_y * ui_size_x_ + vp.ui_x] &=
        ~(full_mask);  // 取反与操作，清除grid
  }

  /**
 * clearVoxelColumn
 * @brief
 * 清除index对应位置所有z上的点
 *
 * @param[in] index-由x，y转化得到的索引
 * */
  inline void clearVoxelColumn(unsigned int index) {
    assert(index < ui_size_x_ * ui_size_y_);
    p_ui_data_[index] = 0;
  }

  /**
   * clearVoxelInMap
   * @brief
   * 清除index对应位置所有z上的点
   *
   * @param[in] index-由x，y转化得到的索引
  * */
  inline void clearVoxelInMap(const VoxelPoint& vp) {
    if (vp.ui_x >= ui_size_x_ || vp.ui_y >= ui_size_y_ ||
        vp.ui_z >= ui_size_z_) {
      LOG(ERROR) << "Error, voxel out of bounds.";

      return;
    }
    int index = vp.ui_y * ui_size_x_ + vp.ui_x;
    uint32_t* col = &p_ui_data_[index];
    uint32_t full_mask = ((uint32_t)1 << vp.ui_z << 16) | (1 << vp.ui_z);
    *col &= ~(full_mask);  // clear unknown and clear cell

    unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
    unsigned int marked_bits = *col >> 16;

    // 没有unknown_bits或marked_bits时标记costmap为FREE
    if (bitsBelowThreshold(unknown_bits, 1) &&
        bitsBelowThreshold(marked_bits, 1)) {
      p_uc_costmap[index] = 0;
    }
  }

  /**
   * clearVoxelLine
   * @brief
   * 清除两点间的体素
   *
   * @param[in] cp0-起始点的x,y,z坐标
   * @param[in] cp1-目标点的x,y,z坐标
   * @param[in] max_length-清除的最大距离
   * */
  void clearVoxelLine(const CostmapPointXYZ& cp0, const CostmapPointXYZ& cp1,
                      const unsigned int& max_length = UINT_MAX);

  /**
   * clearVoxelLineInMap
   * @brief
   * 在costmap清除两点间的体素
   *
   * @param[in] cp0-起始点的x,y,z坐标
   * @param[in] cp1-目标点的x,y,z坐标
   * @param[in] map_2d-costmap存储的指针
   * @param[in] unknown_threshold-标记为unknown的阙值
   * @param[in] mark_threshold-标记为占用的阙值
   * @param[in] free_cost-free状态的cost值
   * @param[in] unknown_cost-unknown状态的cost值
   * @param[in] max_length-清除的最大距离
   * */
  void clearVoxelLineInMap(const CostmapPointXYZ& cp0,
                           const CostmapPointXYZ& cp1, unsigned char* map_2d,
                           const unsigned int& unknown_threshold,
                           const unsigned int& mark_threshold,
                           unsigned char free_cost = 0,
                           unsigned char unknown_cost = 255,
                           const unsigned int& max_length = UINT_MAX);

  /**
   * bitsBelowThreshold
   * @brief
   * 判断n的二进制中1的位数是否小于bit_threshold
   *
   * @param[in] n-要判断的数
   * @param[in] bit_threshold-阙值
   * @return true-是，false-否
   * */
  inline bool bitsBelowThreshold(unsigned int n,
                                 const unsigned int& bit_threshold) {
    unsigned int bit_count = 0;
    while (n != 0) {
      ++bit_count;
      if (bit_count > bit_threshold) {
        return false;
      }
      n &= n - 1;  // 清除最低位的1
    }
    return true;
  }

  /**
   * numBits
   * @brief
   * 计算有几位为1
   *
   * @param[in] n-要计算的数
   * @return 1的位数
   * */
  static inline unsigned int numBits(unsigned int n) {
    unsigned int bit_count = 0;
    while (n != 0) {
      ++bit_count;
      n &= n - 1;  // 清除最低位的1
    }
    return bit_count;
  }

  /**
   * getVoxel
   * @brief
   * 获取点的占用状态
   *
   * @param[in] x，y，z-点的x,y,z坐标
   * @return 占用状态
   * */
  static VoxelStatus getVoxel(const VoxelPoint& vp, const VoxelPoint& vp_size,
                              const uint32_t* data) {
    if (vp.ui_x >= vp_size.ui_x || vp.ui_y >= vp_size.ui_y ||
        vp.ui_z >= vp_size.ui_z) {
      LOG(ERROR) << "Error, voxel out of bounds. "
                 << "(" << vp.ui_x << ", " << vp.ui_y << ", " << vp.ui_z
                 << " )";
      return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1 << vp.ui_z << 16) | (1 << vp.ui_z);
    uint32_t result = data[vp.ui_y * vp_size.ui_x + vp.ui_x] & full_mask;
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if (bits < 2) {
      if (bits < 1) {
        return FREE;
      }
      return UNKNOWN;
    }
    return MARKED;
  }

  /**
   * getVoxel
   * @brief
   * 获取点的占用状态
   *
   * @param[in] VoxelPoint-点的x,y,z坐标
   * @return 占用状态
   * */
  VoxelStatus getVoxel(const VoxelPoint& vp);

  /**
   * getVoxelColumn
   * @brief
   * 获取(x,y)点的占用状态
   *
   * @param[in] cp_point-点的x,y坐标
   * @param[in] unknown_threshold-标记为unknown的阙值
   * @param[in] mark_threshold-标记为占用的阙值
   * @return 占用状态
   * */
  VoxelStatus getVoxelColumn(const CostmapPoint& cp_point,
                             unsigned int unknown_threshold = 0,
                             unsigned int marked_threshold = 0);

  /**
   * getVoxelBelowBit
   * @brief
   * 获取(x,y)点的bit位以下栅格占用状态
   *
   * @param[in] cp_point-点的x,y坐标
   * @param[in] bit-查询的bit位
   * @param[in] unknown_threshold-标记为unknown的阙值
   * @param[in] mark_threshold-标记为占用的阙值
   * @return 占用状态
   * */
  VoxelStatus getVoxelsBelowBit(const unsigned int& index,
                                const unsigned int& bit,
                                unsigned int unknown_threshold,
                                unsigned int marked_threshold);
  void printVoxelGrid();

  void printColumnGrid();

  /**
   * sizeX
   * @brief
   * 获取x方向的大小
   * @return x方向的大小
   * */
  inline unsigned int sizeX() { return ui_size_x_; }

  /**
   * sizeY
   * @brief
   * 获取y方向的大小
   * @return y方向的大小
   * */
  inline unsigned int sizeY() { return ui_size_y_; }

  /**
   * sizeZ
   * @brief
   * 获取z方向的大小
   * @return z方向的大小
   * */
  inline unsigned int sizeZ() { return ui_size_z_; }

  /**
   * raytraceLine
   * @brief
   * 核心函数，三维坐标上遍历两点射线经过的grid
   * 对经过的grid执行什么操作由ActionType决定
  * */
  template <class ActionType>
  inline void raytraceLine(ActionType at, double x0, double y0, double z0,
                           double x1, double y1, double z1,
                           unsigned int max_length = UINT_MAX) {
    // 选取方向
    int dx = static_cast<int>(x1) - static_cast<int>(x0);
    int dy = static_cast<int>(y1) - static_cast<int>(y0);
    int dz = static_cast<int>(z1) - static_cast<int>(z0);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * ui_size_x_;
    int offset_dz = sign(dz);

    unsigned int z_mask = ((1 << 16) | 1)
                          << (unsigned int)z0;  // 使得n与n+16相应的两个bit为1
    unsigned int offset = (unsigned int)y0 * ui_size_x_ + (unsigned int)x0;

    GridOffset grid_off(offset);
    ZOffset z_off(z_mask);

    // we need to chose how much to scale our dominant dimension, based on the
    // maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) +
                       (z0 - z1) * (z0 - z1));
    double scale = std::min(1.0, max_length / dist);

    // x is dominant
    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    unsigned int abs_dz = abs(dz);
    if (abs_dx >= max(abs_dy, abs_dz)) {
      int error_y = abs_dx / 2;
      int error_z = abs_dx / 2;

      bresenham3D(at, grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz,
                  error_y, error_z, offset_dx, offset_dy, offset_dz, offset,
                  z_mask, (unsigned int)(scale * abs_dx));
      return;
    } else if (abs_dy >= abs_dz) {
      // y is dominant
      int error_x = abs_dy / 2;
      int error_z = abs_dy / 2;

      bresenham3D(at, grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz,
                  error_x, error_z, offset_dy, offset_dx, offset_dz, offset,
                  z_mask, (unsigned int)(scale * abs_dy));

      return;
    } else {
      // otherwise, z is dominant
      int error_x = abs_dz / 2;
      int error_y = abs_dz / 2;

      bresenham3D(at, z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy,
                  error_x, error_y, offset_dz, offset_dx, offset_dy, offset,
                  z_mask, (unsigned int)(scale * abs_dz));
    }
  }

 private:
  /**
   * bresenham3D
   * @brief
   * 核心函数，三维坐标上遍历射线经过的grid
   * 对经过的grid执行什么操作由ActionType决定
  * */
  template <class ActionType, class OffA, class OffB, class OffC>
  inline void bresenham3D(ActionType at, OffA off_a, OffB off_b, OffC off_c,
                          unsigned int abs_da, unsigned int abs_db,
                          unsigned int abs_dc, int error_b, int error_c,
                          int offset_a, int offset_b, int offset_c,
                          unsigned int& offset, unsigned int& z_mask,
                          unsigned int max_length = UINT_MAX) {
    // 在off_a方向上进行step
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i) {
      at(offset, z_mask);
      off_a(offset_a);
      error_b += abs_db;
      error_c += abs_dc;
      if ((unsigned int)error_b >= abs_da) {
        off_b(offset_b);
        error_b -= abs_da;
      }
      if ((unsigned int)error_c >= abs_da) {
        off_c(offset_c);
        error_c -= abs_da;
      }
    }
    at(offset, z_mask);
  }

  /**
   * sign
   * @brief
   * 获取输入数的正负
   * @return 1-正，-1-负
  * */
  inline int sign(int i) { return i > 0 ? 1 : -1; }

  /**
   * max
   * @brief
   * 获取最大值
   * @return 两数中的最大值
  * */
  inline unsigned int max(unsigned int x, unsigned int y) {
    return x > y ? x : y;
  }

  /**
   * MarkVoxel
   * @brief
   * 标记目标体素
  * */
  class MarkVoxel {
   public:
    explicit MarkVoxel(uint32_t* data) : data_(data) {}
    inline void operator()(unsigned int offset, unsigned int z_mask) {
      data_[offset] |= z_mask;  // clear unknown and mark cell
    }

   private:
    uint32_t* data_;
  };

  /**
   * MarkVoxel
   * @brief
   * 清除目标体素
  * */
  class ClearVoxel {
   public:
    explicit ClearVoxel(uint32_t* data) : data_(data) {}
    inline void operator()(unsigned int offset, unsigned int z_mask) {
      data_[offset] &= ~(z_mask);  // clear unknown and clear cell
    }

   private:
    uint32_t* data_;
  };

  /**
   * ClearVoxelInMap
   * @brief
   * 清除目标体素
  * */
  class ClearVoxelInMap {
   public:
    ClearVoxelInMap(uint32_t* data, unsigned char* costmap,
                    unsigned int unknown_clear_threshold,
                    unsigned int marked_clear_threshold,
                    unsigned char free_cost = 0,
                    unsigned char unknown_cost = 255)
        : data_(data),
          costmap_(costmap),
          unknown_clear_threshold_(unknown_clear_threshold),
          marked_clear_threshold_(marked_clear_threshold),
          free_cost_(free_cost),
          unknown_cost_(unknown_cost) {}

    inline void operator()(unsigned int offset, unsigned int z_mask) {
      uint32_t* col = &data_[offset];
      *col &= ~(z_mask);  // clear unknown and clear cell

      unsigned int unknown_bits =
          uint16_t(*col >> 16) ^ uint16_t(*col);  // 异或操作，取11的位
      unsigned int marked_bits = *col >> 16;

      // LOG(ERROR) << offset << " ma " << numBits(marked_bits);
      // LOG(ERROR) << offset << " un " << numBits(unknown_bits);
      // make sure the number of bits in each is below our thesholds
      if (bitsBelowThreshold(marked_bits, marked_clear_threshold_)) {
        if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold_)) {
          costmap_[offset] = free_cost_;
          // LOG(ERROR) << offset << " free ";
        } else {
          costmap_[offset] = unknown_cost_;
        }
      }
    }

   private:
    /**
    * bitsBelowThreshold
    * @brief
    * n的二进制中为1的位数不大于bit_threshold
    *
    * @param[in] n-要测试的数
    * @param[in] bit_threshold-1的位数
    * @return true-不大于bit_threshold，false-大于bit_threshold
    * */
    inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold) {
      unsigned int bit_count;
      for (bit_count = 0; n;) {
        ++bit_count;
        if (bit_count > bit_threshold) {
          return false;
        }
        // 将n的二进制表示中的最低位为1的改为0
        n &= n - 1;  // clear the least significant bit set
      }
      return true;
    }

    uint32_t* data_;
    unsigned char* costmap_;
    unsigned int unknown_clear_threshold_, marked_clear_threshold_;
    unsigned char free_cost_, unknown_cost_;
  };

  /**
   * GridOffset
   * @brief
   * 类似迭代器，执行grid的坐标偏移操作
  * */
  class GridOffset {
   public:
    GridOffset(unsigned int& offset) : offset_(offset) {}
    inline void operator()(int offset_val) { offset_ += offset_val; }

   private:
    unsigned int& offset_;
  };

  /**
   * ZOffset
   * @brief
   * 类似迭代器，执行z的坐标偏移操作
  * */
  class ZOffset {
   public:
    ZOffset(unsigned int& z_mask) : z_mask_(z_mask) {}
    inline void operator()(int offset_val) {
      offset_val > 0 ? z_mask_ <<= 1 : z_mask_ >>= 1;
    }

   private:
    unsigned int& z_mask_;
  };

  unsigned int ui_size_x_, ui_size_y_, ui_size_z_;  ///< 体素的x,y,z方向的大小
  unsigned char* p_uc_costmap;                      ///< 存储costmap的数据

  uint32_t* p_ui_data_;  ///< 存储体素的数据
};

}  // namespace CVTE_BABOT

#endif  // __VOXEL_GRID_HPP