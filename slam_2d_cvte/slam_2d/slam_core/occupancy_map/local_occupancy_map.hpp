
#ifndef LOCAL_OCC_MAP_HPP
#define LOCAL_OCC_MAP_HPP

#include <mutex>
#include <atomic>
#include <glog/logging.h>
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"
#include "common/config/system_config.hpp"
#include "occupancy_map/occupancy_map_common.hpp"
#include "common/math_base/slam_transform.hpp"
#include "common/debug_tools/tic_toc.h"

namespace cvte_lidar_slam {
class MapManager;
class LocalOccMap {
 public:
  ~LocalOccMap() {
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
  }
  static std::shared_ptr<LocalOccMap> getInstance();

  void setConfig(const OccMapConfig &config) {
    occ_map_config_ = config;
    range_size_ = std::round(
        (occ_map_config_.laser_max_angle - occ_map_config_.laser_min_angle) /
        occ_map_config_.laser_angle_increment);
    laser_scan_.resize(range_size_, occ_map_config_.laser_max_range + 1.);
  }

  void clearTmpMap() {
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_keyFrame_index_ = 0;
    for (int i = 0; i < mapArrayLength; ++i) {
      for (int j = 0; j < mapArrayLength; ++j) { map_array_ind_[i][j] = -1; }
    }
  }

  void Reset() {
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_keyFrame_index_ = 0;
    for (int i = 0; i < mapArrayLength; ++i) {
      for (int j = 0; j < mapArrayLength; ++j) { map_array_ind_[i][j] = -1; }
    }
  }

  bool updateLocalMap();

 private:
  void allocateMemory();
  unsigned int getMapArraySize() { return map_array_.size(); }
  int getMapArrayIndX(const int index) { return map_array_[index].indX; }
  int getMapArrayIndY(const int index) { return map_array_[index].indY; }
  bool cloud2LaserScan(const laserCloud::Ptr cloud);
  double laserInvModel(const double &r, const double &R,
                       const double &cell_size);
  bool getPointGrid(const Vec2d &point, Grid &thisGrid);
  int8_t getPointOccupied(const PointType &point_in);
  MapCell *grid2MapCell(const Grid &grid);
  void updateChildMap(const Vec2d &point, const double laser_inv_value);
  void saveMapFile(const UserOccupancyGrid &map, const std::string &map_dir);
  /**
   *subMapExist
   *@brief
   *判读子图是否存在
   *
   *@param[in] cube_x-x坐标
   *@param[in] cube_y-y坐标
   **/
  bool subMapExist(const int cube_x, const int cube_y) {
    if (cube_x >= mapArrayLength || cube_y >= mapArrayLength) {
      return false;
    }
    if (map_array_ind_[cube_x][cube_y] == -1) {
      return false;
    } else {
      return true;
    }
  }
  int **map_array_ind_;  ///< it saves the index of this submap in vector
                         ///< mapArray

  // Map Arrays
  int map_array_count_;
  std::vector<ChildMap> map_array_;
  std::shared_ptr<MapManager> ptr_map_manager_ = nullptr;
  std::vector<float> laser_scan_;
  OccMapConfig occ_map_config_;
  u_int32_t range_size_;

  //模块数据
  UserOccupancyGrid local_occ_map_;
  Grid local_map_origin_grid_;
  PointType local_map_origin_point_;
  size_t last_keyFrame_index_;

 private:
  LocalOccMap();
  static std::shared_ptr<LocalOccMap> ptr_occ_map_;
};
}  // namespace cvte_lidar_slam

#endif
