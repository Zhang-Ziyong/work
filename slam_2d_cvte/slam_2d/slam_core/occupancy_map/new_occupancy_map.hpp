/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: /src/cvte_lidar_slam/slam_core/occupancy_map/new_occupancy_map.hpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-07-22 20:14:18
 ************************************************************************/

#ifndef OCC_MAP_HPP
#define OCC_MAP_HPP

#include <mutex>
#include <atomic>
#include <glog/logging.h>
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"
#include "common/config/system_config.hpp"
#include "occupancy_map/occupancy_map_common.hpp"
#include "common/math_base/slam_transform.hpp"

namespace cvte_lidar_slam {

class MapManager;
class OccMap {
 public:
  ~OccMap() {
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
  }
  static std::shared_ptr<OccMap> getInstance();
  void updateParameter();
  void initLaserScanParam();

  void insertObstacleCloud(const laserCloud::Ptr obstacle_cloud,
                           const Mat34d &pose, const size_t frame_id,
                           const Mat34d &delta_pose);

  void setRobotPose(const Mat34d &pose) {
    robot_transform_mtx_.lock();
    robot_pose_ = pose;
    has_robot_point_ = true;
    robot_transform_mtx_.unlock();
  }

  void setConfig(const OccMapConfig &config) {
    occ_map_config_ = config;
    range_size_ = std::round(
        (occ_map_config_.laser_max_angle - occ_map_config_.laser_min_angle) /
        occ_map_config_.laser_angle_increment);
    laser_scan_.resize(range_size_, occ_map_config_.laser_max_range + 1.);
  }

  void setMapFilePath(const std::string &path);

  void clearTmpMap() {
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_scan_data_size_ = 0;
    for (int i = 0; i < mapArrayLength; ++i) {
      for (int j = 0; j < mapArrayLength; ++j) {
        //
        map_array_ind_[i][j] = -1;
      }
    }
  }

  bool setExtrincs(const Mat34d &T_lo) {
    if (!T_lo.hasNaN()) {
      T_lo_ = T_lo;
      LOG(WARNING) << "setExtrincs T_lo_: \n" << T_lo_;
      return true;
    }
    return false;
  }

  void Reset() {
    v_scan_data_.clear();
    std::vector<ScanData>().swap(v_scan_data_);
    v_scan_data_.reserve(200);
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_scan_data_size_ = 0;
    is_stop_requested_ = false;
    for (int i = 0; i < mapArrayLength; ++i) {
      for (int j = 0; j < mapArrayLength; ++j) { map_array_ind_[i][j] = -1; }
    }
  }

  bool updateGlobalMap();
  void saveGlobalMapFile(const UserOccupancyGrid &map,
                         const std::string &map_dir);
  void saveGlobalMap();

  void saveScanData(const std::string &map_dir);

  bool saveScan(const std::string &map_dir);

  bool loadScan(const std::string &map_dir);

  void resetUpdateValue() { clearTmpMap(); }

  void clearAndSaveMap(const std::string &map_dir);
  void clearAndUpdateMap();

  UserOccupancyGrid getCurrOccMap();

 private:
  void allocateMemory();

  void initializeLocalOccupancyMap();

  bool getPointGrid(const Vec2d &point, Grid &thisGrid);

  MapCell *grid2MapCell(const Grid &grid);

  bool cloud2LaserScan(const laserCloud::Ptr cloud);

  void updateChildMap(const Vec2d &point, const double laser_inv_value);

  double laserInvModel(const double &r, const double &R,
                       const double &cell_size);

  /**
   *subMapExist
   *@brief
   *????????????????????????
   *
   *@param[in] cube_x-x??????
   *@param[in] cube_y-y??????
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

  unsigned int getMapArraySize() { return map_array_.size(); }
  int getMapArrayIndX(const int index) { return map_array_[index].indX; }
  int getMapArrayIndY(const int index) { return map_array_[index].indY; }
  float getCellSize() { return mapResolution; }

  bool getRobotPosition();

  void getMap();
  void getLocalMap();
  void initOccupancyMap();

  //????????????
  UserOccupancyGrid global_occ_map_;
  Grid local_map_origin_grid_;
  Grid global_map_origin_grid_;

  std::vector<ScanData> v_scan_data_;
  std::vector<float> laser_scan_;
  u_int32_t range_size_;
  int **
      map_array_ind_;  ///< it saves the index of this submap in vector mapArray

  // Map Arrays
  int map_array_count_;
  std::vector<ChildMap> map_array_;
  size_t last_scan_data_size_ = 0;
  std::atomic<bool> is_stop_requested_;
  int pub_count_;

  PointType robot_point_;
  PointType local_map_origin_point_;
  PointType origin_robot_point_;
  PointType global_map_origin_point_;

  Pose2d g_robot_pose_;
  Mat34d robot_pose_;

  //????????????
  std::mutex robot_transform_mtx_;
  std::mutex state_mtx_;
  std::atomic<bool> has_robot_point_;

  // ?????????????????????
  OccMapConfig occ_map_config_;
  std::shared_ptr<MapManager> ptr_map_manager_ = nullptr;  ///< ???????????????
  std::string default_map_filepath_;

  pcl::VoxelGrid<PointType> downSize_occ_cloud_;  ///< ??????????????????

  laserCloud::Ptr cloud_trans_;
  Mat34d T_lo_;

 private:
  OccMap();
  static std::shared_ptr<OccMap> ptr_occ_map_;
};
}  // namespace cvte_lidar_slam

#endif
