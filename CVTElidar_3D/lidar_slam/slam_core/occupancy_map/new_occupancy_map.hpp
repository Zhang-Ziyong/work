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

namespace cvte_lidar_slam {

class MapManager;

const float filterHeightLimit = 0.15;  // step diff threshold

const float mapCubeLength = 1.0;  // the length of a sub-map (meters)
const float mapResolution = 0.1;
const int mapCubeArrayLength = mapCubeLength / mapResolution;
const int mapArrayLength = 2000 / mapCubeLength;
const int rootCubeIndex = mapArrayLength / 2;

// Occupancy Params
const float p_occupied_when_laser = 0.9;
const float p_occupied_when_no_laser = 0.2;
const float large_log_odds = 100;
const float max_log_odds_for_belief = 20;

// 2D Map Publish Params
const int localMapLength =
    20;  // length of the local occupancy grid map (meter)
const int localMapArrayLength = localMapLength / mapResolution;

// Robot Params
const float robotRadius = 0.1;
const float sensorHeight = 0.5;

/**
 * ScanData
 * @brief 激光扫描数据
 *
 **/
struct ScanData {
  std::vector<float> v_laser_scan;  // laser scan data
  size_t frame_id;                  // 最近关键帧的位置
  Mat34d delta_pose;  // 当前scan位置与最近关键帧的相对位置
  laserCloud::Ptr curb_cloud;  // 路沿点
  laserCloud::Ptr view_cloud;  // 可视化点云，增强地图细节
  bool cloud_filter_flag = false;
  // Pose2d robot_pose;                   // 2d pose of robot
};

struct UserOccupancyGrid {
  struct Header {
    std::string frame_id;
    double time_stamp = -1.;
  };  // header info
  struct MapInfo {
    float resolution;
    u_int32_t width;
    u_int32_t height;
    Mat34d origin;

  };  // MetaData for the map
  Header header;
  MapInfo info;
  std::vector<int8_t> data;
};  // end of class

/**
 * Grid
 * @brief 地图栅格数据结构
 *
 **/
struct Grid {
  int mapID;
  int cubeX;
  int cubeY;
  int gridX;
  int gridY;
  int gridIndex;
};  // end of class

/**
 * MapCell
 * @brief Cell Definition
 * 1.a cell is a member of a grid in a sub-map
 * 2.a grid can have several cells in it.
 * 3.a cell represent one height information
 **/
struct MapCell {
  float log_odds;

  MapCell() { log_odds = 0.; }

  inline float getBel() { return (1.0 - 1.0 / (1 + exp(log_odds))); }

  void setLogBel(const float log_bel) {
    log_odds += log_bel;
    if (log_odds > 5 || log_odds < -4) {
      log_odds -= log_bel;
    }
  }

  void setCurbLogBel(const float log_bel) {
    log_odds += log_bel;
    if (log_odds > 100 || log_odds < -4) {
      log_odds -= log_bel;
    }
  }
};  // end of class

/**
 * Sub-map Definition
 * @brief Cell Definition
 * 1.ChildMap is a small square. We call it "cellArray".
 * 2.It composes the whole map
 **/
struct ChildMap {
  std::vector<std::vector<MapCell>> cellArray;
  int subInd;     // sub-map's index in 1d mapArray
  int indX;       // sub-map's x index in 2d array mapArrayInd
  int indY;       // sub-map's y index in 2d array mapArrayInd
  float originX;  // sub-map's x root coordinate
  float originY;  // sub-map's y root coordinate
  // pcl::PointCloud<PointType> cloud;

  ChildMap(int id, int indx, int indy) {
    subInd = id;
    indX = indx;
    indY = indy;
    originX = (indX - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;
    originY = (indY - rootCubeIndex) * mapCubeLength - mapCubeLength / 2.0;

    // allocate and initialize each cell
    cellArray.resize(mapCubeArrayLength);
    for (int i = 0; i < mapCubeArrayLength; ++i)
      cellArray[i].resize(mapCubeArrayLength);

    for (int i = 0; i < mapCubeArrayLength; ++i)
      for (int j = 0; j < mapCubeArrayLength; ++j) cellArray[i][j] = MapCell();
    // allocate point cloud for visualization
  }
};  // end of class

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

  bool setExtrincs(const Mat34d &T_lo) {
    if (!T_lo.hasNaN()) {
      T_lo_ = T_lo;
      LOG(WARNING) << "======setExtrincs " << T_lo_;
      return true;
    }
    return false;
  }

  void setMapFilePath(const std::string &path);

  void saveMap(const std::string &map_dir);

  void clearTmpMap() {
    map_array_.clear();
    std::vector<ChildMap>().swap(map_array_);
    map_array_.reserve(100);
    map_array_count_ = 0;
    last_scan_data_size_ = 0;
    for (int i = 0; i < mapArrayLength; ++i) {
      for (int j = 0; j < mapArrayLength; ++j) { map_array_ind_[i][j] = -1; }
    }
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

  bool saveScan(const std::string &map_dir);

  bool loadScan(const std::string &map_dir);

  void resetUpdateValue() { clearTmpMap(); }

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

  unsigned int getMapArraySize() { return map_array_.size(); }
  int getMapArrayIndX(const int index) { return map_array_[index].indX; }
  int getMapArrayIndY(const int index) { return map_array_[index].indY; }
  float getCellSize() { return mapResolution; }

  bool getRobotPosition();

  void getMap();
  void getLocalMap();
  void initOccupancyMap();
  void curbFilter(const laserCloud::Ptr cloud);
  bool rangeFilter(const laserCloud::Ptr cloud);
  void updateCurb(const Vec2d &point, const double laser_inv_value);

  //模块数据
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

  //模块状态
  std::mutex robot_transform_mtx_;
  std::mutex state_mtx_;
  std::atomic<bool> has_robot_point_;

  // 模块依赖和配置
  OccMapConfig occ_map_config_;
  std::shared_ptr<MapManager> ptr_map_manager_ = nullptr;  ///< 当前帧指针
  std::string default_map_filepath_;

  laserCloud::Ptr cloud_curb_;
  laserCloud::Ptr cloud_view_;
  laserCloud::Ptr cloud_range_;
  laserCloud::Ptr cloud_trans_;
  std::vector<std::vector<bool>> init_flag_;
  std::vector<std::vector<float>> min_height_;
  std::vector<std::vector<float>> max_height_;
  pcl::VoxelGrid<PointType> downSize_filter_;  ///< 下采样滤波器
  Mat34d T_lo_;                                ///< laser to odom

 private:
  OccMap();
  static std::shared_ptr<OccMap> ptr_occ_map_;
};

}  // namespace cvte_lidar_slam

#endif
