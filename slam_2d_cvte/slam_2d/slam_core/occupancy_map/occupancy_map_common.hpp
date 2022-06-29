/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file occupancy_map_common.hpp
 *
 *@brief 占用栅格地图相关的类型定义
 *
 *
 *@author caoyong(caoyong@cvte.com)
 *@version 1.0
 *@data 2021-04-21
 ************************************************************************/
#ifndef SLAM_2D_OCCUPANCY_MAP_COMMON_HPP
#define SLAM_2D_OCCUPANCY_MAP_COMMON_HPP

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"

namespace cvte_lidar_slam {
const float filterHeightLimit = 0.15;  // step diff threshold

const float mapCubeLength = 1.0;   // the length of a sub-map (meters)
const float mapResolution = 0.05;  // 0.1
const float localMapResolution = 0.05;
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
 * @brief 激光扫描数据结构
 *
 **/
struct ScanData {
  std::vector<float> v_laser_scan;  // laser scan data
  size_t frame_id;                  // 最邻近的id
  Mat34d delta_pose;           // 当前scan位置与最邻近的相对位置
  depthCloud::Ptr view_cloud;  // 可视化点云
};

/**
 * TraverCloudData
 * @brief 点云数据
 *
 **/
struct CloudData {
  laserCloud::Ptr laser_cloud;  // laser cloud data
  size_t frame_id;              // 最近关键帧的id
  Mat34d delta_pose;  // 当前点云位置与最近关键帧的相对位置
};

/**
 * UserOccupancyGrid
 * @brief 自定义占用栅格数据类型
 *
 **/
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
  float pointX;
  float pointY;
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
  float pointX;
  float pointY;

  MapCell() { log_odds = 0.; }

  inline float getBel() { return (1.0 - 1.0 / (1 + exp(log_odds))); }

  void setLogBel(const float log_bel, const float &max_bel_value) {
    log_odds += log_bel;
    // f (log_odds > 5 || log_odds < -4) {
    // log_odds -= log_bel;
    //}
    // 超过设定的最大值时，不进行更新，保持最大值
    if (log_odds > max_bel_value) {
      log_odds = max_bel_value;
    }
    if (log_odds < -1 * max_bel_value) {
      log_odds = -1 * max_bel_value;
    }
  }

  void setCurbLogBel(const float &log_bel, const float &max_bel_value) {
    log_odds += log_bel;
    // if (log_odds > 100 || log_odds < -4) {
    //   log_odds -= log_bel;
    // }
    if (log_odds > max_bel_value) {
      log_odds = max_bel_value;
    }
  }

  void setCurbDownLogBel(const float &log_bel) {
    log_odds -= log_bel;
    if (log_odds < 0) {
      log_odds = 0.0;
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

  ChildMap(const int &id, const int &indx, const int &indy,
           const int &rootCubeIndex, const int &mapCubeLength,
           const int &mapCubeArrayLength) {
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
}  // namespace cvte_lidar_slam

#endif