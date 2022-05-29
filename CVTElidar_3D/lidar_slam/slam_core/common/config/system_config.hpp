/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file system_config.hpp
 *
 *@brief
 * 参数设置类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <cmath>
#include <iostream>
#include <string>
#include "common/debug_tools/debug_color.h"

namespace cvte_lidar_slam {

/**
 * HorizonFeatureConfig
 * @brief Horizon点云特征提取参数类
 **/
struct HorizonFeatureConfig {
  double range_x = 60;
  double range_y = 60;            ///<>
  double grid_resolution = 0.25;  ///<>
  int neighbor_search_num = 10;
  double plane_scale_threshold = 0.25;
};

/**
 * FeatureConfig
 * @brief 点云特征提取参数类
 **/
struct FeatureConfig {
  unsigned int N_SCAN = 16;
  unsigned int Horizon_SCAN = 1800;
  float ang_res_x = 0.2;
  float ang_res_y = 2.0;
  float ang_bottom = 15.0 + 0.1;
  unsigned int groundScanInd = 7;  //小于７的线数才被考虑为地面点
  float scanPeriod = 0.1;
  int systemDelay = 0;
  int imuQueLength = 200;

  float sensorMountAngle = 0.0;
  float segmentTheta =
      60.0 / 180.0 * M_PI;  // decrese this value may improve accuracy
  unsigned int segmentValidPointNum = 5;
  unsigned int segmentValidLineNum = 3;
  float segmentAlphaX = ang_res_x / 180.0 * M_PI;
  float segmentAlphaY = ang_res_y / 180.0 * M_PI;

  int edgeFeatureNum = 2;
  int surfFeatureNum = 4;
  int sectionsTotal = 6;
  float edgeThreshold = 0.3;
  float surfThreshold = 0.1;
};  // end of class

/**
 * TraverConfig
 * @brief 可通性检测参数类
 **/
struct TraverConfig {
  unsigned int N_SCAN = 16;
  unsigned int Horizon_SCAN = 1800;
  int scanNumCurbFilter = 8;
  int scanNumSlopeFilter = 10;
  float sensorRangeLimit = 20;
  float filterAngleLimit = 20;
  int scanNumMax = std::max(scanNumCurbFilter, scanNumSlopeFilter);
  float mapResolution = 0.1;
  int filterHeightMapArrayLength = sensorRangeLimit * 2 / mapResolution;
  float filterHeightLimit = 0.15;    // step diff threshold
  float predictionKernalSize = 0.1;  // predict elevation within x meters

  double range_x = 40;
  double range_y = 40;            ///<>
  double grid_resolution = 0.25;  ///<>
  int neighbor_search_num = 10;
  double plane_scale_threshold = 0.25;
  double sensor_height_limit = 0.5;
  float height_diff_limit = 0.1;
};

/**
 * FrontendConfig
 * @brief 前端参数类
 **/
struct FrontendConfig {
  bool is_new_tracking = false;
  int iterCount = 25;
  float nearestFeatureSearchSqDist = 25.;
  int maxPointNum = 28800;  // 16 * 1800
  bool is_dynamic_remove = false;

  std::string model_path = "";
  std::string range_path = "";
};  // end of class

/**
 * LoopConfig
 * @brief 闭环参数类
 **/
struct LoopConfig {
  int time_diff = 120;  ///< 闭环检测时间阈值

  int id_diff = 100;  ///< 闭环id阈值

  double icp_score = 0.2;  ///< icp陪准得分阈值

  double relocalization_icp_score = 0.2;

  int history_search_num = 70;

  double max_loop_angle = 1.;  ///<

  double loop_search_radius = 10;

  int last_keyframe_size = 3;

};  // end of class

/**
 * GpsConfig
 * @brief gps参数类
 **/
struct GpsConfig {
  std::string gps_map_path = "";  ///< gps参数地址

  int gps_point_sum = 150;  ///< gps最小数量
};

/**
 * BackendConfig
 * @brief 后端参数类
 **/
struct BackendConfig {
  bool use_ceres_opt = false;
  bool is_localization_mode =
      true;  ///< 系统模式标志，true-定位模式，false-构图模式

  float angle_threshold = 0.5;  ///< 帧创建角度阈值

  float trans_threshold = 0.2;  ///< 帧创建距离阈值

  float surround_search_radius = 50.0;  ///< 临域关键帧搜索半径

  int surround_search_num = 50;  ///< 最近关键帧集合数量

  float loop_search_radius = 7.0;  ///< 闭环关键帧搜索半径

  int recent_frame_number = 10;  ///< 最近关键帧数量

  int history_search_num = 25;  ///< 临域关键帧数量

  float loop_candidate_fitness_score = 0.3;  ///< 闭环icp陪准分数阈值

  float global_map_visualization_radius = 500.0;  ///< 关键帧可视化范围

  float leaf_size = 0.7;  ///< 临域分辨率

  bool is_search_by_time = false;  ///< 按时间搜索

  float min_distance = 0.3;  ///< 关键帧创建距离阈值

  std::string map_path = "";  ///< 地图保存或载入地址

  std::string lidar_type = "velodyne";  ///< 特征提取的方法

  bool using_gps_init = false;  ///< gps使用标志位

  bool using_feature_iris_init = false;  ///< iris激光特征使用标志位
};                                       // end of class

/**
 * MsfConfig
 * @brief 多传感器融合算法参数
 **/
struct MsfConfig {
  bool fix_first_node = true;    ///< 是否固定首个优化节点
  bool fix_gps_extrincs = true;  ///< 是否固定gps外参
  bool using_gps = false;        ///< 是否使用GPS
  unsigned int new_node_size_to_opt =
      0;  ///< 优化条件阈值，超过多少个新节点进行优化
  unsigned int opt_windows_size = 0;  ///< 优化窗口大小
  unsigned int gps_count_to_opt =
      1;  ///< 优化条件阈值，超过多少个新GPS数据进行优化
  unsigned int gps_calculate_tr_count = 0;

  int opt_loop_count = 3;  ///<多少个闭环才优化
};

/**
 * OccMapConfig
 * @brief 2d 栅格地图参数
 **/
struct OccMapConfig {
  float laser_min_angle = -M_PI;
  float laser_max_angle = M_PI;
  float laser_min_range = 0.1;
  float laser_max_range = 20;
  float laser_angle_increment = 1.0f / 180 * M_PI;
  bool use_curb_cloud = true;
  bool use_view_cloud = true;
  float ground_height = -0.52;
  float curb_min_range = 1.0;
  float curb_max_range = 20.0;
  float view_max_height = 2.0;
  float curb_y_limit = 0.5;
  float curb_map_resolution = 0.1;
  int curb_map_length = curb_max_range * 2 / curb_map_resolution;
  float filter_height_limit = 0.3;
  float curb_value = 10.0;
  float view_value = 0.2;
};  // end of class

/**
 * SystemConfig
 * @brief 系统参数类
 **/
struct SystemConfig {
  /**
   * SystemConfig
   * @brief 系统参数类
   * @param[in] file-参数文件
   **/
  SystemConfig(const std::string &file);
  std::string lidar_topic_name;
  FeatureConfig feature_config;
  FrontendConfig frontend_config;
  LoopConfig loop_config;
  GpsConfig gps_config;
  BackendConfig backend_config;
  MsfConfig msf_config;
  OccMapConfig occ_map_config;
};

}  // namespace cvte_lidar_slam

#endif  // SYSTEM_CONFIG_H
