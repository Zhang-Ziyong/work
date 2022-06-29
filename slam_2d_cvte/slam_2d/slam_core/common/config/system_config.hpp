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
 *@modify Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
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
  std::string lidar_type = "velodyne";  ///< 特征提取的方法

  unsigned int N_SCAN = 16;
  unsigned int Horizon_SCAN = 1800;
  float ang_res_x = 0.2;
  float ang_res_y = 2.0;
  float ang_bottom = 15.0 + 0.1;
  unsigned int groundScanInd = 5;  //小于７的线数才被考虑为地面点
  float scanPeriod = 0.1;
  int systemDelay = 0;
  int imuQueLength = 200;

  float min_range = 0.3;
  float max_range = 20.0;

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
  // float edgeThreshold = 0.3;
  // float surfThreshold = 0.1;
  float edgeThreshold = 0.2;
  float surfThreshold = 0.2;
  float edgeThresholdNorm = 0.02;
  float surfThresholdNorm = 0.02;

  float edge_size = 0.1;
  float surf_size = 0.1;

  int min_feature_points = 40;

  float narrow_candidate = 2.0;
  float narrow_region = 1.5;
  float narrow_ratio = 0.8;

  float small_cluster_size = 0.3;

  bool calculate_degenerate = true;

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
  int time_diff = 60;  ///< 闭环检测时间阈值

  int id_diff = 120;  ///< 闭环id阈值

  double loop_icp_score = 0.02;  ///< icp得分阈值

  int history_search_num = 30;

  double max_loop_angle = 0.628;  ///<

  double loop_search_radius = 8.0;

  int last_keyframe_size = 3;

  double history_cloud_ds_size = 0.1;

  double relocalization_search_radius = 10.0;

  double relocalization_icp_score = 0.2;

  double loop_relative_pose = 2.0;

  int relocalization_box_size_x = 2;
  int relocalization_box_size_y = 2;
  int relocalization_box_size_yaw = 2;

  double icp_fitness_score = 2.0;

  int loop_track_count = 10;

  double opt_update_map_time = 10.0;

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
  bool use_ceres_opt = true;
  bool is_localization_mode =
      false;  ///< 系统模式标志，true-定位模式，false-构图模式

  float angle_threshold = 0.4;  ///< 帧创建角度阈值

  float trans_threshold = 0.35;  ///< 帧创建距离阈值

  float time_threshold = 100.0;  ///< 帧创建时间阈值

  float angular_threshold = 0.3;  ///< 帧创建角速度阈值

  float surround_search_radius = 10.0;  ///< 临域关键帧搜索半径

  int surround_search_num = 150;  ///< 最近关键帧集合数量

  float loop_search_radius = 8.0;  ///< 闭环关键帧搜索半径

  int recent_frame_number = 10;  ///< 最近关键帧数量

  int history_search_num = 30;  ///< 临域关键帧数量

  float loop_candidate_fitness_score = 0.1;  ///< 闭环icp陪准分数阈值

  float global_map_visualization_radius = 500.0;  ///< 关键帧可视化范围

  float leaf_size = 0.3;  ///< 临域分辨率 0.7

  bool is_search_by_time = true;  ///< 按时间搜索

  float min_distance = 0.3;  ///< 关键帧创建距离阈值

  std::string map_path = "";  ///< 地图保存或载入地址

  std::string lidar_type = "2D";  ///< 特征提取的方法

  bool using_gps_init = false;  ///< gps使用标志位

  bool USE_IMU = false;
  bool print_debug = false;
  bool pointCloud_debug = false;

  int match_algorithm = 1;    ///< 匹配算法： 1-PCL，2-ceres
  float max_match_dis = 0.3;  ///< 匹配点最大距离阈值

  float edge_size = 0.1;
  float surf_size = 0.1;

  float max_map_match_success_score = 0.06;
  float min_map_match_overlap_ratio = 0.3;

  bool save_keyPose_to_file = false;

  int min_feature_points = 40;

  float odom_match_ratio = 1.0;
  float mapping_match_smooth_ratio = 1.0;

  float fusion_pose_smooth_ratio = 0.005;
  float lidar_odom_smooth_ratio = 0.5;

  float loc_match_smooth_ratio = 0.5;
  float lidar_odom_match_smooth_ratio = 1.0;

  float gps_cov = 20.0;
  float loop_cov = 0.2;
  float relative_cov = 0.02;

  int max_match_failed = 200;
  int max_low_overlap = 200;

  bool map_update = false;
  float increase_ratio = 1.0;
  float decrease_ratio = 1.0;
  float increase_duration = 600.0;
  float decrease_duration = 3600.0;
  float static_duration = 200.0;
  float init_map_prob = 60.0;
  float min_match_prob = 40.0;
  float new_add_prob = 30.0;
  float min_erase_prob = 20.0;

  float degenerate_threshold = 50.0;
  float degenerate_lidar_odom_smooth_ratio = 0.1;
  float degenerate_loc_match_smooth_ratio = 0.1;

};  // end of class

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

  float gps_cov = 20.0;
  float loop_cov = 0.2;
  float relative_cov = 0.02;
};

/**
 * OccMapConfig
 * @brief 2d 栅格地图参数
 **/
struct OccMapConfig {
  float laser_min_angle = -M_PI;
  float laser_max_angle = M_PI;
  float laser_min_range = 0.2;
  float laser_max_range = 20.0;
  // float laser_angle_increment = 1.0f / 180 * M_PI;
  float laser_angle_increment = 0.2f / 180 * M_PI;
  bool use_curb_cloud = false;
  float ground_height = -0.52;
  float curb_min_range = 1.0;
  float curb_max_range = 20.0;
  float curb_y_limit = 0.5;
  float curb_map_resolution = 0.1;
  int curb_map_length = curb_max_range * 2 / curb_map_resolution;
  float filter_height_limit = 0.3;
  float curb_value = 10.0;
  float point_size = 0.05;
  bool use_view_cloud = true;  // 使用点云增强地图细节
  int local_map_size = 20;
  bool dynamic_cloud_remove = false;  // 是否使用局部栅格地图移除动态点云
  float local_map_resolution = 0.1;
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
  SystemConfig();
  SystemConfig(const std::string &file);
  std::string lidar_topic_name;
  std::string wheel_odom_topic_name;
  std::string imu_topic_name;
  bool USE_IMU = false;
  bool use_depth_cloud = false;
  bool print_debug = false;
  bool pointCloud_debug = false;
  FeatureConfig feature_config;
  FrontendConfig frontend_config;
  LoopConfig loop_config;
  GpsConfig gps_config;
  BackendConfig backend_config;
  MsfConfig msf_config;
  OccMapConfig occ_map_config;
  double depthcloud_sample_frequent = 1.0;
};

}  // namespace cvte_lidar_slam

#endif  // SYSTEM_CONFIG_H
