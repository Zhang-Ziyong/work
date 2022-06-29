/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file system_config.cpp
 *
 *@brief
 * 参数设置类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@modify Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/

#include "system_config.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

namespace cvte_lidar_slam {

SystemConfig::SystemConfig() {
  std::cout << "SystemConfig construct" << std::endl;
}

SystemConfig::SystemConfig(const std::string &file) {
  cv::FileStorage fsSettings(file.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << file << std::endl;
    exit(-1);
  }

  std::cout << GREEN << "This is lidar slam demo " << std::endl;

  // cv::FileNode features = fs2["features"];

  int temp = fsSettings["USE_IMU"];
  USE_IMU = bool(temp);

  backend_config.USE_IMU = USE_IMU;
  temp = fsSettings["use_depth_cloud"];
  use_depth_cloud = bool(temp);

  temp = fsSettings["print_debug"];
  print_debug = bool(temp);
  backend_config.print_debug = print_debug;

  temp = fsSettings["pointCloud_debug"];
  pointCloud_debug = bool(temp);
  backend_config.pointCloud_debug = pointCloud_debug;

  backend_config.surround_search_radius = fsSettings["surround_search_radius"];
  backend_config.surround_search_num = fsSettings["surround_search_num"];
  backend_config.angle_threshold = fsSettings["angle_threshold"];
  backend_config.trans_threshold = fsSettings["trans_threshold"];
  backend_config.time_threshold = fsSettings["time_threshold"];
  backend_config.min_distance = fsSettings["min_distance"];
  fsSettings["map_path"] >> backend_config.map_path;
  fsSettings["lidar_type"] >> backend_config.lidar_type;
  fsSettings["lidar_topic_name"] >> lidar_topic_name;
  fsSettings["wheel_odom_topic_name"] >> wheel_odom_topic_name;
  fsSettings["imu_topic_name"] >> imu_topic_name;
  temp = fsSettings["is_new_tracking"];
  frontend_config.is_new_tracking = bool(temp);
  temp = fsSettings["is_localization_mode"];
  backend_config.is_localization_mode = bool(temp);
  temp = fsSettings["is_search_by_time"];
  backend_config.is_search_by_time = bool(temp);

  if (backend_config.is_localization_mode) {
    std::cout << GREEN << "SYSTEM_MODE: Localization " << std::endl;
  } else {
    std::cout << GREEN << "SYSTEM_MODE: Mapping " << std::endl;
  }

  temp = fsSettings["using_gps_init"];
  backend_config.using_gps_init = bool(temp);
  temp = fsSettings["using_ceres_opt"];
  backend_config.use_ceres_opt = bool(temp);

  backend_config.match_algorithm = fsSettings["match_algorithm"];
  backend_config.max_match_dis = fsSettings["max_match_dis"];
  backend_config.edge_size = fsSettings["edge_size"];
  backend_config.surf_size = fsSettings["surf_size"];
  backend_config.max_map_match_success_score =
      fsSettings["max_map_match_success_score"];
  backend_config.min_map_match_overlap_ratio =
      fsSettings["min_map_match_overlap_ratio"];
  backend_config.mapping_match_smooth_ratio =
      fsSettings["mapping_match_smooth_ratio"];
  backend_config.loc_match_smooth_ratio = fsSettings["loc_match_smooth_ratio"];
  backend_config.lidar_odom_smooth_ratio =
      fsSettings["lidar_odom_smooth_ratio"];
  backend_config.fusion_pose_smooth_ratio =
      fsSettings["fusion_pose_smooth_ratio"];
  backend_config.odom_match_ratio = fsSettings["odom_match_ratio"];
  backend_config.lidar_odom_match_smooth_ratio =
      fsSettings["lidar_odom_match_smooth_ratio"];

  backend_config.gps_cov = fsSettings["gps_cov"];
  backend_config.loop_cov = fsSettings["loop_cov"];
  backend_config.relative_cov = fsSettings["relative_cov"];

  temp = fsSettings["save_keyPose_to_file"];
  backend_config.save_keyPose_to_file = bool(temp);

  backend_config.min_feature_points = fsSettings["min_feature_points"];

  backend_config.angular_threshold = fsSettings["angular_threshold"];

  temp = fsSettings["map_update"];
  backend_config.map_update = bool(temp);
  backend_config.increase_ratio = fsSettings["increase_ratio"];
  backend_config.decrease_ratio = fsSettings["decrease_ratio"];
  backend_config.increase_duration = fsSettings["increase_duration"];
  backend_config.decrease_duration = fsSettings["decrease_duration"];
  backend_config.static_duration = fsSettings["static_duration"];
  backend_config.init_map_prob = fsSettings["init_map_prob"];
  backend_config.min_match_prob = fsSettings["min_match_prob"];
  backend_config.new_add_prob = fsSettings["new_add_prob"];
  backend_config.min_erase_prob = fsSettings["min_erase_prob"];

  gps_config.gps_map_path = backend_config.map_path + "gps/";

  std::string dir = backend_config.map_path;

  if (!backend_config.is_localization_mode) {
    if (access(dir.c_str(), 0) == -1) {
      int flag = mkdir(dir.c_str(), 0777);
      int gps_flag = mkdir((dir + "gps/").c_str(), 0777);
      if (flag == 0 && gps_flag == 0) {
        std::cout << GREEN << dir << " make successfully" << std::endl;
      } else {
        std::cout << "make errorly" << std::endl;
      }
    } else {
      if (access((dir + "keyframe_pos.pcd").c_str(), 0) == 0) {
        std::cout << dir << "  is exist, make errorly" << std::endl;
        exit(-1);
      }
    }
  }

  gps_config.gps_point_sum = fsSettings["gps_point_sum"];
  loop_config.history_search_num = fsSettings["history_search_num"];
  loop_config.loop_icp_score = fsSettings["loop_icp_score"];
  loop_config.time_diff = fsSettings["time_diff"];
  loop_config.id_diff = fsSettings["id_diff"];
  loop_config.loop_search_radius = fsSettings["loop_search_radius"];
  loop_config.max_loop_angle = fsSettings["max_loop_angle"];
  loop_config.last_keyframe_size = fsSettings["last_keyframe_size"];
  loop_config.history_cloud_ds_size = fsSettings["history_cloud_ds_size"];
  loop_config.relocalization_icp_score = fsSettings["relocalization_icp_score"];
  loop_config.relocalization_search_radius =
      fsSettings["relocalization_search_radius"];

  feature_config.edge_size = fsSettings["edge_size"];
  feature_config.surf_size = fsSettings["surf_size"];
  feature_config.min_range = fsSettings["min_range"];
  feature_config.max_range = fsSettings["max_range"];
  feature_config.min_feature_points = fsSettings["min_feature_points"];

  feature_config.narrow_candidate = fsSettings["narrow_candidate"];
  feature_config.narrow_region = fsSettings["narrow_region"];
  feature_config.narrow_ratio = fsSettings["narrow_ratio"];

  feature_config.small_cluster_size = fsSettings["small_cluster_size"];

  temp = fsSettings["fix_first_node"];
  msf_config.fix_first_node = bool(temp);
  temp = fsSettings["fix_gps_extrincs"];
  msf_config.fix_gps_extrincs = bool(temp);
  temp = fsSettings["using_gps"];
  msf_config.using_gps = bool(temp);

  temp = fsSettings["new_node_size_to_opt"];
  if (temp < 0) {
    temp = 0;
  }
  msf_config.new_node_size_to_opt = temp;

  temp = fsSettings["opt_windows_size"];
  if (temp < 0) {
    temp = 0;
  }
  msf_config.opt_windows_size = temp;

  msf_config.opt_loop_count = fsSettings["opt_loop_count"];
  temp = fsSettings["gps_count_to_opt"];
  if (temp < 0) {
    temp = 0;
  }
  msf_config.gps_count_to_opt = temp;

  temp = fsSettings["gps_point_sum"];
  if (temp < 0) {
    temp = 0;
  }
  msf_config.gps_calculate_tr_count = temp;
  msf_config.gps_cov = fsSettings["gps_cov"];
  msf_config.loop_cov = fsSettings["loop_cov"];
  msf_config.relative_cov = fsSettings["relative_cov"];

  std::cout << RESET << std::endl;
}

}  // namespace cvte_lidar_slam