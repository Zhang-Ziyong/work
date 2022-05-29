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
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/

#include "system_config.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

namespace cvte_lidar_slam {
SystemConfig::SystemConfig(const std::string &file) {
  cv::FileStorage fsSettings(file.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "Failed to open settings file at: " << file << std::endl;
    exit(-1);
  }

  std::cout << GREEN << "This is lidar slam demo " << std::endl;

  // cv::FileNode features = fs2["features"];

  backend_config.surround_search_radius = fsSettings["surround_search_radius"];
  backend_config.angle_threshold = fsSettings["angle_threshold"];
  backend_config.trans_threshold = fsSettings["trans_threshold"];
  backend_config.min_distance = fsSettings["min_distance"];
  fsSettings["map_path"] >> backend_config.map_path;
  fsSettings["lidar_type"] >> backend_config.lidar_type;
  fsSettings["lidar_topic_name"] >> lidar_topic_name;
  int temp1 = fsSettings["is_new_tracking"];
  frontend_config.is_new_tracking = bool(temp1);
  int temp = fsSettings["is_localization_mode"];
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
  temp = fsSettings["using_iris_feature"];
  backend_config.using_feature_iris_init = bool(temp);
  temp = fsSettings["using_ceres_opt"];
  backend_config.use_ceres_opt = bool(temp);

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
  loop_config.icp_score = fsSettings["icp_score"];
  loop_config.time_diff = fsSettings["time_diff"];
  loop_config.id_diff = fsSettings["id_diff"];
  loop_config.loop_search_radius = fsSettings["loop_search_radius"];
  loop_config.max_loop_angle = fsSettings["max_loop_angle"];
  loop_config.last_keyframe_size = fsSettings["last_keyframe_size"];
  loop_config.relocalization_icp_score = fsSettings["relocalization_icp_score"];

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

  temp = fsSettings["using_occ_curb"];
  occ_map_config.use_curb_cloud = bool(temp);
  occ_map_config.curb_value = fsSettings["occ_curb_value"];

  temp = fsSettings["using_occ_view"];
  occ_map_config.use_view_cloud = bool(temp);

  std::cout << RESET << std::endl;
}

}  // namespace cvte_lidar_slam