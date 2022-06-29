/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file load_param_ros2.hpp
 *
 *@brief
 * 相关参数配置类
 *
 *@author caoyong(caoyong@cvte.com)
 *@version v1.0
 *@data 2022-04-27
 ************************************************************************/
#include "load_param_ros2.hpp"
#include "public_parameters/PublicParameters.hpp"

namespace cvte_lidar_slam {

LoadParamsRos2::LoadParamsRos2(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr) {
  ptr_config_ = std::make_shared<SystemConfig>();
}
LoadParamsRos2::~LoadParamsRos2() {}

void LoadParamsRos2::loadSystemConfig() {
  getParameter("print_debug", ptr_config_->print_debug, false);
  getParameter("lidar_topic_name", ptr_config_->lidar_topic_name,
               std::string("/scan"));
  getParameter("wheel_odom_topic_name", ptr_config_->wheel_odom_topic_name,
               std::string("/odom"));
  getParameter("imu_topic_name", ptr_config_->imu_topic_name,
               std::string("/spatial_imu"));
  getParameter("depthcloud_sample_frequent",
               ptr_config_->depthcloud_sample_frequent, 1.0);

  getParameter("use_imu", ptr_config_->USE_IMU, false);
  getParameter("use_depth_camera", ptr_config_->use_depth_cloud, true);
  getParameter("pointCloud_debug", ptr_config_->pointCloud_debug, false);

  loadFeatureConfig();
  loadBackConfig();
  loadLoopConfig();
  loadMSFConfig();
  loadOccMapConfig();
  loadDepthCameraParams();
}

void LoadParamsRos2::loadFeatureConfig() {
  getParameter("feature_config.lidar_type",
               ptr_config_->feature_config.lidar_type, std::string("2D"));
  getParameter("feature_config.edge_size",
               ptr_config_->feature_config.edge_size, static_cast<float>(0.1));
  getParameter("feature_config.surf_size",
               ptr_config_->feature_config.surf_size, static_cast<float>(0.1));
  getParameter("feature_config.min_feature_points",
               ptr_config_->feature_config.min_feature_points, 60);
  getParameter("feature_config.narrow_candidate",
               ptr_config_->feature_config.narrow_candidate,
               static_cast<float>(2.0));
  getParameter("feature_config.narrow_region",
               ptr_config_->feature_config.narrow_region,
               static_cast<float>(1.5));
  getParameter("feature_config.narrow_ratio",
               ptr_config_->feature_config.narrow_ratio,
               static_cast<float>(0.8));
  getParameter("feature_config.small_cluster_size",
               ptr_config_->feature_config.small_cluster_size,
               static_cast<float>(0.3));
  getParameter("feature_config.calculate_degenerate",
               ptr_config_->feature_config.calculate_degenerate, true);
}

void LoadParamsRos2::loadLoopConfig() {
  getParameter("loop_config.history_search_num",
               ptr_config_->loop_config.history_search_num, 20);
  getParameter("loop_config.loop_icp_score",
               ptr_config_->loop_config.loop_icp_score, 0.08);
  getParameter("loop_config.time_diff", ptr_config_->loop_config.time_diff, 60);
  getParameter("loop_config.id_diff", ptr_config_->loop_config.id_diff, 120);
  getParameter("loop_config.loop_search_radius",
               ptr_config_->loop_config.loop_search_radius, 5.0);
  getParameter("loop_config.max_loop_angle",
               ptr_config_->loop_config.max_loop_angle, 0.314);
  getParameter("loop_config.last_keyframe_size",
               ptr_config_->loop_config.last_keyframe_size, 1);
  getParameter("loop_config.history_cloud_ds_size",
               ptr_config_->loop_config.history_cloud_ds_size, 0.1);
  getParameter("loop_config.relocalization_icp_score",
               ptr_config_->loop_config.relocalization_icp_score, 0.2);
  getParameter("loop_config.relocalization_search_radius",
               ptr_config_->loop_config.relocalization_search_radius, 10.0);
  getParameter("loop_config.loop_relative_pose",
               ptr_config_->loop_config.loop_relative_pose, 5.0);
  getParameter("loop_config.relocalization_box_size_x",
               ptr_config_->loop_config.relocalization_box_size_x, 2);
  getParameter("loop_config.relocalization_box_size_y",
               ptr_config_->loop_config.relocalization_box_size_y, 2);
  getParameter("loop_config.relocalization_box_size_yaw",
               ptr_config_->loop_config.relocalization_box_size_yaw, 2);
  getParameter("loop_config.icp_fitness_score",
               ptr_config_->loop_config.icp_fitness_score, 2.0);
  getParameter("loop_config.loop_track_count",
               ptr_config_->loop_config.loop_track_count, 10);
  getParameter("loop_config.opt_update_map_time",
               ptr_config_->loop_config.opt_update_map_time, 20.0);
}

void LoadParamsRos2::loadOccMapConfig() {
  getParameter("occupancy_map.laser_min_angle",
               ptr_config_->occ_map_config.laser_min_angle,
               static_cast<float>(-3.14159));
  getParameter("occupancy_map.laser_max_angle",
               ptr_config_->occ_map_config.laser_max_angle,
               static_cast<float>(3.14159));
  getParameter("occupancy_map.laser_min_range",
               ptr_config_->occ_map_config.laser_min_range,
               static_cast<float>(0.2));
  getParameter("occupancy_map.laser_max_range",
               ptr_config_->occ_map_config.laser_max_range,
               static_cast<float>(20.0));
  getParameter("occupancy_map.laser_angle_increment",
               ptr_config_->occ_map_config.laser_angle_increment,
               static_cast<float>(0.00175312));
  getParameter("occupancy_map.use_view_cloud",
               ptr_config_->occ_map_config.use_view_cloud, true);
  getParameter("occupancy_map.point_size",
               ptr_config_->occ_map_config.point_size,
               static_cast<float>(0.05));
  getParameter("occupancy_map.local_map_size",
               ptr_config_->occ_map_config.local_map_size, 20);
  getParameter("occupancy_map.dynamic_cloud_remove",
               ptr_config_->occ_map_config.dynamic_cloud_remove, false);
  getParameter("occupancy_map.local_map_resolution",
               ptr_config_->occ_map_config.local_map_resolution,
               static_cast<float>(0.1));
}

void LoadParamsRos2::loadMSFConfig() {
  getParameter("msf_config.fix_first_node",
               ptr_config_->msf_config.fix_first_node, true);
  getParameter("msf_config.new_node_size_to_opt",
               ptr_config_->msf_config.new_node_size_to_opt,
               static_cast<unsigned int>(1));
  getParameter("msf_config.opt_windows_size",
               ptr_config_->msf_config.opt_windows_size,
               static_cast<unsigned int>(10));
  getParameter("msf_config.opt_loop_count",
               ptr_config_->msf_config.opt_loop_count, 1);
  getParameter("backend_config.loop_cov", ptr_config_->msf_config.loop_cov,
               static_cast<float>(0.2));
  getParameter("backend_config.relative_cov",
               ptr_config_->msf_config.relative_cov, static_cast<float>(0.02));
}

void LoadParamsRos2::loadBackConfig() {
  getParameter("backend_config.surround_search_radius",
               ptr_config_->backend_config.surround_search_radius,
               static_cast<float>(10.0));
  getParameter("backend_config.surround_search_num",
               ptr_config_->backend_config.surround_search_num, 150);
  getParameter("backend_config.leaf_size",
               ptr_config_->backend_config.leaf_size, static_cast<float>(0.3));
  getParameter("backend_config.edge_size",
               ptr_config_->backend_config.edge_size, static_cast<float>(0.1));
  getParameter("backend_config.surf_size",
               ptr_config_->backend_config.surf_size, static_cast<float>(0.1));
  getParameter("backend_config.angle_threshold",
               ptr_config_->backend_config.angle_threshold,
               static_cast<float>(0.4));
  getParameter("backend_config.trans_threshold",
               ptr_config_->backend_config.trans_threshold,
               static_cast<float>(0.35));
  getParameter("backend_config.time_threshold",
               ptr_config_->backend_config.time_threshold,
               static_cast<float>(1.0));
  getParameter("backend_config.angular_threshold",
               ptr_config_->backend_config.angular_threshold,
               static_cast<float>(0.3));
  getParameter("backend_config.min_distance",
               ptr_config_->backend_config.min_distance,
               static_cast<float>(0.3));
  getParameter("backend_config.is_localization_mode",
               ptr_config_->backend_config.is_localization_mode, true);
  getParameter("backend_config.match_algorithm",
               ptr_config_->backend_config.match_algorithm, 1);
  getParameter("backend_config.max_match_dis",
               ptr_config_->backend_config.max_match_dis,
               static_cast<float>(0.3));
  getParameter("backend_config.max_map_match_success_score",
               ptr_config_->backend_config.max_map_match_success_score,
               static_cast<float>(0.06));
  getParameter("backend_config.min_map_match_overlap_ratio",
               ptr_config_->backend_config.min_map_match_overlap_ratio,
               static_cast<float>(0.3));
  getParameter("backend_config.mapping_match_smooth_ratio",
               ptr_config_->backend_config.mapping_match_smooth_ratio,
               static_cast<float>(1.0));
  getParameter("backend_config.loc_match_smooth_ratio",
               ptr_config_->backend_config.loc_match_smooth_ratio,
               static_cast<float>(1.0));
  getParameter("backend_config.odom_match_ratio",
               ptr_config_->backend_config.odom_match_ratio,
               static_cast<float>(1.0));
  getParameter("backend_config.lidar_odom_match_smooth_ratio",
               ptr_config_->backend_config.lidar_odom_match_smooth_ratio,
               static_cast<float>(1.0));
  getParameter("backend_config.lidar_odom_smooth_ratio",
               ptr_config_->backend_config.lidar_odom_smooth_ratio,
               static_cast<float>(1.0));
  getParameter("backend_config.fusion_pose_smooth_ratio",
               ptr_config_->backend_config.fusion_pose_smooth_ratio,
               static_cast<float>(0.01));
  getParameter("backend_config.loop_cov", ptr_config_->backend_config.loop_cov,
               static_cast<float>(0.2));
  getParameter("backend_config.relative_cov",
               ptr_config_->backend_config.relative_cov,
               static_cast<float>(0.02));
  getParameter("backend_config.use_ceres_opt",
               ptr_config_->backend_config.use_ceres_opt, true);
  getParameter("backend_config.save_keyPose_to_file",
               ptr_config_->backend_config.save_keyPose_to_file, false);
  getParameter("backend_config.max_match_failed",
               ptr_config_->backend_config.max_match_failed, 200);
  getParameter("backend_config.max_low_overlap",
               ptr_config_->backend_config.max_low_overlap, 200);
  getParameter("backend_config.map_update",
               ptr_config_->backend_config.map_update, false);
  getParameter("backend_config.increase_ratio",
               ptr_config_->backend_config.increase_ratio,
               static_cast<float>(1.0));
  getParameter("backend_config.decrease_ratio",
               ptr_config_->backend_config.decrease_ratio,
               static_cast<float>(1.0));
  getParameter("backend_config.increase_duration",
               ptr_config_->backend_config.increase_duration,
               static_cast<float>(600.0));
  getParameter("backend_config.decrease_duration",
               ptr_config_->backend_config.decrease_duration,
               static_cast<float>(3600.0));
  getParameter("backend_config.static_duration",
               ptr_config_->backend_config.static_duration,
               static_cast<float>(200.0));
  getParameter("backend_config.init_map_prob",
               ptr_config_->backend_config.init_map_prob,
               static_cast<float>(60.0));
  getParameter("backend_config.min_match_prob",
               ptr_config_->backend_config.min_match_prob,
               static_cast<float>(40.0));
  getParameter("backend_config.new_add_prob",
               ptr_config_->backend_config.new_add_prob,
               static_cast<float>(30.0));
  getParameter("backend_config.min_erase_prob",
               ptr_config_->backend_config.min_erase_prob,
               static_cast<float>(20.0));
  getParameter("backend_config.degenerate_threshold",
               ptr_config_->backend_config.degenerate_threshold,
               static_cast<float>(100.0));
  getParameter("backend_config.degenerate_lidar_odom_smooth_ratio",
               ptr_config_->backend_config.degenerate_lidar_odom_smooth_ratio,
               static_cast<float>(0.1));
  getParameter("backend_config.degenerate_loc_match_smooth_ratio",
               ptr_config_->backend_config.degenerate_loc_match_smooth_ratio,
               static_cast<float>(0.1));
}

void LoadParamsRos2::loadDepthCameraParams() {
  getParameter("depth_occupancy_map.do_calibration",
               depth_camera_options_.do_calibration, false);
  bool read_from_params_system = false;
  getParameter("depth_occupancy_map.read_from_params_system",
               read_from_params_system, read_from_params_system);
  bool flag = false;
  if (read_from_params_system) {
    LOG(INFO) << "read camera params from parameter system.";
    std::string front_up_transform_name;
    std::string front_down_transform_name;
    int count = 0;
    getParameter("occupancy_map.front_up_transform_name",
                 front_up_transform_name, front_up_transform_name);
    getParameter("occupancy_map.front_down_transform_name",
                 front_down_transform_name, front_down_transform_name);

    PublicParameters public_parameters;
    // 判断参数读取是否正确
    if (public_parameters.getParameter(
            front_up_transform_name, depth_camera_options_.front_up_transform,
            depth_camera_options_.front_up_transform) &&
        public_parameters.getParameter(
            front_down_transform_name,
            depth_camera_options_.front_down_transform,
            depth_camera_options_.front_down_transform)) {
      flag = true;
    }
  }

  if (!flag) {
    LOG(WARNING) << "using camera params set in yaml.";
    node_ptr_->get_parameter_or("depth_occupancy_map.front_up_transform",
                                depth_camera_options_.front_up_transform,
                                depth_camera_options_.front_up_transform);
    node_ptr_->get_parameter_or("depth_occupancy_map.front_down_transform",
                                depth_camera_options_.front_down_transform,
                                depth_camera_options_.front_down_transform);
  }
  LOG(WARNING) << "camera front up transfrom: \n"
               << depth_camera_options_.front_up_transform[0] << " "
               << depth_camera_options_.front_up_transform[1] << " "
               << depth_camera_options_.front_up_transform[2] << " "
               << depth_camera_options_.front_up_transform[3] << " "
               << depth_camera_options_.front_up_transform[4] << " "
               << depth_camera_options_.front_up_transform[5] << " "
               << depth_camera_options_.front_up_transform[6] << " "
               << depth_camera_options_.front_up_transform[7] << " "
               << depth_camera_options_.front_up_transform[8] << " "
               << depth_camera_options_.front_up_transform[9] << " "
               << depth_camera_options_.front_up_transform[10] << " "
               << depth_camera_options_.front_up_transform[11] << " "
               << depth_camera_options_.front_up_transform[12] << " "
               << depth_camera_options_.front_up_transform[13] << " "
               << depth_camera_options_.front_up_transform[14] << " "
               << depth_camera_options_.front_up_transform[15];

  LOG(WARNING) << "camera front down transfrom: \n"
               << depth_camera_options_.front_down_transform[0] << " "
               << depth_camera_options_.front_down_transform[1] << " "
               << depth_camera_options_.front_down_transform[2] << " "
               << depth_camera_options_.front_down_transform[3] << " "
               << depth_camera_options_.front_down_transform[4] << " "
               << depth_camera_options_.front_down_transform[5] << " "
               << depth_camera_options_.front_down_transform[6] << " "
               << depth_camera_options_.front_down_transform[7] << " "
               << depth_camera_options_.front_down_transform[8] << " "
               << depth_camera_options_.front_down_transform[9] << " "
               << depth_camera_options_.front_down_transform[10] << " "
               << depth_camera_options_.front_down_transform[11] << " "
               << depth_camera_options_.front_down_transform[12] << " "
               << depth_camera_options_.front_down_transform[13] << " "
               << depth_camera_options_.front_down_transform[14] << " "
               << depth_camera_options_.front_down_transform[15];

  node_ptr_->get_parameter_or("depth_occupancy_map.search_area_point",
                              depth_occupancy_map_options_.search_area_point,
                              depth_occupancy_map_options_.search_area_point);
  LOG(WARNING) << "depth_occupancy_map search_area: \n"
               << depth_occupancy_map_options_.search_area_point[0] << " "
               << depth_occupancy_map_options_.search_area_point[1] << " "
               << depth_occupancy_map_options_.search_area_point[2] << " "
               << depth_occupancy_map_options_.search_area_point[3] << " "
               << depth_occupancy_map_options_.search_area_point[4] << " "
               << depth_occupancy_map_options_.search_area_point[5] << " "
               << depth_occupancy_map_options_.search_area_point[6] << " "
               << depth_occupancy_map_options_.search_area_point[7];

  getParameter("depth_occupancy_map.add_bel_value",
               depth_occupancy_map_options_.add_bel_value,
               depth_occupancy_map_options_.add_bel_value);
  getParameter("depth_occupancy_map.reduce_bel_value",
               depth_occupancy_map_options_.reduce_bel_value,
               depth_occupancy_map_options_.reduce_bel_value);
  getParameter("depth_occupancy_map.max_bel_value",
               depth_occupancy_map_options_.max_bel_value,
               depth_occupancy_map_options_.max_bel_value);

  getParameter("depth_occupancy_map.v_limit", depth_camera_options_.occ_v_limit,
               0.01);
  getParameter("depth_occupancy_map.w_limit", depth_camera_options_.occ_w_limit,
               0.01);
  getParameter("depth_occupancy_map.depth_dist_value",
               depth_camera_options_.depth_dist_value, 2.3);
  getParameter("depth_occupancy_map.depth_seg_ratio",
               depth_camera_options_.depth_seg_ratio, 0.7);
  getParameter("depth_occupancy_map.high_filter_value",
               depth_camera_options_.high_filter_value, 0.03);
  getParameter("depth_occupancy_map.high_filter_value",
               depth_camera_options_.high_filter_value, 0.03);

  getParameter("depth_occupancy_map.laser_min_angle",
               depth_occupancy_map_options_.laser_min_angle,
               depth_occupancy_map_options_.laser_min_angle);
  getParameter("depth_occupancy_map.laser_max_angle",
               depth_occupancy_map_options_.laser_max_angle,
               depth_occupancy_map_options_.laser_max_angle);
  getParameter("depth_occupancy_map.laser_min_range",
               depth_occupancy_map_options_.laser_min_range,
               depth_occupancy_map_options_.laser_min_range);
  getParameter("depth_occupancy_map.laser_max_range",
               depth_occupancy_map_options_.laser_max_range,
               depth_occupancy_map_options_.laser_max_range);
  getParameter("depth_occupancy_map.laser_angle_increment",
               depth_occupancy_map_options_.laser_angle_increment,
               depth_occupancy_map_options_.laser_angle_increment);
  getParameter("depth_occupancy_map.use_view_cloud",
               depth_occupancy_map_options_.use_view_cloud,
               depth_occupancy_map_options_.use_view_cloud);
}

template <typename ParameterT>
void LoadParamsRos2::getParameter(const std::string &name, ParameterT &value,
                                  const ParameterT &alternative_value) const {
  if (node_ptr_->get_parameter_or(name, value, alternative_value)) {
    LOG(INFO) << "Read param: " << name << " value:" << value;
  } else {
    LOG(WARNING) << "Get param failed. param:" << name
                 << " default value:" << value;
  }
}
}  // namespace cvte_lidar_slam
