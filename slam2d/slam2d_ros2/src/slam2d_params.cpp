/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file slam2d_params.cpp
 *
 *@brief
 * slam2d_core相关参数配置类的具体实现.
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version v1.0
 *@data 2021-04-01
 ************************************************************************/
#include "slam2d_params.hpp"
#include "public_parameters/PublicParameters.hpp"

namespace slam2d_ros2 {

Slam2dParamsRos2::Slam2dParamsRos2(const rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_(node_ptr) {}
Slam2dParamsRos2::~Slam2dParamsRos2() {}

void Slam2dParamsRos2::loadLocalTrajectoryBuilderParams() {
  getParameter("local_trajectory_builder.min_range",
               local_trajectory_options_.min_range, static_cast<float>(0.0));
  getParameter("local_trajectory_builder.max_range",
               local_trajectory_options_.max_range, static_cast<float>(30.0));
  getParameter("local_trajectory_builder.voxel_filter_size",
               local_trajectory_options_.voxel_filter_size,
               static_cast<float>(0.05));
  getParameter("local_trajectory_builder.missing_data_ray_length",
               local_trajectory_options_.missing_data_ray_length,
               static_cast<float>(5.0));
  getParameter("local_trajectory_builder.use_online_correlative_scan_matching",
               local_trajectory_options_.use_online_correlative_scan_matching,
               false);

  getParameter(
      "local_trajectory_builder.adaptive_voxel_filter_options.max_range",
      local_trajectory_options_.adaptive_voxel_filter_options.max_range,
      static_cast<float>(30.0));
  getParameter(
      "local_trajectory_builder.adaptive_voxel_filter_options.max_length",
      local_trajectory_options_.adaptive_voxel_filter_options.max_length,
      static_cast<float>(1.0));
  getParameter(
      "local_trajectory_builder.adaptive_voxel_filter_options.min_num_points",
      local_trajectory_options_.adaptive_voxel_filter_options.min_num_points,
      static_cast<float>(100.0));

  getParameter("local_trajectory_builder.submaps_options.resolution",
               local_trajectory_options_.submaps_options.resolution,
               static_cast<float>(0.05));
  getParameter("local_trajectory_builder.submaps_options.num_range_data",
               local_trajectory_options_.submaps_options.num_range_data, 90);

  getParameter(
      "local_trajectory_builder.submaps_options.probability_grid_range_data_"
      "options.hit_probability",
      local_trajectory_options_.submaps_options
          .probability_grid_range_data_inserter_options.hit_probability,
      0.55);
  getParameter(
      "local_trajectory_builder.submaps_options.probability_grid_range_data_"
      "options.miss_probability",
      local_trajectory_options_.submaps_options
          .probability_grid_range_data_inserter_options.miss_probability,
      0.49);
  getParameter(
      "local_trajectory_builder.submaps_options.probability_grid_range_data_"
      "options.insert_free_space",
      local_trajectory_options_.submaps_options
          .probability_grid_range_data_inserter_options.insert_free_space,
      true);

  getParameter(
      "local_trajectory_builder.motion_filter_options.max_time_seconds",
      local_trajectory_options_.motion_filter_options.max_time_seconds, 5.0);
  getParameter(
      "local_trajectory_builder.motion_filter_options.max_angle_radians",
      local_trajectory_options_.motion_filter_options.max_angle_radians,
      (M_PI / 180.0));
  getParameter(
      "local_trajectory_builder.motion_filter_options.max_distance_meters",
      local_trajectory_options_.motion_filter_options.max_distance_meters, 0.2);

  getParameter(
      "local_trajectory_builder.real_time_correlative_scan_matcher.linear_"
      "search_window",
      local_trajectory_options_.real_time_correlative_scan_matcher_options
          .linear_search_window,
      0.1);
  getParameter(
      "local_trajectory_builder.real_time_correlative_scan_matcher.angular_"
      "search_window",
      local_trajectory_options_.real_time_correlative_scan_matcher_options
          .angular_search_window,
      (M_PI / 9.0));
  getParameter(
      "local_trajectory_builder.real_time_correlative_scan_matcher.rotation_"
      "delta_cost_weight",
      local_trajectory_options_.real_time_correlative_scan_matcher_options
          .rotation_delta_cost_weight,
      1e-1);
  getParameter(
      "local_trajectory_builder.real_time_correlative_scan_matcher.translation_"
      "delta_cost_weight",
      local_trajectory_options_.real_time_correlative_scan_matcher_options
          .translation_delta_cost_weight,
      10.0);

  getParameter(
      "local_trajectory_builder.ceres_scan_matcher_options.rotation_weight",
      local_trajectory_options_.ceres_scan_matcher_options.rotation_weight,
      40.0);
  getParameter(
      "local_trajectory_builder.ceres_scan_matcher_options.translation_weight",
      local_trajectory_options_.ceres_scan_matcher_options.translation_weight,
      10.0);
  getParameter(
      "local_trajectory_builder.ceres_scan_matcher_options.occupied_space_"
      "weight",
      local_trajectory_options_.ceres_scan_matcher_options
          .occupied_space_weight,
      1.0);
  getParameter(
      "local_trajectory_builder.ceres_scan_matcher_options.ceres_solver_"
      "options.num_threads",
      local_trajectory_options_.ceres_scan_matcher_options.ceres_solver_options
          .num_threads,
      1);
  getParameter(
      "local_trajectory_builder.ceres_scan_matcher_options.ceres_solver_"
      "options.max_num_iterations",
      local_trajectory_options_.ceres_scan_matcher_options.ceres_solver_options
          .max_num_iterations,
      20);
  getParameter(
      "local_trajectory_builder.ceres_scan_matcher_options.ceres_solver_"
      "options.use_nonmonotonic_steps",
      local_trajectory_options_.ceres_scan_matcher_options.ceres_solver_options
          .use_nonmonotonic_steps,
      false);
}

void Slam2dParamsRos2::loadPoseGraphParams() {
  getParameter("pose_graph_options.global_sampling_ratio",
               pose_graph_options_.global_sampling_ratio, 0.003);
  getParameter("pose_graph_options.optimize_every_n_nodes",
               pose_graph_options_.optimize_every_n_nodes, 90);
  getParameter("pose_graph_options.num_background_threads",
               pose_graph_options_.num_background_threads, 4);
  getParameter("pose_graph_options.log_residual_histograms",
               pose_graph_options_.log_residual_histograms, true);
  getParameter("pose_graph_options.max_num_final_iterations",
               pose_graph_options_.max_num_final_iterations, 200);
  getParameter("pose_graph_options.matcher_rotation_weight",
               pose_graph_options_.matcher_rotation_weight, 1.6e3);
  getParameter("pose_graph_options.matcher_translation_weight",
               pose_graph_options_.matcher_translation_weight, 5e2);
  getParameter("pose_graph_options.global_constraint_search_after_n_seconds",
               pose_graph_options_.global_constraint_search_after_n_seconds,
               10.0);

  getParameter("pose_graph_options.constraint_builder_options.min_score",
               pose_graph_options_.constraint_builder_options.min_score, 0.55);
  getParameter("pose_graph_options.constraint_builder_options.log_matches",
               pose_graph_options_.constraint_builder_options.log_matches,
               true);
  getParameter("pose_graph_options.constraint_builder_options.sampling_ratio",
               pose_graph_options_.constraint_builder_options.sampling_ratio,
               0.3);
  getParameter(
      "pose_graph_options.constraint_builder_options.max_constraint_distance",
      pose_graph_options_.constraint_builder_options.max_constraint_distance,
      15.0);
  getParameter(
      "pose_graph_options.constraint_builder_options."
      "global_localization_min_score",
      pose_graph_options_.constraint_builder_options
          .global_localization_min_score,
      0.6);
  getParameter(
      "pose_graph_options.constraint_builder_options."
      "loop_closure_rotation_weight",
      pose_graph_options_.constraint_builder_options
          .loop_closure_rotation_weight,
      1e5);
  getParameter(
      "pose_graph_options.constraint_builder_options."
      "loop_closure_translation_weight",
      pose_graph_options_.constraint_builder_options
          .loop_closure_translation_weight,
      1.1e4);
  getParameter(
      "pose_graph_options.constraint_builder_options."
      "loop_closure_error_ratio",
      pose_graph_options_.constraint_builder_options.loop_closure_error_ratio,
      0.1);
  getParameter(
      "pose_graph_options.constraint_builder_options."
      "ceres_scan_matcher.rotation_weight",
      pose_graph_options_.constraint_builder_options.ceres_scan_matcher_options
          .rotation_weight,
      1.0);

  getParameter(
      "pose_graph_options.constraint_builder_options.ceres_scan_matcher."
      "translation_weight",
      pose_graph_options_.constraint_builder_options.ceres_scan_matcher_options
          .translation_weight,
      10.0);

  getParameter(
      "pose_graph_options.constraint_builder_options.ceres_scan_matcher."
      "occupied_space_weight",
      pose_graph_options_.constraint_builder_options.ceres_scan_matcher_options
          .occupied_space_weight,
      20.0);

  getParameter(
      "pose_graph_options.constraint_builder_options.ceres_scan_matcher.ceres_"
      "solver.num_threads",
      pose_graph_options_.constraint_builder_options.ceres_scan_matcher_options
          .ceres_solver_options.num_threads,
      1);

  getParameter(
      "pose_graph_options.constraint_builder_options.ceres_scan_matcher.ceres_"
      "solver.max_num_iterations",
      pose_graph_options_.constraint_builder_options.ceres_scan_matcher_options
          .ceres_solver_options.max_num_iterations,
      10);

  getParameter(
      "pose_graph_options.constraint_builder_options.ceres_scan_matcher.ceres_"
      "solver.use_nonmonotonic_steps",
      pose_graph_options_.constraint_builder_options.ceres_scan_matcher_options
          .ceres_solver_options.use_nonmonotonic_steps,
      true);

  getParameter(
      "pose_graph_options.constraint_builder_options.fast_correlative_scan_"
      "matcher.linear_search_window",
      pose_graph_options_.constraint_builder_options
          .fast_correlative_scan_matcher_options.linear_search_window,
      7.0);

  getParameter(
      "pose_graph_options.constraint_builder_options.fast_correlative_scan_"
      "matcher.angular_search_window",
      pose_graph_options_.constraint_builder_options
          .fast_correlative_scan_matcher_options.angular_search_window,
      (M_PI / 6.0));

  getParameter(
      "pose_graph_options.constraint_builder_options.fast_correlative_scan_"
      "matcher.branch_and_bound_depth",
      pose_graph_options_.constraint_builder_options
          .fast_correlative_scan_matcher_options.branch_and_bound_depth,
      7);

  getParameter("pose_graph_options.optimization_options.huber_scale",
               pose_graph_options_.optimization_problem_options.huber_scale,
               1e1);

  getParameter("pose_graph_options.optimization_options.rotation_weight",
               pose_graph_options_.optimization_problem_options.rotation_weight,
               1.6e4);

  getParameter(
      "pose_graph_options.optimization_options.acceleration_weight",
      pose_graph_options_.optimization_problem_options.acceleration_weight,
      1.1e2);

  getParameter("pose_graph_options.optimization_options.use_odom",
               pose_graph_options_.optimization_problem_options.use_odom, true);

  getParameter("pose_graph_options.optimization_options.use_local_pose",
               pose_graph_options_.optimization_problem_options.use_local_pose,
               true);

  getParameter(
      "pose_graph_options.optimization_options.log_solver_summary",
      pose_graph_options_.optimization_problem_options.log_solver_summary,
      false);

  getParameter(
      "pose_graph_options.optimization_options.odometry_rotation_weight",
      pose_graph_options_.optimization_problem_options.odometry_rotation_weight,
      1e5);

  getParameter(
      "pose_graph_options.optimization_options.odometry_translation_weight",
      pose_graph_options_.optimization_problem_options
          .odometry_translation_weight,
      1e5);

  getParameter(
      "pose_graph_options.optimization_options.local_slam_pose_rotation_weight",
      pose_graph_options_.optimization_problem_options
          .local_slam_pose_rotation_weight,
      1e5);

  getParameter(
      "pose_graph_options.optimization_options.local_"
      "slam_pose_translation_weight",
      pose_graph_options_.optimization_problem_options
          .local_slam_pose_translation_weight,
      1e5);

  getParameter(
      "pose_graph_options.optimization_options.gyro_yaw_rotation_weight",
      pose_graph_options_.optimization_problem_options.gyro_yaw_rotation_weight,
      1e6);
  getParameter(
      "pose_graph_options.optimization_options.gyro_yaw_bias_vary_weight",
      pose_graph_options_.optimization_problem_options
          .gyro_yaw_bias_vary_weight,
      1e7);
  getParameter(
      "pose_graph_options.optimization_options.gyro_yaw_bias_prior",
      pose_graph_options_.optimization_problem_options.gyro_yaw_bias_prior, 0.);
  getParameter(
      "pose_graph_options.optimization_options.gyro_yaw_bias_prior_weight",
      pose_graph_options_.optimization_problem_options
          .gyro_yaw_bias_prior_weight,
      1e7);
  getParameter(
      "pose_graph_options.optimization_options.num_node_one_segment",
      pose_graph_options_.optimization_problem_options.num_node_one_segment,
      300);

  getParameter(
      "pose_graph_options.optimization_options.ceres_solver.num_"
      "threads",
      pose_graph_options_.optimization_problem_options.ceres_solver_options
          .num_threads,
      7);

  getParameter(
      "pose_graph_options.optimization_options.ceres_"
      "solver.max_num_iterations",
      pose_graph_options_.optimization_problem_options.ceres_solver_options
          .max_num_iterations,
      50);

  getParameter(
      "pose_graph_options.optimization_options.ceres_"
      "solver.use_nonmonotonic_steps",
      pose_graph_options_.optimization_problem_options.ceres_solver_options
          .use_nonmonotonic_steps,
      false);

  getParameter("pose_graph_options.sampling_ratio_options.scan_sampling_ratio",
               pose_graph_options_.scan_sampling_ratio, 0.3);
  getParameter(
      "pose_graph_options.sampling_ratio_options.odometry_sampling_ratio",
      pose_graph_options_.odometry_sampling_ratio, 0.3);
  getParameter("pose_graph_options.sampling_ratio_options.imu_sampling_ratio",
               pose_graph_options_.imu_sampling_ratio, 1.0);
}

void Slam2dParamsRos2::loadLocalizationParams() {
  getParameter("loc.init_score_min", loc_options.init_score_min, 0.35);
  getParameter("loc.score_min_threshold", loc_options.score_min_threshold,
               0.15);
  getParameter("loc.loc_recovery_score_min", loc_options.loc_recovery_score_min,
               0.9);
  getParameter("loc.inner_point_dist_threshold",
               loc_options.inner_point_dist_threshold, 0.3);
  getParameter("loc.predict_deque_length", loc_options.predict_deque_length,
               10.0);
  getParameter("loc.predict_deque_angle", loc_options.predict_deque_angle,
               10.0);
  getParameter("loc.predict_to_observe_length_threshold",
               loc_options.predict_to_observe_length_threshold, 0.25);
  getParameter("loc.predict_to_observe_angle_threshold",
               loc_options.predict_to_observe_angle_threshold, 0.4);
  getParameter("loc.use_fast_loo", loc_options.use_fast_loo, false);
  getParameter("loc.amcl_reinit_pose_score", loc_options.amcl_reinit_pose_score,
               0.5);
  getParameter("loc.use_laserodom_constrain",
               loc_options.use_laserodom_constrain, true);
  getParameter("loc.odometry_data_cov", loc_options.odometry_data_cov, 100.0);
  getParameter("loc.laserodom_data_cov", loc_options.laserodom_data_cov, 1.0);
  getParameter("loc.init_localizaiton_map_range",
               loc_options.init_localizaiton_map_range, 50.0);
  getParameter("loc.laserodom_correct_threshold",
               loc_options.laserodom_correct_threshold, 0.3);
}

void Slam2dParamsRos2::loadAmclParams() {
  getParameter("amcl.map_path", amcl_options_.map_path,
               std::string("/home/droid/Development/robot_data"));
  getParameter("amcl.max_occ_dist", amcl_options_.max_occ_dist, 0.5);
  getParameter("amcl.init_max_samples", amcl_options_.init_max_samples,
               static_cast<unsigned int>(5000));
  getParameter("amcl.init_min_samples", amcl_options_.init_min_samples,
               static_cast<unsigned int>(100));
  getParameter("amcl.min_samples", amcl_options_.min_samples,
               static_cast<unsigned int>(100));
  getParameter("amcl.max_samples", amcl_options_.max_samples,
               static_cast<unsigned int>(1000));
  getParameter("amcl.alpha_slow", amcl_options_.alpha_slow, 0.0);
  getParameter("amcl.alpha_fast", amcl_options_.alpha_fast, 0.0);
  getParameter("amcl.pop_err", amcl_options_.pop_err, 0.01);
  getParameter("amcl.pop_z", amcl_options_.pop_z, 0.7);
  getParameter("amcl.dist_treshold", amcl_options_.dist_treshold, 1.0);

  getParameter("amcl.alpha1_", amcl_options_.alpha1_, 0.2);
  getParameter("amcl.alpha2_", amcl_options_.alpha2_, 0.2);
  getParameter("amcl.alpha3_", amcl_options_.alpha3_, 0.2);
  getParameter("amcl.alpha4_", amcl_options_.alpha4_, 0.2);
  getParameter("amcl.alpha5_", amcl_options_.alpha5_, 0.2);

  getParameter("amcl.cell_size_x_", amcl_options_.cell_size_x_, 0.5);
  getParameter("amcl.cell_size_y_", amcl_options_.cell_size_y_, 0.5);
  getParameter("amcl.cell_size_yaw_", amcl_options_.cell_size_yaw_, 0.5);

  getParameter("amcl.max_occ_dist_", amcl_options_.max_occ_dist_, 0.5);
  getParameter("amcl.do_beamskip_", amcl_options_.do_beamskip_, false);
  getParameter("amcl.beam_skip_distance_", amcl_options_.beam_skip_distance_,
               0.5);
  getParameter("amcl.beam_skip_threshold_", amcl_options_.beam_skip_threshold_,
               0.3);
  getParameter("amcl.beam_skip_error_threshold_",
               amcl_options_.beam_skip_error_threshold_, 0.9);

  getParameter("amcl.z_hit_", amcl_options_.z_hit_, 0.5);
  getParameter("amcl.z_rand_", amcl_options_.z_rand_, 0.5);
  getParameter("amcl.sigma_hit_", amcl_options_.sigma_hit_, 0.08);
  getParameter("amcl.z_short_", amcl_options_.z_short_, 0.05);
  getParameter("amcl.z_max_", amcl_options_.z_max_, 0.05);
  getParameter("amcl.lambda_short_", amcl_options_.lambda_short_, 0.1);

  getParameter("amcl.chi_outlier_", amcl_options_.chi_outlier_, 0.0);
  getParameter("amcl.max_beams_", amcl_options_.max_beams_,
               static_cast<unsigned int>(181));

  getParameter("amcl.laser_pose_x_", amcl_options_.laser_pose_x_, 0.13);
  getParameter("amcl.laser_pose_y_", amcl_options_.laser_pose_y_, 0.0);
  getParameter("amcl.laser_pose_yaw_", amcl_options_.laser_pose_yaw_, 0.0);

  getParameter("amcl.update_trans_thr_", amcl_options_.update_trans_thr_, 0.2);

  getParameter("amcl.update_angle_thr_", amcl_options_.update_angle_thr_, 0.2);
  getParameter("amcl.update_trans_unormal_thr_",
               amcl_options_.update_trans_unormal_thr_, 0.6);
  getParameter("amcl.update_angle_unormal_thr_",
               amcl_options_.update_angle_unormal_thr_, 0.5);

  getParameter("amcl.resample_interval_", amcl_options_.resample_interval_,
               static_cast<unsigned int>(1));
}

void Slam2dParamsRos2::loadMSFParams() {
  getParameter("msf.do_node_merge", msf_options_.do_node_merge, true);
  getParameter("msf.fix_first_node", msf_options_.fix_first_node, true);

  getParameter("msf.do_time_synchronization",
               msf_options_.do_time_synchronization, true);
  getParameter("msf.do_time_check", msf_options_.do_time_check, false);

  getParameter("msf.compute_cov", msf_options_.compute_cov, true);
  getParameter("msf.WINDOW_SIZE", msf_options_.WINDOW_SIZE,
               static_cast<size_t>(50));
  getParameter("msf.bad_match_node_number_",
               msf_options_.bad_match_node_number_, 100);
}

void Slam2dParamsRos2::loadGSMParams() {
  getParameter("gsm.do_global_search", gsm_options_.do_global_search, true);
  getParameter("gsm.do_window_search", gsm_options_.do_window_search, true);
  getParameter("gsm.num_map_depth", gsm_options_.num_map_depth, 6);
  getParameter("gsm.min_score", gsm_options_.min_score, 0.4f);
  getParameter("gsm.search_angle_resolution",
               gsm_options_.search_angle_resolution, 0.104667f);
  getParameter("gsm.search_windows_x_width",
               gsm_options_.search_windows_x_width, 10.0f);
  getParameter("gsm.search_windows_y_width",
               gsm_options_.search_windows_y_width, 10.0f);
  getParameter("gsm.search_windows_angle", gsm_options_.search_windows_angle,
               180.0f);
}

void Slam2dParamsRos2::loadDepthCameraParams() {
  node_ptr_->get_parameter_or("occupancy_map.do_calibration",
                              depth_camera_options_.do_calibration, false);
  bool read_from_params_system = false;
  node_ptr_->get_parameter_or("occupancy_map.read_from_params_system",
                              read_from_params_system, read_from_params_system);
  bool flag = false;
  if (read_from_params_system) {
    LOG(INFO) << "read camera params from parameter system.";
    std::string front_up_transform_name;
    std::string front_down_transform_name;
    int count = 0;
    node_ptr_->get_parameter_or("occupancy_map.front_up_transform_name",
                                front_up_transform_name,
                                front_up_transform_name);
    node_ptr_->get_parameter_or("occupancy_map.front_down_transform_name",
                                front_down_transform_name,
                                front_down_transform_name);

    PublicParameters public_parameters;
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
    LOG(INFO) << "using camera params set in yaml.";
    node_ptr_->get_parameter_or("occupancy_map.front_up_transform",
                                depth_camera_options_.front_up_transform,
                                depth_camera_options_.front_up_transform);
    node_ptr_->get_parameter_or("occupancy_map.front_down_transform",
                                depth_camera_options_.front_down_transform,
                                depth_camera_options_.front_down_transform);
  }
  LOG(INFO) << "camera front up transfrom: "
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

  LOG(INFO) << "camera front down transfrom: "
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

  node_ptr_->get_parameter_or("occupancy_map.search_area_point",
                              depth_occupancy_map_options_.search_area_point,
                              depth_occupancy_map_options_.search_area_point);
  node_ptr_->get_parameter_or("occupancy_map.add_bel_value",
                              depth_occupancy_map_options_.add_bel_value,
                              depth_occupancy_map_options_.add_bel_value);
  node_ptr_->get_parameter_or("occupancy_map.reduce_bel_value",
                              depth_occupancy_map_options_.reduce_bel_value,
                              depth_occupancy_map_options_.reduce_bel_value);
  node_ptr_->get_parameter_or("occupancy_map.max_bel_value",
                              depth_occupancy_map_options_.max_bel_value,
                              depth_occupancy_map_options_.max_bel_value);

  node_ptr_->get_parameter_or("occupancy_map.v_limit",
                              depth_camera_options_.occ_v_limit, 0.01);
  node_ptr_->get_parameter_or("occupancy_map.w_limit",
                              depth_camera_options_.occ_w_limit, 0.01);
  node_ptr_->get_parameter_or("occupancy_map.depth_dist_value",
                              depth_camera_options_.depth_dist_value, 2.3);
  node_ptr_->get_parameter_or("occupancy_map.depth_seg_ratio",
                              depth_camera_options_.depth_seg_ratio, 0.7);
  node_ptr_->get_parameter_or("occupancy_map.high_filter_value",
                              depth_camera_options_.high_filter_value, 0.03);

  node_ptr_->get_parameter_or("ros2_params.use_depth_camera",
                              depth_camera_options_.use_depth_camera, false);
}

template <typename ParameterT>
void Slam2dParamsRos2::getParameter(const std::string &name, ParameterT &value,
                                    const ParameterT &alternative_value) const {
  if (node_ptr_->get_parameter_or(name, value, alternative_value)) {
    LOG(INFO) << "Read param: " << name << " value:" << value;
  } else {
    LOG(WARNING) << "Get param failed. param:" << name
                 << " default value:" << value;
  }
}

}  // namespace slam2d_ros2
