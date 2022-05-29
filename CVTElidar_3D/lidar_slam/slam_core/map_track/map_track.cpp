/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file map_track.cpp
 *
 *@brief
 * 1.后端MAP_TRACK类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@version 1.0
 *@data 2019-11-04
 ************************************************************************/

#include "map_track/map_track.hpp"
#include "map_track/pose_solver.hpp"

#include <glog/logging.h>

#include "map/map_manager.hpp"
#include "msf/pose_graph_error_term.hpp"
#include "common/math_base/slam_transform.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/config/system_config.hpp"
#include "common/debug_tools/tic_toc.h"

namespace cvte_lidar_slam {
MapTrack::MapTrack(BackendConfig *config) : config_(config) {
  InitParameters();
  ptr_state_machine_ = SlamStateMachine::getInstance();

  if (!config_->use_ceres_opt) {
    loss_function_ = std::make_shared<HuberLoss>(0.1);
    pose_solver_ = std::make_shared<PoseSolver>();
  }
}

MapTrack::~MapTrack() {}

void MapTrack::Reset() {
  match_failed_count_ = 0;
  ds_corner_from_map_num_ = 0;
  ds_surf_from_map_num_ = 0;
  cur_corner_cloud_num_ = 0;
  cur_surf_cloud_num_ = 0;
  pose_cov_ = 10.;
  predict_error_ratio_ = 0.0;
  last_frame_time_ = -1.;

  last_frame_pose_ = Mathbox::Identity34();
  last_keyframe_pose_ = Mathbox::Identity34();
  last_frame_odom_pose_ = Mathbox::Identity34();
  last_keyframe_odom_pose_ = Mathbox::Identity34();
  map_to_odom_ = Mathbox::Identity34();
  cur_frame_pose_ = Mathbox::Identity34();
  keyframe_count_ = 0;
  is_first_keyframe_ = true;
  is_accept_keyframes_ = true;
  is_match_ok_ = true;
  is_stopped_ = false;
  is_stop_requested_ = false;
  is_map_ready_ = false;
  is_local_map_ready_ = false;
  is_global_map_ready_ = false;
  is_set_init_pose_ = false;
  is_kdtree_initialized_ = false;
}

void MapTrack::InitParameters() {
  ptr_cur_keyframe_ = std::make_shared<KeyFrame>();
  ptr_map_manager_ = MapManager::getInstance();

  ptr_kdtree_surf_from_map_.reset(new pclKdTree());
  ptr_kdtree_corner_from_map_.reset(new pclKdTree());

  corner_cloud_from_map_.reset(new laserCloud());
  surf_cloud_from_map_.reset(new laserCloud());
  ds_corner_cloud_from_map_.reset(new laserCloud());
  ds_surf_cloud_from_map_.reset(new laserCloud());
  ds_cur_corner_cloud_.reset(new laserCloud());
  ds_cur_surf_cloud_.reset(new laserCloud());
  corner_cloud_from_map_copy_.reset(new laserCloud());
  surf_cloud_from_map_copy_.reset(new laserCloud());

  downsize_filter_corner_.setLeafSize(0.2, 0.2, 0.2);
  downsize_filter_surf_.setLeafSize(0.4, 0.4, 0.4);
  downsize_filter_cur_corner_.setLeafSize(0.2, 0.2, 0.2);
  downsize_filter_cur_surf_.setLeafSize(0.4, 0.4, 0.4);

  Reset();
}

bool MapTrack::processNewKeyFrame(std::shared_ptr<KeyFrame> ptr_cur_keyframe,
                                  bool is_update_map) {
  if (nullptr == ptr_cur_keyframe) {
    return false;
  }

  ptr_cur_keyframe_ = ptr_cur_keyframe;
  if (dowmSampleCurFrame()) {
    if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode() &&
        is_first_keyframe_) {
      is_first_keyframe_ = false;
      LOG(WARNING) << "Insert first keyframe ";
      return true;
    }
    cov_ = Mat6d::Identity();
    if (++keyframe_count_ % 5 == 1 || is_update_map == true) {
      if (!getSurroundingKeyFrames()) {
        LOG(ERROR) << "Get surrounding keyframes failed";
        return false;
      }
    }

    scanToLocalMap();

    LOG(INFO) << "distance error " << distance_error_;

    cov_.block<3, 3>(0, 0) = 0.4 * distance_error_ * Mat3d::Identity();
    cov_.block<3, 3>(3, 3) = distance_error_ * Mat3d::Identity();

    if (is_match_ok_) {
      match_failed_count_ = 0;
      updateTransform();
    } else {
      match_failed_count_++;
      ptr_cur_keyframe_->laser_odom_cov_ = Mat6d::Identity();
      ptr_cur_keyframe_->map_cov_ =
          (0.1 * match_failed_count_ + 0.5) * Mat6d::Identity();
      LOG(WARNING) << "match failed.  " << match_failed_count_;
    }
    pose_cov_ = ptr_cur_keyframe_->map_cov_(3, 3);  //
    // LOG(INFO) << "Pose cov:  " << pose_cov_;
    if (pose_cov_ > 3) {
      LOG(WARNING) << "Unstable pose  " << pose_cov_;
    }
    clearCloud();
  }
  // LOG(INFO) << "per frame match time cost  " << frame_cost_time.toc()
  //           << "   ms\n\n"
  //           << std::endl;
  return is_match_ok_;
}

bool MapTrack::getSurroundingKeyFrames() {
  TicToc get_surround;
  auto cur_frame_pos = ptr_cur_keyframe_->getPosition();
  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    setMapStatus(true);
    return is_kdtree_initialized_;
  } else if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    std::vector<std::shared_ptr<KeyFrame>> surroundingKeyFrames;
    if (config_->is_search_by_time) {
      surroundingKeyFrames = ptr_map_manager_->DetectCandidatesByTime(
          config_->surround_search_num);
    } else {
      surroundingKeyFrames = ptr_map_manager_->DetectCandidatesByDistance(
          cur_frame_pos, config_->surround_search_radius);
    }

    if (surroundingKeyFrames.empty()) {
      LOG(ERROR) << "surround keyframes Empty. ";
      return false;
    }
    int surroundingKeyFramesNums = surroundingKeyFrames.size();

    for (int i = 0; i < surroundingKeyFramesNums; i++) {
      Mat34d it_frame_pose = surroundingKeyFrames[i]->getPose();

      laserCloud::Ptr surround_corner_cloud = Transformbox::transformPointCloud(
          it_frame_pose, surroundingKeyFrames[i]->corner_cloud_);
      laserCloud::Ptr surround_surf_cloud = Transformbox::transformPointCloud(
          it_frame_pose, surroundingKeyFrames[i]->surf_cloud_);
      *corner_cloud_from_map_ += *surround_corner_cloud;
      *surf_cloud_from_map_ += *surround_surf_cloud;
    }
    // 下采样
    // Downsample the surrounding corner key frames (or map)
    downsize_filter_corner_.setInputCloud(corner_cloud_from_map_);
    downsize_filter_corner_.filter(*ds_corner_cloud_from_map_);
    ds_corner_from_map_num_ = ds_corner_cloud_from_map_->points.size();
    // Downsample the surrounding surf key frames (or map)
    downsize_filter_surf_.setInputCloud(surf_cloud_from_map_);
    downsize_filter_surf_.filter(*ds_surf_cloud_from_map_);
    ds_surf_from_map_num_ = ds_surf_cloud_from_map_->points.size();
    setMapStatus(false);
    corner_cloud_from_map_copy_->clear();
    surf_cloud_from_map_copy_->clear();
    *corner_cloud_from_map_copy_ = *ds_corner_cloud_from_map_;
    *surf_cloud_from_map_copy_ = *ds_surf_cloud_from_map_;
    setMapStatus(true);
    LOG(INFO) << "get surrounding time cost  " << get_surround.toc() << "  ms "
              << std::endl;
    return true;
  } else {
    LOG(ERROR) << "This function works Only in mapping mode. ";
    return false;
  }
}

bool MapTrack::findCorrespondingFeatures() {
  // ceres opt
  ceres::LossFunction *ceres_loss_function = nullptr;
  ceres::LocalParameterization *pose_parameterization = nullptr;
  ceres::Problem *problem = nullptr;
  if (config_->use_ceres_opt) {
    ceres::Problem::Options problem_options;
    ceres_loss_function = new ceres::HuberLoss(0.1);
    pose_parameterization = new PosePara();
    problem = new ceres::Problem(problem_options);
    problem->AddParameterBlock(cur_frame_pose_.data(), 12,
                               pose_parameterization);
  } else {
    pose_solver_->clearFactors();
  }
  int valid_corners = 0;
  int valid_surfs = 0;

  PointType pointOri, pointSel;
  float error_sum = 0.0;
  TicToc match_cost;
  for (int i = 0; i < cur_corner_cloud_num_; i++) {
    float distance = 1.0;
    pointOri = ds_cur_corner_cloud_->points[i];
    Transformbox::pointAssociateToMap(&pointOri, &pointSel, cur_frame_pose_);
    std::vector<int> pointSearchInd;  // TODO:事先resize 提高效率
    pointSearchInd.reserve(5);
    std::vector<float> pointSearchSqDis;
    pointSearchSqDis.reserve(5);
    if (!pcl_isfinite(pointSel.x) || !pcl_isfinite(pointSel.y) ||
        !pcl_isfinite(pointSel.z) || !pcl_isfinite(pointSel.intensity)) {
      continue;
    }
    ptr_kdtree_corner_from_map_->nearestKSearch(pointSel, 5, pointSearchInd,
                                                pointSearchSqDis);
    if (pointSearchSqDis[4] < 1.0) {
      vVec3d near_corners;
      near_corners.reserve(5);
      Vec3d center(0., 0., 0.);
      for (int j = 0; j < 5; j++) {
        Vec3d temp_point(
            ds_corner_cloud_from_map_->points[pointSearchInd[j]].x,
            ds_corner_cloud_from_map_->points[pointSearchInd[j]].y,
            ds_corner_cloud_from_map_->points[pointSearchInd[j]].z);
        center += temp_point;
        near_corners.push_back(temp_point);
      }
      center = 0.2 * center;

      Mat3d cov_mat = Mat3d::Zero();
      for (int j = 0; j < 5; j++) {
        Vec3d temp_zero_mean = near_corners[j] - center;
        cov_mat += temp_zero_mean * temp_zero_mean.transpose();
      }
      cov_mat = 0.2 * cov_mat;
      Eigen::SelfAdjointEigenSolver<Mat3d> saes(cov_mat);
      Vec3d unit_direction = saes.eigenvectors().col(2);

      Vec3d curr_point(pointOri.x, pointOri.y, pointOri.z);

      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        Vec3d point_on_line = center;
        Vec3d point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        Vec3d curr_map_point =
            Mathbox::multiplePoint(cur_frame_pose_, curr_point);
        Vec3d l_pa_cross_l_pb =
            (curr_map_point - point_a).cross(curr_map_point - point_b);
        distance = 5 * l_pa_cross_l_pb.norm();
        double scale = 1. - 0.9 * fabs(distance);
        if (scale > 0.1) {
          valid_corners++;
          if (config_->use_ceres_opt) {
            ceres::CostFunction *cost_function =
                new EdgeFactor(curr_point, point_a, point_b, scale);
            problem->AddResidualBlock(cost_function, ceres_loss_function,
                                      cur_frame_pose_.data());

            // auto cost_function = std::make_shared<EdgeFactor>(
            //     curr_point, point_a, point_b, scale);
            // problem->AddResidualBlock(cost_function.get(),
            //                           ceres_loss_function.get(),
            //                           cur_frame_pose_.data());
          } else {
            auto edge_factor = std::make_shared<EdgePointFactor>(
                cur_frame_pose_, curr_point, point_a, point_b, scale);
            edge_factor->SetLossFunction(loss_function_.get());
            pose_solver_->addFactor(edge_factor);
          }
        }
      }
    }
    error_sum += distance;
  }

  LOG(INFO) << "corner feature  cost  " << match_cost.toc() << "   ms"
            << std::endl;
  TicToc surf_cost;
  for (int i = 0; i < cur_surf_cloud_num_; i++) {
    pointOri = ds_cur_surf_cloud_->points[i];
    Transformbox::pointAssociateToMap(&pointOri, &pointSel, cur_frame_pose_);
    std::vector<int> pointSearchInd;  // TODO:事先resize 提高效率
    pointSearchInd.reserve(5);
    std::vector<float> pointSearchSqDis;
    pointSearchSqDis.reserve(5);
    if (!pcl_isfinite(pointSel.x) || !pcl_isfinite(pointSel.y) ||
        !pcl_isfinite(pointSel.z) || !pcl_isfinite(pointSel.intensity)) {
      continue;
    }
    ptr_kdtree_surf_from_map_->nearestKSearch(pointSel, 5, pointSearchInd,
                                              pointSearchSqDis);

    float distance = 1.0;
    Mat53d A0;
    Vec5d B0 = -1 * Vec5d::Ones();
    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        A0(j, 0) = ds_surf_cloud_from_map_->points[pointSearchInd[j]].x;
        A0(j, 1) = ds_surf_cloud_from_map_->points[pointSearchInd[j]].y;
        A0(j, 2) = ds_surf_cloud_from_map_->points[pointSearchInd[j]].z;
      }
      Vec3d norm = A0.colPivHouseholderQr().solve(B0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool plane_valid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(
                norm(0) * ds_surf_cloud_from_map_->points[pointSearchInd[j]].x +
                norm(1) * ds_surf_cloud_from_map_->points[pointSearchInd[j]].y +
                norm(2) * ds_surf_cloud_from_map_->points[pointSearchInd[j]].z +
                negative_OA_dot_norm) > 0.2) {
          plane_valid = false;
          break;
        }
      }
      Vec3d curr_point(pointOri.x, pointOri.y, pointOri.z);

      if (plane_valid) {
        Vec3d curr_map_point =
            Mathbox::multiplePoint(cur_frame_pose_, curr_point);
        distance = fabs(norm.dot(curr_map_point) + negative_OA_dot_norm);
        double scale = 1. - 0.9 * distance;
        if (scale > 0.1) {
          valid_surfs++;
          if (config_->use_ceres_opt) {
            ceres::CostFunction *cost_function =
                new PlaneFactor(curr_point, norm, scale, negative_OA_dot_norm);
            problem->AddResidualBlock(cost_function, ceres_loss_function,
                                      cur_frame_pose_.data());
          } else {
            auto plane_factor = std::make_shared<PlanePointFactor>(
                cur_frame_pose_, curr_point, norm, scale, negative_OA_dot_norm);
            plane_factor->SetLossFunction(loss_function_.get());
            pose_solver_->addFactor(plane_factor);
          }
        }
      }
    }
    error_sum += distance;
  }
  LOG(INFO) << "surf feature  cost  " << surf_cost.toc() << "   ms"
            << std::endl;
  LOG(INFO) << "corner and surf cloud number  " << cur_corner_cloud_num_ << ' '
            << cur_surf_cloud_num_ << " " << valid_corners << " " << valid_surfs
            << std::endl;
  if (valid_corners < 10 || valid_surfs < 50) {
    LOG(ERROR) << "Too small valid points in current frame";
    return false;
  }
  distance_error_ = error_sum / (cur_corner_cloud_num_ + cur_surf_cloud_num_);

  TicToc opt_cost;
  if (config_->use_ceres_opt) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 5;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    LOG(INFO) << summary.BriefReport() << std::endl;
    delete problem;
  } else {
    pose_solver_->setPose(cur_frame_pose_);
    if (!pose_solver_->Solve(5)) {
      LOG(WARNING) << "Iter failed";
      return false;
    }
    cur_frame_pose_ = pose_solver_->getPose();
  }

  LOG(INFO) << "opt  cost  " << opt_cost.toc() << "   ms" << std::endl;
  LOG(INFO) << "find candidates and match  cost  " << match_cost.toc()
            << "   ms" << std::endl;
  return true;
}

void MapTrack::scanToLocalMap() {
  if (ds_corner_cloud_from_map_->size() > 10 &&
      ds_surf_cloud_from_map_->size() > 100) {
    TicToc tt;
    if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
      ptr_kdtree_corner_from_map_->setInputCloud(ds_corner_cloud_from_map_);
      ptr_kdtree_surf_from_map_->setInputCloud(ds_surf_cloud_from_map_);
    }

    cur_frame_pose_ = ptr_cur_keyframe_->getPose();
    is_match_ok_ = true;

    for (int iterCount = 0; iterCount < 10; iterCount++) {
      Mat34d last_temp_pose = cur_frame_pose_;
      if (!findCorrespondingFeatures()) {
        is_match_ok_ = false;
        return;
      }
      Mat34d delta_pose =
          Mathbox::deltaPose34d(last_temp_pose, cur_frame_pose_);
      double delta_rotation =
          Mathbox::rotationMatrixToEulerAngles(delta_pose.block<3, 3>(0, 0))
              .norm();
      double delta_trans = delta_pose.block<3, 1>(0, 3).norm();
      LOG(INFO) << "iter time " << iterCount << "  trans " << delta_rotation
                << ' ' << delta_trans << std::endl;
      if (delta_rotation < 0.002 && delta_trans < 0.01) {
        if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode() &&
            distance_error_ > 0.8) {
          LOG(WARNING) << "Too big distance error";
          is_match_ok_ = false;
        }
        break;
      } else if (9 == iterCount) {
        if (distance_error_ > 0.65) {
          is_match_ok_ = false;
          LOG(WARNING) << "real distance error " << distance_error_;
          LOG(INFO) << "map_to_odom_: \n" << map_to_odom_;
          LOG(INFO) << "opt cur_frame_pose_: \n" << cur_frame_pose_;
          LOG(INFO) << "bef cur_frame_pose_: \n" << ptr_cur_keyframe_->T_wb_;
        }
      }
    }
    const Mat34d &delta_odom_pose =
        Mathbox::deltaPose34d(last_frame_odom_pose_, ptr_cur_keyframe_->T_wo_);
    const Mat34d &delta_match_pose =
        Mathbox::deltaPose34d(last_frame_pose_, cur_frame_pose_);
    last_frame_odom_pose_ = ptr_cur_keyframe_->T_wo_;
    last_frame_pose_ = cur_frame_pose_;
    const Mat34d &delta_odom_and_match =
        Mathbox::deltaPose34d(delta_odom_pose, delta_match_pose);
    LOG(INFO) << "Delta pose pre & match \n " << delta_odom_and_match
              << std::endl;
    predict_error_ratio_ = delta_odom_and_match.block<3, 1>(0, 3).norm() /
                           config_->trans_threshold;
    if (predict_error_ratio_ > 0.5) {
      LOG(ERROR) << "predict_error_ratio  " << predict_error_ratio_
                 << std::endl;
      LOG(ERROR) << "frame id  " << ptr_cur_keyframe_->index_ << std::endl;
      laserCloud::Ptr cloud_bef_macth = Transformbox::transformPointCloud(
          ptr_cur_keyframe_->T_wb_, ptr_cur_keyframe_->surf_cloud_);
      // TODO:only for debug
      // laserCloud::Ptr cloud_after_macth = Transformbox::transformPointCloud(
      //     cur_frame_pose_, ptr_cur_keyframe_->surf_cloud_);
      // std::string before_file =
      // "/mnt/df233fce-51ce-45b9-beaa-b47d44fcc7ef/map_data/1204_c5/temp/before_"
      //                           + std::to_string(ptr_cur_keyframe_->index_) +
      //                           ".pcd";
      // std::string after_file =
      // "/mnt/df233fce-51ce-45b9-beaa-b47d44fcc7ef/map_data/1204_c5/temp/after_"
      //                           + std::to_string(ptr_cur_keyframe_->index_) +
      //                           ".pcd";
      // pcl::io::savePCDFileBinaryCompressed(before_file,
      //                                      *cloud_bef_macth);
      // pcl::io::savePCDFileBinaryCompressed(after_file, *cloud_after_macth);
    } else {
      LOG(INFO) << "predict_error_ratio  " << predict_error_ratio_ << std::endl;
    }

    // LOG(INFO) << "time cost  " << tt.toc() << "   ms" << std::endl;
  } else {
    LOG(ERROR) << "Too small map points  ";
    is_match_ok_ = false;
    if (is_first_keyframe_) {
      LOG(ERROR) << "Happened in first keyframe";
      LOG(WARNING) << "Ready to restart map track !!";
      Reset();
    }
  }
}

bool MapTrack::dowmSampleCurFrame() {
  TicToc downsample_cost;
  downsize_filter_cur_corner_.setInputCloud(ptr_cur_keyframe_->corner_cloud_);
  downsize_filter_cur_corner_.filter(*ds_cur_corner_cloud_);
  cur_corner_cloud_num_ = ds_cur_corner_cloud_->points.size();

  downsize_filter_cur_surf_.setInputCloud(ptr_cur_keyframe_->surf_cloud_);
  downsize_filter_cur_surf_.filter(*ds_cur_surf_cloud_);
  cur_surf_cloud_num_ = ds_cur_surf_cloud_->points.size();

  if (cur_surf_cloud_num_ < 100) {
    LOG(WARNING) << "Too few points  " << cur_surf_cloud_num_;
    return false;
  }
  return true;

  // LOG(INFO) << "downsample time cost  " << downsample_cost.toc() << "   ms"
  //           << std::endl;
}

bool MapTrack::buildKdtree() {
  ds_corner_cloud_from_map_ = ptr_map_manager_->getCornerMapCloud();
  ds_surf_cloud_from_map_ = ptr_map_manager_->getSurfMapCloud();
  if (0 == ds_corner_cloud_from_map_->size() ||
      0 == ds_surf_cloud_from_map_->size()) {
    return false;
  }
  ptr_kdtree_corner_from_map_->setInputCloud(ds_corner_cloud_from_map_);
  ptr_kdtree_surf_from_map_->setInputCloud(ds_surf_cloud_from_map_);
  is_kdtree_initialized_ = true;
  LOG(ERROR) << "Kdtree initialized OK" << std::endl;
  return true;
}

void MapTrack::clearCloud() {
  corner_cloud_from_map_->clear();
  surf_cloud_from_map_->clear();
  ds_cur_corner_cloud_->clear();
  ds_cur_surf_cloud_->clear();
}

Mat34d MapTrack::getMap2Odom() {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  return map_to_odom_;
}

void MapTrack::setMap2Odom(const Mat34d &map_to_odom) {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  map_to_odom_ = map_to_odom;
}

bool MapTrack::isKeyFrame(bool use_rotation) {
  Mat34d cur_frame_pose;
  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    cur_frame_pose = ptr_cur_keyframe_->T_wm_;
  } else {
    cur_frame_pose = ptr_cur_keyframe_->getPose();
  }
  Mat34d delta_pose =
      Mathbox::deltaPose34d(last_keyframe_pose_, cur_frame_pose);
  float distance = delta_pose.block<3, 1>(0, 3).norm();
  float delta_angle =
      Mathbox::rotationMatrixToEulerAngles(delta_pose.block<3, 3>(0, 0)).norm();
  if (std::string("velodyne") != config_->lidar_type || use_rotation) {
    //固态激光,平移0.3m　或旋转30°就判定为关键帧
    distance = std::max(distance, delta_angle);
  }
  if (distance > config_->min_distance) {
    last_keyframe_pose_ = cur_frame_pose;
    return true;
  } else {
    return false;
  }
}

void MapTrack::setPoseCov(const double cov) {
  pose_cov_ = cov;
}

bool MapTrack::isJunkFrame(const Mat34d &cur_odom_pose) {
  Mat34d delta_odom_pose =
      Mathbox::deltaPose34d(last_keyframe_odom_pose_, cur_odom_pose);
  double delta_angle =
      Mathbox::rotationMatrixToEulerAngles(delta_odom_pose.block<3, 3>(0, 0))
          .norm();
  double delta_trans = delta_odom_pose.block<3, 1>(0, 3).norm();

  if ((delta_angle) > config_->angle_threshold ||
      (delta_trans) > config_->trans_threshold) {
    last_keyframe_odom_pose_ = cur_odom_pose;
    return false;
  }
  return true;
}

void MapTrack::updateTransform() {
  std::unique_lock<std::mutex> lock(pose_mutex_);
  std::unique_lock<std::mutex> lock1(correct_pose_mutex_);
  // TODO:重新梳理一下，这个逻辑有很多多余的判断
  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    ptr_cur_keyframe_->updatePose(cur_frame_pose_);
    ptr_cur_keyframe_->updateLaserOdomPose(cur_frame_pose_);
    map_to_odom_ = Mathbox::multiplePose34d(
        ptr_cur_keyframe_->getPose(),
        Mathbox::inversePose34d(ptr_cur_keyframe_->getWheelPose()));
    ptr_cur_keyframe_->laser_odom_cov_ = cov_;
    ptr_cur_keyframe_->map_cov_ = cov_;
  } else {
    ptr_cur_keyframe_->T_wm_ = cur_frame_pose_;
    ptr_cur_keyframe_->map_cov_ = cov_;
    if (distance_error_ < 0.4 && predict_error_ratio_ > 0.5) {
      LOG(ERROR) << "Wheel odom Skidding Capture!!";
      ptr_cur_keyframe_->is_wheel_skidded_ = true;
      map_to_odom_ = Mathbox::multiplePose34d(
          cur_frame_pose_,
          Mathbox::inversePose34d(ptr_cur_keyframe_->getWheelPose()));
    }
  }
}

}  // namespace cvte_lidar_slam
