/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, CVTE.
 * All rights reserved.
 *
 *@file map_track.cpp
 *
 *@brief
 * 1.后端MAP_TRACK类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@modify Yun Su(robosu12@gmail.com)
 *@version 0.8
 *@data 2022-04-27
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

MapTrack::~MapTrack() {
  if (config_->save_keyPose_to_file) {
    frame_pose_result_.close();
    Lodom_pose_result_.close();
    wheel_pose_result_.close();
  }
}

void MapTrack::Reset() {
  match_failed_count_ = 0;
  low_overlap_count_ = 0;
  ds_corner_from_map_num_ = 0;
  ds_surf_from_map_num_ = 0;
  curr_corner_cloud_num_ = 0;
  curr_surf_cloud_num_ = 0;

  last_corner_cloud_num_ = 0;
  last_surf_cloud_num_ = 0;

  pose_cov_ = 10.;
  predict_error_ratio_ = 0.0;
  last_frame_time_ = -1.;

  last_frame_pose_ = Mathbox::Identity34();
  last_keyframe_pose_ = Mathbox::Identity34();
  last_frame_odom_pose_ = Mathbox::Identity34();
  last_keyframe_odom_pose_ = Mathbox::Identity34();
  map_to_odom_ = Mathbox::Identity34();
  last_map_to_odom_ = Mathbox::Identity34();

  cur_frame_pose_ = Mathbox::Identity34();
  keyframe_count_ = 0;
  is_first_keyframe_ = true;
  is_accept_keyframes_ = true;
  is_match_ok_ = true;
  is_last_match_ok_ = true;
  is_stopped_ = false;
  is_stop_requested_ = false;
  is_map_ready_ = false;
  is_local_map_ready_ = false;
  is_global_map_ready_ = false;
  is_set_init_pose_ = false;
  is_kdtree_initialized_ = false;

  map_odom_update_flag_ = true;

  ptr_cur_keyframe_ = std::make_shared<KeyFrame>();
  ptr_last_keyframe_ = std::make_shared<KeyFrame>();

  corner_cloud_from_map_.reset(new laserCloud());
  surf_cloud_from_map_.reset(new laserCloud());
  ds_corner_cloud_from_map_.reset(new laserCloud());
  ds_surf_cloud_from_map_.reset(new laserCloud());
  ds_curr_corner_cloud_.reset(new laserCloud());
  ds_curr_surf_cloud_.reset(new laserCloud());

  new_corner_cloud_from_map_.reset(new laserCloud());
  observed_corner_cloud_from_map_.reset(new laserCloud());

  ds_last_corner_cloud_.reset(new laserCloud());
  ds_last_surf_cloud_.reset(new laserCloud());

  moved_distance_ = 0.0;
  static_duration_ = 0.0;

  observed_keyframe_set_.clear();
}

void MapTrack::InitParameters() {
  ptr_cur_keyframe_ = std::make_shared<KeyFrame>();
  ptr_last_keyframe_ = std::make_shared<KeyFrame>();
  ptr_map_manager_ = MapManager::getInstance();

  ptr_kdtree_surf_from_map_.reset(new pclKdTree());
  ptr_kdtree_corner_from_map_.reset(new pclKdTree());

  ptr_kdtree_corner_from_keyframe_.reset(new pclKdTree());
  ptr_kdtree_new_corner_map_.reset(new pclKdTree());

  corner_cloud_from_map_.reset(new laserCloud());
  surf_cloud_from_map_.reset(new laserCloud());
  ds_corner_cloud_from_map_.reset(new laserCloud());
  ds_surf_cloud_from_map_.reset(new laserCloud());
  ds_curr_corner_cloud_.reset(new laserCloud());
  ds_curr_surf_cloud_.reset(new laserCloud());

  ds_last_corner_cloud_.reset(new laserCloud());
  ds_last_surf_cloud_.reset(new laserCloud());

  corner_cloud_from_map_copy_.reset(new laserCloud());
  surf_cloud_from_map_copy_.reset(new laserCloud());
  ds_cur_corner_cloud_copy_.reset(new laserCloud());
  ds_cur_surf_cloud_copy_.reset(new laserCloud());

  new_corner_cloud_from_map_.reset(new laserCloud());
  observed_corner_cloud_from_map_.reset(new laserCloud());

  // double leaf_size = 0.1;
  downsize_filter_corner_.setLeafSize(config_->edge_size, config_->edge_size,
                                      config_->edge_size);
  downsize_filter_surf_.setLeafSize(config_->surf_size, config_->surf_size,
                                    config_->surf_size);
  downsize_filter_cur_corner_.setLeafSize(
      config_->edge_size, config_->edge_size, config_->edge_size);
  downsize_filter_cur_surf_.setLeafSize(config_->surf_size, config_->surf_size,
                                        config_->surf_size);

  if (config_->save_keyPose_to_file) {
    frame_pose_result_.open("./frame_pose_result.txt");
    Lodom_pose_result_.open("./Lodom_pose_result.txt");
    wheel_pose_result_.open("./wheel_pose_result.txt");
  }

  Reset();
}

bool MapTrack::processNewKeyFrame(std::shared_ptr<KeyFrame> ptr_cur_keyframe,
                                  bool is_update_map) {
  // LOG(WARNING) << "Robot State " << ptr_state_machine_->getCurrentMode();

  if (nullptr == ptr_cur_keyframe) {
    LOG(ERROR) << "processNewKeyFrame: ptr_cur_keyframe == nullptr !!! ";
    return false;
  }

  ptr_last_keyframe_ = nullptr;
  ds_last_corner_cloud_->clear();
  ds_last_surf_cloud_->clear();

  is_last_match_ok_ = is_match_ok_;

  ptr_last_keyframe_ = ptr_cur_keyframe_;
  *ds_last_corner_cloud_ = *ds_curr_corner_cloud_;
  *ds_last_surf_cloud_ = *ds_curr_surf_cloud_;
  last_corner_cloud_num_ = curr_corner_cloud_num_;
  last_surf_cloud_num_ = curr_surf_cloud_num_;

  ptr_cur_keyframe_ = ptr_cur_keyframe;

  // LOG(WARNING) << "last keyframe: " << ptr_last_keyframe_->index_ << " , curr
  // keyframe: " << ptr_cur_keyframe_->index_;

  // 每帧点云处理流程：
  // 1. 先对当前帧进行降采样；
  // 2. 然后搜索关键帧构建局部地图；
  // 3. 利用里程计预测的位姿，与地图匹配进行位姿求解；

  bool ds_flag = dowmSampleCurFrame();

  if (ds_flag == true) {
    if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode() &&
        is_first_keyframe_) {
      last_frame_odom_pose_ = ptr_cur_keyframe_->getLaserOdomPose();
      last_frame_pose_ = ptr_cur_keyframe_->getPose();

      is_first_keyframe_ = false;
      LOG(WARNING) << "MapTrack::processNewKeyFrame: Insert first keyframe !!!";
      return true;
    }
    cov_ = Mat6d::Identity();
    keyframe_count_++;
    if (keyframe_count_ % 3 == 1 || is_update_map == true) {
      TicToc build_local_map_cost;
      bool local_map_flag = getSurroundingKeyFrames();

      if (local_map_flag == false) {
        LOG(ERROR) << "MapTrack::processNewKeyFrame: Get surrounding keyframes "
                      "failed !!!";
        return false;
      }
    }

    scanToLocalMap();

    cov_.block<3, 3>(0, 0) = Mat3d::Identity() * match_cov_;
    cov_.block<3, 3>(3, 3) = Mat3d::Identity() * match_cov_;
    ptr_cur_keyframe_->map_cov_ = cov_;

    if (is_match_ok_) {
      match_failed_count_ = 0;
      low_overlap_count_ = 0;
      updateTransform();
    } else {
      LOG(ERROR) << "MapTrack: map match failed ! ";
    }

    Mat34d delta_key_pose =
        Mathbox::deltaPose34d(last_frame_pose_, ptr_cur_keyframe_->getPose());
    double delta_key_dis = delta_key_pose.block<3, 1>(0, 3).norm();
    moved_distance_ += delta_key_dis;
    ptr_cur_keyframe_->moved_distance_ = moved_distance_;

    last_frame_odom_pose_ = ptr_cur_keyframe_->getLaserOdomPose();
    last_frame_pose_ = ptr_cur_keyframe_->getPose();

    Vec3d cur_frame_pos = ptr_cur_keyframe_->getPose().block<3, 1>(0, 3);
    Vec3d cur_frame_rpy = Mathbox::rotationMatrixToEulerAngles(
                              ptr_cur_keyframe_->getPose().block<3, 3>(0, 0)) *
                          Rad2Deg;
    Vec3d cur_Lodom_pos =
        ptr_cur_keyframe_->getLaserOdomPose().block<3, 1>(0, 3);
    Vec3d cur_Lodom_rpy =
        Mathbox::rotationMatrixToEulerAngles(
            ptr_cur_keyframe_->getLaserOdomPose().block<3, 3>(0, 0)) *
        Rad2Deg;
    Vec3d cur_wheel_pos = ptr_cur_keyframe_->getWheelPose().block<3, 1>(0, 3);
    Vec3d cur_wheel_rpy =
        Mathbox::rotationMatrixToEulerAngles(
            ptr_cur_keyframe_->getWheelPose().block<3, 3>(0, 0)) *
        Rad2Deg;
    if (is_match_ok_) {
      LOG(WARNING) << "curr frame_pose (" << cur_frame_pos.transpose().x()
                   << " , " << cur_frame_pos.transpose().y() << " , "
                   << cur_frame_rpy.transpose()[2] << ") "
                   << "cur laser odom pose (" << cur_Lodom_pos.transpose().x()
                   << " , " << cur_Lodom_pos.transpose().y() << " , "
                   << cur_Lodom_rpy.transpose()[2] << ") "
                   << "cur wheel odom pose (" << cur_wheel_pos.transpose().x()
                   << " , " << cur_wheel_pos.transpose().y() << " , "
                   << cur_wheel_rpy.transpose()[2] << ") ";
    } else {
      LOG(ERROR) << "curr frame_pose (" << cur_frame_pos.transpose().x()
                 << " , " << cur_frame_pos.transpose().y() << " , "
                 << cur_frame_rpy.transpose()[2] << ") "
                 << "cur laser odom pose (" << cur_Lodom_pos.transpose().x()
                 << " , " << cur_Lodom_pos.transpose().y() << " , "
                 << cur_Lodom_rpy.transpose()[2] << ") "
                 << "cur wheel odom pose (" << cur_wheel_pos.transpose().x()
                 << " , " << cur_wheel_pos.transpose().y() << " , "
                 << cur_wheel_rpy.transpose()[2] << ") ";
    }

    if (config_->save_keyPose_to_file) {
      Eigen::Quaterniond cur_frame_q =
          Eigen::Quaterniond(ptr_cur_keyframe_->getPose().block<3, 3>(0, 0));
      Eigen::Quaterniond cur_Lodom_q = Eigen::Quaterniond(
          ptr_cur_keyframe_->getLaserOdomPose().block<3, 3>(0, 0));
      Eigen::Quaterniond cur_wheel_q = Eigen::Quaterniond(
          ptr_cur_keyframe_->getWheelPose().block<3, 3>(0, 0));
      cur_frame_q.normalize();
      cur_Lodom_q.normalize();
      cur_wheel_q.normalize();
      frame_pose_result_ << std::fixed << ptr_cur_keyframe_->time_stamp_ << ' '
                         << cur_frame_pos.x() << ' ' << cur_frame_pos.y() << ' '
                         << cur_frame_pos.z() << ' ' << cur_frame_q.x() << ' '
                         << cur_frame_q.y() << ' ' << cur_frame_q.z() << ' '
                         << cur_frame_q.w() << std::endl;
      Lodom_pose_result_ << std::fixed << ptr_cur_keyframe_->time_stamp_ << ' '
                         << cur_Lodom_pos.x() << ' ' << cur_Lodom_pos.y() << ' '
                         << cur_Lodom_pos.z() << ' ' << cur_Lodom_q.x() << ' '
                         << cur_Lodom_q.y() << ' ' << cur_Lodom_q.z() << ' '
                         << cur_Lodom_q.w() << std::endl;
      wheel_pose_result_ << std::fixed << ptr_cur_keyframe_->time_stamp_ << ' '
                         << cur_wheel_pos.x() << ' ' << cur_wheel_pos.y() << ' '
                         << cur_wheel_pos.z() << ' ' << cur_wheel_q.x() << ' '
                         << cur_wheel_q.y() << ' ' << cur_wheel_q.z() << ' '
                         << cur_wheel_q.w() << std::endl;
    }

    if (config_->map_update == true) {
      updateMapPointProbability();
    }

    clearCloud();
  }

  return true;
}

bool MapTrack::dowmSampleCurFrame() {
  TicToc downsample_cost;

  ds_curr_corner_cloud_->clear();
  ds_curr_surf_cloud_->clear();

  // 角点本来就比较少，而且一般是离散分布在边缘，降采样后可能会改变其在边缘的位置，因此不用降采样；
  *ds_curr_corner_cloud_ = *(ptr_cur_keyframe_->corner_cloud_);
  curr_corner_cloud_num_ = ds_curr_corner_cloud_->size();

  if (config_->print_debug) {
    printf(
        "--MapTrack::dowmSampleCurFrame: corner: %d, surf: %d, time: %.1f \n",
        curr_corner_cloud_num_, curr_surf_cloud_num_, downsample_cost.toc());
  }

  if (curr_corner_cloud_num_ < config_->min_feature_points) {
    LOG(WARNING) << "dowmSampleCurFrame: Too few points! cur_corner: "
                 << curr_corner_cloud_num_;
    return false;
  }

  return true;

  // LOG(INFO) << "downsample time cost  " << downsample_cost.toc() << "   ms"
  //           << std::endl;
}

bool MapTrack::getSurroundingKeyFrames() {
  static int surround_search_num = config_->surround_search_num;
  static float surround_search_radius = config_->surround_search_radius;

  TicToc get_surround;

  // 创建关键帧时已经利用预测值对pos进行赋值；
  auto cur_frame_pos = ptr_cur_keyframe_->getPosition();

  // 定位模式初始时直接将整个地图构建为kdtree，不用再进行地图构建；
  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    if (config_->match_algorithm == 2 || config_->match_algorithm == 3) {
      if (is_kdtree_initialized_ == false) {
        bool build_flag = buildKdtree();
        if (build_flag == false) {
          LOG(ERROR) << "Kdtree initialized failed !!!";
        }
      }
      setMapStatus(true);
      return is_kdtree_initialized_;
    }
  }

  std::vector<std::shared_ptr<KeyFrame>> surroundingKeyFrames;

  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    surroundingKeyFrames = ptr_map_manager_->DetectCandidatesByDistance(
        cur_frame_pos, surround_search_radius);
  } else if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    if (config_->is_search_by_time)  // 找时间最近的帧构建局部地图；
    {
      surroundingKeyFrames =
          ptr_map_manager_->DetectCandidatesByTime(surround_search_num);
    } else  // 根据空间距离范围内的帧构建局部地图；
    {
      surroundingKeyFrames = ptr_map_manager_->DetectCandidatesByDistance(
          cur_frame_pos, config_->surround_search_radius);
    }
  } else {
    LOG(ERROR) << "This function works Only in mapping mode. ";
    return false;
  }

  if (surroundingKeyFrames.empty()) {
    LOG(ERROR) << "surround keyframes Empty. ";
    return false;
  }

  // TODO:
  // 可以先对搜索到的关键帧进行1m网格大小的降采样，减小地图点的总规模，从而减少地图点的降采样时间；

  corner_cloud_from_map_->clear();
  surf_cloud_from_map_->clear();

  int surroundingKeyFramesNums = surroundingKeyFrames.size();

  // 根据位姿拼接地图；
  for (int i = 0; i < surroundingKeyFramesNums; i++) {
    Mat34d it_frame_pose = surroundingKeyFrames[i]->getPose();

    laserCloud::Ptr surround_corner_cloud = Transformbox::transformPointCloud(
        it_frame_pose, surroundingKeyFrames[i]->corner_cloud_);
    *corner_cloud_from_map_ += *surround_corner_cloud;
  }

  std::unique_lock<std::mutex> lock(curr_surround_map_mutex_);

  setMapStatus(false);

  ds_corner_cloud_from_map_->clear();
  ds_surf_cloud_from_map_->clear();
  // 下采样
  // Downsample the surrounding corner key frames (or map)
  downsize_filter_corner_.setInputCloud(corner_cloud_from_map_);
  downsize_filter_corner_.filter(*ds_corner_cloud_from_map_);
  ds_corner_from_map_num_ = ds_corner_cloud_from_map_->points.size();

  setMapStatus(true);

  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    if (config_->match_algorithm == 2 || config_->match_algorithm == 3) {
      ptr_kdtree_corner_from_map_->setInputCloud(ds_corner_cloud_from_map_);
    }
  }

  LOG(INFO) << "MapTrack: get surround map time cost: " << get_surround.toc()
            << "  ms " << std::endl;
  return true;
}

void MapTrack::scanToLocalMap() {
  static double max_map_match_distance = config_->max_map_match_success_score;
  static double odom_match_ratio = config_->odom_match_ratio;

  if (ds_corner_cloud_from_map_->size() > config_->min_feature_points) {
    TicToc tt;

    cur_frame_pose_ = ptr_cur_keyframe_->getPose();
    is_match_ok_ = true;
    int max_iteration = 3;

    // if (config_->match_algorithm == 2 || config_->match_algorithm == 3) {
    //   if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    //     max_iteration = max_iteration * 4;
    //   } else if (AD_MODE::AD_LOCALIZATION ==
    //              ptr_state_machine_->getCurrentMode()) {
    //     max_iteration = max_iteration * 2;
    //   }
    // }

    for (int iterCount = 0; iterCount < max_iteration; iterCount++) {
      Mat34d last_temp_pose = cur_frame_pose_;

      // 与地图点进行匹配，对位姿进行优化；

      if (config_->match_algorithm == 1) {
        // match by PCLICP
        PCLICPMatch();
      } else if (config_->match_algorithm == 2) {
        // match by CeresICP
        if (!CeresICPMatch()) {
          // is_match_ok_ = false;
        }
      } else if (config_->match_algorithm == 3) {
        // match by CeresICP
        if (!CeresPLICPMatch()) {
          // is_match_ok_ = false;
        }
      }

      Mat34d delta_pose =
          Mathbox::deltaPose34d(last_temp_pose, cur_frame_pose_);
      double delta_rotation =
          Mathbox::rotationMatrixToEulerAngles(delta_pose.block<3, 3>(0, 0))
              .norm();
      double delta_trans = delta_pose.block<3, 1>(0, 3).norm();
      LOG(INFO) << "MapTrack:iterCount: " << iterCount
                << "; delta_trans: " << delta_trans
                << ", delta_rot: " << delta_rotation << std::endl;
      // LOG(WARNING) << "iterCount: " << iterCount;

      if (delta_rotation < 0.002 && delta_trans < 0.01) {
        if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
          if (curr_distance_error_ > max_map_match_distance) {
            is_match_ok_ = false;
            LOG(ERROR) << "MapTrack: Too big match distance: "
                       << curr_distance_error_;
          }
          break;
        }
      } else if ((max_iteration - 1) == iterCount) {
        if (curr_distance_error_ > max_map_match_distance) {
          is_match_ok_ = false;
          LOG(ERROR)
              << "MapTrack: reach max_iteration ! Too big match distance: "
              << curr_distance_error_;

          // LOG(WARNING) << "real distance error " << curr_distance_error_;
          // LOG(WARNING) << "map_to_odom_: " << map_to_odom_;
          // LOG(WARNING) << "opt cur_frame_pose_: " << cur_frame_pose_;
          // LOG(WARNING) << "bef cur_frame_pose_: " <<
          // ptr_cur_keyframe_->T_wb_;
        }
      }
    }

    bool PLANE_MODE = true;
    if (PLANE_MODE == true) {
      Vec3d p_predict = cur_frame_pose_.block<3, 1>(0, 3);
      Mat3d r_predict = cur_frame_pose_.block<3, 3>(0, 0);
      Vec3d rpy_predict = Mathbox::rotation2rpy(r_predict);

      p_predict.z() = 0.0;
      rpy_predict.x() = 0.0;
      rpy_predict.y() = 0.0;
      r_predict = Mathbox::rpyToRotationMatrix(rpy_predict);

      cur_frame_pose_.block<3, 1>(0, 3) = p_predict;
      cur_frame_pose_.block<3, 3>(0, 0) = r_predict;
    }

    const Mat34d &delta_odom_pose = Mathbox::deltaPose34d(
        last_frame_odom_pose_, ptr_cur_keyframe_->getLaserOdomPose());
    const Mat34d &delta_match_pose =
        Mathbox::deltaPose34d(last_frame_pose_, cur_frame_pose_);

    // 轮子里程计增量 与 匹配优化增量 之间的差值；用来判断轮子打滑情况；
    const Mat34d &delta_odom_and_match =
        Mathbox::deltaPose34d(delta_odom_pose, delta_match_pose);
    // LOG(INFO) << "Delta pose between odom and match: \n " <<
    // delta_odom_and_match << std::endl;

    Mat34d delta_pose_accept = Mathbox::Interp_SE3(
        delta_odom_pose, delta_match_pose, odom_match_ratio);  // 1.0

    if (map_odom_update_flag_ == true) {
      last_frame_pose_ = cur_frame_pose_;
      delta_pose_accept = Mathbox::Identity34();
      map_odom_update_flag_ = false;
      LOG(WARNING) << "update last_frame_pose_: \n" << last_frame_pose_;
    }
    cur_frame_pose_ =
        Mathbox::multiplePose34d(last_frame_pose_, delta_pose_accept);

    Vec3d d_dis_odom = delta_odom_pose.block<3, 1>(0, 3);
    Vec3d d_rot_odom =
        Mathbox::rotationMatrixToEulerAngles(delta_odom_pose.block<3, 3>(0, 0));

    Vec3d d_dis_match = delta_match_pose.block<3, 1>(0, 3);
    Vec3d d_rot_match = Mathbox::rotationMatrixToEulerAngles(
        delta_match_pose.block<3, 3>(0, 0));

    // 通过两个运动增量之间的差值大小来判断是否打滑；
    double between_dis = delta_odom_and_match.block<3, 1>(0, 3).norm();
    double between_rot = Mathbox::rotationMatrixToEulerAngles(
                             delta_odom_and_match.block<3, 3>(0, 0))
                             .norm();
    double predict_dis_ratio = between_dis / config_->trans_threshold;
    double predict_rot_ratio = between_rot / config_->angle_threshold;

    predict_error_ratio_ = std::max(predict_dis_ratio, predict_rot_ratio);

    // if (predict_error_ratio_ > 0.5 || is_match_ok_ == false)
    if (predict_error_ratio_ > 0.5) {
      LOG(ERROR) << "MapTrack: big gap between Lodom and match !!! ";
      LOG(ERROR) << "predict_error_ratio: " << predict_error_ratio_;
      LOG(ERROR) << "map match distance : " << curr_distance_error_;
      LOG(ERROR) << "d_dis_odom : " << d_dis_odom.norm()
                 << ", d_rot_odom: " << d_rot_odom.norm();
      LOG(ERROR) << "d_dis_match: " << d_dis_match.norm()
                 << ", d_rot_match: " << d_rot_match.norm();
    } else {
      LOG(INFO) << "MapTrack: map match distance: " << curr_distance_error_;
    }

    double init_match_cov = 0.02;
    if (d_dis_match.norm() > 0.5) {
      match_cov_ = init_match_cov * d_dis_match.norm() * 2.0;
    } else {
      match_cov_ = init_match_cov;
    }

    static double map_match_failed_distance =
        6.0 * config_->max_map_match_success_score;
    if (curr_distance_error_ > map_match_failed_distance &&
        d_dis_match.norm() > 0.1) {
      match_failed_count_++;
      LOG(ERROR) << "MapTrack: too big match distance ! match_failed_count: "
                 << match_failed_count_;
    }

    static double min_map_match_overlap_ratio =
        config_->min_map_match_overlap_ratio;
    if (overlap_ratio_ < min_map_match_overlap_ratio &&
        d_dis_match.norm() > 0.1) {
      low_overlap_count_++;
      LOG(ERROR) << "MapTrack: too low match overlap ! overlap_ratio: "
                 << overlap_ratio_;
      LOG(ERROR) << "MapTrack: too low match overlap ! low_overlap_count: "
                 << low_overlap_count_;
    }
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

void MapTrack::PCLICPMatch() {
  static Eigen::Isometry3f _T_w_init, _T_w_updated;
  static Eigen::Matrix3d _r_w_curr;
  static Eigen::Vector3d _t_w_curr;

  _r_w_curr = cur_frame_pose_.block<3, 3>(0, 0);
  _t_w_curr = cur_frame_pose_.block<3, 1>(0, 3);

  _T_w_init = Eigen::Isometry3f::Identity();
  _T_w_init.rotate(_r_w_curr.cast<float>());
  _T_w_init.pretranslate(_t_w_curr.cast<float>());

  pcl::PointCloud<PointType>::Ptr lidarCloud(new pcl::PointCloud<PointType>());
  *lidarCloud += *ds_curr_corner_cloud_;
  // *lidarCloud += *ds_curr_surf_cloud_;

  pcl::PointCloud<PointType>::Ptr localMapCloud(
      new pcl::PointCloud<PointType>());
  *localMapCloud += *ds_corner_cloud_from_map_;

  LOG(INFO) << "PCLICPMatch-used points: "
            << "laser: " << lidarCloud->size()
            << ", localMap: " << localMapCloud->size();

  pcl::PointCloud<PointType>::Ptr preAlignedCloud(
      new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*lidarCloud, *preAlignedCloud, _T_w_init);

  TicToc t_ICPAlign;
  static pcl::IterativeClosestPoint<PointType, PointType> icp_tmp;

  size_t max_iter = 20;
  double max_match_dis = config_->max_match_dis;
  if (is_last_match_ok_ == false &&
      AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    max_iter = max_iter * 2.0;
    max_match_dis = config_->max_match_dis * 2.0;
  }
  // if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
  //   max_match_dis = config_->max_match_dis * 0.5;
  // }
  if (ptr_cur_keyframe_->environment_flag_ == 1) {
    max_match_dis = 0.5 * config_->max_match_dis;
    LOG(ERROR) << "narrow space ! max_match_dis: " << max_match_dis;
  }

  // ICP Settings
  icp_tmp.setMaxCorrespondenceDistance(
      max_match_dis);  // 0.26的取值原则：地图的分辨率为0.1，大于0.25的距离能保证找到最近的3个点；
  icp_tmp.setMaximumIterations(max_iter);
  icp_tmp.setTransformationEpsilon(1e-6);
  icp_tmp.setEuclideanFitnessEpsilon(1e-6);

  // Align clouds
  pcl::PointCloud<PointType>::Ptr align_result(
      new pcl::PointCloud<PointType>());
  icp_tmp.setInputSource(preAlignedCloud);
  icp_tmp.setInputTarget(localMapCloud);
  icp_tmp.align(*align_result);

  Eigen::Isometry3f dT_correct = Eigen::Isometry3f::Identity();
  dT_correct = icp_tmp.getFinalTransformation();

  Eigen::Vector3d _dp_correct;
  Eigen::Matrix3d _dr_correct;
  _dp_correct = dT_correct.translation().cast<double>();
  _dr_correct = dT_correct.rotation().cast<double>();

  // curr_distance_error_ = icp_tmp.getFitnessScore(max_match_dis * 6.0);

  // Transform the input cloud using the final transformation
  pcl::PointCloud<PointType>::Ptr aligned_cloud(
      new pcl::PointCloud<PointType>());
  Mat34d dT_correct_34 = Mathbox::Identity34();
  dT_correct_34.block<3, 3>(0, 0) = _dr_correct;
  dT_correct_34.block<3, 1>(0, 3) = _dp_correct;
  aligned_cloud =
      Transformbox::transformPointCloud(dT_correct_34, preAlignedCloud);

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  auto KdTreePtr = icp_tmp.getSearchMethodTarget();

  size_t inlier_num = 0;
  float match_dis_sum = 0.0;
  float match_dis_ave = 0.99;
  float inlier_dis = max_match_dis * 6.0;
  size_t cloud_size = aligned_cloud->points.size();
  for (size_t i = 0; i < cloud_size; ++i) {
    // Find its nearest neighbor in the target
    KdTreePtr->nearestKSearch(aligned_cloud->points[i], 1, nn_indices,
                              nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] < inlier_dis) {
      match_dis_sum += nn_dists[0];
      inlier_num++;
    }
  }

  if (inlier_num > 10) {
    match_dis_ave = match_dis_sum / inlier_num;
  }

  overlap_ratio_ = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio_ < 0.1) {
    overlap_ratio_ = 0.1;
  }

  // double overlap_trans = (overlap_ratio + 0.1) * (overlap_ratio + 0.1);
  double overlap_trans = overlap_ratio_ / 0.9;

  double overlap_score = overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  if (icp_tmp.hasConverged() == false) {
    LOG(ERROR) << "MapTrack: icp not converge, match_dis: "
               << curr_distance_error_ << ", time: " << t_ICPAlign.toc();

    // TODO: limit matched delta R,T
    // dT_correct = Eigen::Isometry3f::Identity();
  }

  _T_w_updated = dT_correct * _T_w_init;  // 得到位姿修正量，累加得到当前位姿；

  _t_w_curr = _T_w_updated.translation().cast<double>();
  _r_w_curr = _T_w_updated.rotation().cast<double>();

  cur_frame_pose_.block<3, 3>(0, 0) = _r_w_curr;
  cur_frame_pose_.block<3, 1>(0, 3) = _t_w_curr;
}

bool MapTrack::CeresICPMatch() {
  Mat34d dT_correct = Mathbox::Identity34();

  pcl::PointCloud<PointType>::Ptr pre_aligned_curr_corner(
      new pcl::PointCloud<PointType>());
  pre_aligned_curr_corner =
      Transformbox::transformPointCloud(cur_frame_pose_, ds_curr_corner_cloud_);

  // ceres opt
  ceres::LossFunction *ceres_loss_function = nullptr;
  ceres::LocalParameterization *pose_parameterization = nullptr;
  ceres::Problem *problem = nullptr;

  ceres::Problem::Options problem_options;
  ceres_loss_function = new ceres::HuberLoss(0.1);
  pose_parameterization = new PosePara();
  problem = new ceres::Problem(problem_options);

  // problem->AddParameterBlock(dT_correct.data(), 12, pose_parameterization);

  double para_dq[4] = {0.0, 0.0, 0.0, 1.0};
  double para_dt[3] = {0.0, 0.0, 0.0};
  Eigen::Map<Eigen::Quaterniond> dq_correct(para_dq);
  Eigen::Map<Eigen::Vector3d> dt_correct(para_dt);
  ceres::LocalParameterization *q_parameterization =
      new ceres::EigenQuaternionParameterization();
  problem->AddParameterBlock(para_dq, 4, q_parameterization);
  problem->AddParameterBlock(para_dt, 3);

  size_t max_iter = 20;
  double max_match_dis = config_->max_match_dis;
  if (is_last_match_ok_ == false &&
      AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    max_iter = max_iter * 2.0;
    max_match_dis = config_->max_match_dis * 2.0;
  }
  // if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
  //   max_match_dis = config_->max_match_dis * 0.5;
  // }
  if (ptr_cur_keyframe_->environment_flag_ == 1) {
    max_match_dis = 0.5 * config_->max_match_dis;
    LOG(ERROR) << "narrow space ! max_match_dis: " << max_match_dis;
  }

  int valid_corners = 0;
  int valid_surfs = 0;

  PointType pointOri, pointSel;
  PointType PCLPoint[5];
  Vec3d near_map_point[5];
  float error_sum = 0.0;
  std::set<int> map_point_index_set;
  TicToc match_cost;

  int edge_point_num = pre_aligned_curr_corner->size();
  for (int i = 0; i < edge_point_num; i++) {
    float distance = 1.0;
    pointOri = pre_aligned_curr_corner->points[i];

    std::vector<int> pointSearchInd;  // TODO:事先resize 提高效率
    pointSearchInd.reserve(5);
    std::vector<float> pointSearchSqDis;
    pointSearchSqDis.reserve(5);

    // if (ptr_kdtree_corner_from_map_->radiusSearch(
    //         pointOri, config_->max_match_dis, pointSearchInd,
    //         pointSearchSqDis, 1) > 0) {
    if (ptr_kdtree_corner_from_map_->nearestKSearch(pointOri, 1, pointSearchInd,
                                                    pointSearchSqDis) > 0) {
      // if (map_point_index_set.find(pointSearchInd[0]) !=
      //     map_point_index_set.end()) {
      //   // LOG(ERROR) << "MapTrack::CeresICPMatch: already used point !";
      //   continue;
      // }

      // map_point_index_set.insert(pointSearchInd[0]);

      if (pointSearchSqDis[0] < max_match_dis) {
        PCLPoint[0] = ds_corner_cloud_from_map_->points[pointSearchInd[0]];
        near_map_point[0] = Vec3d(PCLPoint[0].x, PCLPoint[0].y, PCLPoint[0].z);

        Vec3d lidar_point(pointOri.x, pointOri.y, pointOri.z);

        distance = pointSearchSqDis[0];

        ceres::CostFunction *point_dis_factor =
            PointDistanceFactor::Create(lidar_point, near_map_point[0]);
        problem->AddResidualBlock(point_dis_factor, ceres_loss_function,
                                  para_dq, para_dt);

        valid_corners++;
        error_sum += distance;
      }
    }
  }

  TicToc surf_cost;
  for (int i = 0; i < curr_corner_cloud_num_; i++) {
    //
  }

  LOG(INFO) << "MapTrack::CeresICPMatch: valid_corners: " << valid_corners
            << ", localMap: " << ds_corner_cloud_from_map_->size();

  if (valid_corners < config_->min_feature_points) {
    LOG(ERROR) << "MapTrack::CeresICPMatch: Too small valid_corners in current "
                  "frame !!!";
  } else {
    TicToc opt_cost;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 5;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-6;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    // LOG(WARNING) << summary.FullReport() << std::endl;
    // LOG(INFO) << summary.BriefReport() << std::endl;
  }

  dq_correct.normalize();
  dT_correct.block<3, 3>(0, 0) = dq_correct.toRotationMatrix();
  dT_correct.block<3, 1>(0, 3) = dt_correct;
  cur_frame_pose_ = Mathbox::multiplePose34d(dT_correct, cur_frame_pose_);

  // Transform the input cloud using the final transformation
  pcl::PointCloud<PointType>::Ptr aligned_curr_corner_cloud(
      new pcl::PointCloud<PointType>());
  aligned_curr_corner_cloud =
      Transformbox::transformPointCloud(dT_correct, pre_aligned_curr_corner);

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  size_t inlier_num = 0;
  float match_dis_sum = 0.0;
  float match_dis_ave = 0.99;
  float inlier_dis = config_->max_match_dis * 6.0;
  size_t cloud_size = aligned_curr_corner_cloud->points.size();
  for (size_t i = 0; i < cloud_size; ++i) {
    // Find its nearest neighbor in the target
    ptr_kdtree_corner_from_map_->nearestKSearch(
        aligned_curr_corner_cloud->points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] < inlier_dis) {
      match_dis_sum += nn_dists[0];
      inlier_num++;
    }
  }

  if (inlier_num > 10) {
    match_dis_ave = match_dis_sum / inlier_num;
  }

  overlap_ratio_ = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio_ < 0.1) {
    overlap_ratio_ = 0.1;
  }

  // double overlap_trans = (overlap_ratio + 0.1) * (overlap_ratio + 0.1);
  double overlap_trans = overlap_ratio_ / 0.9;

  double overlap_score = overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  // LOG(INFO) << "MapTrack::CeresICPMatch: overlap_ratio: " << overlap_ratio_;
  // LOG(INFO) << "MapTrack::CeresICPMatch: overlap_score: " << overlap_score;

  delete problem;

  // LOG(INFO) << "CeresICPMatch cost: " << match_cost.toc() << " ms";
  return true;
}

bool MapTrack::CeresPLICPMatch() {
  static float min_match_prob = config_->min_match_prob;
  static float init_map_prob = config_->init_map_prob;

  Mat34d dT_correct = Mathbox::Identity34();

  pcl::PointCloud<PointType>::Ptr pre_aligned_curr_corner(
      new pcl::PointCloud<PointType>());
  pre_aligned_curr_corner =
      Transformbox::transformPointCloud(cur_frame_pose_, ds_curr_corner_cloud_);

  // ceres opt
  ceres::LossFunction *ceres_loss_function = nullptr;
  ceres::LocalParameterization *pose_parameterization = nullptr;
  ceres::Problem *problem = nullptr;

  ceres::Problem::Options problem_options;
  ceres_loss_function = new ceres::HuberLoss(0.1);
  pose_parameterization = new PosePara();
  problem = new ceres::Problem(problem_options);

  double para_dq[4] = {0.0, 0.0, 0.0, 1.0};
  double para_dt[3] = {0.0, 0.0, 0.0};
  Eigen::Map<Eigen::Quaterniond> dq_correct(para_dq);
  Eigen::Map<Eigen::Vector3d> dt_correct(para_dt);
  ceres::LocalParameterization *q_parameterization =
      new ceres::EigenQuaternionParameterization();
  problem->AddParameterBlock(para_dq, 4, q_parameterization);
  problem->AddParameterBlock(para_dt, 3);

  size_t max_iter = 20;
  double max_match_dis = config_->max_match_dis;
  if (is_last_match_ok_ == false &&
      AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    max_iter = max_iter * 2.0;
    max_match_dis = config_->max_match_dis * 2.0;
  }
  // if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
  //   max_match_dis = config_->max_match_dis * 0.5;
  // }
  if (ptr_cur_keyframe_->environment_flag_ == 1) {
    max_match_dis = 0.5 * config_->max_match_dis;
    LOG(ERROR) << "narrow space ! max_match_dis: " << max_match_dis;
  }

  int valid_corners = 0;
  int valid_surfs = 0;

  static std::vector<int> pointSearchInd(5);  // TODO:事先resize 提高效率
  static std::vector<float> pointSearchSqDis(5);
  PointType pointOri, pointSel;
  PointType PCLPoint[5];
  Vec3d near_map_point[5];
  float error_sum = 0.0;
  std::set<int> map_point_index_set;
  TicToc match_cost;

  int edge_point_num = pre_aligned_curr_corner->size();
  for (int i = 0; i < edge_point_num; i++) {
    float distance = 1.0;
    pointOri = pre_aligned_curr_corner->points[i];

    if (ptr_kdtree_corner_from_map_->nearestKSearch(pointOri, 2, pointSearchInd,
                                                    pointSearchSqDis) > 1) {
      // if (map_point_index_set.find(pointSearchInd[0]) !=
      //     map_point_index_set.end()) {
      //   // LOG(ERROR) << "MapTrack::CeresICPMatch: already used point !";
      //   continue;
      // }

      // map_point_index_set.insert(pointSearchInd[0]);

      if (pointSearchSqDis[1] < max_match_dis) {
        PCLPoint[0] = ds_corner_cloud_from_map_->points[pointSearchInd[0]];
        PCLPoint[1] = ds_corner_cloud_from_map_->points[pointSearchInd[1]];
        near_map_point[0] = Vec3d(PCLPoint[0].x, PCLPoint[0].y, PCLPoint[0].z);
        near_map_point[1] = Vec3d(PCLPoint[1].x, PCLPoint[1].y, PCLPoint[1].z);

        if (PCLPoint[0].intensity < min_match_prob ||
            PCLPoint[1].intensity < min_match_prob) {
          continue;
        }

        Vec3d lidar_point(pointOri.x, pointOri.y, pointOri.z);

        Vec3d l_pa_cross_l_pb = (lidar_point - near_map_point[0])
                                    .cross(lidar_point - near_map_point[1]);

        float edge_dis = (near_map_point[0] - near_map_point[1]).norm();
        if (edge_dis < 0.01) {
          continue;
        }
        distance = l_pa_cross_l_pb.norm() / edge_dis;

        ceres::CostFunction *point_to_line_factor =
            PointToLineDistanceFactor::Create(lidar_point, near_map_point[0],
                                              near_map_point[1]);
        problem->AddResidualBlock(point_to_line_factor, ceres_loss_function,
                                  para_dq, para_dt);

        if (pointSearchSqDis[0] < 0.3 * max_match_dis) {
          ceres::CostFunction *point_dis_factor =
              PointDistanceFactor::Create(lidar_point, near_map_point[0]);
          problem->AddResidualBlock(point_dis_factor, ceres_loss_function,
                                    para_dq, para_dt);
        }

        valid_corners++;
        error_sum += distance;
      }
    }
  }

  TicToc surf_cost;
  for (int i = 0; i < curr_corner_cloud_num_; i++) {
    //
  }

  LOG(INFO) << "MapTrack::CeresPLICPMatch: valid_corners: " << valid_corners
            << ", localMap: " << ds_corner_cloud_from_map_->size();

  if (valid_corners < config_->min_feature_points) {
    LOG(ERROR)
        << "MapTrack::CeresPLICPMatch: Too small valid_corners in current "
           "frame !!!";
  } else {
    TicToc opt_cost;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 5;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-6;
    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    // LOG(WARNING) << summary.FullReport() << std::endl;
    // LOG(INFO) << summary.BriefReport() << std::endl;
  }

  dq_correct.normalize();
  dT_correct.block<3, 3>(0, 0) = dq_correct.toRotationMatrix();
  dT_correct.block<3, 1>(0, 3) = dt_correct;
  cur_frame_pose_ = Mathbox::multiplePose34d(dT_correct, cur_frame_pose_);

  // Transform the input cloud using the final transformation
  pcl::PointCloud<PointType>::Ptr aligned_curr_corner_cloud(
      new pcl::PointCloud<PointType>());
  aligned_curr_corner_cloud =
      Transformbox::transformPointCloud(dT_correct, pre_aligned_curr_corner);

  size_t inlier_num = 0;
  float match_dis_sum = 0.0;
  float match_dis_ave = 0.99;
  float inlier_dis = config_->max_match_dis * 6.0;
  size_t cloud_size = aligned_curr_corner_cloud->points.size();
  for (size_t i = 0; i < cloud_size; ++i) {
    // Find its nearest neighbor in the target
    if (ptr_kdtree_corner_from_map_->nearestKSearch(
            aligned_curr_corner_cloud->points[i], 1, pointSearchInd,
            pointSearchSqDis) > 0) {
      // Deal with occlusions (incomplete targets)
      if (pointSearchSqDis[0] < inlier_dis) {
        float prob_i =
            ds_corner_cloud_from_map_->points[pointSearchInd[0]].intensity;
        if (prob_i > init_map_prob) {
          prob_i = init_map_prob;
        }
        float match_dis_i = pointSearchSqDis[0] * init_map_prob / prob_i;
        match_dis_sum += match_dis_i;
        if (prob_i > min_match_prob)
          inlier_num++;
      }
    }
  }

  if (inlier_num > 10) {
    match_dis_ave = match_dis_sum / inlier_num;
  }

  overlap_ratio_ = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio_ < 0.1) {
    overlap_ratio_ = 0.1;
  }

  // double overlap_trans = (overlap_ratio + 0.1) * (overlap_ratio + 0.1);
  double overlap_trans = overlap_ratio_ / 0.9;

  double overlap_score = overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  delete problem;

  // LOG(INFO) << "CeresPLICPMatch cost: " << match_cost.toc() << " ms";

  return true;
}

bool MapTrack::buildKdtree() {
  ds_corner_cloud_from_map_ = ptr_map_manager_->getCornerMapCloud();
  if (0 == ds_corner_cloud_from_map_->size()) {
    return false;
  }

  ptr_kdtree_corner_from_map_->setInputCloud(ds_corner_cloud_from_map_);
  is_kdtree_initialized_ = true;
  LOG(WARNING) << "Kdtree initialized OK" << std::endl;
  return true;
}

void MapTrack::clearCloud() {
  corner_cloud_from_map_->clear();
  surf_cloud_from_map_->clear();
  ds_curr_corner_cloud_->clear();
  ds_curr_surf_cloud_->clear();
}

pcl::PointCloud<PointType>::Ptr MapTrack::getCurrSurroundMap() {
  pcl::PointCloud<PointType>::Ptr surroundMapCloud(
      new pcl::PointCloud<PointType>());
  std::lock_guard<std::mutex> lock(curr_surround_map_mutex_);
  *surroundMapCloud = *ds_corner_cloud_from_map_;

  return surroundMapCloud;
}

Mat34d MapTrack::getMap2Odom() {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  return map_to_odom_;
}

void MapTrack::setMap2Odom(const Mat34d &map_to_odom, bool is_update_map) {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  map_to_odom_ = map_to_odom;
  last_map_to_odom_ = map_to_odom;
  map_odom_update_flag_ = true;
  // if (ptr_cur_keyframe_ != nullptr && is_update_map) {
  //   last_frame_pose_ = ptr_cur_keyframe_->getPose();
  //   LOG(INFO) << "setMap2Odom last_frame_pose_:" << last_frame_pose_;
  // }
}

bool MapTrack::isKeyFrame(bool use_rotation) {
  static double key_dis_threshold = config_->min_distance;
  static double key_rot_threshold = config_->angle_threshold;

  Mat34d cur_frame_pose = ptr_cur_keyframe_->getPose();

  double dis_to_map = cur_frame_pose.block<3, 1>(0, 3).norm();

  Mat34d delta_pose =
      Mathbox::deltaPose34d(last_keyframe_pose_, cur_frame_pose);
  float delta_dis = delta_pose.block<3, 1>(0, 3).norm();
  float delta_rot =
      Mathbox::rotationMatrixToEulerAngles(delta_pose.block<3, 3>(0, 0)).norm();

  if (delta_dis > key_dis_threshold || delta_rot > key_rot_threshold ||
      dis_to_map < 3.0) {
    last_keyframe_pose_ = cur_frame_pose;

    // if (config_->print_debug)
    // {
    //   static int count = 0;
    //   printf("--MapTrack::add keyFrame: count: %d, distance: %f, angle: %f
    //   \n", count++, distance, delta_angle / M_PI * 180);
    // }

    return true;
  } else {
    return false;
  }
}

void MapTrack::setPoseCov(const double cov) {
  pose_cov_ = cov;
}

bool MapTrack::isJunkFrame(const double time, const Mat34d &cur_odom_pose) {
  static double _last_frame_time = 0.0;

  Mat34d delta_odom_pose =
      Mathbox::deltaPose34d(last_keyframe_odom_pose_, cur_odom_pose);

  double delta_angle =
      Mathbox::rotationMatrixToEulerAngles(delta_odom_pose.block<3, 3>(0, 0))
          .norm();
  double delta_trans = delta_odom_pose.block<3, 1>(0, 3).norm();
  double delta_time = std::abs(time - _last_frame_time);

  // 运动距离或者角度超过设定值才进行优化；
  if (delta_angle > config_->angle_threshold ||
      delta_trans > config_->trans_threshold ||
      delta_time > config_->time_threshold) {
    last_keyframe_odom_pose_ = cur_odom_pose;
    _last_frame_time = time;
    return false;
  }

  return true;
}

void MapTrack::updateTransform() {
  static double mapping_match_smooth_ratio =
      config_->mapping_match_smooth_ratio;
  static double loc_match_smooth_ratio = config_->loc_match_smooth_ratio;
  std::unique_lock<std::mutex> lock(pose_mutex_);
  std::unique_lock<std::mutex> lock1(correct_pose_mutex_);

  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    ptr_cur_keyframe_->updatePose(cur_frame_pose_);
    ptr_cur_keyframe_->map_cov_ = cov_;

    // 利用地图匹配优化后的位姿和里程计位姿来计算最新的矫正值；
    map_to_odom_ = Mathbox::multiplePose34d(
        ptr_cur_keyframe_->getPose(),
        Mathbox::inversePose34d(ptr_cur_keyframe_->getLaserOdomPose()));

    Mat34d smooth_map_to_odom = Mathbox::Interp_SE3(
        last_map_to_odom_, map_to_odom_, mapping_match_smooth_ratio);  // 1.0
    map_to_odom_ = smooth_map_to_odom;

    last_map_to_odom_ = map_to_odom_;
  } else {
    ptr_cur_keyframe_->updatePose(cur_frame_pose_);
    ptr_cur_keyframe_->T_wm_ = cur_frame_pose_;
    ptr_cur_keyframe_->map_cov_ = cov_;

    // 利用地图匹配优化后的位姿和里程计位姿来计算最新的矫正值；
    map_to_odom_ = Mathbox::multiplePose34d(
        ptr_cur_keyframe_->getPose(),
        Mathbox::inversePose34d(ptr_cur_keyframe_->getLaserOdomPose()));

    if (ptr_cur_keyframe_->H_matrix_(3, 0) < config_->degenerate_threshold ||
        ptr_cur_keyframe_->H_matrix_(4, 0) < config_->degenerate_threshold) {
      LOG(ERROR) << "degenerate frame: " << ptr_cur_keyframe_->H_matrix_;
      loc_match_smooth_ratio = config_->degenerate_loc_match_smooth_ratio;
    } else {
      loc_match_smooth_ratio = config_->loc_match_smooth_ratio;
    }

    Mat34d smooth_map_to_odom = Mathbox::Interp_SE3(
        last_map_to_odom_, map_to_odom_, loc_match_smooth_ratio);  // 0.2
    map_to_odom_ = smooth_map_to_odom;

    last_map_to_odom_ = map_to_odom_;

    // 如果激光点与地图的匹配距离小，同时轮子里程计增量 与 匹配优化增量
    // 之间的差别较大，则认为轮子发生打滑；
    if (curr_distance_error_ < 0.4 && predict_error_ratio_ > 0.5) {
      LOG(ERROR) << "Wheel odom Skidding Capture!!";
    }
  }
}

void MapTrack::updateMapPointProbability() {
  static float increase_ratio = config_->increase_ratio;
  static float decrease_ratio = config_->decrease_ratio;
  static float increase_duration = config_->increase_duration;
  static float decrease_duration = config_->decrease_duration;
  static float static_duration = config_->static_duration;
  static float new_add_prob = config_->new_add_prob;
  static float min_erase_prob = config_->min_erase_prob;

  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    // 1.如果地图点被重复观测到，概率会随着重复观测而增加；
    // 2.如果地图点长时间没有被观测到，概率会随着时间流逝而消亡；
    // 3.新观测到的点以偏低的概率加到地图中，初始概率小于加入匹配的概率；

    TicToc map_update_cost;
    static pcl::PointCloud<PointType>::Ptr _new_added_corner(
        new pcl::PointCloud<PointType>());
    static pcl::PointCloud<PointType>::Ptr _observed_pose_cloud(
        new pcl::PointCloud<PointType>());
    static size_t _increase_count = 0;

    double curr_keyframe_time = ptr_cur_keyframe_->time_stamp_;

    static size_t _last_new_added_corner_size = _new_added_corner->size();
    size_t new_added_corner_size = _new_added_corner->size();

    if (new_added_corner_size > 1 &&
        _last_new_added_corner_size != new_added_corner_size) {
      ptr_kdtree_new_corner_map_->setInputCloud(_new_added_corner);
      _last_new_added_corner_size = new_added_corner_size;
    }

    // 找到当前帧中观测到的地图点和地图中没有的点；
    pcl::PointCloud<PointType>::Ptr observed_points_in_map(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr curr_corner(
        new pcl::PointCloud<PointType>());
    // 只在匹配成功的时候进行概率更新；
    if (is_match_ok_ && static_duration_ <= static_duration) {
      Mat34d curr_frame_pose = ptr_cur_keyframe_->getPose();
      curr_corner = Transformbox::transformPointCloud(
          curr_frame_pose, ptr_cur_keyframe_->corner_cloud_);
      _observed_pose_cloud->push_back(ptr_cur_keyframe_->getPosition());
      _increase_count++;
    }
    size_t curr_corner_size = curr_corner->size();

    static PointType curr_point;
    static PointType nearest_point;
    static std::vector<int> pointInd;
    static std::vector<float> pointSqDis;

    for (size_t i = 0; i < curr_corner_size; i++) {
      curr_point = curr_corner->points[i];

      if (ptr_kdtree_corner_from_map_->nearestKSearch(curr_point, 1, pointInd,
                                                      pointSqDis) > 0) {
        // 如果观测到地图附近点，则对最近点进行更新；
        if (pointSqDis[0] < 0.1) {
          curr_point.intensity = 80.0;                      // init probability
          curr_point.normal_x = curr_keyframe_time;         // update time
          curr_point.normal_y = i;                          // internal index
          curr_point.normal_z = ptr_cur_keyframe_->index_;  // keyFrame id
          observed_points_in_map->push_back(curr_point);

          nearest_point = ds_corner_cloud_from_map_->points[pointInd[0]];
          if (nearest_point.intensity < 100.0) {
            float time_ratio =
                std::abs(curr_keyframe_time - nearest_point.normal_x) /
                increase_duration;
            if (time_ratio > 1.0) {
              time_ratio = 1.0;
            }
            if (time_ratio < 0.1) {
              time_ratio = 0.1;
            }
            float delta_p = increase_ratio * time_ratio * 5.0;
            nearest_point.intensity += delta_p;
            nearest_point.normal_x = curr_keyframe_time;  // updata time
            ds_corner_cloud_from_map_->points[pointInd[0]] = nearest_point;
          }
        }
        // 如果距离地图点比较远，则认为可能是新增加的点；
        if (pointSqDis[0] > 0.2 && pointSqDis[0] < 20.0) {
          // 如果该点不在当前新增点云中，且最近距离大于分辨率，则将该点加入到新增点云；
          if (new_added_corner_size > 1) {
            if (ptr_kdtree_new_corner_map_->nearestKSearch(
                    curr_point, 1, pointInd, pointSqDis) > 0) {
              if (pointSqDis[0] < 0.03) {
                if (_new_added_corner->points[pointInd[0]].intensity < 60.0) {
                  _new_added_corner->points[pointInd[0]].intensity += 1.0;
                }
                continue;
              }
            }
          }

          // 根据距离进行添加新点，保证地图的稀疏性；
          size_t corner_map_size = ds_corner_cloud_from_map_->size();
          curr_point.intensity = new_add_prob;              // init probability
          curr_point.normal_x = curr_keyframe_time;         // update time
          curr_point.normal_y = corner_map_size;            // internal index
          curr_point.normal_z = ptr_cur_keyframe_->index_;  // keyFrame id
          _new_added_corner->push_back(curr_point);

          // new_corner_cloud_from_map_->push_back(curr_point);
          // ds_corner_cloud_from_map_->push_back(curr_point);
        }
      }
    }

    // 获取当前运行周期内浏览过的关键帧
    auto cur_frame_pos = ptr_cur_keyframe_->getPosition();
    std::vector<std::shared_ptr<KeyFrame>> near_keyframes;
    near_keyframes = ptr_map_manager_->GetNearKeyFrames(cur_frame_pos, 3.0);
    if (near_keyframes.empty() == true) {
      LOG(ERROR) << "updateMapProbability: not find near keyframe !";
      LOG(ERROR) << "updateMapProbability: cur_frame_pos: " << cur_frame_pos;
      return;
    }

    for (size_t i = 0; i < near_keyframes.size(); i++) {
      observed_keyframe_set_.insert(near_keyframes[i]->index_);
    }

    static double _last_key_dis = ptr_cur_keyframe_->moved_distance_;
    static double _last_key_time = ptr_cur_keyframe_->time_stamp_;
    static double _last_decrease_dis = ptr_cur_keyframe_->moved_distance_;
    static double _last_decrease_time = ptr_cur_keyframe_->time_stamp_;

    if (std::abs(ptr_cur_keyframe_->moved_distance_ - _last_key_dis) < 0.1) {
      double delta_static_time =
          std::abs(ptr_cur_keyframe_->time_stamp_ - _last_key_time);
      static_duration_ += delta_static_time;
    } else {
      static_duration_ = 0.0;
    }
    _last_key_dis = ptr_cur_keyframe_->moved_distance_;
    _last_key_time = ptr_cur_keyframe_->time_stamp_;

    // 静止时间超过阈值则：准备进行地图概率的遍历衰减；
    // 与上次进行地图概率衰减的时间间隔超过阈值：即保证地图衰减的时间频率；
    // 与上次进行地图概率衰减的运动距离超过阈值：即对环境进行过充分观测；
    // 当前周期内地图概率增长的次数超过阈值：即保证概率增长和下降的对等；
    // 才执行地图概率的遍历衰减操作；

    float observe_ratio = 0.8;
    if (static_duration_ >= static_duration) {
      LOG(WARNING) << "updateMapProbability: increase_count: "
                   << _increase_count;

      pcl::PointCloud<PointType>::Ptr key_pose_cloud =
          ptr_map_manager_->getKeyPoses();
      size_t key_pose_size = key_pose_cloud->size();
      if (std::abs(_last_decrease_time - curr_keyframe_time) >
              decrease_duration &&
          std::abs(_last_decrease_dis - _last_key_dis) >
              (0.3 * observe_ratio * key_pose_size) &&
          _increase_count > (observe_ratio * key_pose_size)) {
        LOG(WARNING) << "updateMapProbability: start map decrease !";

        pcl::PointCloud<PointType>::Ptr un_observed_points_in_map(
            new pcl::PointCloud<PointType>());
        std::vector<std::shared_ptr<KeyFrame>> v_keyframes =
            ptr_map_manager_->getAllKeyFrames();
        size_t key_size = v_keyframes.size();
        for (size_t i = 0; i < key_size; i++) {
          if (observed_keyframe_set_.find(v_keyframes[i]->index_) ==
              observed_keyframe_set_.end()) {
            Mat34d frame_pose = v_keyframes[i]->getPose();
            laserCloud::Ptr corner_map = Transformbox::transformPointCloud(
                frame_pose, v_keyframes[i]->corner_cloud_);
            *un_observed_points_in_map += *(corner_map);
          }
        }
        pcl::PointCloud<PointType>::Ptr ds_un_observed_points_in_map(
            new pcl::PointCloud<PointType>());
        downsize_filter_corner_.setInputCloud(un_observed_points_in_map);
        downsize_filter_corner_.filter(*ds_un_observed_points_in_map);

        // 避免点云为空
        curr_point.intensity = 40.0;
        curr_point.x = 0.0;
        curr_point.y = 0.0;
        ds_un_observed_points_in_map->push_back(curr_point);

        pclKdTree::Ptr ptr_kdtree_un_observed_map;
        ptr_kdtree_un_observed_map.reset(new pclKdTree());
        ptr_kdtree_un_observed_map->setInputCloud(ds_un_observed_points_in_map);

        pcl::PointCloud<PointType>::Ptr ds_observed_pose_cloud(
            new pcl::PointCloud<PointType>());
        pclDownsampler downsize_pose_cloud;
        downsize_pose_cloud.setLeafSize(0.5, 0.5, 0.5);
        downsize_pose_cloud.setInputCloud(_observed_pose_cloud);
        downsize_pose_cloud.filter(*ds_observed_pose_cloud);
        pclKdTree::Ptr ptr_kdtree_observed_pose_cloud;
        ptr_kdtree_observed_pose_cloud.reset(new pclKdTree());
        ptr_kdtree_observed_pose_cloud->setInputCloud(ds_observed_pose_cloud);

        // std::string map_path =
        //     "/home/suyun/CVTE/map/2chan_canting/2chan_canting_2/3d_map/";
        // pcl::io::savePCDFileBinaryCompressed(
        //     map_path + "aa_un_observed_map.pcd",
        //     *ds_un_observed_points_in_map);

        size_t corner_map_size = ds_corner_cloud_from_map_->size();
        for (size_t i = 0; i < corner_map_size; i++) {
          curr_point = ds_corner_cloud_from_map_->points[i];

          if (ptr_kdtree_observed_pose_cloud->nearestKSearch(
                  curr_point, 1, pointInd, pointSqDis) > 0) {
            if (pointSqDis[0] > 1.0) {
              if (ptr_kdtree_un_observed_map->nearestKSearch(
                      curr_point, 1, pointInd, pointSqDis) > 0) {
                if (pointSqDis[0] < 0.1)
                  continue;
              }
            }
          }

          float delta_p = decrease_ratio * 5.0;
          // 概率衰减时，只更新概率，不更新时间；
          ds_corner_cloud_from_map_->points[i].intensity -= delta_p;
          // ds_corner_cloud_from_map_->points[i].normal_x =
          // curr_keyframe_time;
          if (ds_corner_cloud_from_map_->points[i].intensity < min_erase_prob) {
            ds_corner_cloud_from_map_->erase(
                ds_corner_cloud_from_map_->begin() + i);
            corner_map_size = ds_corner_cloud_from_map_->size();
            i--;
          }
        }
        _last_decrease_time = curr_keyframe_time;
        _last_decrease_dis = ptr_cur_keyframe_->moved_distance_;
        _increase_count = 0;
        observed_keyframe_set_.clear();

        size_t new_added_corner_size = _new_added_corner->size();
        for (size_t i = 0; i < new_added_corner_size; i++) {
          curr_point = _new_added_corner->points[i];
          if (curr_point.intensity > (new_add_prob + 3.0)) {
            size_t corner_map_size = ds_corner_cloud_from_map_->size();
            curr_point.normal_x = curr_keyframe_time;         // update time
            curr_point.normal_y = corner_map_size;            // internal index
            curr_point.normal_z = ptr_cur_keyframe_->index_;  // keyFrame id
            ds_corner_cloud_from_map_->push_back(curr_point);
          }
        }
        _new_added_corner->clear();
        _observed_pose_cloud->clear();

        ptr_map_manager_->saveNewCornerCloudMap(ds_corner_cloud_from_map_);
        // ptr_map_manager_->saveNewAddedCornerMap(new_corner_cloud_from_map_);

        if (ds_corner_cloud_from_map_->size() > 0) {
          LOG(WARNING) << "Kdtree begin initialize ...";
          ptr_kdtree_corner_from_map_->setInputCloud(ds_corner_cloud_from_map_);
          is_kdtree_initialized_ = true;
          LOG(WARNING) << "Kdtree initialized OK .";
        } else {
          is_kdtree_initialized_ = false;
          LOG(ERROR) << std::endl;
          LOG(ERROR) << "Kdtree initialized failed !!!";
          LOG(ERROR) << std::endl;
        }
        LOG(WARNING) << "updateMapProbability: finish map decrease !";
      }
    }
    LOG(INFO) << "updateMapProbability: time cost: " << map_update_cost.toc()
              << " ms";
  }
}

laserCloud::Ptr MapTrack::getCornerMapCloud() {
  laserCloud::Ptr cloud(new laserCloud());
  *cloud = *ds_corner_cloud_from_map_;
  return cloud;
}

}  // namespace cvte_lidar_slam
