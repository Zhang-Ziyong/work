/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, CVTE.
 * All rights reserved.
 *
 *@file lidar_odom.cpp
 *
 *@brief
 * lidar odometry
 *
 *@author Yun Su(robosu12@gmail.com)
 *@version 0.8
 *@data 2022-04-27
 ************************************************************************/

#include "lidar_odom/lidar_odom.hpp"
#include <glog/logging.h>
#include "map/map_manager.hpp"
#include "msf/pose_graph_error_term.hpp"
#include "common/math_base/slam_transform.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/config/system_config.hpp"
#include "common/debug_tools/tic_toc.h"

namespace cvte_lidar_slam {
LidarOdom::LidarOdom(BackendConfig *config) : config_(config) {
  InitParameters();
  ptr_state_machine_ = SlamStateMachine::getInstance();
}

LidarOdom::~LidarOdom() {}

void LidarOdom::Reset() {
  match_failed_count_ = 0;
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
  is_match_stable_ = true;
  is_wheel_stable_ = true;

  d_recent_lidar_cloud_in_map_.clear();

  ptr_cur_keyframe_ = std::make_shared<KeyFrame>();
  ptr_last_keyframe_ = std::make_shared<KeyFrame>();

  ptr_kdtree_surf_from_map_.reset(new pclKdTree());
  ptr_kdtree_corner_from_map_.reset(new pclKdTree());

  corner_cloud_from_map_.reset(new laserCloud());
  surf_cloud_from_map_.reset(new laserCloud());
  ds_corner_cloud_from_map_.reset(new laserCloud());
  ds_surf_cloud_from_map_.reset(new laserCloud());
  ds_curr_corner_cloud_.reset(new laserCloud());
  ds_curr_surf_cloud_.reset(new laserCloud());
}

void LidarOdom::InitParameters() {
  ptr_cur_keyframe_ = std::make_shared<KeyFrame>();
  ptr_last_keyframe_ = std::make_shared<KeyFrame>();
  ptr_map_manager_ = MapManager::getInstance();

  ptr_kdtree_surf_from_map_.reset(new pclKdTree());
  ptr_kdtree_corner_from_map_.reset(new pclKdTree());

  corner_cloud_from_map_.reset(new laserCloud());
  surf_cloud_from_map_.reset(new laserCloud());
  ds_corner_cloud_from_map_.reset(new laserCloud());
  ds_surf_cloud_from_map_.reset(new laserCloud());
  ds_curr_corner_cloud_.reset(new laserCloud());
  ds_curr_surf_cloud_.reset(new laserCloud());

  downsize_filter_map_corner_.setLeafSize(
      config_->edge_size, config_->edge_size, config_->edge_size);
  downsize_filter_map_surf_.setLeafSize(config_->surf_size, config_->surf_size,
                                        config_->surf_size);
  downsize_filter_cur_corner_.setLeafSize(
      config_->edge_size, config_->edge_size, config_->edge_size);
  downsize_filter_cur_surf_.setLeafSize(config_->surf_size, config_->surf_size,
                                        config_->surf_size);

  Reset();
}

bool LidarOdom::processNewKeyFrame(std::shared_ptr<KeyFrame> ptr_cur_keyframe,
                                   bool is_update_map) {
  static double lidar_odom_match_smooth_ratio =
      config_->lidar_odom_match_smooth_ratio;

  if (nullptr == ptr_cur_keyframe) {
    LOG(ERROR) << "LidarOdom: ptr_cur_keyframe is nullptr !!! ";
    return false;
  }

  ptr_cur_keyframe_ = ptr_cur_keyframe;

  // 利用轮式里程计位姿进行预测；
  cur_frame_pose_ =
      Mathbox::multiplePose34d(map_to_odom_, ptr_cur_keyframe_->getWheelPose());

  ptr_cur_keyframe_->updateLaserOdomPose(cur_frame_pose_);

  // 每帧点云处理流程：
  // 1. 先对当前帧进行降采样；
  // 2. 然后搜索关键帧构建局部地图；
  // 3. 利用里程计预测的位姿，与地图匹配进行位姿求解；

  bool ds_flag = dowmSampleCurFrame();

  if (ds_flag == true) {
    scanToLocalMap();

    // TODO: smooth lidar_odom pose using raw wheel_odom pose;

    if (is_match_ok_) {
      match_failed_count_ = 0;

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

      // 用匹配优化后的位姿对里程计结果进行更新；
      ptr_cur_keyframe_->updateLaserOdomPose(cur_frame_pose_);
      ptr_cur_keyframe_->laser_odom_cov_ = Mat6d::Identity() * 0.02;

      map_to_odom_ = Mathbox::multiplePose34d(
          cur_frame_pose_,
          Mathbox::inversePose34d(ptr_cur_keyframe_->getWheelPose()));

      if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode() &&
          (ptr_cur_keyframe_->H_matrix_(3, 0) < config_->degenerate_threshold ||
           ptr_cur_keyframe_->H_matrix_(4, 0) <
               config_->degenerate_threshold)) {
        LOG(ERROR) << "degenerate frame: " << ptr_cur_keyframe_->H_matrix_;
        lidar_odom_match_smooth_ratio =
            config_->degenerate_lidar_odom_smooth_ratio;
      } else {
        lidar_odom_match_smooth_ratio = config_->lidar_odom_match_smooth_ratio;
      }
      Mat34d smooth_map_to_odom =
          Mathbox::Interp_SE3(last_map_to_odom_, map_to_odom_,
                              lidar_odom_match_smooth_ratio);  // 1.0
      map_to_odom_ = smooth_map_to_odom;

      last_map_to_odom_ = map_to_odom_;
    } else {
      match_failed_count_++;
      ptr_cur_keyframe_->laser_odom_cov_ =
          (0.02 * match_failed_count_ + 0.02) * Mat6d::Identity();
      LOG(ERROR) << "lidar_odom match failed ! count: " << match_failed_count_;
    }

    pose_cov_ = ptr_cur_keyframe_->laser_odom_cov_(3, 3);  //
    if (pose_cov_ > 1.0) {
      LOG(ERROR) << "lidar_odom - Unstable lidar_odom pose ! cov: "
                 << pose_cov_;
    }

    last_frame_odom_pose_ = ptr_cur_keyframe_->getWheelPose();
    last_frame_pose_ = ptr_cur_keyframe_->getLaserOdomPose();

    if (isKeyFrame() || is_first_keyframe_) {
      updateLocalMap();
    }

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
      LOG(WARNING) << "cur laser odom pose (" << cur_Lodom_pos.transpose().x()
                   << " , " << cur_Lodom_pos.transpose().y() << " , "
                   << cur_Lodom_rpy.transpose()[2] << ")"
                   << " cur wheel odom pose (" << cur_wheel_pos.transpose().x()
                   << " , " << cur_wheel_pos.transpose().y() << " , "
                   << cur_wheel_rpy.transpose()[2] << ")";
    } else {
      LOG(ERROR) << "cur laser odom pose (" << cur_Lodom_pos.transpose().x()
                 << " , " << cur_Lodom_pos.transpose().y() << " , "
                 << cur_Lodom_rpy.transpose()[2] << ")"
                 << " cur wheel odom pose (" << cur_wheel_pos.transpose().x()
                 << " , " << cur_wheel_pos.transpose().y() << " , "
                 << cur_wheel_rpy.transpose()[2] << ")";
    }

    // clearCloud();
  } else {
    return false;
  }

  return is_match_ok_;
}

bool LidarOdom::dowmSampleCurFrame() {
  ds_curr_corner_cloud_->clear();
  ds_curr_surf_cloud_->clear();

  *ds_curr_corner_cloud_ = *(ptr_cur_keyframe_->corner_cloud_);
  curr_corner_cloud_num_ = ds_curr_corner_cloud_->size();

  if (curr_corner_cloud_num_ < config_->min_feature_points) {
    LOG(ERROR) << "Too few lidar points:  " << curr_corner_cloud_num_;
    return false;
  }

  return true;
}

void LidarOdom::scanToLocalMap() {
  static double max_map_match_distance =
      config_->max_map_match_success_score * 6.0;
  static double odom_match_ratio = config_->odom_match_ratio;

  // 判断地图角点数量
  if (ds_corner_cloud_from_map_->size() > config_->min_feature_points) {
    is_match_ok_ = true;
    int max_iteration = 3;

    // if (config_->match_algorithm == 2 || config_->match_algorithm == 3) {
    //   max_iteration = max_iteration * 2;
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
      LOG(WARNING) << "LidarOdom: iterCount: " << iterCount
                   << "; delta_dis: " << delta_trans
                   << ", delta_rot: " << delta_rotation << std::endl;
      // LOG(WARNING) << "iterCount: " << iterCount;

      if (delta_rotation < 0.002 && delta_trans < 0.01) {
        if (curr_distance_error_ > max_map_match_distance) {
          is_match_ok_ = false;
          LOG(ERROR) << "LidarOdom: Too big match distance: "
                     << curr_distance_error_;
        }
        break;
      } else if ((max_iteration - 1) == iterCount) {
        if (curr_distance_error_ > max_map_match_distance) {
          is_match_ok_ = false;
          LOG(ERROR) << "LidarOdom: reach max_iteration ! big match distance: "
                     << curr_distance_error_;

          // LOG(WARNING) << "real distance error " << curr_distance_error_;
          // LOG(WARNING) << "map_to_odom_: " << map_to_odom_;
          // LOG(WARNING) << "opt cur_frame_pose_: " << cur_frame_pose_;
          // LOG(WARNING) << "bef cur_frame_pose_: " <<
        }
      }
    }

    const Mat34d &delta_odom_pose = Mathbox::deltaPose34d(
        last_frame_odom_pose_, ptr_cur_keyframe_->getWheelPose());
    const Mat34d &delta_match_pose =
        Mathbox::deltaPose34d(last_frame_pose_, cur_frame_pose_);

    double d_dis_odom = delta_odom_pose.block<3, 1>(0, 3).norm();
    double d_rot_odom =
        Mathbox::rotationMatrixToEulerAngles(delta_odom_pose.block<3, 3>(0, 0))
            .norm();

    double d_dis_match = delta_match_pose.block<3, 1>(0, 3).norm();
    double d_rot_match =
        Mathbox::rotationMatrixToEulerAngles(delta_match_pose.block<3, 3>(0, 0))
            .norm();

    double predict_dis_ratio =
        std::abs(d_dis_match - d_dis_odom) / config_->trans_threshold;
    double predict_rot_ratio =
        std::abs(d_rot_match - d_rot_odom) / config_->angle_threshold;

    is_match_stable_ = true;
    is_wheel_stable_ = true;
    if (predict_dis_ratio > 0.5) {
      if (d_dis_match > d_dis_odom) {
        is_match_stable_ = false;
      } else {
        is_wheel_stable_ = false;
      }
    }
    if (predict_rot_ratio > 0.5) {
      if (d_rot_match > d_rot_odom) {
        is_match_stable_ = false;
      } else {
        is_wheel_stable_ = false;
      }
    }

    Mat34d delta_pose_accept = Mathbox::Interp_SE3(
        delta_odom_pose, delta_match_pose, odom_match_ratio);  // 1.0
    cur_frame_pose_ =
        Mathbox::multiplePose34d(last_frame_pose_, delta_pose_accept);

    if (is_match_stable_ == false) {
      static int count = 0;
      is_match_ok_ = false;
      LOG(ERROR) << "LidarOdom: big gap between odom and match "
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! count:  "
                 << count++;
      LOG(ERROR) << "map match distance : " << curr_distance_error_;
      LOG(ERROR) << "predict_dis_ratio: " << predict_dis_ratio
                 << ", predict_rot_ratio: " << predict_rot_ratio;
      LOG(ERROR) << "d_dis_odom : " << d_dis_odom
                 << ", d_rot_odom: " << d_rot_odom;
      LOG(ERROR) << "d_dis_match: " << d_dis_match
                 << ", d_rot_match: " << d_rot_match;
    } else {
      LOG(INFO) << "LidarOdom: map match distance: " << curr_distance_error_;
    }

    if (is_wheel_stable_ == false) {
      LOG(ERROR) << "LidarOdom: wheel slipping "
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ";
      LOG(ERROR) << "map match distance : " << curr_distance_error_;
      LOG(ERROR) << "predict_dis_ratio: " << predict_dis_ratio
                 << ", predict_rot_ratio: " << predict_rot_ratio;
      LOG(ERROR) << "d_dis_odom : " << d_dis_odom
                 << ", d_rot_odom: " << d_rot_odom;
      LOG(ERROR) << "d_dis_match: " << d_dis_match
                 << ", d_rot_match: " << d_rot_match;
    }
  } else {
    LOG(ERROR) << "LidarOdom: too small map points: "
               << ds_corner_cloud_from_map_->size();
    is_match_ok_ = false;
    if (is_first_keyframe_) {
      LOG(ERROR) << "LidarOdom: Happened in first keyframe !!!";
    }
  }
}

bool LidarOdom::updateLocalMap() {
  static int keyframe_count = 0;
  static int local_window_size = config_->surround_search_num;

  TicToc updateLocalMap;

  laserCloud::Ptr ds_curr_corner_cloud_in_map =
      Transformbox::transformPointCloud(ptr_cur_keyframe_->getLaserOdomPose(),
                                        ds_curr_corner_cloud_);

  if (d_recent_lidar_cloud_in_map_.size() > local_window_size) {
    d_recent_lidar_cloud_in_map_.pop_front();
  }
  d_recent_lidar_cloud_in_map_.emplace_back(ds_curr_corner_cloud_in_map);

  keyframe_count_++;
  if (keyframe_count_ % 3 == 1 || is_first_keyframe_) {
    if (is_first_keyframe_) {
      is_first_keyframe_ = false;
      LOG(WARNING) << "LidarOdom: Insert first keyframe !!!";
    }

    // 将滑动窗口中点云累加起来；
    laserCloud::Ptr CloudMap(new laserCloud());
    for (int i = 0; i < d_recent_lidar_cloud_in_map_.size(); i++) {
      *CloudMap += *d_recent_lidar_cloud_in_map_[i];  //
    }

    // 对局部点云地图进行降采样；
    ds_corner_cloud_from_map_->clear();
    ds_surf_cloud_from_map_->clear();

    downsize_filter_map_corner_.setInputCloud(CloudMap);
    downsize_filter_map_corner_.filter(*ds_corner_cloud_from_map_);
    ds_corner_from_map_num_ = ds_corner_cloud_from_map_->size();

    if (config_->match_algorithm == 2 || config_->match_algorithm == 3) {
      ptr_kdtree_corner_from_map_->setInputCloud(ds_corner_cloud_from_map_);
    }
  }

  // LOG(INFO) << "updateLocalMap time cost: " << updateLocalMap.toc() ;
  return true;
}

bool LidarOdom::getSurroundingKeyFrames() {
  TicToc get_surround;

  static unsigned int search_method =
      2;  // 1:search_by_time; 2:search_by_distance;

  FramePosition cur_frame_pos;
  Mat34d keyframe_pose = ptr_cur_keyframe_->getLaserOdomPose();
  cur_frame_pos.x = keyframe_pose(0, 3);
  cur_frame_pos.y = keyframe_pose(1, 3);
  cur_frame_pos.z = keyframe_pose(2, 3);
  cur_frame_pos.intensity = ptr_cur_keyframe_->index_;

  std::vector<std::shared_ptr<KeyFrame>> surroundingKeyFrames;

  if (search_method == 1)  // 找时间最近的帧构建局部地图；
  {
    surroundingKeyFrames =
        ptr_map_manager_->DetectCandidatesByTime(config_->surround_search_num);
  } else if (search_method == 2)  // 根据空间距离范围内的帧构建局部地图；
  {
    surroundingKeyFrames = ptr_map_manager_->DetectCandidatesByDistanceNewMap(
        cur_frame_pos, config_->surround_search_radius);
  }

  if (surroundingKeyFrames.empty()) {
    LOG(ERROR) << "LidarOdom: surround keyframes Empty !!! ";
    return false;
  }

  corner_cloud_from_map_->clear();
  surf_cloud_from_map_->clear();

  unsigned int surroundingKeyFramesNums = surroundingKeyFrames.size();

  // 根据位姿拼接地图；
  for (unsigned int i = 0; i < surroundingKeyFramesNums; i++) {
    Mat34d it_frame_pose = surroundingKeyFrames[i]->getLaserOdomPose();

    laserCloud::Ptr surround_corner_cloud = Transformbox::transformPointCloud(
        it_frame_pose, surroundingKeyFrames[i]->corner_cloud_);

    *corner_cloud_from_map_ += *surround_corner_cloud;
  }

  ds_corner_cloud_from_map_->clear();
  ds_surf_cloud_from_map_->clear();

  // 下采样
  // Downsample the surrounding corner key frames (or map)
  downsize_filter_map_corner_.setInputCloud(corner_cloud_from_map_);
  downsize_filter_map_corner_.filter(*ds_corner_cloud_from_map_);
  ds_corner_from_map_num_ = ds_corner_cloud_from_map_->points.size();

  return true;
}

void LidarOdom::PCLICPMatch() {
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

  if (ptr_cur_keyframe_->environment_flag_ == 1) {
    max_match_dis = 0.7 * max_match_dis;
  }

  // ICP Settings
  icp_tmp.setMaxCorrespondenceDistance(
      max_match_dis);  // 0.26的取值原则：地图的分辨率为0.1，大于0.25的距离能保证找到最近的3个点；
  icp_tmp.setMaximumIterations(max_iter);
  icp_tmp.setTransformationEpsilon(1e-6);
  icp_tmp.setEuclideanFitnessEpsilon(1e-6);
  // icp_tmp.setRANSACIterations(0);

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

  double overlap_ratio = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio < 0.1) {
    overlap_ratio = 0.1;
  }

  // double overlap_trans = (overlap_ratio + 0.1) * (overlap_ratio + 0.1);
  double overlap_trans = overlap_ratio / 0.9;

  double overlap_score = overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  if (icp_tmp.hasConverged() == false)
  // if (alignDistance > 0.4 || std::abs(dis) > 0.3 || std::abs(dyaw) > 0.1)
  {
    LOG(ERROR) << "LidarOdom: icp not converge, match_dis: "
               << curr_distance_error_ << ", time: " << t_ICPAlign.toc();
  }

  _T_w_updated = dT_correct * _T_w_init;  // 得到位姿修正量，累加得到当前位姿；

  _t_w_curr = _T_w_updated.translation().cast<double>();
  _r_w_curr = _T_w_updated.rotation().cast<double>();

  cur_frame_pose_.block<3, 3>(0, 0) = _r_w_curr;
  cur_frame_pose_.block<3, 1>(0, 3) = _t_w_curr;
}

bool LidarOdom::CeresICPMatch() {
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

  double max_match_dis = config_->max_match_dis;
  if (ptr_cur_keyframe_->environment_flag_ == 1) {
    max_match_dis = 0.7 * max_match_dis;
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

    if (ptr_kdtree_corner_from_map_->nearestKSearch(pointOri, 1, pointSearchInd,
                                                    pointSearchSqDis) > 0) {
      // if (map_point_index_set.find(pointSearchInd[0]) !=
      //     map_point_index_set.end()) {
      //   // LOG(ERROR) << "LidarOdom::CeresICPMatch: already used point !";
      //   continue;
      // }

      // map_point_index_set.insert(pointSearchInd[0]);

      if (pointSearchSqDis[0] < max_match_dis) {
        PCLPoint[0] = ds_corner_cloud_from_map_->points[pointSearchInd[0]];
        near_map_point[0] = Vec3d(PCLPoint[0].x, PCLPoint[0].y, PCLPoint[0].z);

        Vec3d lidar_point(pointOri.x, pointOri.y, pointOri.z);

        double distance = pointSearchSqDis[0];

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

  LOG(INFO) << "LidarOdom::CeresICPMatch: valid_corners: " << valid_corners
            << ", localMap: " << ds_corner_cloud_from_map_->size();

  if (valid_corners < config_->min_feature_points) {
    LOG(ERROR)
        << "LidarOdom::CeresICPMatch: Too small valid_corners in current "
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

  double overlap_ratio = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio < 0.1) {
    overlap_ratio = 0.1;
  }

  // double overlap_trans = (overlap_ratio + 0.1) * (overlap_ratio + 0.1);
  double overlap_trans = overlap_ratio / 0.9;

  double overlap_score = overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  delete problem;

  // LOG(INFO) << "CeresICPMatch cost: " << match_cost.toc() << " ms";

  return true;
}

bool LidarOdom::CeresPLICPMatch() {
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

  double max_match_dis = config_->max_match_dis;
  if (ptr_cur_keyframe_->environment_flag_ == 1) {
    max_match_dis = 0.7 * max_match_dis;
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

    if (ptr_kdtree_corner_from_map_->nearestKSearch(pointOri, 2, pointSearchInd,
                                                    pointSearchSqDis) > 1) {
      // if (map_point_index_set.find(pointSearchInd[0]) !=
      //     map_point_index_set.end()) {
      //   // LOG(ERROR) << "LidarOdom::CeresICPMatch: already used point !";
      //   continue;
      // }

      // map_point_index_set.insert(pointSearchInd[0]);

      if (pointSearchSqDis[1] < max_match_dis) {
        PCLPoint[0] = ds_corner_cloud_from_map_->points[pointSearchInd[0]];
        PCLPoint[1] = ds_corner_cloud_from_map_->points[pointSearchInd[1]];
        near_map_point[0] = Vec3d(PCLPoint[0].x, PCLPoint[0].y, PCLPoint[0].z);
        near_map_point[1] = Vec3d(PCLPoint[1].x, PCLPoint[1].y, PCLPoint[1].z);

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

  LOG(INFO) << "LidarOdom::CeresPLICPMatch: valid_corners: " << valid_corners
            << ", localMap: " << ds_corner_cloud_from_map_->size();

  if (valid_corners < config_->min_feature_points) {
    LOG(ERROR)
        << "LidarOdom::CeresPLICPMatch: Too small valid_corners in current "
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

  double overlap_ratio = 1.0 * inlier_num / cloud_size;
  if (overlap_ratio < 0.1) {
    overlap_ratio = 0.1;
  }

  // double overlap_trans = (overlap_ratio + 0.1) * (overlap_ratio + 0.1);
  double overlap_trans = overlap_ratio / 0.9;

  double overlap_score = overlap_trans * overlap_trans;

  curr_distance_error_ = match_dis_ave / overlap_score;

  delete problem;

  // LOG(INFO) << "CeresPLICPMatch cost: " << match_cost.toc() << " ms";

  return true;
}

void LidarOdom::clearCloud() {
  corner_cloud_from_map_->clear();
  surf_cloud_from_map_->clear();
  ds_curr_corner_cloud_->clear();
  ds_curr_surf_cloud_->clear();
}

pcl::PointCloud<PointType>::Ptr LidarOdom::getCurrSurroundMap() {
  pcl::PointCloud<PointType>::Ptr surroundMapCloud(
      new pcl::PointCloud<PointType>());
  std::lock_guard<std::mutex> lock(curr_surround_map_mutex_);
  *surroundMapCloud = *ds_corner_cloud_from_map_;

  return surroundMapCloud;
}

Mat34d LidarOdom::getMap2Odom() {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  return map_to_odom_;
}

void LidarOdom::setMap2Odom(const Mat34d &map_to_odom) {
  std::lock_guard<std::mutex> lock(correct_pose_mutex_);
  map_to_odom_ = map_to_odom;
  last_map_to_odom_ = map_to_odom;
}

bool LidarOdom::isKeyFrame(bool use_rotation) {
  static double key_dis_threshold = config_->min_distance * 2.0;
  static double key_rot_threshold = config_->angle_threshold * 2.0;

  Mat34d cur_frame_pose = ptr_cur_keyframe_->getLaserOdomPose();

  Mat34d delta_pose =
      Mathbox::deltaPose34d(last_keyframe_pose_, cur_frame_pose);
  float delta_distance = delta_pose.block<3, 1>(0, 3).norm();
  float delta_angle =
      Mathbox::rotationMatrixToEulerAngles(delta_pose.block<3, 3>(0, 0)).norm();

  if (delta_distance > key_dis_threshold || delta_angle > key_rot_threshold) {
    last_keyframe_pose_ = cur_frame_pose;

    return true;
  } else {
    return false;
  }
}

bool LidarOdom::isJunkFrame(const double time, const Mat34d &cur_odom_pose) {
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

}  // namespace cvte_lidar_slam
