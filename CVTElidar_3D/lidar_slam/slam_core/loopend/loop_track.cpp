#include "loopend/loop_track.hpp"

#include <iostream>
#include "glog/logging.h"

#include "map/map_manager.hpp"
#include "msf/data_type.hpp"
#include "frontend/feature_extractor.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/config/system_config.hpp"
#include "common/math_base/slam_transform.hpp"
#include "common/registration/cloud_registration.hpp"
#include "common/registration/loam_registration.hpp"

namespace cvte_lidar_slam {
double getSystemTime() {
  struct timespec timespec_now;
  clock_gettime(CLOCK_REALTIME, &timespec_now);
  return timespec_now.tv_sec + timespec_now.tv_nsec * (1.0 * 1e-9);
}

LoopTrack::LoopTrack(LoopConfig *config) {
  loop_config_ = *config;
  is_finish_requested_ = false;
  is_finished_ = false;
  is_new_keyframe_ = false;
  is_request_stop_ = false;
  is_reset_requested_ = false;
  is_stop_ = false;
  stop_icp_ = false;
  l_ptr_keyframe_.clear();
  v_ptr_history_keyframe_.clear();
  downsize_filter_history_cloud_.setLeafSize(0.2, 0.2, 0.2);
  downsize_filter_cur_cloud_.setLeafSize(0.4, 0.4, 0.4);
  ptr_cur_keycloud_.reset(new laserCloud);
  ptr_history_keycloud_.reset(new laserCloud);
  ptr_ds_cur_keycloud_.reset(new laserCloud);
  ptr_ds_history_keycloud_.reset(new laserCloud);
  ptr_map_manager_ = MapManager::getInstance();
  ptr_loam_registration_ = std::make_shared<LoamRegistration>();

  T_lo_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
}

LoopTrack::~LoopTrack() {
  is_stop_ = true;
}

void LoopTrack::Reset() {
  is_finish_requested_ = false;
  is_finished_ = false;
  is_new_keyframe_ = false;
  is_request_stop_ = false;
  is_reset_requested_ = false;
  is_stop_ = false;
  stop_icp_ = false;
  l_ptr_keyframe_.clear();
  v_ptr_history_keyframe_.clear();
  std::vector<std::shared_ptr<KeyFrame>>().swap(v_ptr_history_keyframe_);
}

void LoopTrack::insertKeyFrame(std::shared_ptr<KeyFrame> ptr_keyframe) {
  // std::unique_lock<std::mutex> lock(keyframe_mutex_);
  // l_ptr_keyframe_.emplace_back(ptr_keyframe);
  ptr_cur_keyframe_ = ptr_keyframe;
  is_new_keyframe_ = true;
}

bool LoopTrack::checkNewKeyFrames() {
  // std::unique_lock<std::mutex> lock(keyframe_mutex_);
  if (is_request_stop_) {
    return false;
  }
  if (is_new_keyframe_) {
    is_new_keyframe_ = false;
    return true;
  } else {
    return false;
  }
}

bool LoopTrack::detectLoop(const Mat34d &cur_pose) {
  std::unique_lock<std::mutex> lock(keyframe_mutex_);
  FramePosition pose;
  pose.x = cur_pose(0, 3);
  pose.y = cur_pose(1, 3);
  pose.z = cur_pose(2, 3);
  v_ptr_history_keyframe_ = ptr_map_manager_->DetectCandidatesByDistance(
      pose, loop_config_.loop_search_radius);
  history_frame_id_ = -1;
  if (!v_ptr_history_keyframe_.empty()) {
    for (auto it = v_ptr_history_keyframe_.begin();
         it != v_ptr_history_keyframe_.end(); it++) {
      // Mat34d key_frame_pose = (*it)->getPose();
      // Mat3d delta_rotation =
      // cur_pose.block<3,3>(0,0).inverse()*key_frame_pose.block<3,3>(0,0);
      // Vec3d ypr = Mathbox::rotationMatrixToEulerAngles(delta_rotation);
      if (fabs(ptr_cur_keyframe_->time_stamp_ - (*it)->time_stamp_) >
              loop_config_.time_diff &&
          fabs(ptr_cur_keyframe_->index_ - (*it)->index_) >
              loop_config_.id_diff) {
        history_frame_id_ = (*it)->index_;
        break;
      }
    }
  }
  if (history_frame_id_ == -1) {
    return false;
  } else {
    // std::cout << "history frame id:" << history_frame_id_ << std::endl;
    return true;
  }
}
long int LoopTrack::getLoopCandidateID(const Mat34d &cur_pose) {
  FramePosition pose;
  long int history_frame_id = -1;
  pose.x = cur_pose(0, 3);
  pose.y = cur_pose(1, 3);
  pose.z = cur_pose(2, 3);
  v_ptr_history_keyframe_ = ptr_map_manager_->DetectCandidatesByDistance(
      pose, loop_config_.loop_search_radius);

  if (!v_ptr_history_keyframe_.empty()) {
    for (auto it = v_ptr_history_keyframe_.begin();
         it != v_ptr_history_keyframe_.end(); it++) {
      // Mat34d key_frame_pose = (*it)->getPose();
      // Mat3d delta_rotation =
      // cur_pose.block<3,3>(0,0).inverse()*key_frame_pose.block<3,3>(0,0);
      // Vec3d ypr = Mathbox::rotationMatrixToEulerAngles(delta_rotation);
      if (fabs(ptr_cur_keyframe_->time_stamp_ - (*it)->time_stamp_) >
              loop_config_.time_diff &&
          fabs(ptr_cur_keyframe_->index_ - (*it)->index_) >
              loop_config_.id_diff) {
        history_frame_id = (*it)->index_;
        break;
      }
    }
  }
  return history_frame_id;
}

bool LoopTrack::correctLoop(const size_t history_frame_id, double &cov,
                            const Mat34d &pose, Mat34d &transformation,
                            size_t last_msf_id) {
  ptr_cur_keycloud_->clear();
  ptr_history_keycloud_->clear();
  ptr_ds_cur_keycloud_->clear();
  ptr_ds_history_keycloud_->clear();
  std::string regist_type;
  if (ptr_cur_keyframe_->index_ - last_msf_id > 50) {
    regist_type = "icp";
  } else {
    regist_type = "loam";
  }

  laserCloud::Ptr ptr_cur_surf_points(new laserCloud());
  laserCloud::Ptr ptr_cur_corner_points(new laserCloud());

  std::vector<std::shared_ptr<KeyFrame>> v_ptr_cur_keyframe_ =
      ptr_map_manager_->DetectCandidatesByTime(loop_config_.last_keyframe_size);
  for (auto key_it = v_ptr_cur_keyframe_.begin();
       key_it != v_ptr_cur_keyframe_.end(); key_it++) {
    Mat34d cur_frame_pose = (*key_it)->getPose();
    laserCloud::Ptr curCloudCorner = Transformbox::transformPointCloud(
        cur_frame_pose, (*key_it)->corner_cloud_);
    laserCloud::Ptr curCloudSurf = Transformbox::transformPointCloud(
        cur_frame_pose, (*key_it)->surf_cloud_);
    if (regist_type == "loam") {
      *ptr_cur_surf_points += *(curCloudSurf);
      *ptr_cur_corner_points += *(curCloudCorner);
    } else {
      *ptr_cur_keycloud_ += *(curCloudCorner);
      *ptr_cur_keycloud_ += *(curCloudSurf);
    }
  }
  size_t search_num;
  if (regist_type == "loam") {
    search_num = loop_config_.history_search_num / 2;
  } else {
    search_num = loop_config_.history_search_num;
  }
  std::vector<std::shared_ptr<KeyFrame>> history_candidate =
      ptr_map_manager_->DetectCandidatesByIndex(history_frame_id, search_num);
  laserCloud::Ptr history_surf_points(new laserCloud());
  laserCloud::Ptr history_edge_points(new laserCloud());
  for (auto key_it = history_candidate.begin();
       key_it != history_candidate.end(); key_it++) {
    Mat34d cur_frame_pose = (*key_it)->getPose();
    laserCloud::Ptr surroundingCloudCorner = Transformbox::transformPointCloud(
        cur_frame_pose, (*key_it)->corner_cloud_);
    laserCloud::Ptr surroundingCloudSurf = Transformbox::transformPointCloud(
        cur_frame_pose, (*key_it)->surf_cloud_);
    if (regist_type == "loam") {
      *history_surf_points += *(surroundingCloudSurf);
      *history_edge_points += *(surroundingCloudCorner);
    } else {
      *ptr_history_keycloud_ += *(surroundingCloudCorner);
      *ptr_history_keycloud_ += *(surroundingCloudSurf);
    }
  }
  downsize_filter_history_cloud_.setInputCloud(ptr_history_keycloud_);
  downsize_filter_history_cloud_.filter(*ptr_ds_history_keycloud_);
  double cov_result;
  if (regist_type == "icp") {
    cov_result =
        cloudRegistration("icp", ptr_cur_keycloud_, ptr_ds_history_keycloud_,
                          loop_config_.icp_score, transformation_);
  } else if (regist_type == "loam") {
    LOG(ERROR) << "****************loam******************";
    transformation = Mathbox::Identity34();
    bool ret = ptr_loam_registration_->loam_registration(
        ptr_cur_corner_points, ptr_cur_surf_points, history_surf_points,
        history_edge_points, transformation);
    if (ret) {
      cov_result = 0.2;
    } else {
      cov_result = -1;
    }
  }
  if (cov_result > 0.) {
    cov = cov_result;
    if (regist_type == "icp") {
      transformation = transformation_.block<3, 4>(0, 0);
    }
    LOG(INFO) << "loop transformation: " << std::endl
              << transformation << std::endl;
    return true;
  } else {
    LOG(WARNING) << "icp failed";
    cov = 10000;
    return false;
  }
}

bool LoopTrack::reLocalization(const Vec3d &vec3d_pose,
                               const std::shared_ptr<KeyFrame> pr_frame,
                               const double pre_yaw, Mat34d &pose34d) {
  if (!is_stop_) {
    FramePosition pose;
    pose.x = vec3d_pose(0);
    pose.y = vec3d_pose(1);
    pose.z = vec3d_pose(2);

    v_ptr_history_keyframe_ =
        ptr_map_manager_->DetectCandidatesByDistance(pose, 10.0);
    if (v_ptr_history_keyframe_.empty()) {
      LOG(ERROR) << "history keyframe was empty";
      return false;
    }

    laserCloud::Ptr candidate_cloud(new laserCloud);
    candidate_cloud->clear();
    for (auto key_it = v_ptr_history_keyframe_.begin();
         key_it != v_ptr_history_keyframe_.end(); key_it++) {
      Mat34d cur_frame_pose = (*key_it)->getPose();
      laserCloud::Ptr curCloudCorner = Transformbox::transformPointCloud(
          cur_frame_pose, (*key_it)->corner_cloud_);
      laserCloud::Ptr curCloudSurf = Transformbox::transformPointCloud(
          cur_frame_pose, (*key_it)->surf_cloud_);

      *candidate_cloud += *curCloudCorner;
      *candidate_cloud += *curCloudSurf;
    }

    laserCloud::Ptr ds_candidate_cloud(new laserCloud);
    downsize_filter_history_cloud_.setInputCloud(candidate_cloud);
    downsize_filter_history_cloud_.filter(*ds_candidate_cloud);
    int count = 0;
    if (stop_icp_) {
      stop_icp_ = false;
      return false;
    }
    double t1 = getSystemTime();
    Vec3d euler_angle;
    euler_angle(0) = pre_yaw;
    euler_angle(1) = 0;
    euler_angle(2) = 0;
    Mat34d curTrans34d;
    curTrans34d = Mathbox::Euler2Mat34d(euler_angle, vec3d_pose);

    laserCloud::Ptr curCloudInMap(new laserCloud());
    curCloudInMap->clear();

    laserCloud::Ptr curCloudCornerInMap =
        Transformbox::transformPointCloud(curTrans34d, pr_frame->corner_cloud_);
    laserCloud::Ptr curCloudSurfInMap =
        Transformbox::transformPointCloud(curTrans34d, pr_frame->surf_cloud_);
    laserCloud::Ptr curCloudoutlerInMap = Transformbox::transformPointCloud(
        curTrans34d, pr_frame->without_ground_cloud_);

    *curCloudInMap += *curCloudCornerInMap;
    *curCloudInMap += *curCloudSurfInMap;
    double t2 = getSystemTime();
    Mat4d transformation;
    transformation.setZero();
    transformation.block<3, 3>(0, 0).setIdentity();
    if (cloudRegistration("icp", curCloudInMap, ds_candidate_cloud,
                          loop_config_.relocalization_icp_score,
                          transformation) > 0) {
      Mat34d transform_in_odom =
          Mathbox::multiplePose34d(T_lo_, transformation.block<3, 4>(0, 0));
      transform_in_odom = Mathbox::multiplePose34d(
          transform_in_odom, Mathbox::inversePose34d(T_lo_));
      pose34d = Mathbox::multiplePose34d(transform_in_odom, curTrans34d);
      LOG(INFO) << "transformation: + " << transformation << std::endl;
      LOG(INFO) << "init-match-pose: " << pose34d;
      return true;
    }
    double t3 = getSystemTime();
    count++;
    return false;
  }
  return false;
}

bool LoopTrack::reLocalization(const Vec3d &vec3d_pose,
                               const laserCloud::Ptr &cur_cloud,
                               const double pre_yaw, Mat34d &pose34d) {
  if (!is_stop_) {
    if (cur_cloud->points.empty()) {
      return false;
    }

    auto ptr_feature_extractor = std::make_shared<FeatureExtractor>();
    laserCloud::Ptr corner_cloud(new laserCloud());
    laserCloud::Ptr surf_cloud(new laserCloud());
    laserCloud::Ptr cloud_for_occ_map(new laserCloud());
    ptr_feature_extractor->resetVariables();
    if (!ptr_feature_extractor->setInputCloud(cur_cloud)) {
      return false;
    }
    if (!ptr_feature_extractor->cloudExtractor(corner_cloud, surf_cloud,
                                               cloud_for_occ_map)) {
      return false;
    }

    FramePosition pose;
    pose.x = vec3d_pose(0);
    pose.y = vec3d_pose(1);
    pose.z = vec3d_pose(2);

    v_ptr_history_keyframe_ =
        ptr_map_manager_->DetectCandidatesByDistance(pose, 10.0);
    if (v_ptr_history_keyframe_.empty()) {
      LOG(ERROR) << "history keyframe was empty";
      return false;
    }

    laserCloud::Ptr candidate_cloud(new laserCloud);
    candidate_cloud->clear();
    for (auto key_it = v_ptr_history_keyframe_.begin();
         key_it != v_ptr_history_keyframe_.end(); key_it++) {
      Mat34d cur_frame_pose = (*key_it)->getPose();
      laserCloud::Ptr curCloudCorner = Transformbox::transformPointCloud(
          cur_frame_pose, (*key_it)->corner_cloud_);
      laserCloud::Ptr curCloudSurf = Transformbox::transformPointCloud(
          cur_frame_pose, (*key_it)->surf_cloud_);

      *candidate_cloud += *curCloudCorner;
      *candidate_cloud += *curCloudSurf;
    }

    laserCloud::Ptr ds_candidate_cloud(new laserCloud);
    downsize_filter_history_cloud_.setInputCloud(candidate_cloud);
    downsize_filter_history_cloud_.filter(*ds_candidate_cloud);
    int count = 0;
    if (stop_icp_) {
      stop_icp_ = false;
      return false;
    }
    double t1 = getSystemTime();
    Vec3d euler_angle;
    euler_angle(0) = pre_yaw;
    euler_angle(1) = 0;
    euler_angle(2) = 0;
    Mat34d curTrans34d;
    curTrans34d = Mathbox::Euler2Mat34d(euler_angle, vec3d_pose);

    laserCloud::Ptr curCloudInMap(new laserCloud());
    curCloudInMap->clear();

    laserCloud::Ptr curCloudCornerInMap =
        Transformbox::transformPointCloud(curTrans34d, corner_cloud);
    laserCloud::Ptr curCloudSurfInMap =
        Transformbox::transformPointCloud(curTrans34d, surf_cloud);

    *curCloudInMap += *curCloudCornerInMap;
    *curCloudInMap += *curCloudSurfInMap;
    double t2 = getSystemTime();
    Mat4d transformation;
    transformation.setZero();
    transformation.block<3, 3>(0, 0).setIdentity();
    if (cloudRegistration("icp", curCloudInMap, ds_candidate_cloud,
                          loop_config_.relocalization_icp_score,
                          transformation) > 0) {
      Mat34d transform_in_odom =
          Mathbox::multiplePose34d(T_lo_, transformation.block<3, 4>(0, 0));
      transform_in_odom = Mathbox::multiplePose34d(
          transform_in_odom, Mathbox::inversePose34d(T_lo_));
      pose34d = Mathbox::multiplePose34d(transform_in_odom, curTrans34d);
      LOG(INFO) << "transformation: + " << transformation << std::endl;
      LOG(INFO) << "init-match-pose: " << pose34d;
      return true;
    }
    double t3 = getSystemTime();
    count++;
    return false;
  }
  return false;
}

bool LoopTrack::getRoadThtha(const std::vector<double> &scan, double angle_min,
                             double angle_increment, double &ththa) {
  std::vector<Eigen::Vector2d> pointVec;
  for (int i = 0; i < scan.size(); i++) {
    Eigen::Vector2d point;
    double angle = angle_min + i * angle_increment;
    point(0) = scan[i] * cos(angle);
    point(1) = scan[i] * sin(angle);
    pointVec.push_back(point);
  }
  double ththa_histogram[181] = {0};
  double rate = 180.0 / M_PI;
  for (int i = 0; i < pointVec.size(); i++) {
    for (int j = i + 1; j < pointVec.size(); j++) {
      if (pointVec[i](1) - pointVec[j](1) != 0) {
        double temp = atan2(pointVec[i](1) - pointVec[j][1],
                            pointVec[i](0) - pointVec[j](0));
        int index = round(temp * rate);
        if (index < -90) {
          index += 180;
        } else if (index > 90) {
          index -= 180;
        }
        ththa_histogram[index + 90]++;
      }
    }
  }
  int index_max = 0;
  int histogram_max = ththa_histogram[index_max];
  for (int i = 0; i < 180; i++) {
    if (histogram_max < ththa_histogram[i]) {
      index_max = i;
      histogram_max = ththa_histogram[i];
    }
  }
  // std::cout << histogram_max << std::endl;
  ththa = (index_max - 90) * M_PI / 180.0;
  return true;
}

double LoopTrack::getPointCloudDirect(const laserCloud::Ptr ptr_cloud) {
  double angle_min = -M_PI;
  double angle_max = M_PI;
  double angle_increment = 1.0 * M_PI / 180;
  double time_increment = 0;
  double range_min = 0.3;
  double range_max = 100;

  int range_size = std::ceil((angle_max - angle_min) / angle_increment);
  std::vector<double> local_map_laser_scan;
  local_map_laser_scan.resize(range_size);
  std::fill(local_map_laser_scan.begin(), local_map_laser_scan.end(), 1000000);
  int local_map_Size = (*ptr_cloud).points.size();
  for (int i = 0; i < local_map_Size; ++i) {
    PointType *point = &(*ptr_cloud).points[i];
    float x = point->x;
    float y = point->y;
    float z = point->z;
    if (z > -0.3) {
      double range = std::sqrt(x * x + y * y);
      float angle = std::atan2(y, x);
      int index = (angle - angle_min) / angle_increment;
      if (index >= 0 && index < local_map_laser_scan.size())
        local_map_laser_scan[index] =
            std::min(local_map_laser_scan[index], range);
    }
  }
  double cloud_angle;
  getRoadThtha(local_map_laser_scan, angle_min, angle_increment, cloud_angle);
  return cloud_angle;
}

}  // namespace cvte_lidar_slam