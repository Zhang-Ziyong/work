
#include "frontend/horizon_feature_extractor.hpp"
#include "common/math_base/slam_math.hpp"
#include "common/debug_tools/tic_toc.h"

namespace cvte_lidar_slam {

HorizonFeatureExtractor::HorizonFeatureExtractor() {
  Initialize();
}

void HorizonFeatureExtractor::Initialize() {
  // construct object
  laser_cloud_in_.reset(new laserCloud());
  ds_cloud_.reset(new laserCloud());
  ground_cloud_.reset(new laserCloud());
  not_ground_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  corner_cloud_.reset(new laserCloud());
  outlier_cloud_.reset(new laserCloud());
  ptr_kdtree_surf_points_.reset(new pclKdTree());

  // set parameters
  downsize_filter_.setLeafSize(0.2, 0.2, 0.2);

  v_corner_grid_.reserve(100);

  // allocate memory
  const int grid_length_x = config_.range_x / config_.grid_resolution;
  const int grid_length_y = config_.range_y / config_.grid_resolution;
  vv_grid_pointts_.resize(grid_length_x);
  for (int j = 0; j < grid_length_x; ++j) {
    vv_grid_pointts_[j].resize(grid_length_y);
  }

  // fill the grid计算点位于哪个网格，填充
  for (int j = 0; j < grid_length_x; ++j) {
    for (int i = 0; i < grid_length_y; ++i) {
      vv_grid_pointts_[j][i].clear();
      vv_grid_pointts_[j][i].reserve(100);
    }
  }
}

void HorizonFeatureExtractor::resetVariables() {
  laser_cloud_in_->clear();
  ds_cloud_->clear();
  ground_cloud_->clear();
  not_ground_cloud_->clear();
  surf_cloud_->clear();
  corner_cloud_->clear();
  outlier_cloud_->clear();
  v_corner_grid_.clear();
  for (int j = 0; j < config_.range_x / config_.grid_resolution; ++j) {
    for (int i = 0; i < config_.range_y / config_.grid_resolution; ++i) {
      vv_grid_pointts_[j][i].clear();
    }
  }
}

bool HorizonFeatureExtractor::setInputCloud(const laserCloud &cloud_in) {
  // Remove Nan points
  if (cloud_in.points.empty()) {
    return false;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_in, *laser_cloud_in_, indices);
  if (laser_cloud_in_->points.empty()) {
    return false;
  }
  return true;
  // std::cout << "input_cloud size " << laser_cloud_in_->size() << std::endl;
}

bool HorizonFeatureExtractor::setInputCloud(const laserCloud::Ptr ptr_cloud) {
  *laser_cloud_in_ = *ptr_cloud;
  if (laser_cloud_in_->points.empty()) {
    return false;
  }
  return true;
  // std::cout << "input_cloud size " << laser_cloud_in_->size() << std::endl;
}

bool HorizonFeatureExtractor::downsampleCloud() {
  if (laser_cloud_in_->points.empty()) {
    std::cout << " empty cloud " << std::endl;
    return false;
  }
  downsize_filter_.setInputCloud(laser_cloud_in_);
  downsize_filter_.filter(*ds_cloud_);
  // std::cout << "ds_cloud_ size " << ds_cloud_->size() << std::endl;
  return true;
}

void HorizonFeatureExtractor::groundRemoval() {
  if (ds_cloud_->size() <= 0) {
    return;
  }
  PointType tmp_pt;
  int grid_I, grid_J;
  const int max_x = config_.range_x / config_.grid_resolution;
  const int max_y = config_.range_y / config_.grid_resolution;
  for (size_t i = 0; i < ds_cloud_->size(); ++i) {
    tmp_pt = ds_cloud_->points[i];

    grid_I = tmp_pt.x / config_.grid_resolution;
    grid_J = (config_.range_y / 2 + tmp_pt.y) / config_.grid_resolution;
    if (grid_I >= max_x || grid_I < 0 || grid_J < 0 || grid_J >= max_y) {
      continue;
    }

    vv_grid_pointts_[grid_I][grid_J].push_back(tmp_pt);
  }

  // calculate the grid height
  for (unsigned int j = 0; j < vv_grid_pointts_.size(); ++j) {
    for (unsigned int i = 0; i < vv_grid_pointts_[j].size(); ++i) {
      if (vv_grid_pointts_[j][i].empty()) {
        continue;
      }
      std::sort(vv_grid_pointts_[j][i].begin(), vv_grid_pointts_[j][i].end(),
                [](const PointType &a, const PointType &b) {
                  return a.z < b.z;
                });  //按高度从小到大排序
      std::vector<PointType>::iterator max = vv_grid_pointts_[j][i].end() - 1;
      std::vector<PointType>::iterator min = vv_grid_pointts_[j][i].begin();
      float Height_dif = max->z - min->z;  // 网格中的高度差
      // std::cout << "\033[31m" << "Height size " << Height_dif << ' ' <<
      // max->z << "\033[0m" << std::endl;
      if (Height_dif < 0.2 && max->z < 0.) {
        //根据高度差阈值和高度阈值，初步将点分为地面点和非地面点
        for (size_t k = 0; k < vv_grid_pointts_[j][i].size(); ++k) {
          ground_cloud_->push_back(vv_grid_pointts_[j][i][k]);
        }
      } else {
        if (vv_grid_pointts_[j][i].size() > 8) {
          grid temp(j, i, vv_grid_pointts_[j][i].size());
          v_corner_grid_.push_back(std::forward<grid>(temp));
        }
        for (size_t k = 0; k < vv_grid_pointts_[j][i].size(); ++k) {
          not_ground_cloud_->push_back(vv_grid_pointts_[j][i][k]);
        }
      }
    }
  }
}

void HorizonFeatureExtractor::adjustDistortion() {
  return;
}

void HorizonFeatureExtractor::extractFeatures() {
  const int not_ground_cloud_num = not_ground_cloud_->size();
  if (0 == not_ground_cloud_num) {
    return;
  }
  ptr_kdtree_surf_points_->setInputCloud(not_ground_cloud_);

  for (int i = 0; i < not_ground_cloud_num; i++) {
    PointType pointOri = not_ground_cloud_->points[i];
    const float distance =
        sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y +
             pointOri.z * pointOri.z);
    if (fabs(distance) < 1.) {
      continue;
    }
    const int search_num = config_.neighbor_search_num;
    std::vector<int> pointSearchInd;
    pointSearchInd.clear();
    pointSearchInd.reserve(search_num);
    std::vector<float> pointSearchSqDis;
    pointSearchSqDis.clear();
    pointSearchSqDis.reserve(search_num);

    ptr_kdtree_surf_points_->nearestKSearch(pointOri, search_num,
                                            pointSearchInd, pointSearchSqDis);
    vVec3d near_corners;
    near_corners.clear();
    near_corners.reserve(search_num);
    Vec3d center = Vec3d::Zero();

    const double max_neighbor_dist = std::max(1.2, 0.03 * distance);
    if (pointSearchSqDis[search_num - 1] < max_neighbor_dist) {
      for (int j = 0; j < search_num; j++) {
        Vec3d temp_point(not_ground_cloud_->points[pointSearchInd[j]].x,
                         not_ground_cloud_->points[pointSearchInd[j]].y,
                         not_ground_cloud_->points[pointSearchInd[j]].z);
        center += temp_point;
        near_corners.push_back(temp_point);
      }
      center = center / search_num;
      Mat3d cov_mat = Mat3d::Zero();
      for (int j = 0; j < search_num; j++) {
        Vec3d temp_zero_mean = near_corners[j] - center;
        cov_mat += temp_zero_mean * temp_zero_mean.transpose();
      }
      cov_mat = 0.1 * cov_mat;  // TODO:是否多余
      Eigen::SelfAdjointEigenSolver<Mat3d> saes(cov_mat);
      // std::cout << "eigenvalues:  " << saes.eigenvalues().transpose() <<
      // std::endl;
      float plane_scale = fabs((saes.eigenvalues()[1] - saes.eigenvalues()[0]) /
                                   saes.eigenvalues()[2] +
                               1e-5);
      // std::cout << "plane_scale: " << plane_scale << std::endl;
      if (plane_scale > config_.plane_scale_threshold) {
        surf_cloud_->push_back(pointOri);
      }
    }
  }

  // find corner points
  if (!v_corner_grid_.empty()) {
    ptr_kdtree_surf_points_->setInputCloud(surf_cloud_);
    int count = 0;
    for (const auto &it : v_corner_grid_) {
      int surf_points_count = 0;
      for (size_t k = 0; k < it.size; ++k) {
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        PointType pointOri = vv_grid_pointts_[it.x][it.y][k];
        ptr_kdtree_surf_points_->nearestKSearch(pointOri, 1, pointSearchInd,
                                                pointSearchSqDis);
        if (pointSearchSqDis[0] < 0.05) {
          ++surf_points_count;
        }
      }
      double ratio = 1. * surf_points_count / it.size;
      if (ratio < 0.7) {
        for (size_t k = 0; k < it.size; ++k) {
          corner_cloud_->push_back(vv_grid_pointts_[it.x][it.y][k]);
        }
      }
      count += it.size;
    }
    // std::cout << "count" << count << " " << corner_cloud_->size() <<
    // std::endl;
  }
}

bool HorizonFeatureExtractor::cloudExtractor(
    laserCloud::Ptr corner_cloud, laserCloud::Ptr surf_cloud,
    laserCloud::Ptr without_ground_cloud) {
  std::unique_lock<std::mutex> lock(mutex_);
  downsampleCloud();
  groundRemoval();
  extractFeatures();

  *corner_cloud = *corner_cloud_;
  *surf_cloud = *surf_cloud_;
  *surf_cloud += *ground_cloud_;
  *without_ground_cloud = *corner_cloud_;
  *without_ground_cloud += *surf_cloud_;

  return true;
}

bool HorizonFeatureExtractor::cloudExtractor(laserCloud::Ptr surf_cloud) {
  std::unique_lock<std::mutex> lock(mutex_);
  downsampleCloud();
  groundRemoval();
  extractFeatures();
  *surf_cloud = *surf_cloud_;
  *surf_cloud += *ground_cloud_;
  return true;
}

}  // namespace cvte_lidar_slam