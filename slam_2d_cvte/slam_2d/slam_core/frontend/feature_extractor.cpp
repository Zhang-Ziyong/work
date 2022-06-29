#include "frontend/feature_extractor.hpp"
#include "common/data_struct/keyframe.hpp"
#include <glog/logging.h>
#include "common/debug_tools/tic_toc.h"

namespace cvte_lidar_slam {
FeatureExtractor::FeatureExtractor() {
  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = 0;

  allocateMemory();
  resetVariables();
}

FeatureExtractor::FeatureExtractor(const FeatureConfig &config) {
  config_ = config;

  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = 0;

  allocateMemory();
  resetVariables();
}

void FeatureExtractor::allocateMemory() {
  laser_cloud_in_.reset(new laserCloud());
  ground_cloud_.reset(new laserCloud());
  segmented_cloud_.reset(new laserCloud());
  segmented_cloud_pure_.reset(new laserCloud());
  outlier_cloud_.reset(new laserCloud());
  without_ground_cloud_.reset(new laserCloud());

  std::pair<int8_t, int8_t> neighbor;
  neighbor.first = -1;
  neighbor.second = 0;
  neighbor_iterator_.push_back(neighbor);
  neighbor.first = 0;
  neighbor.second = 1;
  neighbor_iterator_.push_back(neighbor);
  neighbor.first = 0;
  neighbor.second = -1;
  neighbor_iterator_.push_back(neighbor);
  neighbor.first = 1;
  neighbor.second = 0;
  neighbor_iterator_.push_back(neighbor);

  all_pushed_ind_x_ = new uint16_t[config_.N_SCAN * config_.Horizon_SCAN];
  all_pushed_ind_y_ = new uint16_t[config_.N_SCAN * config_.Horizon_SCAN];

  queue_ind_x_ = new uint16_t[config_.N_SCAN * config_.Horizon_SCAN];
  queue_ind_y_ = new uint16_t[config_.N_SCAN * config_.Horizon_SCAN];

  cloud_curvature_.resize(config_.N_SCAN * config_.Horizon_SCAN);
  cloud_curvature_norm_.resize(config_.N_SCAN * config_.Horizon_SCAN);
  cloud_neighbor_picked_.resize(config_.N_SCAN * config_.Horizon_SCAN);
  cloud_label_.resize(config_.N_SCAN * config_.Horizon_SCAN);
  cloud_smoothness_.resize(config_.N_SCAN * config_.Horizon_SCAN);

  corner_points_.reset(new laserCloud());
  surf_points_.reset(new laserCloud());
  corner_points_top_.reset(new laserCloud());
  surf_points_top_.reset(new laserCloud());
  raw_cloud_.reset(new laserCloud());

  surf_points_scan_.reset(new laserCloud());
  surf_points_scan_DS_.reset(new laserCloud());

  downSize_filter_edge_.setLeafSize(config_.edge_size, config_.edge_size,
                                    config_.edge_size);
  downSize_filter_surf_.setLeafSize(config_.surf_size, config_.surf_size,
                                    config_.surf_size);
  downSize_narrow_cloud_.setLeafSize(config_.edge_size * 0.5,
                                     config_.edge_size * 0.5,
                                     config_.edge_size * 0.5);
}

void FeatureExtractor::resetVariables() {
  laser_cloud_in_->clear();
  ground_cloud_->clear();
  segmented_cloud_->clear();
  segmented_cloud_pure_->clear();
  outlier_cloud_->clear();
  without_ground_cloud_->clear();

  corner_points_->clear();
  surf_points_->clear();
  corner_points_top_->clear();
  surf_points_top_->clear();
  raw_cloud_->clear();

  segmented_range_.clear();

  label_count_ = 1;
}

bool FeatureExtractor::setInputCloud(const laserCloud &cloud_in) {
  if (cloud_in.points.empty()) {
    return false;
  }

  PointType p_t;
  size_t cloudSize = cloud_in.size();

  for (size_t i = 0; i < cloudSize; i++) {
    p_t = cloud_in.points[i];

    if (std::isnan(p_t.x) || std::isnan(p_t.y) || std::isnan(p_t.z)) {
      continue;
    }

    laser_cloud_in_->push_back(p_t);
  }

  if (laser_cloud_in_->empty()) {
    return false;
  }
  return true;
}

bool FeatureExtractor::setInputCloud(const laserCloud::Ptr ptr_cloud) {
  if (ptr_cloud->points.empty()) {
    return false;
  }

  PointType p_t;
  size_t cloudSize = ptr_cloud->size();

  for (size_t i = 0; i < cloudSize; i++) {
    p_t = ptr_cloud->points[i];

    if (std::isnan(p_t.x) || std::isnan(p_t.y) || std::isnan(p_t.z) ||
        std::isnan(p_t.intensity)) {
      continue;
    }

    laser_cloud_in_->push_back(p_t);
  }
  // printf("--FeatureExtractor::setInputCloud: cloudSize: %d \n",
  // laser_cloud_in_->size());

  if (laser_cloud_in_->empty()) {
    return false;
  }
  return true;
}

void FeatureExtractor::adjustDistortion(const Mat34d &delta_pose) {
  LOG(INFO) << "Undistort angle: "
            << Mathbox::rotation2rpy(delta_pose.block<3, 3>(0, 0)).transpose();
  LOG(INFO) << "Undistort trans: " << delta_pose.block<3, 1>(0, 3).transpose();
  const size_t &number = laser_cloud_in_->size();
  bool halfPassed = false;
  float startOri =
      -atan2(laser_cloud_in_->points[0].y, laser_cloud_in_->points[0].x);
  float endOri = -atan2(laser_cloud_in_->points[number - 1].y,
                        laser_cloud_in_->points[number - 1].x) +
                 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }
  PointType pointSel;
  for (size_t i = 0; i < number; ++i) {
    auto &pi = laser_cloud_in_->points[i];
    float ori = -atan2(pi.y, pi.x);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }
    if (ori > endOri) {
      ori -= 2 * M_PI;
    }
    float relTime = (ori - startOri) / (endOri - startOri);
    Mat34d delta_end_2_i =
        Mathbox::Interp_SE3(Mathbox::Identity34(), delta_pose, 1.0 - relTime);
    Transformbox::pointAssociateToMap(&laser_cloud_in_->points[i], &pointSel,
                                      delta_end_2_i);
    laser_cloud_in_->points[i] = pointSel;
  }
}

void FeatureExtractor::clusterSegmentation2D() {
  float small_cluster_size = config_.small_cluster_size;

  size_t cloud_size = laser_cloud_in_->size();
  PointType p;
  PointType p1, p2;

  for (int i = 0; i < cloud_size; i++)  //计算点深度；
  {
    p = laser_cloud_in_->points[i];
    range_array_[i] = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    bad_point_marked_[i] = 0;
  }

  // 根据撕裂点进行分割，计算小目标并标记；
  float cluster_size = 0.0;
  float diff_th = 0.3;
  float diff_range = 0.0;
  size_t cluster_num = 0;
  size_t point_count = 0;
  for (size_t i = 1; i < cloud_size - 1; i++)  //计算点深度；
  {
    // 该聚类点计数器累加；
    point_count++;

    diff_range = std::abs(range_array_[i] - range_array_[i + 1]);

    if (diff_range > diff_th)  // 如果点深度的差值小于一定阈值，则认为为撕裂点；
    {
      // 聚类组计数器累加，聚类点计数器清零；
      cluster_num++;

      p1 = laser_cloud_in_->points[i - point_count + 1];
      p2 = laser_cloud_in_->points[i];

      cluster_size = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                               (p1.y - p2.y) * (p1.y - p2.y) +
                               (p1.z - p2.z) * (p1.z - p2.z));

      bool small_cluster_flag = false;

      // if (point_count < 20 && range_array_[i] < 6.0) {
      //   small_cluster_flag = true;
      // } else if (point_count < 15 && range_array_[i] < 10.0) {
      //   small_cluster_flag = true;
      // } else if (point_count < 10 && range_array_[i] < 14.0) {
      //   small_cluster_flag = true;
      // } else if (point_count < 7) {
      //   small_cluster_flag = true;
      // }

      if (cluster_size < small_cluster_size) {
        small_cluster_flag = true;
      }

      if (small_cluster_flag == true) {
        for (size_t j = 0; j < point_count; j++) {
          bad_point_marked_[i - j] = 1;  //
        }
      }
      // printf(
      //     "--laserFeature: cluster_num: %d, point_count: %d ,cluster_size: "
      //     "%.2f \n",
      //     cluster_num, point_count, cluster_size);

      point_count = 0;
    }
  }

  segmented_cloud_->clear();
  outlier_cloud_->clear();
  segmented_range_.clear();
  for (size_t i = 3; i < cloud_size - 3; i++)  //得到分割点
  {
    if (bad_point_marked_[i] == 0) {
      segmented_cloud_->push_back(laser_cloud_in_->points[i]);
      segmented_range_.push_back(range_array_[i]);
    } else {
      outlier_cloud_->push_back(laser_cloud_in_->points[i]);
    }
  }
}

void FeatureExtractor::calculateSmoothness2D() {
  size_t cloud_size = segmented_cloud_->size();
  for (size_t i = 5; i < cloud_size - 5; i++) {
    float diffRange =
        range_array_[i - 5] + range_array_[i - 4] + range_array_[i - 3] +
        range_array_[i - 2] + range_array_[i - 1] - range_array_[i] * 10 +
        range_array_[i + 1] + range_array_[i + 2] + range_array_[i + 3] +
        range_array_[i + 4] + range_array_[i + 5];

    // cloud_curvature_[i] = diffRange * diffRange;
    cloud_curvature_[i] = std::abs(diffRange);
    cloud_curvature_norm_[i] = std::abs(diffRange / range_array_[i]);

    cloud_neighbor_picked_[i] = 0;
    cloud_label_[i] = 0;

    cloud_smoothness_[i].value = cloud_curvature_[i];
    cloud_smoothness_[i].ind = i;
  }
}

void FeatureExtractor::extractFeatures2D() {
  size_t cloud_size = segmented_cloud_->size();
  size_t sector_num = 6;
  size_t sector_size = cloud_size / sector_num;

  for (size_t j = 0; j < sector_num; j++) {
    size_t sp = sector_size * j;
    size_t ep = sector_size * (j + 1);

    if (ep > cloud_size) {
      ep = cloud_size;
    }

    std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep,
              by_value());

    size_t largestPickedNum = 0;
    for (size_t k = ep; k > sp; k--) {
      size_t ind = cloud_smoothness_[k].ind;

      if (cloud_neighbor_picked_[ind] == 0 &&
          cloud_curvature_[ind] > config_.edgeThreshold) {
        if (largestPickedNum < 6) {
          cloud_label_[ind] = 2;
          corner_points_->push_back(segmented_cloud_->points[ind]);
        } else {
          break;
        }

        largestPickedNum++;
        cloud_neighbor_picked_[ind] = 1;

        //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
        for (int l = -5; l <= 5; l++) {
          cloud_neighbor_picked_[ind + l] = 1;  //
        }
      }
    }

    size_t smallestPickedNum = 0;
    for (size_t k = sp; k < ep; k++) {
      size_t ind = cloud_smoothness_[k].ind;

      if (cloud_curvature_[ind] < config_.surfThreshold) {
        if (smallestPickedNum < 30) {
          cloud_label_[ind] = -2;
          surf_points_->push_back(segmented_cloud_->points[ind]);
        } else {
          break;
        }
        smallestPickedNum++;
      }
    }
  }
}

bool FeatureExtractor::cloudExtractor(laserCloud::Ptr corner_cloud,
                                      laserCloud::Ptr surf_cloud,
                                      laserCloud::Ptr corner_cloud_top,
                                      laserCloud::Ptr surf_cloud_top,
                                      laserCloud::Ptr raw_cloud,
                                      laserCloud::Ptr without_ground_cloud) {
  return false;
}

bool FeatureExtractor::cloudExtractor(laserCloud::Ptr corner_cloud,
                                      laserCloud::Ptr surf_cloud,
                                      laserCloud::Ptr without_ground_cloud,
                                      int &environment_flag, Vec6d &h_matrix) {
  static int min_feature_points = config_.min_feature_points;
  static float narrow_candidate = config_.narrow_candidate;
  static float narrow_region = config_.narrow_region;
  static float narrow_ratio = config_.narrow_ratio;
  TicToc extractFeatures_cost;

  clusterSegmentation2D();

  pcl::PointCloud<PointType>::Ptr laserCloudDS(
      new pcl::PointCloud<PointType>());

  // downSize_filter_surf_.setInputCloud(laser_cloud_in_);
  downSize_filter_surf_.setInputCloud(segmented_cloud_);
  downSize_filter_surf_.filter(*laserCloudDS);

  size_t segmented_cloud_size = laserCloudDS->size();

  PointType p;
  int near_point_count = 0;
  float range_tmp = 0.0;
  float range_sum = 0.0;
  float range_ave = 0.0;
  for (size_t i = 0; i < segmented_cloud_size; i++) {
    p = laserCloudDS->points[i];
    range_tmp = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    range_sum += range_tmp;
    if (range_tmp < narrow_candidate) {
      near_point_count++;
    }
  }
  range_ave = range_sum / segmented_cloud_size;
  float near_ratio = 1.0 * near_point_count / segmented_cloud_size;

  bool narrow_flag = false;
  if (range_ave < narrow_region && near_ratio > narrow_ratio) {
    LOG(ERROR) << "Robot moved in narrow space ! laserCloudDS: "
               << segmented_cloud_size;
    LOG(ERROR) << "Robot moved in narrow space ! near_ratio: " << near_ratio;
    LOG(ERROR) << "Robot moved in narrow space ! range_ave: " << range_ave;
    narrow_flag = true;
    // return false;
  }

  // if (segmented_cloud_size < min_feature_points) {
  //   LOG(ERROR) << "Robot moved in narrow space ! Too few points, laserCloudDS
  //   :"
  //              << segmented_cloud_size;
  //   narrow_flag = true;
  //   // return false;
  // }

  if (narrow_flag == false) {
    PointType point;
    double frontAngleThred = 5.0 / 180.0 * M_PI;  // tan(5 deg)*20m = 1.6m
    for (size_t i = 0; i < segmented_cloud_->size(); i++) {
      point = segmented_cloud_->points[i];

      if (std::abs(point.y) > 0.8)
        continue;

      float ang_rad = std::atan2(point.y, point.x);
      if (std::abs(ang_rad) < frontAngleThred)  // 根据扫描线角度进行选择
      {
        laserCloudDS->push_back(point);
      }
    }
  } else {
    laserCloudDS->clear();
    downSize_narrow_cloud_.setInputCloud(segmented_cloud_);
    downSize_narrow_cloud_.filter(*laserCloudDS);
  }

  *corner_cloud = *laserCloudDS;
  *surf_cloud = *laserCloudDS;
  *without_ground_cloud = *laser_cloud_in_;
  environment_flag = 0;
  if (narrow_flag == true) {
    environment_flag = 1;
  }

  // LOG(INFO) << "corner: " << corner_cloud->size() << ", surf: " <<
  // surf_cloud->size() << ", raw: " << laser_cloud_in_->size();

  if (corner_cloud->size() < config_.min_feature_points) {
    LOG(ERROR) << "too few corner points! corner: " << corner_cloud->size()
               << ", raw: " << laser_cloud_in_->size();
    // return false;
  }
  if (config_.calculate_degenerate) {
    if (corner_cloud->size() < config_.min_feature_points) {
      h_matrix << 500, 500, 25000, 1, 1, 500;
    } else {
      calculateCloudHessian(corner_cloud, h_matrix);
    }
  }

  return true;
}

void FeatureExtractor::calculateCloudHessian(const laserCloud::Ptr input_cloud,
                                             Vec6d &h_matrix) {
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
  pclKdTree::Ptr ptr_kdtree_corner;
  ptr_kdtree_corner.reset(new pclKdTree());
  ptr_kdtree_corner->setInputCloud(input_cloud);
  Eigen::Matrix<double, 6, 6> Hessian = Eigen::Matrix<double, 6, 6>::Zero();
  int corner_num = 0;

  for (size_t j = 0; j < input_cloud->size(); ++j) {
    PointType pointOri = input_cloud->points[j];
    std::vector<int> pointSearchInd;  // TODO:事先resize 提高效率
    pointSearchInd.reserve(5);
    std::vector<float> pointSearchSqDis;
    pointSearchSqDis.reserve(5);
    if (ptr_kdtree_corner->nearestKSearch(pointOri, 5, pointSearchInd,
                                          pointSearchSqDis) > 4) {
      if (pointSearchSqDis[4] < 1.0) {
        std::vector<Eigen::Vector3d> nearCorners;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++) {
          Eigen::Vector3d tmp(input_cloud->points[pointSearchInd[j]].x,
                              input_cloud->points[pointSearchInd[j]].y,
                              input_cloud->points[pointSearchInd[j]].z);
          center = center + tmp;
          nearCorners.push_back(tmp);
        }
        center = center / 5.0;

        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++) {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
        Eigen::Vector3d lp(pointOri.x, pointOri.y, pointOri.z);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
          Eigen::Vector3d point_on_line = center;
          Eigen::Vector3d last_point_a, last_point_b;
          last_point_a = 0.1 * unit_direction + point_on_line;
          last_point_b = -0.1 * unit_direction + point_on_line;

          Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
          Eigen::Vector3d de = last_point_a - last_point_b;
          double de_norm = de.norm();
          if (de.norm() == 0 || nu.norm() == 0) {
            continue;
          }

          Eigen::Matrix3d skew_lp = Mathbox::skew(lp);
          Eigen::Matrix<double, 3, 6> dp_by_se3;
          dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
          (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
          Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J_se3;
          J_se3.setZero();
          Eigen::Matrix3d skew_de = Mathbox::skew(de);
          J_se3.block<1, 6>(0, 0) =
              -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
          corner_num++;
          Hessian += J_se3.transpose() * J_se3;
        }
      }
    }
  }
  Eigen::Matrix<double, 6, 6> Hessian_tmp = Hessian / corner_num * 1000;
  h_matrix << Hessian_tmp(0, 0), Hessian_tmp(1, 1), Hessian_tmp(2, 2),
      Hessian_tmp(3, 3), Hessian_tmp(4, 4), Hessian_tmp(5, 5);
}
}  // namespace cvte_lidar_slam