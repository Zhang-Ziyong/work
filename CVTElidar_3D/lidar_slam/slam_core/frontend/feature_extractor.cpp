#include "frontend/feature_extractor.hpp"
#include "common/data_struct/keyframe.hpp"
#include <glog/logging.h>

namespace cvte_lidar_slam {

FeatureExtractor::FeatureExtractor() {
  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();
  nan_point_.intensity = -1;

  allocateMemory();
  resetVariables();
}

void FeatureExtractor::allocateMemory() {
  laser_cloud_in_.reset(new laserCloud());
  full_cloud_.reset(new laserCloud());
  full_info_cloud_.reset(new laserCloud());
  ground_cloud_.reset(new laserCloud());
  segmented_cloud_.reset(new laserCloud());
  segmented_cloud_pure_.reset(new laserCloud());
  outlier_cloud_.reset(new laserCloud());
  without_ground_cloud_.reset(new laserCloud());

  full_cloud_->points.resize(config_.N_SCAN * config_.Horizon_SCAN);
  full_info_cloud_->points.resize(config_.N_SCAN * config_.Horizon_SCAN);

  seg_info_.start_ring_index.assign(config_.N_SCAN, 0);
  seg_info_.end_ring_index.assign(config_.N_SCAN, 0);
  seg_info_.segmented_cloud_ground_flag.assign(
      config_.N_SCAN * config_.Horizon_SCAN, false);
  seg_info_.segmented_cloud_col_ind.assign(
      config_.N_SCAN * config_.Horizon_SCAN, 0);
  seg_info_.segmented_cloud_range.assign(config_.N_SCAN * config_.Horizon_SCAN,
                                         0);
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
  cloud_neighbor_picked_.resize(config_.N_SCAN * config_.Horizon_SCAN);
  cloud_label_.resize(config_.N_SCAN * config_.Horizon_SCAN);
  cloud_smoothness_.resize(config_.N_SCAN * config_.Horizon_SCAN);

  corner_points_.reset(new laserCloud());
  surf_points_.reset(new laserCloud());

  surf_points_scan_.reset(new laserCloud());
  surf_points_scan_DS_.reset(new laserCloud());

  downSize_filter_.setLeafSize(0.2, 0.2, 0.2);
}

void FeatureExtractor::resetVariables() {
  laser_cloud_in_->clear();
  ground_cloud_->clear();
  segmented_cloud_->clear();
  segmented_cloud_pure_->clear();
  outlier_cloud_->clear();
  without_ground_cloud_->clear();

  range_mat_ = cv::Mat(config_.N_SCAN, config_.Horizon_SCAN, CV_32F,
                       cv::Scalar::all(FLT_MAX));
  ground_mat_ =
      cv::Mat(config_.N_SCAN, config_.Horizon_SCAN, CV_8S, cv::Scalar::all(0));
  label_mat_ =
      cv::Mat(config_.N_SCAN, config_.Horizon_SCAN, CV_32S, cv::Scalar::all(0));
  label_count_ = 1;

  std::fill(full_cloud_->points.begin(), full_cloud_->points.end(), nan_point_);
  std::fill(full_info_cloud_->points.begin(), full_info_cloud_->points.end(),
            nan_point_);
}

bool FeatureExtractor::setInputCloud(const laserCloud &cloud_in) {
  if (cloud_in.points.empty()) {
    return false;
  }
  // Remove Nan points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_in, *laser_cloud_in_, indices);
  if (laser_cloud_in_->points.empty()) {
    return false;
  }
  return true;
}
bool FeatureExtractor::setInputCloud(const laserCloud::Ptr ptr_cloud) {
  if (ptr_cloud->points.empty()) {
    return false;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*ptr_cloud, *laser_cloud_in_, indices);
  if (laser_cloud_in_->points.empty()) {
    return false;
  }
  return true;
}
void FeatureExtractor::findStartEndAngle() {
  seg_info_.start_orientation =
      -atan2(laser_cloud_in_->points[0].y, laser_cloud_in_->points[0].x);
  seg_info_.end_orientation =
      -atan2(laser_cloud_in_->points[laser_cloud_in_->points.size() - 1].y,
             laser_cloud_in_->points[laser_cloud_in_->points.size() - 1].x) +
      2 * M_PI;
  if (seg_info_.end_orientation - seg_info_.start_orientation > 3 * M_PI) {
    seg_info_.end_orientation -= 2 * M_PI;
  } else if (seg_info_.end_orientation - seg_info_.start_orientation < M_PI)
    seg_info_.end_orientation += 2 * M_PI;
  seg_info_.orientation_diff =
      seg_info_.end_orientation - seg_info_.start_orientation;
}

void FeatureExtractor::projectPointCloud() {
  // range image projection
  float verticalAngle, horizonAngle, range;
  size_t rowIdn, columnIdn, index, cloudSize;
  PointType thisPoint;

  cloudSize = laser_cloud_in_->points.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    thisPoint.x = laser_cloud_in_->points[i].x;
    thisPoint.y = laser_cloud_in_->points[i].y;
    thisPoint.z = laser_cloud_in_->points[i].z;
    // find the row and column index in the iamge for this point
    verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x +
                                            thisPoint.y * thisPoint.y)) *
                    180 / M_PI;
    rowIdn = (verticalAngle + config_.ang_bottom) / config_.ang_res_y;
    if (rowIdn < 0 || rowIdn >= config_.N_SCAN)
      continue;
    horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

    columnIdn = -round((horizonAngle - 90.0) / config_.ang_res_x) +
                config_.Horizon_SCAN / 2;
    if (columnIdn >= config_.Horizon_SCAN)
      columnIdn -= config_.Horizon_SCAN;

    if (columnIdn < 0 || columnIdn >= config_.Horizon_SCAN)
      continue;

    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y +
                 thisPoint.z * thisPoint.z);
    if (range < 0.4)
      continue;

    range_mat_.at<float>(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float) rowIdn + (float) columnIdn / 10000.0;

    index = columnIdn + rowIdn * config_.Horizon_SCAN;
    full_cloud_->points[index] = thisPoint;
    full_info_cloud_->points[index] = thisPoint;
    full_info_cloud_->points[index].intensity =
        range;  // the corresponding range of a point is saved as "intensity"
  }
}

void FeatureExtractor::groundRemoval() {
  size_t lowerInd, upperInd;
  float diffX, diffY, diffZ, angle;
  // groundMat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < config_.Horizon_SCAN; ++j) {
    for (size_t i = 0; i < config_.groundScanInd; ++i) {
      lowerInd = j + (i) *config_.Horizon_SCAN;
      upperInd = j + (i + 1) * config_.Horizon_SCAN;

      if (full_cloud_->points[lowerInd].intensity == -1 ||
          full_cloud_->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        ground_mat_.at<int8_t>(i, j) = -1;
        continue;
      }

      diffX = full_cloud_->points[upperInd].x - full_cloud_->points[lowerInd].x;
      diffY = full_cloud_->points[upperInd].y - full_cloud_->points[lowerInd].y;
      diffZ = full_cloud_->points[upperInd].z - full_cloud_->points[lowerInd].z;

      angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

      if (abs(angle - config_.sensorMountAngle) <= 10) {
        ground_mat_.at<int8_t>(i, j) = 1;
        ground_mat_.at<int8_t>(i + 1, j) = 1;
      }
    }
  }
  // extract ground cloud (groundMat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation
  // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label
  // matrix for the 16th scan
  for (size_t i = 0; i < config_.N_SCAN; ++i) {
    for (size_t j = 0; j < config_.Horizon_SCAN; ++j) {
      if (ground_mat_.at<int8_t>(i, j) == 1 ||
          range_mat_.at<float>(i, j) == FLT_MAX) {
        label_mat_.at<int>(i, j) = -1;
      }
    }
  }

  for (size_t i = 0; i <= config_.groundScanInd; ++i) {
    for (size_t j = 0; j < config_.Horizon_SCAN; ++j) {
      if (ground_mat_.at<int8_t>(i, j) == 1)
        ground_cloud_->push_back(
            full_cloud_->points[j + i * config_.Horizon_SCAN]);
    }
  }

  for (size_t i = 0; i < config_.N_SCAN; ++i) {
    for (size_t j = 0; j < config_.Horizon_SCAN; ++j) {
      if (ground_mat_.at<int8_t>(i, j) == 1 ||
          ground_mat_.at<int8_t>(i, j) == -1 ||
          range_mat_.at<float>(i, j) == FLT_MAX) {
        continue;
      }
      without_ground_cloud_->push_back(
          full_cloud_->points[j + i * config_.Horizon_SCAN]);
    }
  }
}

void FeatureExtractor::labelComponents(int row, int col) {
  // use std::queue std::vector std::deque will slow the program down greatly
  float d1, d2, alpha, angle;
  unsigned int fromIndX, fromIndY, thisIndX, thisIndY;
  bool lineCountFlag[config_.N_SCAN] = {false};

  queue_ind_x_[0] = row;
  queue_ind_y_[0] = col;
  int queueSize = 1;
  int queueStartInd = 0;
  int queueEndInd = 1;

  all_pushed_ind_x_[0] = row;
  all_pushed_ind_y_[0] = col;
  unsigned int allPushedIndSize = 1;

  while (queueSize > 0) {
    // Pop point
    fromIndX = queue_ind_x_[queueStartInd];
    fromIndY = queue_ind_y_[queueStartInd];
    --queueSize;
    ++queueStartInd;
    // Mark popped point
    label_mat_.at<int>(fromIndX, fromIndY) = label_count_;
    // Loop through all the neighboring grids of popped grid
    for (auto iter = neighbor_iterator_.begin();
         iter != neighbor_iterator_.end(); ++iter) {
      // new index
      thisIndX = fromIndX + (*iter).first;
      thisIndY = fromIndY + (*iter).second;
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= config_.N_SCAN)
        continue;
      // at range image margin (left or right side)
      if (thisIndY < 0)
        thisIndY = config_.Horizon_SCAN - 1;
      if (thisIndY >= config_.Horizon_SCAN)
        thisIndY = 0;
      // prevent infinite loop (caused by put already examined point back)
      if (label_mat_.at<int>(thisIndX, thisIndY) != 0)
        continue;

      d1 = std::max(range_mat_.at<float>(fromIndX, fromIndY),
                    range_mat_.at<float>(thisIndX, thisIndY));
      d2 = std::min(range_mat_.at<float>(fromIndX, fromIndY),
                    range_mat_.at<float>(thisIndX, thisIndY));

      if ((*iter).first == 0)
        alpha = config_.segmentAlphaX;
      else
        alpha = config_.segmentAlphaY;

      angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

      if (angle > config_.segmentTheta) {
        queue_ind_x_[queueEndInd] = thisIndX;
        queue_ind_y_[queueEndInd] = thisIndY;
        ++queueSize;
        ++queueEndInd;

        label_mat_.at<int>(thisIndX, thisIndY) = label_count_;
        lineCountFlag[thisIndX] = true;

        all_pushed_ind_x_[allPushedIndSize] = thisIndX;
        all_pushed_ind_y_[allPushedIndSize] = thisIndY;
        ++allPushedIndSize;
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (allPushedIndSize >= 30)
    feasibleSegment = true;
  else if (allPushedIndSize >= config_.segmentValidPointNum) {
    unsigned int lineCount = 0;
    for (size_t i = 0; i < config_.N_SCAN; ++i)
      if (lineCountFlag[i] == true)
        ++lineCount;
    if (lineCount >= config_.segmentValidLineNum)
      feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++label_count_;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < allPushedIndSize; ++i) {
      label_mat_.at<int>(all_pushed_ind_x_[i], all_pushed_ind_y_[i]) = 999999;
    }
  }
}

void FeatureExtractor::cloudSegmentation() {
  // segmentation process
  // for (size_t i = 0; i < config_.N_SCAN; ++i)
  //   for (size_t j = 0; j < config_.Horizon_SCAN; ++j)
  //     if (label_mat_.at<int>(i, j) == 0) labelComponents(i, j);

  // for (size_t i = 0; i < config_.N_SCAN; ++i) {
  //   for (size_t j = 0; j < config_.Horizon_SCAN; ++j) {
  //     if (label_mat_.at<int>(i, j) > 0) {
  //       if (label_mat_.at<int>(i, j) != 999999) {
  //         segmented_cloud_pure_->push_back(
  //             full_cloud_->points[j + i * config_.Horizon_SCAN]);
  //         segmented_cloud_pure_->points.back().intensity =
  //             label_mat_.at<int>(i, j);
  //       }
  //     }
  //   }
  // }
  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < config_.N_SCAN; ++i) {
    // 开始4点和末尾6点舍去不要
    seg_info_.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

    for (size_t j = 0; j < config_.Horizon_SCAN; ++j) {
      if (label_mat_.at<int>(i, j) == 0)
        labelComponents(i, j);
      if (label_mat_.at<int>(i, j) > 0 || ground_mat_.at<int8_t>(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (label_mat_.at<int>(i, j) == 999999 &&
            label_mat_.at<int>(i, j) != -10) {
          if (i > config_.groundScanInd && j % 5 == 0) {
            outlier_cloud_->push_back(
                full_cloud_->points[j + i * config_.Horizon_SCAN]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (ground_mat_.at<int8_t>(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < config_.Horizon_SCAN - 5)
            continue;
        }
        // mark ground points so they will not be considered as edge features
        // later
        seg_info_.segmented_cloud_ground_flag[sizeOfSegCloud] =
            (ground_mat_.at<int8_t>(i, j) == 1);
        // mark the points' column index for marking occlusion later
        seg_info_.segmented_cloud_col_ind[sizeOfSegCloud] = j;
        // save range info
        seg_info_.segmented_cloud_range[sizeOfSegCloud] =
            range_mat_.at<float>(i, j);
        // save seg cloud
        segmented_cloud_->push_back(
            full_cloud_->points[j + i * config_.Horizon_SCAN]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    seg_info_.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
  }
}

void FeatureExtractor::adjustDistortion() {
  bool halfPassed = false;
  int cloudSize = segmented_cloud_->points.size();
  PointType point;

  for (int i = 0; i < cloudSize; i++) {
    point.x = segmented_cloud_->points[i].x;
    point.y = segmented_cloud_->points[i].y;
    point.z = segmented_cloud_->points[i].z;

    float ori = -atan2(point.y, point.x);
    if (!halfPassed) {
      if (ori < seg_info_.start_orientation - M_PI / 2)
        ori += 2 * M_PI;
      else if (ori > seg_info_.start_orientation + M_PI * 3 / 2)
        ori -= 2 * M_PI;

      if (ori - seg_info_.start_orientation > M_PI)
        halfPassed = true;
    } else {
      ori += 2 * M_PI;

      if (ori < seg_info_.end_orientation - M_PI * 3 / 2)
        ori += 2 * M_PI;
      else if (ori > seg_info_.end_orientation + M_PI / 2)
        ori -= 2 * M_PI;
    }

    float relTime =
        (ori - seg_info_.start_orientation) / seg_info_.orientation_diff;
    point.intensity = int(segmented_cloud_->points[i].intensity) +
                      config_.scanPeriod * relTime;

    segmented_cloud_->points[i] = point;
  }
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
void FeatureExtractor::calculateSmoothness() {
  int cloudSize = segmented_cloud_->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange = seg_info_.segmented_cloud_range[i - 5] +
                      seg_info_.segmented_cloud_range[i - 4] +
                      seg_info_.segmented_cloud_range[i - 3] +
                      seg_info_.segmented_cloud_range[i - 2] +
                      seg_info_.segmented_cloud_range[i - 1] -
                      seg_info_.segmented_cloud_range[i] * 10 +
                      seg_info_.segmented_cloud_range[i + 1] +
                      seg_info_.segmented_cloud_range[i + 2] +
                      seg_info_.segmented_cloud_range[i + 3] +
                      seg_info_.segmented_cloud_range[i + 4] +
                      seg_info_.segmented_cloud_range[i + 5];

    cloud_curvature_[i] = diffRange * diffRange;

    cloud_neighbor_picked_[i] = 0;
    cloud_label_[i] = 0;

    cloud_smoothness_[i].value = cloud_curvature_[i];
    cloud_smoothness_[i].ind = i;
  }
}

void FeatureExtractor::markOccludedPoints() {
  int cloudSize = segmented_cloud_->points.size();

  for (int i = 5; i < cloudSize - 6; ++i) {
    float depth1 = seg_info_.segmented_cloud_range[i];
    float depth2 = seg_info_.segmented_cloud_range[i + 1];
    int columnDiff = std::abs(int(seg_info_.segmented_cloud_col_ind[i + 1] -
                                  seg_info_.segmented_cloud_col_ind[i]));

    if (columnDiff < 10) {
      // 点深度变化比较剧烈,点处在近似与激光束平行的斜面上
      if (depth1 - depth2 > 0.3) {
        cloud_neighbor_picked_[i - 5] = 1;
        cloud_neighbor_picked_[i - 4] = 1;
        cloud_neighbor_picked_[i - 3] = 1;
        cloud_neighbor_picked_[i - 2] = 1;
        cloud_neighbor_picked_[i - 1] = 1;
        cloud_neighbor_picked_[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloud_neighbor_picked_[i + 1] = 1;
        cloud_neighbor_picked_[i + 2] = 1;
        cloud_neighbor_picked_[i + 3] = 1;
        cloud_neighbor_picked_[i + 4] = 1;
        cloud_neighbor_picked_[i + 5] = 1;
        cloud_neighbor_picked_[i + 6] = 1;
      }
    }

    float diff1 = std::abs(seg_info_.segmented_cloud_range[i - 1] -
                           seg_info_.segmented_cloud_range[i]);
    float diff2 = std::abs(seg_info_.segmented_cloud_range[i + 1] -
                           seg_info_.segmented_cloud_range[i]);
    // 这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
    if (diff1 > 0.02 * seg_info_.segmented_cloud_range[i] &&
        diff2 > 0.02 * seg_info_.segmented_cloud_range[i])
      cloud_neighbor_picked_[i] = 1;
  }
}

void FeatureExtractor::extractFeatures() {
  corner_points_->clear();
  surf_points_->clear();

  for (unsigned int i = 0; i < config_.N_SCAN; i++) {
    surf_points_scan_->clear();

    for (int j = 0; j < 6; j++) {
      int sp = (seg_info_.start_ring_index[i] * (6 - j) +
                seg_info_.end_ring_index[i] * j) /
               6;
      int ep = (seg_info_.start_ring_index[i] * (5 - j) +
                seg_info_.end_ring_index[i] * (j + 1)) /
                   6 -
               1;

      if (sp >= ep)
        continue;

      std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep,
                by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloud_smoothness_[k].ind;
        if (cloud_neighbor_picked_[ind] == 0 &&
            cloud_curvature_[ind] > config_.edgeThreshold &&
            seg_info_.segmented_cloud_ground_flag[ind] == false) {
          largestPickedNum++;
          if (largestPickedNum <= 20) {
            cloud_label_[ind] = 1;
            corner_points_->push_back(segmented_cloud_->points[ind]);
          } else {
            break;
          }

          cloud_neighbor_picked_[ind] = 1;
          //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
          for (int l = 1; l <= 5; l++) {
            int columnDiff =
                std::abs(int(seg_info_.segmented_cloud_col_ind[ind + l] -
                             seg_info_.segmented_cloud_col_ind[ind + l - 1]));
            if (columnDiff > 10)
              break;
            if ((ind + l) < 0 ||
                (ind + l) >= (config_.N_SCAN * config_.Horizon_SCAN)) {
              continue;
            }
            cloud_neighbor_picked_[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            int columnDiff =
                std::abs(int(seg_info_.segmented_cloud_col_ind[ind + l] -
                             seg_info_.segmented_cloud_col_ind[ind + l + 1]));
            if (columnDiff > 10)
              break;
            if ((ind + l) < 0 ||
                (ind + l) >= (config_.N_SCAN * config_.Horizon_SCAN)) {
              continue;
            }
            cloud_neighbor_picked_[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        int ind = cloud_smoothness_[k].ind;
        if (cloud_label_[k] <= 0 &&
            cloud_curvature_[ind] < config_.surfThreshold) {
          surf_points_scan_->push_back(segmented_cloud_->points[k]);
        }
      }
    }

    surf_points_scan_DS_->clear();
    downSize_filter_.setInputCloud(surf_points_scan_);
    downSize_filter_.filter(*surf_points_scan_DS_);

    *surf_points_ += *surf_points_scan_DS_;
  }
}

bool FeatureExtractor::cloudExtractor(laserCloud::Ptr corner_cloud,
                                      laserCloud::Ptr surf_cloud,
                                      laserCloud::Ptr without_ground_cloud) {
  // std::unique_lock<std::mutex> lock(mutex_);
  findStartEndAngle();
  projectPointCloud();
  groundRemoval();
  cloudSegmentation();
  // adjustDistortion();
  calculateSmoothness();
  markOccludedPoints();
  extractFeatures();

  *corner_cloud = *corner_points_;
  *surf_cloud = *surf_points_;
  *surf_cloud += *outlier_cloud_;
  *without_ground_cloud = *without_ground_cloud_;
  LOG(INFO) << "corner_points size:" << corner_cloud->points.size();
  LOG(INFO) << "suf_points size:" << surf_cloud->points.size();
  // LOG(INFO) << "without_ground_cloud size:" <<
  // without_ground_cloud->points.size();
  if (surf_cloud->points.size() < 100) {
    return false;
  }
  return true;
}

bool FeatureExtractor::fullInfoExtractor(laserCloud::Ptr full_info_cloud) {
  findStartEndAngle();
  projectPointCloud();

  *full_info_cloud = *full_info_cloud_;

  return true;
}

bool FeatureExtractor::groudRemoveExtractor(
    laserCloud::Ptr without_ground_cloud) {
  findStartEndAngle();
  projectPointCloud();
  groundRemoval();

  for (size_t i = 0; i < config_.N_SCAN; ++i) {
    for (size_t j = 0; j < config_.Horizon_SCAN; ++j) {
      if (ground_mat_.at<int8_t>(i, j) == 1 ||
          ground_mat_.at<int8_t>(i, j) == -1 ||
          range_mat_.at<float>(i, j) == FLT_MAX) {
        continue;
      }
      without_ground_cloud->push_back(
          full_cloud_->points[j + i * config_.Horizon_SCAN]);
    }
  }
  return true;
}
}  // namespace cvte_lidar_slam