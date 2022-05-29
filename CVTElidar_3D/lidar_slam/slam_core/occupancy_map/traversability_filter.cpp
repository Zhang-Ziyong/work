/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file traversability_filter.cpp
 *
 *@brief
 * 1.traversability mapping前端处理
 *
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author caoyong(caoyong@cvte.com)
 *@data 2019-09-30
 ************************************************************************/
#include "occupancy_map/traversability_filter.hpp"

namespace cvte_lidar_slam {
TraversabilityFilter::TraversabilityFilter() {}

void TraversabilityFilter::allocateMemory() {
  laser_cloud_in_.reset(new laserCloud());
  laser_cloud_out_.reset(new laserCloud());
  laser_cloud_obstacles_.reset(new laserCloud());
  obstacle_matrix_ = cv::Mat(config_.N_SCAN, config_.Horizon_SCAN, CV_32S,
                             cv::Scalar::all(-1));
  range_matrix_ = cv::Mat(config_.N_SCAN, config_.Horizon_SCAN, CV_32F,
                          cv::Scalar::all(-1));
  laser_cloud_matrix_.resize(config_.N_SCAN);
  for (int i = 0; i < config_.N_SCAN; ++i)
    laser_cloud_matrix_[i].resize(config_.Horizon_SCAN);

  init_flag_ = new bool *[config_.filterHeightMapArrayLength];
  for (int i = 0; i < config_.filterHeightMapArrayLength; ++i)
    init_flag_[i] = new bool[config_.filterHeightMapArrayLength];

  obst_flag_ = new bool *[config_.filterHeightMapArrayLength];
  for (int i = 0; i < config_.filterHeightMapArrayLength; ++i)
    obst_flag_[i] = new bool[config_.filterHeightMapArrayLength];

  min_height_ = new float *[config_.filterHeightMapArrayLength];
  for (int i = 0; i < config_.filterHeightMapArrayLength; ++i)
    min_height_[i] = new float[config_.filterHeightMapArrayLength];

  max_height_ = new float *[config_.filterHeightMapArrayLength];
  for (int i = 0; i < config_.filterHeightMapArrayLength; ++i)
    max_height_[i] = new float[config_.filterHeightMapArrayLength];

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

  resetVariables();
}

void TraversabilityFilter::resetVariables() {
  laser_cloud_in_->clear();
  laser_cloud_out_->clear();
  laser_cloud_obstacles_->clear();
  obstacle_matrix_ = cv::Mat(config_.N_SCAN, config_.Horizon_SCAN, CV_32S,
                             cv::Scalar::all(-1));
  range_matrix_ = cv::Mat(config_.N_SCAN, config_.Horizon_SCAN, CV_32F,
                          cv::Scalar::all(-1));

  for (int i = 0; i < config_.filterHeightMapArrayLength; ++i) {
    for (int j = 0; j < config_.filterHeightMapArrayLength; ++j) {
      init_flag_[i][j] = false;
      obst_flag_[i][j] = false;
    }
  }

  for (int j = 0; j < config_.range_x / config_.grid_resolution; ++j) {
    for (int i = 0; i < config_.range_y / config_.grid_resolution; ++i) {
      vv_grid_pointts_[j][i].clear();
    }
  }
}

void TraversabilityFilter::setInputCloud(const laserCloud &cloud_in) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloud_in, *laser_cloud_in_, indices);

  // extract range info
  // for (int i = 0; i < config_.N_SCAN; ++i) {
  //   for (int j = 0; j < config_.Horizon_SCAN; ++j) {
  //     int index = j + i * config_.Horizon_SCAN;
  //     // skip NaN point
  //     if (laser_cloud_in_->points[index].intensity ==
  //         std::numeric_limits<float>::quiet_NaN())
  //       continue;
  //     // save range info
  //     range_matrix_.at<float>(i, j) =
  //     laser_cloud_in_->points[index].intensity;
  //     // reset obstacle status to 0 - free
  //     obstacle_matrix_.at<int>(i, j) = 0;
  //   }
  // }
}

void TraversabilityFilter::cloud2Matrix() {
  for (int i = 0; i < config_.N_SCAN; ++i) {
    for (int j = 0; j < config_.Horizon_SCAN; ++j) {
      int index = j + i * config_.Horizon_SCAN;
      PointType p = laser_cloud_in_->points[index];
      laser_cloud_matrix_[i][j] = p;
    }
  }
}

void TraversabilityFilter::applyFilter() {
  positiveCurbFilter();
  negativeCurbFilter();
  slopeFilter();
}

void TraversabilityFilter::positiveCurbFilter() {
  int range_campare_neighbor_num = 3;
  float diff[config_.Horizon_SCAN - 1];

  for (int i = 0; i < config_.scanNumCurbFilter; ++i) {
    // calculate range difference
    for (int j = 0; j < config_.Horizon_SCAN - 1; ++j) {
      diff[j] =
          range_matrix_.at<float>(i, j) - range_matrix_.at<float>(i, j + 1);
    }

    for (int j = range_campare_neighbor_num;
         j < config_.Horizon_SCAN - range_campare_neighbor_num; ++j) {
      // Point that has been verified by other filters
      if (obstacle_matrix_.at<int>(i, j) == 1)
        continue;

      bool breakFlag = false;
      // point is too far away, skip comparison since it can be inaccurate
      if (range_matrix_.at<float>(i, j) > config_.sensorRangeLimit)
        continue;
      // make sure all points have valid range info
      for (int k = -range_campare_neighbor_num; k <= range_campare_neighbor_num;
           ++k)
        if (range_matrix_.at<float>(i, j + k) == -1) {
          breakFlag = true;
          break;
        }
      if (breakFlag == true)
        continue;
      // range difference should be monotonically increasing or decresing
      for (int k = -range_campare_neighbor_num;
           k < range_campare_neighbor_num - 1; ++k)
        if (diff[j + k] * diff[j + k + 1] <= 0) {
          breakFlag = true;
          break;
        }
      if (breakFlag == true)
        continue;
      // the range difference between the start and end point of neighbor points
      // is smaller than a threashold, then continue
      if (abs(range_matrix_.at<float>(i, j - range_campare_neighbor_num) -
              range_matrix_.at<float>(i, j + range_campare_neighbor_num)) /
              range_matrix_.at<float>(i, j) <
          0.03)
        continue;
      // if "continue" is not used at this point, it is very likely to be an
      // obstacle point
      obstacle_matrix_.at<int>(i, j) = 1;
    }
  }
}

void TraversabilityFilter::negativeCurbFilter() {
  int range_campare_neighbor_num = 3;

  for (int i = 0; i < config_.scanNumCurbFilter; ++i) {
    for (int j = 0; j < config_.Horizon_SCAN; ++j) {
      // Point that has been verified by other filters
      if (obstacle_matrix_.at<int>(i, j) == 1)
        continue;
      // point without range value cannot be verified
      if (range_matrix_.at<float>(i, j) == -1)
        continue;
      // point is too far away, skip comparison since it can be inaccurate
      if (range_matrix_.at<float>(i, j) > config_.sensorRangeLimit)
        continue;
      // check neighbors
      for (int m = -range_campare_neighbor_num; m <= range_campare_neighbor_num;
           ++m) {
        int k = j + m;
        if (k < 0 || k >= config_.Horizon_SCAN)
          continue;
        if (range_matrix_.at<float>(i, k) == -1)
          continue;
        // height diff greater than threashold, might be a negative curb
        if (laser_cloud_matrix_[i][j].z - laser_cloud_matrix_[i][k].z > 0.1 &&
            pointDistance(laser_cloud_matrix_[i][j],
                          laser_cloud_matrix_[i][k]) <= 1.0) {
          obstacle_matrix_.at<int>(i, j) = 1;
          break;
        }
      }
    }
  }
}

void TraversabilityFilter::slopeFilter() {
  for (int i = 0; i < config_.scanNumSlopeFilter; ++i) {
    for (int j = 0; j < config_.Horizon_SCAN; ++j) {
      // Point that has been verified by other filters
      if (obstacle_matrix_.at<int>(i, j) == 1)
        continue;
      // point without range value cannot be verified
      if (range_matrix_.at<float>(i, j) == -1 ||
          range_matrix_.at<float>(i + 1, j) == -1)
        continue;
      // point is too far away, skip comparison since it can be inaccurate
      if (range_matrix_.at<float>(i, j) > config_.sensorRangeLimit)
        continue;
      // Calculate slope angle
      float diffX =
          laser_cloud_matrix_[i + 1][j].x - laser_cloud_matrix_[i][j].x;
      float diffY =
          laser_cloud_matrix_[i + 1][j].y - laser_cloud_matrix_[i][j].y;
      float diffZ =
          laser_cloud_matrix_[i + 1][j].z - laser_cloud_matrix_[i][j].z;
      float angle =
          atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;
      // Slope angle is larger than threashold, mark as obstacle point
      if (angle < -config_.filterAngleLimit ||
          angle > config_.filterAngleLimit) {
        obstacle_matrix_.at<int>(i, j) = 1;
        continue;
      }
    }
  }
}

void TraversabilityFilter::extractFilteredCloud() {
  for (int i = 0; i < config_.scanNumMax; ++i) {
    for (int j = 0; j < config_.Horizon_SCAN; ++j) {
      // invalid points and points too far are skipped
      if (range_matrix_.at<float>(i, j) > config_.sensorRangeLimit ||
          range_matrix_.at<float>(i, j) == -1)
        continue;
      // update point intensity (occupancy) into
      PointType p = laser_cloud_matrix_[i][j];
      p.intensity = obstacle_matrix_.at<int>(i, j) == 1 ? 100 : 0;
      // save updated points
      laser_cloud_out_->push_back(p);
      // extract obstacle points and convert them to laser scan
      if (p.intensity == 100)
        laser_cloud_obstacles_->push_back(p);
    }
  }
}

void TraversabilityFilter::downsampleCloud() {
  float roundedX = float(int(robot_point_.x * 10.0f)) / 10.0f;
  float roundedY = float(int(robot_point_.y * 10.0f)) / 10.0f;
  // height map origin
  local_map_origin_.x = roundedX - config_.sensorRangeLimit;
  local_map_origin_.y = roundedY - config_.sensorRangeLimit;

  // convert from point cloud to height map
  int cloudSize = laser_cloud_out_->points.size();
  for (int i = 0; i < cloudSize; ++i) {
    int idx = (laser_cloud_out_->points[i].x - local_map_origin_.x) /
              config_.mapResolution;
    int idy = (laser_cloud_out_->points[i].y - local_map_origin_.y) /
              config_.mapResolution;

    // points out of boundry
    if (idx < 0 || idy < 0 || idx >= config_.filterHeightMapArrayLength ||
        idy >= config_.filterHeightMapArrayLength)
      continue;

    // obstacle point (decided by curb or slope filter)
    if (laser_cloud_out_->points[i].intensity == 100)
      obst_flag_[idx][idy] = true;
    // save min and max height of a grid
    if (init_flag_[idx][idy] == false) {
      min_height_[idx][idy] = laser_cloud_out_->points[i].z;
      max_height_[idx][idy] = laser_cloud_out_->points[i].z;
      init_flag_[idx][idy] = true;
    } else {
      min_height_[idx][idy] =
          std::min(min_height_[idx][idy], laser_cloud_out_->points[i].z);
      max_height_[idx][idy] =
          std::max(max_height_[idx][idy], laser_cloud_out_->points[i].z);
    }
  }
  // intermediate cloud
  pcl::PointCloud<PointType>::Ptr laserCloudTemp(
      new pcl::PointCloud<PointType>());
  // convert from height map to point cloud
  for (int i = 0; i < config_.filterHeightMapArrayLength; ++i) {
    for (int j = 0; j < config_.filterHeightMapArrayLength; ++j) {
      // no point at this grid
      if (init_flag_[i][j] == false)
        continue;
      // convert grid to point
      PointType thisPoint;
      thisPoint.x = local_map_origin_.x + i * config_.mapResolution +
                    config_.mapResolution / 2.0;
      thisPoint.y = local_map_origin_.y + j * config_.mapResolution +
                    config_.mapResolution / 2.0;
      thisPoint.z = max_height_[i][j];

      if (obst_flag_[i][j] == true /*||
          max_height_[i][j] - min_height_[i][j] > config_.filterHeightLimit*/
      ) {
        obst_flag_[i][j] = true;
        thisPoint.intensity = 100;  // obstacle
        laserCloudTemp->push_back(thisPoint);
      } else {
        thisPoint.intensity = 0;  // free
        laserCloudTemp->push_back(thisPoint);
      }
    }
  }
  *laser_cloud_out_ = *laserCloudTemp;
}

void TraversabilityFilter::predictCloudBGK() {
  int kernelGridLength =
      int(config_.predictionKernalSize / config_.mapResolution);

  for (int i = 0; i < config_.filterHeightMapArrayLength; ++i) {
    for (int j = 0; j < config_.filterHeightMapArrayLength; ++j) {
      // skip observed point
      if (init_flag_[i][j] == true)
        continue;
      PointType testPoint;
      testPoint.x = local_map_origin_.x + i * config_.mapResolution +
                    config_.mapResolution / 2.0;
      testPoint.y = local_map_origin_.y + j * config_.mapResolution +
                    config_.mapResolution / 2.0;
      testPoint.z = robot_point_.z;  // this value is not used except for
                                     // computing distance with robotPoint
      // skip grids too far
      if (pointDistance(testPoint, robot_point_) > config_.sensorRangeLimit)
        continue;
      // Training data
      std::vector<float> xTrainVec;      // training data x and y coordinates
      std::vector<float> yTrainVecElev;  // training data elevation
      std::vector<float> yTrainVecOccu;  // training data occupancy
      // Fill trainig data (vector)
      for (int m = -kernelGridLength; m <= kernelGridLength; ++m) {
        for (int n = -kernelGridLength; n <= kernelGridLength; ++n) {
          // skip grids too far
          if (std::sqrt(float(m * m + n * n)) * config_.mapResolution >
              config_.predictionKernalSize)
            continue;
          int idx = i + m;
          int idy = j + n;
          // index out of boundry
          if (idx < 0 || idy < 0 || idx >= config_.filterHeightMapArrayLength ||
              idy >= config_.filterHeightMapArrayLength)
            continue;
          // save only observed grid in this scan
          if (init_flag_[idx][idy] == true) {
            xTrainVec.push_back(local_map_origin_.x +
                                idx * config_.mapResolution +
                                config_.mapResolution / 2.0);
            xTrainVec.push_back(local_map_origin_.y +
                                idy * config_.mapResolution +
                                config_.mapResolution / 2.0);
            yTrainVecElev.push_back(max_height_[idx][idy]);
            yTrainVecOccu.push_back(obst_flag_[idx][idy] == true ? 1 : 0);
          }
        }
      }
      // no training data available, continue
      if (xTrainVec.size() == 0)
        continue;
      // convert from vector to eigen
      Eigen::MatrixXf xTrain =
          Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
              xTrainVec.data(), xTrainVec.size() / 2, 2);
      Eigen::MatrixXf yTrainElev =
          Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
              yTrainVecElev.data(), yTrainVecElev.size(), 1);
      Eigen::MatrixXf yTrainOccu =
          Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
              yTrainVecOccu.data(), yTrainVecOccu.size(), 1);
      // Test data (current grid)
      std::vector<float> xTestVec;
      xTestVec.push_back(testPoint.x);
      xTestVec.push_back(testPoint.y);
      Eigen::MatrixXf xTest =
          Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(
              xTestVec.data(), xTestVec.size() / 2, 2);
      // Predict
      Eigen::MatrixXf Ks;            // covariance matrix
      covSparse(xTest, xTrain, Ks);  // sparse kernel

      Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
      Eigen::MatrixXf ybarOccu = (Ks * yTrainOccu).array();
      Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

      // Update Elevation with Prediction
      if (std::isnan(ybarElev(0, 0)) || std::isnan(ybarOccu(0, 0)) ||
          std::isnan(kbar(0, 0)))
        continue;

      if (kbar(0, 0) == 0)
        continue;

      float elevation = ybarElev(0, 0) / kbar(0, 0);
      float occupancy = ybarOccu(0, 0) / kbar(0, 0);

      PointType p;
      p.x = xTestVec[0];
      p.y = xTestVec[1];
      p.z = elevation;
      p.intensity = (occupancy > 0.5) ? 100 : 0;

      laser_cloud_out_->push_back(p);
    }
  }
}

void TraversabilityFilter::covSparse(const Eigen::MatrixXf &xStar,
                                     const Eigen::MatrixXf &xTrain,
                                     Eigen::MatrixXf &Kxz) const {
  dist(xStar / (config_.predictionKernalSize + 0.1),
       xTrain / (config_.predictionKernalSize + 0.1), Kxz);
  Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) *
          (1.0f - Kxz.array()) / 3.0f) +
         (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f))
            .matrix() *
        1.0f;
  // Clean up for values with distance outside length scale, possible because
  // Kxz <= 0 when dist >= predictionKernalSize
  for (int i = 0; i < Kxz.rows(); ++i)
    for (int j = 0; j < Kxz.cols(); ++j)
      if (Kxz(i, j) < 0)
        Kxz(i, j) = 0;
}

void TraversabilityFilter::dist(const Eigen::MatrixXf &xStar,
                                const Eigen::MatrixXf &xTrain,
                                Eigen::MatrixXf &d) const {
  d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
  for (int i = 0; i < xStar.rows(); ++i) {
    d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
  }
}

bool TraversabilityFilter::processFilter(laserCloud::Ptr cloud_out,
                                         laserCloud::Ptr cloud_obstacle,
                                         bool mapping_mode) {
  filterGround();
  *cloud_obstacle = *laser_cloud_obstacles_;
  *cloud_out = *laser_cloud_out_;

  return true;
}

bool TraversabilityFilter::filterGround() {
  laserCloud::Ptr cloud(new laserCloud);

  for (int i = 0; i < laser_cloud_in_->points.size(); i++) {
    double range =
        sqrt(laser_cloud_in_->points[i].x * laser_cloud_in_->points[i].x +
             laser_cloud_in_->points[i].y * laser_cloud_in_->points[i].y);
    if (range < config_.sensorRangeLimit &&
        laser_cloud_in_->points[i].z < config_.sensor_height_limit) {
      cloud->push_back(laser_cloud_in_->points[i]);
      laser_cloud_out_->push_back(laser_cloud_in_->points[i]);
    }
  }

  PointType tmp_pt;
  int grid_I, grid_J;
  const int max_x = config_.range_x / config_.grid_resolution;
  const int max_y = config_.range_y / config_.grid_resolution;
  for (size_t i = 0; i < cloud->size(); ++i) {
    tmp_pt = cloud->points[i];

    grid_I = (config_.range_x / 2 + tmp_pt.x) / config_.grid_resolution;
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

      if (Height_dif < config_.height_diff_limit && max->z < 0.) {
        //根据高度差阈值和高度阈值，初步将点分为地面点和非地面点
      } else {
        for (size_t k = 0; k < vv_grid_pointts_[j][i].size(); ++k) {
          laser_cloud_obstacles_->push_back(vv_grid_pointts_[j][i][k]);
        }
      }
    }
  }

  return true;
}
}  // namespace cvte_lidar_slam