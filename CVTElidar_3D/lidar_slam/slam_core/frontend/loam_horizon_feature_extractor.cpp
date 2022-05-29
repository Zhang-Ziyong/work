#include "frontend/loam_horizon_feature_extractor.hpp"
#include "glog/logging.h"

namespace cvte_lidar_slam {
LoamHorizonFeatureExtractor::LoamHorizonFeatureExtractor() {
  allocateMemory();
  resetVariables();
}

void LoamHorizonFeatureExtractor::allocateMemory() {
  surf_points_scan_.reset(new laserCloud);
  surf_points_scan_DS_.reset(new laserCloud);
  outlier_points_scan_.reset(new laserCloud);
  outlier_points_scan_DS_.reset(new laserCloud);
  outlier_cloud_.reset(new laserCloud);
  corner_cloud_.reset(new laserCloud);
  surf_cloud_.reset(new laserCloud);
  full_cloud_.reset(new laserCloud);
  laser_cloud_in_.reset(new laserCloud);
  laser_scan_.resize(6);
  cloudNeighborPicked_.resize(6);
  cloud_curvature_.resize(6);
  cloud_label_.resize(6);
  cloud_sort_ind_.resize(6);
  points_reflectivity_.resize(6);
  scan_smooth_vec_.resize(6);
  downSize_filter_.setLeafSize(0.2, 0.2, 0.2);
}

void LoamHorizonFeatureExtractor::resetVariables() {
  surf_points_scan_->clear();
  surf_points_scan_DS_->clear();
  outlier_points_scan_->clear();
  outlier_points_scan_DS_->clear();
  outlier_cloud_->clear();
  corner_cloud_->clear();
  full_cloud_->clear();
  laser_cloud_in_->clear();
  surf_cloud_->clear();
}

template <typename PointT>
void LoamHorizonFeatureExtractor::removeClosedTooFarPointCloud(
    const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out,
    float thres1, float thres2) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;
  double thres1_2 = thres1 * thres1;
  double thres2_2 = thres2 * thres2;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    double length = cloud_in.points[i].x * cloud_in.points[i].x +
                    cloud_in.points[i].y * cloud_in.points[i].y +
                    cloud_in.points[i].z * cloud_in.points[i].z;
    // int reflecity = floor(cloud_in.points[i].intensity);
    // reflecity = reflecity / 10;
    // double intensity = reflecity / length;
    if (length < thres1_2 || length > thres2_2)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}

bool LoamHorizonFeatureExtractor::setInputCloud(const laserCloud &cloud_in) {
  if (cloud_in.points.empty()) {
    return false;
  }
  std::vector<int> indices;
  laserCloud::Ptr laser_cloud_temp(new laserCloud);
  pcl::removeNaNFromPointCloud(cloud_in, *laser_cloud_temp, indices);
  removeClosedTooFarPointCloud(*laser_cloud_temp, *laser_cloud_in_, 1., 100);
  if (laser_cloud_in_->points.empty()) {
    return false;
  }
  return true;
  //   std::cout << cloud_in.points.size() << std::endl;
  //   *laser_cloud_in_ = cloud_in;
}
bool LoamHorizonFeatureExtractor::setInputCloud(
    const laserCloud::Ptr ptr_cloud) {
  std::vector<int> indices;
  if (ptr_cloud->points.empty()) {
    return false;
  }
  laserCloud::Ptr laser_cloud_temp(new laserCloud);
  pcl::removeNaNFromPointCloud(*ptr_cloud, *laser_cloud_temp, indices);
  removeClosedTooFarPointCloud(*laser_cloud_temp, *laser_cloud_in_, 1., 100);
  if (laser_cloud_in_->points.empty()) {
    return false;
  }
  return true;
  //   *laser_cloud_in_ = *ptr_cloud;
}

void LoamHorizonFeatureExtractor::extractFeatures() {
  for (size_t i = 0; i < 6; i++) {
    laser_scan_[i].resize(10000);
    cloudNeighborPicked_[i].resize(10000);
    cloud_curvature_[i].resize(10000);
    cloud_label_[i].resize(10000);
    cloud_sort_ind_[i].resize(10000);
    scan_smooth_vec_[i].resize(10000);
  }
  int point_count[6] = {0};
  for (size_t i = 0; i < laser_cloud_in_->points.size(); i++) {
    PointType point = laser_cloud_in_->points[i];
    int index = round(point.intensity);
    point.intensity = index / 10;
    index = index % 10;
    laser_scan_[index][point_count[index]] = point;

    point_count[index]++;
  }
  for (size_t i = 0; i < 6; i++) {
    laser_scan_[i].resize(point_count[i]);
    cloudNeighborPicked_[i].resize(point_count[i]);
    cloud_curvature_[i].resize(point_count[i]);
    cloud_label_[i].resize(point_count[i]);
    cloud_sort_ind_[i].resize(point_count[i]);
    scan_smooth_vec_[i].resize(point_count[i]);
  }
  for (size_t j = 0; j < laser_scan_.size(); j++) {
    if (laser_scan_[j].size() <= 6) {
      continue;
    }
    for (unsigned int i = 5; i < laser_scan_[j].size() - 6; i++) {
      float diffX =
          laser_scan_[j].points[i - 5].x + laser_scan_[j].points[i - 4].x +
          laser_scan_[j].points[i - 3].x + laser_scan_[j].points[i - 2].x +
          laser_scan_[j].points[i - 1].x - 10 * laser_scan_[j].points[i].x +
          laser_scan_[j].points[i + 1].x + laser_scan_[j].points[i + 2].x +
          laser_scan_[j].points[i + 3].x + laser_scan_[j].points[i + 4].x +
          laser_scan_[j].points[i + 5].x;
      float diffY =
          laser_scan_[j].points[i - 5].y + laser_scan_[j].points[i - 4].y +
          laser_scan_[j].points[i - 3].y + laser_scan_[j].points[i - 2].y +
          laser_scan_[j].points[i - 1].y - 10 * laser_scan_[j].points[i].y +
          laser_scan_[j].points[i + 1].y + laser_scan_[j].points[i + 2].y +
          laser_scan_[j].points[i + 3].y + laser_scan_[j].points[i + 4].y +
          laser_scan_[j].points[i + 5].y;
      float diffZ =
          laser_scan_[j].points[i - 5].z + laser_scan_[j].points[i - 4].z +
          laser_scan_[j].points[i - 3].z + laser_scan_[j].points[i - 2].z +
          laser_scan_[j].points[i - 1].z - 10 * laser_scan_[j].points[i].z +
          laser_scan_[j].points[i + 1].z + laser_scan_[j].points[i + 2].z +
          laser_scan_[j].points[i + 3].z + laser_scan_[j].points[i + 4].z +
          laser_scan_[j].points[i + 5].z;
      double curve = diffX * diffX + diffY * diffY + diffZ * diffZ;
      scan_smooth_vec_[j][i].value = curve;
      scan_smooth_vec_[j][i].ind = i;
      cloud_curvature_[j][i] = curve;
      cloud_label_[j][i] = 0;
      double length =
          sqrt(laser_scan_[j].points[i].x * laser_scan_[j].points[i].x +
               laser_scan_[j].points[i].y * laser_scan_[j].points[i].y +
               laser_scan_[j].points[i].z * laser_scan_[j].points[i].z);
      double intensity = laser_scan_[j].points[i].intensity / length;
      if (intensity < 20) {
        cloudNeighborPicked_[j][i] = 0;
      }
    }

    for (unsigned int i = 5; i < laser_scan_[j].size() - 6; i++) {
      //计算曲率
      float diff_x =
          laser_scan_[j].points[i + 1].x - laser_scan_[j].points[i].x;
      float diff_y =
          laser_scan_[j].points[i + 1].y - laser_scan_[j].points[i].y;
      float diff_z =
          laser_scan_[j].points[i + 1].z - laser_scan_[j].points[i].z;
      float diff = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

      //曲率阈值过滤
      if (diff > 0.1) {
        float depth1 =
            sqrt(laser_scan_[j].points[i].x * laser_scan_[j].points[i].x +
                 laser_scan_[j].points[i].y * laser_scan_[j].points[i].y +
                 laser_scan_[j].points[i].z * laser_scan_[j].points[i].z);

        float depth2 = sqrt(
            laser_scan_[j].points[i + 1].x * laser_scan_[j].points[i + 1].x +
            laser_scan_[j].points[i + 1].y * laser_scan_[j].points[i + 1].y +
            laser_scan_[j].points[i + 1].z * laser_scan_[j].points[i + 1].z);

        //针对paper中(b)情况
        if (depth1 > depth2) {
          diff_x = laser_scan_[j].points[i + 1].x -
                   laser_scan_[j].points[i].x * depth2 / depth1;
          diff_y = laser_scan_[j].points[i + 1].y -
                   laser_scan_[j].points[i].y * depth2 / depth1;
          diff_z = laser_scan_[j].points[i + 1].z -
                   laser_scan_[j].points[i].z * depth2 / depth1;

          if (sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) /
                  depth2 <
              0.1) {
            cloudNeighborPicked_[j][i - 5] = 1;
            cloudNeighborPicked_[j][i - 4] = 1;
            cloudNeighborPicked_[j][i - 3] = 1;
            cloudNeighborPicked_[j][i - 2] = 1;
            cloudNeighborPicked_[j][i - 1] = 1;
            cloudNeighborPicked_[j][i] = 1;
          }
        } else {
          diff_x = laser_scan_[j].points[i + 1].x * depth1 / depth2 -
                   laser_scan_[j].points[i].x;
          diff_y = laser_scan_[j].points[i + 1].y * depth1 / depth2 -
                   laser_scan_[j].points[i].y;
          diff_z = laser_scan_[j].points[i + 1].z * depth1 / depth2 -
                   laser_scan_[j].points[i].z;

          if (sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) /
                  depth1 <
              0.1) {
            cloudNeighborPicked_[j][i + 1] = 1;
            cloudNeighborPicked_[j][i + 2] = 1;
            cloudNeighborPicked_[j][i + 3] = 1;
            cloudNeighborPicked_[j][i + 4] = 1;
            cloudNeighborPicked_[j][i + 5] = 1;
            cloudNeighborPicked_[j][i + 6] = 1;
          }
        }
      }

      //针对paper中(a)情况
      float diff_x_2 =
          laser_scan_[j].points[i].x - laser_scan_[j].points[i - 1].x;
      float diff_y_2 =
          laser_scan_[j].points[i].y - laser_scan_[j].points[i - 1].y;
      float diff_z_2 =
          laser_scan_[j].points[i].z - laser_scan_[j].points[i - 1].z;
      float diff_2 =
          diff_x_2 * diff_x_2 + diff_y_2 * diff_y_2 + diff_z_2 * diff_z_2;

      float dis = laser_scan_[j].points[i].x * laser_scan_[j].points[i].x +
                  laser_scan_[j].points[i].y * laser_scan_[j].points[i].y +
                  laser_scan_[j].points[i].z * laser_scan_[j].points[i].z;

      if (diff > 0.0002 * dis && diff_2 > 0.0002 * dis) {
        cloudNeighborPicked_[j][i] = 1;
      }
    }
  }
  for (int i = 0; i < 6; i++) {
    surf_points_scan_->clear();
    outlier_points_scan_->clear();
    for (int j = 0; j < 6; j++) {
      if (laser_scan_[i].points.size() <= 6) {
        break;
      }
      int sp = 5 + (laser_scan_[i].points.size() - 6) * j / 6;
      int ep = 5 + (laser_scan_[i].points.size() - 6) * (j + 1) / 6;
      std::sort(scan_smooth_vec_[i].begin() + sp,
                scan_smooth_vec_[i].begin() + ep + 1, com_value());
      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = scan_smooth_vec_[i][k].ind;

        if (cloudNeighborPicked_[i][ind] == 0 && cloud_curvature_[i][ind] > 1) {
          // std::cout << "scan_smooth_vec_[i][k].value:" <<
          // scan_smooth_vec_[i][k].value << std::endl;
          largestPickedNum++;
          if (largestPickedNum <= 20) {
            cloud_label_[i][ind] = 1;
            corner_cloud_->push_back(laser_scan_[i].points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked_[i][ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laser_scan_[i].points[ind + l].x -
                          laser_scan_[i].points[ind + l - 1].x;
            float diffY = laser_scan_[i].points[ind + l].y -
                          laser_scan_[i].points[ind + l - 1].y;
            float diffZ = laser_scan_[i].points[ind + l].z -
                          laser_scan_[i].points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.2) {
              break;
            }

            cloudNeighborPicked_[i][ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laser_scan_[i].points[ind + l].x -
                          laser_scan_[i].points[ind + l + 1].x;
            float diffY = laser_scan_[i].points[ind + l].y -
                          laser_scan_[i].points[ind + l + 1].y;
            float diffZ = laser_scan_[i].points[ind + l].z -
                          laser_scan_[i].points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.2) {
              break;
            }

            cloudNeighborPicked_[i][ind + l] = 1;
          }
        }
      }
      for (int k = sp; k <= ep; k++) {
        if (cloud_label_[i][k] <= 0) {
          if (scan_smooth_vec_[i][k].value < 1) {
            surf_points_scan_->push_back(laser_scan_[i].points[k]);
          } else {
            outlier_points_scan_->push_back(laser_scan_[i].points[k]);
          }
        }
      }
    }
    surf_points_scan_DS_->clear();
    outlier_points_scan_DS_->clear();
    downSize_filter_.setInputCloud(surf_points_scan_);
    downSize_filter_.filter(*surf_points_scan_DS_);
    downSize_filter_.setInputCloud(outlier_points_scan_);
    downSize_filter_.filter(*outlier_points_scan_DS_);
    *surf_cloud_ += *surf_points_scan_DS_;
    *outlier_cloud_ += *outlier_points_scan_DS_;
  }
}

bool LoamHorizonFeatureExtractor::cloudExtractor(
    laserCloud::Ptr corner_cloud, laserCloud::Ptr surf_cloud,
    laserCloud::Ptr without_ground_cloud) {
  std::unique_lock<std::mutex> lock(mutex_);
  extractFeatures();
  *corner_cloud = *corner_cloud_;
  *surf_cloud = *surf_cloud_;
  *without_ground_cloud = *outlier_cloud_;  // 这里的数据是随便给的
  *without_ground_cloud += *surf_cloud_;
  *without_ground_cloud += *corner_cloud_;
  LOG(INFO) << "corner_points size:" << corner_cloud_->points.size();
  LOG(INFO) << "suf_points size:" << surf_cloud->points.size();
  return true;
}

}  // namespace cvte_lidar_slam