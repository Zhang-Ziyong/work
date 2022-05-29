/*
 * @Author: your name
 * @Date: 2020-10-31 10:34:18
 * @LastEditTime: 2020-12-17 09:50:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/laser_object_track/laser_object_track.cpp
 */
#include "laser_object_track.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include "log.hpp"
#include <algorithm>
#include <map>
namespace TRACKING_MOTION {
LaserObjectTrack::LaserObjectTrack(const LaserObjectTrackConfig &config) {
  config_ = config;
  is_init_track_ = false;
  int grid_length = 2 * config_.region_interest_ / config_.grid_resolution_;
  grid_pts_.resize(grid_length);
  for (int j = 0; j < grid_length; ++j) { grid_pts_[j].resize(grid_length); }
  tracking_pose_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
}

bool LaserObjectTrack::isInitTrack() {
  return is_init_track_;
}

void LaserObjectTrack::setPredictPose(const Mat34d &predict_pose) {
  predict_pose_mutex_.lock();
  predict_pose_ = predict_pose;
  predict_pose_mutex_.unlock();
}

void LaserObjectTrack::getTrackingPose(Mat34d &tracking_pose) {
  tracking_pose_mutex_.lock();
  tracking_pose = tracking_pose_;
  tracking_pose_mutex_.unlock();
}

bool LaserObjectTrack::gridMapFilter(const laserCloud::Ptr ptr_cloud_in,
                                     const laserCloud::Ptr ptr_no_ground,
                                     const laserCloud::Ptr ptr_ground) {
  if (ptr_cloud_in == nullptr || ptr_no_ground == nullptr ||
      ptr_ground == nullptr) {
    LOG(ERROR) << "input cloud has nullptr";
    return false;
  }
  // grid map init
  int grid_length = 2 * config_.region_interest_ / config_.grid_resolution_;

  // fill the grid计算点位于哪个网格，填充
  for (auto &vp_grid : vvp_grid_filled_) {
    vp_grid->clear();
    vp_grid->shrink_to_fit();
  }
  vvp_grid_filled_.clear();
  vvp_grid_filled_.shrink_to_fit();

  for (const auto &point : ptr_cloud_in->points) {
    int GridI = (config_.region_interest_ - point.x) / config_.grid_resolution_;
    int GridJ = (config_.region_interest_ - point.y) / config_.grid_resolution_;

    auto vp_grid = &grid_pts_[GridI][GridJ];

    // 先前没有被填充点
    if (vp_grid->size() == 0) {
      vvp_grid_filled_.push_back(vp_grid);
    }

    vp_grid->push_back(point);
  }

  // calculate the grid height
  for (const auto &vp_grid : vvp_grid_filled_) {
    // 有更高效的方法?
    auto max = std::max_element(
        vp_grid->begin(), vp_grid->end(),
        [](PointType left, PointType right) { return left.z < right.z; });
    auto min = std::min_element(
        vp_grid->begin(), vp_grid->end(),
        [](PointType left, PointType right) { return left.z < right.z; });

    float Height_dif = max->z - min->z;  // 网格中的高度差
    if (Height_dif < config_.th_grid_het_ && max->z < config_.sensor_height_) {
      //根据高度差阈值和高度阈值，初步将点分为地面点和非地面点
      for (const auto &point : *vp_grid) {
        ptr_ground->points.push_back(point);
      }
    } else {
      for (const auto &point : *vp_grid) {
        ptr_no_ground->points.push_back(point);
      }
    }
  }
  return true;
}

bool LaserObjectTrack::initTracking(const laserCloud::Ptr laser_cloud_in) {
  if (laser_cloud_in == nullptr) {
    LOG(ERROR) << "input point cloud is nullptr";
    return false;
  }
  laserCloud::Ptr laser_cloud_interes(new laserCloud());
  for (size_t index = 0; index < laser_cloud_in->points.size(); index++) {
    if (laser_cloud_in->points[index].x > 1 &&
        laser_cloud_in->points[index].x < 3.5 &&
        fabs(laser_cloud_in->points[index].y) < 1.0 &&
        fabs(laser_cloud_in->points[index].z) < 2.0) {
      laser_cloud_interes->push_back(laser_cloud_in->points[index]);
    }
  }
  laserCloud::Ptr ptr_no_ground(new laserCloud());
  laserCloud::Ptr ptr_ground(new laserCloud());
  if (gridMapFilter(laser_cloud_interes, ptr_no_ground, ptr_ground)) {
    if (ptr_no_ground->points.size() < 50) {
      LOG(ERROR) << "no groud points size is small than 50 : "
                 << ptr_no_ground->points.size();
      return false;
    }
    pcl::search::KdTree<pcl::PointXYZI>::Ptr ptr_kd_tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    ptr_kd_tree->setInputCloud(ptr_no_ground);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean_cluster;
    euclidean_cluster.setClusterTolerance(0.3);  //设置近邻搜索的搜索半径为2cm
    euclidean_cluster.setMinClusterSize(
        50);  //设置一个聚类需要的最少点数目为100
    euclidean_cluster.setMaxClusterSize(
        25000);  //设置一个聚类需要的最大点数目为25000
    euclidean_cluster.setSearchMethod(ptr_kd_tree);  //设置点云的搜索机制
    euclidean_cluster.setInputCloud(ptr_no_ground);
    euclidean_cluster.extract(
        cluster_indices);  //从点云中提取聚类，并将点云索引保存在cluster_indices中
    /*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
    //迭代访问点云索引cluster_indices，直到分割出所有聚类
    if (cluster_indices.size() == 0) {
      return false;
    }
    LOG(ERROR) << "cluster size: " << cluster_indices.size();
    std::vector<laserCloud::Ptr> vec_laser_cluster;
    for (std::vector<pcl::PointIndices>::const_iterator it =
             cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
      laserCloud::Ptr cloud_cluster(new laserCloud);
      float min_x = 1000;
      float max_x = -1000;
      float min_y = 1000;
      float max_y = -1000;
      float min_z = 1000;
      float max_z = -1000;
      //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
      for (std::vector<int>::const_iterator pit = it->indices.begin();
           pit != it->indices.end(); ++pit) {
        cloud_cluster->points.push_back(ptr_no_ground->points[*pit]);  //*
        min_x = std::min(ptr_no_ground->points[*pit].x, min_x);
        max_x = std::max(ptr_no_ground->points[*pit].x, max_x);
        min_y = std::min(ptr_no_ground->points[*pit].y, min_y);
        max_y = std::max(ptr_no_ground->points[*pit].y, max_y);
        min_z = std::min(ptr_no_ground->points[*pit].z, min_z);
        max_z = std::max(ptr_no_ground->points[*pit].z, max_z);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      LOG(INFO) << "max_x: " << max_x << " min_x: " << min_x
                << " max_z: " << max_z << " min_z: " << min_z;
      double x = 0.5 * (max_x + min_x);
      double limit_height;
      if (x < config_.sensor_height_ / tan(15 * M_PI / 180.0)) {
        limit_height = 2 * x * tan(15 * M_PI / 180.0);
      } else {
        limit_height = config_.sensor_height_ + x * tan(15 * M_PI / 180.0);
      }
      LOG(INFO) << "limit_height: " << limit_height;
      if (limit_height > 1.6) {
        if (max_z + config_.sensor_height_ < 1.3) {
          continue;
        }
      } else if (max_z + config_.sensor_height_ < limit_height - 0.3) {
        continue;
      }
      vec_laser_cluster.push_back(cloud_cluster);
    }
    if (vec_laser_cluster.size() < 1) {
      LOG(ERROR) << "no object";
      return false;
    }
    double dist_y = 100;
    Eigen::Vector4d object_centroid(0, 0, 0, 0);
    for (size_t index = 0; index < vec_laser_cluster.size(); index++) {
      Eigen::Vector4d temp_centroid;
      pcl::compute3DCentroid(*vec_laser_cluster[index], temp_centroid);
      if (temp_centroid(1) < dist_y) {
        dist_y = fabs(temp_centroid(1));
        object_centroid = temp_centroid;
      }
    }
    LOG(INFO) << "init pose: " << object_centroid.transpose();
    tracking_pose_.block<3, 1>(0, 3) = object_centroid.block<3, 1>(0, 0);
    is_init_track_ = true;
    LOG(INFO) << "laser tracking init succeed";
    return true;
  } else {
    return false;
  }
}

bool LaserObjectTrack::updateTracking(const laserCloud::Ptr laser_cloud_in) {
  if (laser_cloud_in == nullptr) {
    LOG(ERROR) << "input point cloud is nullptr";
    return false;
  }
  laserCloud::Ptr laser_cloud_interes(new laserCloud());
  //   LOG(ERROR) << "predict pose: " << std::endl << predict_pose_ <<
  //   std::endl;
  for (size_t index = 0; index < laser_cloud_in->points.size(); index++) {
    if (laser_cloud_in->points[index].x > predict_pose_(0, 3) - 0.9 &&
        laser_cloud_in->points[index].x < predict_pose_(0, 3) + 0.9 &&
        laser_cloud_in->points[index].y > predict_pose_(1, 3) - 1.0 &&
        laser_cloud_in->points[index].y < predict_pose_(1, 3) + 1.0) {
      laser_cloud_interes->push_back(laser_cloud_in->points[index]);
    }
  }
  LOG(ERROR) << "laser_cloud_interes size: "
             << laser_cloud_interes->points.size();
  laserCloud::Ptr ptr_no_ground(new laserCloud());
  laserCloud::Ptr ptr_ground(new laserCloud());
  if (gridMapFilter(laser_cloud_interes, ptr_no_ground, ptr_ground)) {
    if (ptr_no_ground->points.size() < 20) {
      LOG(ERROR) << "no groud points size is small than 20 : "
                 << ptr_no_ground->points.size();
      return false;
    }
    pcl::search::KdTree<pcl::PointXYZI>::Ptr ptr_kd_tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    ptr_kd_tree->setInputCloud(ptr_no_ground);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> euclidean_cluster;
    euclidean_cluster.setClusterTolerance(0.2);  //设置近邻搜索的搜索半径为2cm
    euclidean_cluster.setMinClusterSize(
        20);  //设置一个聚类需要的最少点数目为100
    euclidean_cluster.setMaxClusterSize(
        25000);  //设置一个聚类需要的最大点数目为25000
    euclidean_cluster.setSearchMethod(ptr_kd_tree);  //设置点云的搜索机制
    euclidean_cluster.setInputCloud(ptr_no_ground);
    euclidean_cluster.extract(
        cluster_indices);  //从点云中提取聚类，并将点云索引保存在cluster_indices中
    /*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
    //迭代访问点云索引cluster_indices，直到分割出所有聚类
    if (cluster_indices.size() == 0) {
      return false;
    }
    std::vector<pcl::PointIndices> people_cluster;
    // DetectPeople(ptr_no_ground, cluster_indices, people_cluster);
    std::vector<laserCloud::Ptr> vec_laser_cluster;
    for (std::vector<pcl::PointIndices>::const_iterator it =
             cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
      float min_x = 1000;
      float max_x = -1000;
      float min_y = 1000;
      float max_y = -1000;
      float min_z = 1000;
      float max_z = -1000;
      laserCloud::Ptr cloud_cluster(new laserCloud);
      //创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
      for (std::vector<int>::const_iterator pit = it->indices.begin();
           pit != it->indices.end(); ++pit) {
        cloud_cluster->points.push_back(ptr_no_ground->points[*pit]);  //*
        min_x = std::min(ptr_no_ground->points[*pit].x, min_x);
        max_x = std::max(ptr_no_ground->points[*pit].x, max_x);
        min_y = std::min(ptr_no_ground->points[*pit].y, min_y);
        max_y = std::max(ptr_no_ground->points[*pit].y, max_y);
        min_z = std::min(ptr_no_ground->points[*pit].z, min_z);
        max_z = std::max(ptr_no_ground->points[*pit].z, max_z);
      }
      LOG(INFO) << "max_x: " << max_x << " min_x: " << min_x
                << " max_z: " << max_z << " min_z: " << min_z;
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      double x = 0.5 * (max_x + min_x);
      double limit_height;
      if (x < config_.sensor_height_ / tan(15 * M_PI / 180.0)) {
        limit_height = 2 * x * tan(15 * M_PI / 180.0);
      } else {
        limit_height = config_.sensor_height_ + x * tan(15 * M_PI / 180.0);
      }
      LOG(INFO) << "limit_height: " << limit_height;
      if (limit_height > 1.6) {
        if (max_z + config_.sensor_height_ < 1.3) {
          continue;
        }
      } else if (max_z + config_.sensor_height_ < limit_height - 0.3) {
        continue;
      }
      vec_laser_cluster.push_back(cloud_cluster);
    }
    if (vec_laser_cluster.size() < 1) {
      return false;
    }
    if (vec_laser_cluster.size() == 1) {
      Eigen::Vector4d object_centroid;
      pcl::compute3DCentroid(*vec_laser_cluster[0], object_centroid);
      tracking_pose_.block<3, 1>(0, 3) = object_centroid.block<3, 1>(0, 0);
      return true;
    }
    double predict_error = 10000000;
    Eigen::Vector4d object_centroid;
    for (size_t index = 0; index < vec_laser_cluster.size(); index++) {
      Eigen::Vector4d temp_centroid;
      pcl::compute3DCentroid(*vec_laser_cluster[index], temp_centroid);
      double error = (temp_centroid(0) - predict_pose_(0, 3)) *
                         (temp_centroid(0) - predict_pose_(0, 3)) +
                     (temp_centroid(1) - predict_pose_(1, 3)) *
                         (temp_centroid(1) - predict_pose_(1, 3));
      if (error < predict_error) {
        predict_error = error;
        object_centroid = temp_centroid;
      }
    }
    tracking_pose_.block<3, 1>(0, 3) = object_centroid.block<3, 1>(0, 0);
    return true;
  }
}
}  // namespace TRACKING_MOTION
