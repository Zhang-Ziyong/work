#include "frontend/relocalization_extractor.hpp"

#include <glog/logging.h>
#include <pcl/filters/random_sample.h>
#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace cvte_lidar_slam;

RelocalizationExtractor::RelocalizationExtractor() {
  downSize_filter_.setLeafSize(0.2, 0.2, 0.1);
}
RelocalizationExtractor::~RelocalizationExtractor() {}

bool RelocalizationExtractor::fixSizeFilter(const laserCloud::Ptr cloud_in,
                                            laserCloud::Ptr cloud_out,
                                            const double distance,
                                            const int down_size_num) {
  if (down_size_num < 0 || distance < 0 || cloud_in == nullptr ||
      cloud_out == nullptr) {
    return false;
  }
  cloud_out->clear();
  PointType point;

  laserCloud::Ptr ptr_cloud_tmp;
  ptr_cloud_tmp.reset(new laserCloud());
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    point = cloud_in->points[i];
    if (point.x < distance && point.y < distance && point.z < distance) {
      ptr_cloud_tmp->push_back(point);
    }
  }
  std::cout << "input cloud size: " << ptr_cloud_tmp->points.size()
            << std::endl;

  if (ptr_cloud_tmp->points.size() == down_size_num) {
    *cloud_out = *cloud_in;
    return true;
  } else if (ptr_cloud_tmp->points.size() > down_size_num) {
    pcl::RandomSample<PointType> rs_filter;
    rs_filter.setInputCloud(ptr_cloud_tmp);
    rs_filter.setSample(down_size_num);
    rs_filter.filter(*cloud_out);
    return true;
  } else {
    laserCloud::Ptr ds_cloud_tmp;
    ds_cloud_tmp.reset(new laserCloud());
    pcl::RandomSample<PointType> rs_filter;
    rs_filter.setInputCloud(ptr_cloud_tmp);
    rs_filter.setSample(down_size_num - ptr_cloud_tmp->points.size());
    rs_filter.filter(*ds_cloud_tmp);
    *cloud_out = *ds_cloud_tmp + *ptr_cloud_tmp;
    return true;
  }
}

bool RelocalizationExtractor::downSizeFilter(const laserCloud::Ptr cloud_in,
                                             laserCloud::Ptr cloud_out,
                                             const double distance,
                                             const int down_size_num) {
  if (down_size_num < 0 || distance < 0 || cloud_in == nullptr ||
      cloud_out == nullptr) {
    return false;
  }
  if (down_size_num > cloud_in->points.size()) {
    LOG(WARNING) << "points: " << cloud_in->points.size();
    return false;
  } else if (cloud_in->points.size() == down_size_num) {
    *cloud_out = *cloud_in;
    return true;
  }

  cloud_out->clear();
  PointType point;

  laserCloud::Ptr ptr_cloud_tmp;
  ptr_cloud_tmp.reset(new laserCloud());
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    point = cloud_in->points[i];
    if (point.x < distance && point.y < distance && point.z < distance) {
      ptr_cloud_tmp->push_back(point);
    }
  }

  // TODO: 体素滤波，并据密度进行采样？
  laserCloud::Ptr ptr_cloud_ds;
  ptr_cloud_ds.reset(new laserCloud());
  downSize_filter_.setInputCloud(ptr_cloud_tmp);
  downSize_filter_.filter(*ptr_cloud_ds);
  // std::cout << "down_size points: " << ptr_cloud_ds->points.size() <<
  // std::endl;
  if (ptr_cloud_ds->points.size() == down_size_num) {
    *cloud_out = *ptr_cloud_ds;
    // std::cout << "=" << std::endl;
  } else if (ptr_cloud_ds->points.size() < down_size_num) {
    unsigned int sample_count = down_size_num - ptr_cloud_ds->points.size();
    laserCloud random_cloud;
    pcl::RandomSample<PointType> rs_filter;
    rs_filter.setInputCloud(cloud_in);
    rs_filter.setSample(sample_count);
    rs_filter.filter(random_cloud);
    *cloud_out = *ptr_cloud_ds + random_cloud;
    // std::cout << "<" << std::endl;
  } else {
    // TODO: 如果遇到数据非常多的情况需要先做滤波？
    pcl::RandomSample<PointType> rs_filter;
    rs_filter.setInputCloud(ptr_cloud_ds);
    rs_filter.setSample(down_size_num);
    rs_filter.filter(*cloud_out);
    // std::cout << ">" << std::endl;
  }
  return true;
}

bool RelocalizationExtractor::downSizeFilterTop(const laserCloud::Ptr cloud_in,
                                                laserCloud::Ptr cloud_out,
                                                const double distance,
                                                const int down_size_num) {
  laserCloud::Ptr ptr_cloud_top;
  ptr_cloud_top.reset(new laserCloud());
  PointType point;
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    point = cloud_in->points[i];
    if (point.z > 1.0) {
      ptr_cloud_top->push_back(point);
    }
  }
  return downSizeFilter(ptr_cloud_top, cloud_out, distance, down_size_num);
}

bool RelocalizationExtractor::clusterFilter(const laserCloud::Ptr cloud_in,
                                            laserCloud::Ptr cloud_out,
                                            const double distance,
                                            const int down_size_num) {
  if (down_size_num < 0 || distance < 0 || cloud_in == nullptr ||
      cloud_out == nullptr) {
    return false;
  }
  if (down_size_num > cloud_in->points.size()) {
    LOG(WARNING) << "points: " << cloud_in->points.size();
    return false;
  } else if (cloud_in->points.size() == down_size_num) {
    *cloud_out = *cloud_in;
    return true;
  }
  cloud_out->clear();

  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(0.5);  //设置近邻搜索的搜索半径为20cm
  ec.setMinClusterSize(1000);  //设置一个聚类需要的最少点数目为100
  ec.setMaxClusterSize(30000);  //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod(tree);  //设置点云的搜索机制
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);

  int i = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<PointType>::Ptr cloud_cluster(
        new pcl::PointCloud<PointType>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      cloud_cluster->points.push_back(cloud_in->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
    }

    pcl::io::savePCDFileASCII(
        "/home/jun/3DSLAM/data/local_map/cluster" + std::to_string(i) + ".pcd",
        *cloud_cluster);
    i++;
    *cloud_out = *cloud_out + *cloud_cluster;
  }
  pcl::io::savePCDFileASCII("/home/jun/3DSLAM/data/local_map/org.pcd",
                            *cloud_in);
  return false;
}
