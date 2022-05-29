/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file min_box_object_builder.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#include <glog/logging.h>

#include "object_builders/min_box_object_builder.hpp"

namespace CVTE_BABOT {
void MinBoxObjectBuilder::buildObjects(
    const std::vector<pcl::PointCloud<VPoint>::Ptr> &v_ptr_point_cloud,
    std::vector<ObjectPtr> &objects_obsved) {
  if (!v_ptr_point_cloud.empty()) {
    objects_obsved.reserve(v_ptr_point_cloud.size());
  }

  for (const auto &ptr_point_cloud : v_ptr_point_cloud) {
    ObjectPtr ptr_object = std::make_shared<Object>();
    bool build_status = buildObject(ptr_point_cloud, *ptr_object);
    if (build_status) {
      objects_obsved.push_back(ptr_object);
    }
  }
}

bool MinBoxObjectBuilder::buildObject(
    const pcl::PointCloud<VPoint>::Ptr &ptr_point_cloud, Object &object) {
  if (ptr_point_cloud->size() == 0) {
    return false;
  }

  object.ptr_point_cloud = ptr_point_cloud;

  // centriodPoint记录点云集所形成bounding box的形心中心点坐标
  // 计算centriodPoint的中心坐标点坐标
  double min_x = std::numeric_limits<double>::max(),
         max_x = -std::numeric_limits<double>::max(),
         min_y = std::numeric_limits<double>::max(),
         max_y = -std::numeric_limits<double>::max(),
         max_z = -std::numeric_limits<double>::max();
  for (size_t j = 0; j < ptr_point_cloud->size(); j++) {
    if (ptr_point_cloud->points[j].z > 0.8) {
      continue;
    }

    if (ptr_point_cloud->points[j].x < min_x) {
      min_x = ptr_point_cloud->points[j].x;
    }
    if (ptr_point_cloud->points[j].x > max_x) {
      max_x = ptr_point_cloud->points[j].x;
    }

    if (ptr_point_cloud->points[j].y < min_y) {
      min_y = ptr_point_cloud->points[j].y;
    }
    if (ptr_point_cloud->points[j].y > max_y) {
      max_y = ptr_point_cloud->points[j].y;
    }

    if (ptr_point_cloud->points[j].z > max_z) {
      max_z = ptr_point_cloud->points[j].z;
    }
  }
  /**
   * 计算bounding box消息类新所需要的参数值
   * length：长，若该类别点云集平行于Z轴方向分布，则定义其长为0.6m
   * width：宽，若该类别点云集平行于Z轴方向分布，则定义其宽为0.6m
   * heigth：高
   */
  object.length = max_x - min_x;
  object.width = max_y - min_y;
  object.height = max_z;

  object.object_center(0) = (max_x + min_x) / 2.0;
  object.object_center(1) = (max_y + min_y) / 2.0;
  object.object_center(2) = max_z / 2.0;

  if (object.length < 0.3) {
    object.length = 0.3;
  }

  if (object.width < 0.3) {
    object.width = 0.3;
  }

  return true;
}

}  // namespace CVTE_BABOT
