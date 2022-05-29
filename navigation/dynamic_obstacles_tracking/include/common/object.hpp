/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file object.hpp
 *
 *@brief 聚类的对象
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-06-05
 ************************************************************************/
#ifndef __OBJECT_HPP
#define __OBJECT_HPP
#include <eigen3/Eigen/Core>
#include <memory>

#include "type.hpp"

namespace CVTE_BABOT {

struct Object {
  void copyPointsFeature(const Object& obj) {
    ptr_point_cloud = obj.ptr_point_cloud;
    object_center = obj.object_center;

    length = obj.length;
    width = obj.width;
    height = obj.height;
  }

  void setMotionFeature(const Trajectory& traj, const double& vel_x,
                        const double& vel_y) {
    trajectory = traj;
    velocity(0) = vel_x;
    velocity(1) = vel_y;
  }

  bool ifAssignedId() { return id != 0; }

  // 点云相关
  pcl::PointCloud<VPoint>::Ptr ptr_point_cloud = nullptr;
  Eigen::Vector3d object_center;  // x, y, z

  double length = 0.00;
  double width = 0.00;
  double height = 0.00;

  Trajectory trajectory;
  Eigen::Vector2d velocity;

  IdType id = 0;
  TrackingState tracking_state = NEW_OBJECT;
  MotionState motion_state = IMMOVABLE_OBJECT;
  ObjectType object_type = UNKNOWN_TYPE;
};

typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<const Object> ConstObjectPtr;

typedef std::vector<ObjectPtr> ObjectPtrs;
}  // namespace CVTE_BABOT
#endif
