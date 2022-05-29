/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file type.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-06-05
 ************************************************************************/
#ifndef __TYPE_HPP
#define __TYPE_HPP

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace CVTE_BABOT {

typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint>::Ptr VPointsPtr;
typedef pcl::PointXY VPoint2D;

typedef uint32_t IdType;

enum TrackingState { NEW_OBJECT = 0, TRACKED_OBJECT, TEMPORARY_OBJECT };

enum MotionState { IMMOVABLE_OBJECT = 0, MOBILE_OBJECT };

enum ObjectType { UNKNOWN_TYPE = 0, STATIC_OBJECT, CAR_OBJECT, OTHER_OBJECT };

typedef Eigen::Vector2d TrajectoryPoint;
typedef std::vector<TrajectoryPoint> Trajectory;

typedef std::pair<int, int> TrackerObsvPair;

typedef std::vector<std::pair<IdType, Trajectory>> ObjectsTrajactory;

typedef std::shared_ptr<ObjectsTrajactory> ObjectsTrajPtr;
}  // namespace CVTE_BABOT
#endif
