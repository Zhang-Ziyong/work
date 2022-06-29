/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath:
 /src/cvte_lidar_slam/slam_core/common/data_struct/our_cloud_downsample.cpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-07-14 16:45:38
 ************************************************************************/
#include "common/data_struct/our_cloud_downsample.hpp"

namespace cvte_lidar_slam
{
CloudDownsample::CloudDownsample()
{
  frame_index_map_.clear();
  leaf_x_ = 0.5;
  leaf_y_ = 0.5;
  leaf_z_ = 0.5;
}

CloudDownsample::~CloudDownsample()
{
  frame_index_map_.clear();
}

void CloudDownsample::setLeafSize(const double x, const double y, const double z)
{
  if (x < 0.3 || y < 0.3 || z < 0.3)
  {
    std::cout << "leaf size is too small !!!" << std::endl;
    return;
  }
  leaf_x_ = std::fabs(x);
  leaf_y_ = std::fabs(y);
  leaf_z_ = std::fabs(z);
}

void CloudDownsample::InsertPoint(const PointType &point)
{
  long int x = (long int) (point.x / leaf_x_);
  long int y = (long int) (point.y / leaf_y_);
  long int z = (long int) (point.z / leaf_z_);
  Point cur_point(x, y, z);
  if (frame_index_map_.find(cur_point) == frame_index_map_.end())
  {
    frame_index_map_.insert(std::pair<Point, PointType>(cur_point, point));
  }
}

void CloudDownsample::InsertCloud(const laserCloud::Ptr &cloud_in)
{
  unsigned int frame_number = cloud_in->size();
  if (0 == frame_number)
    return;
  for (unsigned int i = 0; i < frame_number; ++i)
  {
    InsertPoint(cloud_in->points[i]);  //
  }
}

bool CloudDownsample::setOutputCloud(laserCloud::Ptr &cloud_out)
{
  if (0 == frame_index_map_.size())
  {
    return false;
  }
  laserCloud::Ptr tmp_cloud(new laserCloud());
  tmp_cloud->clear();
  for (const auto &it : frame_index_map_)
  {
    tmp_cloud->points.push_back(it.second);  //
  }
  *cloud_out = *tmp_cloud;
  return true;
}
void CloudDownsample::Clear()
{
  frame_index_map_.clear();
}

}  // namespace cvte_lidar_slam