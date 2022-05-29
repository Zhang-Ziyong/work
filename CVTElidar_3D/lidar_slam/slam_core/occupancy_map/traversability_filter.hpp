/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file traversability_filter.hpp
 *
 *@brief
 * 1.traversability mapping前端处理
 *
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author caoyong(caoyong@cvte.com)
 *@version V1.0
 *@data 2019-09-30
 ************************************************************************/

#ifndef TRAVERSABILITY_FILTER_HPP
#define TRAVERSABILITY_FILTER_HPP

#include <opencv/cv.h>
#include "common/data_struct/pc_base.hpp"
#include "common/config/system_config.hpp"

namespace cvte_lidar_slam {

/**
 * TraversabilityFilter
 * @brief 导航所需点云滤波类
 * 1. 点云前处理
 * 2. 提取所需的scan
 * 3. 生成占用地图所需要的点云
 **/
class TraversabilityFilter {
 public:
  TraversabilityFilter();
  ~TraversabilityFilter() {}

  /**
   *setInputCloud
   *@brief
   *获取输入点云数据
   *
   *@param[in] cloud_in-输入点云
   **/
  void setInputCloud(const laserCloud &cloud_in);

  /**
   *setTransformedCloud
   *@brief
   *输入坐标系变换到map坐标的点云
   *
   *@param[in] cloud_in-输入点云, map坐标系
   **/
  void setTransformedCloud(const laserCloud &cloud_in) {
    *laser_cloud_in_ = cloud_in;
  }

  /**
   *setRobotPoint
   *@brief
   *输入机器人坐标
   *
   *@param[in] robot_point-机器人坐标, map坐标系
   **/
  void setRobotPoint(const PointType &robot_point) {
    robot_point_ = robot_point;
  }

  /**
   *allocateMemory
   *@brief
   *提前分配内存
   *
   * */
  void allocateMemory();
  /**
   *resetVariables
   *@brief
   *清空部分中间变量
   *
   **/
  void resetVariables();

  /**
   *processFilter
   *@brief
   *处理滤波, 并且获取结果点云
   *@param[in] mapping_mode-true:构图模式,false:定位模式
   *@param[out] cloud_out-输出点云
   *@param[out] cloud_obstacle-障碍物点云
   **/
  bool processFilter(laserCloud::Ptr cloud_out, laserCloud::Ptr cloud_obstacle,
                     bool mapping_mode);

  /**
   *getObstacleCloud
   *@brief
   *障碍物点云
   *@param[out] cloud_out-输出点云
   **/
  void getObstacleCloud(laserCloud &cloud_out) {
    cloud_out = *laser_cloud_obstacles_;
  }

  /**
   *getOutCloud
   *@brief
   *结果点云
   *@param[out] cloud_out-输出点云
   **/
  void getOutCloud(laserCloud &cloud_out) { cloud_out = *laser_cloud_out_; }

  /**
   *getRangeCloud
   *@brief
   *带有range信息的点云
   *@param[out] cloud_out-输出点云
   **/
  void getRangeCloud(laserCloud &cloud_out) { cloud_out = *laser_cloud_in_; }

 private:
  void cloud2Matrix();

  void applyFilter();

  bool filterGround();

  void positiveCurbFilter();

  void negativeCurbFilter();

  void slopeFilter();

  void extractFilteredCloud();

  void downsampleCloud();

  void predictCloudBGK();

  void covSparse(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain,
                 Eigen::MatrixXf &Kxz) const;

  void dist(const Eigen::MatrixXf &xStar, const Eigen::MatrixXf &xTrain,
            Eigen::MatrixXf &d) const;

  float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
  }

 private:
  laserCloud::Ptr laser_cloud_in_;   ///< 输入点云
  laserCloud::Ptr laser_cloud_out_;  ///< filtered and downsampled point cloud
  laserCloud::Ptr laser_cloud_obstacles_;  ///< cloud for saving points that are
                                           //< classified as obstables, convert
                                           //< them to laser scan
  TraverConfig config_;                    ///< 配置文件

  // Matrice
  cv::Mat obstacle_matrix_;  ///< -1 - invalid, 0 - free, 1 - obstacle
  cv::Mat range_matrix_;     ///< -1 - invalid, >0 - valid range value

  PointType robot_point_;       ///< 机器人在map坐标系中的位置点
  PointType local_map_origin_;  ///< localmap的原点

  std::vector<std::vector<PointType>>
      laser_cloud_matrix_;  ///< point cloud saved as N_SCAN * Horizon_SCAN form

  float **min_height_;
  float **max_height_;
  bool **obst_flag_;
  bool **init_flag_;

  std::vector<std::vector<std::vector<PointType>>> vv_grid_pointts_;
};  // end of class
}  // namespace cvte_lidar_slam

#endif  // TRAVERSABILITY_FILTER_HPP