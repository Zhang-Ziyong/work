/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_range_data.hpp
 *
 *@brief range sensor data storage class.
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *
 *@modified huabo wu(wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-11
 ************************************************************************/

#ifndef __COSTMAP_RANGE_DATA_HPP
#define __COSTMAP_RANGE_DATA_HPP
#include <string>
#include "costmap_utils.hpp"
#include "eigen3/Eigen/Core"
namespace CVTE_BABOT {

/**
 * CostmapRangeData
 * @brief
 * costmap中范围传感器数据结构，比如超声、声呐
 **/
struct CostmapRangeData {
  CostmapRangeData()
      : max_range_(0.0), min_range_(0.0), field_of_view_(0.0), range_(0.0) {}

  ~CostmapRangeData() = default;
  CostmapRangeData(const CostmapRangeData &obs) = default;
  CostmapRangeData &operator=(const CostmapRangeData &obs) = default;

  bool isRangeMax() const { return fabs(range_ - max_range_) < 0.0001; }

  std::string s_topic_name_;
  std::string s_fram_id_;

  double max_range_;      ///< 最大范围
  double min_range_;      ///< 最小范围
  double field_of_view_;  ///< 可视范围
  double range_;          ///< 传感器数据，障碍物距离

  WorldmapPose sensor_pose_;  ///<传感器姿态
  WorldmapPose world_pose_;   ///<世界坐标

  std::vector<Eigen::Vector2d> v_clear_product;  ///<清除框法向量
  std::vector<Eigen::Vector2d> v_clear_origin;   ///<清除框法向量
};
}  // namespace CVTE_BABOT
#endif  // __COSTMAP_RANGE_DATA_HPP
