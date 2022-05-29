/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file data_type.hpp
 *
 *@brief
 * 1.3D多传感器融合MSF库数据类型
 *
 *@modified by CJ(caojun4345@cvte.com)
 *
 *@author CJ(caojun4345@cvte.com)
 *@version V1.0
 *@data 2019-11-25
 ************************************************************************/

#ifndef DATA_TYPE_HPP
#define DATA_TYPE_HPP
#include <memory>
#include "common/math_base/slam_math.hpp"

namespace cvte_lidar_slam {
class KeyFrame;

/**
 * LoopConstrain
 * @brief 闭环约束数据类型
 **/
class LoopConstrain {
 public:
  /**
   * LoopConstrain
   * @brief 构造函数
   **/
  LoopConstrain() = default;
  /**
   * ～LoopConstrain
   * @brief 析构函数
   **/
  ~LoopConstrain() = default;
  std::shared_ptr<KeyFrame> ptr_keyframe1;  ///< 节点1对应keyframe数据的指针
  std::shared_ptr<KeyFrame> ptr_keyframe2;  ///< 节点2对应keyframe数据的指针
  size_t loop_node_id1;                     ///< 闭环边一端连接的node ID
  size_t loop_node_id2;  ///< 闭环边另一端连接的node ID
  Mat34d relative_pose;  ///< 闭环边连接的两个node相对pose
  Mat6d rp_cov;          ///< 闭环边连接的两个node相对pose 协方差
};

/**
 * Node
 * @brief 节点数据类型
 **/
class Node {
 public:
  /**
   * Node
   * @brief 构造函数
   **/
  Node() = default;
  /**
   *～ Node
   * @brief 析构函数
   **/
  ~Node() = default;

  size_t idx;  ///< 节点ID

  std::shared_ptr<KeyFrame> ptr_keyframe;  ///< 节点对应keyframe数据的指针
  Mat34d relative_pose_constraint;  ///< 与上一节点连接的二元边（相对pose）
  Mat6d relative_pose_cov;  ///< 与上一节点连接的二元边（相对pose）协方差
  bool is_odom_valid = true; ///< 相对约束是否有效，eg：轮子打滑下就无效

  Mat34d global_pose_constraint;  ///< 节点对应的一元边（全局定位pose）
  Mat6d global_pose_cov;  ///< 节点对应的一元边（全局定位pose）协方差

  bool gps_valued = false;  ///< 节点对应的gps一元边（gps定位point）是否有效
  Vec3d gps_pose;  ///< 节点对应的gps一元边（gps定位point）
  Mat3d gps_cov;   ///< 节点对应的gps一元边（gps定位point）协方差
};

}  // namespace cvte_lidar_slam
#endif