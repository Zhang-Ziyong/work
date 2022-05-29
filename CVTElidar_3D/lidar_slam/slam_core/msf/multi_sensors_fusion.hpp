/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file multi_sensors_fusion.hpp
 *
 *@brief
 * 1.3D多传感器融合MSF库实现
 *
 *@modified by CJ(caojun4345@cvte.com)
 *
 *@author CJ(caojun4345@cvte.com)
 *@version V1.0
 *@data 2019-11-25
 ************************************************************************/

#ifndef CVTE_LIDAR_SLAM_MULTI_SENSORS_FUSION_HPP
#define CVTE_LIDAR_SLAM_MULTI_SENSORS_FUSION_HPP
#include <mutex>
#include <list>
#include <atomic>
#include "msf/data_type.hpp"
#include "common/math_base/slam_math.hpp"
#include "common/config/system_config.hpp"
#include "state_machine/slam_state_machine.hpp"

namespace cvte_lidar_slam {

enum OPT_MODEL { MSF_MAPPING, MSF_LOCALIZATION };

class GpsAdapter;
class KeyFrame;
class MapManager;
// class MsfConfig;

/**
 * MutliSensorsFusion
 * @brief 实现多传感器融合算法类
 * 1. 融合多种传感器数据
 * 2. 修改graph内keyframe状态
 **/
class MutliSensorsFusion {
 public:
  /**
   * MutliSensorsFusion
   * @brief 构造函数
   * @param[in] options-算法设置参数
   **/
  MutliSensorsFusion(const MsfConfig &config);

  /**
   * ～MutliSensorsFusion
   * @brief 析构函数
   **/
  ~MutliSensorsFusion();

  /**
   * addKeyFrame
   * @brief 添加3D激光关键帧数据
   * @param[in] ptr_keyframe-关键帧数据
   **/
  void addKeyFrame(const std::shared_ptr<KeyFrame> ptr_keyframe);

  void addMapKeyFrame(const std::shared_ptr<KeyFrame> ptr_keyframe);

  /**
   * addLoop
   * @brief 添加3D激光闭环信息
   * @param[in] loop-闭环信息
   **/
  bool addLoop(const LoopConstrain &loop);

  /**
   * optimization
   * @brief 运行优化
   **/
  bool optimization();
  /**
   * Reset
   * @brief 重载优化器状态
   **/
  void Reset();

  void setGpsTrans(const Mat34d &gps_trans);

  Mat34d getGpsTrans();

  /**
   * getMutex
   * @brief 获取graph锁
   * @return graph锁
   **/
  inline std::recursive_mutex &getMutex() { return graph_mutex_; }

  /**
   * getNodeSize
   * @brief 获取graph大小
   * @return graph大小
   **/
  inline unsigned int getNodeSize() { return ptr_l_node_->size(); }

  /**
   * msfRunning
   * @brief 获取优化器运行状态
   * @return true-正在运行， false-停止运行
   **/
  inline bool msfRunning() { return is_optimize_runing_; }

  /**
   * getLoopConstraints
   * @brief 获取参与优化的loop数据
   * @return loop数据
   **/
  std::shared_ptr<std::list<LoopConstrain>> getLoopConstraints();

  /**
   * getNode
   * @brief 获取参与优化的node数据
   * @return node数据
   **/
  std::shared_ptr<std::list<Node>> getNode();

  /**
   * getLastNodePose
   * @brief 获取最后一个node的pose数据
   * @return pose数据
   **/
  Mat34d getLastNodePose();

  /**
   * getLastOdomPose
   * @brief 获取最后一个node的odom pose数据
   * @return odom pose数据
   **/
  Mat34d getLastOdomPose();

  bool saveGraph(const std::string &filename);

  bool loadGraph(const std::string &filename);

 private:
  /**
   * buildNode
   * @brief 利用关键帧数据构建node
   * @param[in] ptr_keyframe-关键帧
   * @return node数据
   **/
  Node &buildNode(const std::shared_ptr<KeyFrame> ptr_keyframe);
  Node &buildMapNode(const std::shared_ptr<KeyFrame> ptr_keyframe);

      std::atomic<bool> is_optimize_runing_;  ///< 优化器运行状态

  std::shared_ptr<std::list<Node>> ptr_l_node_;  ///<参与优化的node数据
  std::shared_ptr<std::list<std::shared_ptr<KeyFrame>>>
      ptr_l_ptr_keyframe_buffer_;  ///< keyfram缓存区

  std::shared_ptr<std::list<LoopConstrain>>
      ptr_l_loop_constrain_;  ///< 参与优化的loop数据

  std::shared_ptr<MapManager> ptr_map_manager_ = nullptr;  ///< 数据容器指针

  std::recursive_mutex buffer_mutex_;  ///< ptr_l_ptr_keyframe_buffer_数据锁
  std::recursive_mutex graph_mutex_;   ///< graph 数据锁
  std::recursive_mutex loop_constrain_mutex_;  ///< ptr_l_loop_constrain_数据锁

  unsigned int new_node_size_;  ///< 新节点计算

  MsfConfig config_;  ///< 优化器参数对象

  bool set_gps_ex_;  ///< 是否已设置GPS外参标志位
  Mat34d T_ex_;      ///< GPS外参

  unsigned int new_loop_count_;  ///< 新loop数据计数
  unsigned int loop_count_;      ///< loop数据计数
  unsigned int new_gps_count_;   ///< 新GPS数据计数

  std::shared_ptr<SlamStateMachine> ptr_state_machine_;
};

}  // namespace cvte_lidar_slam

#endif