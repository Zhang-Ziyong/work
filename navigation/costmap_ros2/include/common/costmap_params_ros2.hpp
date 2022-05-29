/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_params_ros2.hpp
 *
 *@brief
 * 通过ros配置各层的参数的参数类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev
 *@data 2019-04-25
 ************************************************************************/
#ifndef __COSTMAP_PARAMS_ROS2_H
#define __COSTMAP_PARAMS_ROS2_H

#include <costmap_params.hpp>
#include <costmap_utils.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "costmap_mediator.hpp"

namespace CVTE_BABOT {
enum { LaserScan, PointCloud };
/**
 *
 * @brief 通过ros配置各层的参数的参数类
 **/
class CostmapParamsRos2 {
 public:
  CostmapParamsRos2(const rclcpp::Node::SharedPtr ptr_node);

  ~CostmapParamsRos2() = default;

  void setCostmapParameters(
      const std::shared_ptr<CostmapParameters> &ptr_costmap_params) {
    ptr_costmap_params_ = ptr_costmap_params;
  }

  /**
   * loadStaticLayerParams
   * @brief
   * 加载静态层参数
   *
   * */
  void loadStaticLayerParams();

  /**
   * loadInflationLayerParams
   * @brief
   * 加载膨胀层参数
   *
   * */
  void loadInflationLayerParams();

  /**
   * loadObstacleLayerParams
   * @brief
   * 加载障碍层参数
   *
   * */
  void loadObstacleLayerParams();

  /**
   * loadSonarLayerParams
   * @brief
   * 加载超声层参数
   *
   * */
  void loadSonarLayerParams();

  void loadRangeSensorLayer2Params();
  /**
   * loadCollisionLayerParams
   * @brief
   * 加载碰撞开关层参数
   *
   * */
  void loadCollisionLayerParams();

  /**
   * loadPointCloudParams
   * @brief
   * 加载3d处理器参数
   *
   * */
  void loadPointCloudParams();

  /**
   * loadRangeParams
   * @brief
   * 加载超声处理器参数
   *
   * */
  void loadRangeParams();

  /**
   * loadDynamicObsParams
   * @brief
   * 加载动态障碍处理器参数
   *
   * */
  void loadDynamicObsParams();

  /**
   * loadSwitchParams
   * @brief
   * 加载碰撞传感器参数
   *
   * */
  void loadSwitchParams();

  /**
   * loadRangeSwitchParams
   * @brief
   *     加载断崖红外传感器参数
   */
  void loadRangeSwitchParams();

  /**
   * @brief 加载避障红外参数
   * 
   */
  void loadInfraredObstacleSwitchRangeParams();

  void loadRangeIRParams();

  std::map<std::string, std::string> topic_to_layer_;  ///<

  LayerParammeter layer_param_;  ///< 存储需要生成的layer的类型和名字
  std::shared_ptr<CostmapParameters> ptr_costmap_params_;  ///< costmap参数

 private:
  template <typename ExpectType>
  ExpectType getAndSetParam(const std::string &expect_index,
                            const std::string &target_index,
                            const ExpectType &default_value) {
    ExpectType param_value;
    node_->get_parameter_or(expect_index, param_value, default_value);
    ptr_costmap_params_->setParam(target_index, param_value);
    return param_value;
  }

  rclcpp::Node::SharedPtr node_;  ///< 节点
};

}  // namespace CVTE_BABOT

#endif