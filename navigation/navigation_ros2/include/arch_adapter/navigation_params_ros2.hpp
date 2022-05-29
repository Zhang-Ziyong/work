/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file navigation_params_ros2.hpp
 *
 *@brief
 *
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev
 *@data 2020-01-16
 ************************************************************************/
#ifndef __NAVIGATION_PARAMS_ROS2_H
#define __NAVIGATION_PARAMS_ROS2_H

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "navigation_params.hpp"

namespace CVTE_BABOT {

class NavigationParamsRos2 {
 public:
  NavigationParamsRos2(
      const rclcpp::Node::SharedPtr ptr_node,
      const std::shared_ptr<NavigationParameters> ptr_navigation_params);
  ~NavigationParamsRos2() = default;

  void loadParams();

  void loadPathFollowerParams();

  void loadLogicControllerParams();

  void loadDWAControllerParams();

  void loadPIDControllerParams();

  void loadLYPUControllerParams();

  void loadStanleyControllerParams();

  void loadMPCcontrollerParams();

  void loadPathOptimizeParams();

  void loadGlobalParams();

  void loadNarrowParams();

  void loadPitParams();

  void loadPlannerDecisionParams();

  void loadAdditionCleanParams();

  std::shared_ptr<NavigationParameters> ptr_navigation_params_;

 private:
  template <typename ParamType>
  void getAndSetParam(const std::string &params_name, ParamType &params,
                      const ParamType &default_value) {
    node_->get_parameter_or(params_name, params, default_value);
    ptr_navigation_params_->setParam(params_name, params);
  }

  rclcpp::Node::SharedPtr node_;
};

}  // namespace CVTE_BABOT

#endif
