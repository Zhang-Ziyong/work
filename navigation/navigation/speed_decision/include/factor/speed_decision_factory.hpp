/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file speed_decision_factory.hpp
 *
 *@brief 速度决策工厂类
 *
 *@author chenweijian chenweijian@cvte.com
 *@modified
 *@version
 *@data
 ************************************************************************/
#ifndef SPEED_DECISION_FACTOR_HPP_
#define SPEED_DECISION_FACTOR_HPP_
#include "rclcpp/rclcpp.hpp"
#include "speed_decision_base.hpp"
#include "C5/speed_decision_C5.hpp"
#include "C3/speed_decision_C3.hpp"
#include "navigation_mediator.hpp"
#include "planner_utils.hpp"
#include "robot_info.hpp"

namespace CVTE_BABOT {
class SpeedDecision_Factor {
 public:
  SpeedDecision_Factor() = default;

  /**
   * @brief Create a Seed Decision object
   *
   */
  static void createSeepDecision(const rclcpp::Node::SharedPtr &node,
                                 const double &controller_frequency) {
    controller_frequency_ = controller_frequency;
    speed_base_config_ = std::make_shared<SpeedDecisionBaseConfig>();
    setConfig(node);
    std::shared_ptr<SpeedDecisionBase> ptr_speed_decision_;
    ROBOTTYPE robot_type = RobotInfo::getPtrInstance()->getRobotType();
    if (robot_type == KAVA_CLEAN_C5) {
      ptr_speed_decision_ =
          std::make_shared<SpeedDecisionC5>(speed_base_config_);
    } else if (robot_type == KAVA_CLEAN_C3) {
      ptr_speed_decision_ =
          std::make_shared<SpeedDecisionC3>(speed_base_config_);
    } else {
      ptr_speed_decision_ =
          std::make_shared<SpeedDecisionC5>(speed_base_config_);
    }
    SpeedDecisionBase::setInstance(ptr_speed_decision_);
  }

  static void setConfig(const rclcpp::Node::SharedPtr &node) {
    double t_bl = 0.8;
    node->get_parameter_or("speed_decision_base.T_bl", t_bl, t_bl);
    Eigen::Matrix<double, 2, 3> T_bl;
    T_bl << 1, 0, t_bl, 0, 1, 0;
    speed_base_config_->T_bl = T_bl;

    double max_v_inc = 0.3;
    double max_w_inc = 0.3;
    double max_v_dec = -3.0;
    double max_w_dec = -3.0;

    node->get_parameter_or("speed_decision_base.path_slow_length",
                           speed_base_config_->path_slow_length,
                           speed_base_config_->path_slow_length);
    node->get_parameter_or("speed_decision_base.max_v",
                           speed_base_config_->max_v,
                           speed_base_config_->max_v);
    node->get_parameter_or("speed_decision_base.min_v",
                           speed_base_config_->min_v,
                           speed_base_config_->min_v);
    node->get_parameter_or("speed_decision_base.max_w",
                           speed_base_config_->max_w,
                           speed_base_config_->max_w);
    node->get_parameter_or("speed_decision_base.min_w",
                           speed_base_config_->min_w,
                           speed_base_config_->min_w);
    node->get_parameter_or("speed_decision_base.max_v_inc", max_v_inc,
                           max_v_inc);
    node->get_parameter_or("speed_decision_base.max_w_inc", max_w_inc,
                           max_w_inc);
    node->get_parameter_or("speed_decision_base.max_v_dec", max_v_dec,
                           max_v_dec);
    node->get_parameter_or("speed_decision_base.max_w_dec", max_w_dec,
                           max_w_dec);
    node->get_parameter_or("speed_decision_base.extra_stop_v_radio",
                           speed_base_config_->extra_stop_v_radio,
                           speed_base_config_->extra_stop_v_radio);
    node->get_parameter_or("speed_decision_base.extra_stop_w_radio",
                           speed_base_config_->extra_stop_w_radio,
                           speed_base_config_->extra_stop_w_radio);
    node->get_parameter_or("speed_decision_base.slow_cost",
                           speed_base_config_->slow_cost,
                           speed_base_config_->slow_cost);
    node->get_parameter_or("speed_decision_base.stop_cost",
                           speed_base_config_->stop_cost,
                           speed_base_config_->stop_cost);
    node->get_parameter_or("speed_decision_base.max_slope",
                           speed_base_config_->max_slope,
                           speed_base_config_->max_slope);
    node->get_parameter_or("speed_decision_base.min_slope",
                           speed_base_config_->min_slope,
                           speed_base_config_->min_slope);
    node->get_parameter_or("speed_decision_base.costmap_traverse_inc",
                           speed_base_config_->costmap_traverse_inc,
                           speed_base_config_->costmap_traverse_inc);

    node->get_parameter_or("speed_decision_base.max_camera_distance",
                           speed_base_config_->max_camera_distance,
                           speed_base_config_->max_camera_distance);

    node->get_parameter_or("speed_decision_base.driver_type",
                           speed_base_config_->driver_type,
                           speed_base_config_->driver_type);

    speed_base_config_->max_v_inc = max_v_inc / controller_frequency_;
    speed_base_config_->max_w_inc = max_w_inc / controller_frequency_;
    speed_base_config_->max_v_dec = max_v_dec / controller_frequency_;
    speed_base_config_->max_w_dec = max_w_dec / controller_frequency_;

    LOG(INFO) << "speed_decision params set : " << std::endl
              << " t_bl: " << t_bl << std::endl
              << " max_v: " << speed_base_config_->max_v
              << " min_v: " << speed_base_config_->min_v << std::endl
              << " max_w: " << speed_base_config_->max_w
              << " min_w: " << speed_base_config_->min_w << std::endl
              << " max_v_inc: " << speed_base_config_->max_v_inc
              << " max_w_inc: " << speed_base_config_->max_w_inc << std::endl
              << " max_v_dec: " << speed_base_config_->max_v_dec
              << " max_w_dec: " << speed_base_config_->max_w_dec << std::endl
              << " extra_stop_v_radio: "
              << speed_base_config_->extra_stop_v_radio << std::endl
              << " extra_stop_w_radio: "
              << speed_base_config_->extra_stop_w_radio << std::endl
              << " slow_cost: " << speed_base_config_->slow_cost << std::endl
              << " stop_cost: " << speed_base_config_->stop_cost << std::endl
              << " max_slope: " << speed_base_config_->max_slope
              << " min_slope: " << speed_base_config_->min_slope << std::endl
              << " costmap_traverse_inc: "
              << speed_base_config_->costmap_traverse_inc << std::endl
              << "driver_type: " << speed_base_config_->driver_type;

    std::string using_name;
    node->get_parameter_or("speed_decision_base.shape.name", using_name,
                           using_name);
    std::string shape_name;
    std::stringstream ss_(using_name);
    while (ss_ >> shape_name) {
      // creat a shape_map;
      speed_base_config_->m_shape[shape_name];

      // read dir_shape from each shap
      std::string shape_path = "speed_decision_base.shape." + shape_name + ".";
      std::string using_dir;
      node->get_parameter_or(shape_path + "dir_name", using_dir, using_dir);
      std::string dir_name;
      std::stringstream ss(using_dir);

      // set each dir shape
      while (ss >> dir_name) {
        std::string dir_path = shape_path + dir_name + ".";
        StopShapeConfig temp_shape;

        temp_shape.key = dir_name;
        node->get_parameter_or(dir_path + "key", temp_shape.key,
                               temp_shape.key);
        node->get_parameter_or(dir_path + "params", temp_shape.params,
                               temp_shape.params);
        node->get_parameter_or(dir_path + "start_theta", temp_shape.start_theta,
                               temp_shape.start_theta);
        node->get_parameter_or(dir_path + "end_theta", temp_shape.end_theta,
                               temp_shape.end_theta);

        speed_base_config_->m_shape[shape_name].push_back(temp_shape);

        // prinrf config
        LOG(INFO) << "speed_shape config set " << std::endl
                  << "dir_name : " << dir_name << std::endl
                  << "dir_path : " << dir_path << std::endl
                  << "key : "
                  << speed_base_config_->m_shape[shape_name].back().key
                  << std::endl
                  << "start_theta "
                  << speed_base_config_->m_shape[shape_name].back().start_theta
                  << std::endl
                  << "end_theta "
                  << speed_base_config_->m_shape[shape_name].back().end_theta
                  << std::endl;

        // LOG(INFO) << dir_path;
        // for (int i = 0;
        //      i <
        //      speed_base_config_->m_shape[shape_name].back().params.size();
        //      i++) {
        //   LOG(INFO) << "params [" << i << "] : "
        //             <<
        //             speed_base_config_->m_shape[shape_name].back().params[i]
        //             << std::endl;
        // }
      }
    }
  }

  static std::shared_ptr<SpeedDecisionBaseConfig> speed_base_config_;
  static double controller_frequency_;
};
}  // namespace CVTE_BABOT
#endif