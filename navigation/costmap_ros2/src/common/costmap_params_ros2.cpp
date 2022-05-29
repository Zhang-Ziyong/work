/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_utils_ros2.cpp
 *
 *@brief 加载各层参数
 *
 *@author chenmingjian (chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-25
 ************************************************************************/
#include "common/costmap_params_ros2.hpp"

#include <glog/logging.h>

#include <iostream>
#include <sstream>

#include "processor_utils.hpp"
#include "public_parameters/PublicParameters.hpp"

namespace CVTE_BABOT {
CostmapParamsRos2::CostmapParamsRos2(const rclcpp::Node::SharedPtr ptr_node)
    : node_(ptr_node) {}

void CostmapParamsRos2::loadStaticLayerParams() {
  ptr_costmap_params_->setParam("static_layer.enabled", true);
  getAndSetParam("static_layer.use_maximum", "static_layer.use_maximum", false);

  getAndSetParam("static_layer.trinary_costmap", "static_layer.trinary_costmap",
                 true);

  getAndSetParam("static_layer.lethal_cost_threshold",
                 "static_layer.lethal_cost_threshold", 100);
  getAndSetParam("static_layer.inflate_cost", "static_layer.inflate_cost", 253);
  getAndSetParam("static_layer.unknown_cost_value",
                 "static_layer.unknown_cost_value", 255);

  layer_param_.addLayer(CVTE_BABOT::StaticLayerType, "static_layer");
}

void CostmapParamsRos2::loadObstacleLayerParams() {
  std::string layer_name;
  std::string name;

  node_->get_parameter_or("obstacle_layer.names", layer_name,
                          std::string("obstacle_layer"));
  std::stringstream ss(layer_name);
  bool enabled = false;
  while (ss >> name) {
    node_->get_parameter_or(name + ".enabled", enabled, false);
    if (!enabled) {
      continue;
    }

    ptr_costmap_params_->setParam(name + ".enabled", true);

    getAndSetParam(name + ".use_for_navigation", name + ".use_for_navigation",
                   true);
    getAndSetParam(name + ".use_for_avoidance", name + ".use_for_avoidance",
                   true);

    getAndSetParam(name + ".footprint_clearing_enabled",
                   name + ".footprint_clearing_enabled", true);

    getAndSetParam(name + ".max_obstacle_height", name + ".max_obstacle_height",
                   2.0);
    getAndSetParam(name + ".min_obstacle_height", name + ".min_obstacle_height",
                   -0.3);

    getAndSetParam(name + ".obstacle_range", name + ".obstacle_range", 5.0);
    getAndSetParam(name + ".raytrace_range", name + ".raytrace_range", 5.0);

    getAndSetParam(name + ".combination_method", name + ".combination_method",
                   0);

    std::string layer_type;
    node_->get_parameter_or(name + ".layer_type", layer_type,
                            std::string("ObstacleLayer"));
    if (layer_type == "ObstacleLayer") {
      layer_param_.addLayer(CVTE_BABOT::ObstacleLayerType, name);

      std::string avoidance_sources = getAndSetParam(
          name + ".avoidance_cloud_sources", name + ".avoidance_cloud_sources",
          std::string("origin_scan"));
      getAndSetParam(name + ".marking_cloud_sources",
                     name + ".marking_cloud_sources", std::string(""));
      getAndSetParam(name + ".clearing_cloud_sources",
                     name + ".clearing_cloud_sources", std::string(" "));
    } else if (layer_type == "VoxelLayer") {
      layer_param_.addLayer(CVTE_BABOT::VoxelLayerType, name);

      getAndSetParam(name + ".clearing_time_threshold",
                     name + ".clearing_time_threshold", 5.0);
      getAndSetParam(name + ".quick_clearing_bit_threshold",
                     name + ".quick_clearing_bit_threshold", 10);
      getAndSetParam(name + ".quick_clearing_time_threshold",
                     name + ".quick_clearing_time_threshold", 5.0);

      getAndSetParam(name + ".origin_z", name + ".origin_z", 0.0);
      getAndSetParam(name + ".z_voxels", name + ".z_voxels", 10);
      getAndSetParam(name + ".z_resolution", name + ".z_resolution", 0.1);

      getAndSetParam(name + ".unknown_threshold", name + ".unknown_threshold",
                     10);
      getAndSetParam(name + ".mark_threshold", name + ".mark_threshold", 0);

      getAndSetParam(name + ".marking_cloud_sources",
                     name + ".marking_cloud_sources", std::string(""));
      getAndSetParam(name + ".clearing_cloud_sources",
                     name + ".clearing_cloud_sources", std::string(""));
    } else if (layer_type == "ProbabilityVoxelLayer") {
      layer_param_.addLayer(CVTE_BABOT::ProbabilityVoxelLayerType, name);

      getAndSetParam(name + ".clearing_time_threshold",
                     name + ".clearing_time_threshold", 5.0);
      getAndSetParam(name + ".quick_clearing_bit_threshold",
                     name + ".quick_clearing_bit_threshold", 10);
      getAndSetParam(name + ".quick_clearing_time_threshold",
                     name + ".quick_clearing_time_threshold", 5.0);

      getAndSetParam(name + ".origin_z", name + ".origin_z", 0.0);
      getAndSetParam(name + ".z_voxels", name + ".z_voxels", 10);
      getAndSetParam(name + ".z_resolution", name + ".z_resolution", 0.1);

      getAndSetParam(name + ".unknown_threshold", name + ".unknown_threshold",
                     10);
      getAndSetParam(name + ".mark_threshold", name + ".mark_threshold", 0);

      getAndSetParam(name + ".marking_cloud_sources",
                     name + ".marking_cloud_sources", std::string(""));
      getAndSetParam(name + ".clearing_cloud_sources",
                     name + ".clearing_cloud_sources", std::string(""));
    } else if (layer_type == "NegativeObstaclesLayer") {
      layer_param_.addLayer(CVTE_BABOT::NegativeObstaclesLayerType, name);
      getAndSetParam(name + ".marking_cloud_sources",
                     name + ".marking_cloud_sources", std::string(""));
    } else {
    }
  }
}

void CostmapParamsRos2::loadSonarLayerParams() {
  ptr_costmap_params_->setParam("sonar_layer.enabled", true);
  std::string topics;
  double clear_time;
  getAndSetParam("sonar_layer.topics", "sonar_layer.topics", topics);
  getAndSetParam("sonar_layer.clear_time", "sonar_layer.clear_time",
                 clear_time);

  layer_param_.addLayer(CVTE_BABOT::RangeSensorLayer2Type, "sonar_layer");

  LOG(INFO) << "sonarLayer config "
            << "tpoic: " << topics << ",  clear_time: " << clear_time;
}

void CostmapParamsRos2::loadRangeSensorLayer2Params() {
  ptr_costmap_params_->setParam("range_layer.enabled", true);
  std::string topics;
  double clear_time;
  getAndSetParam("range_layer.topics", "range_layer.topics", topics);
  getAndSetParam("range_layer.clear_time", "range_layer.clear_time", 10.0);

  layer_param_.addLayer(CVTE_BABOT::RangeSensorLayer2Type, "range_layer");

  LOG(INFO) << "rangeLayer config "
            << "tpoic: " << topics << ",  clear_time: " << clear_time;
}

void CostmapParamsRos2::loadCollisionLayerParams() {
  ptr_costmap_params_->setParam("collision_layer.enabled", true);

  std::string topics;
  double clear_time;
  getAndSetParam("collision_layer.topics", "collision_layer.topics", topics);
  getAndSetParam("collision_layer.keep_time", "collision_layer.keep_time",
                 clear_time);

  layer_param_.addLayer(CVTE_BABOT::CollisionLayerType, "collision_layer");

  LOG(INFO) << "collisionLayer config "
            << "tpoic: " << topics << ",  clear_time: " << clear_time;
}

void CostmapParamsRos2::loadInflationLayerParams() {
  ptr_costmap_params_->setParam("inflation_layer.enabled", true);

  getAndSetParam("inflation_layer.inflation_radius",
                 "inflation_layer.inflation_radius", 0.6);
  getAndSetParam("inflation_layer.cost_scaling_factor",
                 "inflation_layer.cost_scaling_factor", 5.0);

  layer_param_.addLayer(CVTE_BABOT::InflationLayerType, "inflation_layer");
}

void CostmapParamsRos2::loadPointCloudParams() {
  LOG(INFO) << "loadPointCloudParams";
  // 要订阅的数据

  std::string topics_2d = getAndSetParam(
      "data_manager.topics_2d", "data_manager.topics_2d", std::string(""));
  std::string topics_3d = getAndSetParam(
      "data_manager.topics_3d", "data_manager.topics_3d", std::string(""));

  std::stringstream ss_2d(topics_2d);
  std::stringstream ss_3d(topics_3d);
  std::string source;
  while (ss_2d >> source || ss_3d >> source) {
    std::string frame_id;

    frame_id =
        getAndSetParam(source + ".frame_id", source + ".frame_id", frame_id);

    //判断是否存参数服务器读取
    std::vector<double> sensor_tf(6, 0.0);
    bool read_from_params_system = false;
    bool flag = false;
    node_->get_parameter_or(frame_id + ".read_tf_from_params_system",
                            read_from_params_system, read_from_params_system);
    std::vector<double> sensor_trans(16, 0.0);
    if (read_from_params_system) {
      LOG(INFO) << "read from params system";
      std::string trans_name;
      node_->get_parameter_or(frame_id + ".tf_name", trans_name, trans_name);
      PublicParameters public_parameters;
      public_parameters.getParameter(trans_name, sensor_trans, sensor_trans);
    }
    if (flag) {
      double sy = std::sqrt(sensor_trans[0] * sensor_trans[0] +
                            sensor_trans[4] * sensor_trans[4]);

      sensor_tf[0] = sensor_trans[3];
      sensor_tf[1] = sensor_trans[7];
      sensor_tf[2] = sensor_trans[11];

      if (sy < 1e-6) {
        sensor_tf[3] = std::atan2(-sensor_trans[6], sensor_trans[5]);
        sensor_tf[4] = std::atan2(-sensor_trans[8], sy);
        sensor_tf[5] = 0;
      } else {
        sensor_tf[3] = std::atan2(sensor_trans[9], sensor_trans[10]);
        sensor_tf[4] = std::atan2(-sensor_trans[8], sy);
        sensor_tf[5] = std::atan2(sensor_trans[4], sensor_trans[0]);
      }
      LOG(INFO) << "read trans of " << frame_id << ": " << sensor_trans[0]
                << " " << sensor_trans[1] << " " << sensor_trans[2] << " "
                << sensor_trans[3] << " " << sensor_trans[4] << " "
                << sensor_trans[5] << " " << sensor_trans[6] << " "
                << sensor_trans[7] << " " << sensor_trans[8] << " "
                << sensor_trans[9] << " " << sensor_trans[10] << " "
                << sensor_trans[11] << " " << sensor_trans[12] << " "
                << sensor_trans[13] << " " << sensor_trans[14] << " "
                << sensor_trans[15];
      LOG(INFO) << "as euler: " << sensor_tf[0] << " " << sensor_tf[1] << " "
                << sensor_tf[2] << " " << sensor_tf[3] << " " << sensor_tf[4]
                << " " << sensor_tf[5];
      ptr_costmap_params_->setParam(frame_id + ".sensor_tf", sensor_tf);
    } else {
      sensor_tf = getAndSetParam(frame_id + ".sensor_tf",
                                 frame_id + ".sensor_tf", sensor_tf);
    }

    LOG(INFO) << source << "'s frame_id: " << frame_id
              << ", sensor_tf: " << sensor_tf.at(0) << " " << sensor_tf.at(1)
              << " " << sensor_tf.at(2) << " " << sensor_tf.at(3) << " "
              << sensor_tf.at(4) << " " << sensor_tf.at(5);

    getAndSetParam(frame_id + ".sensor_height", frame_id + ".sensor_height",
                   0.79);

    getAndSetParam(frame_id + ".need_transform_to_map",
                   frame_id + ".need_transform_to_map", false);
    getAndSetParam(frame_id + ".v_fov", frame_id + ".v_fov", 0.0);
    getAndSetParam(frame_id + ".h_fov", frame_id + ".h_fov", 0.0);
    getAndSetParam(frame_id + ".min_d", frame_id + ".min_d", 0.0);
    getAndSetParam(frame_id + ".max_d", frame_id + ".max_d", 0.0);
    getAndSetParam(frame_id + ".negative_min_d", frame_id + ".negative_min_d",
                   0.0);

    getAndSetParam(frame_id + ".negative_max_d", frame_id + ".negative_max_d",
                   0.0);

    getAndSetParam(frame_id + ".min_h", frame_id + ".min_h", 0.0);
    getAndSetParam(frame_id + ".max_h", frame_id + ".max_h", 0.0);
  }
}

void CostmapParamsRos2::loadRangeParams() {
  LOG(INFO) << "loadRangeParams";
  std::string range_topics =
      getAndSetParam("data_manager.range_topics", "data_manager.range_topics",
                     std::string("sonarfl"));
  std::stringstream topic_list(range_topics);
  std::string topic_temp;
  std::vector<double> default_tf(3, 0.0);
  double min_d_default = 0.0;
  double max_d_default = 0.0;
  double clear_range_default = 0.0;
  std::vector<double> v_clear_fov_value_default;
  bool enable_flag = false;
  while (topic_list >> topic_temp) {
    std::string frame_id_list;
    frame_id_list = getAndSetParam(topic_temp + ".frame_id",
                                   topic_temp + ".frame_id", frame_id_list);
    std::stringstream id_list(frame_id_list);
    std::string id_temp;
    while (id_list >> id_temp) {
      std::vector<double> tf_temp =
          getAndSetParam(topic_temp + "." + id_temp + ".sensor_tf",
                         topic_temp + "." + id_temp + ".sensor_tf", default_tf);
      double min_d =
          getAndSetParam(topic_temp + "." + id_temp + ".min_d",
                         topic_temp + "." + id_temp + ".min_d", min_d_default);
      double max_d =
          getAndSetParam(topic_temp + "." + id_temp + ".max_d",
                         topic_temp + "." + id_temp + ".max_d", max_d_default);
      bool enable =
          getAndSetParam(topic_temp + "." + id_temp + ".enable",
                         topic_temp + "." + id_temp + ".enable", enable_flag);
      int rves_flg =
          getAndSetParam(topic_temp + "." + id_temp + ".rves_flg",
                         topic_temp + "." + id_temp + ".rves_flg", 0);

      double clear_range = getAndSetParam(
          topic_temp + "." + id_temp + ".clear_range",
          topic_temp + "." + id_temp + ".clear_range", clear_range_default);

      bool filter_enable =
          getAndSetParam(topic_temp + "." + id_temp + ".filter_enable",
                         topic_temp + "." + id_temp + ".filter_enable", false);

      // clearFov参数读取
      std::vector<double> v_clear_fov_value;
      node_->get_parameter_or(topic_temp + "." + id_temp + ".clear_fov",
                              v_clear_fov_value, v_clear_fov_value_default);

      if (v_clear_fov_value.empty() || v_clear_fov_value.size() % 2 == 1) {
        LOG(ERROR) << topic_temp + "." + id_temp << " clear_fov error";
      } else {
        // 计算端点向量
        std::vector<Eigen::Vector2d> v_endpoint_;
        std::vector<Eigen::Vector2d> v_product;
        std::vector<Eigen::Vector2d> v_origin;
        LOG(INFO) << topic_temp + "." + id_temp << " size "
                  << v_clear_fov_value.size();
        for (int i = 0; i < v_clear_fov_value.size() - 1; i += 2) {
          v_endpoint_.emplace_back(v_clear_fov_value[i],
                                   v_clear_fov_value[i + 1]);
          LOG(INFO) << "x " << v_endpoint_.back()(0) << " y "
                    << v_endpoint_.back()(1);
        }

        // 计算法向量
        Eigen::Matrix2d R;
        R << 0.0, 1.0, -1.0, 0.0;  //{cos,-sin,sin,cos} -90°旋转矩阵
        for (int i = 0; i < v_endpoint_.size() - 1; i++) {
          // 法向量
          Eigen::Vector2d origin = (v_endpoint_[i + 1] + v_endpoint_[i]) / 2;
          Eigen::Vector2d vector =
              R * (v_endpoint_[i + 1] - v_endpoint_[i]) + origin;

          v_product.emplace_back(vector);
          v_origin.emplace_back(origin);
          LOG(INFO) << "local vector x " << v_product.back()(0) << " y "
                    << v_product.back()(1);
          LOG(INFO) << "local origin x " << v_origin.back()(0) << " y "
                    << v_origin.back()(1);
        }
        // 构建最后一个法向量
        Eigen::Vector2d origin = (v_endpoint_.back() + v_endpoint_[0]) / 2;
        Eigen::Vector2d vector =
            R * (v_endpoint_[0] - v_endpoint_.back()) + origin;

        v_product.emplace_back(vector);
        v_origin.emplace_back(origin);
        LOG(INFO) << "local vector x " << v_product.back()(0) << " y "
                  << v_product.back()(1);
        LOG(INFO) << "local origin x " << v_origin.back()(0) << " y "
                  << v_origin.back()(1);

        Eigen::Matrix3d T;
        T << cos(tf_temp[2]), -sin(tf_temp[2]), tf_temp[0],  //
            sin(tf_temp[2]), cos(tf_temp[2]), tf_temp[1],    //
            0, 0, 1;
        LOG(INFO) << topic_temp + "." + id_temp << " T " << std::endl << T;
        // 变换至baselink
        for (int i = 0; i < v_product.size(); ++i) {
          Eigen::Vector3d product(v_product[i](0), v_product[i](1), 1);
          Eigen::Vector3d origin(v_origin[i](0), v_origin[i](1), 1);
          product = T * product;
          origin = T * origin;

          v_product[i] = Eigen::Vector2d(product(0), product(1));
          v_origin[i] = Eigen::Vector2d(origin(0), origin(1));
          LOG(INFO) << topic_temp + "." + id_temp + ".clear_product "
                    << v_product[i](0) - v_origin[i](0) << " "
                    << v_product[i](1) - v_origin[i](1);
          LOG(INFO) << topic_temp + "." + id_temp + ".clear_origin "
                    << v_origin[i](0) << " " << v_origin[i](1);
        }

        // 设置进参数中间缓存位置
        ptr_costmap_params_->setParam(
            topic_temp + "." + id_temp + ".clear_product", v_product);
        ptr_costmap_params_->setParam(
            topic_temp + "." + id_temp + ".clear_origin", v_origin);
      }

      if (enable) {
        CostmapMediator::getPtrInstance()->setSensorEnable(topic_temp + "." +
                                                           id_temp);
      }

      LOG(INFO) << "topic name : " << topic_temp << ", "
                << "fram_id: " << id_temp << ", "
                << "tf:[  " << tf_temp[0] << ", " << tf_temp[1] << "], "
                << "min_d " << min_d << ", "
                << "max_d " << max_d << ", "
                << "clear_range " << clear_range << ", "
                << "enable " << enable << ", "
                << "filter_enable " << filter_enable << ", "
                << "rves_flg " << rves_flg;
    }
  }
}

void CostmapParamsRos2::loadDynamicObsParams() {
  LOG(INFO) << "loadDynamicObsParams";

  // ros的topic名
  std::string dynamic_obs_topic = getAndSetParam(
      "data_manager.dynamic_obs_topic", "data_manager.dynamic_obs_topic",
      std::string("tracking/tracking_objects"));

  std::string frame_id;
  getAndSetParam(dynamic_obs_topic + ".frame_id",
                 dynamic_obs_topic + ".frame_id", frame_id);
  getAndSetParam(frame_id + ".sensor_height", frame_id + ".sensor_height",
                 0.72);
}

void CostmapParamsRos2::loadSwitchParams() {
  LOG(INFO) << "loadSwitchParams";
  std::string switch_topics =
      getAndSetParam("data_manager.switch_topic", "data_manager.switch_topic",
                     std::string("collision_switch"));
  std::stringstream topic_list(switch_topics);
  std::string topic_temp;
  std::vector<double> default_tf(2, 0.0);
  double min_d_default = 0.0;
  double max_d_default = 0.0;
  bool enable_flag = false;
  while (topic_list >> topic_temp) {
    std::string frame_id_list;
    frame_id_list = getAndSetParam(topic_temp + ".frame_id",
                                   topic_temp + ".frame_id", frame_id_list);
    std::stringstream id_list(frame_id_list);
    std::string id_temp;
    while (id_list >> id_temp) {
      // topic.framid.sensor_tf
      std::vector<double> tf_temp =
          getAndSetParam(topic_temp + "." + id_temp + ".sensor_tf",
                         topic_temp + "." + id_temp + ".sensor_tf", default_tf);
      double min_d =
          getAndSetParam(topic_temp + "." + id_temp + ".min_d",
                         topic_temp + "." + id_temp + ".min_d", min_d_default);
      double max_d =
          getAndSetParam(topic_temp + "." + id_temp + ".max_d",
                         topic_temp + "." + id_temp + ".max_d", max_d_default);
      bool enable =
          getAndSetParam(topic_temp + "." + id_temp + ".enable",
                         topic_temp + "." + id_temp + ".enable", enable_flag);
      int rves_flg =
          getAndSetParam(topic_temp + "." + id_temp + ".rves_flg",
                         topic_temp + "." + id_temp + ".rves_flg", 0);

      if (enable) {
        CostmapMediator::getPtrInstance()->setSensorEnable(topic_temp + "." +
                                                           id_temp);
      }

      LOG(INFO) << "topic name :" << topic_temp << ", "
                << "fram_id:" << id_temp << ", "
                << "tf:[ " << tf_temp[0] << ", " << tf_temp[1] << "], "
                << "min_d " << min_d << ", "
                << "max_d " << max_d << ", "
                << "enable " << enable << ","
                << "rves_flg " << rves_flg;
    }
  }
}

void CostmapParamsRos2::loadRangeSwitchParams() {
  LOG(INFO) << "loadRangeSwitchParams";
  std::string switch_topics;
  switch_topics =
      getAndSetParam("data_manager.switch_range_topic",
                     "data_manager.switch_range_topic", switch_topics);
  std::stringstream topic_list(switch_topics);
  std::string topic_temp;
  std::vector<double> default_tf(2, 0.0);
  double min_d_default = 0.0;
  double max_d_default = 0.0;
  bool enable_flag = false;
  while (topic_list >> topic_temp) {
    std::string frame_id_list;
    frame_id_list = getAndSetParam(topic_temp + ".frame_id",
                                   topic_temp + ".frame_id", frame_id_list);
    std::stringstream id_list(frame_id_list);
    std::string id_temp;
    while (id_list >> id_temp) {
      // topic.framid.sensor_tf
      std::vector<double> tf_temp =
          getAndSetParam(topic_temp + "." + id_temp + ".sensor_tf",
                         topic_temp + "." + id_temp + ".sensor_tf", default_tf);
      double min_d =
          getAndSetParam(topic_temp + "." + id_temp + ".min_d",
                         topic_temp + "." + id_temp + ".min_d", min_d_default);
      double max_d =
          getAndSetParam(topic_temp + "." + id_temp + ".max_d",
                         topic_temp + "." + id_temp + ".max_d", max_d_default);
      bool enable =
          getAndSetParam(topic_temp + "." + id_temp + ".enable",
                         topic_temp + "." + id_temp + ".enable", enable_flag);
      int rves_flg =
          getAndSetParam(topic_temp + "." + id_temp + ".rves_flg",
                         topic_temp + "." + id_temp + ".rves_flg", 0);

      if (enable) {
        CostmapMediator::getPtrInstance()->setSensorEnable(topic_temp + "." +
                                                           id_temp);
      }

      LOG(INFO) << "topic name :" << topic_temp << ", "
                << "fram_id:" << id_temp << ", "
                << "tf:[ " << tf_temp[0] << ", " << tf_temp[1] << "], "
                << "min_d" << min_d << ", "
                << "max_d " << max_d << ", "
                << "enable " << enable << ","
                << "rves_flg " << rves_flg;
    }
  }
}

void CostmapParamsRos2::loadInfraredObstacleSwitchRangeParams(){
  LOG(INFO) << "loadInfraredObstacleSwitchRangeParams";
  std::string switch_topics;
  switch_topics = 
      getAndSetParam("data_manager.infrared_obstacle_switch_range_topic",
                     "data_manager.infrared_obstacle_switch_range_topic", switch_topics);

  std::stringstream topic_list(switch_topics);
  std::string topic_temp;
  std::vector<double> default_tf(2, 0.0);
  double min_d_default = 0.0;
  double max_d_default = 0.0;
  bool enable_flag = false;
  while(topic_list >> topic_temp)
  {
    std::string frame_id_list;
    frame_id_list = getAndSetParam(topic_temp + ".frame_id",
                                   topic_temp + ".frame_id", frame_id_list);
    std::stringstream id_list(frame_id_list);
    std::string id_temp;
    while(id_list >> id_temp)
    {
      std::vector<double> tf_temp = 
        getAndSetParam(topic_temp + "." + id_temp + ".sensor_tf",
                       topic_temp + "." + id_temp + ".sensor_tf", default_tf);

      double min_d = 
        getAndSetParam(topic_temp + "." + id_temp + ".min_d",
                       topic_temp + "." + id_temp + ".min_d", min_d_default);

      double max_d = 
        getAndSetParam(topic_temp + "." + id_temp + ".max_d",
                       topic_temp + "." + id_temp + ".max_d", max_d_default);

      bool enable = 
        getAndSetParam(topic_temp + "." + id_temp + ".enable",
                       topic_temp + "." + id_temp + ".enable", enable_flag);

      int rves_flg = 
        getAndSetParam(topic_temp + "." + id_temp + ".rves_flg",
                       topic_temp + "." + id_temp + ".rves_flg", 0);

      if(enable)
      {
        CostmapMediator::getPtrInstance()->setSensorEnable(topic_temp + "." + id_temp);
      }

      LOG(INFO) << "topic name :" << topic_temp << ", "
                << "fram_id:" << id_temp << ", "
                << "tf:[ " << tf_temp[0] << ", " << tf_temp[1] << "], "
                << "min_d" << min_d << ", "
                << "max_d " << max_d << ", "
                << "enable " << enable << ","
                << "rves_flg " << rves_flg;

    }
  }  
}

void CostmapParamsRos2::loadRangeIRParams() {
  LOG(INFO) << "loadRangeIRParams";
  std::string ir_topic;
  ir_topic = getAndSetParam("data_manager.ir_topic", "data_manager.ir_topic",
                            ir_topic);
  std::stringstream topic_list(ir_topic);

  std::string topic_temp;
  std::vector<double> default_tf(3, 0.0);
  double min_d_default = 0.0;
  double max_d_default = 0.0;
  bool enable_flag = false;
  while (topic_list >> topic_temp) {
    std::string frame_id_list;
    frame_id_list = getAndSetParam(topic_temp + ".frame_id",
                                   topic_temp + ".frame_id", frame_id_list);
    std::stringstream id_list(frame_id_list);
    std::string id_temp;
    while (id_list >> id_temp) {
      // topic.framid.sensor_tf
      std::vector<double> tf_temp =
          getAndSetParam(topic_temp + "." + id_temp + ".sensor_tf",
                         topic_temp + "." + id_temp + ".sensor_tf", default_tf);
      double min_d =
          getAndSetParam(topic_temp + "." + id_temp + ".min_d",
                         topic_temp + "." + id_temp + ".min_d", min_d_default);
      double max_d =
          getAndSetParam(topic_temp + "." + id_temp + ".max_d",
                         topic_temp + "." + id_temp + ".max_d", max_d_default);
      bool enable =
          getAndSetParam(topic_temp + "." + id_temp + ".enable",
                         topic_temp + "." + id_temp + ".enable", enable_flag);
      int rves_flg =
          getAndSetParam(topic_temp + "." + id_temp + ".rves_flg",
                         topic_temp + "." + id_temp + ".rves_flg", 0);

      if (enable) {
        CostmapMediator::getPtrInstance()->setSensorEnable(topic_temp + "." +
                                                           id_temp);
      }

      LOG(INFO) << "topic name :" << topic_temp << ", "
                << "fram_id:" << id_temp << ", "
                << "tf:[ " << tf_temp[0] << ", " << tf_temp[1] << "], "
                << "min_d " << min_d << ", "
                << "max_d " << max_d << ", "
                << "enable " << enable << ","
                << "rves_flg " << rves_flg;
    }
  }
}

}  // namespace CVTE_BABOT