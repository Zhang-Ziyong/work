/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file data_register.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-01
 ************************************************************************/
#include "data_register.hpp"
#include "costmap_cloud.hpp"
#include "costmap_mediator.hpp"
#include "costmap_range_data.hpp"
#include "processor_utils.hpp"

namespace CVTE_BABOT {

template <typename data_type>
void initFuntion(const std::string &data_maneger,
                 const data_type &register_type) {
  std::string topics_;
  CostmapMediator::getPtrInstance()->getParam(
      std::string("data_manager." + data_maneger), topics_, std::string(""));
  LOG(INFO) << "############register: " << topics_;
  if (topics_.size() == 0) {
    LOG(ERROR) << "data_manager." << data_maneger << " not define";
  } else {
    std::stringstream ss(topics_);
    std::string topic_temp_;
    while (ss >> topic_temp_) {
      auto ptr_data = std::make_shared<data_type>();
      std::string ids_;
      CostmapMediator::getPtrInstance()->getParam(
          std::string(topic_temp_ + ".frame_id"), ids_, std::string(""));
      std::stringstream id_ss_(ids_);
      std::string id_temp_;
      LOG(INFO) << "############frame_ids : " << ids_;
      while (id_ss_ >> id_temp_) {
        LOG(INFO) << "--------register: " << topics_ << "." << id_temp_;
        CostmapMediator::getPtrInstance()
            ->registerUpdateFunc<std::shared_ptr<data_type>>(
                topic_temp_ + "." + id_temp_, ptr_data);
      }
    }
  }
}

void DataRegister::createSubscribers() {
  //超声波
  CostmapRangeData sonar_type;
  initFuntion("range_topics", sonar_type);

  // 碰撞开关
  SensorSwitchObs SensorSwitchObs_type;
  initFuntion("switch_topic", SensorSwitchObs_type);

  //防跌落红外
  SensorSwitchObs SensorSwitchObs_rang_type;
  initFuntion("switch_range_topic", SensorSwitchObs_rang_type);

  //订阅避障红外
  SensorSwitchObs SensorSwitchObs_range_type;
  initFuntion("infrared_obstacle_switch_range_topic",SensorSwitchObs_range_type);
  

  //红外系列
  CostmapRangeData CostmapRangeData_type;
  initFuntion("ir_topic", CostmapRangeData_type);

  // 激光数据
  std::string topics_2d;
  CostmapMediator::getPtrInstance()->getParam(
      std::string("data_manager.topics_2d"), topics_2d, std::string("scan"));
  std::string topics_3d;
  CostmapMediator::getPtrInstance()->getParam("data_manager.topics_3d",
                                              topics_3d, std::string(""));
  std::stringstream ss_2d(topics_2d);
  std::stringstream ss_3d(topics_3d);
  std::string point_cloud_name;
  while (ss_2d >> point_cloud_name || ss_3d >> point_cloud_name) {
    LOG(INFO) << "subscribe point cloud topic: " << point_cloud_name;

    auto ptr_origin_cloud = std::make_shared<CostmapCloud>();
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>(point_cloud_name,
                                                            ptr_origin_cloud);

    std::string frame_id;
    CostmapMediator::getPtrInstance()->getParam(point_cloud_name + ".frame_id",
                                                frame_id, std::string("laser"));

    bool b_need_transform_to_map = true;
    CostmapMediator::getPtrInstance()->getParam(
        frame_id + ".need_transform_to_map", b_need_transform_to_map, true);

    if (b_need_transform_to_map) {
      auto ptr_cloud = std::make_shared<CostmapCloud>();
      CostmapMediator::getPtrInstance()
          ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>(
              point_cloud_name + "_map", ptr_cloud);
    }
  }

  // 动态障碍
  std::string dynamic_obs_topic;
  CostmapMediator::getPtrInstance()->getParam(
      std::string("data_manager.dynamic_obs_topic"), dynamic_obs_topic,
      std::string("dynamic_obstacles"));

  // 切记要初始化指针
  std::shared_ptr<DynamicObstacles> ptr_dynamic_obstacles =
      std::make_shared<DynamicObstacles>();
  CostmapMediator::getPtrInstance()
      ->registerUpdateFunc<std::shared_ptr<DynamicObstacles>>(
          dynamic_obs_topic + "_obj", ptr_dynamic_obstacles);

  auto ptr_dynamic_cloud = std::make_shared<CostmapCloud>();
  CostmapMediator::getPtrInstance()
      ->registerUpdateFunc<std::shared_ptr<CostmapCloud>>(dynamic_obs_topic,
                                                          ptr_dynamic_cloud);
}

}  // namespace CVTE_BABOT
