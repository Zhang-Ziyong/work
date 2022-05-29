/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file processor_utils.hpp
 *
 *@brief processor工具类
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2020-02-25
 ************************************************************************/
#ifndef __PROCESSOR_UTILS_HPP
#define __PROCESSOR_UTILS_HPP
#include <memory>
#include <vector>
#include "costmap_utils.hpp"

namespace CVTE_BABOT {
enum ObstacleType { Pedestrian = 1, Car = 2 };

struct obstacleSize {
  double d_length = 0.00;  // x
  double d_width = 0.00;   // y
  double d_height = 0.00;  // z
};

typedef CostmapPointCloud ProcessorCloud;

typedef CostmapPointXYZ ProcessorPoint;

struct DynamicObject {
  // init the ptr_v_cloud_
  DynamicObject() : id(0), ptr_v_cloud_(std::make_shared<ProcessorCloud>()) {}

  unsigned int id;
  ObstacleType obstacle_type;
  std::shared_ptr<ProcessorCloud> ptr_v_cloud_;
  obstacleSize size;
  ProcessorPoint position;
  ProcessorPoint direction;
  ProcessorPoint velocity;
};

struct DynamicObstacles {
  std::vector<DynamicObject> v_dynamic_object_;
  CostmapPointXYZ sensor_origin_;
};

struct DetectionBox {
  DetectionBox(const u_int16_t &left_up_x, const u_int16_t &left_up_y,
               const u_int16_t &right_down_x, const u_int16_t &right_down_y) {
    box[0] = left_up_x;
    box[1] = left_up_y;
    box[2] = right_down_x;
    box[3] = right_down_y;
  }

  std::array<u_int16_t, 4> box;
};

enum CameraType { InvalidCamera = 0, FrontCamera };

struct CameraResult {
  enum CameraType id = InvalidCamera;
  std::vector<DetectionBox> boxes;
};

struct CarDetectionResult {
  bool push_back(const CameraResult &camera_result) {
    if (camera_result.id == InvalidCamera) {
      return false;
    }

    for (const auto &result : v_camera_result) {
      if (camera_result.id == result.id) {
        return false;
      }
    }

    v_camera_result.push_back(camera_result);
    return true;
  }

  double d_time = -1e6;
  std::vector<CameraResult> v_camera_result;
};

struct SensorSwitchObs {
  bool status;        // 传感器开关状态
  WorldmapPose pose;  // 机器人位姿
  time_t time;
  std::string s_topic_name_;
  std::string s_fram_id_;

  // 开关的三个特征点
  // std::vector<double> lp{2, 0.0};
  // std::vector<double> mp{2, 0.0};
  // std::vector<double> rp{2, 0.0};

  // std::vector<double> touched_tf{2,0.0};
  std::vector<double> touched_tfs;
};

}  // namespace CVTE_BABOT
#endif
