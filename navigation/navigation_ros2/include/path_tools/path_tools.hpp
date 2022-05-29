/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file navigation_arch_adapter_ros2.hpp
 *
 *@brief
 * 路径相关的一些工具函数
 *
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author liangjiajun(liangjiajun@cvte.com)
 *@version current_algo.dev
 *@data 2019-04-24
 ************************************************************************/

#ifndef PATH_TOOLS_H_
#define PATH_TOOLS_H_

#include <glog/logging.h>
#include <fstream>
#include <string>
#include "nav_msgs/msg/path.hpp"
#include "pose2d/pose2d.hpp"
#include "yaml-cpp/yaml.h"

namespace CVTE_BABOT {

template <typename T>
void operator>>(const YAML::Node &node, T &i) {
  i = node.as<T>();
}

class PathTools {
 public:
  static bool path_transform(const std::string filepath,
                             nav_msgs::msg::Path &path_msg) {
    std::ifstream fin(filepath.c_str());
    if (fin.fail()) {
      RCUTILS_LOG_ERROR("Path_transform could not open %s.", filepath.c_str());
      exit(-1);
    }

    YAML::Node doc = YAML::LoadFile(filepath);

    const YAML::Node &point_node = doc["point"];

    if (0 == point_node.size()) {
      return false;
    }

    path_msg.header.frame_id = "map";
    bool with_yaw = point_node[0].size() == 3 ? true : false;
    double yaw = 0;
    AngleCalculate::Quaternion q;

    for (unsigned j = 0; j < point_node.size(); ++j) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.z = 0.0;
      point_node[j]["x"] >> pose.pose.position.x;
      point_node[j]["y"] >> pose.pose.position.y;
      if (with_yaw) {
        point_node[j]["yaw"] >> yaw;
        q = AngleCalculate::yawToQuaternion(0, 0, yaw);
        pose.pose.orientation.x = q.x;
        pose.pose.orientation.y = q.y;
        pose.pose.orientation.z = q.z;
        pose.pose.orientation.w = q.w;
      } else {
        point_node[j]["qx"] >> pose.pose.orientation.x;
        point_node[j]["qy"] >> pose.pose.orientation.y;
        point_node[j]["qz"] >> pose.pose.orientation.z;
        point_node[j]["qw"] >> pose.pose.orientation.w;
      }
      path_msg.poses.push_back(pose);
    }
    return true;
  }

  static bool yaw_path_transform(const std::string filepath,
                                 nav_msgs::msg::Path &path_msg) {
    std::ifstream fin(filepath.c_str());
    if (fin.fail()) {
      RCUTILS_LOG_ERROR("Path_transform could not open %s.", filepath.c_str());
      exit(-1);
    }

    YAML::Node doc = YAML::LoadFile(filepath);

    const YAML::Node &point_node = doc["point"];

    if (0 == point_node.size()) {
      return false;
    }

    path_msg.header.frame_id = "map";
    bool with_yaw = point_node[0].size() == 3 ? true : false;
    double yaw = 0;
    AngleCalculate::Quaternion q;

    for (unsigned j = 0; j < point_node.size(); ++j) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.z = 0.0;
      point_node[j]["x"] >> pose.pose.position.x;
      point_node[j]["y"] >> pose.pose.position.y;

      if (with_yaw) {
        point_node[j]["yaw"] >> yaw;
        q = AngleCalculate::yawToQuaternion(0, 0, yaw);
        pose.pose.orientation.x = q.x;
        pose.pose.orientation.y = q.y;
        pose.pose.orientation.z = q.z;
        pose.pose.orientation.w = q.w;
      } else {
        point_node[j]["qx"] >> pose.pose.orientation.x;
        point_node[j]["qy"] >> pose.pose.orientation.y;
        point_node[j]["qz"] >> pose.pose.orientation.z;
        point_node[j]["qw"] >> pose.pose.orientation.w;
      }
      path_msg.poses.push_back(pose);
    }
    return true;
  }

  static bool getPathFromFile(const std::string filepath,
                              SubPath &reference_path, int path_type = 1) {
    std::ifstream fin(filepath.c_str());
    if (fin.fail()) {
      LOG(ERROR) << "load path file error: file is not exist";
      return false;
    }
    LOG(ERROR) << "Read Path File: " << filepath.c_str();
    YAML::Node doc = YAML::LoadFile(filepath);
    const YAML::Node &point_node = doc["point"];

    auto path_size = point_node.size();

    if (0 == path_size) {
      LOG(ERROR) << "path file error: path size eroor";
      return false;
    }

    reference_path.wps.resize(path_size);
    AngleCalculate::Quaternion q;
    // bool with_yaw = point_node[0].size() == 3 ? true : false;
    bool using_quaternion = false;
    bool has_curve = false;
    bool has_v = false;
    bool has_w = false;
    for (YAML::const_iterator it = point_node[0].begin();
         it != point_node[0].end(); ++it) {
      std::string key = it->first.as<std::string>();
      LOG(INFO) << "path data key: " << key;
      if ("qx" == key) {
        using_quaternion = true;
      }
      if ("yaw" == key) {
        using_quaternion = false;
      }
      if ("curve" == key) {
        has_curve = true;
      }
      if ("v" == key) {
        has_v = true;
      }
      if ("w" == key) {
        has_w = true;
      }
    }
    // if (has_curve || has_v || has_w) {
    reference_path.wpis.resize(path_size);
    // }
    for (unsigned j = 0; j < point_node.size(); ++j) {
      point_node[j]["x"] >> reference_path.wps[j].x;
      point_node[j]["y"] >> reference_path.wps[j].y;
      if (!path_type) {
        if (point_node[j]["type"]) {
          point_node[j]["type"] >> reference_path.wpis[j].is_edge_wise;
        } else {
          reference_path.wpis[j].is_edge_wise = 0;
        }
      } else {
        reference_path.wpis[j].is_edge_wise = 0;
      }
      if (using_quaternion) {
        point_node[j]["qx"] >> q.x;
        point_node[j]["qy"] >> q.y;
        point_node[j]["qz"] >> q.z;
        point_node[j]["qw"] >> q.w;

        reference_path.wps[j].yaw = AngleCalculate::quaternionToYaw(q);
      } else {
        point_node[j]["yaw"] >> reference_path.wps[j].yaw;
      }
      if (has_curve) {
        point_node[j]["curve"] >> reference_path.wpis[j].curve;
        LOG(INFO) << "curve " << reference_path.wpis[j].curve;
      }
      if (has_v) {
        point_node[j]["v"] >> reference_path.wpis[j].v;
      }
      if (has_w) {
        point_node[j]["w"] >> reference_path.wpis[j].w;
      }
    }

    return true;
  }

  static std::string getMapNameFromFile(const std::string filepath) {
    std::string map_name;
    std::ifstream fin(filepath.c_str());
    if (fin.fail()) {
      LOG(ERROR) << "load path file error: file is not exist";
      return map_name;
    }
    LOG(INFO) << "Read Path File: " << filepath.c_str();
    YAML::Node doc = YAML::LoadFile(filepath);
    doc["map_name"] >> map_name;
    LOG(INFO) << "Map name: " << map_name;
    return map_name;
  }
};

}  // namespace CVTE_BABOT

#endif