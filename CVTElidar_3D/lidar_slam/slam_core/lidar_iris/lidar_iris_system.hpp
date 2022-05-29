/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file lidar_iris_system.hpp
 *
 *@brief
 * 激光虹膜特征的构建和匹配系统接口
 *
 *@modified by caoyong(caoyong@cvte.com)
 *
 *@author caoyong(caoyong@cvte.com)
 *@version V1.0
 *@data 2020-11-03
 ************************************************************************/

#ifndef LIADR_IRIS_SYSTEM_HPP
#define LIADR_IRIS_SYSTEM_HPP

#include <glog/logging.h>

#include <map>
#include <memory>
#include <mutex>

#include "common/config/system_config.hpp"
#include "common/data_struct/pc_base.hpp"
#include "common/math_base/slam_math.hpp"
#include "lidar_iris.hpp"

namespace cvte_lidar_slam {

struct IrisFeatureData {
  LidarIris::FeatureDesc feature_desc;  // feature data
  size_t frame_id;                      // 最近关键帧的位置
};
class LidarIrisSystem {
 public:
  ~LidarIrisSystem() {}
  static std::shared_ptr<LidarIrisSystem> getInstance();

  void insertFeatureCloud(const laserCloud::Ptr ori_cloud,
                          const size_t frame_id);

  bool saveFeatures(const std::string &map_dir);

  bool loadFeature(const std::string &map_dir);

  bool findNearestFrame(const laserCloud::Ptr ori_cloud,
                        std::pair<size_t, float> &compare_result,
                        int &angle_bias);

  bool findNearestFramebyFrameids(const laserCloud::Ptr ori_cloud,
                                  const std::vector<size_t> &v_frame_id,
                                  std::pair<size_t, float> &compare_result,
                                  int &angle_bias);

  void Reset() {
    feature_mutex_.lock();
    v_feature_data_.clear();
    std::vector<IrisFeatureData>().swap(v_feature_data_);
    v_feature_data_.reserve(200);
    um_feature_data_.clear();
    std::unordered_map<size_t, IrisFeatureData>().swap(um_feature_data_);
    feature_mutex_.unlock();
    std::cout << "IrisFeatureData Reset" << std::endl;
  }

 private:
  std::shared_ptr<LidarIris> ptr_lidar_iris_ = nullptr;
  std::vector<IrisFeatureData> v_feature_data_;
  std::unordered_map<size_t, IrisFeatureData> um_feature_data_;

 private:
  LidarIrisSystem();
  static std::shared_ptr<LidarIrisSystem> ptr_lidar_iris_system_;
  std::mutex feature_mutex_;
};
}  // namespace cvte_lidar_slam

#endif
