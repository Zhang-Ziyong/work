/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath:
 /src/cvte_lidar_slam/slam_core/frontend/feature_extract_factory.hpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-07-15 10:45:16
 ************************************************************************/
#ifndef FEATURE_EXTRACT_FACTORY_HPP
#define FEATURE_EXTRACT_FACTORY_HPP

#include "frontend/feature_extractor.hpp"
#include <memory>

namespace cvte_lidar_slam {
class FeatureExtractorFactory {
 public:
  static std::shared_ptr<BaseFeatureExtractor> creatExtractor(
      const FeatureConfig &config) {
    if (nullptr == ptr_instance_) {
      if (std::string("horizon") == config.lidar_type) {
      } else {
        ptr_instance_ = std::make_shared<FeatureExtractor>(config);
        // ptr_instance_.reset(new FeatureExtractor);
      }
    }
    return ptr_instance_;
  }

 private:
  static std::shared_ptr<BaseFeatureExtractor> ptr_instance_;
  FeatureExtractorFactory() = default;
};  // end of class

std::shared_ptr<BaseFeatureExtractor> FeatureExtractorFactory::ptr_instance_ =
    nullptr;

}  // namespace cvte_lidar_slam

#endif