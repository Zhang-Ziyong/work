/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file layer.cpp
 *
 *@brief layer基类.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-08
 ************************************************************************/
#include "layer.hpp"
#include <glog/logging.h>
#include <iostream>
namespace CVTE_BABOT {

Layer::Layer() : b_current_(false), b_enabled_(false), s_name_() {}

bool Layer::initialize(const std::string &name) {
  s_name_ = name;

  origin_stop_bound_.push_back({-0.8, -1.0});
  origin_stop_bound_.push_back({1.6, 1.0});
  stop_bound_ = origin_stop_bound_;

  origin_slow_bound_.push_back({-1.0, -1.3});
  origin_slow_bound_.push_back({1.8, 1.3});
  slow_bound_ = origin_slow_bound_;

  extra_area_after_stop_size_x_ = 0.7;

  robot_type_ = CENTER_DRIVE;

  ptr_field_.reset();
  ptr_field_ = boost::shared_array<bool>(new bool[66]);
  memset(ptr_field_.get(), false, 66 * sizeof(bool));
  return onInitialize();
}

// const std::vector<CostmapPoint> &Layer::getFootprint() const
// {
//   return ptr_costmap_mediator_->getFootprint();
// }

}  // namespace CVTE_BABOT
