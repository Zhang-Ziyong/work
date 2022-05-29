/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file world_map_data.hpp
 *
 *@brief world map data storage class.
 *
 *@author chenmingjian(chenmingian@cvte.com)
 *
 *@modified chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-14
 ************************************************************************/

#ifndef __WORLD_MAP_DATA_HPP
#define __WORLD_MAP_DATA_HPP
#include "costmap_utils.hpp"
#include <memory>
#include <string>
namespace CVTE_BABOT {

/**
 * WorldmapData
 * @brief
 * 全局地图数据，以及地图信息
 **/
struct WorldmapData {
  WorldmapData()
      : ui_width_(0), ui_height_(0), d_resolution_(0.0),
        ptr_data_(std::make_shared<std::vector<unsigned char>>()) {
    origin_.d_x = 0.0;
    origin_.d_y = 0.0;
  }

  WorldmapData(const WorldmapPoint &origin, const unsigned int &ui_width,
               const unsigned int &ui_height, double d_resolution,
               const std::vector<unsigned char> &v_uc)
      : origin_(origin), ui_width_(ui_width), ui_height_(ui_height),
        d_resolution_(d_resolution),
        ptr_data_(std::make_shared<std::vector<unsigned char>>(v_uc)) {}

  WorldmapData(const WorldmapData &obs)
      : origin_(obs.origin_), ui_width_(obs.ui_width_),
        ui_height_(obs.ui_height_), d_resolution_(obs.d_resolution_),
        ptr_data_(
            std::make_shared<std::vector<unsigned char>>(*obs.ptr_data_)) {}

  WorldmapData &operator=(const WorldmapData &obs) {
    origin_ = obs.origin_;
    d_resolution_ = obs.d_resolution_;
    ui_width_ = obs.ui_width_;
    ui_height_ = obs.ui_height_;

    ptr_data_ = std::make_shared<std::vector<unsigned char>>(*(obs.ptr_data_));
    return *this;
  }

  ~WorldmapData() = default;
  bool readMapFromYaml(const std::string &map_path);

  WorldmapPoint origin_;   ///<地图原点
  unsigned int ui_width_;  ///<地图宽
  unsigned int ui_height_; ///<地图高
  double d_resolution_;    ///<地图分辨率

  std::shared_ptr<std::vector<unsigned char>> ptr_data_; ///<地图数据
};

} // namespace CVTE_BABOT
#endif // __COSTMAP_RANGE_DATA_HPP
