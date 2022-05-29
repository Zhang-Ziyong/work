/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file footprint_utils.hpp
 *
 *@brief footprint_utils.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-25
 ************************************************************************/
#ifndef __FOOTPRINT_UTILS_HPP
#define __FOOTPRINT_UTILS_HPP

#include <math.h>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "world_map_data.hpp"

namespace CVTE_BABOT {

/**
 *sign0
 *@brief
 *一个阶跃函数.
 *
 * @param[in] d_x - 阶跃输入数据
 * return 阶跃输出数据
 * */
double sign0(const double &d_x);

/**
 *padFootprint
 *@brief
 *扩充机器人轮廓.
 *
 * @param[in&out] v_footprint - 机器人多边形
 * @param[in] d_padding - 扩充系数
 * return 阶跃输出数据
 * */
void padFootprint(const double &d_padding,
                  std::vector<WorldmapPoint> &v_footprint);

/**
 *parseVVF
 *@brief
 *从字符串中解析点集.
 *
 * @param[in] input - string类型多边形点
 * @param[out] error_return - 出错信息
 * @return result-点集
 * */
std::vector<std::vector<double>> parseVVF(const std::string &input,
                                          std::string &error_return);

/**
 *makeFootprintFromString
 *@brief
 *从字符串中获取机器人多边形的点.
 *
 * @param[in] str_footprint - string类型多边形点
 * @param[out] v_footprint - 机器人多边形点集
 * */
bool makeFootprintFromString(const std::string &str_footprint,
                             std::vector<WorldmapPoint> &v_footprint);

}  // namespace CVTE_BABOT

#endif