/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file navigation_arch_target.hpp
 *
 *@brief
 * 架构相关接口的基类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev
 *@data 2020-01-07
 ************************************************************************/
#ifndef __NAVIGATION_ARCH_TARGET_H
#define __NAVIGATION_ARCH_TARGET_H

#include <functional>
#include <vector>
#include "pose2d/pose2d.hpp"

namespace CVTE_BABOT {

/**
 * NavigationArchTarget
 * @brief 架构相关接口的基类
 **/
class NavigationArchTarget {
 public:
  virtual ~NavigationArchTarget() = default;

  /**
   *spin
   *@brief
   *用于spin循环运行各类事件
   *
   *@param[in] none
   *@param[out] none
   * */
  virtual void spin() = 0;

  /**
   *systemInit
   *@brief
   *架构相关通讯的初始化
   *
   *@param[in] none
   *@param[out] bool类型，true初始化成功，false为失败
   * */
  virtual bool systemInit() = 0;

  /**
   *stopNavigation
   *@brief
   * 停止导航
   *@param[out] true 代表停止成功
   * */
  virtual bool stopNavigation() = 0;

  /**
   *startNavigation
   *@brief
   * 启动导航
   *@param[out] true 代表启动成功
   * */
  virtual bool startNavigation() = 0;

  /**
   * startWithDefaultNavigation
   * @brief
   * 用于不带任务管理启动导航接口
   *
   * @param[out] true 代表启动成功
   * */
  virtual bool startWithDefaultNavigation() = 0;

  /**
   * updateParameter
   * @brief
   * 用于从外部更新navigation的参数
   *
   * @param[in] none
   * @param[out] none
   * */
  virtual bool updateParameter() = 0;
};

}  // namespace CVTE_BABOT

#endif