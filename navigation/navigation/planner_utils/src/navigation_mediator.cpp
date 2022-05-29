/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file navigation_mediator.cpp
 *
 *@brief 导航路径类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-16
 ************************************************************************/

#include "navigation_mediator.hpp"

namespace CVTE_BABOT {
std::shared_ptr<NavigationMediator> NavigationMediator::ptr_instance_ = nullptr;
std::map<std::string, std::mutex> NavigationMediator::m_lock_;

NavigationMediator::NavigationMediator() { ptr_navigation_params_ = nullptr; }

std::shared_ptr<NavigationMediator> NavigationMediator::getPtrInstance() {
  if (!ptr_instance_.get()) {
    ptr_instance_.reset(new NavigationMediator());
    /// add code here
    /*to read param from yaml or other type files*/
  }
  return ptr_instance_;
}

} // namespace CVTE_BABOT
