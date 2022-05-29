/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file supervisor_chain.hpp
 *
 *@brief .
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-25
 ************************************************************************/
#ifndef __SUPERVISOR_CHAIN_HPP
#define __SUPERVISOR_CHAIN_HPP

#include <list>
#include "supervisor.hpp"

namespace CVTE_BABOT {

/************************************************
 * 单例模式，每个状态下进行监督时，可以直接获取同一个监督器
 * **********************************************/

class SupervisorChain {
 public:
  /**
   *getPtrInstance
   *@brief
   *  获取当前的单例指针
   **/
  static std::shared_ptr<SupervisorChain> getPtrInstance() {
    if (vis_chain_ptr_.get() == nullptr) {
      vis_chain_ptr_.reset(new SupervisorChain());
    }
    return vis_chain_ptr_;
  }

  /**
   *addSupervisor
   *@brief
   *  在监督链中添加不同的监督器，后续每次监督操作，都会执行里面
   *  所有监督器的要求操作
   *
   *@param[in] supervisor - 不同监督内容的指针
   **/
  void addSupervisor(Supervisor::Ptr supervisor);

  /**
   *supervise
   *@brief
   *  执行一次监督操作，包括监督链（list）中所有的监督类
   *
   *@param[in] state - 监督时需要使用的信息，一般为路径和状态信息
   *@return 监督结果
   **/
  SupervisorResult supervise(SupervisorState &state);

  /**
   *superviseCycle
   *@brief
   *  循环监督操作，包括监督链（list）中所有的监督类，监督链中不同监督会有不同频率
   *
   *@param[in] state - 监督时需要使用的信息，一般为路径和状态信息
   *@return 监督结果
   **/
  SupervisorResult superviseCycle(SupervisorState &state);

  /**
   *superviseAssignOne
   *@brief
   *  进行指定监督链中某一监督器操作
   *
   *@param[in] state - 监督时需要使用的信息，一般为路径和状态信息
   *@param[in] name - 指定监督器的名字
   *@return 监督结果
   **/
  SupervisorResult superviseAssignOne(SupervisorState &state,
                                      const std::string &names);

 private:
  SupervisorChain();
  SupervisorChain(const SupervisorChain &obj) = delete;
  SupervisorChain &operator=(const SupervisorChain &obj) = delete;

  std::list<Supervisor::Ptr> supervisors_;

  static std::shared_ptr<SupervisorChain> vis_chain_ptr_;
};

}  // namespace CVTE_BABOT

#endif  // __SUPERVISORCHAIN_H
