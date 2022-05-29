/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, CVTE.
 * All rights reserved.
 *
 *@file navigation_mediator.hpp
 *
 *@brief 导航路径类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2019-12-16
 ************************************************************************/
#ifndef __NAVIGATION_MEDIATOR_HPP
#define __NAVIGATION_MEDIATOR_HPP
#include <glog/logging.h>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "navigation_params.hpp"

namespace CVTE_BABOT {

/**
 * type_trait
 * @brief 解析数据存储类型的模块类，针对每种类型可定义特化
 * @param[in] data_type-要解析的类型
 * @param[out] storage_type-数据存储类型
 *
 **/
template <typename data_type>
struct type_trai {
  typedef std::shared_ptr<std::vector<data_type>> storage_type;
};

class NavigationMediator final {
 public:
  ~NavigationMediator() = default;
  /**
   *getPtrInstance
   *@brief
   *获取当前的单例指针
   *
   **/
  static std::shared_ptr<NavigationMediator> getPtrInstance();

  /**
   *setCostmapParameters
   *@brief
   *设置获取参数的类
   *
   * @param[in] ptr_costmap_params-参数存储类指针
   **/
  void setNavigationParameters(
      std::shared_ptr<NavigationParameters> ptr_navigation_params) {
    ptr_navigation_params_ = ptr_navigation_params;
  }

  /**
   * getParam
   * @brief
   * 通过string类型索引设置参数
   *
   * @param[in] s_index-参数名索引
   * @param[out] param-要设置的参数
   * @param[in] default_param-默认值
   * @return true-找到索引并设置值， false-没有找到索引，使用默认值
   * */
  template <typename param_type>
  void getParam(const std::string &s_index, param_type &param,
                const param_type &default_param) {
    if (!ptr_navigation_params_->getParam(s_index, param)) {
      param = default_param;
      LOG(WARNING) << s_index
                   << " use default value: ";  // << default_param << " ";
    }
  }

  /**
   * isParameterReady
   * @brief
   * 参数类是否被初始化
   *
   * @return true-已初始化， false-未初始化
   * */
  bool isParameterReady() { return (ptr_navigation_params_.get() != NULL); }

  /**
   * registerDataType
   * @brief 请求在mediator中存储某种类型， 并注册函数用于传入数据给这种类型
   * @param[in] data_type-要存储的类型
   * @param[in] data_name-存储索引，用于之后获取数据
   * @param[in]
   *init_value-数据初值，如果存储类型是指针，需在外界传入分配好内存的指针！
   *
   **/
  template <typename data_type>
  static void registerDataType(const std::string &data_name,
                               const data_type &init_value) {
    if (m_lock_.find(data_name) == m_lock_.end()) {
      m_lock_[data_name];
    }
    m_data_<data_type>[data_name] = init_value;
  }

  /**
   * getData
   * @brief 向mediator请求某种类型的数据，
   *数据必须是调用registerDataType注册过的！
   * @param[in] data_type-要获取的类型，可由data参数自动解析
   * @param[in] data_name-存储索引
   * @param[in] data-数据载体，获取到的数据赋给它！
   *
   **/
  template <typename data_type>
  static void getData(const std::string &data_name, data_type &data) {
    if (m_lock_.find(data_name) != m_lock_.end()) {
      m_lock_[data_name].lock();
      data = m_data_<data_type>[data_name];
      m_lock_[data_name].unlock();
    } else {
      LOG(ERROR) << "Data: " << data_name << " not registered yet. ";
    }
  }

  /**
   * registerUpdateFunc
   * @brief
   * 注册string类型索引的存储载体，和函数用于传入数据给载体
   * @param[in] input_data_type-传入数据类型
   * @param[in] data_name-存储索引
   * @param[in] updateFunc-传入数据给载体的函数，指明添加数据到载体
   *  @param[in] input_data-传入的数据
   *  @param[out] storage_data-存储载体
   * @param[in]
   * init_value-载体初值，如果存储载体是指针，需在外界传入分配好内存的指针！
   *
   * */
  template <typename input_data_type>
  static void registerUpdateFunc(
      const std::string &data_name,
      std::function<
          void(const input_data_type &input_data,
               typename type_trai<input_data_type>::storage_type &storage_data)>
          updateFunc,
      const typename type_trai<input_data_type>::storage_type &init_value) {
    if (m_lock_.find(data_name) == m_lock_.end()) {
      m_update_funcs_<input_data_type>[data_name] = updateFunc;
      m_lock_[data_name];
      registerDataType(data_name, init_value);
    } else {
      LOG(ERROR) << "Data: " << data_name << " has been registered yet. ";
    }
  }

  /**
   * updateData
   * @brief
   * 通过string类型索引设置参数
   * @param[in] data_type-要获取的类型，可由data参数自动解析
   * @param[in] data_name-存储索引
   * @param[in] data-数据载体，获取到的数据赋给它！
   *
   * */
  template <typename input_data_type>
  static bool updateData(const std::string &data_name,
                         const input_data_type &input_data) {
    auto it = m_update_funcs_<input_data_type>.find(data_name);
    if (it != m_update_funcs_<input_data_type>.end()) {
      m_lock_[data_name].lock();
      it->second(
          input_data,
          m_data_<
              typename type_trai<input_data_type>::storage_type>[data_name]);
      m_lock_[data_name].unlock();
      return true;
    } else {
      LOG(ERROR) << "updateData failed for update function not registered."
                 << std::endl;
      return false;
    }
  }

  /**
   * updateRegisterData
   * @brief
   * 通过string类型索引设置数据,如果不想数据内容被改变,请传入副本,重要!!!!
   * @param[in] data_type-要获取的类型，可由data参数自动解析
   * @param[in] data_name-存储索引
   * @param[in] data-数据载体，获取到的数据赋给它！
   *
   * */
  template <typename input_data_type>
  static bool updateRegisterData(const std::string &data_name,
                                 const input_data_type &input_data) {
    bool data_registered = (m_lock_.find(data_name) != m_lock_.end());
    if (data_registered) {
      m_lock_[data_name].lock();
      m_data_<input_data_type>[data_name] = input_data;
      m_lock_[data_name].unlock();
      return true;
    } else {
      LOG(ERROR) << "data_name: " << data_name
                 << ", updateData failed for update type not registered.";
      return false;
    }
  }

 private:
  NavigationMediator();

  NavigationMediator(const NavigationMediator &obj) = delete;

  NavigationMediator &operator=(const NavigationMediator &obj) = delete;

  template <typename data_type>
  static std::map<std::string, data_type> m_data_;

  template <typename data_type>
  static std::map<std::string,
                  std::function<void(const data_type &input_data,
                                     typename type_trai<data_type>::storage_type
                                         &target_data)>>
      m_update_funcs_;

  static std::map<std::string, std::mutex> m_lock_;
  static std::shared_ptr<NavigationMediator>
      ptr_instance_;  ///< 数据传输类的单例实现

  std::shared_ptr<NavigationParameters>
      ptr_navigation_params_;  ///< 存储与查找获取参数的类
};

/*类模板静态成员需要在头文件定义*/
template <typename data_type>
std::map<std::string,
         std::function<void(
             const data_type &input_data,
             typename type_trai<data_type>::storage_type &target_data)>>
    NavigationMediator::m_update_funcs_;

/*类模板静态成员需要在头文件定义*/
template <typename data_type>
std::map<std::string, data_type> NavigationMediator::m_data_;

}  // namespace CVTE_BABOT

#endif
