/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file navigation_pamams.hpp
 *
 *@brief params storage class.
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *
 *@modified chenmingjian (chenmingjian@cvte.com)
 *@version multi_motion.dev.1.0
 *@data 2020-01-16
 ************************************************************************/

#ifndef __NAVIGATION_PARAMS_HPP
#define __NAVIGATION_PARAMS_HPP
#include <map>
#include <vector>
#include <string>

namespace CVTE_BABOT {

/**
 * NavigationParameters
 * @brief
 * NavigationParameters中参数管理类
 **/
class NavigationParameters {
 public:
  /**
   *getParam
   *@brief
   *获取参数
   *
   *@param[in] s_index-参数名
   *@param[out] param-具体参数
   *@return true-获取成功，false-获取失败
   * */
  template <typename param_tpye>
  bool getParam(const std::string &s_index, param_tpye &param) {
    return findParam(getMap(param), s_index, param);
  }

  /**
   *setParam
   *@brief
   *设置参数
   *
   *@param[in] s_index-参数名
   *@param[out] param-具体参数
   *@return true-设置成功，false-设置失败
   * */
  template <typename param_tpye>
  void setParam(const std::string &s_index, const param_tpye &param) {
    setParam(getMap(param), s_index, param);
  }

 private:
  /**
   *findParam
   *@brief
   *查找参数
   *
   *@param[in] container-参数容器
   *@param[in] s_index-参数名
   *@param[out] param-具体参数
   *@return true-查找成功，false-查找失败
   * */
  template <typename param_tpye>
  bool findParam(std::map<std::string, param_tpye> &container,
                 const std::string &s_index, param_tpye &param) const {
    auto it = container.find(s_index);
    if (it != container.end()) {
      param = it->second;
      return true;
    }
    return false;
  }

  /**
   *setParam
   *@brief
   *设置参数
   *
   *@param[in] container-参数容器
   *@param[in] s_index-参数名
   *@param[out] param-具体参数
   *@return true-设置成功，false-设置失败
   * */
  template <typename param_tpye>
  void setParam(std::map<std::string, param_tpye> &container,
                const std::string &s_index, const param_tpye &param) {
    container[s_index] = param;
  }

  /**
   *getMap
   *@brief
   *获得 bool类型所有参数
   *
   *@return b_map_-bool的所有参数
   * */
  std::map<std::string, bool> &getMap(bool) { return b_map_; }

  /**
   *getMap
   *@brief
   *获得 int类型所有参数
   *
   *@return i_map_-int的所有参数
   * */
  std::map<std::string, int> &getMap(int) { return i_map_; }

  /**
   *getMap
   *@brief
   *获得 double类型所有参数
   *
   *@return d_map_-double的所有参数
   * */
  std::map<std::string, double> &getMap(double) { return d_map_; }

  /**
   *getMap
   *@brief
   *获得 string类型所有参数
   *
   *@return s_map_-string的所有参数
   * */
  std::map<std::string, std::string> &getMap(std::string) { return s_map_; }

  /**
   *getMap
   *@brief
   *获得 std::vector<double>类型所有参数
   *
   *@return v_map_-std::vector<double>的所有参数
   * */
  std::map<std::string, std::vector<double>> &getMap(std::vector<double>) {
    return v_map_;
  }

  std::map<std::string, bool> b_map_;         ///< 存储所有bool参数
  std::map<std::string, int> i_map_;          ///< 存储所有int参数
  std::map<std::string, double> d_map_;       ///< 存储所有double参数
  std::map<std::string, std::string> s_map_;  ///< 存储所有string参数
  std::map<std::string, std::vector<double>> v_map_;  ///< 存储所有vector参数
};

}  // namespace CVTE_BABOT
#endif  // __COSTMAP_PARAMS_HPP