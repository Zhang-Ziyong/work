/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_mediator.hpp
 *
 *@brief 中介者,用于传递参数与数据
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified huabo wu(wuhuabo@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-11
 ************************************************************************/
#ifndef __COSTMAP_MEDIATOR_HPP
#define __COSTMAP_MEDIATOR_HPP
#include <glog/logging.h>

#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <vector>

#include "costmap_params.hpp"
#include "costmap_utils.hpp"
#include "layered_costmap.hpp"
#include "eigen3/Eigen/Core"
#include <set>

namespace CVTE_BABOT {
class Costmap2d;
class LayeredCostmap;
class CostmapParameters;
const unsigned int POSE_BUFFER_SIZE = 100;

/**
 * CameraConfig
 * @brief  相机参数
 *  用于动态设置相机数据是否使用，动态设置相机的感知范围
 *
 **/
struct CameraConfig {
  bool use = true;
  double v_fov_ = -1;           ///<  垂直fov
  double h_fov_ = -1;           ///< 水平fov
  double min_d_ = -1;           ///<  最小测量距离
  double max_d_ = -1;           ///< 最大测量距离
  double negative_min_d_ = -1;  ///< 负向障碍最小测量距离
  double negative_max_d_ = -1;  ///< 负向障碍物最大测量距离
  double min_h_ = -1;           ///<  最小测量高度
  double max_h_ = -1;           ///<  最大测量高度
};

/**
 * CostmapMediator
 * @brief costmap各layer从外界获取数据及参数的类
 * 1. 通过CostmapParameters获取参数
 * 2. 为各layer提供获取数据的接口,降低layer间的耦合性
 * 3. 只有一个单例，所有costmap通过它获取数据
 **/
class CostmapMediator final {
  friend class CostmapMediatorTester;

 public:
  ~CostmapMediator() {
    LOG(INFO) << "CostmapMediator destructor." << std::endl;
  }

  /**
   *getPtrInstance
   *@brief
   *获取当前的单例指针
   *
   **/
  static std::shared_ptr<CostmapMediator> getPtrInstance();

  /**
   *setCostmapParameters
   *@brief
   *设置获取参数的类
   *
   * @param[in] ptr_costmap_params-参数存储类指针
   **/
  void setCostmapParameters(
      std::shared_ptr<CostmapParameters> ptr_costmap_params) {
    ptr_costmap_params_ = ptr_costmap_params;
  }

  /**
   *setCamreaConfig
   *@brief
   * 设置相机参数
   *
   * @param[in] camera_name - 对应相机topic名
   * @param[in] config - 设置相机参数
   **/
  inline void setCamreaConfig(const std::string &camera_name,
                              const CameraConfig &config) {
    camera_configs_[camera_name] = config;
  }
  /**
   *resetCameraConfig
   *@brief
   * 重置相机参数
   *
   * @param[in] camera_name - 对应相机topic名
   **/
  inline bool resetCameraConfig(const std::string &camera_name) {
    auto it = camera_configs_.find(camera_name);
    if (it != camera_configs_.end()) {
      camera_configs_.erase(camera_name);
      return true;
    } else {
      return false;
    }
  }
  /**
   *getCameraConfig
   *@brief
   * 重置相机参数
   *
   * @param[in] camera_name - 对应相机topic名
   * @param[out] camera_name - 相机参数
   **/
  inline bool getCameraConfig(const std::string &camera_name,
                              CameraConfig &config) {
    auto it = camera_configs_.find(camera_name);
    if (it != camera_configs_.end()) {
      config = it->second;
      return true;
    } else {
      return false;
    }
  }

  /**
   *setLayeredCostmap
   *@brief
   *设置LayeredCostmap指针
   *
   * @param[in] ptr_layered_costmap-LayeredCostmap指针
   *
   **/
  void setLayeredCostmap(
      const std::shared_ptr<LayeredCostmap> &ptr_layered_costmap) {
    ptr_layered_costmap_ = ptr_layered_costmap;
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
    if (!ptr_costmap_params_->getParam(s_index, param)) {
      param = default_param;
      LOG(WARNING) << s_index
                   << " use default value: ";  //<< default_param << " ";
    }
  }

  /**
   * getParam
   * @brief
   * 函数重载通过string类型索引设置std::vector<double>参数,
   * 使用默认值时的打印方式不一样
   *
   * @param[in] s_index-参数名索引
   * @param[out] param-要设置的参数
   * @param[in] default_param-默认值
   * @return true-找到索引并设置值， false-没有找到索引，使用默认值
   * */
  void getParam(const std::string &s_index, std::vector<double> &param,
                const std::vector<double> &default_param) {
    if (!ptr_costmap_params_->getParam(s_index, param)) {
      param = default_param;
      LOG(WARNING) << s_index << " use default value: ";
      for (size_t i = 0; i < default_param.size(); i++) {
        std::cout << default_param[i];
      }
      std::cout << std::endl;
    }
  }

  /**
   * getParam
   * @brief
   * 函数重载通过string类型索引设置std::vector<double>参数,
   * 使用默认值时的打印方式不一样
   *
   * @param[in] s_index-参数名索引
   * @param[out] param-要设置的参数
   * @param[in] default_param-默认值
   * @return true-找到索引并设置值， false-没有找到索引，使用默认值
   * */
  void getParam(const std::string &s_index, std::vector<WorldmapPoint> &param,
                const std::vector<WorldmapPoint> &default_param) {
    if (!ptr_costmap_params_->getParam(s_index, param)) {
      param = default_param;
      LOG(WARNING) << s_index << " use default value: ";
      for (size_t i = 0; i < default_param.size(); i++) {
        std::cout << "(" << default_param[i].d_x << ", " << default_param[i].d_y
                  << ") ";
      }
      std::cout << std::endl;
    }
  }

  /**
   * getCostmap
   * @brief
   * 通过string类型索引设置参数，有相应
   *
   * @param[in] s_index-参数名索引
   * @param[out] param-要设置的参数
   * @param[in] default_param-默认值
   * @return true-找到索引并设置值， false-没有找到索引，使用默认值
   * */
  std::shared_ptr<const Costmap2d> getCostmap();

  /**
   * getInscribedRadius
   * @brief
   * 获取机器人圆形半径.
   *
   * @return inscribed_radius_-机器人圆形半径.
   * */
  inline double getInscribedRadius() {
    return ptr_layered_costmap_->getInscribedRadius();
  }

  /**
   * resetStaticMap
   * @brief
   * 重新加载static_layer地图
   *
   * */
  inline void setResetStaticMapFlag(const bool &reset_static_map) {
    b_reset_static_map_ = reset_static_map;
  }

  /**
   * ifResetStaticMap
   * @brief
   * 是否重新加载static_layer地图.
   *
   * @return true-是， false-不是
   * */
  inline bool ifResetStaticMap() { return b_reset_static_map_; }

  /**
   * resetStaticMap
   * @brief
   * 重新加载static_layer地图
   *
   * */
  inline void setResetCostmapFlag(const bool &reset_costmap) {
    b_reset_costmap_ = reset_costmap;
  }

  /**
   * ifResetCostmap
   * @brief
   * 是否重新加载costmap地图.
   *
   * @return true-是， false-不是
   * */
  inline bool ifResetCostmap() { return b_reset_costmap_; }

  /**
   * isRolling
   * @brief
   * 判断layered_costmap是否是rolling window.
   *
   * @return true-是rolling window， false-不是rolling window
   * */
  inline bool isRolling() { return ptr_layered_costmap_->isRolling(); }

  /**
   * isMarkingUnknown
   * @brief
   * 判断layered_costmap地图是否使用NO_INFORMATION点.
   *
   * @return true-使用， false-不使用
   * */
  inline bool isMarkingUnknown() {
    return ptr_layered_costmap_->isMarkingUnknown();
  }

  /**
   * isParameterReady
   * @brief
   * 参数类是否被初始化
   *
   * @return true-已初始化， false-未初始化
   * */
  bool isParameterReady() { return (ptr_costmap_params_.get() != NULL); }

  /**
   * isLayeredCostmapReady
   * @brief
   * layered_costmap是否被初始化
   *
   * @return true-已初始化， false-未初始化
   * */
  bool isLayeredCostmapReady() { return (ptr_layered_costmap_.get() != NULL); }

  /**
   * resizeMap
   * @brief
   * 更新layered_costmap的costmap
   *
   * @param[in] size_x-地图x坐标大小,单位个
   * @param[in] size_y-地图y坐标大小,单位个
   * @param[in] resolution-地图分辨率,单位m
   * @param[in] origin_x-地图原点x坐标,
   * @param[in] origin_y-地图原点y坐标,
   * @param[in] size_locked-地图是否锁住
   * @return true-成功，false-失败
   * */
  void resizeMap(const unsigned int &size_x, const unsigned int &size_y,
                 const double &resolution, const double &origin_x,
                 const double &origin_y, const bool &size_locked = false) {
    ptr_layered_costmap_->resizeMap(size_x, size_y, resolution, origin_x,
                                    origin_y, size_locked);
  }

  /**
   *setPose
   *@brief
   *简介
   *
   *@param[in] current_pose-当前的时间戳与位姿
   **/
  void setPose(const WorldmapPoseWithTime &current_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    // 使用list还存储一定长度的位姿
    l_current_pose_.push_front(current_pose);
    if (l_current_pose_.size() > POSE_BUFFER_SIZE) {
      l_current_pose_.pop_back();
    }
  }

  /**
   *getPose
   *@brief
   *输出与查询的时间戳最匹配的位姿
   *
   *@param[in] expect_time-想查询的时间戳
   *@param[out] match_pose-相应的位姿
   **/
  void getPose(const double &expect_time, WorldmapPose &match_pose) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    double min_time = 1e30;
    double time_error = 0.000;
    auto min_it = l_current_pose_.begin();
    for (auto it = l_current_pose_.begin(); it != l_current_pose_.end(); it++) {
      time_error = fabsf64x(it->d_time - expect_time);
      if (time_error < min_time) {
        min_time = time_error;
        min_it = it;
      } else {
        break;
      }
    }

    match_pose = {min_it->d_x, min_it->d_y, min_it->d_yaw};
  }

  /**
   *getCurrentTime
   *@brief
   *输出时间戳
   *
   *@param[out] current_time-想查询的时间戳
   **/
  bool getCurrentTime(double &current_time) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!l_current_pose_.empty()) {
      current_time = l_current_pose_.front().d_time;
      return true;
    } else {
      return false;
    }
  }

  /**
   *findDynamicCar
   *@brief
   *查询是否有需要避让的汽车,每次查询状态后清除标志
   *
   *@param[out]
   *dynamic_car_point-需要避让的汽车所在的位置,没有需要避让的汽车的话不对其操作
   *@return true-有需要避让的汽车, false-没有
   **/
  bool findDynamicCar(WorldmapPoint &dynamic_car_point);

  /**
   *setDynamicCar
   *@brief
   *设置需要避让的车辆所在的世界坐标
   *
   *@param[in] dynamic_car-需要避让的车辆所在的世界坐标
   *@return true-, false-
   **/
  void setDynamicCar(const WorldmapPoint &dynamic_car) {
    std::lock_guard<std::mutex> lock(dynamic_mutex_);
    avoidance_car_position_ = dynamic_car;
    b_find_dynamic_cars_ = true;
  }

  /**
   * @brief Set the Clear Polygons object
   *
   * @param polygons
   */
  void setClearPolygons(
      const std::vector<std::vector<WorldmapPoint>> &polygons) {
    clear_polygons_ = polygons;
  }
  /**
   * @brief Get the Clear Polygons object
   *
   * @return const std::vector<std::vector<WorldmapPoint>>&
   */
  std::vector<std::vector<WorldmapPoint>> getClearPolygons() const {
    return clear_polygons_;
  }
  /**
   * @brief Set the Prohibited Polygons object
   *
   * @param polygons
   */
  void setProhibitedPolygons(
      const std::vector<std::vector<WorldmapPoint>> &polygons) {
    prohibited_polygons_ = polygons;
  };
  /**
   * @brief Get the Prohibited Polygons object
   *
   * @return const std::vector<std::vector<WorldmapPoint>>&
   */
  std::vector<std::vector<WorldmapPoint>> getProhibitedPolygons() const {
    return prohibited_polygons_;
  }

  /**
   * getFootprint
   * @brief
   * 获取机器人占用的范围点集
   *
   * @return 范围点集
   * */
  const std::vector<CostmapPoint> &getFootprint();

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
  static bool getData(const std::string &data_name, data_type &data) {
    bool data_registered = (m_lock_.find(data_name) != m_lock_.end());
    if (data_registered) {
      m_lock_[data_name].lock();
      data = m_data_<data_type>[data_name];
      m_lock_[data_name].unlock();
      return true;
    } else {
      LOG(ERROR) << "Data: " << data_name << " not registered yet. ";
      return false;
    }
  }

  /**
   * registerUpdateFunc
   * @brief
   * 注册string类型索引的存储载体，和函数用于传入数据给载体
   * @param[in] input_data_type-传入数据类型
   * @param[in] data_name-存储索引
   * @param[in]
   * init_value-载体初值，如果存储载体是指针，需在外界传入分配好内存的指针！
   * @param[in]
   * updateFunc-传入数据给载体的函数，指明添加数据到载体,默认方法是用输入替换原先存储的值
   *  @param[in] input_data-传入的数据
   *  @param[out] storage_data-存储载体
   *
   * */
  template <typename input_data_type>
  static void registerUpdateFunc(const std::string &data_name,
                                 const input_data_type &init_value) {
    bool data_not_registered = (m_lock_.find(data_name) == m_lock_.end());
    if (data_not_registered) {
      m_lock_[data_name];
      registerDataType(data_name, init_value);
    } else {
      LOG(INFO) << "Data: " << data_name << " has been registered yet. ";
    }
  }

  /**
   * updateData
   * @brief
   * 通过string类型索引设置数据,如果不想数据内容被改变,请传入副本,重要!!!!
   * @param[in] data_type-要获取的类型，可由data参数自动解析
   * @param[in] data_name-存储索引
   * @param[in] data-数据载体，获取到的数据赋给它！
   *
   * */
  template <typename input_data_type>
  static bool updateData(const std::string &data_name,
                         const input_data_type &input_data) {
    bool data_registered = (m_lock_.find(data_name) != m_lock_.end());
    if (data_registered) {
      m_lock_[data_name].lock();
      m_data_<input_data_type>[data_name] = input_data;
      m_lock_[data_name].unlock();
      return true;
    } else {
      LOG(ERROR) << "data_name: " << data_name
                 << ", updateData failed for update function not registered.";
      return false;
    }
  }

  bool checkSensorEnable(std::string key) {
    return (s_sensor_enable.find(key) != s_sensor_enable.end());
  }

  void setSensorEnable(std::string key) { s_sensor_enable.insert(key); }

  void resetSensorEnable(std::string key) {
    if (key.size() == 0) {
      s_sensor_enable.clear();
    } else {
      s_sensor_enable.erase(key);
    }
  }

 private:
  CostmapMediator();

  CostmapMediator(const CostmapMediator &obj) = delete;

  CostmapMediator &operator=(const CostmapMediator &obj) = delete;

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
    bool data_not_registered = (m_lock_.find(data_name) == m_lock_.end());
    if (data_not_registered) {
      m_lock_[data_name];
    }
    m_data_<data_type>[data_name] = init_value;
  }

  static std::map<std::string, std::mutex> m_lock_;

  template <typename data_type>
  static std::map<std::string, data_type> m_data_;
  static std::shared_ptr<CostmapMediator>
      ptr_instance_;  ///< 数据传输类的单例实现

  std::shared_ptr<LayeredCostmap> ptr_layered_costmap_;

  std::shared_ptr<CostmapParameters>
      ptr_costmap_params_;               ///< 存储与查找获取参数的类
  std::vector<CostmapPoint> footprint_;  ///< 机器人范围点集

  bool b_reset_static_map_ = true;
  bool b_reset_costmap_ = false;

  WorldmapPose current_velocity_;
  std::list<WorldmapPoseWithTime> l_current_pose_;

  std::mutex velocity_mutex_;
  std::mutex pose_mutex_;
  std::mutex dynamic_mutex_;

  bool b_find_dynamic_cars_ = false;
  WorldmapPoint avoidance_car_position_;
  std::vector<std::vector<WorldmapPoint>>
      clear_polygons_;  ///< 设置需要清空为自由区域的多边形区域
  std::vector<std::vector<WorldmapPoint>>
      prohibited_polygons_;  ///< 设置需要设置为致命值的多边形区域

  std::set<std::string> s_sensor_enable;  ///<传感器使能标志
  std::map<std::string, CameraConfig> camera_configs_;
};

/*类模板静态成员需要在头文件定义*/
template <typename data_type>
std::map<std::string, data_type> CostmapMediator::m_data_;

}  // namespace CVTE_BABOT

#endif  // __COSTMAP_MEDIATOR_HPP
