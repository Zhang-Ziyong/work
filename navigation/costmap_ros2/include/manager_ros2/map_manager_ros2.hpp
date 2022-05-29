/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file costmap_arch_adapter_ros2.hpp
 *
 *@brief
 * 架构相关接口类
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev
 *@data 2019-04-21
 ************************************************************************/
#ifndef __COSTMAP_ARCH_ADAPTER_ROS2_H
#define __COSTMAP_ARCH_ADAPTER_ROS2_H
#include <tf2/utils.h>

#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/int8.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "costmap_2d.hpp"
#include "costmap_mediator.hpp"
#include "costmap_utils.hpp"
#include "message_filter/message_filter.hpp"
#include "pose2d/pose2d.hpp"
namespace CVTE_BABOT {
class DataManagerRos2;
class CostmapVisualizerRos2;
class CostmapParamsRos2;
/**
 * CostmapArchAdapter
 * @brief 架构相关接口的ROS2实现类
 **/
class CostmapArchAdapter final {
 public:
  static std::shared_ptr<CostmapArchAdapter> getPtrInstance() {
    static std::shared_ptr<CostmapArchAdapter> ptr_instance = nullptr;
    if (ptr_instance == nullptr) {
      ptr_instance.reset(new CostmapArchAdapter("costmap"));
    }
    return ptr_instance;
  }

  ~CostmapArchAdapter() {
    stop_thread_ = true;
    if (cosmap_thread_ != nullptr && cosmap_thread_->joinable()) {
      cosmap_thread_->join();
    }
    if (pub_cosmap_thread_ != nullptr && pub_cosmap_thread_->joinable()) {
      pub_cosmap_thread_->join();
    }
  }

  /**
   *spin
   *@brief
   *用于spin循环运行各类事件
   *
   *@param[in] none
   *@param[out] none
   * */
  void spin();

  /**
   *systemInit
   *@brief
   *架构相关通讯的初始化
   *
   *@param[in] none
   *@param[out] bool类型，true初始化成功，false为失败
   * */
  bool systemInit();

  /**
   *startCostmapTimer
   *@brief
   *启动costmap的定时器
   *
   *@param[in] none
   *@param[out] none
   * */
  void startCostmapTimer();

  /**
   *startAvoidanceTimer
   *@brief
   *启动避障的定时器
   *
   *@param[in] none
   *@param[out] none
   * */
  void startAvoidanceTimer();

  /**
   *stopCostmapTimer
   *@brief
   *停止costmap的定时器
   *
   *@param[in] none
   *@param[out] none
   * */
  void stopCostmapTimer();

  /**
   * updateParameter
   * @brief
   * 用于从传入的yaml文件中更新base_parameter的参数
   *
   * @param[in] none
   * @param[out] none
   * */
  void updateParameter();

  /**
   * @brief
   * 打开特定层costmap
   * @param layer
   */
  void activeLayer(LayerName layer);
  /**
   * @brief
   * 关闭特定层costmap
   * @param layer
   */
  void deactiveLayer(LayerName layer);

  /**
   * @brief
   * reset特定层costmap
   * @param layer
   */
  void resetLayer(LayerName layer);

  /**
   *getCostmap
   *@brief
   *提供给导航，用于获取costmap.
   *
   *@param[out] ptr_costmap_ - 拷贝后的costmap
   * */
  inline std::shared_ptr<Costmap2d> getCostmap() {
    std::unique_lock<std::recursive_mutex> lock(*ptr_costmap_->getMutx());
    return ptr_costmap_;
  }

  inline std::shared_ptr<Costmap2d> getLayerCostmapByname(
      const std::string layer_name) {
    ptr_layered_costmap_->getLayerCostmapByname(layer_name);
  }

  /**
   *getNodeHander
   *@brief
   *提供给导航，用于获取当前node.
   *
   *@return node_ - node指针
   * */
  inline rclcpp::Node::SharedPtr getNodeHander() { return node_; }
  inline rclcpp::Node::SharedPtr getTimerNodeHander() { return timer_node_; }
  /**
   *setLocalizationData
   *@brief
   *用于传入定位数据.
   *
   * @param[in] localization_msg - 定位数据消息
   * */
  void setLocalizationData(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
          localization_msg) {
    localizationDataCallback(localization_msg);
  }

  /**
   * resetStaticMap
   * @brief
   * 用于重新加载static_layer地图
   *
   * @param[in] map_path-地图存储地址
   * @param[out] bool类型，true设置成功，false为失败
   * */
  bool resetStaticMap(const std::string &map_path);

  /**
   * existStaticMap
   * @brief
   * 判断是否通过配置参数获取代价地图，并返回地图路径
   *
   * @param[out] map_path-地图存储地址
   * @param[out] bool类型，true设置成功，false为失败
   * */
  bool existInitStaticMap(std::string &map_path);

  /**
   * rangeTopicReceiveSet
   * @brief
   * range类型接收控制
   *
   * @param[in] dir-方向
   * @param[in] bool类型，true设置打开，false为关闭
   * */
  void rangeTopicReceiveSet(int dir, bool enable);

  // inline bool setSlopeArea(
  //     const std::vector<std::vector<WorldmapPose>> &clear_areas) {
  //   if (ptr_layered_costmap_ == nullptr) {
  //     return false;
  //   }
  //   ptr_layered_costmap_->setCleanAreas(clear_areas);
  //   return true;
  // }

  inline void setUpdateInShope(const bool update_in_shope) {
    if (ptr_layered_costmap_ == nullptr) {
      return;
    }
    ptr_layered_costmap_->setUpdateInShope(update_in_shope);
  }

  inline unsigned int getCostmapId() { return costmap_id_; }

 private:
  explicit CostmapArchAdapter(const std::string &name);

  inline void upateCostmapId() {
    if (costmap_id_ > 65530) {
      costmap_id_ = 0;
    } else {
      costmap_id_++;
    }
  }
  /**
   *prepareGrid
   *@brief
   *将costmap转换成栅格地图.
   *
   * */
  void prepareGrid();

  /**
   *costmapTimerCallback
   *@brief
   *costmap定时器回调函数.
   *
   * */
  void costmapTimerCallback();

  /**
   * @brief 发布代价地图线程
   *
   */
  void pubCostmapTimeCallback();

  /**
   *laserScanCallback
   *@brief
   *将激光数据转换成点云数据后输入给避障,不变换坐标系
   *
   * @param[in] ptr_scan_msg - 激光数据消息
   * */
  void laserScanCallback(
      const sensor_msgs::msg::LaserScan::SharedPtr ptr_scan_msg,
      const std::string &topic_name);

  /**
   *localizationDataCallback
   *@brief
   *定位回调函数.
   *
   * @param[in] localization_msg - 定位数据消息
   * */
  void localizationDataCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
          localization_msg);

  /**
   *setUnpaddedRobotFootprint
   *@brief
   *设置扩展后的机器人多边形.
   *
   * @param[in] points - 机器人多边形点集
   * */
  void setpaddedRobotFootprint(const std::vector<WorldmapPoint> &points);

  double width_;                ///< costmap宽
  double height_;               ///< costmap高
  double resolution_;           ///< costmap分辨率
  double update_frequency_;     ///< costmap更新频率
  double d_footprint_padding_;  ///< 机器人多边形扩展系数
  unsigned int ui_reset_costmap_time_threshhold_;  ///<
  ///进入多少次costmapTimer清除costmap障碍层

  unsigned int ui_reset_costmap_count_ = 0;  ///<
  ///大于ui_reset_costmap_time_threshhold_清除costmap障碍层

  std::string map_path_;          ///< 地图名字
  std::string node_name_;         ///< 节点名字
  std::string pose_topic_;        ///< 机器人姿态的topic名字
  std::string str_footprint_;     ///< 机器人多边形点
  std::string s_stop_footprint_;  ///< 机器人多边形点

  bool b_init_static_map_ =
      true;  ///<
             ///是否初始化static_map,不初始化则static_layer需要调用resetStaticMap初始化
  bool rolling_window_ = false;        ///< 是否滚动
  bool b_mark_unknown_space_ = false;  ///< 是否扩展未知区域

  rclcpp::Node::SharedPtr timer_node_;          ///< 定时器节点
  rclcpp::Node::SharedPtr node_;                ///< 用于收发数据的节点
  rclcpp::TimerBase::SharedPtr costmap_timer_;  ///< costmap定时器
  // rclcpp::TimerBase::SharedPtr avoidance_timer_;  ///< 避障定时器
  std::shared_ptr<CostmapParamsRos2>
      costmap_params_ros2_;  ///< 和ros2的参数接口

  int laser_scan_callback_count_ = 0;              ///< 雷达回调降频
  std::vector<WorldmapPoint> v_padded_footprint_;  ///< 扩展后的机器人多边形

  static boost::shared_array<char> ptr_cost_translation_table_;  ///< 存储代价值

  std::shared_ptr<CostmapParameters> ptr_costmap_params_;  ///< costmap所有参数
  std::shared_ptr<LayeredCostmap>
      ptr_layered_costmap_;                 ///< costmap layer管理器
  std::shared_ptr<Costmap2d> ptr_costmap_;  ///< 当前costmap
  unsigned int costmap_id_ = 0;

  WorldmapPose robot_current_pose_;  ///<机器人当前姿态

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      localization_sub_;  ///< 用于订阅localization数据
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr
      ptr_reset_costmap_sub_;  //清除costmap
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
  //     ptr_speed_pub_;  //发布速度
  // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
  //     ptr_repulsion_pub_;  //发布斥力
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cmd_vel_sub_ =
      nullptr;  ///< 用于订阅控制速度
  std::thread *cosmap_thread_;
  std::thread *pub_cosmap_thread_;
  bool run_thread_ = false;
  bool stop_thread_ = false;

  std::mutex localization_mutex_;
  std::mutex cmd_vel_mutex_;
  std::mutex avoidance_mutex_;

  WorldmapPoint extra_slow_down_area_;
  WorldmapPoint extra_stop_area_;

  double d_extra_slow_down_area_size_x_;
  double d_extra_slow_down_area_size_y_;
  double d_extra_slow_down_area_threshold_x_;
  double d_extra_slow_down_area_threshold_w_;

  double d_extra_stop_area_size_x_;
  double d_extra_stop_area_size_y_;
  double d_extra_stop_area_threshold_x_;
  double d_extra_stop_area_threshold_w_;
  double d_extra_area_after_stop_size_x_;

  bool b_publish_costmap_;

  bool b_use_navigation_localization_;
  bool b_use_navigation_cmd_vel_;

  bool b_publish_voxel_, b_publish_clearing_points_;
  std::string s_cleraing_cloud_name_;

  bool clean_area_enable_ = false;

  std::shared_ptr<DataManagerRos2> ptr_data_manager_ros2_;
  std::shared_ptr<CostmapVisualizerRos2> ptr_costmap_visualizer_ros2_;
};

}  // namespace CVTE_BABOT

#endif
