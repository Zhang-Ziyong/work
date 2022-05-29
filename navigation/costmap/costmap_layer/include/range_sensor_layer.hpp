/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file range_sensor_layer.hpp
 *
 *@brief costmap range sensor layer.
 *
 *@modified by chenmingjian(chenmingjian@cvte.com)

 *@paper A comparison of three uncertainty calculi for building sonar-based
 occcupancy grids

 @modified by FangpingYang(yangfangping@cvte.com)
 @date 2021-09-02
 * 1.前/后向超声，大于1米的障碍检测，视为无效，直接清除
 * 2.侧向向超声，大于0.5米的障碍检测，视为无效，直接清除
 * 3.上述判断在processVariableRangeMsg中生效
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@date 2019-04-08
 ************************************************************************/
#ifndef __RANGE_SENSOR_LAYER_HPP
#define __RANGE_SENSOR_LAYER_HPP

#include <list>
#include <time.h>
#include "costmap_layer.hpp"
namespace CVTE_BABOT {
class CostmapRangeData;
/**
 * RangeSensorLayer
 * @brief 实现costmap超声层，有如下特征：
 * 1.获取范围数据costmap_range_msg更新地图
 * 2.不同高度的超声一般创建不同的超声层
 **/
class RangeSensorLayer final : public CostmapLayer {
  friend class range_sensor_layer_tester;

 public:
  RangeSensorLayer();
  virtual ~RangeSensorLayer();
  RangeSensorLayer(const RangeSensorLayer &obj) = delete;
  RangeSensorLayer &operator=(const RangeSensorLayer &obj) = delete;

  /**
   * onInitialize
   * @brief
   * 初始化当前层的参数，在基类的initialize函数中被调用
   *
   * @return true-初始成功，false-初始失败
   * */
  bool onInitialize();

  void matchSize() override;

  /**
   * getParams
   * @brief
   * 初始化当前层的参数
   *
   * */
  void getParams() override;

  /**
   * updateSensorData
   * @brief
   * 更新传感器数据
   *
   * */
  bool updateSensorData();

  /**
   * updateCostmap
   * @brief
   * 更新costmap
   *
   * @param[in] rangeData-传感器数据
   * @param[in] clear_sensor_cone-是否清除
   * */
  void updateCostmap(const CostmapRangeData &rangeData, bool clear_sensor_cone);
  /**
   * updateBounds
   * @brief
   * 根据机器人坐标以及层的大小，更新layer的边界
   *
   * @param[in] d_robot_x-机器人x坐标,单位m
   * @param[in] d_robot_y-机器人y坐标,单位m
   * @param[in] d_robot_yaw-机器人角度,单位rad
   * @param[in] d_min_x-边界x轴的最小值
   * @param[in] d_min_y-边界y轴的最小值
   * @param[in] d_max_x-边界y轴的最小值
   * @param[in] d_max_y-边界y轴的最小值
   *
   * */
  bool updateBounds(const WorldmapPose &wp_robot_pose,
                    CostmapBound &cb_costmap_bound) override;

  /**
   * updateCosts
   * @brief
   * 更新costmap的代价值
   *
   * @param[in] master_grid-更新的costmap对象
   * @param[in] i_min_i-更新范围的最小x
   * @param[in] i_min_j-更新范围的最小x
   * @param[in] i_max_i-更新范围的最大y
   * @param[in] i_max_j-更新范围的最大y
   *
   * */
  bool updateCosts(const std::shared_ptr<Costmap2d> master_grid,
                   const int &i_min_i, const int &i_min_j, const int &i_max_i,
                   const int &i_max_j) override;

  /**
   *updateOrigin
   *@brief
   *重写更新costmap原点接口
   *
   *@param[in] d_new_origin_x-原点x轴坐标
   *@param[in] d_new_origin_y-原点y轴坐标
   * */
  void updateOrigin(const double &d_new_origin_x, const double &d_new_origin_y);

  /**
   * activate
   * @brief
   * 使能当前层
   *
   * */
  void activate() override;

  /**
   * deactivate
   * @brief
   * 使当前层失效不更新
   *
   * */
  void deactivate() override;

  /**
   * reset
   * @brief
   * 重置当前层的数据
   *
   * */
  void reset() override;

 private:
  /**
   * processRangeMsg
   * @brief
   * 处理传感器数据
   *
   * @param[in] range_data-传感器数据
   * */
  bool processRangeMsg(CostmapRangeData &range_data);

  /**
   * processRangeMsg
   * @brief
   * 处理传感器数据
   *
   * @param[in] range_data-传感器数据
   * */
  bool processFixedRangeMsg(CostmapRangeData &range_data);

  /**
   * processRangeMsg
   * @brief
   * 处理传感器数据
   * 1.前/后向超声，大于1米的障碍检测，视为无效，设置为最大距离，直接清除
   * 2.侧向向超声，大于0.5米的障碍检测，视为无效，设置为最大距离，直接清除
   * @param[in] range_data-传感器数据
   * */
  bool processVariableRangeMsg(CostmapRangeData &range_data);

  /**
   * gamma
   * @brief
   * 计算gamma
   *
   * @param[in] theta-角度
   * @return 输出gamma角度
   * */
  inline double gamma(const double &theta);

  /**
   * delta
   * @brief
   * delta
   *
   * @param[in] phi-角度
   * @return 输出delta角度
   * */
  inline double delta(const double &phi);

  /**
   * sensorModel
   * @brief
   * 超声置信度模型
   *
   * @param[in] r-观测数据
   * @param[in] phi-更新位置的徑向距离
   * @param[in] theta-更新位置的角度值
   *
   * @return 观测r下，(phi, theta)对应cell的置信度
   * */
  double sensorModel(const double &r, const double &phi, const double &theta);

  /**
   * getDeltas
   * @brief
   * 超声置信度模型
   *
   * @param[in] angle-角度
   * @param[in] dx-x
   * @param[in] dy-y
   *
   * @return 观测r下，(phi, theta)对应cell的置信度
   * */
  void getDeltas(const double &angle, double &dx, double &dy);

  /**
   * updateCell
   * @brief
   * 更新栅格地图
   *
   * @param[in] wp_pose-姿态
   * @param[in] r-距离
   * @param[in] wp_point-世界地图点
   * @param[in] clear-是否清除
   *
   * */
  void updateCell(const WorldmapPose &wp_pose, const double &r,
                  const WorldmapPoint &wp_point, const bool &clear);
  /**
   * toProb
   * @brief
   * costmap数据转化成double用于计算概率
   *
   * @param[in] c-costmap值
   * @return 对应的double值
   * */
  inline double toProb(const unsigned char &c) {
    return static_cast<double>(c) / CVTE_BABOT::LETHAL_OBSTACLE;
  }

  bool handleRangeData(const std::shared_ptr<CostmapRangeData> &ptr_range_data,
                       std::vector<bool> &obstacle_area);
  /**
   * toCost
   * @brief
   * 概率值转化成costmap数据值
   *
   * @param[in] p-概率值
   * @return 对应的costmap数据值
   * */
  unsigned char toCost(const double &p) {
    return static_cast<unsigned char>(p * CVTE_BABOT::LETHAL_OBSTACLE);
  }

  inline float area(const int &x1, const int &y1, const int &x2, const int &y2,
                    const int &x3, const int &y3) {
    return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
  }

  inline int orient2d(const int &Ax, const int &Ay, const int &Bx,
                      const int &By, const int &Cx, const int &Cy) {
    return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
  }

  static inline double normalizeAngle(const double &angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  bool clear_on_max_reading_;
  double inflate_cone_;  /// 圆锥参数
  double max_angle_;
  double phi_v_;  ///< 超声传感模型的一个参数，计算delta, is a predefined
                  ///< distance from the sensor where a smooth transition occurs
                  ///< from certainty to uncertainty

  std::mutex range_data_mutex_;
  double clear_threshold_;  ///<  清除阈值
  double mark_threshold_;   ///< 标志阈值

  std::vector<std::string> range_data_buffer_names_;  ///<传感器名的缓存

  CostmapBound layer_bound_;  ///< 层边界
  std::function<bool(CostmapRangeData &range_data)> processRangeDataFunc_;

  double max_obs_range_{1.0};  ///< 最大用于添加障碍物的有效距离

  uint8_t clear_cost_;  ///< cost 小于该值将会被清除
  uint8_t mark_cost_;  ///< cost 大于该值将会被认为是障碍物更新到mater层

  boost::shared_array<time_t> ptr_obs_time_;  ///< 记录障碍物添加时刻
  double max_obs_keep_duration_{20.0};        ///< 障碍物最久保留时间
};
}  // namespace CVTE_BABOT

#endif