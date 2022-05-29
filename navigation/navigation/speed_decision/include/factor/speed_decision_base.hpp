/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file speed_devision_base.hpp
 *
 *@brief 速度决策基类
 *
 *@author linyanlong
 *@modified
 *@version
 *@data
 ************************************************************************/
#ifndef SPEED_DECISION_BASE_HPP_
#define SPEED_DECISION_BASE_HPP_
#include <chrono>
#include <map>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include "costmap_2d.hpp"
#include "costmap_utils.hpp"
#include "costmap_cloud.hpp"
#include "eigen3/Eigen/Core"
#include "marker_map.hpp"
#include "path.hpp"
#include "planner_utils.hpp"
#include "speed_decision_config.hpp"
#include "stop_shape.hpp"
namespace CVTE_BABOT {

class SpeedDecisionBase {
 public:
  /**
   * @brief Set the Instance object
   *
   * @param[in] ptr 传入单例指针
   */
  static void setInstance(const std::shared_ptr<SpeedDecisionBase> &ptr);

  /**
   * @brief Get the Instance object
   *
   * @return std::shared_ptr<SpeedDecisionBase> 获取单例
   */
  static std::shared_ptr<SpeedDecisionBase> getInstance();

  /**
   * @brief Set the Config object
   *
   * @param[in] config
   */
  void setConfig();

  /**
   * @brief 传感器输入
   *
   * @param[in] scan  激光雷达
   * @param[in] attitude  倾角
   * @param[in] range_points  超声波数据
   * @param[in] range_points  红外数据
   */
  void inputScan(const std::vector<Eigen::Vector2d> &scan);
  void inputAttitude(const Eigen::Matrix<double, 3, 3> &attitude);
  void inputSonar(const std::vector<Eigen::Vector2d> &range_points);
  void inputInfrared(const std::vector<Eigen::Vector2d> &range_points);

  /**
   * @brief 设置最大速度
   *
   * @param[in] 最大速度
   */
  void setMissionManagerSpeedValue(double value);
  /**
   * @brief 设置减速带轮廓
   *
   * @param[in] json_file
   */
  void setMissionMangerStripMark(const Json::Value &json_file);
  /**
   * @brief 获取速度相关的东西
   *
   * @return double 最大速度
   */
  double getMissionManagerSpeedValue();

  /**
   * @brief 设置任务速度等级
   * @brief 设置本地速度等级
   * @brief 设置轮廓区域速度等级
   *
   * @param speed_radio
   */
  void setMissionManagerSpeedRadio(const SpeedRadio &speed_radio);
  void setLocalizationSpeedRadio(const SpeedRadio &speed_radio);
  void setMarkSpeedRadio(const SpeedRadio &speed_radio);

  /**
   * @brief 获取速度决策结果
   *
   * @return SpeedDecisionResult 结果
   */
  SpeedDecisionResult getSpeedResult();
  /**
   * @brief 获取限幅速度
   *
   * @param controller_vel
   * @return Velocity
   */
  Velocity getActualVel(const Velocity &controller_vel);

  /**
   * @brief 设置参考路径
   *
   * @param[in] refer_path 参考路径
   * @param[in] refer_index 当前索引
   */
  void setReferPath(std::shared_ptr<SubPath> refer_path);
  void setReferIndex(size_t refer_index);

  /**
   * @brief 设置代价地图
   * @brief 设置全局代价地图(当前该地图只有静态层和膨胀层)
   * @param[in] ptr_costmap_2d
   */
  void setCostmap(std::shared_ptr<Costmap2d> ptr_costmap_2d);
  void setGlobalCostmap(std::shared_ptr<Costmap2d> ptr_costmap_2d);

  /**
   * @brief 速度决策主线程
   *
   */
  void updateVel();

  /**
   * @brief 获取脱困因子
   *
   * @param[in] dir 需要查询的方向 可多个方向同时查询
   * @param[in] T_wb_ptr 变换矩阵 可以外面配置
   * @return double 因子
   */
  virtual double getRecoverFactor(
      const RecoveDir dir_key, Eigen::Matrix<double, 2, 3> *T_wb_ptr = nullptr);

  /**
   * @brief Get the Recover Factor object
   *
   * @param dir_key
   * @param T_wb_ptr
   * @return double
   */
  double getArcRecoverFactor(const RecoveDir dir_key,
                             Eigen::Matrix<double, 2, 3> *T_wb_ptr = nullptr);

  /**
   * @brief 检查各个方向是否存在障碍物
   *
   * @return true
   * @return false
   */
  bool virtual checkFrontDanger();

  /**
   * @brief 获取停障框最小外包矩形
   *
   * @param[in] shape_name  停障框名称:"stop" "recover" "danger"
   * @return std::vector<double>
   */
  std::vector<Eigen::Vector2d> getBoundingbox(const std::string shape_name);

  /**
   * @brief 判断是否脱困成功
   *
   * @return true  脱困成功
   * @return false
   */
  bool virtual checkOutofRecovery();

 protected:
  /**
   * @brief 构造与析构
   *
   */
  SpeedDecisionBase();
  SpeedDecisionBase(const SpeedDecisionBase &obj) = delete;
  SpeedDecisionBase &operator=(const SpeedDecisionBase &obj) = delete;

  /**
   * @brief 更新传感器时间
   *
   */
  void updateScanTime();
  void updateAttitudeTime();
  std::chrono::duration<double> calcuScanTimeDiff();
  std::chrono::duration<double> calcuAttitudeTimeDiff();

  /**
   * @brief 停障判断
   * @return true   发生停障
   * @return false
   */
  virtual bool isObstacleInStopBound();

  /**
   * @brief 获取速度因子
   * @param CurSpeed  当前速度因子 (选最小)
   * @param SlowBound 减速框
   * @param PathSlow  路径被占用
   * @param Marker    地图标记 TODO: 后面全部删掉
   * @param CurSpeed 当前速度因子setConfig
   *
   * @return double
   */
  virtual double getCurSpeedFactor();  // 获取速度因子
  double getPathSlowFactor();          // 路径
  double getSlowBoundFactor();         // 减速框
  double getMarkerFactor();            // 地图标记
  double getCameraFactor();            // 相机点云
  double getInfraredObstacleFactor();  // 避障红外
  double getSlopeFactor();             // 斜坡
  double getElevatorFactor();          // 电梯

  /**
   * @brief 获取贴边标志
   *
   * @return true 当前为贴边路径
   * @return false
   */
  bool getEdgeFlag();  // 获取是否在沿边状态

  /**
   * @brief 获取 costmap 的代价值
   *
   * @param[in] point 坐标
   * @return int 代价值
   */
  int checkPointCostValue(const Eigen::Vector2d &point);

  /**
   * @brief 角度换算
   *
   * @param[in] R 旋转矩阵
   * @return Eigen::Vector3d
   */
  Eigen::Vector3d rotation2rpy(const Eigen::Matrix3d &R);

  /**
   * @brief 速度平滑
   *
   * @param[in] vel 当前速度
   * @return Velocity 平滑后的速度
   */
  Velocity smoothVel(const Velocity &vel);

  /**
   * @brief 速度限幅
   *
   * @param[in] vel 需要检查的速度
   * @param[out] vel 检查完的速度
   */
  void checkMinVel(Velocity &vel);

  /**
   * @brief 停障框配置
   *
   * @param[in] address   需要配置的地址
   * @param[in] v_shape   配置参数
   */
  void setRobotShapeConfig(StopShape &address,
                           const std::vector<StopShapeConfig> &v_shape,
                           double resolution = 1);

  /**
   * @brief Get the Danger Shape object
   *
   * @param[in] shape_name 需要查询的框的名称
   * @return true
   * @return false
   */
  bool getDangerShape(const std::string &shape_name);

  // 配置及单例指针
  bool config_flag_ = false;  // 已配置的标志
  std::shared_ptr<SpeedDecisionBaseConfig> ptr_config_ = nullptr;  // 配置项目
  static std::shared_ptr<SpeedDecisionBase> ptr_speed_decision_;  // 单例指针

  // 决策结果
  SpeedDecisionResult speed_deci_result_;

  // 代价地图
  std::shared_ptr<Costmap2d> ptr_costmap_2d_;
  std::shared_ptr<Costmap2d> ptr_global_costmap_2d_;  // 全局代价地图
  std::shared_ptr<MarkerMap> ptr_marker_map_{nullptr};

  // 传感器数据
  std::vector<Eigen::Vector2d> scan_;
  std::vector<Eigen::Vector2d> sonar_sensor_data_;
  std::vector<Eigen::Vector2d> ir_sensor_data_;
  Eigen::Matrix<double, 3, 3> robot_attitude_;
  // 传感器时间戳
  std::chrono::time_point<std::chrono::system_clock> last_scan_time_;
  std::chrono::time_point<std::chrono::system_clock> last_attitude_time_;

  // 路径
  std::shared_ptr<SubPath> refer_path_;
  size_t refer_index_ = 0;

  // 速度等级 及 对应的减速百分比
  SpeedRadio mission_manager_speed_radio_;  // 任务速度等级
  SpeedRadio localization_speed_radio_;     // local速度等级
  SpeedRadio stop_bound_speed_radio_;  // 停障速度等级 (只有全速与停障)
  SpeedRadio mark_speed_radio_;        // 地图标记速度等级
  std::map<SpeedRadio, double> speed_radio2double_;

  // 上一次的速度
  Velocity last_vel_;

  // 贴边标志
  bool edge_flag_ = false;

  // 停障框
  StopShape stop_shape_;     // 停障框检测
  StopShape slow_shape_;     // 减速框
  StopShape recover_shape_;  // 停障恢复框(需要障碍物脱离该区域)
  StopShape danger_shape_;   // 致命框
  StopShape arc_recover_shape_;                          // 弧线脱困框
  std::map<RecoveDir, std::string> m_recover_coverage_;  // 图层映射

  // 锁
  std::mutex scan_mutex_;
  std::mutex path_mutex_;
  std::mutex sonar_data_mutex_;
  std::mutex ir_data_mutex_;
  std::mutex camera_mutex_;
  std::mutex scan_time_mutex_;
  std::mutex attitude_time_mutex_;
  std::mutex speed_deci_result_mutex_;
  std::mutex costmap_mutex_;
  std::mutex attitude_mutex_;
};
}  // namespace CVTE_BABOT

#endif
