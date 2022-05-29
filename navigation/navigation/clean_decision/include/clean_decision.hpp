/*
 * @Author: your name
 * @Date: 2021-09-28 16:35:50
 * @LastEditTime: 2021-10-19 16:29:37
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/clean_decision/include/clean_decision.hpp
 */
#ifndef CLEAN_DECISION_HPP_
#define CLEAN_DECISION_HPP_
#include "clean_config.hpp"
#include <memory>
#include <vector>
#include <string>
#include "data_base/obstacle_map.hpp"
#include "fcp_planner/fcp_path.hpp"
#include "eigen3/Eigen/Core"
#include "path.hpp"
#include "robot_info.hpp"
using namespace full_coverage_planner;
namespace CVTE_BABOT {
typedef Eigen::Vector2d Vectex;
typedef std::vector<Vectex> Area;
typedef std::vector<Area> MutilArea;
class ObstacleMap;
class CleanDecision {
 public:
  CleanDecision();
  CleanDecision(const CleanConfig &config);
  CleanDecision(const CleanDecision &obj) = delete;
  CleanDecision &operator=(const CleanDecision &obj) = delete;
  /**
   * @brief 更新已清扫区域
   *
   * @param[] cur_pose 机器人当前位置
   */
  void updateAlreadyCleanArea(const Eigen::Vector3d &cur_pose);
  /**
   * @brief 设置清扫地图
   *
   * @param[] map_path - 清扫地图路径
   * @return true - 读取地图成功
   * @return false - 读取地图失败
   */
  bool setCleanMap(const std::string &map_path);
  /**
   * @brief 保存已清扫区域地图
   *
   * @param[] map_path - 保存地图路径
   * @return true - 保存成功
   * @return false - 保存失败
   */
  bool saveAlreadyCleanMap(const std::string &map_path);
  /**
   * @brief 补扫规划
   *
   * @param[] map_path - 地图路径
   * @param[] multi_area - 清扫区域
   * @param[] path - 返回路径
   * @return true - 补扫规划成功
   * @return false - 补扫规划失败
   */
  bool AdditionalCoverPlan(const std::string &map_path,
                           const MutilArea &multi_area, SubPath &path);
  /**
   * @brief 重置地图和标志位
   *
   */
  void reset();
  /**
   * @brief 设置任务类型
   *
   * @param[] type -
   */
  inline void setMissionType(ActionType type) { mission_type_ = type; }

  inline void setConfig(const CleanConfig &config) { config_ = config; }

  inline void setMultiArea(const MutilArea &multi_area) {
    multi_area_ = multi_area;
  }

 private:
  CleanConfig config_;
  ActionType mission_type_;
  Eigen::Vector3d last_pose_;
  bool init_pose_ = false;
  bool init_map_ = false;
  std::shared_ptr<ObstacleMap> ptr_obstacle_map_;
  std::shared_ptr<ObstacleMap> ptr_clean_map_;
  std::vector<std::vector<bool>> need_clean_area_;
  std::vector<std::vector<bool>> already_clean_area_;
  std::vector<Eigen::Vector2d> clean_polygon_;
  std::vector<Eigen::Vector2d> mark_area_;

  MutilArea multi_area_;

 private:
  /**
   * @brief 计算机器人清扫框内点
   *
   */
  void calcuMarkArea();
  /**
   * @brief 提取区域内所有点
   *
   * @param[] area -
   * @return std::vector<GridmapPoint> -
   */
  std::vector<GridmapPoint> getAreaPoint(const Area &area) const;
  /**
   * @brief 广度优先搜索聚类
   *
   * @param[] multi_area_points - 各个清扫区域点
   * @param[] cluster_area_points - 未清扫区域聚类点
   */
  void clusterUncleanArea(
      const std::vector<std::vector<GridmapPoint>> &multi_area_points,
      std::vector<std::vector<GridmapPoint>> &cluster_area_points);
  /**
   * @brief 产生全覆盖路径
   *
   * @param bons_areas
   * @return true
   * @return false
   */
  bool generaCoverPath(const std::vector<std::vector<AreaVertex>> &bons_areas,
                       const std::string &map_path, SubPath &path) const;

  /**
   * @brief 获取最小包围矩形
   *
   * @param area
   * @param mini_rectangle 端点集合
   * @param mini_area 矩形面积
   */
  void minimumEncloseRect(const std::vector<GridmapPoint> &area,
                          std::vector<GridmapPoint> &mini_rectangle,
                          int &mini_area) const;
};

}  // namespace CVTE_BABOT

#endif