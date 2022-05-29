/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file perception_map.hpp
 *
 *@brief 感知地图类, 保存栅格地图及动态障碍信息
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.3
 *@data 2020-04-15
 ************************************************************************/
#ifndef __PERCEPTION_MAP_HPP
#define __PERCEPTION_MAP_HPP
#include <boost/smart_ptr.hpp>
#include <memory>

#include <opencv2/opencv.hpp>

#include "costmap_2d.hpp"
#include "processor_utils.hpp"
namespace CVTE_BABOT {
// 常量
const unsigned int MAX_ID_NUM =
    100;  ///< 提前分配内存存储有效的id, id号超出的不处理
const unsigned int BUFFER_SIZE =
    1000;  ///< 提前分配内存缓存每个id对应的栅格数量, 避免内存频繁分配

// 栅格状态
struct GridStatus {
  unsigned int value = 0;  ///< 栅格的值,普通障碍,动态障碍与膨胀等有各自的值
  unsigned int id = 0;             ///< 栅格对应障碍的id
  unsigned int obstacle_type = 0;  ///< 障碍类型
};

/**
 * PerceptionMap
 * @brief
 * 1.感知地图类, 保存栅格地图, 用于判断地图上的障碍类型
 * 2.保存动态障碍(行人,汽车)信息, 用于判断是否需要避让
 * 3.需要传入外部数据更新地图
 **/
class PerceptionMap {
 public:
  PerceptionMap(const unsigned int& ui_x, const unsigned int& ui_y);
  ~PerceptionMap() = default;

  PerceptionMap(const PerceptionMap&);
  PerceptionMap& operator=(const PerceptionMap&);

  // 栅格
  /**
   *getGridStatus
   *@brief
   *获取栅格状态
   *
   *@param[in] d_x-世界坐标x
   *@param[in] d_y-世界坐标y
   *@param[out] grid_status-栅格状态
   *@return true-成功, false-该点超出地图范围
  **/
  bool getGridStatus(const double& d_x, const double& d_y,
                     GridStatus& grid_status) const;

  // id
  /**
   *getObject
   *@brief
   *获取id对应的动态障碍
   *
   *@param[in] id-id
   *@param[out] dynamic_object-对应的动态障碍
   *@return true-获取成功, false-该id不存在
  **/
  bool getObject(const unsigned int& id, DynamicObject& dynamic_object);

  /**
   *getCostmapResetCorGrids
   *@brief
   *返回一张当前的costmap,上面重置了id对应所有栅格的值.如果id不存在,则返回当前costmap
   *
   *@param[in] id-id
   *@return 清除了相应栅格的costmap
  **/
  std::shared_ptr<Costmap2d> getCostmapResetCorGrids(
      const unsigned int& id) const;

  // 更新地图
  /**
   *updateCostmap
   *@brief
   *更新costmap
   *
   *@param[in] ptr_costmap-costmap
  **/
  void updateCostmap(const std::shared_ptr<const Costmap2d>& ptr_costmap) {
    std::lock_guard<std::mutex> lock(mutex_);
    ptr_costmap_ = ptr_costmap;
  }

  /**
   *updateDynamicObject
   *@brief
   *更新动态障碍
   *
   *@param[in] ptr_dynamic_objects-动态障碍
  **/
  void updateDynamicObject(
      const std::shared_ptr<const DynamicObstacles>& ptr_dynamic_objects) {
    std::lock_guard<std::mutex> lock(mutex_);
    ptr_dynamic_objects_ = ptr_dynamic_objects;
  }

  void updateData();

  /**
   *updateMap
   *@brief
   *更新感知地图, 确保已经传了costmap和动态障碍信息
   *
  **/
  void updateMap();

  cv::Mat getVisMat() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return perception_mat_.clone();
  }

 private:
  /**
   *copyPerceptionMap
   *@brief
   *拷贝所有变量, 必须复制所有变量,
   *不然没被显著拷贝的变量会被赋默认值而不会被拷贝
   *
   *@param[in] in-
   *@param[out] out-
   *@return true-, false-
  **/
  void copyPerceptionMap(const PerceptionMap& perception_map);

  // 转换坐标
  /**
  *worldToMap
  *@brief
  *转换世界坐标到地图坐标
  *
  *@param[in] d_x-世界坐标x
  *@param[in] d_y-世界坐标y
  *@param[out] ui_x-地图坐标x
  *@param[out] ui_y-地图坐标y
  *@return true-转换成功, false-坐标超出地图范围
 **/
  bool worldToMap(const double& d_x, const double& d_y, unsigned int& ui_x,
                  unsigned int& ui_y) const;

  /**
   *getIndex
   *@brief
   *获取点在数据数组中的索引
   *
   *@param[in] ui_mx-地图坐标x
   *@param[in] ui_my-地图坐标y
   *@return 点在数据数组中的索引
  **/
  inline unsigned int getIndex(const unsigned int& ui_mx,
                               const unsigned int& ui_my) const {
    return ptr_costmap_->getIndex(ui_mx, ui_my);
  }

  // 栅格

  /**
   *getGridStatus
   *@brief
   *获取栅格状态,必须确保传入的值在地图范围内.
   *
   *@param[in] ui_mx-地图坐标x
   *@param[in] ui_my-地图坐标y
   *@return 对应的栅格状态
  **/
  inline GridStatus getGrid(const unsigned int& ui_mx,
                            const unsigned int& ui_my) const {
    return ptr_grid_data_[getIndex(ui_mx, ui_my)];
  }

  /**
   *setGridStatus
   *@brief
   *设置栅格状态,必须确保传入的点在地图范围内.
   *
   *@param[in] ui_mx-地图坐标x
   *@param[in] ui_my-地图坐标y
   *@param[in] target_status-想设置的栅格状态
  **/
  inline void setGrid(const unsigned int& ui_mx, const unsigned int& ui_my,
                      const GridStatus& target_status) {
    ptr_grid_data_[getIndex(ui_mx, ui_my)] = target_status;
  }

  /**
   *setGridStatus
   *@brief
   *设置栅格状态, 如果该点超出地图或对应栅格已存在更高级的障碍,设置失败
   *
   *@param[in] d_x-世界坐标x
   *@param[in] d_y-世界坐标y
   *@param[out] target_status-想设置的栅格状态
   *@return true-成功, false-超出地图或已存在更高级的障碍
  **/
  bool setGridStatus(const double& d_x, const double& d_y,
                     const GridStatus& target_status);

  // 对象信息
  /**
   *updateDynamicGrids
   *@brief
   *更新动态障碍对应的栅格
   *
   *@param[in] dynamic_object-动态障碍
  **/
  void updateDynamicGrids(const DynamicObject& dynamic_object);

  /**
   *updateCorrespondGrids
   *@brief
   *更新记录每个id对应的所有栅格索引
   *
  **/
  void updateCorrespondGrids();

  /**
   *getCorrespondGrids
   *@brief
   *获取该id对象所包含的所有栅格
   *
   *@param[in] id-id
   *@return 该id对象所包含的所有栅格索引
  **/
  std::vector<unsigned int> getCorrespondGrids(const unsigned int& id) const;

  /**
   *resetGridData
   *@brief
   *重置栅格地图,每次更新地图前调用
   *
  **/
  void resetGridData();

  /**
   *inflateMap
   *@brief
   *对栅格点膨胀,使区域连续
   *
   *@param[in] ui_inflate_grid_radius-膨胀的栅格半径
  **/
  void inflateMap(const unsigned int& ui_inflate_grid_radius);

  void updateVisMat();

  mutable std::mutex mutex_;  ///< 设置为可变,函数内可加锁

  // 动态障碍
  std::vector<std::vector<unsigned int>>
      vv_ui_correspond_grids_;  ///<记录每个id对应的所有栅格索引,第一层vector存储所有id,第二层存储每个id对应的栅格

  std::shared_ptr<const DynamicObstacles> ptr_dynamic_objects_ =
      nullptr;  ///< 存储动态障碍

  //　栅格
  boost::shared_array<GridStatus> ptr_grid_data_ =
      nullptr;  ///< 存储栅格状态的数据

  std::shared_ptr<const Costmap2d> ptr_costmap_ = nullptr;  ///< costmap
  unsigned int ui_size_x_ = 100;  ///< 地图x大小, 与costmap匹配
  unsigned int ui_size_y_ = 100;  ///< 地图y大小, 与costmap匹配

  cv::Mat perception_mat_;
};

}  // namespace CVTE_BABOT
#endif
