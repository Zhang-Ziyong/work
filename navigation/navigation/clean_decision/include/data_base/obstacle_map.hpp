/*
 * @Author: your name
 * @Date: 2021-10-20 16:02:34
 * @LastEditTime: 2021-10-21 14:58:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/clean_decision/include/obstacle_map.hpp
 */
#ifndef _OBSTACLE_MAP_HPP_
#define _OBSTACLE_MAP_HPP_

#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include "eigen3/Eigen/Core"
namespace CVTE_BABOT {

typedef Eigen::Vector2d GlobalmapPoint;
typedef Eigen::Matrix<unsigned int, 2, 1> GridmapPoint;

class ObstacleMap {
 public:
  ObstacleMap();
  ObstacleMap(const ObstacleMap &map);
  ObstacleMap(unsigned int ui_size_x, unsigned int ui_size_y,
              double d_resolution, double d_origin_x, double d_origin_y,
              unsigned char uc_default_value);
  ObstacleMap &operator=(const ObstacleMap &map);

  /**
   *readMap
   *@brief
   *从给定路径读取地图
   *
   *@param[in] map_path 地图路径
   * */
  bool readMap(const std::string &map_path);

  /**
   *saveMap
   *@brief
   *保存地图
   *
   *@param[in] map_path 地图路径
   * */
  bool saveMap(const std::string &map_path);

  /**
   *initMaps
   *@brief
   *初始化map,根据大小
   *
   *@param[in] ui_size_x-x轴大小
   *@param[in] ui_size_y-y轴大小
   * */
  void initMaps(unsigned int ui_size_x, unsigned int ui_size_y);

  /**
   *resizeMap
   *@brief
   *重新设置map大小、分辨率、原点
   *
   *@param[in] ui_size_x-x轴大小
   *@param[in] ui_size_y-y轴大小
   *@param[in] d_resolution-分辨率
   *@param[in] d_origin_x-原点x轴坐标
   *@param[in] d_origin_y-原点y轴坐标
   * */
  void resizeMap(unsigned int ui_size_x, unsigned int ui_size_y,
                 double d_resolution, double d_origin_x, double d_origin_y);

  /**
   *resetMaps
   *@brief
   *将map重新赋值默认代价
   *
   * */
  void resetMaps();

  /**
   *mapToWorld
   *@brief
   *将Map2d的栅格坐标转化成世界地图坐标
   *
   *@param[in] cp_point-栅格x，y坐标
   *@param[out] wm_point-世界地图x，y坐标
   *
   * */
  void mapToWorld(const GridmapPoint &cp_point, GlobalmapPoint &wm_point) const;

  /**
   *worldToMap
   *@brief
   *将世界地图坐标转化成costmap的栅格坐标(有边界限制)
   *
   *@param[in] wm_point-世界地图x，y坐标
   *@param[out] cp_point-栅格x，y坐标
   *@return true-表示栅格坐标在map2d的范围里面，false-表示超出了costmap范围
   * */
  bool worldToMap(const GlobalmapPoint &wm_point, GridmapPoint &cp_point) const;
  /**
   *worldToMapNoBounds
   *@brief
   *将世界地图坐标转化成costmap的栅格坐标(没有边界限制)
   *
   *@param[in] d_wx-世界地图x坐标
   *@param[in] d_wx-世界地图y坐标
   *@param[out] ui_mx-栅格x坐标
   *@param[out] ui_my-栅格y坐标
   * */
  void worldToMapNoBounds(const GlobalmapPoint &wm_point,
                          GridmapPoint &cp_point) const;

  /**
   *worldToMapEnforceBounds
   *@brief
   *将世界地图坐标转化成costmap的栅格坐标(强制转换到边界)
   *
   *@param[in] d_wx-世界地图x坐标
   *@param[in] d_wx-世界地图y坐标
   *@param[out] ui_mx-栅格x坐标
   *@param[out] ui_my-栅格y坐标
   * */
  void worldToMapEnforceBounds(const GlobalmapPoint &wm_point,
                               GridmapPoint &cp_point) const;

  /**
   *getMapPointValue
   *@brief
   *获取栅格地图点栅格值
   *
   *@param[in] cp_point-栅格地图点
   *@param[out] 栅格值
   **/
  inline unsigned char getMapPointValue(const GridmapPoint &cp_point) const {
    // std::lock_guard<std::recursive_mutex> lock(map_mutex_);
    return obstacle_map_[getIndex(cp_point(0), cp_point(1))];
  }
  /**
   *getWorldPointValue
   *@brief
   *获取世界地图点栅格值
   *
   *@param[in] wm_point-世界地图点
   *@param[out] 栅格值
   **/
  inline unsigned char getWorldPointValue(
      const GlobalmapPoint &wm_point) const {
    GridmapPoint cp_point;
    if (worldToMap(wm_point, cp_point)) {
      return getMapPointValue(cp_point);
    } else {
      return 254;
    }
  }

  /**
   *getIndex
   *@brief
   *已知栅格地图的坐标，计算在一维数组的下标
   *
   *@param[in] ui_mx-x坐标
   *@param[in] ui_my-y坐标
   *@return 在一维数组的下标
   * */
  inline unsigned int getIndex(unsigned int ui_mx, unsigned int ui_my) const {
    return ui_my * ui_size_x_ + ui_mx;
  }
  /**
   *indexToCells
   *@brief
   *已知在一位数组的下标，计算costmap的栅格坐标
   *
   *@param[in] ui_index-一维数组的下标
   *@param[out] ui_mx-x坐标
   *@param[out] ui_my-y坐标
   * */
  inline void indexToCells(unsigned int ui_index, unsigned int &ui_mx,
                           unsigned int &ui_my) const {
    ui_my = ui_index / ui_size_x_;
    ui_mx = ui_index - (ui_my * ui_size_x_);
  }

  /**
   *setDefaultValue
   *@brief
   *设置costmap的默认代价值
   *
   *@param[in] uc_c-默认代价值
   * */
  inline void setDefaultValue(unsigned char uc_c) { uc_default_value_ = uc_c; }
  /**
   *setThreshHlod
   *@brief
   *设置占据栅格阈值
   *
   *@param[in] occupied_thresh-栅格阈值
   * */
  inline void setOccupyThresh(unsigned char occupied_thresh) {
    occupied_thresh_ = occupied_thresh;
  }
  /**
   *setFreeThresh
   *@brief
   *设置空闲栅格阈值
   *
   *@param[in] occupied_thresh-栅格阈值
   * */
  inline void setFreeThresh(unsigned char free_thresh) {
    free_thresh_ = free_thresh;
  }

  inline unsigned char getOccupyThresh() { return occupied_thresh_; }

  inline unsigned char getFreeThresh() { return free_thresh_; }

  /**
   *setMapPointValue
   *@brief
   *设置栅格地图栅格值
   *
   *@param[in] cp_point-栅格地图点
   *@param[in] value-栅格值
   * */
  inline bool setMapPointValue(const GridmapPoint &cp_point,
                               unsigned char value) {
    // std::lock_guard<std::recursive_mutex> lock(map_mutex_);
    unsigned int index = getIndex(cp_point[0], cp_point[1]);
    if (index > obstacle_map_.size() - 1) {
      return false;
    } else {
      obstacle_map_[index] = value;
      return true;
    }
  }
  /**
   *setWorldPointValue
   *@brief
   *设置世界地图栅格值
   *
   *@param[in] wm_point-世界地图点
   *@param[in] value-栅格值
   * */
  inline bool setWorldPointValue(const GlobalmapPoint &wm_point,
                                 unsigned char value) {
    GridmapPoint map_point;
    if (!worldToMap(wm_point, map_point)) {
      return false;
    } else {
      return setMapPointValue(map_point, value);
    }
  }
  /**
   *isWorldPointOccupy
   *@brief
   *世界点是否被占据
   *
   *@param[in] wm_point-世界地图点
   *@param[out] true: 被占据或者未知　false: 空闲
   * */
  inline bool isWorldPointOccupy(const GlobalmapPoint &wm_point) {
    if (getWorldPointValue(wm_point) > free_thresh_) {
      return true;
    } else {
      return false;
    }
  }
  /**
   *isMapPointOccupy
   *@brief
   *世界点是否被占据
   *
   *@param[in] wm_point-栅格地图点
   *@param[out] true: 被占据或者未知　false: 空闲
   * */
  inline bool isMapPointOccupy(const GridmapPoint &cp_point) {
    if (getMapPointValue(cp_point) > free_thresh_) {
      return true;
    } else {
      return false;
    }
  }

  inline double getResolution() const { return d_resolution_; }
  inline unsigned int getSizeInCellsX() const { return ui_size_x_; }
  inline unsigned int getSizeInCellsY() const { return ui_size_y_; }
  inline double getOriginX() const { return d_origin_x_; }
  inline double getOriginY() const { return d_origin_y_; }
  inline std::vector<unsigned char> getCharMap() const { return obstacle_map_; }

 private:
  double d_origin_x_ = 0.0;  ///< Map2d的起点，不是中点，在全局地图的坐标
  double d_origin_y_ = 0.0;  ///< Map2d的起点，不是中点，在全局地图的坐标
  double d_resolution_ = 0.0;             ///< 地图分辨率
  unsigned int ui_size_x_ = 0;            ///< 地图X方向宽度
  unsigned int ui_size_y_ = 0;            ///< 地图Y方向宽度
  unsigned char uc_default_value_ = 0.0;  ///< 默认灰度值
  unsigned char thresh_hold_ = 0;         ///< 占据阈值
  unsigned char free_thresh_ = 0;
  unsigned char occupied_thresh_ = 0;
  std::string map_filename_;
  std::vector<unsigned char> obstacle_map_;
  std::recursive_mutex map_mutex_;
};
}  // namespace CVTE_BABOT
#endif