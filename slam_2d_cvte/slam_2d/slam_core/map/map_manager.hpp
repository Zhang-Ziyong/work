/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file map_manager.hpp
 *
 *@brief
 * 地图管理类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@modify Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/
#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

#include <deque>
#include <list>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "common/debug_tools/debug_color.h"
#include "common/data_struct/pc_base.hpp"
#include "common/math_base/slam_math.hpp"
#include "common/config/system_config.hpp"
#include "common/data_struct/sensors_type.hpp"
#include "state_machine/slam_state_machine.hpp"

namespace cvte_lidar_slam {
struct FramsDis {
  size_t frame_id;
  double distance;
};

class KeyFrame;

class CloudDownsample;

class KeyFrameInfo;

/**
 * MapManager
 * @brief 地图管理类
 **/
class MapManager {
 public:
  ~MapManager();
  /**
   * MapManager
   * @brief 地图管理类
   * @param[in] config-参数指针
   **/
  static std::shared_ptr<MapManager> getInstance();
  static std::shared_ptr<MapManager> getNewMapInstance();

  /**
   * initParameters
   * @brief 重置
   **/
  void initParameters(const BackendConfig &config);

  /**
   * Reset
   * @brief 重置
   **/
  void Reset();

  /**
   *saveKeyFrame
   *@brief
   *保存关键帧
   *
   *@param[in] file-保存地址
   *@return true-成功，false-失败
   **/
  bool saveKeyFrame(const std::string &file, const std::string &temp_file = "");

  bool saveAllKeyFrame(const std::string &file,
                       const std::string &temp_file = "");

  bool saveNewCornerCloudMap(const laserCloud::Ptr all_corner_cloud);

  bool saveNewAddedCornerMap(const laserCloud::Ptr new_added_corner);

  bool saveDeletedCornerMap(const laserCloud::Ptr deleted_corner);

  /**
   *loadKeyFrame
   *@brief
   *载入关键帧
   *
   *@param[in] file-载入地址
   *@return true-成功，false-失败
   **/
  bool loadKeyFrame(const std::string &file);

  bool loadKeyFrameInfo(const std::string &file);

  /**
   *loadLocalKeyFrame
   *@brief
   *载入局部关键帧
   *
   *@param[in] file-载入地址
   *@param[in] pos-载入位置
   *@return true-成功，false-失败
   **/
  bool loadLocalKeyFrame(const std::string &file, FramePosition pos);

  /**
   *loadLocalKeyFrame
   *@brief
   *载入剩余关键帧
   *
   *@param[in] file-载入地址
   *@return true-成功，false-失败
   **/
  bool loadReservedKeyFrame(const std::string &file);

  /**
   *addBufferToMap
   *@brief
   *添加剩余关键帧
   *
   *@return true-成功，false-失败
   **/
  bool addBufferToMap();

  /**
   *Add
   *@brief
   *添加关键帧到数据库
   *
   *@param[in] ptr_keyframe-关键帧指针
   *@return true-成功，false-失败
   **/
  void Add(std::shared_ptr<KeyFrame> ptr_keyframe);
  void AddNewMap(std::shared_ptr<KeyFrame> ptr_keyframe);

  /**
   *Update
   *@brief
   *更新数据库
   *
   **/
  void Update();

  /**
   *Clear
   *@brief
   *清空数据库
   *
   **/
  void Clear();

  /**
   *getGlobalCloud
   *@brief
   *获取全部点云
   *
   *@return laserCloud::Ptr
   **/
  laserCloud::Ptr getGlobalCloud();

  /**
   *getSurroundCloud
   *@brief
   *获取周围点云
   *@param[in] pos-搜索位置
   *@param[in] radius-搜索半径
   *@return laserCloud::Ptr
   **/
  laserCloud::Ptr getSurroundCloud(const FramePosition &pos,
                                   const double radius);

  /**
   *getKeyPoses
   *@brief
   *获取全部关键帧位置
   *
   *@return laserCloud::Ptr
   **/
  laserCloud::Ptr getKeyPoses();

  /**
   *getCornerMapCloud
   *@brief
   *获取全部线点点云
   *
   *@return laserCloud::Ptr
   **/
  laserCloud::Ptr getCornerMapCloud();

  /**
   *getSurfMapCloud
   *@brief
   *获取全部面点点云
   *
   *@return laserCloud::Ptr
   **/
  laserCloud::Ptr getSurfMapCloud();

  /**
   *DetectCandidatesIdByHorizontalDistance
   *@brief
   *获取周围水平距离内候选帧
   *@param[in] pos-搜索位置
   *@param[in] radius-搜索半径
   *@return std::vector<size_t>
   **/
  std::vector<size_t> DetectCandidatesIdByHorizontalDistance(
      const FramePosition &pos, const double radius);

  /**
   *DetectCandidatesByHorizontalDistance
   *@brief
   *获取周围水平距离内候选帧
   *@param[in] pos-搜索位置
   *@param[in] radius-搜索半径
   *@return std::vector<std::shared_ptr<KeyFrame>>
   **/
  std::vector<std::shared_ptr<KeyFrame>> DetectCandidatesByHorizontalDistance(
      const FramePosition &pos, const double radius);

  /**
   *DetectCandidatesByDistance
   *@brief
   *获取周围距离内候选帧
   *@param[in] pos-搜索位置
   *@param[in] radius-搜索半径
   *@return std::vector<std::shared_ptr<KeyFrame>>
   **/
  std::vector<std::shared_ptr<KeyFrame>> DetectCandidatesByDistance(
      const FramePosition &pos, const double radius);
  std::vector<std::shared_ptr<KeyFrame>> DetectCandidatesByDistanceNewMap(
      const FramePosition &pos, const double radius);

  /**
   *DetectCandidatesByTime
   *@brief
   *获取最近N个候选帧
   *@param[in] number-数量
   *@return std::vector<std::shared_ptr<KeyFrame>>
   **/
  std::vector<std::shared_ptr<KeyFrame>> DetectCandidatesByTime(
      const unsigned int number);

  /**
   *DetectCandidatesByIndex
   *@brief
   *获取某一帧左右个N个候选帧
   *@param[in] number-数量
   *@param[in] index-索引
   *@return std::vector<std::shared_ptr<KeyFrame>>
   **/
  std::vector<std::shared_ptr<KeyFrame>> DetectCandidatesByIndex(
      const int index, const int number);

  std::vector<std::shared_ptr<KeyFrame>> getAllKeyFrames();

  /**
   *getLatestFrameID
   *@brief
   *获取最近的id
   **/
  int getLatestFrameID();

  bool hasKeyFrame();

  bool activeKeyFrame(const std::string file_name,
                      std::shared_ptr<KeyFrame> ptr_keyframe);

  bool unactiveKeyFrame(const std::string file_name,
                        std::shared_ptr<KeyFrame> ptr_keyframe);

  std::shared_ptr<KeyFrame> getKeyFrameFromID(const size_t index);

  /**
   *getKeyFrameDataPose
   *@brief
   *获取keyFrame的位姿列表
   *@return  std::vector<Mat34d>
   **/
  std::vector<std::pair<size_t, Mat34d>> getKeyFrameDataPose();

  std::map<size_t, std::shared_ptr<KeyFrame>> getKeyFrameDatabase() {
    return keyframe_database_;
  }

  /**
   *getPoseFromID
   *@brief
   *由frame id获取位姿
   *@return  Mat34d
   **/
  bool getPoseFromID(const size_t frame_id, Mat34d &pose34d);

  bool getOdomPoseFromID(const size_t frame_id, Mat34d &pose34d);

  bool getPositionFromId(const size_t frame_id, Vec3d &pos);

  static bool compFrameByDis(const FramsDis &a, const FramsDis &b) {
    return a.distance < b.distance;
  }

  std::shared_ptr<KeyFrame> GetNearestKeyFrame(const FramePosition &pos);

  std::vector<std::shared_ptr<KeyFrame>> GetNearKeyFrames(
      const FramePosition &pos, const double radius);

 private:
  MapManager();

  static std::shared_ptr<MapManager> ptr_instance_;
  static std::shared_ptr<MapManager> ptr_instance_new_map_;

  static std::mutex instance_mutex_;
  static std::mutex instance_new_map_mutex_;

  std::list<std::shared_ptr<KeyFrame>> l_ptr_keyframe_;  /// < 关键帧链表容器
  std::list<std::shared_ptr<KeyFrame>>
      l_ptr_new_map_keyframe_;  /// < 关键帧链表容器

  std::map<size_t, std::shared_ptr<KeyFrame>>
      keyframe_database_;  ///< 关键帧数据库
  std::map<size_t, std::shared_ptr<KeyFrame>>
      new_map_keyframe_database_;  ///< 关键帧数据库

  std::unordered_map<size_t, KeyFrameInfo> um_keyframes_info_;

  size_t keyframe_id_;  ///< 关键帧id

  laserCloud::Ptr frame_pose_tree_;          ///< 关键帧pose构成的点云
  laserCloud::Ptr frame_pose_tree_new_map_;  ///< 关键帧pose构成的点云
  laserCloud::Ptr surround_frame_;           ///< 周围关键帧
  laserCloud::Ptr surround_frame_new_map_;   ///< 周围关键帧
  laserCloud::Ptr all_surf_cloud_;           ///< 全局面点点云
  laserCloud::Ptr all_corner_cloud_;         ///< 全局线点点云
  laserCloud::Ptr ds_surround_frame_;        ///< 下采样的周围关键帧
  laserCloud::Ptr ds_global_frame_;          ///< 下采样的全局关键帧
  laserCloud::Ptr reserved_global_frame_;    ///< 待载入的全局关键帧
  pclKdTree::Ptr ptr_kdtree_frame_;          ///< frame的kdtree搜索结构
  pclKdTree::Ptr ptr_kdtree_frame_new_map_;  ///< frame的kdtree搜索结构

  pclDownsampler ds_surround_frame_pose_;     ///< 附近关键帧下采样
  pclDownsampler global_frame_pose_;          ///< 全局关键帧下采样
  pclDownsampler global_frame_pose_for_map_;  ///< 全局地图关键帧下采样
  pclDownsampler global_cloud_ds_;            ///< 全局点云下采样
  pclDownsampler global_raw_cloud_ds_;        ///< 全局点云下采样
  std::shared_ptr<CloudDownsample> ptr_cloud_downsampler_;  ///< 全局点云下采样
  std::shared_ptr<CloudDownsample>
      ptr_surroundcloud_downsampler_;  ///< 附近点云下采样
  std::shared_ptr<CloudDownsample>
      ptr_viewcloud_downsampler_;  ///< 可视化点云下采样
  unsigned int latest_frame_id_;   ///< 最近关键帧id

  BackendConfig config_;  ///< 后端参数指针

  // mutex
  std::mutex mutex_;               ///< 数据锁
  std::mutex recent_frame_mutex_;  ///< 关键帧锁

  std::shared_ptr<SlamStateMachine> ptr_state_machine_;

  std::string curr_map_path_;

};  // end of class

}  // namespace cvte_lidar_slam

#endif  // MAP_MANAGER_HPP_