/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file system.hpp
 *
 *@brief
 * 系統类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@modify Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/
#ifndef SYSTEM_HPP_
#define SYSTEM_HPP_

#include <glog/logging.h>

#include <atomic>
#include <deque>
#include <list>
#include <string>
#include <thread>

#include "map/map_manager.hpp"
// #include "frontend/feature_extract_factory.hpp"
#include "common/config/system_config.hpp"
#include "common/data_struct/keyframe.hpp"
#include "common/data_struct/sensors_type.hpp"
#include "common/debug_tools/tic_toc.h"
#include "common/gps/LocalCartesian.hpp"
#include "frontend/base_feature_extractor.hpp"
#include "loopend/loop_track.hpp"
#include "lidar_odom/lidar_odom.hpp"
#include "map_track/map_track.hpp"
#include "msf/multi_sensors_fusion.hpp"
#include "occupancy_map/new_occupancy_map.hpp"
#include "occupancy_map/depth_camera_occ_map.hpp"
#include "occupancy_map/local_occupancy_map.hpp"

namespace cvte_lidar_slam {
enum INIT_MODEL { NOTINIT = 0, RELIABLE_POSE, FIX_POSE, GPS, LIDAR };

/**
 * System
 * @brief 系统类
 **/
class System {
 public:
  ~System();

  static System *getInstance();

  void setConfig(const std::shared_ptr<SystemConfig> ptr_config);

  /**
   * setInitModel
   * @brief 设置初始化模式
   * @param[in] init_model-初始化模式
   **/
  void setInitModel(const INIT_MODEL init_model);

  /**
   * setInitPose
   * @brief 设置初始化模式
   * @param[in] init_pose-初始化姿态
   **/
  void setInitPose(const Mat34d &init_pose);

  void setReliablePose(const Mat34d &reliable_pose);

  bool setInitKeyFrameId(const unsigned int id);

  bool setInitKeyFrameIdList(const std::list<unsigned int> ids);

  void setCloudMapPath(const std::string &map_path) {
    temp_cloud_filepath_ = map_path;
  }

  std::string getCloudMapPath() { return temp_cloud_filepath_; }

  bool setExtrincs(const Mat34d &T_lo) {
    if (!T_lo.hasNaN()) {
      T_lo_ = T_lo;
      return true;
    }
    return false;
  }

  /**
   * InitParameters
   * @brief 系统参数初始化
   *
   **/
  void InitParameters();

  /**
   * addOdom
   * @brief 添加odom数据
   * @param[in] odom-odom数据
   **/
  void addOdom(const OdomMeasure &odom);
  void addOdom(const Mat34d &pose, const double time_stamp);

  void addIMU(const ImuMeasure &IMU_t);
  void addIMU(const Vec3d &acc, const Vec3d &gyro, const double time_stamp);
  /**
   * addCloudFrame
   * @brief 跟踪帧
   * @param[in] cloud_in-输入点云
   * @param[in] time_stamp-时间戳
   **/
  void addCloudFrame(const CloudMeasure &cloud);
  void addCloudFrame(const laserCloud::Ptr cloud_in, const double time_stamp);

  void syncFrame(const laserCloud::Ptr cloud_in, const double time_stamp);

  /**
   * creatFrame
   * @brief 生成待处理帧
   **/
  bool creatFrame();
  /**
   * failueDetect
   * @brief 失效检测
   **/
  bool failueDetect();

  /**
   * getPoseCov
   * @brief 获取pose协方差
   **/
  double getPoseCov();

  /**
   * Reset
   * @brief 重置
   **/
  void Reset();

  /**
   * shutdown
   * @brief 关闭系统
   **/
  void shutdown();

  /**
   * saveTrajectory
   * @brief 保存轨迹
   **/
  void saveTrajectory();

  /**
   * saveMap
   * @brief 保存地图
   **/
  void saveMap();

  /**
   * saveMap
   * @brief 保存地图
   * @param[in]: map_dir-保存3D地图路径
   **/
  void saveMap(const std::string &map_dir);

  /**
   * loadMap
   * @brief 载入地图
   **/
  bool loadMap();

  /**
   * loadMap
   * @brief 载入地图
   * @param[in]: map_dir-读取
   * 3D地图路径
   **/
  bool loadMap(const std::string &map_dir);

  bool loadWholeData(const std::string &map_dir);

  /**
   * getViewCloud
   * @brief 跟踪帧
   * @param[in] view_cloud-可视化点云
   * @param[in] frame_pose-关键帧位置
   * @return true-成功，false-失败
   **/
  bool getViewCloud(laserCloud::Ptr &view_cloud, laserCloud::Ptr &frame_pose);

  /**
   * getWholeMap
   * @brief 获取地图
   **/
  laserCloud::Ptr getWholeMap();

  laserCloud::Ptr getCurrSurroundCloud();

  laserCloud::Ptr getCurrKeyPoses();

  UserOccupancyGrid getCurrOccMap();

  UserOccupancyGrid getCurrDepthOccMap();

  const RobotState &getRobotState();
  const RobotState getLidarOdomState();
  const Mat34d getKeyFramePose();
  const Mat34d getKeyFrameLidarOdomPose();

  /**
   *getLatestFrameID
   *@brief
   *获取最近的id
   *
   **/
  long unsigned int getLatestFrameID();

  bool hasKeyFrame();
  bool isNewKeyFrame();

  double getCurKeyFrameTimeStamp();

  /**
   *getKeyFrameDataPose
   *@brief
   *获取keyFrame的位姿列表
   *@return  std::vector<Mat34d>
   **/
  std::vector<std::pair<long unsigned int, Mat34d>> getKeyFrameDataPose();

  std::shared_ptr<std::list<LoopConstrain>> getLoopInfo();

  /**
   *getPoseFromID
   *@brief
   *由frame id获取位姿
   *@return  Mat34d
   **/
  bool getPoseFromID(const long unsigned int frame_id, Mat34d &pose34d);
  /**
   *isLocalizationInit
   *@brief
   *定位是否初始化成功
   *@return  bool
   **/
  bool isLocalizationInit();

  bool isLocalizationSuccess();

  void requestStop();

  /**
   *isMapUpdate
   *@brief
   *获取优化完成标志
   *
   **/
  bool isMapUpdate() {
    const bool flag = is_opt_finished_;
    is_opt_finished_ = false;
    return flag;
  }

  void setStopRequest(const bool &flag) { is_stop_requested_ = flag; }

 public:
  // 地图和可视化信息
  double curr_keyframe_time_ = 0.0;  ///< 关键帧时间
  double last_keyframe_time_ = 0.0;  ///< 上一关键帧时间
  Mat34d curr_keyframe_pose_;
  Mat34d last_keyframe_pose_;
  laserCloud::Ptr cloud_in_;           ///< 输入点云指针
  laserCloud::Ptr view_cloud_;         ///< 可视化点云指针
  laserCloud::Ptr corner_cloud_;       ///< 全局点云指针
  laserCloud::Ptr surf_cloud_;         ///< 全局点云指针
  laserCloud::Ptr corner_cloud_top_;   ///< 全局点云指针
  laserCloud::Ptr surf_cloud_top_;     ///< 全局点云指针
  laserCloud::Ptr raw_cloud_;          ///< 全局点云指针
  laserCloud::Ptr global_cloud_;       ///< 全局点云指针
  laserCloud::Ptr cloud_for_occ_map_;  ///< 用于occ map的点云

  std::vector<std::pair<PointType, PointType>> curr_loop_pos_;

  // 获取定位状态
  bool getLocalizationStatus();

 private:
  /**
   * System
   * @brief 系统类
   * @param[in] config_file-参数指针
   **/
  System();

  void lidarOdomThread();
  void dynamicLocalMapThread();

  /**
   * mapTrackThread
   * @brief 地图跟踪线程
   **/
  void mapTrackThread();

  /**
   * mapTrackThread
   * @brief occupancyMap线程
   **/
  void occupancyMapThread();

  /**
   * loopTrackThread
   * @brief 闭环检测线程
   **/
  void loopTrackThread();

  void initLocalizationThread();

  bool initLocalizationByLidarFeature(const laserCloud::Ptr cloud_in,
                                      const FramePosition &pose,
                                      Vec3d &init_predict_pose,
                                      double &init_predict_yaw);

  void saveScanThread(const std::string &map_2d_path);
  void saveKeyFrameThread(const std::string &map_3d_path);
  void saveMapThread(const std::string &map_2d_path);

  bool init_gps_;
  Vec3d gps_origin_;
  Mat34d gps_trans_;
  GeographicLib::LocalCartesian geo_converter_;

  laserCloud::Ptr frame_pose_;  ///< 关键帧pose

  std::string temp_cloud_filepath_;  ///< 点云临时地图

  //历史信息记录

  Mat34d last_map_to_odom_;           ///< 上一地图转换
  Mat34d cur_map_to_odom_;            ///< 当前地图转换
  Mat34d last_lidar_odom_to_odom_;    ///< 当前地图转换
  Mat34d lidar_odom_to_odom_;         ///< 当前地图转换
  Mat34d last_frame_pose_;            ///< 上一地图转换
  Mat34d cur_frame_pose_;             ///< 当前地图转换
  Mat34d T_lo_;                       ///< laser to odom
  Mat3d r_il_, r_li_;                 ///< laser to odom
  Mat34d T_il_, T_li_;                ///< laser to odom
  Mat34d curr_correspond_odom_pose_;  ///<
  Mat34d last_correspond_odom_pose_;  ///<

  RobotState cur_robot_state_;       ///< 当前机器人状态
  RobotState cur_lidar_odom_state_;  ///< 当前机器人状态

  ImuMeasure IMU_cur_;
  ImuMeasure IMU_last_;
  Mid_Filter accx_MF, accy_MF, accz_MF;
  Mid_Filter gyrx_MF, gyry_MF, gyrz_MF;
  // spatial imu
  Eigen::Vector3d ba0_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  // Eigen::Vector3d bg0_ = Eigen::Vector3d(-0.0018, 0.0018, 0.00023);
  Eigen::Vector3d bg0_ = Eigen::Vector3d(0.0, 0.0, 0.0);

  // 系统数据容器
  std::deque<OdomMeasure> d_odom_measures_;  ///< 轮速里程计测量队列容器
  std::deque<OdomMeasure> d_odom_for_lidar_odom_;  ///< 轮速里程计测量队列容器
  std::deque<OdomMeasure> d_lidar_odom_;  ///< 激光里程计测量队列容器
  std::deque<GPSMeasure> d_gps_measures_;      ///< GPS测量队列容器
  std::deque<ImuMeasure> d_imu_measures_;      ///< Imu测量队列容器
  std::deque<CloudMeasure> d_cloud_measures_;  ///< 点晕测量队列容器
  std::deque<CloudMeasure> d_cloud_for_odom_;  ///< 点晕测量队列容器

  // 容器数据锁
  std::mutex odom_data_mutex_;   ///< odom  数据锁
  std::mutex gps_data_mutex_;    ///< gps   数据锁
  std::mutex imu_data_mutex_;    ///< imu   数据锁
  std::mutex cloud_data_mutex_;  ///< cloud 数据锁
  std::mutex keyframe_mutex_;    ///< 关键帧锁>
  std::mutex occ_save_mutex_;    ///< 保存占用地图锁

  // 系统子模块
  std::shared_ptr<SystemConfig> ptr_config_ = nullptr;  ///< 系统参数配置指针
  std::shared_ptr<MapManager> ptr_map_manager_ = nullptr;  ///< 地图管理模块指针
  std::shared_ptr<LidarOdom> ptr_lidar_odom_ = nullptr;  ///< 地图跟踪模块指针
  std::shared_ptr<MapTrack> ptr_map_track_ = nullptr;  ///< 地图跟踪模块指针
  std::shared_ptr<BaseFeatureExtractor> ptr_feature_extractor_ =
      nullptr;  ///< 特征提取模块指针
  std::shared_ptr<LoopTrack> ptr_loop_track_ = nullptr;  ///< 闭环检测模块指针
  std::shared_ptr<MutliSensorsFusion> ptr_msf_ =
      nullptr;  ///< 多传感器融合模块指针
  std::shared_ptr<OccMap> ptr_occ_map_ = nullptr;  ///< 栅格地图指针
  std::shared_ptr<DepthOccupancyMap> ptr_depth_occ_map_ =
      nullptr;  // 深度地图指针
  std::shared_ptr<LocalOccMap> ptr_local_occ_map_ = nullptr;
  std::shared_ptr<KeyFrame> ptr_cur_keyframe_ = nullptr;  ///< 当前帧指针
  std::shared_ptr<KeyFrame> ptr_cur_lidar_odom_keyframe_ =
      nullptr;  ///< 当前帧指针

  // 系统状态和标志位
  bool is_first_cloud_ = true;     ///< 第一个lidar数据标志
  bool is_first_odom_ = true;      ///< 第一个里程计数据标志
  bool is_first_keyframe_ = true;  ///< 第一帧点云数据
  bool new_keyframe_flag_ = false;
  bool is_local_map_ready_ = false;     ///< 地图初始化状态标志
  bool is_feature_data_ready_ = false;  ///< 特征数据初始化成功标志
  std::atomic<bool> is_opt_finished_;   ///< 优化完成标志位
  std::atomic<bool> is_localization_init_;
  bool is_match_ok_ = false;
  bool has_new_keyframe_;                  ///< 是否有新的关键帧
  bool process_new_keyframe_finish_flag_;  ///< 是否有新的关键帧
  bool is_update_map_;
  size_t id_;           ///< 帧id
  size_t last_msf_id_;  ///< 上一次执行位姿图优化的帧ID；
  size_t lost_frame_count_;
  std::atomic<bool> run_opt_;            ///< 多传感器运行标志
  std::atomic<bool> is_reset_;           ///< Reset flag
  std::atomic<bool> is_stopped_;         ///< 是否停止
  std::atomic<bool> is_stop_requested_;  ///<>
  double temp_frame_time_ = 0.;          ///< 当前帧时间
  double last_frame_time_ = 0.;          ///< 上一帧时间
  double last_undis_time_ = 0.;          ///< 上一帧时间
  size_t odom_count_ = 0;
  size_t imu_count_ = 0;
  size_t lidar_count_ = 0;
  std::chrono::system_clock::time_point last_opt_finish_time_;

  bool init_first_IMU_rotation_flag_ = false;

  bool USE_IMU = false;
  bool print_debug = true;

  unsigned int loop_count_ = 0;  ///< loop数据计数

  // 子模块多线程
  std::thread *ptr_lidarOdom_thread_;  ///< 地图跟踪线程指针
  std::thread *ptr_maptrack_thread_;   ///< 地图跟踪线程指针
  std::thread *ptr_looptrack_thread_;  ///< 闭环检测线程指针
  std::thread *ptr_occ_map_thread_;    ///< 动态地图线程指针
  std::thread *ptr_msf_thread_;        ///< 多传感器融合线程指针
  std::thread *ptr_init_localization_thread_;  ///< 定位初始化指针
  std::thread *ptr_map_update_thread_;         ///< 地图更新
  std::thread *ptr_dynamic_local_map_thread_;

  std::condition_variable msf_cv_;           ///< 多传感器优化条件变量
  std::condition_variable loop_tracker_cv_;  ///< 闭环检测条件变量
  std::condition_variable map_tracker_cv_;   ///< 地图跟踪条件变量

  INIT_MODEL init_mode_;
  Mat34d init_pose_;

  pclDownsampler downsize_filter_lidarOdom_;

  UserOccupancyGrid global_occ_map_;

  // 判断定位成功标志位
  bool is_localization_sucess_ = false;
  bool is_init_localization_sucess_ = false;

  std::thread save_keyframe_thread_;
  std::thread save_map_thread_;
  std::thread save_scandata_thread_;

  bool finish_save_keyframe_ = false;
  bool finish_save_map_ = false;
  bool finish_save_scandata_ = false;

 private:
  static System *ptr_system_;
  std::shared_ptr<SlamStateMachine> ptr_state_machine_;

};  // end of class

typedef std::shared_ptr<System> SystemPtr;

}  // namespace cvte_lidar_slam

#endif  // SYSTEM_HPP_