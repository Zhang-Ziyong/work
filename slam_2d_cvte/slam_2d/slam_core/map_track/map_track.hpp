/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 201９, CVTE.
 * All rights reserved.
 *
 *@file map_track.hpp
 *
 *@brief
 * 1.后端MAP_TRACK类
 *
 *@author Wei Zhang(zhangwei@cvte.com)
 *@modify Yun Su(robosu12@gmail.com)
 *@version 0.5
 *@data 2022-03-16
 ************************************************************************/

#ifndef MAP_TRACK_HPP_
#define MAP_TRACK_HPP_

#include "opencv2/opencv.hpp"

#include <atomic>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>

#include "common/math_base/slam_math.hpp"
#include "common/data_struct/pc_base.hpp"
#include "state_machine/slam_state_machine.hpp"

namespace cvte_lidar_slam {
struct BackendConfig;
class KeyFrame;
class MapManager;
class HuberLoss;
class PoseSolver;

class MapTrack {
 public:
  /**
   * MapTrack
   * @brief 地图跟踪类
   * @param[in] config-参数指针
   **/
  explicit MapTrack(BackendConfig *config);

  ~MapTrack();

  /**
   * Reset
   * @brief 重置
   **/
  void Reset();

  /**
   * InitParameters
   * @brief 系统参数初始化
   *
   **/
  void InitParameters();

  /**
   * requestStop
   * @brief 请求停止
   *
   **/
  void requestStop() {
    is_stop_requested_ = true;
    cv_.notify_one();
  }

  /**
   * isStopRequested
   * @brief 是否有请求停止
   * return true-成功
   *
   **/
  bool isStopRequested() { return is_stop_requested_; }

  /**
   * requestReset
   * @brief 请求重置
   *
   **/
  void requestReset();

  /**
   * Stop
   * @brief 停止标志
   * return true-成功
   *
   **/
  bool Stop() {
    if (is_stop_requested_) {
      is_stopped_ = true;
      return true;
    }
    return false;
  }

  /**
   * isStopped
   * @brief 是否停止标志
   * return true-成功
   *
   **/
  bool isStopped() { return is_stopped_; }

  /**
   * isMapReady
   * @brief 地图是否准备好
   * return true-成功
   *
   **/
  bool isMapReady() { return is_map_ready_; }

  /**
   * setMapStatus
   * @brief 地图是否准备好
   * param[in] flag-地图状态标志
   *
   **/
  void setMapStatus(bool flag) { is_map_ready_ = flag; }

  /**
   * stopRequested
   * @brief 是否有请求停止标志
   * return true-成功
   *
   **/
  bool stopRequested() { return is_stop_requested_; }

  /**
   * setInitPose
   * @brief 设置初始位置
   * param[in] init pose-初始pose
   * param[in] init_odom-初始里程计
   *
   **/
  void setInitPose(const Mat34d &init_pose, const Mat34d &init_odom);

  /**
   * relocalization
   * @brief 重定位成功标志
   * return true-成功
   *
   **/
  bool relocalization();

  /**
   * acceptKeyFrames
   * @brief 查看列表中是否有等待被插入的关键帧
   * @return 如果存在，返回true
   */
  bool acceptKeyFrames() { return is_accept_keyframes_; }

  /**
   * setAcceptKeyFrames
   * @brief 设置关键帧插入标志
   * param[in] flag-插入允许标志
   */
  void setAcceptKeyFrames(bool flag) { is_accept_keyframes_ = flag; }

  /**
   * checkNewKeyFrames
   * @brief 查看列表中是否有等待被插入的关键帧
   * @return true-存在
   */
  bool checkNewKeyFrames();

  /**
   * getSurroundingCornerCloud
   * @brief 获取局部地图线点点云指针
   * return laserCloud::Ptr
   */
  laserCloud::Ptr getSurroundingCornerCloud() {
    return corner_cloud_from_map_copy_;
  }

  /**
   * getSurroundingSurfCloud
   * @brief 获取局部地图面点点云指针
   * return laserCloud::Ptr
   */
  laserCloud::Ptr getSurroundingSurfCloud() {
    return surf_cloud_from_map_copy_;
  }

  /**
   * processNewKeyFrame
   * @brief 处理列表中的关键帧
   */
  bool processNewKeyFrame(std::shared_ptr<KeyFrame> ptr_cur_keyframe,
                          bool is_update_map);

  /**
   * getSurroundingKeyFrames
   * @brief 获取附近的关键帧
   * return true 表示获取成功
   */
  bool getSurroundingKeyFrames();

  bool FeatureMatchAndFusionOptimization();

  /**
   * Optimization
   * @brief LM优化求解
   * @param[in] iter_count-迭代次数
   */
  bool Optimization(int iter_count);

  /**
   * scanToLocalMap
   * @brief 优化求解的封装函数
   */
  void scanToLocalMap();

  void PCLICPMatch();

  bool CeresICPMatch();

  bool CeresPLICPMatch();

  /**
   * dowmSampleCurFrame
   * @brief 下采样当前帧点云
   * return true 表示采样正常，点太少就会失败
   */
  bool dowmSampleCurFrame();

  /**
   * Run
   * @brief MapTrack的主流程
   */
  void Run();

  /**
   * buildKdtree
   * @brief 定位模式下构建kdtree
   */
  bool buildKdtree();

  /**
   * insertKeyFrame
   * @brief tracking线程向此线程插入关键帧
   * @param[in] ptr_keyframe-帧指针
   */
  void insertKeyFrame(std::shared_ptr<KeyFrame> ptr_keyframe);

  pcl::PointCloud<PointType>::Ptr getCurrSurroundMap();

  /**
   *getMap2Odom
   *@brief
   *获取map到odom的变换
   *
   *@return Mat34d
   **/
  Mat34d getMap2Odom();

  /**
   *setMap2Odom
   *@brief
   *设置map到odom的变换
   *
   *@param[in] map_to_odom－map到odom的变换
   **/
  void setMap2Odom(const Mat34d &map_to_odom, bool is_update_map);

  /**
   *getPoseCov
   *@brief
   *获取pose的协方差
   *
   **/
  inline double getPoseCov() const { return pose_cov_; }

  /**
   *clearCloud
   *@brief
   *清除点云
   *
   **/
  void clearCloud();  ///< TODO:一定要记得赋值前要，清除原来的点云

  /**
   *isKeyFrame
   *@brief
   *当前帧到上个关键帧的几何距离
   *
   **/
  bool isKeyFrame(bool use_rotation = false);

  /**
   *odomDistance2LastFrame
   *@brief
   *根据里程计相对上一帧的运动量
   *
   **/
  bool isJunkFrame(const double time, const Mat34d &cur_odom_pose);

  /**
   *updateTransform
   *@brief
   *更新pose信息
   *
   **/
  void updateTransform();

  /**
   *setPoseCov
   *@brief
   *设置pose的协方差
   *
   **/
  void setPoseCov(const double cov);

  size_t getmatchFailedCount() { return match_failed_count_; }

  size_t getLowOverlapCount() { return low_overlap_count_; }

  void updateMapPointProbability();

  laserCloud::Ptr getCornerMapCloud();

 private:
  std::shared_ptr<MapManager> ptr_map_manager_ = nullptr;  ///< 数据容器指针

  std::shared_ptr<KeyFrame> ptr_cur_keyframe_ = nullptr;   ///< 当前帧指针
  std::shared_ptr<KeyFrame> ptr_last_keyframe_ = nullptr;  ///< 当前帧指针

  BackendConfig *config_;  ///< 后端参数配置指针

  pclKdTree::Ptr ptr_kdtree_surf_from_map_;
  ///< localmap平面点云的kdtree搜索结构
  pclKdTree::Ptr ptr_kdtree_corner_from_map_;
  ///< localmap线点云的kdtree搜索结构
  pclKdTree::Ptr ptr_kdtree_corner_from_keyframe_;
  ///< localmap线点云的kdtree搜索结构
  pclKdTree::Ptr ptr_kdtree_new_corner_map_;
  ///< localmap线点云的kdtree搜索结构

  laserCloud::Ptr corner_cloud_from_map_;  ///< localmap线点点云指针
  laserCloud::Ptr surf_cloud_from_map_;    ///< localmap面点点云指针
  laserCloud::Ptr ds_corner_cloud_from_map_;  ///< localmap降采样线点点云指针
  laserCloud::Ptr ds_surf_cloud_from_map_;  ///< localmap降采样面点点云指针
  laserCloud::Ptr ds_curr_corner_cloud_;  ///< 当前帧降采样线点点云指针
  laserCloud::Ptr ds_curr_surf_cloud_;  ///< 当前帧降采样面点点云指针
  laserCloud::Ptr ds_curr_outlier_cloud_;  ///< 当前帧降采样外点点云指针
  laserCloud::Ptr
      corner_cloud_from_map_copy_;  ///< localmap降采样线点点云副本指针
  laserCloud::Ptr
      surf_cloud_from_map_copy_;  ///< localmap降采样面点点云副本指针

  laserCloud::Ptr ds_last_corner_cloud_;  ///< 当前帧降采样线点点云指针
  laserCloud::Ptr ds_last_surf_cloud_;  ///< 当前帧降采样面点点云指针
  laserCloud::Ptr ds_last_outlier_cloud_;  ///< 当前帧降采样外点点云指针

  laserCloud::Ptr ds_cur_corner_cloud_copy_;  ///< 当前帧降采样线点点云指针
  laserCloud::Ptr ds_cur_surf_cloud_copy_;  ///< 当前帧降采样面点点云指针

  laserCloud::Ptr new_corner_cloud_from_map_;       ///<
  laserCloud::Ptr observed_corner_cloud_from_map_;  ///<

  // eigen based lm opt
  std::shared_ptr<HuberLoss> loss_function_ = nullptr;
  std::shared_ptr<PoseSolver> pose_solver_ = nullptr;

  long unsigned int id_;       ///< 关键帧索引
  size_t match_failed_count_;  ///< 配准失败计数
  double overlap_ratio_;
  size_t low_overlap_count_;  ///< 低重叠度计数
  size_t keyframe_count_;
  float curr_distance_error_;       ///< 误差距离
  float last_distance_error_;       ///< 误差距离
  double pose_cov_;                 ///< pose协方差
  double predict_error_ratio_;      ///< 预测误差率
  double last_frame_time_;          ///< 上一帧时间戳
  Mat6d cov_;                       ///< 协防差
  Mat34d last_keyframe_pose_;       ///< 上个关键帧的pose
  Mat34d last_frame_pose_;          ///< 上帧的pose
  Mat34d last_frame_odom_pose_;     ///< 上个帧里程计的pose
  Mat34d last_keyframe_odom_pose_;  ///< 上个关键帧里程计的pose
  Mat34d cur_frame_pose_;           ///< 当前关键帧的pose
  Mat34d map_to_odom_;              // TODO: 加锁
  Mat34d last_map_to_odom_;         // TODO: 加锁
  double match_cov_;
  double moved_distance_;
  double static_duration_;

  std::atomic<bool> is_abortBA_;           ///< 打断标志
  std::atomic<bool> is_stopped_;           ///< 结束标志
  std::atomic<bool> is_stop_requested_;    ///< 结束请求标志
  std::atomic<bool> is_not_stop_;          ///< 结束标志
  std::atomic<bool> is_accept_keyframes_;  ///< 关键帧接受标志位
  std::atomic<bool> is_map_ready_;         ///< 地图是否准备好标志
  std::atomic<bool> is_local_map_ready_;  ///< 局部地图是否准备好标志
  std::atomic<bool> is_global_map_ready_;  ///< 全局地图是否准备好标志

  bool is_set_init_pose_;       ///< 是否设置初始位置
  bool is_first_keyframe_;      ///< 第一个关键帧
  bool is_match_ok_;            ///<匹配是否成功
  bool is_last_match_ok_;       ///<匹配是否成功
  bool is_kdtree_initialized_;  ///< 是否构建成功kdtree
  bool map_odom_update_flag_;

  std::mutex new_keyframe_mutex_;       ///< 关键帧的锁
  std::mutex stop_mutex_;               ///< 停止标志位锁
  std::mutex accept_mutex_;             ///< 接收标志锁
  std::mutex pose_mutex_;               ///< pose锁
  std::mutex correct_pose_mutex_;       ///< pose锁
  std::mutex cloud_mutex_;              ///< 点云锁
  std::mutex curr_surround_map_mutex_;  ///< pose锁

  std::condition_variable cv_;  ///< 条件变量

  pclDownsampler downsize_filter_corner_;
  ///< 下采样当前帧corner点云
  pclDownsampler downsize_filter_surf_;
  ///< 下采样当前帧surf点云
  pclDownsampler downsize_filter_cur_corner_;
  ///< 下采样局部地图corner点云
  pclDownsampler downsize_filter_cur_surf_;
  ///< 下采样局部地图surf点云

  int ds_corner_from_map_num_;  ///< 下采样局部地图corner点云数量
  int ds_surf_from_map_num_;    ///< 下采样局部地图surf点云数量
  int curr_corner_cloud_num_;   ///< 下采样当前帧corner点云数量
  int curr_surf_cloud_num_;     ///< 下采样当前帧surf点云数量

  int last_corner_cloud_num_;  ///< 下采样当前帧corner点云数量
  int last_surf_cloud_num_;    ///< 下采样当前帧surf点云数量

  std::shared_ptr<SlamStateMachine> ptr_state_machine_;

  std::ofstream frame_pose_result_;
  std::ofstream Lodom_pose_result_;
  std::ofstream wheel_pose_result_;

  std::set<size_t> observed_keyframe_set_;

};  // end of class

}  // namespace cvte_lidar_slam

#endif  // MAP_TRACK_HPP_