#ifndef LOOP_TRACK_HPP_
#define LOOP_TRACK_HPP_
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <list>
#include <memory>

#include "common/data_struct/pc_base.hpp"
#include "common/math_base/slam_math.hpp"
#include "common/config/system_config.hpp"

namespace cvte_lidar_slam {

class KeyFrame;
class MapManager;
class MutliSensorsFusion;
class LoamRegistration;

class LoopTrack {
 public:
  explicit LoopTrack(LoopConfig *config);
  ~LoopTrack();
  LoopTrack(const LoopTrack &obj) = delete;
  LoopTrack &operator=(const LoopTrack &obj) = delete;

  /**
   * Reset
   * @brief 重置
   **/
  void Reset();

  /**
   *@brief 插入关键帧
   *
   *@param ptr_keyframe
   */

  void insertKeyFrame(std::shared_ptr<KeyFrame> ptr_keyframe);

  std::shared_ptr<KeyFrame> getCurKeyFramePtr() const {
    return ptr_cur_keyframe_;
  }
  /**
   *@brief 请求重启
   *
   */
  void requestReset() {}
  /**
   *@brief 请求停止
   *
   */
  void requestStop() {
    is_request_stop_ = true;
    stop_icp_ = true;
    is_stop_ = true;
  }
  /**
   *@brief 停止主函数
   *
   */

  void stop() {
    if (is_request_stop_) {
      is_stop_ = true;
    }
  }

  /**
   *@brief 检查是否有新的关键帧
   *
   *@return true
   *@return false
   */

  bool checkNewKeyFrames();
  /**
   *@brief 检查是否有回环
   *
   *@return true
   *@return false
   */

  bool detectLoop(const Mat34d &);

  /**
   *@brief 检查回环候选帧
   *
   *@return id
   *@return false
   */
  long int getLoopCandidateID(const Mat34d &);
  /**
   *@brief 计算回环约束
   *
   *@return true
   *@return false
   */
  bool correctLoop(const size_t history_frame_id, double &cov, const Mat34d &,
                   Mat34d &, size_t last_msf_id);
  /**
   *@brief
   *
   */
  void resetIfRequested();
  /**
   *@brief 程序是否结束
   *
   *@return true
   *@return false
   */
  bool checkFinish();
  /**
   *@brief 结束程序
   *
   */
  void setFinish();

  /**
   *@brief 重定位接口
   *
   *@param vec3d_pose
   *@param pr_frame
   *@param pre_yaw
   *@param pose34d
   *@return true
   *@return false
   */
  bool reLocalization(const Vec3d &vec3d_pose,
                      const std::shared_ptr<KeyFrame> pr_frame,
                      const double pre_yaw, Mat34d &pose34d);

  double reLocalization(const Vec3d &vec3d_pose,
                        const laserCloud::Ptr &cur_cloud, const double pre_yaw,
                        Mat34d &pose34d);

  bool getRoadThtha(const std::vector<double> &scan, double angle_min,
                    double angle_increment, double &ththa);

  double getPointCloudDirect(const laserCloud::Ptr ptr_cloud);

 private:
  int history_frame_id_;

  int curren_frame_id_;

  LoopConfig loop_config_;

  std::list<std::shared_ptr<KeyFrame>> l_ptr_keyframe_;  ///< 插入关键帧缓存
  std::vector<std::shared_ptr<KeyFrame>>
      v_ptr_history_keyframe_;                  ///< 历史关键帧点云
  std::shared_ptr<KeyFrame> ptr_cur_keyframe_;  ///< 当前关键帧指针
  std::atomic<bool> is_new_keyframe_;           ///< 是否有新的关键帧

  laserCloud::Ptr ptr_cur_keycloud_;         ///< 当前关键帧点云
  laserCloud::Ptr ptr_history_keycloud_;     ///< 历史点云
  laserCloud::Ptr ptr_ds_cur_keycloud_;      ///< 当前关键帧点云降采样
  laserCloud::Ptr ptr_ds_history_keycloud_;  ///< 历史关键帧点云降采样

  std::mutex keyframe_mutex_;  ///< 关键帧锁

  Mat4d transformation_;  ///< 回环约束变换
  Mat34d T_lo_;           ///< odom到激光的变换

  std::atomic<bool> stop_icp_;
  std::atomic<bool> is_reset_requested_;
  std::atomic<bool> is_finish_requested_;
  std::atomic<bool> is_finished_;
  std::atomic<bool> is_stop_;
  std::atomic<bool> is_request_stop_;
  std::condition_variable cv_;

  std::shared_ptr<MapManager> ptr_map_manager_;  ///< 地图管理器指针

  std::shared_ptr<LoamRegistration> ptr_loam_registration_;

  pcl::VoxelGrid<PointType> downsize_filter_history_cloud_;
  pcl::VoxelGrid<PointType> downsize_filter_cur_cloud_;

};  // end of class
}  // namespace cvte_lidar_slam

#endif  // !LOOP_TRACK_HPP_