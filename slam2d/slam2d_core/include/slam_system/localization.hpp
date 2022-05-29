#ifndef _LOCALIZATION_H_
#define _LOCALIZATION_H_

#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "amcl/amcl.hpp"
#include "amcl/sample.hpp"
#include "common/odometry_data.hpp"
#include "common/rigid_transform.hpp"
#include "msf/multi_sensors_fusion.hpp"
#include "scan_matcher/global_scan_matcher_2d.hpp"
#include "state_machine/slam_state_machine.hpp"

#include "scan_matcher/laser_odometry_fast_loo.hpp"
namespace slam2d_core {
namespace slam_system {

enum class InitModle { NOTINIT = 0, RELIABLE_POSE, FIX_POSE };

struct LocalizationOptions {
  double init_score_min = 0.35;  // 初始化成功最小匹配得分
  double score_min_threshold = 0.15;
  double loc_recovery_score_min =
      0.9;  // 定位恢复（从失败到成功）时匹配的得分阈值
  double inner_point_dist_threshold = 0.3;  // 匹配点的距离阈值

  double predict_deque_length = 10.0;  // 维持预测数据（里程计）队列长度
  double predict_deque_angle = 10.0;  // 维持预测数据（里程计）最长角度
  double predict_to_observe_length_threshold =
      0.25;  // 预测和观测位姿差xy维度阈值
  double predict_to_observe_angle_threshold = 0.4;  // 预测和观测位姿差角度阈值
  std::vector<double> laser_tf = {0.0, 0.0, 0.0};  // laser_tf
  bool use_fast_loo = false;
  double amcl_reinit_pose_score = 0.5;
  bool use_laserodom_constrain = true;
  double odometry_data_cov = 100.0;
  double laserodom_data_cov = 1.0;
  double init_localizaiton_map_range = 50.0;  // 缩减生成定位多分辨率地图范围
  double laserodom_correct_threshold = 0.3;  // 激光里程计icp纠偏阈值
};

class Localization {
 public:
  Localization();
  ~Localization();

  Localization(const Localization &) = delete;
  Localization &operator=(const Localization &) = delete;

  void startLocalization(const std::string &map_file_path,
                         const InitModle &init_modle,
                         const slam2d_core::common::Rigid3 &init_pose);
  void stopLocalization();
  void addOdomData(const slam2d_core::common::OdometryData &odometry_data);
  void addScanData(const slam2d_core::amcl::LaserScanData &scan_data);
  inline bool isLocalizationInit() { return is_localization_init_; }

  inline void getLocalizationResult(Eigen::Vector3d &result_cov,
                                    slam2d_core::common::Rigid3 &result_pose,
                                    bool &loc_status) {
    result_mutex_.lock();
    result_cov = result_cov_;
    result_pose = result_pose_;

    // loc_status = loc_state_;
    // debug代码，用来协助软件调试
    if (new_node_id_ > 500 && options_.predict_deque_angle > 4.8) {
      loc_status = false;
    } else {
      loc_status = loc_state_;
    }

    result_mutex_.unlock();
  }

  inline bool getLocalizaitonStatus() { return loc_state_; }

  inline void getCurrentPFSet(
      std::vector<slam2d_core::amcl::Sample> &current_set,
      bool &pf_set_pub_flag) {
    if (amcl_ptr_ != nullptr)
      return amcl_ptr_->getCurrentPFSet(current_set, pf_set_pub_flag);
  }

  inline std::shared_ptr<amcl::OccupancyGrid> getMap() {
    if (amcl_ptr_ != nullptr) {
      return amcl_ptr_->getMap();
    } else {
      return nullptr;
    }
  }

  void setParams(
      const slam2d_core::slam_system::LocalizationOptions &loc_options,
      const slam2d_core::amcl::AmclOptions &amcl_options,
      const slam2d_core::msf::MultiSensorsFusionOptions &msf_options,
      const slam2d_core::scan_matcher::GlobalScanMatcher2DOptions &gsm_options);

  //获取激光里程计数据接口(可视化)
  bool getLaserOdometryData(slam2d_core::common::OdometryData &OdometryData);

  // 获取里程计队列，用于定位状态判断
  void addPredictPose(const double &delt_trans, const double &delt_yaw,
                      const slam2d_core::common::OdometryData &odometry_data);

  struct TimePose {
    common::Time time;
    common::Rigid3 pose;
  };

  // 传感器状态
  inline slam2d_core::state_machine::SENSOR_STATUS getCurrentSensorStatus() {
    return cur_sensor_statu_;
  }

  inline void setSensorStatusNormal() {
    cur_sensor_statu_ = slam2d_core::state_machine::SENSOR_STATUS::NORMAL;
  }

  // 设置电梯参数
  inline void setElevatorStatus(bool is_elevator_params) {
    is_elevator_params_ = is_elevator_params;
  }

  // inline void getOdometryDataUpdate(
  //     slam2d_core::common::OdometryData &odometry_data) {
  //   odometry_data = cur_odometry_data_update_;
  // }

  void createLocalizationLocmap(const common::Rigid3 &init_pose);

  // obtained laser cloud
  inline void getLaserOdomCloud(
      pcl::PointCloud<PointType>::Ptr &pre_aligned_laser_cloud,
      pcl::PointCloud<PointType>::Ptr &local_laser_cloud_map) {
    fast_loo_laser_odom_ptr_->getLaserOdomCloud(pre_aligned_laser_cloud,
                                                local_laser_cloud_map);
    if (pre_aligned_laser_cloud == nullptr) {
      LOG(ERROR) << "pre_aligned_laser_cloud == nullptr";
    }
    if (local_laser_cloud_map == nullptr) {
      LOG(ERROR) << "local_laser_cloud_map == nullptr";
    }
  }

  inline void getFirstMapToLaserTransfrom(
      slam2d_core::common::Rigid3 &first_transfrom) {
    first_transfrom = first_transfrom_;
  }

 private:
  void startInitPose();
  void initFixPose();
  void optimizeThread();
  void locThread();
  void initReliablePose();
  void scanMatchInitThread(const slam2d_core::amcl::LaserScanData &scan_data,
                           const common::Rigid3 &init_pose,
                           const common::Rigid3 &odom_start_pose);

  float computeLocScore(const slam2d_core::common::PointCloud &pointcloud,
                        std::shared_ptr<amcl::OccupancyGrid> map,
                        const slam2d_core::common::Rigid3 &global_pose,
                        const double &dist_threshold);
  std::pair<float, float> incrementalComputeDifference(
      const std::deque<TimePose> &predict_pose_deque,
      const std::deque<TimePose> &observe_pose_deque);

  TimePose poseInterpolate(const TimePose &sp, const TimePose &ep,
                           const common::Time &time);
  void trimOdomDataBuffer(const slam2d_core::common::OdometryData &odom_data);

  common::Rigid3 lookUpOdomPose(const common::Time &time);

  LocalizationOptions options_;

  std::thread optimize_thread_;
  std::thread loc_thread_;
  bool scan_available_ = false;
  std::mutex scan_mutex_;
  std::atomic_bool stop_thread_ = true;

  std::atomic_bool is_localization_init_ = false;

  common::Rigid3 init_pose_;
  InitModle init_modle_ = InitModle::NOTINIT;

  std::mutex result_mutex_;
  Eigen::Vector3d result_cov_;
  slam2d_core::common::Rigid3 result_pose_;

  std::mutex odom_mutex_;
  slam2d_core::common::Rigid3 odom_pose_;
  slam2d_core::common::Rigid3 last_odom_pose_;
  bool odom_received_flag_ = false;
  slam2d_core::common::OdometryData last_odom_data_;
  slam2d_core::common::Time odom_time_;
  std::deque<TimePose> odom_data_buffer_;  //缓存里程计数据

  slam2d_core::amcl::AmclOptions amcl_options_;
  std::shared_ptr<slam2d_core::amcl::Amcl> amcl_ptr_ = nullptr;
  std::mutex amcl_mutex_;

  slam2d_core::msf::MultiSensorsFusionOptions msf_options_;
  std::shared_ptr<slam2d_core::msf::MultiSensorsFusion> ptr_msf_;
  std::mutex msf_mutex_;

  slam2d_core::scan_matcher::GlobalScanMatcher2DOptions gsm_options_;
  std::shared_ptr<slam2d_core::scan_matcher::GlobalScanMatcher2D> ptr_gsm_;
  std::thread gsm_search_thread_;
  bool gsm_search_thread_flag_ = false;
  bool gsm_need_search_flag_ = true;

  bool loc_state_ = true;
  bool odom_update_status = false;

  int unreliable_count_ = 0;

  std::mutex pose_deque_mutex_;
  std::deque<TimePose> predict_pose_deque_;
  double predict_cumulate_dist_ = 0.0;
  double predict_cumulate_angle_ = 0.0;
  std::deque<TimePose> observe_pose_deque_;
  std::deque<std::pair<TimePose, TimePose>> match_pair_;
  double dist_delta_sum_ = 0.0;
  double angle_delta_sum_ = 0.0;

  bool laser_odom_scan_available_ = false;
  amcl::LaserScanData current_scan_;

  // fast_loo 激光里程计
  void startFastLooLaserOdom();
  std::shared_ptr<
      slam2d_core::scan_matcher::laser_odometry_fast_loo::LaserOdomtryFastLoo>
      fast_loo_laser_odom_ptr_;
  slam2d_core::scan_matcher::laser_odometry_fast_loo::LaserOdometryFastLooOption
      fast_loo_options_;
  void fastlooLaserOdomThread();
  std::thread fast_loo_laser_odom_thread_;
  bool fast_loo_stop_laser_odom_ = true;
  void stopFastLooLaserOdom();

  common::Rigid3 last_odom_data_pose_;
  common::Rigid3 last_laser_odom_pose_;

  size_t new_node_id_ = 0;  ///< 节点编号

  // 传感器状态
  slam2d_core::state_machine::SENSOR_STATUS cur_sensor_statu_ =
      slam2d_core::state_machine::SENSOR_STATUS::NORMAL;

  // current odometry_data_update
  // slam2d_core::common::OdometryData cur_odometry_data_update_;

  std::shared_ptr<slam2d_core::amcl::OccupancyGrid> ptr_localmap_ =
      std::make_shared<slam2d_core::amcl::OccupancyGrid>();

  // 是否使用电梯参数
  bool is_elevator_params_ = false;
  double localization_time_delay_ = 50.0;

  //
  bool is_first_print_ = false;
  slam2d_core::common::Rigid3 first_transfrom_;
  double first_fusion_yaw_ = 0.0, first_odom_yaw_ = 0.0;
  double last_fusion_yaw_ = 0.0, last_odom_yaw_ = 0.0;
};

}  // namespace slam_system
}  // namespace slam2d_core

#endif