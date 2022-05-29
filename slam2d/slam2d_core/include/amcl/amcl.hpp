#ifndef _AMCL_H_
#define _AMCL_H_

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "common/odometry_data.hpp"
#include "laser_model.hpp"
#include "motion_model.hpp"
#include "particle_filter.hpp"

namespace slam2d_core {
namespace amcl {

class AmclOptions {
 public:
  std::string map_path = "/home/droid/Development/robot_data/map_2d.yaml";

  double max_occ_dist = 0.5;  ///< 计算cache distance map的最大距离

  // particle filter
  unsigned int min_samples = 100;   ///< 最小粒子数
  unsigned int max_samples = 1000;  ///< 最大粒子数
  double alpha_slow = 0;            ///< 生成随机粒子参数
  double alpha_fast = 0;            ///< 生成随机粒子参数

  double pop_err = 0.01;     ///< KLD参数
  double pop_z = 0.7;        ///< KLD参数
  double dist_treshold = 1;  ///< 粒子最大的距离

  // system model parameter
  double alpha1_ = 0.2;  ///< 模型参数
  double alpha2_ = 0.2;  ///< 模型参数
  double alpha3_ = 0.2;  ///< 模型参数
  double alpha4_ = 0.2;  ///< 模型参数
  double alpha5_ = 0.2;  ///< 模型参数

  double cell_size_x_ = 0.5;    ///< 统计直方图分辨率
  double cell_size_y_ = 0.5;    ///< 统计直方图分辨率
  double cell_size_yaw_ = 0.5;  ///< 统计直方图分辨率

  // measurement model parameter
  double max_occ_dist_ = 0.5;  ///< 观测模型算法参数，最大比较距离
  bool do_beamskip_ = false;  ///< 观测模型算法是否对激光点进行降采样
  double beam_skip_distance_ = 0.5;         ///< 观测模型算法参数
  double beam_skip_threshold_ = 0.3;        ///< 观测模型算法参数
  double beam_skip_error_threshold_ = 0.9;  ///< 观测模型算法参数

  double z_hit_ = 0.5;         ///< 观测模型算法参数
  double z_rand_ = 0.5;        ///< 观测模型算法参数
  double sigma_hit_ = 0.08;    ///< 观测模型算法参数
  double z_short_ = 0.05;      ///< 观测模型算法参数
  double z_max_ = 0.05;        ///< 观测模型算法参数
  double lambda_short_ = 0.1;  ///< 观测模型算法参数
  double chi_outlier_ = 0.0;   ///< 观测模型算法参数

  unsigned int max_beams_ = 181;  ///< 最大的匹配激光点数

  // laser parameter
  double laser_pose_x_ = 0.13;  ///< base->laser 坐标转换
  double laser_pose_y_ = 0;     ///< base->laser 坐标转换
  double laser_pose_yaw_ = 0;   ///< base->laser 坐标转换

  // update
  double update_trans_thr_ = 0.2;  ///< 运动的平移超过这个阈值，才会执行更新
  double update_angle_thr_ = 0.2;  ///< 运动的旋转超过这个阈值，才会执行更新
  double update_trans_unormal_thr_ =
      0.6;  ///< 运动的平移超过这个阈值，会认为是异常值
  double update_angle_unormal_thr_ =
      0.5;  ///< 运动的旋转超过这个阈值，会认为是异常值

  // resample
  unsigned int resample_interval_ = 1;  ///< 间隔多少次更新进行一次重采样

  // init parameters
  unsigned int init_max_samples = 5000;
  unsigned int init_min_samples = 100;
};

class Amcl {
 public:
  explicit Amcl(const AmclOptions &options);
  ~Amcl();

  Amcl(const Amcl &) = delete;
  Amcl &operator=(const Amcl &) = delete;

  bool setInitPose(const common::Rigid3 &init_pose, const Eigen::Vector3d &cov);

  void setInitOdomData(const common::Rigid3 &init_odom_pose) {
    last_odom_pose_ = init_odom_pose;
    set_init_odom_falg_ = true;
  }

  bool update(const LaserScanData &laser, const common::Rigid3 &pose);

  bool getLocaliztonResult(common::Rigid3 &pose, Eigen::Vector3d &cov,
                           common::Time &loc_time);

  void getCurrentPFSet(std::vector<Sample> &current_set, bool &pf_set_pub_flag);

  bool isPoseInFreeSpace(const common::Rigid3 &pose);

  inline std::vector<Sample> &getSamples() { return ptr_pf_->getSamples(); }

  unsigned int getSamplesCount() { return ptr_pf_->getSamplesCount(); }

  inline std::shared_ptr<OccupancyGrid> getMap() { return ptr_map_; }

  bool setInitStatus(const Eigen::Vector3d &init_status,
                     const Eigen::Vector3d &init_cov);

  bool setUpdateStatus(const double thr_trans, const double thr_angle,
                       const Eigen::Vector3d &init_status,
                       const Eigen::Vector3d &init_cov);

 private:
  void initialization();

  LaserModel laser_model_;
  MotionModel motion_model_;
  AmclOptions amcl_options_;

  std::recursive_mutex update_mutex_;  ///< 更新锁

  std::shared_ptr<ParticleFilter> ptr_pf_;
  std::shared_ptr<OccupancyGrid> ptr_map_;

  bool init_ = false;
  common::Time update_time_;
  common::Rigid3 last_odom_pose_;
  unsigned int resample_count_ = 0;
  bool set_init_odom_falg_ = false;  ///< 是否设置初始化odom的标志

  bool pf_set_pub_flag_ = false;
};

}  // namespace amcl
}  // namespace slam2d_core

#endif