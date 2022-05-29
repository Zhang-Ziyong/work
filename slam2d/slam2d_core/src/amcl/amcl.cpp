#include "amcl/amcl.hpp"
namespace slam2d_core {
namespace amcl {
Amcl::Amcl(const AmclOptions &options) {
  amcl_options_ = options;
  init_ = false;
  initialization();
}

Amcl::~Amcl() {}

void Amcl::initialization() {
  ParticleFilterOptions pf_options;
  pf_options.min_samples = amcl_options_.min_samples;
  pf_options.max_samples = amcl_options_.max_samples;
  pf_options.alpha_slow = amcl_options_.alpha_slow;
  pf_options.alpha_fast = amcl_options_.alpha_fast;
  pf_options.pop_err = amcl_options_.pop_err;
  pf_options.pop_z = amcl_options_.pop_z;
  pf_options.dist_treshold = amcl_options_.dist_treshold;

  ptr_pf_ = std::make_shared<ParticleFilter>(pf_options);
  ptr_map_ = std::make_shared<OccupancyGrid>(amcl_options_.map_path,
                                             amcl_options_.max_occ_dist);
  motion_model_.setMotionMode(amcl_options_.alpha1_, amcl_options_.alpha2_,
                              amcl_options_.alpha3_, amcl_options_.alpha4_,
                              amcl_options_.alpha5_);
  motion_model_.setHistogramCellSize(amcl_options_.cell_size_x_,
                                     amcl_options_.cell_size_y_,
                                     amcl_options_.cell_size_yaw_);

  // laser model
  laser_model_.setMap(ptr_map_);
  laser_model_.setLaserTF(common::Rigid3{
      Eigen::Vector3d(amcl_options_.laser_pose_x_, amcl_options_.laser_pose_y_,
                      0.0),
      common::yawToQuaternion(0.0, 0.0, amcl_options_.laser_pose_yaw_)});
  laser_model_.setModelLikelihoodField(
      amcl_options_.z_hit_, amcl_options_.z_rand_, amcl_options_.sigma_hit_,
      amcl_options_.max_occ_dist_, amcl_options_.max_beams_);
}

bool Amcl::update(const LaserScanData &laser, const common::Rigid3 &pose) {
  std::lock_guard<std::recursive_mutex> lck_g(update_mutex_);
  // LOG(INFO) << std::fixed
  //           << "laser_time:" << common::toUniversal(laser.time) / 1e7;
  // LOG(INFO) << std::fixed << "interval:"
  //           << common::toSeconds(odom_data_buffer_.back().time - laser.time);
  if (!init_) {
    LOG_EVERY_N(WARNING, 100) << "update fatal, not init..";
    return false;
  }

  if (!set_init_odom_falg_) {
    set_init_odom_falg_ = true;
    last_odom_pose_ = pose;
    return false;
  }

  common::Rigid3 detal_pose = last_odom_pose_.inverse() * pose;

  double trans =
      hypot(detal_pose.translation().x(), detal_pose.translation().y());

  double yaw =
      common::angleDiff(common::quaternionToYaw(pose.rotation()),
                        common::quaternionToYaw(last_odom_pose_.rotation()));
  if (trans < amcl_options_.update_trans_thr_ &&
      std::fabs(yaw) < amcl_options_.update_angle_thr_) {
    return false;
  }
  // LOG(INFO) << "the detal_pose trans : " << trans << " yaw " << yaw;
  bool move_control_restart = false;
  if (std::abs(pose.translation().x()) < 0.1 &&
      std::abs(pose.translation().y()) < 0.1 &&
      std::abs(slam2d_core::common::quaternionToYaw(pose.rotation())) < 0.1) {
    LOG(WARNING) << " ---- move control restart ----";
    move_control_restart = true;
  }  // 处理move_control 重启

  if ((trans > amcl_options_.update_trans_unormal_thr_ ||
       std::fabs(yaw) > amcl_options_.update_angle_unormal_thr_) &&
      move_control_restart) {
    LOG(WARNING) << "the detal_pose trans : " << trans << " yaw " << yaw;
    last_odom_pose_ = pose;
    return false;
  }

  // LOG(INFO) << "action------------------------------";
  bool u_action = ptr_pf_->updateAction(motion_model_, pose, last_odom_pose_);

  if (!u_action) {
    LOG(WARNING) << "update Action fatal..." << std::endl;
    return false;
  } else {
    last_odom_pose_ = pose;
    update_time_ = laser.time;
  }

  bool u_sensor = ptr_pf_->updateSensor(laser_model_, laser);
  if (!u_sensor) {
    LOG(WARNING) << "update Sensor fatal..";
    return false;
  }

  if (!(++resample_count_ % amcl_options_.resample_interval_)) {
    ptr_pf_->updateResample(motion_model_);
  }

  pf_set_pub_flag_ = true;
  return true;
}

bool Amcl::getLocaliztonResult(common::Rigid3 &pose, Eigen::Vector3d &cov,
                               common::Time &loc_time) {
  if (!init_) {
    LOG(WARNING) << "update fatal, not init..";
    return false;
  }
  loc_time = update_time_;

  Eigen::Vector3d pose_mean;
  bool cs_flag = ptr_pf_->computeStatus(motion_model_, pose_mean, cov);
  if (!cs_flag) {
    LOG(WARNING) << "compute status  fatal...";
    return false;
  }

  pose = common::Rigid3{Eigen::Vector3d(pose_mean[0], pose_mean[1], 0.0),
                        common::yawToQuaternion(0.0, 0.0, pose_mean[2])};
  return true;
}

void Amcl::getCurrentPFSet(std::vector<Sample> &current_set,
                           bool &pf_set_pub_flag) {
  pf_set_pub_flag = pf_set_pub_flag_;

  if (pf_set_pub_flag_ && ptr_pf_ != nullptr) {
    std::lock_guard<std::recursive_mutex> lck_g(update_mutex_);
    ptr_pf_->getCurrentPFSet(current_set);
  }

  pf_set_pub_flag_ = false;
}

bool Amcl::isPoseInFreeSpace(const common::Rigid3 &pose) {
  Cell lc = ptr_map_->getCell(pose.translation().x(), pose.translation().y());
  if (lc.occ_state < 0) {
    return true;
  } else {
    LOG(WARNING) << "amcl in un-free space...";
    return false;
  }
}

bool Amcl::setInitPose(const common::Rigid3 &init_pose,
                       const Eigen::Vector3d &cov) {
  std::lock_guard<std::recursive_mutex> lck_g(update_mutex_);

  Eigen::Vector3d init_mean;
  init_mean[0] = init_pose.translation().x();
  init_mean[1] = init_pose.translation().y();
  init_mean[2] = common::quaternionToYaw(init_pose.rotation());

  init_ = ptr_pf_->setInitStatus(motion_model_, init_mean, cov);
  if (!init_) {
    LOG(WARNING) << "set init status  fatal...";
    return false;
  }
  return true;
}

bool Amcl::setInitStatus(const Eigen::Vector3d &init_status,
                         const Eigen::Vector3d &init_cov) {
  std::lock_guard<std::recursive_mutex> lck_g(update_mutex_);
  amcl_options_.update_trans_thr_ = 0;
  amcl_options_.update_angle_thr_ = 0;

  motion_model_.setMotionMode(amcl_options_.alpha1_, amcl_options_.alpha2_,
                              amcl_options_.alpha3_, amcl_options_.alpha4_,
                              amcl_options_.alpha5_);

  ParticleFilterOptions pf_options;
  pf_options.min_samples = amcl_options_.init_min_samples;
  pf_options.max_samples = amcl_options_.init_max_samples;
  pf_options.alpha_slow = amcl_options_.alpha_slow;
  pf_options.alpha_fast = amcl_options_.alpha_fast;
  pf_options.pop_err = amcl_options_.pop_err;
  pf_options.pop_z = amcl_options_.pop_z;
  pf_options.dist_treshold = amcl_options_.dist_treshold;

  ptr_pf_ = std::make_shared<ParticleFilter>(pf_options);

  init_ = ptr_pf_->setInitStatus(motion_model_, init_status, init_cov);
  if (!init_) {
    LOG(WARNING) << "setUpdateStatus fatal...";
    return false;
  }
  return true;
}

bool Amcl::setUpdateStatus(const double thr_trans, const double thr_angle,
                           const Eigen::Vector3d &init_status,
                           const Eigen::Vector3d &init_cov) {
  std::lock_guard<std::recursive_mutex> lck_g(update_mutex_);
  amcl_options_.update_trans_thr_ = thr_trans;
  amcl_options_.update_angle_thr_ = thr_angle;

  motion_model_.setMotionMode(amcl_options_.alpha1_, amcl_options_.alpha2_,
                              amcl_options_.alpha3_, amcl_options_.alpha4_,
                              amcl_options_.alpha5_);

  ParticleFilterOptions pf_options;
  pf_options.min_samples = amcl_options_.min_samples;
  pf_options.max_samples = amcl_options_.max_samples;
  pf_options.alpha_slow = amcl_options_.alpha_slow;
  pf_options.alpha_fast = amcl_options_.alpha_fast;
  pf_options.pop_err = amcl_options_.pop_err;
  pf_options.pop_z = amcl_options_.pop_z;
  pf_options.dist_treshold = amcl_options_.dist_treshold;

  ptr_pf_ = std::make_shared<ParticleFilter>(pf_options);

  init_ = ptr_pf_->setInitStatus(motion_model_, init_status, init_cov);
  if (!init_) {
    LOG(WARNING) << "setUpdateStatus fatal...";
    return false;
  }
  set_init_odom_falg_ = false;
  return true;
}
}  // namespace amcl
}  // namespace slam2d_core