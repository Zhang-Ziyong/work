#include "scan_matcher/laser_odometry_fast_loo.hpp"

#include <fstream>
#include <iomanip>  //需要加上头文件
#include <utility>

namespace slam2d_core {
namespace scan_matcher {
namespace laser_odometry_fast_loo {

LaserOdomtryFastLoo::LaserOdomtryFastLoo(
    const slam2d_core::scan_matcher::laser_odometry_fast_loo::
        LaserOdometryFastLooOption &options)
    : options_(options) {
  init();
}

void LaserOdomtryFastLoo::Quaternion2EulerAngle(const Eigen::Quaterniond &q,
                                                double &roll, double &pitch,
                                                double &yaw) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
}

void LaserOdomtryFastLoo::init() {
  t_rl_ = Eigen::Vector3d(options_.laser_tf[0], options_.laser_tf[1],
                          options_.laser_tf[2]);
  R_rl_ = Utility::ypr2R(Eigen::Vector3d(0.0, 0.0, 0.0));
  q_rl_ = Eigen::Quaterniond(R_rl_);
  q_rl_.normalize();

  T_rl_ = Eigen::Matrix4d::Identity();
  T_rl_.block<3, 3>(0, 0) = R_rl_;
  T_rl_.block<3, 1>(0, 3) = t_rl_;

  // std::cout << "T_rl: " << T_rl << std::endl;

  T_lr_ = Eigen::Matrix4d::Identity();
  T_lr_ = T_rl_.inverse();
  t_lr_ = T_lr_.block<3, 1>(0, 3);
  R_lr_ = T_lr_.block<3, 3>(0, 0);
  q_lr_ = Eigen::Quaterniond(R_lr_);
  q_lr_.normalize();

  allocateMemory();
  icpConfig();
  laser_odom_init_finished_ = true;

  // std::ofstream fastloo_output("fastloo_output.txt", std::ios::app);
  LOG(INFO) << "--------- fast loo init ---------" << std::endl;
  // fastloo_output.close();
}

void LaserOdomtryFastLoo::allocateMemory() {
  laser_cloud_raw_.reset(new pcl::PointCloud<PointType>());
  laser_cloud_downsample_.reset(new pcl::PointCloud<PointType>());
  pre_aligned_laser_cloud_.reset(new pcl::PointCloud<PointType>());
  local_laser_cloud_map_.reset(new pcl::PointCloud<PointType>());

  for (unsigned int i = 0; i < options_.local_map_windows_size; i++) {
    local_key_frame_cloud_[i].reset(new pcl::PointCloud<PointType>());
    // 改成指针要用push back
  }
}

void LaserOdomtryFastLoo::odometryBufHandler(
    const slam2d_core::common::OdometryData &odometry_data) {
  Eigen::Quaterniond q_odom = odometry_data.pose.rotation();
  Eigen::Vector3d t_odom = odometry_data.pose.translation();
  q_odom.normalize();
  int64_t odom_timestamp = common::toUniversal(odometry_data.time);
  double odom_time_float = (double) odom_timestamp / 10000000.0;

  if (first_odom_) {
    smooth_last_q_w_odom_new_ = q_odom;
    smooth_last_t_w_odom_new_ = t_odom;
    first_odom_ = false;
  }

  q_laser = q_odom * q_rl_;  // 坐标在里程计坐标系下，转换到雷达坐标系
  t_laser = q_odom * t_rl_ + t_odom;
  q_laser.normalize();

  Eigen::Isometry3d odom_rtmatrix = Eigen::Isometry3d::Identity();
  odom_rtmatrix.rotate(q_laser.toRotationMatrix());
  odom_rtmatrix.pretranslate(t_laser);

  // Eigen::Isometry3d odom_rtmatrix = Eigen::Isometry3d::Identity();
  // odom_rtmatrix.rotate(q_odom.toRotationMatrix());
  // odom_rtmatrix.pretranslate(t_odom);

  odom_buf_mutex_.lock();
  odom_buf_.push(std::make_pair(odom_time_float, odom_rtmatrix));
  if (odom_buf_.size() > 100) {
    odom_buf_.pop();
  }
  odom_buf_mutex_.unlock();

  //////////////////
  Eigen::Quaterniond q_w_laser_new = q_wmap_wodom_ * q_laser;
  Eigen::Vector3d t_w_laser_new = q_wmap_wodom_ * t_laser + t_wmap_wodom_;
  q_w_laser_new.normalize();

  Eigen::Quaterniond q_w_odom_new = q_w_laser_new * q_lr_;
  Eigen::Vector3d t_w_odom_new = q_w_laser_new * t_lr_ + t_w_laser_new;
  q_w_odom_new.normalize();

  dt_odom_ = smooth_last_q_w_odom_new_.conjugate() *
             (t_w_odom_new - smooth_last_t_w_odom_new_);
  double dis = dt_odom_.norm();
  double ratio = dis / 0.02;
  if (ratio > 0.5) {
    // dt_odom.x() = dt_odom.x() / ratio;
    dt_odom_.y() = dt_odom_.y() / (ratio * 10.0);
    t_w_odom_new =
        smooth_last_t_w_odom_new_ + smooth_last_q_w_odom_new_ * dt_odom_;
    // LOG(WARNING) << "smooth is working t_w_odom_new.matrix() \n"
    //              << t_w_odom_new.matrix() << "\n dt_odom_.matrix() \n"
    //              << dt_odom_.matrix();
  }
  smooth_last_q_w_odom_new_ = q_w_odom_new;
  smooth_last_t_w_odom_new_ = t_w_odom_new;

  laser_odom_data_mutex_.lock();
  laser_odom_data_.time = common::fromUniversal(odom_timestamp);
  common::Rigid3 laser_odom_data_pose(t_w_odom_new, q_w_odom_new);
  laser_odom_data_.pose = laser_odom_data_pose;
  obtained_laser_odom_data_ = true;
  laser_odom_data_mutex_.unlock();
}

void LaserOdomtryFastLoo::laserScanBufHandler(
    const slam2d_core::amcl::LaserScanData &scan_data) {
  pcl::PointCloud<PointType>::Ptr laser_cloud_ptr(
      new pcl::PointCloud<PointType>);

  int64_t laser_timestamp = common::toUniversal(scan_data.time);
  double laser_time_float = (double) laser_timestamp / 10000000.0;

  PointType laser_cloud_point;
  laser_cloud_point.z = 0.0;
  double new_point_angle;

  unsigned int laser_beam_num = scan_data.ranges.size();
  for (unsigned int i = 0; i < laser_beam_num; i++) {
    if (std::isnan(scan_data.ranges[i])) {
      continue;
    }
    if (scan_data.ranges[i] < options_.laser_min_range ||
        scan_data.ranges[i] > options_.laser_max_range) {
      continue;
    }
    new_point_angle = scan_data.ranges_angle[i];
    laser_cloud_point.x = scan_data.ranges[i] * cos(new_point_angle);
    laser_cloud_point.y = scan_data.ranges[i] * sin(new_point_angle);
    // maybe error !!!!
    laser_cloud_ptr->push_back(laser_cloud_point);
  }

  if (10 > laser_cloud_ptr->size()) {
    return;
  }

  laser_buf_mutex_.lock();
  laser_buf_.push(std::make_pair(laser_time_float, laser_cloud_ptr));
  if (laser_buf_.size() > 30) {
    laser_buf_.pop();
  }
  laser_buf_mutex_.unlock();
}

void LaserOdomtryFastLoo::icpConfig() {
  down_size_filter_icp_.setLeafSize(options_.icp_filter_leafsize,
                                    options_.icp_filter_leafsize,
                                    options_.icp_filter_leafsize);
  down_size_filter_map_.setLeafSize(options_.local_map_filter_leafsize,
                                    options_.local_map_filter_leafsize,
                                    options_.local_map_filter_leafsize);
}

void LaserOdomtryFastLoo::downsampleCurrentLaserCloud() {
  laser_cloud_downsample_->clear();
  down_size_filter_icp_.setInputCloud(laser_cloud_raw_);
  down_size_filter_icp_.filter(*laser_cloud_downsample_);

  PointType point;
  double front_angle_threshold = 1.5 / 180.0 * M_PI;  // tan(5 deg)*10m = 0.8m
  for (size_t i = 0; i < laser_cloud_raw_->size(); i++) {
    point = laser_cloud_raw_->points[i];

    float ang_rad = std::atan2(point.y, point.x);
    if (std::abs(ang_rad) < front_angle_threshold)
    // 根据扫描线角度进行选择 ?? 为什么捞回来
    {
      laser_cloud_downsample_->push_back(point);
    }
  }
  laser_cloud_downsample_size_ = laser_cloud_downsample_->size();
}

bool LaserOdomtryFastLoo::isKeyFrame() {
  static unsigned int frame_count = 0;

  dt_odom_last_keyframe_ = dt_odom_;
  dq_odom_last_keyframe_ = dq_odom_;

  dq_odom_ = q_wodom_last_.conjugate() * q_wodom_curr_;
  dq_odom_.normalize();
  dt_odom_ = q_wodom_last_.conjugate() * (t_wodom_curr_ - t_wodom_last_);

  double droll, dpitch, dyaw;
  Quaternion2EulerAngle(dq_odom_, droll, dpitch, dyaw);
  double dis = dt_odom_.norm();

  frame_count++;

  if (dis > options_.key_frame_distance ||
      std::abs(dyaw) > options_.key_frame_angle ||
      frame_count > options_.frame_count) {
    Eigen::Isometry3f Tt_wodom_curr = Eigen::Isometry3f::Identity();
    Tt_wodom_curr.rotate(q_wodom_curr_.toRotationMatrix().cast<float>());
    Tt_wodom_curr.pretranslate(t_wodom_curr_.cast<float>());

    // LOG(INFO) << " dis " << dis << " std::abs(dyaw) " << std::abs(dyaw)
    //           << " frame_count " << frame_count << std::endl;
    // LOG(INFO) << "Tt_wodom_curr.matrix()" << std::endl;
    // LOG(INFO) << Tt_wodom_curr.matrix() << std::endl;

    // 避免里程计跳变； //
    // if (dis > options_.key_frame_distance * 2 ||
    //     std::abs(dyaw) > options_.key_frame_angle * 2) {
    //   dt_odom_ = dt_odom_last_keyframe_;
    //   dq_odom_ = dq_odom_last_keyframe_;
    // }
    frame_count = 0;
    return true;
  } else {
    return false;
  }
}

void LaserOdomtryFastLoo::lidarOnlyPredict() {
  double droll, dpitch, dyaw;
  Quaternion2EulerAngle(dq_odom_, droll, dpitch, dyaw);

  // if (dt_odom_.norm() > options_.key_frame_distance * 2 ||
  //     std::abs(dyaw) > options_.key_frame_angle * 2) {
  //   // 防止里程计数据跳变过大, 使用之前时刻的里程计数据
  //   dq_odom_ = dq_odom_last_;
  //   dt_odom_ = dt_odom_last_;
  //   LOG(WARNING) << " dq_odom_ to large !!!!" << std::endl;
  // }

  if (first_laser_odom_process_flag_) {
    q_w_curr_ = q_wmap_wodom_ * q_wodom_curr_;
    q_w_curr_.normalize();
    t_w_curr_ = q_wmap_wodom_ * t_wodom_curr_ + t_wmap_wodom_;

    T_w_init_ = Eigen::Isometry3f::Identity();
    T_w_init_.rotate(q_w_curr_.toRotationMatrix().cast<float>());
    T_w_init_.pretranslate(t_w_curr_.cast<float>());
  } else {
    drt_odom_ = Eigen::Isometry3f::Identity();
    drt_odom_.rotate(dq_odom_.toRotationMatrix().cast<float>());
    drt_odom_.pretranslate(dt_odom_.cast<float>());
    T_w_init_ = T_w_last_ * drt_odom_;

    // LOG(INFO) << "drt_odom_ " << std::endl;
    // LOG(INFO) << drt_odom_.matrix() << std::endl;
  }

  // remove z, roll, pitch
  Eigen::Quaterniond print_rotation_old = Eigen::Quaterniond::Identity();
  print_rotation_old = T_w_init_.rotation().cast<double>();
  Quaternion2EulerAngle(print_rotation_old, droll, dpitch, dyaw);
  LOG(ERROR) << " old print_rotation roll " << droll << " pitch " << dpitch
             << " dyaw " << dyaw << " x " << T_w_init_.translation().x()
             << " y " << T_w_init_.translation().y() << " z "
             << T_w_init_.translation().z();

  Eigen::Vector3d ypr{dyaw, 0.0, 0.0};
  Eigen::Matrix3d ypr2matrix3d;
  // rpy2matrix3d = Utility::ypr2R(ypr);
  ypr2matrix3d = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());

  Eigen::Quaterniond ypr2quaterniond = Eigen::Quaterniond::Identity();
  ypr2quaterniond = Eigen::Quaterniond(ypr2matrix3d);
  ypr2quaterniond.normalize();
  Eigen::Vector3d trans_vector3d{T_w_init_.translation().x(),
                                 T_w_init_.translation().y(), 0};
  LOG(WARNING) << " ypr2quaterniond.toRotationMatrix().cast<float>() \n"
               << ypr2quaterniond.toRotationMatrix().cast<float>();

  T_w_init_ = Eigen::Isometry3f::Identity();
  T_w_init_.rotate(ypr2quaterniond.toRotationMatrix().cast<float>());
  T_w_init_.pretranslate(trans_vector3d.cast<float>());

  Eigen::Quaterniond print_rotation_new = Eigen::Quaterniond::Identity();
  print_rotation_new = T_w_init_.rotation().cast<double>();
  Quaternion2EulerAngle(print_rotation_new, droll, dpitch, dyaw);

  LOG(ERROR) << " new print_rotation roll " << droll << " pitch " << dpitch
             << " dyaw " << dyaw << " x " << T_w_init_.translation().x()
             << " y " << T_w_init_.translation().y() << " z "
             << T_w_init_.translation().z();

  dq_odom_last_ = dq_odom_;
  dt_odom_last_ = dt_odom_;
}

void LaserOdomtryFastLoo::transformUpdate() {
  q_wmap_wodom_ = q_w_curr_ * q_wodom_curr_.inverse();
  q_wmap_wodom_.normalize();
  t_wmap_wodom_ = t_w_curr_ - q_wmap_wodom_ * t_wodom_curr_;
}

void LaserOdomtryFastLoo::localMapUpdate(
    const pcl::PointCloud<PointType>::Ptr &laserCloudTemp,
    const Eigen::Vector3d &t_w_curr, const Eigen::Quaterniond &q_w_curr) {
  Eigen::Quaterniond dq_w_last2curr = q_w_last_.inverse() * q_w_curr;

  double droll, dpitch, dyaw;
  Quaternion2EulerAngle(dq_w_last2curr, droll, dpitch, dyaw);
  double dt_w_last2curr = (t_w_curr - t_w_last_).norm();

  if (dt_w_last2curr > options_.key_frame_distance ||
      std::abs(dyaw) > options_.key_frame_angle) {
    // 滑动窗口，设置为20；即只保留最新的20帧点云来构建局部地图；
    size_t Id = local_key_frame_id_ % options_.local_map_windows_size;

    // 将当前帧点云转换到地图坐标系；
    Eigen::Isometry3f T_w_scan2map = Eigen::Isometry3f::Identity();
    T_w_scan2map.rotate(q_w_curr.toRotationMatrix().cast<float>());
    T_w_scan2map.pretranslate(t_w_curr.cast<float>());

    local_key_frame_cloud_[Id]->clear();
    pcl::transformPointCloud(*laserCloudTemp, *local_key_frame_cloud_[Id],
                             T_w_scan2map);

    // 将滑动窗口中点云累加起来；
    pcl::PointCloud<PointType>::Ptr CloudMap(new pcl::PointCloud<PointType>());
    for (unsigned int i = 0; i < options_.local_map_windows_size; i++) {
      *CloudMap += *local_key_frame_cloud_[i];  //
    }

    // 对局部点云地图进行降采样；
    local_laser_cloud_map_->clear();
    down_size_filter_map_.setInputCloud(CloudMap);
    down_size_filter_map_.filter(*local_laser_cloud_map_);

    q_w_last_ = q_w_curr;
    t_w_last_ = t_w_curr;

    local_key_frame_id_++;
  }
}  // namespace laser_odometry_fast_loo

void LaserOdomtryFastLoo::laserOdomProcess() {
  if (!odom_buf_.empty() && !laser_buf_.empty()) {
    // LOG(WARNING) << " buf process ------------------------------- "
    //              << std::endl;
    odom_buf_mutex_.lock();
    laser_buf_mutex_.lock();

    time_laser_curr_ = laser_buf_.front().first;
    time_odom_curr_ = odom_buf_.front().first;

    while (time_odom_curr_ < time_laser_curr_) {
      odom_buf_.pop();
      if (!odom_buf_.empty()) {
        time_odom_curr_ = odom_buf_.front().first;
      } else {
        break;
      }
    }
    if (odom_buf_.empty()) {
      odom_buf_mutex_.unlock();
      laser_buf_mutex_.unlock();
      return;
    }

    while (time_laser_curr_ < (time_odom_curr_ - 0.07)) {
      laser_buf_.pop();
      LOG(WARNING) << "laser_buf_.pop();";
      if (!laser_buf_.empty()) {
        time_laser_curr_ = laser_buf_.front().first;
      } else {
        break;
      }
    }
    if (laser_buf_.empty()) {
      LOG(WARNING) << "laser_buf_.empty();";
      odom_buf_mutex_.unlock();
      laser_buf_mutex_.unlock();
      return;
    }

    laser_cloud_raw_->clear();
    *laser_cloud_raw_ = *(laser_buf_.front().second);
    laser_buf_.pop();

    t_wodom_curr_ = odom_buf_.front().second.translation();
    q_wodom_curr_ = odom_buf_.front().second.rotation();
    q_wodom_curr_.normalize();
    odom_buf_.pop();
    // 处理里程计buf， 使里程计时间戳大于雷达时间戳
    // 还要添加时间戳匹配！！！
    odom_buf_mutex_.unlock();
    laser_buf_mutex_.unlock();

    if (isKeyFrame() || first_laser_odom_process_flag_) {
      downsampleCurrentLaserCloud();
      lidarOnlyPredict();

      // 判断是否为退化场景
      judgeDegradation();

      pre_aligned_laser_cloud_->clear();
      pcl::transformPointCloud(*laser_cloud_downsample_,
                               *pre_aligned_laser_cloud_, T_w_init_);

      if (first_laser_odom_process_flag_) {
        pcl::transformPointCloud(*laser_cloud_downsample_,
                                 *local_laser_cloud_map_, T_w_init_);
        first_laser_odom_process_flag_ = false;
      }

      // Align clouds
      pcl::PointCloud<PointType>::Ptr align_result(
          new pcl::PointCloud<PointType>());

      static pcl::IterativeClosestPoint<PointType, PointType> icp_tmp_;
      icp_tmp_.setMaxCorrespondenceDistance(
          options_.icp_max_correspondence_distance);
      // 0.26的取值原则：地图的分辨率为0.1，大于0.25的距离能保证找到最近的3个点；
      icp_tmp_.setMaximumIterations(options_.icp_max_num_iterations);
      icp_tmp_.setTransformationEpsilon(1e-6);
      icp_tmp_.setEuclideanFitnessEpsilon(1e-6);
      icp_tmp_.setInputSource(pre_aligned_laser_cloud_);
      icp_tmp_.setInputTarget(local_laser_cloud_map_);
      icp_tmp_.align(*align_result);

      pub_pre_aligned_laser_cloud_ = pre_aligned_laser_cloud_;
      pub_local_laser_cloud_map_ = local_laser_cloud_map_;
      if (pub_pre_aligned_laser_cloud_ == nullptr) {
        LOG(ERROR) << "pub_pre_aligned_laser_cloud_ == nullptr";
      }
      if (pub_local_laser_cloud_map_ == nullptr) {
        LOG(ERROR) << "pub_local_laser_cloud_map_ == nullptr";
      }

      float align_distance = icp_tmp_.getFitnessScore();
      Eigen::Isometry3f dt_correct = Eigen::Isometry3f::Identity();
      if (icp_tmp_.hasConverged() == false ||
          align_distance > options_.icp_align_distance_threshold ||
          is_degradation_) {
        LOG(WARNING) << " the align_distance of this icp " << align_distance;
        if (icp_tmp_.hasConverged() == false) {
          LOG(WARNING) << "icp_tmp_.hasConverged() == false";
        }
        // std::ofstream fastloo_output("fastloo_output.txt", std::ios::app);
        // fastloo_output.close();
      } else {
        // LOG(WARNING) << " the dt_correct" << std::endl;
        dt_correct = icp_tmp_.getFinalTransformation();
        // std::cout << dt_correct.matrix() << std::endl;
        // TODO:
        // 处理纠正后的矩阵，在长直走廊场景中，出现的误匹配，还要考虑打滑场景
        double dt_correct_dis = dt_correct.translation().cast<double>().norm();
        Eigen::Quaterniond dt_correct_quaternion(
            dt_correct.rotation().cast<double>());
        double dt_correct_roll, t_correct_pitch, dt_correct_yaw;
        Quaternion2EulerAngle(dt_correct_quaternion, dt_correct_roll,
                              t_correct_pitch, dt_correct_yaw);
        LOG(INFO) << " dt_correct_dis = " << dt_correct_dis
                  << " dt_correct_yaw = " << dt_correct_yaw;

        // 阈值如何调整？？
        // 纠正频率为15Hz ≈ 0.7s, 最大线速度为1m/s, 最大角速度1rad/s
        if (dt_correct_dis > options_.laserodom_correct_threshold) {
          LOG(ERROR) << " unormal icp match !!!";
          dt_correct = Eigen::Isometry3f::Identity();
        }
      }
      is_degradation_ = false;

      {
        T_w_updated_ = dt_correct * T_w_init_;
        // 得到位姿修正量，累加得到当前位姿；
        // LOG(INFO) << "TimeStamp " << odom_timestamp << " T_w_init_ "
        //           << std::endl;
        // LOG(INFO) << " T_w_init_ \n" << T_w_init_.matrix() << std::endl;
        // LOG(INFO) << "dt_correct.matrix() : \n" << dt_correct.matrix();

        t_w_curr_ = T_w_updated_.translation().cast<double>();
        q_w_curr_ = T_w_updated_.rotation().cast<double>();
        q_w_curr_.normalize();
      }

      transformUpdate();  // 求解最新的地图坐标系下的位姿后，更新地图与里程计坐标之间的变换；

      q_wodom_last_ = q_wodom_curr_;
      t_wodom_last_ = t_wodom_curr_;
      // q_w_last_ = q_w_curr_;
      // t_w_last_ = t_w_curr_;
      T_w_last_ = T_w_updated_;
      T_w_init_ = T_w_updated_;

      // LOG(INFO) << "T_w_updated_ T_w_last_ " << std::endl;
      // LOG(INFO) << T_w_updated_.matrix() << std::endl;

      localMapUpdate(laser_cloud_downsample_, t_w_curr_, q_w_curr_);
    }
  }
}  // namespace laser_odometry_fast_loo

bool LaserOdomtryFastLoo::getUpdateOdomPose(
    slam2d_core::common::OdometryData &odometry_data_update) {
  if (obtained_laser_odom_data_) {
    laser_odom_data_mutex_.lock();
    odometry_data_update = laser_odom_data_;
    laser_odom_data_mutex_.unlock();
    return true;
  }
  return false;
}

void LaserOdomtryFastLoo::judgeDegradation() {
  // 点云长度
  if (laser_cloud_downsample_size_ < 100) {
    LOG(WARNING) << " too less points ";
    return;
  }

  Eigen::Vector3d center(0, 0, 0);
  // std::vector<Eigen::Vector3d> nearCorners;
  for (unsigned int i = 0; i < laser_cloud_downsample_size_; i++) {
    Eigen::Vector3d tmp(laser_cloud_downsample_->points[i].x,
                        laser_cloud_downsample_->points[i].y,
                        laser_cloud_downsample_->points[i].z);
    // nearCorners.push_back(tmp);
    center = center + tmp;
  }
  center = center / laser_cloud_downsample_size_;

  Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
  for (unsigned int i = 0; i < laser_cloud_downsample_size_; i++) {
    Eigen::Vector3d tmp(laser_cloud_downsample_->points[i].x,
                        laser_cloud_downsample_->points[i].y,
                        laser_cloud_downsample_->points[i].z);

    Eigen::Matrix<double, 3, 1> tmpZeroMean = tmp - center;
    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
  // if is indeed line feature
  // note Eigen library sort eigenvalues in increasing order
  if (saes.eigenvalues()[2] / saes.eigenvalues()[1] >
      judge_degradation_value_) {
    LOG(WARNING) << " saes.eigenvalues()[2] / saes.eigenvalues()[1] = "
                 << saes.eigenvalues()[2] / saes.eigenvalues()[1];
    is_degradation_ = true;
  }
}
}  // namespace laser_odometry_fast_loo
}  // namespace scan_matcher
}  // namespace slam2d_core