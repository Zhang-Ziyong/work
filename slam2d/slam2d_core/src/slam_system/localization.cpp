#include "slam_system/localization.hpp"

#include <iomanip>  // 需要加上头文件
#include <utility>
namespace slam2d_core {
namespace slam_system {

using namespace std::chrono_literals;

Localization::Localization() {}

Localization::~Localization() {}

void Localization::setParams(
    const slam2d_core::slam_system::LocalizationOptions &loc_options,
    const slam2d_core::amcl::AmclOptions &amcl_options,
    const slam2d_core::msf::MultiSensorsFusionOptions &msf_options,
    const slam2d_core::scan_matcher::GlobalScanMatcher2DOptions &gsm_options) {
  options_ = loc_options;
  amcl_options_ = amcl_options;
  msf_options_ = msf_options;
  gsm_options_ = gsm_options;

  // msf是否使用雷达里程计约束由定位中的参数传入
  msf_options_.use_laserodom_constrain =
      options_.use_fast_loo && options_.use_laserodom_constrain;
  if (msf_options_.use_laserodom_constrain) {
    LOG(INFO) << " the laserodom constrains will be used in localization ";
  }
}

void Localization::startLocalization(
    const std::string &map_file_path, const InitModle &init_modle,
    const slam2d_core::common::Rigid3 &init_pose) {
  LOG(INFO) << "startLocalization " << map_file_path << std::endl;
  new_node_id_ = 0;
  init_pose_ = init_pose;
  init_modle_ = init_modle;
  amcl_options_.map_path = map_file_path;

  ptr_msf_ =
      std::make_shared<slam2d_core::msf::MultiSensorsFusion>(msf_options_);
  ptr_msf_->newAmclUnaryConstrain(1.0);

  std::chrono::system_clock::time_point map_start_time =
      std::chrono::system_clock::now();
  amcl_ptr_ = std::make_shared<slam2d_core::amcl::Amcl>(amcl_options_);
  std::chrono::system_clock::time_point map_end_time =
      std::chrono::system_clock::now();
  std::chrono::duration<double> map_time_span =
      std::chrono::duration_cast<std::chrono::duration<double>>(map_end_time -
                                                                map_start_time);
  LOG(INFO) << " create map time " << map_time_span.count() << " second.";

  ptr_gsm_ = std::make_shared<slam2d_core::scan_matcher::GlobalScanMatcher2D>(
      gsm_options_);

  stop_thread_ = false;
  is_localization_init_ = false;
  loc_state_ = false;

  scan_available_ = false;
  loc_thread_ = std::thread(std::bind(&Localization::locThread, this));

  optimize_thread_ =
      std::thread(std::bind(&Localization::optimizeThread, this));

  startInitPose();

  // fast_loo 激光里程计
  startFastLooLaserOdom();
}

void Localization::stopLocalization() {
  stop_thread_ = true;
  loc_thread_.join();
  optimize_thread_.join();

  if (gsm_search_thread_.joinable()) {
    gsm_search_thread_.join();
  }

  ptr_msf_.reset();
  amcl_ptr_.reset();
  ptr_gsm_.reset();
  init_pose_ = init_pose_.Identity();
  init_modle_ = InitModle::NOTINIT;
  result_cov_ = {0.0, 0.0, 0.0};
  result_pose_ = result_pose_.Identity();
  odom_pose_ = odom_pose_.Identity();
  last_odom_pose_ = last_odom_pose_.Identity();
  odom_received_flag_ = false;
  odom_data_buffer_.clear();

  is_localization_init_ = false;
  gsm_need_search_flag_ = true;

  pose_deque_mutex_.lock();
  predict_pose_deque_.clear();
  predict_cumulate_dist_ = 0.0;
  predict_cumulate_angle_ = 0.0;
  observe_pose_deque_.clear();
  match_pair_.clear();
  dist_delta_sum_ = 0.0;
  angle_delta_sum_ = 0.0;
  pose_deque_mutex_.unlock();
  // 关闭激光里程计
  stopFastLooLaserOdom();
}

void Localization::startInitPose() {
  if (nullptr == ptr_msf_ || nullptr == amcl_ptr_ || nullptr == ptr_gsm_) {
    LOG(WARNING) << "localization is not start.";
    return;
  } else {
    if (InitModle::RELIABLE_POSE == init_modle_) {
      initReliablePose();
    } else if (InitModle::FIX_POSE == init_modle_) {
      initFixPose();
    }
  }
}

void Localization::initFixPose() {
  LOG(INFO) << "start init fixpose.";
  Eigen::Vector3d cov;
  cov[0] = 1.0;
  cov[1] = 1.0;
  cov[2] = 1.0;
  Eigen::Vector3d init_pose;
  init_pose[0] = init_pose_.translation().x();
  init_pose[1] = init_pose_.translation().y();
  init_pose[2] = slam2d_core::common::quaternionToYaw(init_pose_.rotation());

  if (amcl_ptr_->setInitStatus(init_pose, cov)) {
    LOG(INFO) << "set amcl init status.";
  } else {
    LOG(WARNING) << "set amcl init status error.";
  }
}

void Localization::initReliablePose() {
  LOG(INFO) << "start init reliablepose.";

  Eigen::Vector3d init_pose(
      init_pose_.translation().x(), init_pose_.translation().y(),
      slam2d_core::common::quaternionToYaw(init_pose_.rotation()));
  Eigen::Vector3d init_cov(0.2, 0.2, 0.2);
  if (amcl_ptr_->setUpdateStatus(amcl_options_.update_trans_thr_,
                                 amcl_options_.update_angle_thr_, init_pose,
                                 init_cov)) {
    LOG(INFO) << "amcl into update status ok...";
  } else {
    LOG(ERROR) << "amcl into update status error...";
  }
}

void Localization::trimOdomDataBuffer(
    const slam2d_core::common::OdometryData &odom_data) {
  // 维护里程计队列
  if (odom_data_buffer_.size() == 0) {
    odom_data_buffer_.push_back(TimePose{odom_data.time, odom_data.pose});
    return;
  }

  if (common::toSeconds(odom_data.time - odom_data_buffer_.back().time) > 1) {
    LOG(ERROR) << std::fixed
               << "odom time stamp sequence Jumped and clear the buffer!"
               << " last_odom time"
               << common::toUniversal(odom_data_buffer_.back().time) / 1e7
               << " current_odom time"
               << common::toUniversal(odom_data.time) / 1e7;
    odom_data_buffer_.clear();
    return;
  }

  if (odom_data_buffer_.back().time < odom_data.time) {
    odom_data_buffer_.push_back(TimePose{odom_data.time, odom_data.pose});
  } else {
    // LOG(WARNING) << std::fixed << "odom time stamp sequence is wrong"
    //              << " last_odom time"
    //              << common::toUniversal(odom_data_buffer_.back().time) / 1e7
    //              << " current_odom time"
    //              << common::toUniversal(odom_data.time) / 1e7;
    return;
  }

  while (odom_data_buffer_.size() > 50) { odom_data_buffer_.pop_front(); }
}

void Localization::addOdomData(
    const slam2d_core::common::OdometryData &odometry_data) {
  // fast loo
  slam2d_core::common::OdometryData odometry_data_update, laser_odometry_data;

  // 判断是否使用激光里程计
  if (options_.use_fast_loo) {
    // LOG(INFO) << "using fast loo";
    fast_loo_laser_odom_ptr_->odometryBufHandler(odometry_data);

    if (!fast_loo_laser_odom_ptr_->getUpdateOdomPose(laser_odometry_data)) {
      LOG(WARNING) << "didnt obtain the odometry_data_update";
      return;
    }
  } else {
    // LOG(INFO) << "didnt use fast loo";
    // odometry_data_update = odometry_data;
  }

  // 选择使用激光里程计or 里程计作为amcl 预测
  // if (options_.use_fast_loo && options_.use_laserodom_constrain) {
  //   odometry_data_update = laser_odometry_data;
  // } else {
  //   odometry_data_update = odometry_data;
  // }
  odometry_data_update = odometry_data;

  double odometry_data_cov, laserodom_data_cov;
  if (!is_elevator_params_) {
    odometry_data_cov = options_.odometry_data_cov;
    laserodom_data_cov = options_.laserodom_data_cov;
    localization_time_delay_ = 50.0;
  } else {
    odometry_data_cov = 100.0;
    laserodom_data_cov = 1.0;
    localization_time_delay_ = 10.0;
  }
  // LOG(INFO) << "----- odometry_data_cov " << odometry_data_cov
  //           << " laserodom_data_cov " << laserodom_data_cov
  //           << " localization_time_delay_ " << localization_time_delay_;

  // obtain odometry_data_update
  // cur_odometry_data_update_ = odometry_data_update;

  // odometry_data_update = odometry_data;
  // common::Rigid3 dt_odom_pose =
  //     last_odom_data_pose_.inverse() * odometry_data.pose;
  // common::Rigid3 dt_laser_odom_pose =
  //     last_laser_odom_pose_.inverse() * odometry_data_update.pose;

  // double trans_dt_odom_pose =
  //     hypot(dt_odom_pose.translation().x(), dt_odom_pose.translation().y());
  // double yaw_dt_odom_pose =
  //     slam2d_core::common::quaternionToYaw(dt_odom_pose.rotation());

  // double trans_dt_laser_odom_pose =
  // hypot(dt_laser_odom_pose.translation().x(),
  //                                         dt_laser_odom_pose.translation().y());
  // double yaw_dt_laser_odom_pose =
  //     slam2d_core::common::quaternionToYaw(dt_laser_odom_pose.rotation());

  // if (trans_dt_odom_pose > 0.01 || fabs(yaw_dt_odom_pose) > 0.01 ||
  //     trans_dt_laser_odom_pose > 0.01 || fabs(yaw_dt_laser_odom_pose) > 0.01)
  //     {
  //   LOG(INFO) << std::fixed << std::setprecision(3)
  //             << "current_odometry_data_pose: ( "
  //             << odometry_data.pose.translation().x() << " "
  //             << odometry_data.pose.translation().y() << " "
  //             << common::quaternionToYaw(odometry_data.pose.rotation())
  //             << ")_time: " << common::toUniversal(odometry_data.time) / 1e7
  //             << " current_dt_odometry_data_pose( "
  //             << dt_odom_pose.translation().x() << " "
  //             << dt_odom_pose.translation().y() << " "
  //             << common::quaternionToYaw(dt_odom_pose.rotation())
  //             << " ) current_laser_odometry_data_update_odom ("
  //             << odometry_data_update.pose.translation().x() << " "
  //             << odometry_data_update.pose.translation().y() << " "
  //             <<
  //             common::quaternionToYaw(odometry_data_update.pose.rotation())
  //             << ") time:"
  //             << common::toUniversal(odometry_data_update.time) / 1e7
  //             << " current_dt_laser_odometry_data_pose( "
  //             << dt_laser_odom_pose.translation().x() << " "
  //             << dt_laser_odom_pose.translation().y() << " "
  //             << common::quaternionToYaw(dt_laser_odom_pose.rotation()) << "
  //             )";

  //   last_odom_data_pose_ = odometry_data.pose;
  //   last_laser_odom_pose_ = odometry_data_update.pose;
  // }

  // 第一次接收到里程计数据
  if (!odom_received_flag_) {
    last_odom_data_ = odometry_data_update;

    odom_mutex_.lock();
    odom_pose_ = odometry_data_update.pose;
    odom_mutex_.unlock();

    odom_received_flag_ = true;
    return;
  }

  // 计算里程计偏差，判断里程计是否跳变，
  // last_odom_data_仅用来进行里程计是否发生跳变判断
  slam2d_core::common::Rigid3 delta =
      last_odom_data_.pose.inverse() * odometry_data_update.pose;
  double translation = hypot(delta.translation().x(), delta.translation().y());
  double deltangle = slam2d_core::common::quaternionToYaw(delta.rotation());
  if (translation >= 0.3 || deltangle >= 1.57) {
    LOG(ERROR) << std::fixed << "odom data jump! trans: " << translation
               << ",delta_yaw: " << deltangle << "last_odom_pose("
               << last_odom_data_.pose.translation().x() << " "
               << last_odom_data_.pose.translation().y() << " "
               << common::quaternionToYaw(last_odom_data_.pose.rotation())
               << ") time:" << common::toUniversal(last_odom_data_.time) / 1e7
               << " .current_fast_loo_odom_pose"
               << odometry_data_update.pose.translation().x() << " "
               << odometry_data_update.pose.translation().y() << " "
               << common::quaternionToYaw(odometry_data_update.pose.rotation())
               << ") time:"
               << common::toUniversal(odometry_data_update.time) / 1e7
               << " .current_odom_pose" << odometry_data.pose.translation().x()
               << " " << odometry_data.pose.translation().y() << " "
               << common::quaternionToYaw(odometry_data.pose.rotation())
               << ") time:" << common::toUniversal(odometry_data.time) / 1e7;

    cur_sensor_statu_ = slam2d_core::state_machine::ODOM_POSE_JUMP;

    last_odom_data_ = odometry_data_update;
    return;
  }
  last_odom_data_ = odometry_data_update;

  // 添加里程计队列，用于之后的amcl 定位预测值
  odom_mutex_.lock();
  odom_pose_ = odometry_data_update.pose;
  odom_time_ = odometry_data_update.time;
  trimOdomDataBuffer(odometry_data_update);
  odom_mutex_.unlock();

  if (!ptr_msf_->initialization()) {
    // last_odom_pose_ 用于运动滤波器判断
    last_odom_pose_ = odometry_data_update.pose;
  } else {
    // 添加里程计约束
    slam2d_core::msf::UnaryConstrainData odom_constrain;
    odom_constrain.time = odometry_data.time;
    odom_constrain.global_pose = {
        odometry_data.pose.translation().x(),
        odometry_data.pose.translation().y(),
        slam2d_core::common::quaternionToYaw(odometry_data.pose.rotation())};

    odom_constrain.cov(0, 0) = odometry_data_cov;
    odom_constrain.cov(1, 1) = odometry_data_cov;
    odom_constrain.cov(2, 2) = odometry_data_cov;

    // 添加激光里程计约束
    slam2d_core::msf::UnaryConstrainData laserodom_constrain;
    laserodom_constrain.time = laser_odometry_data.time;
    laserodom_constrain.global_pose = {
        laser_odometry_data.pose.translation().x(),
        laser_odometry_data.pose.translation().y(),
        slam2d_core::common::quaternionToYaw(
            laser_odometry_data.pose.rotation())};

    laserodom_constrain.cov(0, 0) = laserodom_data_cov;
    laserodom_constrain.cov(1, 1) = laserodom_data_cov;
    laserodom_constrain.cov(2, 2) = laserodom_data_cov;

    // 运动滤波器判断是否为关键帧
    slam2d_core::common::Rigid3 delta_odom =
        last_odom_pose_.inverse() * odom_pose_;
    double trans =
        hypot(delta_odom.translation().x(), delta_odom.translation().y());
    double delt_yaw =
        slam2d_core::common::quaternionToYaw(delta_odom.rotation());

    // 如果为运动关键帧，则添加到约束中
    if (trans > 0.01 || fabs(delt_yaw) > 0.01) {
      last_odom_pose_ = odometry_data_update.pose;
      msf_mutex_.lock();
      ptr_msf_->addPredictedConstrainData(odom_constrain, laserodom_constrain);
      msf_mutex_.unlock();

      // 添加里程计绝对位姿，用于定位状态判断
      addPredictPose(trans, delt_yaw, odometry_data_update);
    }

    // 获取定位结果
    msf_mutex_.lock();
    slam2d_core::msf::Pose2d result_pose;
    slam2d_core::common::Time result_time;
    Eigen::Vector3d result_cov;
    ptr_msf_->getLocalizationResult(result_pose, result_time);
    ptr_msf_->getCov(result_cov[0], result_cov[1], result_cov[2]);
    msf_mutex_.unlock();

    // 转换定位结果，并打印log
    result_mutex_.lock();
    result_cov_ = result_cov;
    result_pose_ = slam2d_core::common::Rigid3{
        Eigen::Vector3d(result_pose.x, result_pose.y, 0.0),
        slam2d_core::common::yawToQuaternion(0.0, 0.0, result_pose.yaw)};
    result_mutex_.unlock();

    // 提升定位鲁棒性，及时进行amcl 初始化
    if (unreliable_count_ > 3) {
      unreliable_count_ = 0;
      // 提升效果可以调大方差
      Eigen::Vector3d init_cov(0.2, 0.2, 0.2);
      amcl_ptr_->setInitPose(result_pose_, init_cov);
      LOG(INFO) << "current pose is unreliable, and restart init";
    }

    // 仅打印关键帧
    if (trans > 0.01 || fabs(delt_yaw) > 0.01) {
      // obtained first print
      if (!is_first_print_) {
        is_first_print_ = true;
        first_fusion_yaw_ = common::quaternionToYaw(result_pose_.rotation());
        first_odom_yaw_ =
            common::quaternionToYaw(odometry_data.pose.rotation());
        first_transfrom_ = laser_odometry_data.pose.inverse() * result_pose_;
      }

      LOG(INFO) << std::fixed << std::setprecision(3) << "pose: ("
                << result_pose_.translation().x() << " "
                << result_pose_.translation().y() << " "
                << common::quaternionToYaw(result_pose_.rotation()) << ") "
                << "delta_yaw_to_first "
                << common::quaternionToYaw(result_pose_.rotation()) -
                       first_fusion_yaw_
                << " delta_yaw_to_last "
                << common::quaternionToYaw(result_pose_.rotation()) -
                       last_fusion_yaw_
                << " time:" << common::toUniversal(result_time) / 1e7
                << " .laser_odom ("
                << laser_odometry_data.pose.translation().x() << " "
                << laser_odometry_data.pose.translation().y() << " "
                << common::quaternionToYaw(laser_odometry_data.pose.rotation())
                << ") "
                << "time:"
                << common::toUniversal(laser_odometry_data.time) / 1e7
                << " .odom (" << odometry_data.pose.translation().x() << " "
                << odometry_data.pose.translation().y() << " "
                << common::quaternionToYaw(odometry_data.pose.rotation())
                << ") "
                << "delta_yaw_to_first "
                << common::quaternionToYaw(odometry_data.pose.rotation()) -
                       first_odom_yaw_
                << " delta_yaw_to_last "
                << common::quaternionToYaw(odometry_data.pose.rotation()) -
                       last_odom_yaw_
                << " time:" << common::toUniversal(odometry_data.time) / 1e7;

      last_fusion_yaw_ = common::quaternionToYaw(result_pose_.rotation());
      last_odom_yaw_ = common::quaternionToYaw(odometry_data.pose.rotation());
    }
  }
}

// 添加里程计绝对位姿，用于定位状态判断
void Localization::addPredictPose(
    const double &delt_trans, const double &delt_yaw,
    const slam2d_core::common::OdometryData &odometry_data) {
  // 计算和维护运动预测的缓存buffer，用于定位状态判断
  pose_deque_mutex_.lock();
  if (predict_pose_deque_.size() == 0 ||
      (odometry_data.time > predict_pose_deque_.back().time)) {
    predict_pose_deque_.push_back({odometry_data.time, odometry_data.pose});

    if (common::toSeconds(odometry_data.time -
                          predict_pose_deque_.back().time) > 3.0) {
      LOG(ERROR) << "odom time stamp jumped!，clear odom_pose_buffer";
      cur_sensor_statu_ = slam2d_core::state_machine::ODOM_TIME_STAMP_JUMP;
      predict_pose_deque_.clear();
      predict_cumulate_dist_ = 0.0;
      predict_cumulate_angle_ = 0.0;
    }
  }

  if (predict_pose_deque_.size() > 1) {
    // 第1个变化量可能会因为里程计数据比激光晚而产生异常，故第1个的增量不考虑
    predict_cumulate_dist_ += delt_trans;
    predict_cumulate_angle_ += fabs(delt_yaw);
  }

  // 预测位姿由里程计绝对位姿直接构成，如果累计的距离和角度大于设定值，就从最前队列开始减去，并将对应的变化值减去
  while ((predict_cumulate_dist_ > options_.predict_deque_length ||
          predict_cumulate_angle_ > options_.predict_deque_angle) &&
         predict_pose_deque_.size() > 2) {
    double minus_delt_trans =
        hypot(predict_pose_deque_.front().pose.translation().x() -
                  predict_pose_deque_[1].pose.translation().x(),
              predict_pose_deque_.front().pose.translation().y() -
                  predict_pose_deque_[1].pose.translation().y());
    double minus_delt_yaw = slam2d_core::common::quaternionToYaw(
        (predict_pose_deque_.front().pose.inverse() *
         predict_pose_deque_[1].pose)
            .rotation());
    // predict_cumulate_dist_ -=
    //     (trans + options_.rot_to_trans * fabs(delt_yaw));
    predict_cumulate_dist_ -= minus_delt_trans;
    predict_cumulate_angle_ -= fabs(minus_delt_yaw);

    predict_pose_deque_.pop_front();
  }
  pose_deque_mutex_.unlock();
}

common::Rigid3 Localization::lookUpOdomPose(const common::Time &time) {
  // 根据激光时间戳去查询odom数据时间戳数据
  common::Rigid3 lookup_pose = odom_pose_;
  if (odom_data_buffer_.size() <= 1) {
    return lookup_pose;
  }

  if (time > odom_data_buffer_.back().time) {
    // 不进行外推,避免向外插值不准确。
    lookup_pose = odom_data_buffer_.back().pose;

    if (common::toSeconds(time - odom_data_buffer_.back().time) > 0.1) {
      LOG(WARNING) << std::fixed << "Odom is too delay than laserscan ("
                   << common::toSeconds(time - odom_data_buffer_.back().time)
                   << "s), latest odom time："
                   << common::toUniversal(odom_data_buffer_.back().time) / 1e7
                   << " lasertime:" << common::toUniversal(time) / 1e7;

      cur_sensor_statu_ = slam2d_core::state_machine::SENSOR_STATUS::ODOM_DELAY;
    }

  } else if (time < odom_data_buffer_.front().time) {
    lookup_pose = odom_data_buffer_.front().pose;
    if (odom_data_buffer_.size() > 6) {
      LOG(ERROR) << std::fixed
                 << "LaserScan time is too old than odom_data_buffer"
                 << " lasertime:" << common::toUniversal(time) / 1e7
                 << "  odom_data_buffer:"
                 << common::toUniversal(odom_data_buffer_.front().time) / 1e7
                 << " "
                 << common::toUniversal(odom_data_buffer_.back().time) / 1e7
                 << " " << odom_data_buffer_.size();

      cur_sensor_statu_ =
          slam2d_core::state_machine::SENSOR_STATUS::LASER_DELAY;
    }

  } else {
    // cur_sensor_statu_ = slam2d_core::state_machine::SENSOR_STATUS::NORMAL;
    // LOG(INFO) << "******************** sensor status normal
    // **************"; 插值查询
    for (size_t i = odom_data_buffer_.size() - 1; i >= 1; i--) {
      if (odom_data_buffer_[i - 1].time <= time &&
          odom_data_buffer_[i].time >= time) {
        TimePose pose_interpolate = poseInterpolate(odom_data_buffer_[i - 1],
                                                    odom_data_buffer_[i], time);
        lookup_pose = pose_interpolate.pose;
        break;
      }
    }
  }
  return lookup_pose;
}

void Localization::locThread() {
  while (!stop_thread_) {
    if (scan_available_ && odom_received_flag_) {
      scan_available_ = false;

      scan_mutex_.lock();
      slam2d_core::amcl::LaserScanData scan_data = current_scan_;
      scan_mutex_.unlock();

      if (nullptr == amcl_ptr_ || nullptr == ptr_msf_) {
        continue;
      }

      odom_mutex_.lock();
      slam2d_core::common::Rigid3 odom_pose = lookUpOdomPose(scan_data.time);
      odom_mutex_.unlock();

      amcl_mutex_.lock();
      bool updat_flag = amcl_ptr_->update(scan_data, odom_pose);
      amcl_mutex_.unlock();

      // 判断是否开辟线程并进行搜索初始化
      if (!ptr_msf_->initialization() && InitModle::FIX_POSE == init_modle_ &&
          (gsm_options_.do_global_search || gsm_options_.do_window_search) &&
          gsm_need_search_flag_) {
        if (!gsm_search_thread_flag_ && gsm_search_thread_.joinable()) {
          gsm_search_thread_.join();
        }
        if (!gsm_search_thread_.joinable()) {
          gsm_search_thread_flag_ = true;
          gsm_search_thread_ = std::thread(
              std::bind(&Localization::scanMatchInitThread, this,
                        std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3),
              scan_data.DownSample(0.03), init_pose_, odom_pose);
        }
        continue;
      }

      if (!ptr_msf_->initialization() &&
          InitModle::RELIABLE_POSE == init_modle_) {
        slam2d_core::msf::MSFNode node;
        node.time = scan_data.time;
        node.global_pose = {
            init_pose_.translation().x(), init_pose_.translation().y(),
            slam2d_core::common::quaternionToYaw(init_pose_.rotation())};
        node.cov(0, 0) = 1.0;
        node.cov(1, 1) = 1.0;
        node.cov(2, 2) = 1.0;
        node.pc_ptr = std::make_shared<common::PointCloud>(
            LaserScanDataToPointCloud(scan_data.DownSample(0.03)));
        ptr_msf_->setInitializeNode(node);

        // 是否需要这步骤
        float score =
            computeLocScore(*(node.pc_ptr), amcl_ptr_->getMap(), init_pose_,
                            options_.inner_point_dist_threshold);
        if (score > options_.init_score_min) {
          loc_state_ = true;
          LOG(INFO) << "Reliable Pose intial Sucess. pose("
                    << init_pose_.translation().x() << " "
                    << init_pose_.translation().y() << " "
                    << common::quaternionToYaw(init_pose_.rotation()) << ")";
        } else {
          loc_state_ = false;
          LOG(ERROR) << "Reliable Pose intial score is too law!  score is"
                     << score;
        }
        is_localization_init_ = true;
      }
      if (updat_flag) {
        slam2d_core::common::Rigid3 pose;
        Eigen::Vector3d cov;
        slam2d_core::common::Time time;
        amcl_ptr_->getLocaliztonResult(pose, cov, time);

        if (!ptr_msf_->initialization() && InitModle::FIX_POSE == init_modle_) {
          if (cov[0] < 0.01 && cov[1] < 0.01 && cov[2] < 0.01) {
            slam2d_core::msf::MSFNode node;
            node.time = time;
            node.global_pose = {
                pose.translation().x(), pose.translation().y(),
                slam2d_core::common::quaternionToYaw(pose.rotation())};
            node.cov(0, 0) = cov[0] * 10.0;
            node.cov(1, 1) = cov[1] * 10.0;
            node.cov(2, 2) = cov[2] * 10.0;
            node.pc_ptr = std::make_shared<common::PointCloud>(
                LaserScanDataToPointCloud(scan_data.DownSample(0.03)));
            ptr_msf_->setInitializeNode(node);

            Eigen::Vector3d init_pose(
                pose.translation().x(), pose.translation().y(),
                slam2d_core::common::quaternionToYaw(pose.rotation()));
            Eigen::Vector3d init_cov(0.2, 0.2, 0.2);

            if (amcl_ptr_->setUpdateStatus(amcl_options_.update_trans_thr_,
                                           amcl_options_.update_angle_thr_,
                                           init_pose, init_cov)) {
              float score = computeLocScore(
                  *(node.pc_ptr), amcl_ptr_->getMap(), init_pose_,
                  options_.inner_point_dist_threshold);

              if (score > options_.init_score_min) {
                loc_state_ = true;
                LOG(INFO) << "Fxied Pose intial Sucess. pose is("
                          << pose.translation().x() << " "
                          << pose.translation().y() << " "
                          << common::quaternionToYaw(pose.rotation()) << ")";
              } else {
                loc_state_ = false;
                LOG(ERROR) << "Fixed Initial score is too law！ score is"
                           << score;
              }

              is_localization_init_ = true;
              LOG(INFO) << "amcl into update status ok...";
            } else {
              LOG(ERROR) << "amcl into update status error...";
            }
          }
        } else {
          slam2d_core::msf::UnaryConstrainData uc_amcl;
          uc_amcl.time = time;
          uc_amcl.global_pose = {
              pose.translation().x(), pose.translation().y(),
              slam2d_core::common::quaternionToYaw(pose.rotation())};

          uc_amcl.cov(0, 0) = cov[0] * 10.0;
          uc_amcl.cov(1, 1) = cov[1] * 10.0;
          uc_amcl.cov(2, 2) = cov[2] * 10.0;

          uc_amcl.pc_ptr = std::make_shared<common::PointCloud>(
              LaserScanDataToPointCloud(scan_data.DownSample(0.03)));

          ptr_msf_->addAmclUnaryConstrainData(uc_amcl);
          // LOG(INFO) << "add amcl factor: " << pose.translation().x() << " "
          //           << pose.translation().y() << " "
          //           <<
          //           slam2d_core::common::quaternionToYaw(pose.rotation());
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

void Localization::addScanData(
    const slam2d_core::amcl::LaserScanData &scan_data) {
  if (options_.use_fast_loo) {
    fast_loo_laser_odom_ptr_->laserScanBufHandler(scan_data);
  }

  // 缓存激光数据,以供定位使用
  scan_mutex_.lock();
  current_scan_ = scan_data;
  scan_available_ = true;
  scan_mutex_.unlock();
}

void Localization::optimizeThread() {
  size_t last_id = 0;
  bool flag = true;

  while (!stop_thread_) {
    // 判断初始化节点定位状态
    if (ptr_msf_->initialization() && flag) {
      flag = false;
      if (loc_state_) {
        last_id = 0;
      } else {
        last_id = -10000;
      }
    }

    if (ptr_msf_->getPredictedSize() > 20) {
      auto start_time = std::chrono::system_clock::now();
      ptr_msf_->build();
      ptr_msf_->optimize();
      auto end_time = std::chrono::system_clock::now();
      std::chrono::duration<double> cost_time = end_time - start_time;
      // LOG(INFO) << "Msf build & optimize cost time: " << cost_time.count();

      // 计算定位状态
      // 定位检测部分主要分为两个部分，
      // 1. 打分太低，低于于失效阈值，直接判定定位失效
      // 2. 打分较高，高于有效阈值，直接判定定位有效
      // 3. 打分介于失效阈值和有效阈值之间，
      // 使用Amcl和里程计在一段时间的位姿变化是否相似进行判断
      // 即由于激光雷达打分低（环境恶劣）导致将Amcl结果带偏
      std::vector<msf::MSFNode> last_unary_nodes;
      ptr_msf_->getUnaryNodes(last_unary_nodes);

      float score = 0;
      for (size_t i = 0; i < last_unary_nodes.size(); i++) {
        slam2d_core::common::Rigid3 pose = slam2d_core::common::Rigid3{
            Eigen::Vector3d(last_unary_nodes[i].global_pose.x,
                            last_unary_nodes[i].global_pose.y, 0.0),
            slam2d_core::common::yawToQuaternion(
                0.0, 0.0, last_unary_nodes[i].global_pose.yaw)};
        if (i == 0) {
          score = computeLocScore(*(last_unary_nodes[i].pc_ptr),
                                  amcl_ptr_->getMap(), pose,
                                  options_.inner_point_dist_threshold);
        } else {
          score = std::min(
              score, computeLocScore(*(last_unary_nodes[i].pc_ptr),
                                     amcl_ptr_->getMap(), pose,
                                     options_.inner_point_dist_threshold));
        }

        if (options_.amcl_reinit_pose_score > score) {
          ++unreliable_count_;
        } else {
          unreliable_count_ = 0;
        }

        // LOG(INFO) << std::fixed << "Node_d:" << last_unary_nodes[i].node_id
        //           << ",pose[" << last_unary_nodes[i].global_pose.x << ","
        //           << last_unary_nodes[i].global_pose.y << ","
        //           << last_unary_nodes[i].global_pose.yaw << "],Score:" <<
        //           score;
        new_node_id_ = last_unary_nodes[i].node_id;

        // 如果长时间维持一个特别低的匹配得分，则维持一段距离后上报定位失效（里程计模式）。
        if (score > options_.score_min_threshold) {
          last_id = last_unary_nodes[i].node_id;
        }
        if (static_cast<int>(last_unary_nodes[i].node_id) -
                static_cast<int>(last_id) >=
            msf_options_.bad_match_node_number_) {
          loc_state_ = false;
          LOG(ERROR) << "Localization Failed！ score is" << score;
        }

        pose_deque_mutex_.lock();
        TimePose time_pose = {last_unary_nodes[i].time, pose};
        if (observe_pose_deque_.size() == 0 ||
            time_pose.time > observe_pose_deque_.back().time) {
          observe_pose_deque_.push_back(time_pose);
        }
        while (observe_pose_deque_.size() > 2 &&
               predict_pose_deque_.size() > 1 &&
               observe_pose_deque_.front().time <
                   predict_pose_deque_.front().time) {
          observe_pose_deque_.pop_front();
        }
        pose_deque_mutex_.unlock();
      }

      // 采用预测位姿度队列和观测位姿队列计算定位状态（第二种方式）
      pose_deque_mutex_.lock();
      std::pair<float, float> dist_diff = incrementalComputeDifference(
          predict_pose_deque_, observe_pose_deque_);
      pose_deque_mutex_.unlock();

      // 两个队列的偏差大于阈值，判断方法需要修改
      bool b_flag = std::fabs(dist_diff.first) >
                        options_.predict_to_observe_length_threshold + 0.05 ||
                    std::fabs(dist_diff.second) >
                        options_.predict_to_observe_angle_threshold + 0.05 ||
                    (score <= gsm_options_.min_score &&
                     (std::fabs(dist_diff.first) >
                          options_.predict_to_observe_length_threshold ||
                      std::fabs(dist_diff.second) >
                          options_.predict_to_observe_angle_threshold));
      // 如果得分较低，且
      if (loc_state_ && b_flag && (score <= options_.loc_recovery_score_min)) {
        // loc_state_ = false;
        LOG(WARNING) << " long time different for amcl and odom delta pose";
      } else if (!loc_state_ && score > options_.loc_recovery_score_min) {
        pose_deque_mutex_.lock();
        loc_state_ = true;
        predict_pose_deque_.clear();
        predict_cumulate_angle_ = 0.0;
        predict_cumulate_dist_ = 0.0;
        observe_pose_deque_.clear();
        match_pair_.clear();
        dist_delta_sum_ = 0.0;
        angle_delta_sum_ = 0.0;

        pose_deque_mutex_.unlock();
        LOG(INFO) << "Loc auto recovery Successed!";
      }  // else {//维持之前的状态}

      if (!loc_state_) {
        LOG(ERROR) << "Localization Failed！";
      }
      if (match_pair_.size() > 0) {
        // LOG(INFO) << std::fixed << std::setprecision(3) << "deque size:("
        //           << predict_pose_deque_.size() << ","
        //           << observe_pose_deque_.size() << ") odom("
        //           << match_pair_.back().first.pose.translation().x() << ","
        //           << match_pair_.back().first.pose.translation().y() << ","
        //           << common::quaternionToYaw(
        //                  match_pair_.back().first.pose.rotation())
        //           << ") amcl ("
        //           << match_pair_.back().second.pose.translation().x() << ","
        //           << match_pair_.back().second.pose.translation().y() << ","
        //           << common::quaternionToYaw(
        //                  match_pair_.back().second.pose.rotation())
        //           << "), diff:(" << dist_diff.first << "," <<
        //           dist_diff.second
        //           << ")";
      }
    }
    auto localization_time_delay = localization_time_delay_ * 1ms;
    std::this_thread::sleep_for(localization_time_delay);
    // LOG(INFO) << " localization_time_delay_ " << localization_time_delay_;
  }
}

void Localization::scanMatchInitThread(
    const slam2d_core::amcl::LaserScanData &scan_data,
    const common::Rigid3 &init_pose, const common::Rigid3 &odom_start_pose) {
  std::chrono::system_clock::time_point start_time =
      std::chrono::system_clock::now();

  LOG(INFO) << std::fixed << "Odom_start(" << odom_start_pose.translation().x()
            << " " << odom_start_pose.translation().y() << " "
            << common::quaternionToYaw(odom_start_pose.rotation()) << ")";

  bool gsm_success_flag = false;

  std::chrono::system_clock::time_point map_start_time =
      std::chrono::system_clock::now();
  // 生成多分辨率地图
  // ptr_gsm_->setMap(amcl_ptr_->getMap());
  createLocalizationLocmap(init_pose);
  ptr_gsm_->setMap(ptr_localmap_);

  std::chrono::system_clock::time_point map_end_time =
      std::chrono::system_clock::now();
  std::chrono::duration<double> map_time_span =
      std::chrono::duration_cast<std::chrono::duration<double>>(map_end_time -
                                                                map_start_time);
  LOG(INFO) << " create map time " << map_time_span.count() << " second.";

  map_start_time = std::chrono::system_clock::now();
  // 将激光转成点云并转到base坐标系下
  slam2d_core::common::Rigid3 laser_tf =
      common::Rigid3(Eigen::Vector3d(amcl_options_.laser_pose_x_,
                                     amcl_options_.laser_pose_y_, 0.0),
                     slam2d_core::common::yawToQuaternion(
                         0.0, 0.0, amcl_options_.laser_pose_yaw_));
  slam2d_core::common::PointCloud pointcloud =
      transformPointCloud(LaserScanDataToPointCloud(scan_data), laser_tf);

  slam2d_core::common::Rigid2 pose_estimate;
  if (gsm_options_.do_global_search) {
    gsm_success_flag = ptr_gsm_->scanMatchInFullMap(pointcloud, &pose_estimate);

  } else {
    slam2d_core::common::Rigid2 pose_init_rigid2d =
        slam2d_core::common::project2D(init_pose);
    gsm_success_flag = ptr_gsm_->scanMatchNearPose(pointcloud, &pose_estimate,
                                                   pose_init_rigid2d);
  }
  slam2d_core::common::Rigid3 pose_estimate3d = common::embed3D(pose_estimate);
  map_end_time = std::chrono::system_clock::now();
  map_time_span = std::chrono::duration_cast<std::chrono::duration<double>>(
      map_end_time - map_start_time);
  LOG(INFO) << "caluate pose time  " << map_time_span.count() << " second.";

  // 时间补偿
  odom_mutex_.lock();
  slam2d_core::common::Rigid3 end_odom_pose = odom_pose_;
  slam2d_core::common::Time time = odom_time_;
  odom_mutex_.unlock();

  LOG(INFO) << std::fixed << "Odom_End(" << end_odom_pose.translation().x()
            << " " << end_odom_pose.translation().y() << " "
            << common::quaternionToYaw(end_odom_pose.rotation()) << ")";

  if (gsm_success_flag) {
    LOG(INFO) << "gsm pose(" << pose_estimate3d.translation().x() << " "
              << pose_estimate3d.translation().y() << " "
              << common::quaternionToYaw(pose_estimate3d.rotation()) << ")";

    init_pose_ = pose_estimate3d * odom_start_pose.inverse() * end_odom_pose;
    loc_state_ = true;

    // 更新一次amcl状态
    amcl_mutex_.lock();
    initReliablePose();
    amcl_ptr_->update(scan_data, end_odom_pose);
    amcl_mutex_.unlock();

    result_mutex_.lock();
    result_pose_ = init_pose_;
    result_mutex_.unlock();

    // 对因子图进行初始化
    slam2d_core::msf::MSFNode node;
    node.time = time;
    node.global_pose = {
        init_pose_.translation().x(), init_pose_.translation().y(),
        slam2d_core::common::quaternionToYaw(init_pose_.rotation())};
    node.cov(0, 0) = 1.0;
    node.cov(1, 1) = 1.0;
    node.cov(2, 2) = 1.0;
    node.pc_ptr = std::make_shared<common::PointCloud>(
        LaserScanDataToPointCloud(scan_data.DownSample(0.03)));

    msf_mutex_.lock();
    ptr_msf_->setInitializeNode(node);
    last_odom_pose_ = end_odom_pose;
    msf_mutex_.unlock();

    float score =
        computeLocScore(*(node.pc_ptr), amcl_ptr_->getMap(), pose_estimate3d,
                        options_.inner_point_dist_threshold);
    if (score > options_.init_score_min) {
      loc_state_ = true;
      LOG(INFO) << "global search sucess. intial pose("
                << init_pose.translation().x() << " "
                << init_pose.translation().y() << " "
                << common::quaternionToYaw(init_pose.rotation())
                << "), search pose(" << pose_estimate.translation().x() << " "
                << pose_estimate.translation().y() << " "
                << pose_estimate.rotation().angle() << ") score is:" << score;
    } else {
      loc_state_ = false;
      LOG(ERROR) << "Initial Localization in Failed！ score is" << score;
    }
    is_localization_init_ = true;
  }

  std::chrono::system_clock::time_point end_time =
      std::chrono::system_clock::now();
  std::chrono::duration<double> time_span =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                                start_time);
  LOG(INFO) << "Scan Match search spend " << time_span.count() << " second.";
  gsm_search_thread_flag_ = false;
  gsm_need_search_flag_ = false;
}

// 根据全局位置，地图计算激光的匹配度判断定位状态
float Localization::computeLocScore(
    const slam2d_core::common::PointCloud &pointcloud,
    std::shared_ptr<amcl::OccupancyGrid> map,
    const slam2d_core::common::Rigid3 &global_pose,
    const double &dist_threshold) {
  // 计算激光到世界坐标系变换
  slam2d_core::common::Rigid3 laser_to_base =
      common::Rigid3(Eigen::Vector3d(amcl_options_.laser_pose_x_,
                                     amcl_options_.laser_pose_y_, 0.0),
                     slam2d_core::common::yawToQuaternion(
                         0.0, 0.0, amcl_options_.laser_pose_yaw_));

  slam2d_core::common::Rigid3 laser_global_pose = global_pose * laser_to_base;

  // 将点云转换到世界坐标系下
  slam2d_core::common::PointCloud pc =
      transformPointCloud(pointcloud, laser_global_pose);

  if (pc.size() <= 0) {
    return 0;
  }

  int match_point_number = 0;
  for (const common::RangePoint &point : pc.points()) {
    slam2d_core::amcl::Cell cell =
        map->getCell(point.position.x(), point.position.y());
    if (cell.occ_dist < dist_threshold &&
        cell.occ_dist < amcl_options_.max_occ_dist) {
      match_point_number++;
    }
  }
  return static_cast<float>((match_point_number * 1.0) / (pc.size() * 1.0));
}

void Localization::createLocalizationLocmap(const common::Rigid3 &init_pose) {
  // test
  std::shared_ptr<slam2d_core::amcl::OccupancyGrid> ptr_map_ =
      amcl_ptr_->getMap();
  LOG(INFO) << " the original pose of map: ( " << ptr_map_->getOriginX()
            << " , " << ptr_map_->getOriginY() << " )"
            << " the size of map x = " << ptr_map_->getSizeX()
            << " y = " << ptr_map_->getSizeY();

  double map_x_length = ptr_map_->getSizeX() * ptr_map_->getResolution();
  double map_y_length = ptr_map_->getSizeY() * ptr_map_->getResolution();
  double map_x_min = ptr_map_->getOriginX();
  double map_x_max =
      ptr_map_->getOriginX() + map_x_length - ptr_map_->getResolution();
  double map_y_min = ptr_map_->getOriginY();
  double map_y_max =
      ptr_map_->getOriginY() + map_y_length - ptr_map_->getResolution();

  LOG(INFO) << " obtained the concer pose of map origin ( " << map_x_min
            << " , " << map_y_min << " )"
            << " front ( " << map_x_min << " , " << map_y_max << " )"
            << " right ( " << map_x_max << " , " << map_y_min << " )"
            << " right and front  ( " << map_x_max << " , " << map_y_max
            << " )";

  LOG(INFO) << " the valid in start map 1 "
            << ptr_map_->worldValid(map_x_min, map_y_min) << " 2 "
            << ptr_map_->worldValid(map_x_max, map_y_min) << " 3 "
            << ptr_map_->worldValid(map_x_min, map_y_max) << " 4 "
            << ptr_map_->worldValid(map_x_max, map_y_max);

  //
  // LOG(INFO) << __LINE__;
  // 生成局部地图
  // 获取地图原点
  // TODO: 参数引出
  double localmap_range_x = options_.init_localizaiton_map_range,
         localmap_range_y = options_.init_localizaiton_map_range;
  double localmap_origin_x = init_pose.translation().x() - localmap_range_x / 2,
         localmap_origin_y = init_pose.translation().y() - localmap_range_y / 2;

  double localmap_x_min = localmap_origin_x;
  double localmap_x_max =
      localmap_origin_x + localmap_range_x - ptr_map_->getResolution();
  double localmap_y_min = localmap_origin_y;
  double localmap_y_max =
      localmap_origin_y + localmap_range_y - ptr_map_->getResolution();

  LOG(INFO) << " given pose ( " << init_pose.translation().x() << " , "
            << init_pose.translation().y()
            << " ) obtained the concer pose of localmap origin ( "
            << localmap_x_min << " , " << localmap_y_min << " )"
            << " front ( " << localmap_x_min << " , " << localmap_y_max << " )"
            << " right ( " << localmap_x_max << " , " << localmap_y_min << " )"
            << " right and front  ( " << localmap_x_max << " , "
            << localmap_y_max << " )";

  // 按照全局地图裁剪局部地图
  if (localmap_x_min < map_x_min) {
    localmap_x_min = map_x_min;
  }
  if (localmap_x_max > map_x_max) {
    localmap_x_max = map_x_max;
  }
  if (localmap_y_min < map_y_min) {
    localmap_y_min = map_y_min;
  }
  if (localmap_y_max > map_y_max) {
    localmap_y_max = map_y_max;
  }

  // 异常情况处理
  if (localmap_x_min > map_x_max || localmap_x_max < map_x_min ||
      localmap_y_min > map_y_max || localmap_y_max < map_y_min) {
    ptr_localmap_ = ptr_map_;
    LOG(WARNING) << " given init pose error ";
    return;
  }

  // LOG(INFO) << __LINE__;
  // 获取局部地图
  ptr_localmap_->setOriginX(localmap_x_min);
  ptr_localmap_->setOriginY(localmap_y_min);
  ptr_localmap_->setOriginYaw(ptr_map_->getOriginYaw());
  LOG(INFO) << " the localmap origin pose ( " << ptr_localmap_->getOriginX()
            << " , " << ptr_localmap_->getOriginY() << " )";
  ptr_localmap_->setResolution(ptr_map_->getResolution());
  ptr_localmap_->setMaxoccdist(ptr_map_->getMaxOccDist());

  ptr_localmap_->setSizeX(std::floor((localmap_x_max - localmap_x_min) /
                                     ptr_map_->getResolution()));
  ptr_localmap_->setSizeY(std::floor((localmap_y_max - localmap_y_min) /
                                     ptr_map_->getResolution()));
  ptr_localmap_->reseizeMapCells();

  for (double localmap_x = localmap_x_min; localmap_x < localmap_x_max;
       localmap_x += ptr_localmap_->getResolution()) {
    for (double localmap_y = localmap_y_min; localmap_y < localmap_y_max;
         localmap_y += ptr_localmap_->getResolution()) {
      // 生成局部地图对应栅格
      ptr_localmap_->setMapCells(localmap_x, localmap_y,
                                 ptr_map_->getCell(localmap_x, localmap_y));
      // LOG(INFO) << " the index of localmap cell x = " << localmap_cell_x
      //           << " y = " << localmap_cell_y;
    }
  }

  // 进行地图打印
  // std::string mapdatafile = "./testlocalmap.pgm";
  // FILE *out = fopen(mapdatafile.c_str(), "w");

  // fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
  //         ptr_localmap_->getResolution(), ptr_localmap_->getSizeX(),
  //         ptr_localmap_->getSizeY());

  // for (unsigned int j = 0; j < ptr_localmap_->getSizeY(); j++) {
  //   for (unsigned int i = 0; i < ptr_localmap_->getSizeX(); i++) {
  //     if (ptr_localmap_->getCells()[i][ptr_localmap_->getSizeY() - j - 1]
  //             .occ_state == +1) {
  //       fputc(000, out);
  //     } else if (ptr_localmap_->getCells()[i][ptr_localmap_->getSizeY() - j
  //     - 1]
  //                    .occ_state == -1) {
  //       fputc(254, out);
  //     } else {
  //       fputc(205, out);
  //     }
  //   }
  // }

  // fflush(out);
  // fclose(out);
}

std::pair<float, float> Localization::incrementalComputeDifference(
    const std::deque<TimePose> &predict_pose_deque,
    const std::deque<TimePose> &observe_pose_deque) {
  // 统计差异（思考）
  // 对多处的点进行插值，统计出多处的差异情况(尝试比较多个相邻增量绝对值和的大小差异)
  size_t i = 0;
  size_t j = 0;
  while (predict_pose_deque.size() > 2 && observe_pose_deque.size() > 2 &&
         i < predict_pose_deque.size() - 1 && j < observe_pose_deque.size()) {
    if (match_pair_.size() > 0) {
      // 删掉过时的队列数据
      if (match_pair_.front().first.time <= predict_pose_deque.front().time) {
        if (match_pair_.size() > 2) {
          // 队列数据大于2时
          common::Rigid3 delta_pose_pre =
              match_pair_.front().first.pose.inverse() *
              match_pair_[1].first.pose;

          common::Rigid3 delta_pose_obs =
              match_pair_.front().second.pose.inverse() *
              match_pair_[1].second.pose;
          dist_delta_sum_ -= std::fabs(hypot(delta_pose_pre.translation().x(),
                                             delta_pose_pre.translation().y()) -
                                       hypot(delta_pose_obs.translation().x(),
                                             delta_pose_obs.translation().y()));
          angle_delta_sum_ -=
              std::fabs(fabs(slam2d_core::common::quaternionToYaw(
                            delta_pose_pre.rotation())) -
                        fabs(slam2d_core::common::quaternionToYaw(
                            delta_pose_obs.rotation())));

        } else {
          // 队列里头只有一个数据时
          dist_delta_sum_ = 0.0;
          angle_delta_sum_ = 0.0;
        }

        match_pair_.pop_front();
        continue;
      }
      // 定位到最新的位置
      if (observe_pose_deque[j].time <= match_pair_.back().second.time) {
        j++;
        continue;
      }
      if (predict_pose_deque[i + 1].time < match_pair_.back().first.time) {
        i++;
        continue;
      }
    }

    if (observe_pose_deque[j].time < predict_pose_deque[i].time) {
      j++;
      continue;
    }
    if (observe_pose_deque[j].time >= predict_pose_deque[i + 1].time) {
      i++;
      continue;
    }
    if (observe_pose_deque[j].time >= predict_pose_deque[i].time &&
        observe_pose_deque[j].time < predict_pose_deque[i + 1].time) {
      // 位姿插值并累加距离
      TimePose pose_interpolate =
          poseInterpolate(predict_pose_deque[i], predict_pose_deque[i + 1],
                          observe_pose_deque[j].time);
      // LOG(INFO) << "match_pair_.size()" << match_pair_.size();

      if (match_pair_.size() > 0) {
        common::Rigid3 delta_pose_pre =
            match_pair_.back().first.pose.inverse() * pose_interpolate.pose;

        common::Rigid3 delta_pose_obs =
            match_pair_.back().second.pose.inverse() *
            observe_pose_deque[j].pose;
        dist_delta_sum_ += std::fabs(hypot(delta_pose_pre.translation().x(),
                                           delta_pose_pre.translation().y()) -
                                     hypot(delta_pose_obs.translation().x(),
                                           delta_pose_obs.translation().y()));
        angle_delta_sum_ += std::fabs(fabs(slam2d_core::common::quaternionToYaw(
                                          delta_pose_pre.rotation())) -
                                      fabs(slam2d_core::common::quaternionToYaw(
                                          delta_pose_obs.rotation())));
      }
      match_pair_.push_back(std::pair<TimePose, TimePose>(
          pose_interpolate, observe_pose_deque[j]));
      i++;
      j++;
    }
  }

  return std::make_pair<float, float>(dist_delta_sum_, angle_delta_sum_);
}

Localization::TimePose Localization::poseInterpolate(const TimePose &sp,
                                                     const TimePose &ep,
                                                     const common::Time &time) {
  common::Rigid3 delta_pose = sp.pose.inverse() * ep.pose;
  double ratio =
      common::toSeconds(time - sp.time) / common::toSeconds(ep.time - sp.time);
  double rotation_angle =
      common::quaternionToYaw(delta_pose.rotation()) * ratio;
  // 插值计算
  common::Rigid3 result =
      sp.pose *
      common::Rigid3{delta_pose.translation() * ratio,
                     common::yawToQuaternion(0.0, 0.0, rotation_angle)};
  return TimePose({time, result});
}

bool Localization::getLaserOdometryData(
    slam2d_core::common::OdometryData &OdometryData) {
  (void) OdometryData;
  // if (laser_odom_ptr == nullptr || !laser_odom_ptr->is_initialized()) {
  //   return false;
  // }
  // laser_odom_mutex_.lock();
  // OdometryData = current_laser_odom_data_;
  // laser_odom_mutex_.unlock();
  return true;
}

void Localization::fastlooLaserOdomThread() {
  while (fast_loo_laser_odom_ptr_->laser_odom_init_finished_ &&
         !fast_loo_stop_laser_odom_) {
    // auto start_time = std::chrono::system_clock::now();
    fast_loo_laser_odom_ptr_->laserOdomProcess();
    // auto end_time = std::chrono::system_clock::now();
    // std::chrono::duration<double> cost_time = end_time - start_time;
    // LOG(INFO) << "fast loo LaserOdom cost time: " << cost_time.count();
    std::chrono::milliseconds dura(10);
    std::this_thread::sleep_for(dura);
  }
}

void Localization::startFastLooLaserOdom() {
  fast_loo_options_.laser_tf = options_.laser_tf;
  fast_loo_options_.laserodom_correct_threshold =
      options_.laserodom_correct_threshold;
  // LOG(INFO) << " the laser tf ( " << fast_loo_options_.laser_tf[0] << " , "
  //           << fast_loo_options_.laser_tf[1] << " , "
  //           << fast_loo_options_.laser_tf[2] << " )";
  fast_loo_laser_odom_ptr_ = std::make_shared<
      slam2d_core::scan_matcher::laser_odometry_fast_loo::LaserOdomtryFastLoo>(
      fast_loo_options_);
  fast_loo_stop_laser_odom_ = false;
  fast_loo_laser_odom_thread_ =
      std::thread(std::bind(&Localization::fastlooLaserOdomThread, this));
}

void Localization::stopFastLooLaserOdom() {
  fast_loo_stop_laser_odom_ = true;
  fast_loo_laser_odom_thread_.join();
  fast_loo_laser_odom_ptr_.reset();
}

}  // namespace slam_system
}  // namespace slam2d_core
