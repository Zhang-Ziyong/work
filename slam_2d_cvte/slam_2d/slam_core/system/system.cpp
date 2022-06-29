
#include "system.hpp"

#include "common/debug_tools/debug_color.h"
#include "frontend/feature_extract_factory.hpp"
#include "malloc.h"
#include "opencv2/opencv.hpp"
#include <pcl/visualization/cloud_viewer.h>

namespace cvte_lidar_slam {
System *System::ptr_system_ = nullptr;

System::System()
    : accx_MF{201},
      accy_MF{41},
      accz_MF{41},
      gyrx_MF{21},
      gyry_MF{21},
      gyrz_MF{21} {
  LOG(INFO) << "Lidar SLAM version: V1.0.1 - 2022.05.13-17:11 !!!";

  ptr_state_machine_ = SlamStateMachine::getInstance();

  Eigen::Matrix3d _r_li;
  _r_li = Mathbox::rpyToRotationMatrix(Eigen::Vector3d(178.56, -1.81, 91.89) *
                                       Deg2Rad);
  Eigen::Matrix3d _r_li_comp;
  _r_li_comp = Mathbox::rpyToRotationMatrix(Eigen::Vector3d(-0.72, -0.55, 0.0) *
                                            Deg2Rad);
  r_li_ = _r_li_comp * _r_li;
  r_il_ = r_li_.inverse();

  T_il_ = Mat34d::Identity();
  T_il_.block<3, 3>(0, 0) = r_il_;
  T_il_.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.0);
}

System::~System() {
  run_opt_ = false;
  shutdown();
}

System *System::getInstance() {
  static System lidar_slam_system;
  return &lidar_slam_system;
}

void System::setConfig(const std::shared_ptr<SystemConfig> ptr_config) {
  ptr_config_ = ptr_config;
  InitParameters();
}

void System::setInitModel(const INIT_MODEL init_model) {
  init_mode_ = init_model;
}

void System::setInitPose(const Mat34d &init_pose) {
  Mat34d T_ol = Mathbox::inversePose34d(T_lo_);
  Mat34d transform_in_laser = Mathbox::multiplePose34d(init_pose, T_ol);
  init_pose_ = transform_in_laser;
}

void System::setReliablePose(const Mat34d &reliable_pose) {
  Mat34d T_ol = Mathbox::inversePose34d(T_lo_);
  Mat34d transform_in_laser = Mathbox::multiplePose34d(reliable_pose, T_ol);
  // cur_map_to_odom_ 在激光坐标系下的坐标
  cur_map_to_odom_ = transform_in_laser;
  ptr_map_track_->setMap2Odom(cur_map_to_odom_, false);

  cur_robot_state_.pose = cur_map_to_odom_;
  is_localization_init_ = true;
  is_first_odom_ = true;
  odom_count_ = 0;

  // 添加定位成功判断
  is_init_localization_sucess_ = true;
  LOG(WARNING) << " is_init_localization_sucess == true";
}

void System::InitParameters() {
  is_stopped_ = false;
  is_stop_requested_ = false;
  is_localization_init_ = false;
  is_first_cloud_ = true;
  is_first_odom_ = true;
  is_first_keyframe_ = true;
  has_new_keyframe_ = false;
  new_keyframe_flag_ = false;
  is_update_map_ = false;
  is_opt_finished_ = false;
  last_opt_finish_time_ = std::chrono::system_clock::now();
  run_opt_ = true;
  id_ = 0;
  last_msf_id_ = 0;
  lost_frame_count_ = 0;
  init_mode_ = INIT_MODEL::NOTINIT;
  d_odom_measures_.clear();
  d_odom_for_lidar_odom_.clear();
  d_lidar_odom_.clear();
  d_gps_measures_.clear();
  d_cloud_measures_.clear();
  d_cloud_for_odom_.clear();
  odom_count_ = 0;
  imu_count_ = 0;
  lidar_count_ = 0;
  last_map_to_odom_ = Mathbox::Identity34();
  cur_map_to_odom_ = Mathbox::Identity34();
  lidar_odom_to_odom_ = Mathbox::Identity34();
  last_lidar_odom_to_odom_ = Mathbox::Identity34();
  cur_robot_state_.Reset();
  cur_lidar_odom_state_.Reset();

  view_cloud_.reset(new laserCloud());
  corner_cloud_.reset(new laserCloud());
  surf_cloud_.reset(new laserCloud());
  corner_cloud_top_.reset(new laserCloud());
  surf_cloud_top_.reset(new laserCloud());
  raw_cloud_.reset(new laserCloud());
  global_cloud_.reset(new laserCloud());
  frame_pose_.reset(new laserCloud());
  cloud_for_occ_map_.reset(new laserCloud());

  USE_IMU = ptr_config_->USE_IMU;
  print_debug = ptr_config_->print_debug;

  // construct modules
  ptr_map_manager_ = MapManager::getInstance();

  ptr_map_manager_->initParameters(ptr_config_->backend_config);

  ptr_lidar_odom_ = std::make_shared<LidarOdom>(&ptr_config_->backend_config);

  ptr_map_track_ = std::make_shared<MapTrack>(&ptr_config_->backend_config);

  ptr_feature_extractor_ =
      FeatureExtractorFactory::creatExtractor(ptr_config_->feature_config);

  ptr_loop_track_ = std::make_shared<LoopTrack>(&ptr_config_->loop_config);

  ptr_msf_ = std::make_shared<MutliSensorsFusion>(ptr_config_->msf_config);

  ptr_occ_map_ = OccMap::getInstance();
  ptr_depth_occ_map_ = DepthOccupancyMap::getInstance();

  ptr_occ_map_->setConfig(ptr_config_->occ_map_config);
  ptr_occ_map_->setExtrincs(T_lo_);
  if (ptr_config_->occ_map_config.dynamic_cloud_remove) {
    ptr_local_occ_map_ = LocalOccMap::getInstance();
    ptr_local_occ_map_->setConfig(ptr_config_->occ_map_config);
    ptr_dynamic_local_map_thread_ =
        new std::thread(&System::dynamicLocalMapThread, this);
  }

  // start threads
  run_opt_ = true;

  ptr_lidarOdom_thread_ = new std::thread(&System::lidarOdomThread, this);

  ptr_maptrack_thread_ = new std::thread(&System::mapTrackThread, this);

  ptr_looptrack_thread_ = new std::thread(&System::loopTrackThread, this);

  ptr_occ_map_thread_ = new std::thread(&System::occupancyMapThread, this);

  ptr_init_localization_thread_ =
      new std::thread(&System::initLocalizationThread, this);
}

bool System::setInitKeyFrameIdList(const std::list<unsigned int> ids) {
  std::list<Vec3d> init_pos_list;
  Vec3d init_pos;
  for (auto id = ids.begin(); id != ids.end(); id++) {
    // TODO: 如果找不到需要尝试+1和-1
    if (ptr_map_manager_->getPositionFromId(*id, init_pos)) {
      LOG(INFO) << "set init pos by id - " << *id << " :\n" << init_pos;
      std::cout << "set init pos by id - " << *id << " :\n"
                << init_pos << std::endl;
      init_pos_list.push_back(init_pos);
    } else if (ptr_map_manager_->getPositionFromId(*id - 1, init_pos)) {
      LOG(INFO) << "set init pos by id - " << *id - 1 << " :\n" << init_pos;
      std::cout << "set init pos by id - " << *id - 1 << " :\n"
                << init_pos << std::endl;
      init_pos_list.push_back(init_pos);
    } else if (ptr_map_manager_->getPositionFromId(*id + 1, init_pos)) {
      LOG(INFO) << "set init pos by id - " << *id + 1 << " :\n" << init_pos;
      std::cout << "set init pos by id - " << *id + 1 << " :\n"
                << init_pos << std::endl;
      init_pos_list.push_back(init_pos);
    }
  }
  if (init_pos_list.size() > 0) {
    // ptr_map_track_->setInitPredictPoseList(init_pos_list);
    return true;
  } else {
    return false;
  }
}

void System::lidarOdomThread() {
  while (!is_stopped_) {
    AD_STATU robot_state = ptr_state_machine_->getCurrentState();
    if (AD_STATU::INIT_LOCALIZATION == robot_state ||
        AD_STATU::UNKNOW == robot_state || is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    if (new_keyframe_flag_ == true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // 1. 创建关键帧：计算预测值；提取特征；
    TicToc creatFrame_cost;
    bool creatFrame_flag = creatFrame();
    if (creatFrame_flag == false) {
      // LOG(WARNING) << "lidarOdomThread: Failed to create frame !!!";
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    float creat_frame_cost = creatFrame_cost.toc();

    // 2. 进行帧图匹配：降采样；构建局部地图；地图匹配优化；更新当前位姿；
    bool match_flag = false;
    TicToc processKeyFrame_cost;
    match_flag = ptr_lidar_odom_->processNewKeyFrame(
        ptr_cur_lidar_odom_keyframe_, false);

    lidar_odom_to_odom_ = ptr_lidar_odom_->getMap2Odom();

    if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
      Mat34d smooth_lidar_odom_to_odom = Mathbox::Interp_SE3(
          last_lidar_odom_to_odom_, lidar_odom_to_odom_,
          ptr_config_->backend_config.lidar_odom_smooth_ratio);  // 0.2
      lidar_odom_to_odom_ = smooth_lidar_odom_to_odom;

      last_lidar_odom_to_odom_ = lidar_odom_to_odom_;
    }

    new_keyframe_flag_ = true;

    LOG(INFO) << "lidarOdomThread - creatFrame_time: " << creat_frame_cost
              << " ms ; processNewKeyFrame_time : "
              << processKeyFrame_cost.toc() << " ms ";
  }
}

void System::mapTrackThread() {
  while (!is_stopped_) {
    AD_STATU robot_state = ptr_state_machine_->getCurrentState();
    if (AD_STATU::INIT_LOCALIZATION == robot_state ||
        AD_STATU::UNKNOW == robot_state || is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    if (new_keyframe_flag_ == true) {
      ptr_cur_keyframe_ = nullptr;
      ptr_cur_keyframe_ = ptr_cur_lidar_odom_keyframe_;

      Mat34d predict_cur_frame_pose = Mathbox::multiplePose34d(
          cur_map_to_odom_, ptr_cur_keyframe_->getLaserOdomPose());
      ptr_cur_keyframe_->updatePose(predict_cur_frame_pose);

      new_keyframe_flag_ = false;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // 2. 进行帧图匹配：降采样；构建局部地图；地图匹配优化；更新当前位姿；
    TicToc processKeyFrame_cost;
    is_match_ok_ =
        ptr_map_track_->processNewKeyFrame(ptr_cur_keyframe_, is_update_map_);

    // 3. 进行地图更新：根据运动距离进行判断；
    if (is_match_ok_ == true) {
      // 建图模式；
      if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
        // 第一帧不应该更新cur_map_to_odom_
        if (!is_first_keyframe_) {
          cur_map_to_odom_ = ptr_map_track_->getMap2Odom();
        }

        has_new_keyframe_ = true;

        // 匹配优化完成之后根据距离进行关键帧的判断；进行地图更新的操作；
        if (ptr_map_track_->isKeyFrame() || is_first_keyframe_) {
          // for occupancy map
          if (!cloud_for_occ_map_->points.empty()) {
            if (is_first_keyframe_ && 0 == ptr_cur_keyframe_->index_) {
              ptr_occ_map_->insertObstacleCloud(cloud_for_occ_map_,
                                                Mathbox::Identity34(), 0,
                                                Mathbox::Identity34());
            } else {
              Mat34d last_keyframe_pose;
              const size_t &last_keyframe_id =
                  ptr_map_manager_->getLatestFrameID();

              if (ptr_map_manager_->getPoseFromID(last_keyframe_id,
                                                  last_keyframe_pose)) {
                Mat34d cur_frame_pose = ptr_cur_keyframe_->getPose();
                Mat34d delta_pose =
                    Mathbox::deltaPose34d(last_keyframe_pose, cur_frame_pose);
                ptr_occ_map_->insertObstacleCloud(cloud_for_occ_map_,
                                                  cur_frame_pose,
                                                  last_keyframe_id, delta_pose);
              }
            }
          }

          if (is_first_keyframe_) {
            LOG(WARNING) << "*************** mapTrackThread: first_keyframe "
                            "******************"
                         << std::endl
                         << std::endl;
            is_first_keyframe_ = false;
          }

          ptr_map_manager_->Add(ptr_cur_keyframe_);

          static unsigned int _loop_track_count = 0;
          static unsigned int _last_loop_count = 0;
          Mat34d cur_frame_pose = ptr_cur_keyframe_->getPose();
          double dis_to_map = cur_frame_pose.block<3, 1>(0, 3).norm();

          int loop_keyframe_count = ptr_config_->loop_config.loop_track_count;
          if (dis_to_map < 5.0) {
            loop_keyframe_count = loop_keyframe_count / 2;
          }
          if (_last_loop_count == loop_count_) {
            loop_keyframe_count = loop_keyframe_count / 4;
          }
          _loop_track_count++;

          // 每隔多个关键帧进行一次闭环检测；
          if (_loop_track_count > loop_keyframe_count) {
            ptr_loop_track_->insertKeyFrame(ptr_cur_keyframe_);
            _loop_track_count = 0;
            _last_loop_count = loop_count_;
          }

          if (ptr_msf_ != nullptr) {
            // 将当前关键帧插入到因子图优化节点中；
            ptr_msf_->addKeyFrame(ptr_cur_keyframe_);

            // TODO: do opt outside of map track
            // 进行闭环优化（如果检测到闭环的话）；
            TicToc loop_opt_cost;
            if (ptr_msf_->optimization()) {
              if (ptr_config_->print_debug) {
                static int count = 0;
                printf("\n\n--------loop opt: count: %d, time: %.1f \n\n",
                       count++, loop_opt_cost.toc());
              }

              Mat34d msf_map_pose = ptr_msf_->getLastNodePose();
              Mat34d msf_odom_pose = ptr_msf_->getLastOdomPose();
              cur_map_to_odom_ = Mathbox::multiplePose34d(
                  msf_map_pose, Mathbox::inversePose34d(msf_odom_pose));
              ptr_map_track_->setMap2Odom(cur_map_to_odom_, true);
              ptr_map_manager_->Update();

              const double dis_to_map = msf_map_pose.block<3, 1>(0, 3).norm();

              auto now_time = std::chrono::system_clock::now();
              std::chrono::duration<double> opt_time_duration =
                  now_time - last_opt_finish_time_;

              // 有闭环的情况，且要求距离地图原点距离在一定阈值内，才进行栅格地图更新；
              if (opt_time_duration.count() >
                  ptr_config_->loop_config.opt_update_map_time) {
                // 该变量说明产生了全局优化，全局地图发生了更新；
                // 告知栅格地图维护模块，需要重新更新全局栅格地图；
                last_opt_finish_time_ = now_time;
                is_opt_finished_ = true;
                LOG(INFO) << "is_opt_finished: " << dis_to_map;
              }

              last_msf_id_ = ptr_cur_keyframe_->index_;

              // 该变量说明产生了全局优化，全局地图发生了更新；
              // 告知地图匹配模块，需要重新构建局部地图；
              is_update_map_ = true;
            } else {
              is_update_map_ = false;
            }
          }
        }
      }
      // 定位模式；
      else if (AD_MODE::AD_LOCALIZATION ==
               ptr_state_machine_->getCurrentMode()) {
        cur_map_to_odom_ = ptr_map_track_->getMap2Odom();

        if (ptr_map_track_->isKeyFrame(true) || is_first_keyframe_) {
          if (is_first_keyframe_) {
            is_first_keyframe_ = false;
          }
        }
      } else {
        LOG(ERROR) << "mode error.";
      }
    } else {
      LOG(WARNING) << "mapTrackThread::processNewKeyFrame: match failed !!!";
    }

    LOG(INFO) << "mapTrackThread::processNewKeyFrame_time: "
              << processKeyFrame_cost.toc() << " ms" << std::endl;

    if (ptr_config_->print_debug) {
      static int process_count = 0;
      static double total_time = 0;
      double ave_time = 0;

      total_time += processKeyFrame_cost.toc();
      process_count++;
      ave_time = total_time / process_count;
      printf("--mapTrackThread::process_keyFrame_time: %.1f \n\n",
             processKeyFrame_cost.toc());
      // printf("--System::creat_frame_time: %.1f, ave_keyFrame_time: %.1f
      // \n\n", creat_frame_cost, ave_time);
    }
  }
}

void System::loopTrackThread() {
  static double loop_cov = ptr_config_->backend_config.loop_cov;
  static double relative_cov = ptr_config_->backend_config.relative_cov;
  while (!is_stopped_) {
    if (AD_MODE::AD_UNKNOW == ptr_state_machine_->getCurrentMode() ||
        is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }

    TicToc loop_cost;
    if (!ptr_loop_track_->checkNewKeyFrames()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    auto ptr_cur_loop_frame = ptr_loop_track_->getCurKeyFramePtr();

    Mat34d cur_frame_pose = ptr_cur_loop_frame->getPose();

    long int history_frame_id = -1;

    clock_t startTime = clock();

    history_frame_id = ptr_loop_track_->getLoopCandidateID(cur_frame_pose);

    if (history_frame_id > 0) {
      Mat34d loop_frame_pose;
      if (!ptr_map_manager_->getPoseFromID(history_frame_id, loop_frame_pose)) {
        LOG(ERROR) << "Can not find loop frame pose";
        continue;
      }

      Mat34d delta_pose =
          Mathbox::deltaPose34d(loop_frame_pose, cur_frame_pose);
      double delta_rot =
          Mathbox::rotationMatrixToEulerAngles(delta_pose.block<3, 3>(0, 0))
              .norm();
      double delta_dis = delta_pose.block<3, 1>(0, 3).norm();

      LOG(WARNING) << "detected candidate Loop: " << history_frame_id << " and "
                   << ptr_cur_loop_frame->index_ << ", delta_dis: " << delta_dis
                   << ", delta_rot: " << delta_rot;

      // cur_frame_pose(2,3) = loop_frame_pose(2,3); //
      // 这里直接用闭环候选帧的高度，因为闭环的时候因该是在同一高度；
      double cov;
      Mat34d transformation;
      bool correct_loop_succeed;
      // 通过ICP，将当前帧与闭环帧构建的局部地图进行匹配，计算当前帧和闭环帧之间的相对位姿，也就是漂移量；
      correct_loop_succeed = ptr_loop_track_->correctLoop(
          history_frame_id, cov, cur_frame_pose, transformation, last_msf_id_);

      if (correct_loop_succeed) {
        // 当前帧位姿乘以漂移量，得到校正后的位姿；
        Mat34d correct_frame_pose =
            Mathbox::multiplePose34d(transformation, cur_frame_pose);

        // 根据闭环校正结果，构建闭环约束，准备启动位姿图优化；
        LoopConstrain loop_constrain;
        loop_constrain.loop_node_id1 = history_frame_id;
        loop_constrain.loop_node_id2 = ptr_cur_loop_frame->index_;
        loop_constrain.ptr_keyframe1 =
            ptr_map_manager_->getKeyFrameFromID(history_frame_id);
        loop_constrain.ptr_keyframe2 = ptr_cur_loop_frame;
        loop_constrain.relative_pose =
            Mathbox::deltaPose34d(loop_frame_pose, correct_frame_pose);

        double cov_ratio = cov / relative_cov;
        cov_ratio = 2.0 * cov_ratio * cov_ratio;
        if (cov_ratio < 1.0) {
          cov_ratio = 1.0;
        }

        double relative_pose_dis =
            loop_constrain.relative_pose.block<3, 1>(0, 3).norm();
        double relative_ratio = 0.5 * relative_pose_dis * relative_pose_dis;
        if (relative_ratio < 1.0) {
          relative_ratio = 1.0;
        }

        loop_constrain.rp_cov =
            Mat6d::Identity() * loop_cov * cov_ratio * relative_ratio;

        LOG(WARNING) << "loop cov_ratio: " << cov_ratio;
        LOG(WARNING) << "loop relative_ratio: " << relative_ratio;
        LOG(WARNING) << "loop_constrain.rp_cov: " << loop_constrain.rp_cov;

        if (relative_pose_dis > ptr_config_->loop_config.loop_relative_pose) {
          LOG(WARNING)
              << "Loop relative_pose_dis too big and not accept !!! dis: "
              << relative_pose_dis;
        } else {
          if (ptr_msf_ != nullptr && !ptr_msf_->msfRunning()) {
            ptr_msf_->addLoop(loop_constrain);
            loop_count_++;
          }
        }

        clock_t endTime = clock();
      }
    }

    LOG(INFO) << "***************loop track cost: " << loop_cost.toc()
              << "*******************";
  }
}

void System::dynamicLocalMapThread() {
  while (!is_stopped_) {
    AD_STATU robot_state = ptr_state_machine_->getCurrentState();
    if (AD_STATU::UNKNOW == robot_state || is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    if (AD_STATU::MAPPING == robot_state) {
      if (ptr_config_->occ_map_config.dynamic_cloud_remove) {
        ptr_local_occ_map_->updateLocalMap();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void System::occupancyMapThread() {
  while (!is_stopped_) {
    AD_STATU robot_state = ptr_state_machine_->getCurrentState();
    if (AD_STATU::UNKNOW == robot_state || is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    if (AD_STATU::MAPPING == robot_state) {
      TicToc occ_cost;

      if (!ptr_config_->use_depth_cloud) {
        std::lock_guard<std::mutex> lock(occ_save_mutex_);

        if (isMapUpdate()) {
          LOG(WARNING) << "map update....";
          ptr_occ_map_->resetUpdateValue();
        }

        if (ptr_occ_map_->updateGlobalMap()) {
          ptr_occ_map_->saveGlobalMap();
        }
      } else {
        std::lock_guard<std::mutex> lock(occ_save_mutex_);
        if (isMapUpdate()) {
          LOG(WARNING) << "map update....";
          ptr_occ_map_->resetUpdateValue();
          ptr_depth_occ_map_->resetUpdateValue();
        }

        if (ptr_occ_map_->updateGlobalMap()) {
          global_occ_map_ = ptr_occ_map_->getCurrOccMap();

          if (ptr_depth_occ_map_->updateGlobalMap(
                  ptr_map_manager_->getKeyFrameDatabase(), global_occ_map_)) {
            // 栅格地图已保存在/tem目录下，而且已经pub出去，此处不需要再保存；
            // ptr_depth_occ_map_->saveGlobalMap();
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}

void System::initLocalizationThread() {
  while (!is_stopped_) {
    if (AD_STATU::INIT_LOCALIZATION != ptr_state_machine_->getCurrentState() ||
        is_stop_requested_ || is_localization_init_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    laserCloud::Ptr input_cloud(new laserCloud());
    {
      // map_tracker_cv_.wait(lock, [this] {
      //   return !d_cloud_measures_.empty() || is_stopped_ ||
      //   is_stop_requested_;
      // });

      if (is_stop_requested_) {
        LOG(WARNING) << "request stop in initLocalizationThread";
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }
      if (is_stopped_) {
        LOG(INFO) << "System stopped in initLocalizationThread";
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }
      if (d_cloud_measures_.empty()) {
        LOG(WARNING) << "Cloud container is empty!";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }

      std::unique_lock<std::mutex> lock(cloud_data_mutex_);
      *input_cloud = *d_cloud_measures_.front().cloud;
      d_cloud_measures_.clear();
    }

    if (init_mode_ == INIT_MODEL::FIX_POSE) {
      LOG(WARNING) << "-------------------------start "
                      "relocalization------------------------------------ ";

      TicToc relocalization_cost;

      Vec3d init_predict_pos;
      double init_predict_yaw = 0.0;
      init_predict_pos = init_pose_.block<3, 1>(0, 3);
      Vec3d angle =
          Mathbox::rotationMatrixToEulerAngles(init_pose_.block<3, 3>(0, 0));
      init_predict_yaw = angle(2);

      double delta_x = 2.0;
      double delta_y = 2.0;
      double delta_yaw = 0.628;
      double candidate_x = init_predict_pos.x();
      double candidate_y = init_predict_pos.y();
      double candidate_yaw = init_predict_yaw;

      std::vector<Vec3d> candidate_box_pos;
      std::vector<double> candidate_box_yaw;

      for (int dir_x = -ptr_config_->loop_config.relocalization_box_size_x + 1;
           dir_x < ptr_config_->loop_config.relocalization_box_size_x;
           dir_x++) {
        for (int dir_y =
                 -ptr_config_->loop_config.relocalization_box_size_y + 1;
             dir_y < ptr_config_->loop_config.relocalization_box_size_y;
             dir_y++) {
          for (int dir_yaw =
                   -ptr_config_->loop_config.relocalization_box_size_yaw + 1;
               dir_yaw < ptr_config_->loop_config.relocalization_box_size_yaw;
               dir_yaw++) {
            candidate_x = init_predict_pos.x() + delta_x * dir_x;
            candidate_y = init_predict_pos.y() + delta_y * dir_y;
            candidate_yaw = init_predict_yaw + delta_yaw * dir_yaw;
            candidate_box_pos.push_back(Vec3d(candidate_x, candidate_y, 0.0));
            candidate_box_yaw.push_back(candidate_yaw);
          }
        }
      }

      Mat34d reliable_pose = init_pose_;
      bool reLocalization_flag = false;
      double min_match_dis = 0.99;
      for (int i = 0; i < candidate_box_pos.size(); i++) {
        Vec3d candidate_predict_pos = candidate_box_pos[i];
        double candidate_predict_yaw = candidate_box_yaw[i];
        LOG(INFO) << "relocalization: try count: " << i
                  << " find candidates by init_pos: ("
                  << candidate_predict_pos.x() << " "
                  << candidate_predict_pos.y() << " " << candidate_predict_yaw
                  << ")";

        Mat34d temp_pose;
        double match_dis =
            ptr_loop_track_->reLocalization(candidate_predict_pos, input_cloud,
                                            candidate_predict_yaw, temp_pose);
        if (match_dis > 0.0 && match_dis < min_match_dis) {
          min_match_dis = match_dis;
          reliable_pose = temp_pose;
          reLocalization_flag = true;
          LOG(WARNING) << "relocalization: fix pose relocalization success !!!"
                       << "reliable_pose: \n"
                       << reliable_pose;
        }
      }

      if (d_odom_measures_.empty()) {
        LOG(ERROR) << "relocalization: odom container is empty !!!";
        continue;
      }
      Mat34d cur_odom_pose = (d_odom_measures_.rbegin()->pose);
      Mat34d cur_lidar_odom_pose =
          Mathbox::multiplePose34d(lidar_odom_to_odom_, cur_odom_pose);

      cur_map_to_odom_ = Mathbox::multiplePose34d(
          reliable_pose, Mathbox::inversePose34d(cur_lidar_odom_pose));
      ptr_map_track_->setMap2Odom(cur_map_to_odom_, false);

      is_localization_init_ = true;
      is_first_odom_ = true;
      odom_count_ = 0;

      {
        std::lock_guard<std::mutex> lock_odom(odom_data_mutex_);
        std::lock_guard<std::mutex> lock_cloud(cloud_data_mutex_);
        d_odom_measures_.clear();
        d_cloud_measures_.clear();
      }

      if (reLocalization_flag == true) {
        // is_localization_init_ = true;
        LOG(WARNING)
            << "relocalization: relocalization success, use reliable pose !!!"
            << "final reliable_pose: \n"
            << reliable_pose;

        // 添加定位成功判断
        is_init_localization_sucess_ = true;
        LOG(WARNING) << " is_init_localization_sucess_ = true";
      } else {
        LOG(ERROR) << "relocalization: use init_pose for localization !!!";
        is_init_localization_sucess_ = false;
        LOG(ERROR) << " is_init_localization_sucess_ = false";
      }

      LOG(WARNING) << "relocalization total time: " << relocalization_cost.toc()
                   << " ms";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

bool System::creatFrame() {
  AD_STATU robot_state = ptr_state_machine_->getCurrentState();
  if (AD_STATU::UNKNOW == robot_state) {
    return false;
  }

  CloudMeasure cur_cloud_measure;
  OdomMeasure cur_odom_measure, forward_odom_measure;

  laserCloud::Ptr input_cloud(new laserCloud());
  Mat34d end_2_begin;
  Vec3d correspond_gps_pos;
  double gps_cov = 1000.;
  bool has_gps = false;
  double time_stamp = -1.;
  double curr_linear = 0.0;
  double curr_angular = 0.0;
  bool find = false;

  TicToc create_frame_cost;

  // insert cloud data into frame
  {
    std::unique_lock<std::mutex> lock(cloud_data_mutex_);
    // map_tracker_cv_.wait(lock, [this] {
    //   return !d_cloud_measures_.empty() || is_stopped_ || is_stop_requested_;
    // });
    if (d_cloud_measures_.empty()) {
      // LOG(WARNING) << "cloud container is empty!";
      return false;
    }
    // cache cloud data to ensure imu and odom data is filled;
    if (d_cloud_measures_.size() < 2) {
      return false;
    }
    if (is_stop_requested_) {
      LOG(WARNING) << "create frame request stop";
      return false;
    }
    if (is_stopped_) {
      LOG(INFO) << "System stopped in create frame.";
      return false;
    }

    // 点云测量队列容器
    cur_cloud_measure = d_cloud_measures_.front();
    if (d_cloud_measures_.size() > 4) {
      LOG(WARNING) << "pointcloud buffer size: " << d_cloud_measures_.size()
                   << std::endl;
    }

    time_stamp = cur_cloud_measure.time_stamp;

    last_frame_time_ = temp_frame_time_;
    temp_frame_time_ = time_stamp;
  }

  // insert odom data into frame
  {
    std::lock_guard<std::mutex> lock1(odom_data_mutex_);
    std::lock_guard<std::mutex> lock2(cloud_data_mutex_);

    if (d_odom_measures_.size() < 2) {
      LOG(WARNING) << "creatFrame: not enough odom data...";
      // LOG(WARNING) << "Odom container is empty!";
      return false;
    }

    if (cur_cloud_measure.time_stamp > d_odom_measures_.back().time_stamp) {
      LOG(WARNING) << "creatFrame: odom data is delay ! time_diff: "
                   << (cur_cloud_measure.time_stamp -
                       d_odom_measures_.back().time_stamp);
      // printf("--creatFrame: odom data is delay, not enough odom data !!!
      // front_odom_time: %f, back_odom_time: %f, cloud_time: %f \n",
      // d_odom_measures_.front().time_stamp,
      // d_odom_measures_.back().time_stamp, cur_cloud_measure.time_stamp);
      return false;
    }

    cur_odom_measure = d_odom_measures_.front();
    if (d_odom_measures_.size() > 25) {
      LOG(WARNING) << "odom buffer size: " << d_odom_measures_.size()
                   << std::endl;
    }

    // throw old cloud data, ensure odom data is filled;
    // while (cur_cloud_measure.time_stamp < (cur_odom_measure.time_stamp -
    // 0.1)) 激光点云时间戳必须大于odom；
    while (cur_cloud_measure.time_stamp < cur_odom_measure.time_stamp) {
      d_cloud_measures_.pop_front();
      LOG(WARNING) << "creatFrame: throw old cloud data ! time_diff: "
                   << (cur_odom_measure.time_stamp -
                       cur_cloud_measure.time_stamp);
      // printf("--creatFrame: throw old cloud data !!! odom_time: %f,
      // cloud_time: %f \n", cur_odom_measure.time_stamp,
      // cur_cloud_measure.time_stamp);

      if (!d_cloud_measures_.empty()) {
        cur_cloud_measure = d_cloud_measures_.front();
      } else {
        LOG(WARNING) << "creatFrame: not find correspond cloud data...";
        return false;
      }
    }
    // throw old odom data; 0.04s tolerance;
    while (cur_odom_measure.time_stamp <
           (cur_cloud_measure.time_stamp - 0.06)) {
      d_odom_measures_.pop_front();
      // LOG(WARNING) << "creatFrame: throw old odom data !!!"
      //              << "odom_time: " << cur_odom_measure.time_stamp <<
      //              "cloud_time: " << cur_cloud_measure.time_stamp;
      // printf("--creatFrame: throw old odom data !!! odom_time: %f,
      // cloud_time: %f \n", cur_odom_measure.time_stamp,
      // cur_cloud_measure.time_stamp);

      if (d_odom_measures_.size() > 2) {
        cur_odom_measure = d_odom_measures_.front();
      } else {
        LOG(WARNING) << "creatFrame: not enough odom data...";
        return false;
      }
    }

    auto front_iter = d_odom_measures_.begin();
    auto second_iter = d_odom_measures_.begin();
    for (; second_iter != d_odom_measures_.end(); second_iter++) {
      if (cur_cloud_measure.time_stamp >= front_iter->time_stamp &&
          cur_cloud_measure.time_stamp <= second_iter->time_stamp) {
        double delta_time = second_iter->time_stamp - front_iter->time_stamp;
        if (std::abs(delta_time) < 0.00001)
          delta_time = 0.00001;
        double alpha = (cur_cloud_measure.time_stamp - front_iter->time_stamp) /
                       delta_time;

        curr_correspond_odom_pose_ =
            Mathbox::Interp_SE3(front_iter->pose, second_iter->pose, alpha);

        curr_linear =
            (1.0 - alpha) * front_iter->linear + alpha * second_iter->linear;
        curr_angular =
            (1.0 - alpha) * front_iter->angular + alpha * second_iter->angular;

        // LOG(WARNING) << "creatFrame: alpha: " << alpha;
        // LOG(WARNING) << "creatFrame: front  odom: " <<
        // front_iter->pose.block<3, 1>(0, 3).transpose(); LOG(WARNING) <<
        // "creatFrame: lidar  odom: " << curr_correspond_odom_pose_.block<3,
        // 1>(0, 3).transpose(); LOG(WARNING) << "creatFrame: second odom: " <<
        // second_iter->pose.block<3, 1>(0, 3).transpose();

        // const double &dt = second_iter->time_stamp - front_iter->time_stamp;
        // Mat34d delta_odom_pose = Mathbox::deltaPose34d(front_iter->pose,
        // second_iter->pose); end_2_begin =
        // Mathbox::Interp_SE3(Mathbox::Identity34(), delta_odom_pose, 0.1 /
        // dt);

        end_2_begin = Mathbox::deltaPose34d(curr_correspond_odom_pose_,
                                            last_correspond_odom_pose_);

        if (is_first_keyframe_) {
          end_2_begin = Mathbox::Identity34();
        }
        if (std::abs(temp_frame_time_ - last_undis_time_) > 0.15) {
          end_2_begin = Mathbox::Identity34();
        }

        find = true;

        last_correspond_odom_pose_ = curr_correspond_odom_pose_;
        last_undis_time_ = temp_frame_time_;

        break;
      }
      front_iter = second_iter;
    }

    if (!find) {
      LOG(ERROR) << "creatFrame: Can not find correspond odom data !!!";
      return false;
    }

    *input_cloud = *cur_cloud_measure.cloud;
    d_cloud_measures_.pop_front();
  }

  static Mat34d last_origin_odom_pose = Mathbox::Identity34();
  bool origin_keyframe_flag = false;
  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    if (!is_first_keyframe_) {
      Mat34d cur_frame_pose = ptr_cur_keyframe_->getPose();
      double dis_to_map = cur_frame_pose.block<3, 1>(0, 3).norm();
      if (dis_to_map < 3.0) {
        double delta_dis = Mathbox::deltaPose34d(last_origin_odom_pose,
                                                 curr_correspond_odom_pose_)
                               .block<3, 1>(0, 3)
                               .norm();
        if (delta_dis > 0.15) {
          origin_keyframe_flag = true;
          last_origin_odom_pose = curr_correspond_odom_pose_;
        }
      }
    }
  }

  if (std::abs(curr_angular) > ptr_config_->backend_config.angular_threshold &&
      origin_keyframe_flag == false) {
    return false;
  }

  // 运动距离或者角度超过设定值才提取特征进行优化；
  if (is_first_keyframe_ || origin_keyframe_flag ||
      !ptr_lidar_odom_->isJunkFrame(temp_frame_time_,
                                    curr_correspond_odom_pose_)) {
    TicToc feature_cost;

    corner_cloud_->clear();
    surf_cloud_->clear();
    corner_cloud_top_->clear();
    surf_cloud_top_->clear();
    raw_cloud_->clear();
    //非地面点用完后要清除
    cloud_for_occ_map_->clear();

    laserCloud::Ptr corner_cloud(new laserCloud());
    laserCloud::Ptr surf_cloud(new laserCloud());
    laserCloud::Ptr corner_cloud_top(new laserCloud());
    laserCloud::Ptr surf_cloud_top(new laserCloud());
    laserCloud::Ptr raw_cloud(new laserCloud());
    laserCloud::Ptr without_ground_cloud(new laserCloud());

    ptr_feature_extractor_->resetVariables();

    if (!ptr_feature_extractor_->setInputCloud(input_cloud)) {
      LOG(ERROR) << "ptr_feature_extractor_->setInputCloud failed !!!";
      return false;
    }

    if (find) {
      TicToc adjustDistortion_cost;
      // // 根据激光帧间增量进行点云畸变去除，转换到同一时间点下；
      // ptr_feature_extractor_->adjustDistortion(end_2_begin);

      // if (ptr_config_->print_debug)
      // {
      //   printf("--System::creatFrame: adjustDistortion_time: %.1f \n",
      //   adjustDistortion_cost.toc());
      // }
    }

    int environment_flag = 0;
    Vec6d h_matrix;
    h_matrix << 500, 500, 25000, 500, 500, 500;
    if (!ptr_feature_extractor_->cloudExtractor(corner_cloud, surf_cloud,
                                                cloud_for_occ_map_,
                                                environment_flag, h_matrix)) {
      LOG(ERROR) << "ptr_feature_extractor_->cloudExtractor failed !!!";
      return false;
    }

    *corner_cloud_ = *corner_cloud;
    *surf_cloud_ = *surf_cloud;
    *raw_cloud_ = *cloud_for_occ_map_;
    if (ptr_config_->occ_map_config.dynamic_cloud_remove) {
      *without_ground_cloud = *cloud_for_occ_map_;
    }

    // if (ptr_config_->print_debug)
    // {
    //   printf("--System::creatFrame: extract_feature_time: %.1f \n",
    //   feature_cost.toc());
    // }

    // LOG(WARNING) << "corner: " << corner_cloud_->size() << ", corner_top: "
    // << corner_cloud_top_->size() << ", surf: " << surf_cloud_->size() << ",
    // surf_top: " << surf_cloud_top_->size(); LOG(INFO) << "corner: " <<
    // corner_cloud_->size() << ", surf: " << surf_cloud_->size() << ", raw: "
    // << raw_cloud_->size();
    Mat34d correct_lidar_odom_pose = Mathbox::multiplePose34d(
        lidar_odom_to_odom_, curr_correspond_odom_pose_);
    Mat34d predict_cur_frame_pose =
        Mathbox::multiplePose34d(cur_map_to_odom_, correct_lidar_odom_pose);

    bool PLANE_MODE = true;
    if (PLANE_MODE == true) {
      Vec3d p_predict = predict_cur_frame_pose.block<3, 1>(0, 3);
      Mat3d r_predict = predict_cur_frame_pose.block<3, 3>(0, 0);
      Vec3d rpy_predict = Mathbox::rotation2rpy(r_predict);

      p_predict.z() = 0.0;
      rpy_predict.x() = 0.0;
      rpy_predict.y() = 0.0;
      r_predict = Mathbox::rpyToRotationMatrix(rpy_predict);

      predict_cur_frame_pose.block<3, 1>(0, 3) = p_predict;
      predict_cur_frame_pose.block<3, 3>(0, 0) = r_predict;
    }
    if (PLANE_MODE == true) {
      Vec3d p_predict = curr_correspond_odom_pose_.block<3, 1>(0, 3);
      Mat3d r_predict = curr_correspond_odom_pose_.block<3, 3>(0, 0);
      Vec3d rpy_predict = Mathbox::rotation2rpy(r_predict);

      p_predict.z() = 0.0;
      rpy_predict.x() = 0.0;
      rpy_predict.y() = 0.0;
      r_predict = Mathbox::rpyToRotationMatrix(rpy_predict);

      curr_correspond_odom_pose_.block<3, 1>(0, 3) = p_predict;
      curr_correspond_odom_pose_.block<3, 3>(0, 0) = r_predict;
    }

    ptr_cur_lidar_odom_keyframe_ = nullptr;

    // 根据提取的特征和预测值，创建关键帧；
    id_++;
    size_t corner_size = corner_cloud_->size();
    for (size_t i = 0; i < corner_size; i++) {
      // corner_cloud_->points[i].curvature = 0.0;  // init probability
      corner_cloud_->points[i].intensity = 80;         // init probability
      corner_cloud_->points[i].normal_x = time_stamp;  // update time
      corner_cloud_->points[i].normal_y = i;           // internal index
      corner_cloud_->points[i].normal_z = id_;         // keyFrame id
    }
    ptr_cur_lidar_odom_keyframe_ = std::make_shared<KeyFrame>(
        id_, time_stamp, curr_correspond_odom_pose_, predict_cur_frame_pose,
        curr_linear, curr_angular, corner_cloud_, surf_cloud_,
        without_ground_cloud, environment_flag, h_matrix);

    last_keyframe_time_ = curr_keyframe_time_;
    curr_keyframe_time_ = time_stamp;

    LOG(INFO) << "CreateFrame -- "
              << "id: " << id_ << ", corner: " << corner_cloud_->size()
              << ", surf: " << surf_cloud_->size()
              << ", raw: " << raw_cloud_->size()
              << ", time: " << create_frame_cost.toc() << " ms ";
    return true;
  } else {
    return false;
  }
}

bool System::failueDetect() {
  if (ptr_map_track_->getPoseCov() > 10.) {
    LOG(ERROR) << "pose cov is too large";
    return true;
  }
  // TODO:第一次地图的转换会导致delta很大
  Mat34d delta_trans =
      Mathbox::deltaPose34d(last_map_to_odom_, cur_map_to_odom_);
  last_map_to_odom_ = cur_map_to_odom_;
  double delta_rot =
      Mathbox::rotationMatrixToEulerAngles(delta_trans.block<3, 3>(0, 0))
          .norm();
  double delta_translation = delta_trans.block<3, 1>(0, 3).norm();
  if (delta_rot > 0.2) {
    LOG(ERROR) << "Too big rotation  " << delta_rot;
    return true;
  } else if (delta_translation < 0.) {  // 暂时关闭此检测
    LOG(ERROR) << "Too big translation  " << delta_translation;
    LOG(INFO) << "last_map_to_odom_: \n" << last_map_to_odom_;
    LOG(INFO) << "cur_map_to_odom_: \n" << cur_map_to_odom_;
    return true;
  } else {
    return false;
  }
}

void System::addCloudFrame(const laserCloud::Ptr cloud_in,
                           const double time_stamp) {
  static double _first_cloud_time = 0.0;

  std::lock_guard<std::mutex> lock(cloud_data_mutex_);

  if (is_first_cloud_ == true) {
    _first_cloud_time = time_stamp;
    is_first_cloud_ = false;
  }

  // d_cloud_measures_ 用来保存添加的激光点云
  // TODO:继续构图后，保存地图时应该清除buf；
  if (d_cloud_measures_.size() > 100) {
    d_cloud_measures_.pop_front();
    LOG(INFO) << "addCloudFrame: cloud size > 100,  throw old cloud data... "
              << "time_stamp: " << (time_stamp - _first_cloud_time);
  }
  d_cloud_measures_.emplace_back(cloud_in, time_stamp);
  lidar_count_++;

  // map_tracker_cv_.notify_one();
}

void System::addOdom(const OdomMeasure &odom) {
  static Mat34d _last_raw_odom_pose = Mathbox::Identity34();
  static Mat34d _last_smooth_pose = Mathbox::Identity34();
  static Mat34d _last_lidar_odom_to_odom = Mathbox::Identity34();
  static Mat34d _last_map_to_odom = Mathbox::Identity34();
  static double fusion_pose_smooth_ratio =
      ptr_config_->backend_config.fusion_pose_smooth_ratio;

  std::lock_guard<std::mutex> lock(odom_data_mutex_);
  if (d_odom_measures_.size() > 500) {
    d_odom_measures_.pop_front();
    LOG(WARNING) << "addOdom: odom size > 500,  throw old odom data..."
                 << "time_stamp: " << odom.time_stamp;
  }
  d_odom_measures_.emplace_back(odom);
  odom_count_++;

  if (d_odom_for_lidar_odom_.size() > 500) {
    d_odom_for_lidar_odom_.pop_front();
  }
  d_odom_for_lidar_odom_.emplace_back(odom);

  Mat34d correct_lidar_odom_pose =
      Mathbox::multiplePose34d(lidar_odom_to_odom_, odom.pose);
  Mat34d correct_map_pose =
      Mathbox::multiplePose34d(cur_map_to_odom_, correct_lidar_odom_pose);

  if (is_first_odom_ == true) {
    if (odom_count_ < 50) {
      _last_raw_odom_pose = odom.pose;
      _last_smooth_pose = correct_map_pose;
      is_first_odom_ = false;
    }
  }
  // AD_STATU robot_state = ptr_state_machine_->getCurrentState();
  // if (AD_STATU::INIT_LOCALIZATION == robot_state || AD_STATU::UNKNOW ==
  // robot_state || is_stop_requested_)
  // {
  //   _last_smooth_pose = correct_map_pose;
  // }

  Mat34d delta_odom = Mathbox::deltaPose34d(_last_raw_odom_pose, odom.pose);

  if (ptr_lidar_odom_->isWheelStable() == false) {
    delta_odom = Mathbox::Identity34();
  }
  Mat34d predict_robot_pose =
      Mathbox::multiplePose34d(_last_smooth_pose, delta_odom);
  Mat34d smooth_robot_pose =
      Mathbox::Interp_SE3(predict_robot_pose, correct_map_pose,
                          fusion_pose_smooth_ratio);  // 0.01

  cur_robot_state_.pose = smooth_robot_pose;
  cur_robot_state_.time_stamp = odom.time_stamp;

  _last_smooth_pose = smooth_robot_pose;
  _last_raw_odom_pose = odom.pose;

  // Eigen::Vector3d cur_rpy =
  // Mathbox::rotation2rpy(Eigen::Matrix3d(odom.Q_gyr_only)) * Rad2Deg;
  // printf("--System::addOdom: Q_gyr_only: r:%.1f, p:%.1f, yaw:%.1f \n",
  // cur_rpy.x(), cur_rpy.y(), cur_rpy.z());
}

void System::addIMU(const ImuMeasure &IMU_t) {}

void System::requestStop() {
  is_stop_requested_ = true;
  ptr_loop_track_->requestStop();
  usleep(100000);  // sleep for safety stop
  // map_tracker_cv_.notify_one();
}

void System::Reset() {
  is_reset_ = true;
  last_map_to_odom_ = Mathbox::Identity34();
  cur_map_to_odom_ = Mathbox::Identity34();
  lidar_odom_to_odom_ = Mathbox::Identity34();
  last_lidar_odom_to_odom_ = Mathbox::Identity34();

  cur_robot_state_.Reset();
  is_first_cloud_ = true;
  is_first_odom_ = true;
  is_first_keyframe_ = true;
  new_keyframe_flag_ = false;
  is_localization_init_ = false;
  is_opt_finished_ = false;
  last_opt_finish_time_ = std::chrono::system_clock::now();
  init_mode_ = INIT_MODEL::NOTINIT;
  id_ = 0;
  last_msf_id_ = 0;
  lost_frame_count_ = 0;
  {
    std::lock_guard<std::mutex> lock(odom_data_mutex_);
    d_odom_measures_.clear();
    std::deque<OdomMeasure>().swap(d_odom_measures_);
  }
  {
    std::lock_guard<std::mutex> lock(odom_data_mutex_);
    d_odom_for_lidar_odom_.clear();
    std::deque<OdomMeasure>().swap(d_odom_for_lidar_odom_);
  }
  {
    std::lock_guard<std::mutex> lock(odom_data_mutex_);
    d_lidar_odom_.clear();
    std::deque<OdomMeasure>().swap(d_lidar_odom_);
  }
  {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    d_gps_measures_.clear();
    std::deque<GPSMeasure>().swap(d_gps_measures_);
  }
  {
    std::lock_guard<std::mutex> lock(cloud_data_mutex_);
    d_cloud_measures_.clear();
    std::deque<CloudMeasure>().swap(d_cloud_measures_);
  }
  odom_count_ = 0;
  imu_count_ = 0;
  lidar_count_ = 0;

  ptr_lidar_odom_->Reset();
  ptr_map_track_->Reset();
  ptr_map_manager_->Reset();
  ptr_loop_track_->Reset();
  ptr_msf_->Reset();
  ptr_depth_occ_map_->Reset();
  ptr_occ_map_->Reset();
  if (ptr_config_->occ_map_config.dynamic_cloud_remove) {
    ptr_local_occ_map_->Reset();
  }
  is_stop_requested_ = false;
  malloc_trim(0);
  LOG(WARNING) << "System: Reset all data !!!" << std::endl;
}

void System::shutdown() {
  d_odom_measures_.clear();
  d_odom_for_lidar_odom_.clear();
  d_lidar_odom_.clear();
  d_gps_measures_.clear();
  d_cloud_measures_.clear();

  ptr_lidar_odom_->requestStop();
  ptr_lidar_odom_->Stop();
  ptr_map_track_->requestStop();
  ptr_map_track_->Stop();
  ptr_loop_track_->requestStop();
  ptr_loop_track_->stop();
  run_opt_ = false;
  is_stopped_ = true;
  has_new_keyframe_ = false;
  // map_tracker_cv_.notify_one();
  if (ptr_maptrack_thread_->joinable()) {
    ptr_maptrack_thread_->join();
  }
  if (ptr_looptrack_thread_->joinable()) {
    ptr_looptrack_thread_->join();
  }
  if (nullptr != ptr_msf_thread_ && ptr_msf_thread_->joinable()) {
    ptr_msf_thread_->join();
  }
  if (nullptr != ptr_occ_map_thread_ && ptr_occ_map_thread_->joinable()) {
    ptr_occ_map_thread_->join();
  }
  if (nullptr != ptr_dynamic_local_map_thread_ &&
      ptr_dynamic_local_map_thread_->joinable()) {
    ptr_dynamic_local_map_thread_->join();
  }
  if (nullptr != ptr_init_localization_thread_ &&
      ptr_init_localization_thread_->joinable()) {
    ptr_init_localization_thread_->join();
  }
  if (nullptr != ptr_map_update_thread_ && ptr_map_update_thread_->joinable()) {
    ptr_map_update_thread_->join();
  }
  LOG(ERROR) << "system stopped";
  LOG(ERROR) << "Lost frame number:  " << lost_frame_count_;
}

void System::saveTrajectory() {}

void System::saveMap() {
  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    ptr_map_manager_->saveKeyFrame(ptr_config_->backend_config.map_path);
  } else {
    LOG(WARNING) << " No map save in localization mode !!!" << std::endl;
  }
}

void System::saveMap(const std::string &map_dir) {
  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    LOG(WARNING) << " start save map !!!" << std::endl;

    std::lock_guard<std::mutex> lock_odom(odom_data_mutex_);
    std::lock_guard<std::mutex> lock_cloud(cloud_data_mutex_);

    const std::string &gps_path = map_dir + "/gps/";
    const std::string &map_3d_path = map_dir + "/3d_map/";
    const std::string &map_2d_path = map_dir + "/2d_map/";
    const std::string &lidar_feature_path = map_dir + "/lidar_feature/";

    // 保存每个关键帧的角点和平面点云，pose graph，整个地图的角点和平面点云；
    if (!save_keyframe_thread_.joinable()) {
      save_keyframe_thread_ = std::thread(
          std::bind(&System::saveKeyFrameThread, this, std::placeholders::_1),
          map_3d_path);
    }

    if (!save_map_thread_.joinable()) {
      save_map_thread_ = std::thread(
          std::bind(&System::saveMapThread, this, std::placeholders::_1),
          map_2d_path);
    }

    if (!save_scandata_thread_.joinable()) {
      save_scandata_thread_ = std::thread(
          std::bind(&System::saveScanThread, this, std::placeholders::_1),
          map_2d_path);
    }

    while (!finish_save_scandata_ || !finish_save_keyframe_ ||
           !finish_save_map_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (save_keyframe_thread_.joinable()) {
      save_keyframe_thread_.join();
    }

    if (save_map_thread_.joinable()) {
      save_map_thread_.join();
    }

    if (save_scandata_thread_.joinable()) {
      save_scandata_thread_.join();
    }

    ptr_occ_map_->Reset();

  } else {
    LOG(WARNING) << " No map save in localization mode !!!" << std::endl;
  }
}

void System::saveScanThread(const std::string &map_2d_path) {
  auto savedscan_start_time = std::chrono::system_clock::now();
  finish_save_scandata_ = false;
  ptr_occ_map_->saveScanData(map_2d_path);
  finish_save_scandata_ = true;
  auto savedscan_end_time = std::chrono::system_clock::now();

  std::chrono::duration<double> savedscan_cost_time =
      savedscan_end_time - savedscan_start_time;

  LOG(INFO) << " >>----------------savedscan_cost_time : "
            << savedscan_cost_time.count() * 1000;
}

void System::saveKeyFrameThread(const std::string &map_3d_path) {
  auto save_keyframeFile_start_time = std::chrono::system_clock::now();
  finish_save_keyframe_ = false;
  ptr_map_manager_->saveAllKeyFrame(map_3d_path, temp_cloud_filepath_);
  ptr_msf_->saveGraph(map_3d_path);
  finish_save_keyframe_ = true;
  auto save_keyframeFile_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> keyframeFile_cost_time =
      save_keyframeFile_end_time - save_keyframeFile_start_time;

  LOG(INFO) << ">>----------------keyframeFile_cost_time: "
            << keyframeFile_cost_time.count() * 1000;
}

void System::saveMapThread(const std::string &map_2d_path) {
  std::lock_guard<std::mutex> lock(occ_save_mutex_);
  auto save2dmap_start_time = std::chrono::system_clock::now();
  finish_save_map_ = false;
  if (!ptr_config_->use_depth_cloud) {
    ptr_occ_map_->clearAndSaveMap(map_2d_path);
  } else {
    ptr_occ_map_->clearAndUpdateMap();
    UserOccupancyGrid global_occ_map = ptr_occ_map_->getCurrOccMap();
    ptr_depth_occ_map_->updateGlobalMap(ptr_map_manager_->getKeyFrameDatabase(),
                                        global_occ_map);
    ptr_depth_occ_map_->saveMap(map_2d_path);
  }
  finish_save_map_ = true;
  auto save2dmap_end_time = std::chrono::system_clock::now();

  std::chrono::duration<double> save2dmap_cost_time =
      save2dmap_end_time - save2dmap_start_time;

  LOG(INFO) << ">>----------------save2dmap_cost_time: "
            << save2dmap_cost_time.count() * 1000;
}

bool System::loadMap() {
  FramePosition pose;
  pose.x = 0.;
  pose.y = 0.;
  pose.z = 0.;

  is_local_map_ready_ = ptr_map_manager_->loadLocalKeyFrame(
      ptr_config_->backend_config.map_path, pose);
  if (!is_local_map_ready_) {
    return false;
  }
  return true;
}

// 定位模式下，启动时加载所有的关键帧点云　和　整个地图点云；并且直接将整个地图点云构建成kdtree，供定位时匹配搜索；
bool System::loadMap(const std::string &map_dir) {
  FramePosition pose;
  pose.x = 0.;
  pose.y = 0.;
  pose.z = 0.;

  is_local_map_ready_ = ptr_map_manager_->loadLocalKeyFrame(map_dir, pose);
  if (!is_local_map_ready_) {
    return false;
  }
  return true;
}

bool System::loadWholeData(const std::string &map_dir) {
  if (AD_STATU::LOCALIZATION == ptr_state_machine_->getCurrentState()) {
    const std::string &gps_path = map_dir + "/gps/";
    const std::string &map_3d_path = map_dir + "/3d_map/";
    const std::string &map_2d_path = map_dir + "/2d_map/";

    // 恢复system 状态
    size_t last_keyframe_id = ptr_map_manager_->getLatestFrameID();
    id_ = last_keyframe_id + 1;  // FRAME ID 从上次最后一帧接着构图。
    is_first_keyframe_ = true;
    new_keyframe_flag_ = false;

    // 恢复occ map
    ptr_depth_occ_map_->Reset();
    ptr_occ_map_->Reset();
    if (!ptr_occ_map_->loadScan(map_2d_path)) {
      LOG(ERROR) << "Failed to load scan data.";
      return false;
    }
    if (!ptr_depth_occ_map_->loadScan(map_2d_path)) {
      LOG(WARNING) << "Failed to load depth data.";
    }

    // 恢复 map manager
    // 恢复graph
    ptr_msf_->Reset();
    if (!ptr_msf_->loadGraph(map_3d_path)) {
      LOG(WARNING) << "Failed to load loop info.";
    }
    auto v_keyframes = ptr_map_manager_->getAllKeyFrames();
    for (const auto &frame : v_keyframes) {
      if (nullptr != frame) {
        ptr_msf_->addMapKeyFrame(frame);
      }
    }

    // 恢复 map track
    ptr_map_track_->Reset();
    ptr_map_track_->setMap2Odom(cur_map_to_odom_, false);

    return true;
  } else {
    LOG(ERROR) << "This function only supports [ UPDATE_MAPPING ] status.";
    return false;
  }
}

bool System::getViewCloud(laserCloud::Ptr &view_cloud,
                          laserCloud::Ptr &frame_pose) {
  if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    if (ptr_map_track_->isMapReady()) {
      ptr_map_track_->setMapStatus(false);
      view_cloud = ptr_map_manager_->getGlobalCloud();  // TODO:没有加锁，不安全
      frame_pose = ptr_map_manager_->getKeyPoses();  // TODO:没有加锁，不安全
      return true;
    }
  } else if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    view_cloud = ptr_map_track_->getCornerMapCloud();
    frame_pose = ptr_map_manager_->getKeyPoses();
    return true;
  }

  return false;
}

laserCloud::Ptr System::getWholeMap() {
  return ptr_map_manager_->getSurfMapCloud();
}

laserCloud::Ptr System::getCurrSurroundCloud() {
  return ptr_map_track_->getCurrSurroundMap();
}

laserCloud::Ptr System::getCurrKeyPoses() {
  return ptr_map_manager_->getKeyPoses();
}

UserOccupancyGrid System::getCurrOccMap() {
  std::lock_guard<std::mutex> lock(occ_save_mutex_);
  return ptr_occ_map_->getCurrOccMap();
}

UserOccupancyGrid System::getCurrDepthOccMap() {
  std::lock_guard<std::mutex> lock(occ_save_mutex_);
  return ptr_depth_occ_map_->getCurrOccMap();
}

double System::getCurKeyFrameTimeStamp() {
  double t = 0.0;
  if (ptr_cur_keyframe_ != nullptr) {
    t = ptr_cur_keyframe_->time_stamp_;
  }
  return t;
}

const RobotState &System::getRobotState() {
  AD_STATU state = ptr_state_machine_->getCurrentState();
  if (AD_STATU::MAPPING == state || AD_STATU::LOCALIZATION == state) {
    return cur_robot_state_;
  } else {
    cur_robot_state_.Reset();
    return cur_robot_state_;
  }
}

const RobotState System::getLidarOdomState() {
  return cur_lidar_odom_state_;
}

const Mat34d System::getKeyFramePose() {
  Mat34d pose;
  if (ptr_cur_keyframe_ != nullptr) {
    pose = ptr_cur_keyframe_->T_wb_;
  }
  return pose;
}

const Mat34d System::getKeyFrameLidarOdomPose() {
  Mat34d pose;
  if (ptr_cur_keyframe_ != nullptr) {
    pose = ptr_cur_keyframe_->T_wl_;
  }
  return pose;
}

long unsigned int System::getLatestFrameID() {
  return ptr_map_manager_->getLatestFrameID();
}

bool System::hasKeyFrame() {
  return ptr_map_manager_->hasKeyFrame();
}

bool System::isNewKeyFrame() {
  return has_new_keyframe_;
}

std::vector<std::pair<long unsigned int, Mat34d>>
System::getKeyFrameDataPose() {
  return ptr_map_manager_->getKeyFrameDataPose();
}

std::shared_ptr<std::list<LoopConstrain>> System::getLoopInfo() {
  return ptr_msf_->getLoopConstraints();
}

bool System::getPoseFromID(const long unsigned int frame_id, Mat34d &pose34d) {
  return ptr_map_manager_->getPoseFromID(frame_id, pose34d);
}

double System::getPoseCov() {
  return ptr_map_track_->getPoseCov();
}

bool System::isLocalizationInit() {
  return is_localization_init_;
}

bool System::isLocalizationSuccess() {
  return is_match_ok_;
}

bool System::getLocalizationStatus() {
  if (AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode()) {
    if (ptr_map_track_->getmatchFailedCount() >
            ptr_config_->backend_config.max_match_failed ||
        ptr_map_track_->getLowOverlapCount() >
            ptr_config_->backend_config.max_low_overlap) {
      LOG(ERROR) << "localization failed ! matchFailedCount: "
                 << ptr_map_track_->getmatchFailedCount()
                 << " , LowOverlapCount: "
                 << ptr_map_track_->getLowOverlapCount();

      is_localization_sucess_ = false;
    } else {
      is_localization_sucess_ = true;
    }
  }
  return is_localization_sucess_ && is_init_localization_sucess_;
}

}  // namespace cvte_lidar_slam
