
#include "system.hpp"

#include "common/debug_tools/debug_color.h"
#include "common/debug_tools/tic_toc.h"
#include "frontend/feature_extract_factory.hpp"
#include "malloc.h"
#include "opencv2/opencv.hpp"

namespace cvte_lidar_slam {
System *System::ptr_system_ = nullptr;

System::System() {
  ptr_state_machine_ = SlamStateMachine::getInstance();
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
  try_lidar_feature_count_ = 0;
  init_mode_ = init_model;
}

void System::setInitPose(const Mat34d &init_pose) {
  init_pose_ = init_pose;
}

void System::setReliablePose(const Mat34d &reliable_pose) {
  cur_map_to_odom_ = reliable_pose;
  cur_robot_state_.pose = cur_map_to_odom_;
  is_localization_init_ = true;
}

void System::InitParameters() {
  is_stopped_ = false;
  is_stop_requested_ = false;
  is_localization_init_ = false;
  is_gps_param_ok_;
  is_first_keyframe_ = true;
  has_new_keyframe_ = false;
  is_update_map_ = false;
  is_opt_finished_ = false;
  run_opt_ = true;
  id_ = 0;
  last_msf_id_ = 0;
  lost_frame_count_ = 0;
  init_mode_ = INIT_MODEL::NOTINIT;
  d_odom_measures_.clear();
  d_gps_measures_.clear();
  d_cloud_measures_.clear();
  last_map_to_odom_ = Mathbox::Identity34();
  cur_map_to_odom_ = Mathbox::Identity34();
  cur_robot_state_.Reset();

  view_cloud_.reset(new laserCloud());
  global_cloud_.reset(new laserCloud());
  frame_pose_.reset(new laserCloud());
  cloud_for_occ_map_.reset(new laserCloud());
  cloud_for_iris_.reset(new laserCloud());

  // construct modules
  ptr_map_manager_ = MapManager::getInstance();

  ptr_map_manager_->initParameters(ptr_config_->backend_config);

  ptr_map_track_ = std::make_shared<MapTrack>(&ptr_config_->backend_config);

  ptr_sc_manager_ = std::make_shared<SCManager>();

  ptr_feature_extractor_ = FeatureExtractorFactory::creatExtractor(
      ptr_config_->backend_config.lidar_type);

  ptr_loop_track_ = std::make_shared<LoopTrack>(&ptr_config_->loop_config);

  ptr_msf_ = std::make_shared<MutliSensorsFusion>(ptr_config_->msf_config);

  ptr_occ_map_ = OccMap::getInstance();

  ptr_occ_map_->setConfig(ptr_config_->occ_map_config);
  ptr_occ_map_->setExtrincs(T_lo_);

  // start threads
  run_opt_ = true;
  ptr_msf_thread_ = new std::thread(&System::msfThread, this);
  ptr_looptrack_thread_ = new std::thread(&System::mapTrackThread, this);
  ptr_maptrack_thread_ = new std::thread(&System::loopTrackThread, this);
  ptr_occ_map_thread_ = new std::thread(&System::occupancyMapThread, this);
  ptr_init_localization_thread_ =
      new std::thread(&System::initLocalizationThread, this);
  // TODO: 不要打开下面这行
  // ptr_map_update_thread_ = new std::thread(&System::mapUpdateThread, this);
}

bool System::setInitKeyFrameIdList(const std::list<unsigned int> ids) {
  std::list<Vec3d> init_pos_list;
  Vec3d init_pos;
  for (auto id = ids.begin(); id != ids.end(); id++) {
    // TODO: 如果找不到需要尝试+1和-1
    if (ptr_map_manager_->getPositionFromId(*id, init_pos)) {
      LOG(INFO) << "set init pos　by id - " << *id << " :\n" << init_pos;
      std::cout << "set init pos　by id - " << *id << " :\n"
                << init_pos << std::endl;
      init_pos_list.push_back(init_pos);
    } else if (ptr_map_manager_->getPositionFromId(*id - 1, init_pos)) {
      LOG(INFO) << "set init pos　by id - " << *id - 1 << " :\n" << init_pos;
      std::cout << "set init pos　by id - " << *id - 1 << " :\n"
                << init_pos << std::endl;
      init_pos_list.push_back(init_pos);
    } else if (ptr_map_manager_->getPositionFromId(*id + 1, init_pos)) {
      LOG(INFO) << "set init pos　by id - " << *id + 1 << " :\n" << init_pos;
      std::cout << "set init pos　by id - " << *id + 1 << " :\n"
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

void System::msfThread() {
  while (!is_stopped_) {
    // if (AD_MODE::AD_UNKNOW == ptr_state_machine_->getCurrentMode()) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(30));
    //   continue;
    // }
    if (!is_stop_requested_ &&
        AD_MODE::AD_LOCALIZATION == ptr_state_machine_->getCurrentMode() &&
        ptr_msf_->optimization()) {
      LOG(INFO) << "-------- opt ---------" << std::endl;
      Mat34d msf_map_pose = ptr_msf_->getLastNodePose();
      Mat34d msf_odom_pose = ptr_msf_->getLastOdomPose();
      cur_map_to_odom_ = Mathbox::multiplePose34d(
          msf_map_pose, Mathbox::inversePose34d(msf_odom_pose));
      ptr_map_track_->setMap2Odom(cur_map_to_odom_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void System::mapTrackThread() {
  while (!is_stopped_) {
    AD_STATU robot_state = ptr_state_machine_->getCurrentState();
    if (AD_STATU::INIT_LOCALIZATION == robot_state ||
        AD_STATU::UNKNOW == robot_state || is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }
    // LOG(WARNING) << "Robot State " << robot_state;
    if (!creatFrame()) {
      // std::cout << "Failed to create frame" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    TicToc process_keyframe_cost;
    if (ptr_map_track_->processNewKeyFrame(ptr_cur_keyframe_, is_update_map_)) {
      if (AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
        // TODO:第一帧不应该更新cur_map_to_odom_
        if (!is_first_keyframe_) {
          cur_map_to_odom_ = ptr_map_track_->getMap2Odom();
        }

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
        has_new_keyframe_ = true;
        //非地面点用完后要清除
        cloud_for_occ_map_->clear();
        if (ptr_map_track_->isKeyFrame() || is_first_keyframe_) {
          if (is_first_keyframe_) {
            is_first_keyframe_ = false;
          }
          ptr_map_manager_->Add(ptr_cur_keyframe_);
          static unsigned int count = 0;
          if (++count % 3 == 0) {
            ptr_loop_track_->insertKeyFrame(ptr_cur_keyframe_);
          }

          if (ptr_msf_ != nullptr) {
            ptr_msf_->addKeyFrame(ptr_cur_keyframe_);

            // TODO: do opt outside of map track
            if (ptr_msf_->optimization()) {
              std::cout << "---------------- opt ------------------------"
                        << std::endl;
              Mat34d msf_map_pose = ptr_msf_->getLastNodePose();
              Mat34d msf_odom_pose = ptr_msf_->getLastOdomPose();
              cur_map_to_odom_ = Mathbox::multiplePose34d(
                  msf_map_pose, Mathbox::inversePose34d(msf_odom_pose));
              ptr_map_track_->setMap2Odom(cur_map_to_odom_);
              ptr_map_manager_->Update();

              const double dis_to_map = msf_map_pose.block<3, 1>(0, 3).norm();
              // 有闭环的情况，且要求距离地图原点距离在一定阈值内
              if (dis_to_map < 5.) {
                is_opt_finished_ = true;
              }
              last_msf_id_ = ptr_cur_keyframe_->index_;
              is_update_map_ = true;

            } else {
              is_update_map_ = false;
            }
          }
          // 将关键帧对应的原始点云插入到重定位特征计算模块
          if (!cloud_for_iris_->points.empty()) {
            clock_t startTime = clock();
            ptr_sc_manager_->makeAndSaveScancontextAndKeys(*cloud_for_iris_);
            sc_frame_ids_.push_back(ptr_cur_keyframe_->index_);
            clock_t endTime = clock();
            LOG(INFO) << "makeAndSaveScancontextAndKeys "
                      << ptr_cur_keyframe_->index_ << ","
                      << (endTime - startTime) / (double) CLOCKS_PER_SEC
                      << "s.";
          }
        }
      } else if (AD_MODE::AD_LOCALIZATION ==
                 ptr_state_machine_->getCurrentMode()) {
        // TODO: 如果出现打滑，需要及时更新cur_map_to_odom_
        if (ptr_cur_keyframe_->is_wheel_skidded_) {
          cur_map_to_odom_ = ptr_map_track_->getMap2Odom();
        }
        if (ptr_map_track_->isKeyFrame(true) || is_first_keyframe_) {
          if (is_first_keyframe_) {
            is_first_keyframe_ = false;
          }
          if (ptr_msf_ != nullptr) {
            ptr_msf_->addKeyFrame(ptr_cur_keyframe_);
          }
        }
      } else {
        LOG(ERROR) << "mode error.";
      }
    } else {
      LOG(WARNING) << "Process frame failed.";
    }
    LOG(INFO) << "process one keyframe cost: " << process_keyframe_cost.toc();
  }
}

void System::loopTrackThread() {
  while (!is_stopped_) {
    if (AD_MODE::AD_UNKNOW == ptr_state_machine_->getCurrentMode() ||
        is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }
    TicToc loop_cost;
    if (!ptr_loop_track_->checkNewKeyFrames()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    auto ptr_cur_loop_frame = ptr_loop_track_->getCurKeyFramePtr();

    Mat34d cur_frame_pose = ptr_cur_loop_frame->getPose();
    clock_t startTime = clock();
    auto detectResult =
        ptr_sc_manager_
            ->detectLoopClosureID();  // first: nn index, second: yaw diff
    clock_t endTime = clock();
    long int history_frame_id = -1;
    if (detectResult.first < 0) {
      history_frame_id = -1;
    } else {
      history_frame_id = sc_frame_ids_[detectResult.first];
    }
    LOG(INFO) << "detectLoopClosureID " << history_frame_id << ","
              << ptr_cur_loop_frame->index_ << ","
              << (endTime - startTime) / (double) CLOCKS_PER_SEC << "s.";
    if (history_frame_id <= 0) {
      history_frame_id = ptr_loop_track_->getLoopCandidateID(cur_frame_pose);
      LOG(INFO) << "Can not history_frame_id pose" << history_frame_id;
    } else {
      Mat34d loop_frame_pose;
      bool is_get_pose = false;
      if (!ptr_map_manager_->getPoseFromID(history_frame_id, loop_frame_pose)) {
        LOG(ERROR) << "Can not find pose from history_frame_id";
        continue;
      } else {
        is_get_pose = true;
      }
      if (is_get_pose) {
        double dis =
            std::sqrt((loop_frame_pose(0, 3) - cur_frame_pose(0, 3)) *
                          (loop_frame_pose(0, 3) - cur_frame_pose(0, 3)) +
                      (loop_frame_pose(0, 3) - cur_frame_pose(0, 3)) *
                          (loop_frame_pose(0, 3) - cur_frame_pose(0, 3)));
        if (dis > 20 ||
            fabs(ptr_cur_loop_frame->index_ - history_frame_id) < 100) {
          LOG(ERROR) << "dis " << dis << ", " << history_frame_id << ","
                     << ptr_cur_loop_frame->index_;
          history_frame_id =
              ptr_loop_track_->getLoopCandidateID(cur_frame_pose);
        }
      } else {
        LOG(ERROR) << "Can not find pose getLoopCandidateID";
        history_frame_id = ptr_loop_track_->getLoopCandidateID(cur_frame_pose);
      }
    }

    if (history_frame_id > 0) {
      Mat34d loop_frame_pose;
      if (!ptr_map_manager_->getPoseFromID(history_frame_id, loop_frame_pose)) {
        LOG(ERROR) << "Can not find pose";
        continue;
      }
      // cur_frame_pose(2,3) = loop_frame_pose(2,3); //
      // 这里直接用闭环候选帧的高度，因为闭环的时候因该是在同一高度
      double cov;
      Mat34d transformation;
      bool correct_loop_succeed;
      correct_loop_succeed = ptr_loop_track_->correctLoop(
          history_frame_id, cov, cur_frame_pose, transformation, last_msf_id_);
      if (correct_loop_succeed) {
        Mat34d transform_in_odom =
            Mathbox::multiplePose34d(T_lo_, transformation);
        transform_in_odom = Mathbox::multiplePose34d(
            transform_in_odom, Mathbox::inversePose34d(T_lo_));
        Mat34d correct_frame_pose =
            Mathbox::multiplePose34d(transform_in_odom, cur_frame_pose);

        LoopConstrain loop_constrain;
        loop_constrain.loop_node_id1 = history_frame_id;
        loop_constrain.loop_node_id2 = ptr_cur_loop_frame->index_;
        loop_constrain.ptr_keyframe1 =
            ptr_map_manager_->getKeyFrameFromID(history_frame_id);
        loop_constrain.ptr_keyframe2 = ptr_cur_loop_frame;
        loop_constrain.relative_pose =
            Mathbox::deltaPose34d(loop_frame_pose, correct_frame_pose);
        loop_constrain.rp_cov = 1000 * cov * Mat6d::Identity();

        if (ptr_msf_ != nullptr && !ptr_msf_->msfRunning()) {
          ptr_msf_->addLoop(loop_constrain);
        }
      }
    }
    LOG(INFO) << "***************loop track cost: " << loop_cost.toc()
              << "*******************";
  }
}

void System::occupancyMapThread() {
  while (!is_stopped_) {
    AD_STATU robot_state = ptr_state_machine_->getCurrentState();
    if (AD_STATU::UNKNOW == robot_state || is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }
    if (AD_STATU::MAPPING == robot_state) {
      TicToc occ_cost;
      occ_save_mutex_.lock();
      if (isMapUpdate()) {
        LOG(WARNING) << "map update....";
        ptr_occ_map_->resetUpdateValue();
      }
      if (ptr_occ_map_->updateGlobalMap()) {
        ptr_occ_map_->saveGlobalMap();
      }
      occ_save_mutex_.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}

bool System::initLocalizationByLidarFeature(const laserCloud::Ptr cloud_in,
                                            const FramePosition &pose,
                                            Vec3d &init_predict_pose,
                                            double &init_predict_yaw) {
  std::pair<int, float> compare_result;
  compare_result.first = -1;
  int bias;
  clock_t startTime = clock();
  LOG(INFO) << "start find Nearest Frame ";
  std::vector<size_t> v_frame_id =
      ptr_map_manager_->DetectCandidatesIdByHorizontalDistance(pose, 50.0);
  std::vector<size_t> search_frame_id;
  LOG(INFO) << "v_frame_id size " << v_frame_id.size();
  for (size_t i = 0; i < v_frame_id.size(); i++) {
    std::vector<long int>::iterator it =
        find(sc_frame_ids_.begin(), sc_frame_ids_.end(), v_frame_id[i]);

    if (it != sc_frame_ids_.end()) {
      int nPosition = std::distance(sc_frame_ids_.begin(), it);
      search_frame_id.push_back(nPosition);
    } else {
      continue;
    }
  }

  try_lidar_feature_count_++;
  auto detectResult =
      ptr_sc_manager_->findNearestID(*cloud_in, search_frame_id);
  if (detectResult.first < 0) {
    compare_result.first = -1;
  } else {
    compare_result.first = sc_frame_ids_[detectResult.first];
  }

  if (compare_result.first < 0) {
    return false;
  }
  double bias_angle = -1 * detectResult.second;
  LOG(INFO) << "FramePosition " << pose.x << "," << pose.y << "," << pose.z;
  Vec3d euler_angle;
  Vec3d vec3d_pose;
  euler_angle(0) = bias_angle;
  euler_angle(1) = 0;
  euler_angle(2) = 0;
  vec3d_pose(0) = 0;
  vec3d_pose(1) = 0;
  vec3d_pose(2) = 0;
  Mat34d bias_pose = Mathbox::Euler2Mat34d(euler_angle, vec3d_pose);
  LOG(INFO) << "compare_result " << compare_result.first << ","
            << compare_result.second << "," << bias_angle << "," << bias;
  clock_t endTime = clock();
  LOG(INFO) << "findNearestFrame "
            << (endTime - startTime) / (double) CLOCKS_PER_SEC << "s.";
  Mat34d frame_pose;
  if (compare_result.second < 0.35 &&
      ptr_map_manager_->getPoseFromID(compare_result.first, frame_pose)) {
    init_predict_pose = frame_pose.block<3, 1>(0, 3);
    Mat3d cur_R = frame_pose.block<3, 3>(0, 0) * bias_pose.block<3, 3>(0, 0);
    Vec3d frame_angle =
        Mathbox::rotationMatrixToEulerAngles(frame_pose.block<3, 3>(0, 0));
    Vec3d angle = Mathbox::rotationMatrixToEulerAngles(cur_R);
    init_predict_yaw = angle(2);
    LOG(INFO) << "init pose " << init_predict_pose(0) << ","
              << init_predict_pose(1) << "," << init_predict_pose(2) << ","
              << init_predict_yaw << "," << frame_angle(2);
    return true;
  } else {
    return false;
  }
}

void System::initLocalizationThread() {
  while (!is_stopped_) {
    if (AD_STATU::INIT_LOCALIZATION != ptr_state_machine_->getCurrentState() ||
        is_stop_requested_ || is_localization_init_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }
    laserCloud::Ptr input_cloud(new laserCloud());
    {
      std::unique_lock<std::mutex> lock(cloud_data_mutex_);
      map_tracker_cv_.wait(lock, [this] {
        return !d_cloud_measures_.empty() || is_stopped_ || is_stop_requested_;
      });

      if (is_stop_requested_) {
        LOG(WARNING) << "request stop in initLocalizationThread";
        continue;
      }
      if (is_stopped_) {
        LOG(INFO) << "System stopped in initLocalizationThread";
        continue;
      }
      if (d_cloud_measures_.empty()) {
        LOG(WARNING) << "Cloud container is empty!";
        continue;
      }

      *input_cloud = *d_cloud_measures_.front().cloud;
      d_cloud_measures_.clear();
    }

    if (init_mode_ == INIT_MODEL::GPS && is_gps_param_ok_) {
      Vec3d gps_pose = d_gps_measures_.rbegin()->nav_pos;
      init_pose_.block<3, 1>(0, 3) =
          Mathbox::multiplePoint(gps_trans_, gps_pose);
      init_pose_.block<3, 3>(0, 0) = Mat3d::Identity();
      FramePosition pose;
      pose.x = init_pose_(0, 3);
      pose.y = init_pose_(1, 3);
      pose.z = init_pose_(2, 3);
      LOG(INFO) << "find candidates by: " << pose.x << " " << pose.y << " "
                << pose.z;
      std::vector<std::shared_ptr<KeyFrame>> l_kf =
          ptr_map_manager_->DetectCandidatesByHorizontalDistance(pose, 8.0);
      LOG(INFO) << "relocalization find: " << l_kf.size();
      double min_distance = 8.0;
      FramePosition sim_pose;
      for (auto it = l_kf.begin(); it != l_kf.end(); it++) {
        FramePosition pose_it = (*it)->getPosition();
        double distance =
            std::sqrt((pose_it.x - pose.x) * (pose_it.x - pose.x) +
                      (pose_it.y - pose.y) * (pose_it.y - pose.y));
        LOG(INFO) << "find pose: " << pose_it.x << " " << pose_it.y << " "
                  << pose_it.z;
        LOG(INFO) << "distance: " << distance;
        if (distance < min_distance) {
          min_distance = distance;
          sim_pose = pose_it;
        }
      }
      if (min_distance < 8.0) {
        init_pose_(2, 3) = sim_pose.z;
      } else {
        LOG(INFO) << "can't not find sim pose in map:\n "
                  << init_pose_.block<3, 1>(0, 3);
      }
      Vec3d init_predict_pose = init_pose_.block<3, 1>(0, 3);
      Vec3d angle =
          Mathbox::rotationMatrixToEulerAngles(init_pose_.block<3, 3>(0, 0));
      double init_predict_yaw = angle(2);
      Mat34d reliable_pose;
      if (ptr_loop_track_->reLocalization(init_predict_pose, input_cloud,
                                          init_predict_yaw, reliable_pose)) {
        if (d_odom_measures_.empty()) {
          LOG(ERROR) << "odom container is empty!";
          continue;
        }
        Mat34d cur_odom_pose = (d_odom_measures_.rbegin()->pose);
        cur_map_to_odom_ = Mathbox::multiplePose34d(
            reliable_pose, Mathbox::inversePose34d(cur_odom_pose));
        is_localization_init_ = true;
      } else {
        LOG(ERROR) << "gps relocalization failed";
      }
    } else if (init_mode_ == INIT_MODEL::FIX_POSE) {
      FramePosition pose;
      pose.x = init_pose_(0, 3);
      pose.y = init_pose_(1, 3);
      pose.z = init_pose_(2, 3);
      LOG(INFO) << "find candidates by: " << pose.x << " " << pose.y << " "
                << pose.z;
      bool using_feature_iris = false;
      Vec3d init_predict_pose;
      double init_predict_yaw = 0.0;
      LOG(INFO) << "is_feature_data_ready_ " << is_feature_data_ready_ << ","
                << ptr_config_->backend_config.using_feature_iris_init << ","
                << try_lidar_feature_count_;
      if (is_feature_data_ready_ &&
          ptr_config_->backend_config.using_feature_iris_init &&
          try_lidar_feature_count_ < 3) {
        if (initLocalizationByLidarFeature(input_cloud, pose, init_predict_pose,
                                           init_predict_yaw)) {
          using_feature_iris = true;
        }
      }
      if (!using_feature_iris) {
        std::vector<std::shared_ptr<KeyFrame>> l_kf =
            ptr_map_manager_->DetectCandidatesByHorizontalDistance(pose, 8.0);
        LOG(INFO) << "relocalization find: " << l_kf.size();
        double min_distance = 8.0;
        FramePosition sim_pose;
        for (auto it = l_kf.begin(); it != l_kf.end(); it++) {
          FramePosition pose_it = (*it)->getPosition();
          double distance =
              std::sqrt((pose_it.x - pose.x) * (pose_it.x - pose.x) +
                        (pose_it.y - pose.y) * (pose_it.y - pose.y));
          LOG(INFO) << "find pose: " << pose_it.x << " " << pose_it.y << " "
                    << pose_it.z;
          LOG(INFO) << "distance: " << distance;
          if (distance < min_distance) {
            min_distance = distance;
            sim_pose = pose_it;
          }
        }
        if (min_distance < 8.0) {
          init_pose_(2, 3) = sim_pose.z;
        } else {
          LOG(ERROR) << "can't not find sim pose in map:\n "
                     << init_pose_.block<3, 1>(0, 3);
        }
        init_predict_pose = init_pose_.block<3, 1>(0, 3);
        Vec3d angle =
            Mathbox::rotationMatrixToEulerAngles(init_pose_.block<3, 3>(0, 0));
        init_predict_yaw = angle(2);
      }

      Mat34d reliable_pose;
      if (ptr_loop_track_->reLocalization(init_predict_pose, input_cloud,
                                          init_predict_yaw, reliable_pose)) {
        if (d_odom_measures_.empty()) {
          LOG(ERROR) << "odom container is empty!";
          continue;
        }
        Mat34d cur_odom_pose = (d_odom_measures_.rbegin()->pose);
        cur_map_to_odom_ = Mathbox::multiplePose34d(
            reliable_pose, Mathbox::inversePose34d(cur_odom_pose));
        is_localization_init_ = true;
      } else {
        LOG(ERROR) << "fix pose relocalization failed";
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void System::mapUpdateThread() {
  while (!is_stopped_) {
    AD_MODE robot_mode = ptr_state_machine_->getCurrentMode();
    if (AD_MODE::AD_UNKNOW == robot_mode || is_stop_requested_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      continue;
    }
    if (AD_MODE::AD_MAPPING == robot_mode && ("" != temp_cloud_filepath_) &&
        has_new_keyframe_) {
      has_new_keyframe_ = false;
      laserCloud::Ptr key_frame_pose = ptr_map_manager_->getKeyPoses();
      if (nullptr == ptr_cur_keyframe_) {
        continue;
      }
      FramePosition cur_pos = ptr_cur_keyframe_->getPosition();
      for (unsigned int i = 0; i < key_frame_pose->size(); ++i) {
        FramePosition point = key_frame_pose->points[i];
        double distance =
            std::sqrt((point.x - cur_pos.x) * (point.x - cur_pos.x) +
                      (point.y - cur_pos.y) * (point.y - cur_pos.y) +
                      (point.z - cur_pos.z) * (point.z - cur_pos.z));
        size_t frame_id = (size_t) key_frame_pose->points[i].intensity;
        std::shared_ptr<KeyFrame> ptr_candidate_frame =
            ptr_map_manager_->getKeyFrameFromID(frame_id);
        if (std::fabs(distance) < 50) {
          //临近帧非激活状态就需要再激活
          if (!ptr_candidate_frame->isActived()) {
            ptr_map_manager_->activeKeyFrame(temp_cloud_filepath_,
                                             ptr_candidate_frame);
          }

        } else {
          //非临近帧激活状态就需要再禁止激活
          if (ptr_candidate_frame->isActived()) {
            ptr_map_manager_->unactiveKeyFrame(temp_cloud_filepath_,
                                               ptr_candidate_frame);
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}

bool System::creatFrame() {
  AD_STATU robot_state = ptr_state_machine_->getCurrentState();
  if (AD_STATU::UNKNOW == robot_state) {
    return false;
  }
  ptr_cur_keyframe_ = nullptr;
  CloudMeasure cur_cloud_measure;
  laserCloud::Ptr input_cloud(new laserCloud());
  Mat34d correspond_odom_pose;
  Mat34d end_2_begin;
  Vec3d correspond_gps_pos;
  double gps_cov = 1000.;
  bool has_gps = false;
  double time_stamp = -1.;
  bool find = false;
  // insert cloud data into frame
  {
    std::unique_lock<std::mutex> lock(cloud_data_mutex_);
    // map_tracker_cv_.wait(lock, [this] {
    //   return !d_cloud_measures_.empty() || is_stopped_ || is_stop_requested_;
    // });
    if (d_cloud_measures_.empty()) {
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
    cur_cloud_measure = d_cloud_measures_.front();
    d_cloud_measures_.pop_front();
    if (4 == d_cloud_measures_.size()) {
      LOG(WARNING) << "pointcloud buffer size: " << d_cloud_measures_.size()
                   << std::endl;
    }

    time_stamp = cur_cloud_measure.time_stamp;
    *input_cloud = *cur_cloud_measure.cloud;
  }
  // insert odom data into frame
  TicToc create_frame_cost;
  {
    std::lock_guard<std::mutex> lock1(odom_data_mutex_);
    if (d_odom_measures_.empty()) {
      LOG(WARNING) << "Odom container is empty!";
      return false;
    }
    // std::cout << d_odom_measures_.begin()->time_stamp << std::endl;
    // std::cout << d_odom_measures_.rbegin()->time_stamp << std::endl;
    // odom的容器都是按顺序排列的，如果不是还需要用std::sort()排序
    auto last_iter = d_odom_measures_.rbegin();
    auto second_iter = d_odom_measures_.rbegin();
    second_iter++;
    if (time_stamp > last_iter->time_stamp) {
      if ((time_stamp - last_iter->time_stamp) < 0.1) {
        correspond_odom_pose = last_iter->pose;
      } else {
        LOG(WARNING) << "Too much odom data delay  !!!";
        lost_frame_count_++;
        return false;
      }
    } else {
      if (second_iter == d_odom_measures_.rend()) {
        LOG(ERROR) << "Odom data is not reliable.";
        return false;
      }
      for (; second_iter != d_odom_measures_.rend();
           second_iter++, last_iter++) {
        if (time_stamp >= second_iter->time_stamp &&
            time_stamp <= last_iter->time_stamp) {
          double alpha = (time_stamp - second_iter->time_stamp) /
                         (last_iter->time_stamp - second_iter->time_stamp);
          correspond_odom_pose =
              Mathbox::Interp_SE3(second_iter->pose, last_iter->pose, alpha);
          while ((last_iter->time_stamp - second_iter->time_stamp) < 0.1) {
            if ((++second_iter) == d_odom_measures_.rend()) {
              LOG(ERROR) << "Odom data is not reliable.";
              break;
            }
          }
          if (second_iter == d_odom_measures_.rend()) {
            LOG(ERROR) << "second_iter equal to end iter.";
            return false;
          }
          const double &dt = last_iter->time_stamp - second_iter->time_stamp;
          Mat34d delta_odom_pose =
              Mathbox::deltaPose34d(last_iter->pose, second_iter->pose);
          end_2_begin = Mathbox::Interp_SE3(Mathbox::Identity34(),
                                            delta_odom_pose, 0.1 / dt);
          find = true;
          break;
        }
      }
      if (!find) {
        LOG(ERROR) << "Can not find correspond odom data.";
        return false;
      }
    }
  }
  // insert gps data into frame
  {
    std::lock_guard<std::mutex> lock2(gps_data_mutex_);
    if (!d_gps_measures_.empty()) {
      auto gps_last_iter = d_gps_measures_.begin();
      auto gps_second_iter = d_gps_measures_.begin();
      gps_second_iter++;
      if (time_stamp > d_gps_measures_.rbegin()->time_stamp) {
        if (time_stamp - d_gps_measures_.rbegin()->time_stamp < 0.1) {
          correspond_gps_pos = d_gps_measures_.rbegin()->nav_pos;
          gps_cov = d_gps_measures_.rbegin()->cov;
          has_gps = true;
        } else {
          LOG(WARNING) << "Too much gps data delay!";
        }
      } else {
        for (; gps_second_iter != d_gps_measures_.end();
             gps_second_iter++, gps_last_iter++) {
          if (time_stamp <= gps_second_iter->time_stamp &&
              time_stamp >= gps_last_iter->time_stamp) {
            double t2 =
                -gps_last_iter->time_stamp + gps_second_iter->time_stamp;
            if (t2 < 0.3 && t2 > 0.) {
              double alpha = (time_stamp - gps_last_iter->time_stamp) / (t2);
              correspond_gps_pos =
                  gps_second_iter->nav_pos +
                  alpha * (-gps_last_iter->nav_pos + gps_second_iter->nav_pos);
              gps_cov = gps_second_iter->cov +
                        alpha * (gps_last_iter->cov - gps_second_iter->cov);
              has_gps = true;
            } else {
              LOG(ERROR) << "gps delta time too large" << std::endl;
            }
            break;
          } else if (time_stamp > gps_second_iter->time_stamp &&
                     time_stamp - gps_second_iter->time_stamp < 0.2) {
            double t2 =
                -gps_last_iter->time_stamp + gps_second_iter->time_stamp;
            if (t2 < 0.3 && t2 > 0.) {
              double alpha = (time_stamp - gps_second_iter->time_stamp) / t2;
              correspond_gps_pos =
                  gps_second_iter->nav_pos +
                  alpha * (-gps_last_iter->nav_pos + gps_second_iter->nav_pos);
              gps_cov = gps_second_iter->cov +
                        alpha * (gps_last_iter->cov - gps_second_iter->cov);
              has_gps = true;
            } else {
              LOG(ERROR) << "add gps error" << std::endl;
            }
            break;
          }
        }
      }
    }
  }
  if (is_first_keyframe_ ||
      !ptr_map_track_->isJunkFrame(correspond_odom_pose)) {
    laserCloud::Ptr corner_cloud(new laserCloud());
    laserCloud::Ptr surf_cloud(new laserCloud());
    laserCloud::Ptr without_ground_cloud(new laserCloud());
    ptr_feature_extractor_->resetVariables();
    if (!ptr_feature_extractor_->setInputCloud(input_cloud)) {
      return false;
    }
    if (find) {
      ptr_feature_extractor_->adjustDistortion(end_2_begin);
    }

    if (!ptr_feature_extractor_->cloudExtractor(corner_cloud, surf_cloud,
                                                cloud_for_occ_map_)) {
      return false;
    }
    *cloud_for_iris_ = *input_cloud;
    Mat34d predict_cur_frame_pose =
        Mathbox::multiplePose34d(cur_map_to_odom_, correspond_odom_pose);

    if (nullptr != ptr_cur_keyframe_) {
      predict_cur_frame_pose.block<3, 1>(0, 3) =
          ptr_cur_keyframe_->getPose().block<3, 1>(0, 3);
    }

    if (has_gps) {
      ptr_cur_keyframe_ = std::make_shared<KeyFrame>(
          id_++, time_stamp, correspond_odom_pose, predict_cur_frame_pose,
          corner_cloud, surf_cloud, without_ground_cloud, correspond_gps_pos,
          gps_cov);
    } else {
      ptr_cur_keyframe_ = std::make_shared<KeyFrame>(
          id_++, time_stamp, correspond_odom_pose, predict_cur_frame_pose,
          corner_cloud, surf_cloud, without_ground_cloud);
    }
    LOG(INFO) << "Create frame cost: " << create_frame_cost.toc();
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

void System::addOdom(const OdomMeasure &odom) {
  std::lock_guard<std::mutex> lock(odom_data_mutex_);
  if (200 <= d_odom_measures_.size()) {
    d_odom_measures_.pop_front();
  }
  d_odom_measures_.emplace_back(odom);
  cur_robot_state_.pose = Mathbox::multiplePose34d(cur_map_to_odom_, odom.pose);
  cur_robot_state_.time_stamp = odom.time_stamp;
}

void System::addGPS(const GpsMsg &gps_msg) {
  if (is_gps_param_ok_ ||
      AD_MODE::AD_MAPPING == ptr_state_machine_->getCurrentMode()) {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    double xyz[3];
    GPS2XYZ(gps_msg.latitude, gps_msg.longitude, gps_msg.altitude, xyz);
    GPSMeasure gps(Vec3d(xyz[0], xyz[1], xyz[2]), gps_msg.time_stamp,
                   gps_msg.cov, gps_msg.loc_type);
    if (40 <= d_gps_measures_.size()) {
      d_gps_measures_.pop_front();
    }
    d_gps_measures_.emplace_back(gps);
  }
}

void System::GPS2XYZ(double latitude, double longitude, double altitude,
                     double *xyz) {
  if (!init_gps_) {
    gps_origin_(0) = latitude;
    gps_origin_(1) = longitude;
    gps_origin_(2) = altitude;
    geo_converter_.Reset(latitude, longitude, altitude);
    init_gps_ = true;
  }
  geo_converter_.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
}

bool System::saveGpsParams(const std::string &gps_dir) {
  // std::string gps_origin_file_path = gps_dir + "origin.yaml";
  std::string gps_file_path = gps_dir + "gps.yaml";
  // cv::FileStorage origin_file(gps_origin_file_path, cv::FileStorage::WRITE);
  cv::FileStorage gps_file(gps_file_path, cv::FileStorage::WRITE);
  Mat34d gps_trans = ptr_msf_->getGpsTrans();
  if (!gps_file.isOpened()) {
    LOG(ERROR) << "file path:" << gps_file_path << "is not exist";
    return false;
  } else {
    gps_file << "latitude" << gps_origin_(0);
    gps_file << "longtitude" << gps_origin_(1);
    gps_file << "altitude" << gps_origin_(2);
    LOG(INFO) << "latitude:" << gps_origin_(0) << " "
              << "longtitude:" << gps_origin_(1)
              << "altitude:" << gps_origin_(2);
    // origin_file.release();
    cv::Mat cv_global_trans(3, 4, CV_64FC1);
    for (int i = 0; i < gps_trans.rows(); i++) {
      for (int j = 0; j < gps_trans.cols(); j++) {
        cv_global_trans.at<double>(i, j) = gps_trans(i, j);
      }
    }
    gps_file << "gps_trans" << cv_global_trans;
    LOG(INFO) << "gps_trans: " << std::endl << cv_global_trans;
    gps_file.release();
    return true;
  }
}

bool System::loadGpsParams(const std::string &gps_dir) {
  cv::Mat cv_global_trans(3, 4, CV_64FC1);
  Vec3d gps_origin;
  std::string gps_file_path = gps_dir + "gps.yaml";
  cv::FileStorage file(gps_file_path, cv::FileStorage::READ);
  if (!file.isOpened()) {
    LOG(ERROR) << "file path:" << gps_file_path << "is not exist";
    return false;
  } else {
    file["latitude"] >> gps_origin(0);
    file["longtitude"] >> gps_origin(1);
    file["altitude"] >> gps_origin(2);
    LOG(INFO) << "latitude:" << gps_origin(0) << " "
              << "longtitude:" << gps_origin(1)
              << " altitude:" << gps_origin(2);
    file["gps_trans"] >> cv_global_trans;
    file.release();
    for (int i = 0; i < gps_trans_.rows(); i++) {
      for (int j = 0; j < gps_trans_.cols(); j++) {
        gps_trans_(i, j) = cv_global_trans.at<double>(i, j);
      }
    }
    ptr_msf_->setGpsTrans(gps_trans_);
    double xyz[3];
    GPS2XYZ(gps_origin(0), gps_origin(1), gps_origin(2), xyz);
    is_gps_param_ok_ = true;
    return true;
  }
}

void System::addCloudFrame(const laserCloud::Ptr cloud_in,
                           const double time_stamp) {
  std::lock_guard<std::mutex> lock(cloud_data_mutex_);
  if (5 <= d_cloud_measures_.size()) {
    d_cloud_measures_.pop_front();
  }
  d_cloud_measures_.emplace_back(cloud_in, time_stamp);
  map_tracker_cv_.notify_one();
}

void System::requestStop() {
  is_stop_requested_ = true;
  ptr_loop_track_->requestStop();
  usleep(100000);  // sleep for safety stop
  map_tracker_cv_.notify_one();
}

void System::Reset() {
  is_reset_ = true;
  last_map_to_odom_ = Mathbox::Identity34();
  cur_map_to_odom_ = Mathbox::Identity34();
  cur_robot_state_.Reset();
  is_first_keyframe_ = true;
  is_localization_init_ = false;
  is_opt_finished_ = false;
  try_lidar_feature_count_ = 0;
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
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    d_gps_measures_.clear();
    std::deque<GPSMeasure>().swap(d_gps_measures_);
  }
  {
    std::lock_guard<std::mutex> lock(cloud_data_mutex_);
    d_cloud_measures_.clear();
    std::deque<CloudMeasure>().swap(d_cloud_measures_);
  }
  sc_frame_ids_.clear();
  std::vector<long int>().swap(sc_frame_ids_);
  ptr_sc_manager_.reset();
  ptr_sc_manager_ = std::make_shared<SCManager>();

  ptr_map_track_->Reset();
  ptr_map_manager_->Reset();
  ptr_loop_track_->Reset();
  ptr_msf_->Reset();
  ptr_occ_map_->Reset();
  is_stop_requested_ = false;
  malloc_trim(0);
}

void System::shutdown() {
  d_odom_measures_.clear();
  d_gps_measures_.clear();
  d_cloud_measures_.clear();
  sc_frame_ids_.clear();

  ptr_sc_manager_.reset();
  ptr_map_track_->requestStop();
  ptr_map_track_->Stop();
  ptr_loop_track_->requestStop();
  ptr_loop_track_->stop();
  run_opt_ = false;
  is_stopped_ = true;
  has_new_keyframe_ = false;
  map_tracker_cv_.notify_one();
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
    const std::string &gps_path = map_dir + "/gps/";
    const std::string &map_3d_path = map_dir + "/3d_map/";
    const std::string &map_2d_path = map_dir + "/2d_map/";
    const std::string &lidar_feature_path = map_dir + "/lidar_feature/";
    ptr_map_manager_->saveAllKeyFrame(map_3d_path, temp_cloud_filepath_);
    occ_save_mutex_.lock();
    ptr_occ_map_->saveMap(map_2d_path);
    occ_save_mutex_.unlock();
    ptr_sc_manager_->saveScancontext(lidar_feature_path, sc_frame_ids_);
    saveGpsParams(gps_path);
    ptr_msf_->saveGraph(map_3d_path);
    // ptr_map_manager_->loadKeyFrameInfo(map_dir);
    // ptr_msf_->loadGraph(map_dir);
  } else {
    LOG(WARNING) << " No map save in localization mode !!!" << std::endl;
  }
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
  return ptr_map_track_->buildKdtree();
}

bool System::loadMap(const std::string &map_dir) {
  FramePosition pose;
  pose.x = 0.;
  pose.y = 0.;
  pose.z = 0.;

  is_local_map_ready_ = ptr_map_manager_->loadLocalKeyFrame(map_dir, pose);
  if (!is_local_map_ready_) {
    return false;
  }
  return ptr_map_track_->buildKdtree();
}

bool System::loadFeatureData(const std::string &map_dir) {
  sc_frame_ids_.clear();
  is_feature_data_ready_ = ptr_sc_manager_->loadFeature(map_dir, sc_frame_ids_);
  return is_feature_data_ready_;
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

    // 恢复occ map
    ptr_occ_map_->Reset();
    if (!ptr_occ_map_->loadScan(map_2d_path)) {
      LOG(ERROR) << "Failed to load scan data.";
      return false;
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
    ptr_map_track_->setMap2Odom(cur_map_to_odom_);
    return true;

  } else {
    LOG(ERROR) << "This function only supports [ UPDATE_MAPPING ] status.";
    return false;
  }
}

bool System::getViewCloud(laserCloud::Ptr &view_cloud,
                          laserCloud::Ptr &frame_pose) {
  if (ptr_map_track_->isMapReady()) {
    ptr_map_track_->setMapStatus(false);
    view_cloud = ptr_map_manager_->getGlobalCloud();  // TODO:没有加锁，不安全
    frame_pose = ptr_map_manager_->getKeyPoses();  // TODO:没有加锁，不安全
    return true;
  }
  return false;
}

laserCloud::Ptr System::getWholeMap() {
  return ptr_map_manager_->getSurfMapCloud();
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

long unsigned int System::getLatestFrameID() {
  return ptr_map_manager_->getLatestFrameID();
}

bool System::hasKeyFrame() {
  return ptr_map_manager_->hasKeyFrame();
}

std::vector<std::pair<long unsigned int, Mat34d>>
System::getKeyFrameDataPose() {
  return ptr_map_manager_->getKeyFrameDataPose();
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

}  // namespace cvte_lidar_slam
