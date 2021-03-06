/*
 * @Author: your name
 * @Date: 2020-07-17 17:03:40
 * @LastEditTime: 2020-09-11 14:55:35
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /cvte_lidar_slam/ros_adapter/new_mission_manager.cpp
 */
#include "new_mission_manager.hpp"

#include "log.hpp"
#include "new_lidar_slam_ros2.hpp"

namespace cvte_lidar_slam {
MissionManagerRos2::MissionManagerRos2(rclcpp::Node::SharedPtr node) {
  node_ = node;
  ptr_slam_system_ = System::getInstance();
  ptr_occ_map_ = OccMap::getInstance();
  ptr_state_machine_ = SlamStateMachine::getInstance();

  // node_ = std::make_shared<rclcpp::Node>("lidar_slam_mission_manager");

  node_->get_parameter_or(
      "config_file", config_file_,
      std::string("./install/cvte_lidar_slam/params/velodyne.yaml"));
  std::vector<double> t_lo = {1., 0., 0., -0.25, 0., 1.,
                              0., 0., 0., 0.,    1., 0.};
  node_->get_parameter_or("t_lo", t_lo, t_lo);
  T_lo_ << t_lo[0], t_lo[1], t_lo[2], t_lo[3], t_lo[4], t_lo[5], t_lo[6],
      t_lo[7], t_lo[8], t_lo[9], t_lo[10], t_lo[11];

  mission_manager_server_ =
      node_->create_service<mission_manager_msgs::srv::MissionManager>(
          "lidar_slam_mission_manager",
          std::bind(&MissionManagerRos2::missionManagerCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));

  cmd_events_ = {{"start_mapping", EventName::START_MAPPING},
                 {"save_mapping", EventName::SAVE_MAP},
                 {"cancel_mapping", EventName::CANCLE_MAPPING},
                 {"start_localization", EventName::START_LOCALIZATION},
                 {"stop_localization", EventName::STOP_LOCALIZAION},
                 {"get_status", EventName::GET_STATE},
                 {"add_map_point", EventName::SET_PARTOL_POINT},
                 {"return_mapping", EventName::RETURN_MAPPING}};
}

void MissionManagerRos2::missionManagerCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
        request,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
        response) {
  missionHandler(request->send_data, response->ack_data);
}

void MissionManagerRos2::missionHandler(std::string rev_data,
                                        std::string &ack_data) {
  Json::Reader json_reader;
  json_rev_data_.clear();
  json_ack_data_.clear();
  // LOG(INFO) << "receive from mission manager:\n" << rev_data;

  if (!json_reader.parse(rev_data, json_rev_data_)) {
    json_ack_data_["error_msgs"] = " rev_data is illegal!!";
    LOG(ERROR) << "rev_data is illegal!!";
    json_ack_data_["succeed"] = false;
    ack_data = json_ack_data_.toStyledString();
    return;
  }
  std::string cmd = json_rev_data_["cmd"].asString();

  EventName event;
  auto it_cmd = cmd_events_.find(cmd);
  if (it_cmd == cmd_events_.end()) {
    json_ack_data_["error_msgs"] = " rev_data is illegal!!";
    LOG(ERROR) << "rev_data is illegal!!";
    json_ack_data_["succeed"] = false;
    ack_data = json_ack_data_.toStyledString();
    return;
  }
  event = it_cmd->second;
  if (cmd != std::string("get_status")) {
    LOG(ERROR) << " Recevied CMD: " << cmd;
  }

  ptr_state_machine_->sendEvent(event);
  ack_data = json_ack_data_.toStyledString();
}

bool MissionManagerRos2::addPartolPose() {
  if (AD_STATU::MAPPING != ptr_state_machine_->getCurrentState()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "current state is not mapping.";
    return false;
  }
  if (json_rev_data_["point_id"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "point_id empty.";
    return false;
  }
  std::string point_id = json_rev_data_["point_id"].asString();
  if (!ptr_slam_system_->hasKeyFrame()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "keyframe pose empty.";
    return false;
  }
  PartolPointInfo partol_point;
  Mat34d frame_odom;
  Mat34d T_lilj;  //??????????????????????????????????????????pose
  int id = ptr_slam_system_->getLatestFrameID();
  RobotState robot_state = ptr_slam_system_->getRobotState();
  Mat34d cur_odom_pose = robot_state.pose;
  if (!ptr_slam_system_->getPoseFromID(id, frame_odom)) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "keyframe odom pose empty.";
    return false;
  } else {
    T_lilj = Mathbox::deltaPose34d(frame_odom, cur_odom_pose);
  }
  LOG(INFO) << "add_map_point" << std::endl;
  if (id < 0) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "latest frame id < 0.";
    return false;
  } else {
    partol_point.id = id;
    partol_point.pose_compensation = Mathbox::multiplePose34d(T_lilj, T_lo_);
    partol_pose_id_[point_id] = partol_point;
    json_ack_data_["succeed"] = true;
  }
  return true;
}

bool MissionManagerRos2::stopLocalization() {
  if (nullptr == ptr_slam_system_) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "system ptr null.";
    LOG(ERROR) << "system ptr null.";
    return false;
  }
  ptr_slam_system_->requestStop();
  ptr_slam_system_->Reset();
  json_ack_data_["succeed"] = true;
  return true;
}

bool MissionManagerRos2::startLocalization() {
  if (json_rev_data_["dir_path"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "dir_path empty.";
    return false;
  }

  std::string cmd = json_rev_data_["cmd"].asString();

  // TODO: ????????????????????????????????????
  std::string data_dir_path = json_rev_data_["dir_path"].asString();
  if (access((data_dir_path + "/3d_map/keyframe_pos.pcd").c_str(), 0) != 0) {
    LOG(ERROR) << data_dir_path << "  is not exist";
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "map data empty.";
    return false;
  }

  LOG(INFO) << " data path: " << data_dir_path;
  if (!ptr_slam_system_->loadGpsParams(data_dir_path + "/gps/")) {
    LOG(ERROR) << "loading gps params failed.";
    if ("gps" == json_rev_data_["init_model"].asString()) {
      json_ack_data_["succeed"] = false;
      json_ack_data_["error_msgs"] = "loading gps params failed.";
      return false;
    }
  }
  if (!ptr_slam_system_->loadMap(data_dir_path + "/3d_map/")) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "loading map failed.";
    LOG(ERROR) << "loading map failed.";
    return false;
  }
  if (!ptr_slam_system_->loadFeatureData(data_dir_path + "/lidar_feature/")) {
    LOG(WARNING) << "loading feature data failed.";
  }

  // TODO:??????????????????????????????
  INIT_MODEL init_model;
  if ("fix_pose" == json_rev_data_["init_model"].asString() ||
      "reliable_pose" == json_rev_data_["init_model"].asString()) {
    Vec3d euler_angle;
    Vec3d vec3d_pose;
    euler_angle(0) = json_rev_data_["yaw"].asDouble();
    euler_angle(1) = 0;
    euler_angle(2) = 0;
    vec3d_pose(0) = json_rev_data_["x"].asDouble();
    vec3d_pose(1) = json_rev_data_["y"].asDouble();
    vec3d_pose(2) = json_rev_data_["z"].asDouble();

    Mat34d init_predict_pose = Mathbox::Euler2Mat34d(euler_angle, vec3d_pose);
    Mat34d laser_predict_pose = Mathbox::multiplePose34d(
        init_predict_pose, Mathbox::inversePose34d(T_lo_));
    if ("fix_pose" == json_rev_data_["init_model"].asString()) {
      LOG(INFO) << "set init model: fix_pose" << std::endl;
      init_model = INIT_MODEL::FIX_POSE;
      ptr_slam_system_->setInitModel(init_model);
      ptr_slam_system_->setInitPose(laser_predict_pose);
    } else {
      LOG(INFO) << "set init model: reliable_pose" << std::endl;
      ptr_slam_system_->setReliablePose(laser_predict_pose);
    }
  } else if ("gps" == json_rev_data_["init_model"].asString()) {
    LOG(INFO) << "set init model: gps" << std::endl;
    init_model = INIT_MODEL::GPS;
    ptr_slam_system_->setInitModel(init_model);
  }
  // TODO: ?????????????????????????????????????????????running
  json_ack_data_["succeed"] = true;
  return true;
}

bool MissionManagerRos2::startMapping() {
  if (json_rev_data_["realtime_map_file"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "realtime_map_file dir_path empty.";
    LOG(ERROR) << "realtime_map_file dir_path is empty, default temp map path "
                  "will be used.";
    // return; // TODO:????????????????????????????????????????????????return
  } else {
    // TODO:??????????????????????????????
    std::string default_temp_map_filepath =
        json_rev_data_["realtime_map_file"].asString();
    if (default_temp_map_filepath.back() != '/') {
      default_temp_map_filepath.push_back('/');
    }
    ptr_occ_map_->setMapFilePath(default_temp_map_filepath);
    ptr_slam_system_->setCloudMapPath(default_temp_map_filepath);
  }
  json_ack_data_["succeed"] = true;
  LOG(ERROR) << "...................start mapping......................";
  return true;
}

bool MissionManagerRos2::returnMapping() {
  if (json_rev_data_["dir_path"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "dir_path empty.";
    return false;
  }
  std::string data_dir_path = json_rev_data_["dir_path"].asString();
  if (!NavPoints::loadNavPoints(data_dir_path, partol_pose_id_)) {
    LOG(WARNING) << "Warning, there is no nav points";
  }
  ptr_slam_system_->setStopRequest(true);
  if (ptr_slam_system_->loadWholeData(data_dir_path)) {
    json_ack_data_["succeed"] = true;
    LOG(ERROR) << "...................return mapping......................";
  } else {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "load data failed.";
    return false;
  }
  if (json_rev_data_["realtime_map_file"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "realtime_map_file dir_path empty.";
    LOG(ERROR) << "realtime_map_file dir_path is empty, default temp map path "
                  "will be used.";
    // return; // TODO:????????????????????????????????????????????????return
  } else {
    // TODO:??????????????????????????????
    std::string default_temp_map_filepath =
        json_rev_data_["realtime_map_file"].asString();
    if (default_temp_map_filepath.back() != '/') {
      default_temp_map_filepath.push_back('/');
    }
    ptr_occ_map_->setMapFilePath(default_temp_map_filepath);
    ptr_slam_system_->setCloudMapPath(default_temp_map_filepath);
  }
  ptr_slam_system_->setStopRequest(false);
  return true;
}

bool MissionManagerRos2::saveMapping() {
  if (nullptr == ptr_slam_system_) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "system ptr null.";
    LOG(ERROR) << "system ptr null.";
    return false;
  }

  if (json_rev_data_["dir_path"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "dir_path empty.";
    return false;
  }
  std::string save_path = json_rev_data_["dir_path"].asString();

  std::string save_path_file = json_rev_data_["path_file"].asString();

  if (access(save_path.c_str(), 0) == -1) {
    int flag = mkdir(save_path.c_str(), 0777);
    int gps_flag = mkdir((save_path + "/gps/").c_str(), 0777);
    int map_flag = mkdir((save_path + "/3d_map/").c_str(), 0777);
    int map_2d_flag = mkdir((save_path + "/2d_map/").c_str(), 0777);
    int nav_point_flag = mkdir((save_path + "/nav_point/").c_str(), 0777);
    int map_feature_flag = mkdir((save_path + "/lidar_feature/").c_str(), 0777);
    if (flag == 0 && gps_flag == 0 && map_flag == 0 && map_2d_flag == 0 &&
        map_feature_flag == 0 && nav_point_flag == 0) {
      LOG(INFO) << " make dir successfully: " << save_path;
    } else {
      LOG(ERROR) << "make dir failed: " << save_path;
      json_ack_data_["succeed"] = false;
      json_ack_data_["error_msgs"] = "make dir failed.";
      return false;
    }
  } else {
    LOG(ERROR) << "dir_path exist: " << save_path;
    std::string gps_path = save_path + "/gps/";
    std::string map_path = save_path + "/3d_map/";
    std::string map_2d_path = save_path + "/2d_map/";
    std::string nav_point_path = save_path + "/nav_point/";
    std::string map_feature_path = save_path + "/lidar_feature/";
    int gps_flag = -1;
    int map_flag = -1;
    int map_2d_flag = -1;
    int nav_point_flag = -1;
    int map_feature_flag = -1;
    if (access(gps_path.c_str(), 0) == -1) {
      gps_flag = mkdir((gps_path).c_str(), 0777);
    }
    if (access(map_path.c_str(), 0) == -1) {
      map_flag = mkdir((map_path).c_str(), 0777);
    }
    if (access(map_2d_path.c_str(), 0) == -1) {
      map_2d_flag = mkdir((map_2d_path).c_str(), 0777);
    }
    if (access(nav_point_path.c_str(), 0) == -1) {
      nav_point_flag = mkdir((nav_point_path).c_str(), 0777);
    }

    if (access(map_feature_path.c_str(), 0) == -1) {
      map_feature_flag = mkdir((map_feature_path).c_str(), 0777);
    }
    if (gps_flag == 0 && map_flag == 0 && map_2d_flag == 0 &&
        map_feature_flag == 0 && nav_point_flag == 0) {
      LOG(INFO) << " make dir successfully: " << save_path;
    } else {
      LOG(ERROR) << "make dir failed: " << save_path;
    }
  }
  // TODO: ????????????????????????????????????
  Json::Value point_json;
  bool get_navi_point = getPartolPoseJson(point_json);
  bool save_nav_points = NavPoints::saveNavPoints(partol_pose_id_, save_path);
  RobotState robot_state = ptr_slam_system_->getRobotState();
  partol_pose_id_.clear();
  ptr_slam_system_->requestStop();
  ptr_slam_system_->saveMap(save_path);
  ptr_slam_system_->Reset();

  // RobotState robot_state = ptr_slam_system_->getRobotState();
  Mat34d cur_pose = robot_state.pose;

  Vec3d cur_rpy =
      Mathbox::rotationMatrixToEulerAngles(cur_pose.block<3, 3>(0, 0));

  double x = cur_pose(0, 3);
  double y = cur_pose(1, 3);
  double z = cur_pose(2, 3);
  double yaw = cur_rpy(2);
  json_ack_data_["save_node_info"] = true;
  json_ack_data_["gps_trans"] = false;
  json_ack_data_["path_file"] = save_path_file;
  json_ack_data_["last_pose"]["x"] = x;
  json_ack_data_["last_pose"]["y"] = y;
  json_ack_data_["last_pose"]["z"] = z;
  json_ack_data_["last_pose"]["yaw"] = yaw;
  if (get_navi_point) {
    json_ack_data_["navi_point"] = point_json["navi_point"];
  }
  json_ack_data_["succeed"] = true;
  return true;
}

bool MissionManagerRos2::cancelMapping() {
  if (nullptr == ptr_slam_system_) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "system ptr null.";
    LOG(ERROR) << "system ptr null.";
    return false;
  }
  ptr_slam_system_->requestStop();
  ptr_slam_system_->Reset();
  partol_pose_id_.clear();
  json_ack_data_["succeed"] = true;
  return true;
}

bool MissionManagerRos2::getStatus() {
  if (ptr_slam_system_->isLocalizationInit() &&
      ptr_state_machine_->getCurrentState() == AD_STATU::INIT_LOCALIZATION) {
    ptr_state_machine_->sendEvent(EventName::INIT_LOCALIZATION_SUCCESS);
  }
  std::string status = ptr_state_machine_->getCurrentStateName();
  std::string model = ptr_state_machine_->getCurrentModeName();
  RobotState robot_state = ptr_slam_system_->getRobotState();
  Mat34d cur_pose = robot_state.pose;
  Mat34d cur_base_pose = Mathbox::multiplePose34d(robot_state.pose, T_lo_);

  Vec3d cur_rpy =
      Mathbox::rotationMatrixToEulerAngles(cur_pose.block<3, 3>(0, 0));
  Vec3d extrinsic_rpy =
      Mathbox::rotationMatrixToEulerAngles(T_lo_.block<3, 3>(0, 0));
  Vec3d cur_base_rpy =
      Mathbox::rotationMatrixToEulerAngles(cur_base_pose.block<3, 3>(0, 0));

  double x = cur_pose(0, 3);
  double y = cur_pose(1, 3);
  double z = cur_pose(2, 3);
  double yaw = cur_rpy(2);
  double bx = cur_base_pose(0, 3);
  double by = cur_base_pose(1, 3);
  double bz = cur_base_pose(2, 3);
  double byaw = cur_rpy(2);
  json_ack_data_["succeed"] = true;
  json_ack_data_["status"] = status;
  json_ack_data_["model"] = model;
  json_ack_data_["x"] = x;
  json_ack_data_["y"] = y;
  json_ack_data_["z"] = z;
  json_ack_data_["yaw"] = yaw;
  json_ack_data_["dx"] = T_lo_(0, 3);
  json_ack_data_["dy"] = T_lo_(1, 3);
  json_ack_data_["dyaw"] = extrinsic_rpy(2);
  json_ack_data_["bx"] = bx;
  json_ack_data_["by"] = by;
  json_ack_data_["bz"] = bz;
  json_ack_data_["byaw"] = byaw;
  if (ptr_state_machine_->getCurrentMode() == AD_MODE::AD_MAPPING &&
      !partol_pose_id_.empty()) {
    Json::Value point_json;
    if (getPartolPoseJson(point_json)) {
      json_ack_data_["navi_point"] = point_json["navi_point"];
    }
  }
  return true;
}

bool MissionManagerRos2::getPartolPoseJson(Json::Value &point_json) {
  std::unordered_map<std::string, PartolPointInfo>::iterator it;
  it = partol_pose_id_.begin();
  while (it != partol_pose_id_.end()) {
    long unsigned int frame_id = it->second.id;
    Mat34d temp_pose;
    if (ptr_slam_system_->getPoseFromID(frame_id, temp_pose)) {
      Mat34d robot_pose =
          Mathbox::multiplePose34d(temp_pose, it->second.pose_compensation);
      Vec3d rpy = Mathbox::rotation2rpy(robot_pose.block<3, 3>(0, 0));

      double yaw = rpy(2);

      Json::Value pose;
      pose["name"] = it->first;
      pose["x"] = robot_pose(0, 3);
      pose["y"] = robot_pose(1, 3);
      pose["z"] = robot_pose(2, 3);
      pose["yaw"] = yaw;
      point_json["navi_point"].append(pose);
    } else {
      LOG(ERROR) << "can not find the frame id pose.";
      return false;
    }
    it++;
  }
  return true;
}

}  // namespace cvte_lidar_slam

int main(int argc, char **argv) {
  using namespace cvte_lidar_slam;
  rclcpp::init(argc, argv);
  CVTE_BABOT::initGoogleLog("cvte_lidar_slam", "warn");
  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("lidar_slam_mission_manager");
  MissionManagerRos2 mission_node(node);
  LidarSLAM slam_node(node);
  std::shared_ptr<SlamStateMachine> ptr_state_machine =
      SlamStateMachine::getInstance();
  ptr_state_machine->registerTransActionCallback(
      EventName::START_MAPPING,
      std::bind(&MissionManagerRos2::startMapping, &mission_node));
  ptr_state_machine->registerTransActionCallback(
      EventName::SAVE_MAP,
      std::bind(&MissionManagerRos2::saveMapping, &mission_node));
  ptr_state_machine->registerTransActionCallback(
      EventName::CANCLE_MAPPING,
      std::bind(&MissionManagerRos2::cancelMapping, &mission_node));
  ptr_state_machine->registerTransActionCallback(
      EventName::START_LOCALIZATION,
      std::bind(&MissionManagerRos2::startLocalization, &mission_node));
  ptr_state_machine->registerTransActionCallback(
      EventName::STOP_LOCALIZAION,
      std::bind(&MissionManagerRos2::stopLocalization, &mission_node));
  ptr_state_machine->registerTransActionCallback(
      EventName::GET_STATE,
      std::bind(&MissionManagerRos2::getStatus, &mission_node));
  ptr_state_machine->registerTransActionCallback(
      EventName::SET_PARTOL_POINT,
      std::bind(&MissionManagerRos2::addPartolPose, &mission_node));
  ptr_state_machine->registerTransActionCallback(
      EventName::RETURN_MAPPING,
      std::bind(&MissionManagerRos2::returnMapping, &mission_node));
  ptr_state_machine->registerEntryActionCallback(
      AD_STATU::UNKNOW, std::bind(&LidarSLAM::resetDataCallback, &slam_node));
  ptr_state_machine->registerEntryActionCallback(
      AD_STATU::INIT_LOCALIZATION,
      std::bind(&LidarSLAM::registerDataCallback, &slam_node));
  ptr_state_machine->registerEntryActionCallback(
      AD_STATU::MAPPING,
      std::bind(&LidarSLAM::registerDataCallback, &slam_node));

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}