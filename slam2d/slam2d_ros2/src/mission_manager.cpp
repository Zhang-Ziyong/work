#include "mission_manager.hpp"

#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <fstream>

#include "conversion.hpp"
#include "glog/logging.h"
#include "log.hpp"

namespace slam2d_ros2 {

MissionManagerRos2::MissionManagerRos2(rclcpp::Node::SharedPtr node) {
  node_ = node;
  ptr_slam_system_ = slam2d_core::slam_system::SlamSystem::getInstance();

  ptr_depth_occ_map_ =
      slam2d_core::occupancy_map::DepthOccupancyMap::getInstance();
  ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ptr_state_machine_ =
      slam2d_core::state_machine::SlamStateMachine::getInstance();

  mission_manager_server_ =
      node_->create_service<mission_manager_msgs::srv::MissionManager>(
          "lidar_slam_mission_manager",
          std::bind(&MissionManagerRos2::missionManagerCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));

  cmd_events_ = {
      {"start_mapping", slam2d_core::state_machine::EventName::START_MAPPING},
      {"save_mapping", slam2d_core::state_machine::EventName::SAVE_MAP},
      {"cancel_mapping", slam2d_core::state_machine::EventName::CANCLE_MAPPING},
      {"start_localization",
       slam2d_core::state_machine::EventName::START_LOCALIZATION},
      {"stop_localization",
       slam2d_core::state_machine::EventName::STOP_LOCALIZAION},
      {"get_status", slam2d_core::state_machine::EventName::GET_STATE},
      {"add_map_point",
       slam2d_core::state_machine::EventName::SET_PARTOL_POINT},
      {"return_mapping",
       slam2d_core::state_machine::EventName::RETURN_MAPPING}};
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
  if (cmd != std::string("get_status")) {
    LOG(INFO) << "receive from mission manager:\n" << rev_data;
  }

  slam2d_core::state_machine::EventName event;
  auto it_cmd = cmd_events_.find(cmd);
  if (it_cmd == cmd_events_.end()) {
    json_ack_data_["error_msgs"] = " rev_data is illegal!!";
    LOG(ERROR) << "rev_data is illegal!!";
    json_ack_data_["succeed"] = false;
    ack_data = json_ack_data_.toStyledString();
    return;
  }
  event = it_cmd->second;
  ptr_state_machine_->sendEvent(event);
  ack_data = json_ack_data_.toStyledString();

  if (cmd != std::string("get_status")) {
    LOG(INFO) << "ack data:\n" << ack_data;
  }
}

bool MissionManagerRos2::addPartolPose() {
  if (slam2d_core::state_machine::AD_STATU::MAPPING !=
      ptr_state_machine_->getCurrentState()) {
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

  PartolPointInfo partol_point;
  slam2d_core::common::Rigid3 current_pose = ptr_slam_system_->getCurrentPose();
  int last_node_id;
  slam2d_core::common::Rigid3 last_node_pose;
  if (ptr_slam_system_->getLastTrajectoryNode(last_node_id, last_node_pose)) {
    if (last_node_id < 0) {
      json_ack_data_["succeed"] = false;
      json_ack_data_["error_msgs"] = "point_id is < 0.";
      return false;
    }
    auto pose_compensation = last_node_pose.inverse() * current_pose;
    partol_point.id = last_node_id;
    partol_point.pose_compensation = pose_compensation;
    partol_pose_id_[point_id] = partol_point;
    json_ack_data_["succeed"] = true;
    LOG(INFO) << "add_map_point";
  } else {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "trajectory node size is empty.";
    return false;
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

  ptr_slam_system_->stopLocalization();

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
  std::string data_dir_path = json_rev_data_["dir_path"].asString();

  bool map_2d_loc_flag = true;
  bool map_2d_flag = true;
  if (access((data_dir_path + "/2d_map/map_2d_loc.yaml").c_str(), 0) != 0) {
    LOG(WARNING) << data_dir_path << "map_2d_loc is not exist";
    map_2d_loc_flag = false;
  }

  if (access((data_dir_path + "/2d_map/map_2d.yaml").c_str(), 0) != 0) {
    LOG(WARNING) << data_dir_path << "map_2d is not exist";
    map_2d_flag = false;
  }

  LOG(INFO) << " data path: " << data_dir_path;
  std::string map_file_path = data_dir_path + "/2d_map/map_2d.yaml";

  if (map_2d_loc_flag && map_2d_flag) {
    map_file_path = data_dir_path + "/2d_map/map_2d_loc.yaml";
  } else if (map_2d_flag && !map_2d_loc_flag) {
    LOG(WARNING) << data_dir_path << " map version is old";
  } else {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "map data empty.";
    LOG(ERROR) << data_dir_path << "map is not exist";
    return false;
  }

  slam2d_core::slam_system::InitModle init_modle;

  auto init_pose = slam2d_core::common::Rigid3{
      Eigen::Vector3d(json_rev_data_["x"].asDouble(),
                      json_rev_data_["y"].asDouble(), 0.0),
      slam2d_core::common::yawToQuaternion(0.0, 0.0,
                                           json_rev_data_["yaw"].asDouble())};

  LOG(INFO) << "init mode: " << json_rev_data_["init_model"].asString();
  if ("fix_pose" == json_rev_data_["init_model"].asString()) {
    init_modle = slam2d_core::slam_system::InitModle::FIX_POSE;
  } else if ("reliable_pose" == json_rev_data_["init_model"].asString()) {
    init_modle = slam2d_core::slam_system::InitModle::RELIABLE_POSE;
  } else {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "can not find init modle.";
    LOG(ERROR) << "can not find init modle.";
    return false;
  }

  ptr_slam_system_->startLocalization(map_file_path, init_modle, init_pose);

  json_ack_data_["succeed"] = true;
  return true;
}

bool MissionManagerRos2::startMapping() {
  if (json_rev_data_["realtime_map_file"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "realtime_map_file dir_path empty.";
    LOG(ERROR) << "realtime_map_file dir_path is empty, default temp map path "
                  "will be used.";
    // return; // TODO:任务管理暂无此字段接口，暂时不能return
  } else {
    std::string default_temp_map_filepath =
        json_rev_data_["realtime_map_file"].asString();
    if (default_temp_map_filepath.back() != '/') {
      default_temp_map_filepath.push_back('/');
    }

    ptr_depth_occ_map_->Reset();
    if (ptr_depth_occ_map_->useOcc()) {
      ptr_slam_system_->startMapping(default_temp_map_filepath + "map_2d_loc");
      ptr_depth_occ_map_->startMapping(default_temp_map_filepath + "map_2d");
    } else {
      ptr_slam_system_->startMapping(default_temp_map_filepath + "map_2d");
    }
  }

  json_ack_data_["succeed"] = true;
  LOG(INFO) << "**************start mapping*************";
  return true;
}

bool MissionManagerRos2::returnMapping() {
  if (json_rev_data_["dir_path"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "dir_path empty.";
    return false;
  }
  std::string data_dir_path = json_rev_data_["dir_path"].asString();
  if (data_dir_path.back() != '/') {
    data_dir_path.push_back('/');
  }

  if (json_rev_data_["realtime_map_file"].empty()) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "realtime_map_file dir_path empty.";
    LOG(ERROR) << "realtime_map_file dir_path is empty, default temp map path "
                  "will be used.";
    return false;
  }
  std::string default_temp_map_filepath =
      json_rev_data_["realtime_map_file"].asString();
  if (default_temp_map_filepath.back() != '/') {
    default_temp_map_filepath.push_back('/');
  }
  bool map_2d_loc_flag = true;
  bool map_2d_flag = true;
  if (access((data_dir_path + "2d_map/map_2d_loc.pbstream").c_str(), 0) != 0) {
    LOG(WARNING) << data_dir_path + "2d_map/map_2d_loc.pbstream"
                 << " is not exist";
    map_2d_loc_flag = false;
  }

  if (access((data_dir_path + "2d_map/map_2d.pbstream").c_str(), 0) != 0) {
    LOG(WARNING) << data_dir_path + "2d_map/map_2d.pbstream"
                 << " is not exist";
    map_2d_flag = false;
  }

  std::string pb_map_file_path = data_dir_path + "2d_map/map_2d.pbstream";

  if (map_2d_loc_flag) {
    pb_map_file_path = data_dir_path + "2d_map/map_2d_loc.pbstream";
  }
  if (map_2d_flag) {
    LOG(WARNING) << data_dir_path << " map version is old";
  }

  if (!map_2d_flag && !map_2d_loc_flag) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "pbstream data empty.";
    LOG(ERROR) << pb_map_file_path << " pbstream is not exist";
    return false;
  }

  auto pose = ptr_slam_system_->getCurrentPose();

  ptr_slam_system_->stopLocalization();

  if (ptr_depth_occ_map_->useOcc()) {
    ptr_slam_system_->startMapping(default_temp_map_filepath + "map_2d_loc");
    ptr_depth_occ_map_->loadScan(data_dir_path + "2d_map/map_2d");
    ptr_depth_occ_map_->startMapping(default_temp_map_filepath + "map_2d");
  } else {
    ptr_slam_system_->startMapping(default_temp_map_filepath + "map_2d");
  }
  ptr_slam_system_->loadSerializeStateFromFile(pb_map_file_path);
  ptr_slam_system_->setReturnMappingPose(pose);
  return true;
}

bool MissionManagerRos2::saveMapping() {
  auto save_map_all_start_time = std::chrono::system_clock::now();

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

  std::string map_2d_file_path, occ_map_file_path;
  std::string save_path = json_rev_data_["dir_path"].asString();
  std::string save_path_file = json_rev_data_["path_file"].asString();

  if (access(save_path.c_str(), 0) == -1) {
    int flag = mkdir(save_path.c_str(), 0777);
    int map_2d_flag = mkdir((save_path + "/2d_map/").c_str(), 0777);
    int nav_point_flag = mkdir((save_path + "/nav_point/").c_str(), 0777);

    if (flag == 0 && map_2d_flag == 0 && nav_point_flag == 0) {
      LOG(INFO) << " make dir successfully: " << save_path;
    } else {
      LOG(ERROR) << "make dir failed: " << save_path;
      json_ack_data_["succeed"] = false;
      json_ack_data_["error_msgs"] = "make dir failed.";
      return false;
    }
  } else {
    LOG(WARNING) << "dir_path exist: " << save_path;
    std::string map_2d_path = save_path + "/2d_map/";
    std::string nav_point_path = save_path + "/nav_point/";

    int map_2d_flag = -1;
    int nav_point_flag = -1;

    if (access(map_2d_path.c_str(), 0) == -1) {
      map_2d_flag = mkdir((map_2d_path).c_str(), 0777);
    }
    if (access(nav_point_path.c_str(), 0) == -1) {
      nav_point_flag = mkdir((nav_point_path).c_str(), 0777);
    }

    if (map_2d_flag == 0 && nav_point_flag == 0) {
      LOG(INFO) << " make dir successfully: " << save_path;
    } else {
      LOG(ERROR) << "make dir failed: " << save_path;
    }
  }

  Json::Value point_json;
  bool get_navi_point = getPartolPoseJson(point_json);
  // TODO(icanchen): save partol pose to file.

  partol_pose_id_.clear();

  map_2d_file_path = save_path + "/2d_map/map_2d";
  if (ptr_depth_occ_map_->useOcc()) {
    map_2d_file_path = save_path + "/2d_map/map_2d_loc";
    occ_map_file_path = save_path + "/2d_map/map_2d";
  }

  auto runfinaloptimization_start_time = std::chrono::system_clock::now();
  ptr_slam_system_->runFinalOptimization();
  auto runfinaloptimization_end_time = std::chrono::system_clock::now();

  if (!save_ptfile_thread_.joinable()) {
    std::string ptfile_name = map_2d_file_path + ".pbstream";
    save_ptfile_thread_ =
        std::thread(std::bind(&MissionManagerRos2::savePtfileThread, this,
                              std::placeholders::_1),
                    ptfile_name);
  }

  if (!save_map_thread_.joinable()) {
    save_map_thread_ = std::thread(std::bind(&MissionManagerRos2::saveMapThread,
                                             this, std::placeholders::_1),
                                   map_2d_file_path);
  }

  if (ptr_depth_occ_map_->useOcc()) {
    if (!save_depthmap_thread_.joinable()) {
      save_depthmap_thread_ =
          std::thread(std::bind(&MissionManagerRos2::saveDephtmapThread, this,
                                std::placeholders::_1),
                      occ_map_file_path);
    }
  } else {
    finish_save_depthmap_ = true;
  }

  while (!finish_save_ptfile_ || !finish_save_map_ || !finish_save_depthmap_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (save_ptfile_thread_.joinable()) {
    save_ptfile_thread_.join();
  }

  if (save_map_thread_.joinable()) {
    save_map_thread_.join();
  }

  if (save_depthmap_thread_.joinable()) {
    save_depthmap_thread_.join();
  }

  ptr_slam_system_->canceMapping();

  auto current_pose = ptr_slam_system_->getCurrentPose();

  json_ack_data_["save_node_info"] = true;
  json_ack_data_["gps_trans"] = false;
  json_ack_data_["path_file"] = save_path_file;
  json_ack_data_["last_pose"]["x"] = current_pose.translation().x();
  json_ack_data_["last_pose"]["y"] = current_pose.translation().y();
  json_ack_data_["last_pose"]["z"] = 0.0;
  json_ack_data_["last_pose"]["yaw"] =
      slam2d_core::common::quaternionToYaw(current_pose.rotation());

  if (get_navi_point) {
    json_ack_data_["navi_point"] = point_json["navi_point"];
  }
  json_ack_data_["succeed"] = true;

  auto save_map_all_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> save_map_all_cost_time =
      save_map_all_end_time - save_map_all_start_time;
  std::chrono::duration<double> runfinaloptimization_cost_time =
      runfinaloptimization_end_time - runfinaloptimization_start_time;

  LOG(INFO) << ">>----------------save map all time : "
            << save_map_all_cost_time.count() * 1000
            << " runfinaloptimization_cost_time : "
            << runfinaloptimization_cost_time.count() * 1000;

  return true;
}

void MissionManagerRos2::savePtfileThread(const std::string &file_name) {
  auto save_serializeStateToFile_start_time = std::chrono::system_clock::now();
  finish_save_ptfile_ = false;
  ptr_slam_system_->serializeStateToFile(file_name);
  finish_save_ptfile_ = true;
  auto save_serializeStateToFile_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> serializeStateToFile_cost_time =
      save_serializeStateToFile_end_time - save_serializeStateToFile_start_time;

  LOG(INFO) << ">>----------------serializeStateToFile_cost_time: "
            << serializeStateToFile_cost_time.count() * 1000;
}

void MissionManagerRos2::saveMapThread(const std::string &file_name) {
  auto save2dmap_start_time = std::chrono::system_clock::now();
  finish_save_map_ = false;
  ptr_slam_system_->requestSaveMap(file_name);
  finish_save_map_ = true;
  auto save2dmap_end_time = std::chrono::system_clock::now();

  std::chrono::duration<double> save2dmap_cost_time =
      save2dmap_end_time - save2dmap_start_time;

  LOG(INFO) << ">>----------------save2dmap_cost_time: "
            << save2dmap_cost_time.count() * 1000;
}

void MissionManagerRos2::saveDephtmapThread(const std::string &file_name) {
  auto savedepthoccmap_start_time = std::chrono::system_clock::now();
  finish_save_depthmap_ = false;
  ptr_slam_system_->mapLock();
  saveDepthOccMap(file_name);
  ptr_slam_system_->mapUnLock();
  finish_save_depthmap_ = true;
  auto savedepthoccmap_end_time = std::chrono::system_clock::now();

  std::chrono::duration<double> savedepthoccmap_cost_time =
      savedepthoccmap_end_time - savedepthoccmap_start_time;

  LOG(INFO) << " >>----------------savedepthoccmap_cost_time : "
            << savedepthoccmap_cost_time.count() * 1000;
}

void MissionManagerRos2::saveDepthOccMap(const std::string &map_file) {
  ptr_depth_occ_map_->clearTmpMapWithLock();
  auto paint_submap_slices = ptr_slam_system_->paintSubmapSlices(0.05);
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> ptr_map =
      createOccupancyGridMsg(paint_submap_slices, 0.05, "map",
                             ptr_clock_->now());

  if (ptr_map->data.size() <= 0) {
    return;
  }
  slam2d_core::occupancy_map::UserOccupancyGrid global_occ_map;
  global_occ_map.info.width = ptr_map->info.width;
  global_occ_map.info.height = ptr_map->info.height;
  global_occ_map.info.resolution = ptr_map->info.resolution;
  global_occ_map.data.clear();
  global_occ_map.data.resize(ptr_map->info.width * ptr_map->info.height);
  std::fill(global_occ_map.data.begin(), global_occ_map.data.end(), -1);
  global_occ_map.info.origin(0, 3) = ptr_map->info.origin.position.x;
  global_occ_map.info.origin(1, 3) = ptr_map->info.origin.position.y;
  for (size_t i = 0; i < ptr_map->data.size(); i++) {
    global_occ_map.data[i] = ptr_map->data[i];
  }
  if (ptr_depth_occ_map_ != nullptr) {
    if (ptr_depth_occ_map_->useOcc()) {
      const auto node_poses = ptr_slam_system_->getTrajectoryNodePoses();
      if (ptr_depth_occ_map_->updateGlobalMap(node_poses, global_occ_map)) {
        // ptr_depth_occ_map_->saveGlobalMap();
      }
      ptr_depth_occ_map_->saveMap(map_file);
    } else {
      ptr_depth_occ_map_->setGlobalMap(global_occ_map);
      ptr_depth_occ_map_->saveMap(map_file);
    }
  }
}

bool MissionManagerRos2::cancelMapping() {
  if (nullptr == ptr_slam_system_) {
    json_ack_data_["succeed"] = false;
    json_ack_data_["error_msgs"] = "system ptr null.";
    LOG(ERROR) << "system ptr null.";
    return false;
  }

  ptr_slam_system_->runFinalOptimization();
  ptr_slam_system_->canceMapping();

  ptr_depth_occ_map_->Reset();
  partol_pose_id_.clear();
  json_ack_data_["succeed"] = true;
  return true;
}

bool MissionManagerRos2::getStatus() {
  if (ptr_slam_system_->isLocalizationInit() &&
      ptr_state_machine_->getCurrentState() ==
          slam2d_core::state_machine::AD_STATU::INIT_LOCALIZATION) {
    ptr_state_machine_->sendEvent(
        slam2d_core::state_machine::EventName::INIT_LOCALIZATION_SUCCESS);
  }

  std::string status = ptr_state_machine_->getCurrentStateName();
  std::string model = ptr_state_machine_->getCurrentModeName();

  auto pose = ptr_slam_system_->getCurrentPose();
  bool loc_status = ptr_slam_system_->getLocStatus();

  json_ack_data_["bx"] = pose.translation().x();
  json_ack_data_["by"] = pose.translation().y();
  json_ack_data_["bz"] = 0.0;
  json_ack_data_["byaw"] =
      slam2d_core::common::quaternionToYaw(pose.rotation());

  auto laser_tf = ptr_slam_system_->getLaserTf();
  auto laser_pose = pose * laser_tf;

  json_ack_data_["x"] = laser_pose.translation().x();
  json_ack_data_["y"] = laser_pose.translation().y();
  json_ack_data_["z"] = 0.0;
  json_ack_data_["yaw"] =
      slam2d_core::common::quaternionToYaw(laser_pose.rotation());

  json_ack_data_["loc_status"] = loc_status;

  json_ack_data_["succeed"] = true;
  json_ack_data_["status"] = status;
  json_ack_data_["model"] = model;

  json_ack_data_["dx"] = laser_tf.inverse().translation().x();
  json_ack_data_["dy"] = laser_tf.inverse().translation().y();
  json_ack_data_["dyaw"] =
      slam2d_core::common::quaternionToYaw(laser_tf.inverse().rotation());

  if (ptr_state_machine_->getCurrentMode() ==
          slam2d_core::state_machine::AD_MODE::AD_MAPPING &&
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
    int node_id = it->second.id;

    slam2d_core::common::Rigid3 node_pose;

    if (ptr_slam_system_->getTrajectoryNodePose(node_id, node_pose)) {
      slam2d_core::common::Rigid3 partol_pose;
      partol_pose = node_pose * it->second.pose_compensation;
      Json::Value pose;
      pose["name"] = it->first;
      pose["x"] = partol_pose.translation().x();
      pose["y"] = partol_pose.translation().y();
      pose["z"] = 0.0;
      pose["yaw"] =
          slam2d_core::common::quaternionToYaw(partol_pose.rotation());
      point_json["navi_point"].append(pose);
    }

    it++;
  }
  return true;
}

}  // namespace slam2d_ros2

int main(int argc, char **argv) {
  using namespace slam2d_ros2;
  rclcpp::init(argc, argv);
  CVTE_BABOT::initGoogleLog("slam2d", "info");
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr mission_manager_node =
      std::make_shared<rclcpp::Node>("slam2d_mission_manager", options);
  // rclcpp::Node::SharedPtr ros2_adpter_node =
  //     std::make_shared<rclcpp::Node>("slam2d_ros2_adpter", options);

  MissionManagerRos2 mission_node(mission_manager_node);
  Ros2Adapter ros2_adpter(mission_manager_node);
  std::shared_ptr<slam2d_core::state_machine::SlamStateMachine>
      ptr_state_machine =
          slam2d_core::state_machine::SlamStateMachine::getInstance();

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::START_MAPPING,
      std::bind(&MissionManagerRos2::startMapping, &mission_node));

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::SAVE_MAP,
      std::bind(&MissionManagerRos2::saveMapping, &mission_node));

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::CANCLE_MAPPING,
      std::bind(&MissionManagerRos2::cancelMapping, &mission_node));

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::START_LOCALIZATION,
      std::bind(&MissionManagerRos2::startLocalization, &mission_node));

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::STOP_LOCALIZAION,
      std::bind(&MissionManagerRos2::stopLocalization, &mission_node));

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::GET_STATE,
      std::bind(&MissionManagerRos2::getStatus, &mission_node));

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::SET_PARTOL_POINT,
      std::bind(&MissionManagerRos2::addPartolPose, &mission_node));

  ptr_state_machine->registerTransActionCallback(
      slam2d_core::state_machine::EventName::RETURN_MAPPING,
      std::bind(&MissionManagerRos2::returnMapping, &mission_node));

  ptr_state_machine->registerEntryActionCallback(
      slam2d_core::state_machine::AD_STATU::UNKNOW,
      std::bind(&Ros2Adapter::resetDataCallback, &ros2_adpter));

  ptr_state_machine->registerEntryActionCallback(
      slam2d_core::state_machine::AD_STATU::INIT_LOCALIZATION,
      std::bind(&Ros2Adapter::registerDataCallback, &ros2_adpter));

  ptr_state_machine->registerEntryActionCallback(
      slam2d_core::state_machine::AD_STATU::MAPPING,
      std::bind(&Ros2Adapter::registerDataCallback, &ros2_adpter));

  rclcpp::executors::SingleThreadedExecutor exec;
  // rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(mission_manager_node);
  // exec.add_node(ros2_adpter_node);
  exec.spin();
  exec.remove_node(mission_manager_node);
  // exec.remove_node(ros2_adpter_node);

  rclcpp::shutdown();
  return 0;
}
