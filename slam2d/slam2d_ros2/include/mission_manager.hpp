#ifndef _MISSION_MANAGER_H_
#define _MISSION_MANAGER_H_

#include <jsoncpp/json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <map>
#include <thread>
#include <unordered_map>
#include <memory>

#include "state_machine/slam_state_machine.hpp"
#include "mission_manager_msgs/srv/mission_manager.hpp"
#include "ros2_adapter.hpp"
#include "slam_system/slam_system.hpp"
#include "occupancy_map/depth_camera_occ_map.hpp"

namespace slam2d_ros2 {

struct PartolPointInfo {
  int id;
  slam2d_core::common::Rigid3 pose_compensation;
};

class MissionManagerRos2 {
 public:
  explicit MissionManagerRos2(rclcpp::Node::SharedPtr node);
  MissionManagerRos2() = delete;
  MissionManagerRos2 &operator=(const MissionManagerRos2 &obj) = delete;
  MissionManagerRos2(const MissionManagerRos2 &obj) = delete;

  void spin();
  bool startMapping();
  bool returnMapping();
  bool saveMapping();
  bool cancelMapping();
  bool startLocalization();
  bool stopLocalization();
  bool getStatus();
  bool addPartolPose();

 private:
  bool getPartolPoseJson(Json::Value &point_json);

  void missionHandler(std::string rev_data, std::string &ack_data);
  void missionManagerCallback(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
          request,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
          response);
  void saveDepthOccMap(const std::string &map_file);

  void savePtfileThread(const std::string &file_name);
  void saveMapThread(const std::string &file_name);
  void saveDephtmapThread(const std::string &file_name);

 private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Clock::SharedPtr ptr_clock_ = nullptr;  ///< 时钟指针
  rclcpp::Service<mission_manager_msgs::srv::MissionManager>::SharedPtr
      mission_manager_server_;

  slam2d_core::slam_system::SlamSystem *ptr_slam_system_ = nullptr;
  std::shared_ptr<slam2d_core::occupancy_map::DepthOccupancyMap>
      ptr_depth_occ_map_ = nullptr;  ///< 深度相机避障地图
  //   std::shared_ptr<OccMap> ptr_occ_map_;
  std::shared_ptr<slam2d_core::state_machine::SlamStateMachine>
      ptr_state_machine_;

  Json::Value json_rev_data_;
  Json::Value json_ack_data_;

  std::map<std::string, slam2d_core::state_machine::EventName> cmd_events_;

  std::unordered_map<std::string, PartolPointInfo>
      partol_pose_id_;  ///< 存储所有巡逻点ID.

  std::string config_file_;

  std::thread save_ptfile_thread_;
  std::thread save_map_thread_;
  std::thread save_depthmap_thread_;

  bool finish_save_ptfile_ = false;
  bool finish_save_map_ = false;
  bool finish_save_depthmap_ = false;
};

}  // namespace slam2d_ros2

#endif
