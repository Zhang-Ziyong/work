#ifndef _NEW_MISSION_MANAGER_H_
#define _NEW_MISSION_MANAGER_H_
/*
 * @Author: your name
 * @Date: 2020-07-17 16:08:33
 * @LastEditTime: 2020-08-05 09:30:51
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /cvte_lidar_slam/ros_adapter/new_mission_manager.hpp
 */
#include <jsoncpp/json/json.h>
#include <sys/file.h>
#include <fstream>
#include "occupancy_map/new_occupancy_map.hpp"
#include "common/math_base/slam_math.hpp"
#include "system/system.hpp"
#include <rclcpp/rclcpp.hpp>
#include "mission_manager_msgs/srv/mission_manager.hpp"
#include "state_machine/slam_state_machine.hpp"
#include "new_lidar_slam_ros2.hpp"
#include "public_parameters/PublicParameters.hpp"
#include "zmq_server.hpp"

namespace cvte_lidar_slam {
struct PartolPointInfo {
  size_t id;
  Mat34d pose_compensation;
};

class MissionManagerRos2 {
 public:
  MissionManagerRos2(rclcpp::Node::SharedPtr node);
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

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<mission_manager_msgs::srv::MissionManager>::SharedPtr
      mission_manager_server_;

  System *ptr_slam_system_;
  std::shared_ptr<OccMap> ptr_occ_map_;
  std::shared_ptr<DepthOccupancyMap> ptr_depth_occ_map_ =
      nullptr;  // 深度地图指针
  std::shared_ptr<SlamStateMachine> ptr_state_machine_;

  Json::Value json_rev_data_;
  Json::Value json_ack_data_;

  std::map<std::string, EventName> cmd_events_;

  std::unordered_map<std::string, PartolPointInfo>
      partol_pose_id_;  ///< 存储所有巡逻点ID.

  Mat34d T_lo_;

  std::shared_ptr<ZmqService> zmq_server_ = nullptr;
  PublicParameters params_;
  std::string zmq_server_addr_;
  unsigned short slam2d_server_port_;
};
}  // namespace cvte_lidar_slam
#endif