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

namespace cvte_lidar_slam {

struct PartolPointInfo {
  size_t id;
  Mat34d pose_compensation;
};

class NavPoints {
 public:
  using NavPointsType = std::unordered_map<std::string, PartolPointInfo>;
  static bool saveNavPoints(const NavPointsType &nav_points,
                            const std::string &path) {
    if (nav_points.empty()) {
      return false;
    }
    std::ofstream nav_points_file;
    const std::string &nav_points_data_dir =
        path + "/nav_point/nav_points_data.txt";
    nav_points_file.open(nav_points_data_dir);
    nav_points_file << std::fixed;
    if (!nav_points_file.is_open()) {
      LOG(ERROR) << " Can not open file";
      return false;
    }
    nav_points_file << "#format: frame_id tx ty tz qw qx qy qz" << std::endl;
    for (auto &it : nav_points) {
      const std::string &name = it.first;
      const size_t &frame_id = (size_t) it.second.id;
      const Mat34d &temp_pose = it.second.pose_compensation;
      Eigen::Quaterniond quat(temp_pose.block<3, 3>(0, 0));
      quat.normalize();
      nav_points_file << name << ' ' << std::setprecision(0) << frame_id << ' '
                      << std::setprecision(6) << temp_pose(0, 3) << ' '
                      << temp_pose(1, 3) << ' ' << temp_pose(2, 3) << ' '
                      << quat.x() << ' ' << quat.y() << ' ' << quat.z() << ' '
                      << quat.w() << std::endl;
    }
    nav_points_file.close();
    return true;
  }
  static bool loadNavPoints(const std::string &path,
                            NavPointsType &nav_points) {
    std::ifstream nav_points_file;
    const std::string &nav_points_data_dir =
        path + "/nav_point/nav_points_data.txt";
    nav_points_file.open(nav_points_data_dir);
    if (!nav_points_file.is_open()) {
      LOG(ERROR) << " Can not open file";
      return false;
    }
    PartolPointInfo partol_point;
    nav_points.clear();
    while (!nav_points_file.eof()) {
      double pose[7] = {0.};
      std::string s;
      std::string point_name;
      int temp = -1;
      std::getline(nav_points_file, s);
      if (s.front() == '#' || s.empty()) {
        continue;
      }
      std::stringstream ss;
      ss << s;
      ss >> point_name;
      ss >> partol_point.id;
      for (uint i = 0; i < 7; ++i) { ss >> pose[i]; }
      Eigen::Quaterniond Quat(pose[6], pose[3], pose[4], pose[5]);
      Eigen::Map<Eigen::Vector3d> Trans(pose);
      Quat.normalize();

      partol_point.pose_compensation.block<3, 3>(0, 0) =
          Quat.toRotationMatrix();
      partol_point.pose_compensation.block<3, 1>(0, 3) = Trans;
      nav_points[point_name] = partol_point;
    }
    LOG(WARNING) << "Load nav points successfully: " << nav_points.size();
    return true;
  }
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
  std::shared_ptr<SlamStateMachine> ptr_state_machine_;

  Json::Value json_rev_data_;
  Json::Value json_ack_data_;

  std::map<std::string, EventName> cmd_events_;

  std::unordered_map<std::string, PartolPointInfo>
      partol_pose_id_;  ///< 存储所有巡逻点ID.

  Mat34d T_lo_;

  std::string config_file_;
};
}  // namespace cvte_lidar_slam
