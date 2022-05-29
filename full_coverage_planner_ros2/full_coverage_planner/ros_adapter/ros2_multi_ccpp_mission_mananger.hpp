#ifndef ROS2_MULTI_CCPP_MISSION_MANAGER
#define ROS2_MULTI_CCPP_MISSION_MANAGER
#include <rclcpp/rclcpp.hpp>
#include "cpp_interface/full_coverage_planner_config.hpp"
#include "mission_manager_msgs/srv/mission_manager.hpp"

class MultiCCPPMissionManager {
 public:
  MultiCCPPMissionManager(rclcpp::Node::SharedPtr node);
  MultiCCPPMissionManager() = delete;
  MultiCCPPMissionManager &operator=(const MultiCCPPMissionManager &obj) =
      delete;
  MultiCCPPMissionManager(const MultiCCPPMissionManager &obj) = delete;
  ~MultiCCPPMissionManager() = default;

 private:
  void missionManagerCallback(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
          request,
      const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
          response);

  void missionHandler(std::string rev_data, std::string &ack_data);
  void missionHandler();

  void fullCoveragerPlanner(std::string rev_data, std::string &ack_data);

  void fullCoveragerPlanner();
  void weltPathPlanner(std::string rev_data, std::string &ack_data);
  void fullCoveragerAneCloseWeltPlanner(std::string rev_data,
                                        std::string &ack_data);
  void loadParamters();
  FullCoveragePlanerConfig config_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<mission_manager_msgs::srv::MissionManager>::SharedPtr
      mission_manager_server_;
};

#endif