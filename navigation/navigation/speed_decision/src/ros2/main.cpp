/*
 * @Author: your name
 * @Date: 2021-06-08 20:39:54
 * @LastEditTime: 2021-06-09 11:18:46
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/speed_decision/src/main.cpp
 */
#include "speed_decision_adapter.hpp"
#include "log.hpp"

void sigintHandler(int sig) {
  LOG(ERROR) << "kava_communication_ros2 Ctr-C signal: " << sig << std::endl;
  rclcpp::shutdown();
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  CVTE_BABOT::initGoogleLog("speed_decision", "warn");
  signal(SIGINT, sigintHandler);
  CVTE_BABOT::SpeedDecisionAdapter speed_decision_adapter;
  speed_decision_adapter.start();
  speed_decision_adapter.spin();
  rclcpp::shutdown();
  return 0;
}