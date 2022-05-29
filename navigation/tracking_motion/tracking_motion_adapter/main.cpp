/*
 * @Author: your name
 * @Date: 2020-11-02 15:33:36
 * @LastEditTime: 2020-11-14 11:09:06
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /src/tracking_motion/tracking_motion_adapter/main.cpp
 */
#include "tracking_motion_adapter.hpp"
#include <signal.h>
#include "log.hpp"

void sigintHandler(int sig) {
  LOG(ERROR) << "kava_communication_ros2 Ctr-C signal: " << sig << std::endl;
  rclcpp::shutdown();
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  CVTE_BABOT::initGoogleLog("tracking_motion", "warn");
  signal(SIGINT, sigintHandler);
  TRACKING_MOTION::TrackingMotionAdapter tracking_motion_adapter;
  // tracking_motion_adapter.start();
  tracking_motion_adapter.spin();
  rclcpp::shutdown();
  return 0;
}
