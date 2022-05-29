#include "arch_adapter/navigation_arch_adapter_ros2.hpp"
#include "log.hpp"
#include <signal.h>
#include "SwVersion.hpp"

using namespace CVTE_BABOT;

void sigintHandler(int sig) {
  std::cout << "navigation_ros2 Ctr-C signal: " << sig << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  signal(SIGINT, sigintHandler);
  CVTE_BABOT::initGoogleLog("navigation_ros2", "warn");
  LOG(INFO) << "SW_VERSION : " << SW_VERSION;

  CVTE_BABOT::NavigationArchAdapter navigation_arch_adapter;

  navigation_arch_adapter.spin();
  rclcpp::shutdown();

  return 0;
}