#include <rclcpp/rclcpp.hpp>
#include "manager_ros2/map_manager_ros2.hpp"

using namespace CVTE_BABOT;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ptr_costmap_arch = CostmapArchAdapter::getPtrInstance();

  ptr_costmap_arch->updateParameter();
  if (ptr_costmap_arch->systemInit()) {
    ptr_costmap_arch->startCostmapTimer();
    ptr_costmap_arch->spin();
  } else {
    LOG(ERROR) << "costmap_arch_adapter init failed.";
  }

  rclcpp::shutdown();

  return 0;
}