#include "slam_system.hpp"

namespace EIOLIDAR_SLAM {
System *System::ptr_system_ = nullptr;

System::System() {
  // std::cout << " do noting " << std::endl;
}

System *System::getInstance() {
  static System eiolidar_slam_system;
  return &eiolidar_slam_system;
}
}  // namespace EIOLIDAR_SLAM