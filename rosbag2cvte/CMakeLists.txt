cmake_minimum_required(VERSION 3.5)
project(rosbag2cvte)

include(cmake/PICMake.cmake)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

pi_add_target(rosbag2cvte BIN src REQUIRED System rclcpp rosbag2_cpp DEPENDENCY zstd)

pi_report_target()

pi_install(TARGETS rosbag2cvte)

