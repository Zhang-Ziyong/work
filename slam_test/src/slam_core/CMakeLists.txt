# cmake_minimum_required(VERSION 3.5)
project(eiolidar_slam)

# add_definitions(-std=c++11) # Use C++11
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(chassis_interfaces REQUIRED)

# find_package(ament_cmake REQUIRED)
include_directories(
  slam_core
  ${PROJECT_SOURCE_DIR}
  ${PCL_INCLUDE_DIRS}
)

FILE(GLOB SDK_SRC
  "slam_system/*.cpp"
  "slam_system/*.hpp"
  "common/*.cpp"
  "common/*.hpp"

  # "src/impl/unix/*.cpp"
  # "src/impl/unix/*.h"
)

# add_subdirectory(samples)
add_library(eiolidar_slam_lib SHARED ${SDK_SRC})
ament_target_dependencies(eiolidar_slam_lib
  chassis_interfaces

  # "sensor_msgs"
  # "std_msgs"
  # "nav_msgs"
  # "geometry_msgs"
)

target_link_libraries(eiolidar_slam_lib
  rt
  pthread
  -lglog
  -ljsoncpp
  ${PCL_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

# add_executable(eiolidar_slam_exe
# src/ros_adapter/slam_ros.cpp
# )
# target_link_libraries(eiolidar_slam_exe eiolidar_slam ${PCL_LIBRARIES})
# install(
# DIRECTORY params/
# DESTINATION params/
# )

# install(
# DIRECTORY include/
# DESTINATION include/
# )
install(TARGETS
  eiolidar_slam_lib
  DESTINATION lib
)

# install(TARGETS eiolidar_slam_exe
# DESTINATION lib/${PROJECT_NAME})

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})

# ament_package()