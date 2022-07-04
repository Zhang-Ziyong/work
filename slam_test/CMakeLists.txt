cmake_minimum_required(VERSION 3.5)
project(eiolidar_slam)

# add_definitions(-std=c++11) # Use C++11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/src/slam_core/cmake")
set(BUILD_LIB TRUE)

if(BUILD_LIB)
  add_subdirectory(src/slam_core)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(log REQUIRED)
find_package(chassis_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(chassis_interfaces REQUIRED)
include_directories(
  src/slam_core
  src/ros_adapter
  ${log_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

if(BUILD_LIB)
  set(LIDAR_SLAM_LIB eiolidar_slam_lib)
else()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message("CMAKE_SYSTEM_PROCESSOR MATCHES x86_64")
    file(GLOB LIDAR_SLAM_LIB ${PROJECT_SOURCE_DIR}/slam_core/lib/x86/libeiolidar_slam_lib.so)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("CMAKE_SYSTEM_PROCESSOR MATCHES aarch64")
    file(GLOB LIDAR_SLAM_LIB ${PROJECT_SOURCE_DIR}/slam_core/lib/arm/libeiolidar_slam_lib.so)
  endif()
endif()

FILE(GLOB SDK_SRC
  "src/ros_adapter/*.cpp"
  "src/ros_adapter/*.h"

  # "src/impl/unix/*.cpp"
  # "src/impl/unix/*.h"
)

# add_subdirectory(samples)
add_library(eiolidar_slam STATIC ${SDK_SRC})
ament_target_dependencies(eiolidar_slam
  "rclcpp"
  "log"
  "sensor_msgs"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
  chassis_interfaces
)

target_link_libraries(eiolidar_slam
  rt
  pthread
  -lglog
  ${LIDAR_SLAM_LIB}
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(eiolidar_slam_exe
  src/ros_adapter/slam_ros.cpp
)

target_link_libraries(eiolidar_slam_exe eiolidar_slam)

install(
  DIRECTORY params/
  DESTINATION params/
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS eiolidar_slam_exe
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_LIB)
else()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    install(
      FILES slam_core/lib/x86/libeiolidar_slam_lib.so
      DESTINATION lib)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    install(
      FILES slam_core/lib/arm/libeiolidar_slam_lib.so
      DESTINATION lib)
  endif()
endif()

ament_export_include_directories(include)

# ament_export_dependencies("OpenCV")
ament_export_libraries(${PROJECT_NAME})
ament_export_libraries(eiolidar_slam_lib)

ament_package()