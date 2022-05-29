cmake_minimum_required(VERSION 3.5)
project(cvte_lidar_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/slam_core/cmake")
message("--Found CMake in ${PROJECT_SOURCE_DIR}/slam_core/cmake")

set(BUILD_LIB TRUE)

if(BUILD_LIB)
  add_subdirectory(slam_core)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(visualization_msgs REQUIRED)
find_package(mission_manager_msgs REQUIRED)
find_package(tlsf_cpp)
find_package(log REQUIRED)

include_directories(
  slam_core
  ros_adapter
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rtt_executor_INCLUDE_DIRS}
  ${mission_manager_msgs_INCLUDE_DIRS}
  ${cloud_feature_msgs_INCLUDE_DIRS}
  ${pose2d_INCLUDE_DIRS}
  ${log_INCLUDE_DIRS}
)

link_directories(
	${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

if(BUILD_LIB) 
  set(LIDAR_SLAM_LIB slam_lib)
else()
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message("CMAKE_SYSTEM_PROCESSOR MATCHES x86_64")
    file(GLOB LIDAR_SLAM_LIB ${PROJECT_SOURCE_DIR}/slam_core/lib/x86/libslam_lib.so)
  elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") 
    message("CMAKE_SYSTEM_PROCESSOR MATCHES aarch64")
    file(GLOB LIDAR_SLAM_LIB ${PROJECT_SOURCE_DIR}/slam_core/lib/arm/libslam_lib.so)
  endif ()
endif()



#############
#executable##
#############
add_executable(lidar_slam_mission_manager 
   ros_adapter/new_mission_manager.cpp
   ros_adapter/new_lidar_slam_ros2.cpp
)

ament_target_dependencies(lidar_slam_mission_manager
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_msgs"
  "cv_bridge"
  "OpenCV"
  "log"
  "mission_manager_msgs")

target_link_libraries(lidar_slam_mission_manager
   ${LIDAR_SLAM_LIB}
   ${Boost_SYSTEM_LIBRARY}
   ${PCL_LIBRARIES}
   -lglog
)


#############
## Install ##
#############
install(TARGETS
lidar_slam_mission_manager 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lidar_slam_mission_manager
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY params/
  DESTINATION params/)


if(BUILD_LIB)
else()
  message("Precompiled slamcore lib will be installed")
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    install(
      FILES  slam_core/lib/x86/libslam_lib.so
      DESTINATION lib)
  elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") 
    install(
      FILES  slam_core/lib/arm/libslam_lib.so
      DESTINATION lib)
  endif ()

endif()


# install(
#   DIRECTORY config/
#   DESTINATION config/)

ament_export_dependencies("OpenCV")
ament_export_libraries(slam_lib)
  
#############
## Testing ##
#############
ament_package()
