cmake_minimum_required(VERSION 3.5)
project(tracking_motion)

# Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(log REQUIRED)
find_package(mission_manager_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(
common
potential_controller
laser_object_track
KF
tracking_motion_adapter
tracking_motion_adapter_ros2
common_math
${mission_manager_msgs_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)


link_directories(
  ${PCL_LIBRARY_DIRS}
)




add_library(${PROJECT_NAME}_lib 
KF/kalman_filter.cpp
laser_object_track/laser_object_track.cpp
potential_controller/potential_controller.cpp 
tracking_motion_adapter/tracking_motion_adapter.cpp 
tracking_motion_adapter_ros2/tracking_motion_adapter_ros2.cpp 
)

ament_target_dependencies(${PROJECT_NAME}_lib
"rclcpp"
"geometry_msgs"
"sensor_msgs"
"nav_msgs"
"log"
"mission_manager_msgs"
"PCL"
)
target_link_libraries(${PROJECT_NAME}_lib
                      ${PCL_LIBRARIES}
                      -lglog)
add_executable(${PROJECT_NAME}_ros2 tracking_motion_adapter/main.cpp)

target_link_libraries(${PROJECT_NAME}_ros2 
${PROJECT_NAME}_lib
-lglog
-ljsoncpp)

install(
  DIRECTORY params/
  DESTINATION params/)

install(TARGETS ${PROJECT_NAME}_ros2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
