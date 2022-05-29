cmake_minimum_required(VERSION 3.5)
project(slam2d_ros2)

find_package(ament_cmake REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -g)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# find dependencies
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(log REQUIRED)
find_package(slam2d_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mission_manager_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED filesystem)
find_package(chassis_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(public_parameters REQUIRED)


include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)


if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${log_INCLUDE_DIRS}
  ${slam2d_core_INCLUDE_DIRS}
  ${mission_manager_msgs_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(slam2d_ros2
  src/conversion.cpp
  src/slam2d_params.cpp
  src/ros2_adapter.cpp
)

ament_target_dependencies(slam2d_ros2
  "rclcpp"
  "log"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "slam2d_core"
  "mission_manager_msgs"
  "EIGEN3"
  "jsoncpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  chassis_interfaces
  Boost
  OpenCV
  public_parameters
)

target_link_libraries(slam2d_ros2
  -lglog
  ${log_LIBRARIES}
  ${slam2d_core_LIBRARIES}
  ${CAIRO_LIBRARIES}
  ${PCL_LIBRARIES}
  -ljsoncpp
)
target_include_directories(slam2d_ros2 SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")

add_executable(slam2d
  src/mission_manager.cpp )

target_link_libraries(slam2d
  slam2d_ros2)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS slam2d
  DESTINATION lib/${PROJECT_NAME})

install(FILES fastrtps_config.xml
  DESTINATION ./)

install(
  DIRECTORY params/
  DESTINATION params/)
  
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()