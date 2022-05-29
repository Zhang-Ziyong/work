cmake_minimum_required(VERSION 3.5)

project(logic_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(pose2d REQUIRED)
find_package(planner_utils REQUIRED)
find_package(costmap_base REQUIRED)
find_package(global_planner REQUIRED)
find_package(local_planner REQUIRED)
find_package(path_follower REQUIRED)
find_package(local_controller REQUIRED)
find_package(robot_trajectory REQUIRED)
find_package(speed_decision REQUIRED)


include_directories(
 include
 ${GLOG_INCLUDE_DIRS}
 ${pose2d_INCLUDE_DIRS}
 ${planner_utils_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
 ${global_planner_INCLUDE_DIRS}
 ${local_planner_INCLUDE_DIRS}
 ${path_follower_INCLUDE_DIRS}
 ${local_controller_INCLUDE_DIRS}
 ${robot_trajectory_INCLUDE_DIRS}
 ${speed_decision_INCLUDE_DIRS}
)

add_library(logic_controller
 src/logic_controller.cpp
 src/slope_mark.cpp
)

ament_target_dependencies(logic_controller
"speed_decision"
)

target_link_libraries(${PROJECT_NAME}
  -lglog
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

