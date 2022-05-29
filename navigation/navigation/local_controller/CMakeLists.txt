cmake_minimum_required(VERSION 3.5)
project(local_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pose2d REQUIRED)
find_package(robot_trajectory REQUIRED)
find_package(costmap_base REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(planner_utils REQUIRED)
# find_package(path_follower REQUIRED)
#find_package(osqp REQUIRED)
include_directories(
 include
 ${GLOG_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
 ${planner_utils_INCLUDE_DIRS}
 ${pose2d_INCLUDE_DIRS}
 ${robot_trajectory_INCLUDE_DIRS}
)

add_library(local_controller
    src/dwa/cost_function/map_grid_cost_function.cpp
    src/dwa/cost_function/obstacle_cost_function.cpp
    src/dwa/cost_function/oscillation_cost_function.cpp
    src/dwa/cost_function/twirling_cost_function.cpp
    src/dwa/cost_function/angle_cost_function.cpp

    src/dwa/rotation/rotation_check.cpp

    src/dwa/map_grid/map_grid.cpp
    src/dwa/trajectory/simple_trajectory_generator.cpp
    src/dwa/trajectory/simple_trajectory_scorer.cpp
    src/dwa/world_model/costmap_model.cpp
    src/dwa/dwa_planner.cpp

    src/controller_ultis/RamaxxException.cpp
    src/controller_ultis/Stopwatch.cpp
    src/controller_ultis/Line2d.cpp
    src/controller_ultis/MathHelper.cpp

    # src/pid/pid_planner.cpp

    src/lypu/lypu_planner.cpp
    src/stanley/stanley_controller.cpp 
    src/edge/edge_controller.cpp 
    
    #src/mpc/mpc_controller.cpp
    #src/mpc/mpc_osqp.cpp
)

target_link_libraries(${PROJECT_NAME}
    -lglog
    #osqp::osqp
    ${costmap_2d_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  "costmap_mediator"
  "planner_utils"
  # "path_follower"
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_include_directories(include/controller_ultis)
ament_export_libraries(local_controller)

ament_package()
