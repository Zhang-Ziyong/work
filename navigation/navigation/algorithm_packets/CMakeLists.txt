cmake_minimum_required(VERSION 3.5)
project(algorithm_packets)

# Default to C99
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

find_package(ament_cmake REQUIRED)
find_package(pose2d REQUIRED)
find_package(ompl REQUIRED)
find_package(costmap_base REQUIRED)
find_package(Eigen3)
find_package(log REQUIRED)
find_package(planner_utils REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)


include_directories(
 include
 ${GLOG_INCLUDE_DIRS}
 ${pose2d_INCLUDE_DIRS}
 ${OMPL_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
 ${planner_utils_INCLUDE_DIRS}
)

add_library(algorithm_packets
    src/dijkstra/D_star.cpp
    src/dijkstra/expander.cpp
    src/dijkstra/potential_calculator.cpp
    
    src/hybrid/algorithm.cpp
    src/hybrid/bucketedqueue.cpp
    src/hybrid/collisiondetection.cpp
    src/hybrid/dubins.cpp
    src/hybrid/dynamicvoronoi.cpp
    src/hybrid/hybrid_a_star.cpp
    src/hybrid/node2d.cpp
    src/hybrid/node3d.cpp
    src/hybrid/smoother.cpp

    src/kinodynamic/kinodynamic_astar.cpp

    src/bsline/bsline_base/non_uniform_bspline.cpp
    src/bsline/bsline_optimize/bspline_optimizer.cpp
    src/bsline/path_optimizer.cpp

    src/pit_planner/pit_planner.cpp
)

#link_directories(/usr/local/lib/)
target_link_libraries(algorithm_packets ${OMPL_LIBRARIES} ${costmap_2d_LIBRARIES}  ${planner_utils_LIBRARIES} -lglog -lnlopt)

if(true)
  find_package(GTest REQUIRED)
  find_package(costmap_layer REQUIRED)
  find_package(costmap_builder REQUIRED)
  find_package(costmap_base REQUIRED)
  find_package(costmap_mediator REQUIRED)
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}
  ${costmap_layer_INCLUDE_DIRS}
  ${costmap_builder_INCLUDE_DIRS}
  ${costmap_mediator_INCLUDE_DIRS})

  add_executable(algorithm_packets_test test/test_D_star_planner.cpp)
  target_link_libraries(algorithm_packets_test -lgtest -lglog -lpthread 
  algorithm_packets 
  ${costmap_layer_LIBRARIES} 
  ${costmap_base_LIBRARIES} 
  ${costmap_mediator_LIBRARIES} 
  ${costmap_builder_LIBRARIES})
  
  install(TARGETS algorithm_packets_test
  DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  algorithm_packets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(${planner_utils_INCLUDE_DIRS})
ament_export_include_directories(include)
ament_export_libraries(${planner_utils_LIBRARIES})
ament_export_dependencies(pose2d)
ament_export_libraries(algorithm_packets)
ament_export_libraries(${OMPL_LIBRARIES})

ament_package()