cmake_minimum_required(VERSION 3.5)
project(global_planner)

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
find_package(algorithm_packets REQUIRED)
find_package(costmap_base REQUIRED)

include_directories(
 include
 ${GLOG_INCLUDE_DIRS}
 ${algorithm_packets_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
)

add_library(global_planner
    src/global_planner_factory.cpp
    src/global_planner.cpp
)
target_link_libraries(global_planner 
    -lglog 
    ${algorithm_packets_LIBRARIES}
    ${costmap_2d_LIBRARIES}
)
if(false)
  find_package(GTest REQUIRED)

  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  add_executable(global_planner_test test/test_global_planner_factory.cpp)
  target_link_libraries(global_planner_test -lgtest -lglog -lpthread 
  global_planner)

  install(TARGETS global_planner_test
  DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  global_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(global_planner)
ament_export_dependencies(algorithm_packets)

ament_package()
