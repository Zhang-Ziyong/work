cmake_minimum_required(VERSION 3.5)
project(pnc_map)

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
find_package(cvte_hd_map REQUIRED)
# find_package(map_lib REQUIRED)
# find_package(osm_adapter_lib REQUIRED)
find_package(ament_cmake REQUIRED)
# find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(planner_utils REQUIRED)
find_package(Protobuf REQUIRED)
find_package(pose2d REQUIRED)
# find_package(log REQUIRED)
# find_package(Protobuf REQUIRED)
# find_package(GTest REQUIRED)
# find_package(glog REQUIRED)
# find_package(log REQUIRED)

# find_package(lanelet2_core REQUIRED)
# find_package(lanelet2_io REQUIRED)
# find_package(lanelet2_traffic_rules REQUIRED)
# find_package(lanelet2_routing REQUIRED)
# find_package(lanelet2_projection REQUIRED)
# find_package(TinyXML2 REQUIRED)

include_directories(
include 
${Protobuf_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED 
src/pnc_map.cpp 
)


# ament_target_dependencies(${PROJECT_NAME}_lib
# "log"
# )

ament_target_dependencies(${PROJECT_NAME}
"cvte_hd_map"
"planner_utils"
"pose2d"
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
include
)
ament_export_libraries(
pnc_map 
)
ament_export_dependencies(
cvte_hd_map
)

ament_package()
