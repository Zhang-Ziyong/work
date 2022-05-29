cmake_minimum_required(VERSION 3.5)
project(cvte_hd_map)

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
find_package(log REQUIRED)
find_package(Protobuf REQUIRED)
find_package(GTest REQUIRED)
find_package(common_cvte_osm REQUIRED)
# find_package(glog REQUIRED)
# find_package(log REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(
common
include 
proto 
adapter
${Protobuf_INCLUDE_DIRS}
)




add_library(${PROJECT_NAME} SHARED 
common/polygon2d.cc 
common/math_utils.cc 
common/line_segment2d.cc 
common/box2d.cc 
common/aabox2d.cc 
common/hdmap_common.cc 
src/hdmap_impl.cpp 
src/hd_map.cpp 
adapter/osm_adapter/osm_adapter.cpp 
proto/map_c5_area.pb.cc 
proto/map_clear_area.pb.cc  
proto/map_crosswalk.pb.cc 
proto/map_geometry.pb.cc 
proto/map_id.pb.cc 
proto/map_junction.pb.cc  
proto/map_lane.pb.cc 
proto/map_overlap.pb.cc  
proto/map_parking_space.pb.cc  
proto/map_pnc_junction.pb.cc 
proto/map_road.pb.cc 
proto/map_rsu.pb.cc 
proto/map_signal.pb.cc  
proto/map_speed_bump.pb.cc  
proto/map_speed_control.pb.cc  
proto/map_stop_sign.pb.cc 
proto/map_yield_sign.pb.cc 
proto/map.pb.cc 
)

# add_library(osm_adapter_lib 
# adapter/osm_adapter/osm_adapter.cpp 
# )

# add_library(map_lib 
# proto/map_c5_area.pb.cc 
# proto/map_clear_area.pb.cc  
# proto/map_crosswalk.pb.cc 
# proto/map_geometry.pb.cc 
# proto/map_id.pb.cc 
# proto/map_junction.pb.cc  
# proto/map_lane.pb.cc 
# proto/map_overlap.pb.cc  
# proto/map_parking_space.pb.cc  
# proto/map_pnc_junction.pb.cc 
# proto/map_road.pb.cc 
# proto/map_rsu.pb.cc 
# proto/map_signal.pb.cc  
# proto/map_speed_bump.pb.cc  
# proto/map_speed_control.pb.cc  
# proto/map_stop_sign.pb.cc 
# proto/map_yield_sign.pb.cc 
# proto/map.pb.cc 
# )


# ament_target_dependencies(${PROJECT_NAME}_lib
# "log"
# )

ament_target_dependencies(${PROJECT_NAME} 
common_cvte_osm
lanelet2_core
lanelet2_io
lanelet2_traffic_rules
lanelet2_routing
lanelet2_projection
TinyXML2
log 
)

target_link_libraries(${PROJECT_NAME}
                      -lglog
                      ${Protobuf_LIBRARIES})
# target_link_libraries(map_lib 
#                       -lglog
#                       ${Protobuf_LIBRARIES})
# target_link_libraries(osm_adapter_lib
#                       -lglog
#                       map_lib 
#                       )

add_executable(${PROJECT_NAME}_line_segment2d_test common/line_segment2d_test.cc)
add_executable(${PROJECT_NAME}_math_utils_test common/math_utils_test.cc)
add_executable(${PROJECT_NAME}_polygon_test common/polygon2d_test.cc)
add_executable(${PROJECT_NAME}_box2d_test common/box2d_test.cc)
add_executable(${PROJECT_NAME}_aabox2d_test common/aabox2d_test.cc)
add_executable(${PROJECT_NAME}_aaboxkdtree_test common/aaboxkdtree2d_test.cc)
add_executable(${PROJECT_NAME}_hdmap_common_test common/hdmap_common_test.cc)
add_executable(${PROJECT_NAME}_osm_adapter_test adapter/osm_adapter/osm_adapter_test.cpp)

target_link_libraries(${PROJECT_NAME}_line_segment2d_test
${PROJECT_NAME}
-lglog
-lgtest 
-lpthread
)
target_link_libraries(${PROJECT_NAME}_math_utils_test
${PROJECT_NAME}
-lglog
-lgtest 
-lpthread
)
target_link_libraries(${PROJECT_NAME}_polygon_test
${PROJECT_NAME}
-lglog
-lgtest 
-lpthread
)
target_link_libraries(${PROJECT_NAME}_box2d_test
${PROJECT_NAME}
-lglog
-lgtest 
-lpthread
)
target_link_libraries(${PROJECT_NAME}_aabox2d_test
${PROJECT_NAME}
-lglog
-lgtest 
-lpthread
)
target_link_libraries(${PROJECT_NAME}_aaboxkdtree_test
${PROJECT_NAME}
-lglog
-lgtest 
-lpthread
)
target_link_libraries(${PROJECT_NAME}_hdmap_common_test
${PROJECT_NAME}
-lglog
-lgtest 
-lpthread
)
target_link_libraries(${PROJECT_NAME}_osm_adapter_test
${PROJECT_NAME}
# osm_adapter_lib
-lglog
-lgtest 
-lpthread
)

install(TARGETS ${PROJECT_NAME}_line_segment2d_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_math_utils_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_polygon_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_box2d_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_aabox2d_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_aaboxkdtree_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_hdmap_common_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_osm_adapter_test
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY common/
  DESTINATION common/
)

install(
  DIRECTORY proto/
  DESTINATION proto/
)

install(
  DIRECTORY adapter/
  DESTINATION adapter/
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# install(TARGETS map_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(TARGETS osm_adapter_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

ament_export_include_directories(
common
include  
proto 
adapter
${Protobuf_INCLUDE_DIRS}
)
ament_export_libraries(
${PROJECT_NAME}
# map_lib 
# osm_adapter_lib
)
ament_export_dependencies(
lanelet2_core
lanelet2_io
lanelet2_traffic_rules
lanelet2_routing
lanelet2_projection
TinyXML2
# map_lib 
# osm_adapter_lib
Protobuf
)

ament_package()
