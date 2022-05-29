cmake_minimum_required(VERSION 3.5)

project(costmap_base)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
 include
 ${EIGEN3_INCLUDE_DIR}
)

add_library(costmap_base
  src/layer.cpp
  src/costmap_2d.cpp
  src/costmap_utils.cpp
  src/layered_costmap.cpp
  src/footprint_utils.cpp
  src/world_map_data.cpp
  src/costmap_cloud.cpp
  src/debug_log.cpp
  src/voxel_grid.cpp
  src/probability_voxel_grid.cpp
  src/probability_table.cpp
)

ament_target_dependencies(costmap_base
  yaml_cpp_vendor
)

target_link_libraries(${PROJECT_NAME}
${Boost_LIBRARIES}
${SDL_IMAGE_LIBRARIES}
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

if(false)
  find_package(GTest REQUIRED)
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  add_executable(test_layered_costmap test/test_layered_costmap.cpp)
  target_link_libraries(test_layered_costmap ${PROJECT_NAME} -lgtest -lpthread)

  add_executable(test_costmap2d test/test_costmap_2d.cpp)
  target_link_libraries(test_costmap2d ${PROJECT_NAME} -lgtest -lpthread)

  add_executable(test_params test/test_params.cpp)
  target_link_libraries(test_params ${PROJECT_NAME} -lgtest -lpthread)

  add_executable(test_world_map_data test/test_world_map_data.cpp)
  target_link_libraries(test_world_map_data ${PROJECT_NAME} -lgtest -lpthread)

#  有编译警告, 需要时再编译
#  add_executable(test_voxel_grid test/test_voxel_grid.cpp)
#  target_link_libraries(test_voxel_grid ${PROJECT_NAME} -lgtest -lpthread)

  install(TARGETS test_costmap2d test_layered_costmap test_params test_world_map_data #test_voxel_grid
  DESTINATION lib/${PROJECT_NAME})

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(yaml_cpp_vendor)

ament_package()

