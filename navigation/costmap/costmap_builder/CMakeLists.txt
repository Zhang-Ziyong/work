cmake_minimum_required(VERSION 3.5)

project(costmap_builder)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED COMPONENTS)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED COMPONENTS)
find_package(costmap_base REQUIRED)
find_package(costmap_layer REQUIRED)


include_directories(
 include
 ${costmap_layer_INCLUDE_DIRS}
 ${costmap_base_INCLUDE_DIRS}
)

add_library(costmap_builder
  src/costmap2d_builder.cpp
  src/costmap2d_director.cpp
)

ament_target_dependencies(costmap_builder
  rclcpp
  costmap_layer
  costmap_base
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

add_executable(${PROJECT_NAME}_main src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_main
  rclcpp
  costmap_base
  costmap_layer
)

target_link_libraries(
  ${PROJECT_NAME}_main
  ${PROJECT_NAME}
  ${costmap_base_LIBRARIES} 
)

install(TARGETS ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
