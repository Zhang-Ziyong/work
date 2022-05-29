cmake_minimum_required(VERSION 3.5)
project(speed_decision)

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
find_package(planner_utils REQUIRED)
find_package(costmap_base REQUIRED)
find_package(data_manager REQUIRED)
find_package(costmap_mediator REQUIRED)
find_package(pnc_map REQUIRED)
# find_package(OpenCV REQUIRED)


include_directories(
include/factor/
include/marker_map/
include/ros2/
include/stop_shape/
${planner_utils_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/factor/C3/speed_decision_C3.cpp
src/factor/C5/speed_decision_C5.cpp
src/factor/speed_decision_base.cpp
src/marker_map/marker_map.cpp
src/ros2/speed_decision_adapter_ros2.cpp 
src/ros2/speed_decision_adapter.cpp 
src/stop_shape/stop_shape.cpp 
src/test/shape_test.cpp
)

target_link_libraries(${PROJECT_NAME} jsoncpp)

ament_target_dependencies(${PROJECT_NAME}
"planner_utils"
"costmap_base"
"data_manager"
"costmap_mediator"
"rclcpp"
"geometry_msgs"
"sensor_msgs"
"nav_msgs"
"log"
"pnc_map"
)

add_executable(${PROJECT_NAME}_ros2 src/ros2/main.cpp)
add_executable(${PROJECT_NAME}_test src/test/shape_test.cpp)

ament_target_dependencies(${PROJECT_NAME}_ros2
"rclcpp"
"geometry_msgs"
"sensor_msgs"
"nav_msgs"
"log"
)

ament_target_dependencies(${PROJECT_NAME}_test
"rclcpp"
"geometry_msgs"
"sensor_msgs"
"nav_msgs"
"log"
OpenCV
)

target_link_libraries(${PROJECT_NAME}_ros2 
${PROJECT_NAME}
-lglog)

target_link_libraries(${PROJECT_NAME}_test 
${PROJECT_NAME}
-lglog)

install(TARGETS ${PROJECT_NAME}_ros2
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_test
DESTINATION lib/${PROJECT_NAME}_test)

install(
    DIRECTORY 
    include/factor/
    include/marker_map/
    include/ros2/
    include/stop_shape/
    DESTINATION
    include/
  )
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

################################ TEST ###########################
#add_executable(marker_demo src/marker_demo.cpp)
#target_link_libraries(marker_demo ${PROJECT_NAME} -lglog)

ament_package()
