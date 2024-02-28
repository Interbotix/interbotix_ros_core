cmake_minimum_required(VERSION 3.10.0)
project(interbotix_slate_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

set(serial_driver "chassis_driver")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interbotix_slate_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(ROS_DEPENDENCIES
  geometry_msgs
  interbotix_slate_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(slate_base_node
  src/slate_base_node.cpp
  src/base_driver.cpp
)

ament_target_dependencies(slate_base_node ${ROS_DEPENDENCIES})

target_link_libraries(slate_base_node
  ${serial_driver}
)

add_executable(slate_base
  src/slate_base.cpp
)

target_link_libraries(slate_base
  slate_base_node
)

# add_custom_command(
# 	OUTPUT copy_lib
# 	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/lib/* ${CATKIN_DEVEL_PREFIX}/lib
# )
# add_custom_target(COPY_CHASSIS_LIB ALL DEPENDS copy_lib slate_base)

install(
  TARGETS slate_base_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    slate_base
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

install(
  FILES
    lib/lib${serial_driver}.so
  DESTINATION
    lib
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
