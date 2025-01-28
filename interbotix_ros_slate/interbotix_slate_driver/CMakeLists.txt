cmake_minimum_required(VERSION 3.10.0)
project(interbotix_slate_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

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
find_package(trossen_slate REQUIRED)

include_directories(include ${trossen_slate_INCLUDE_DIRS})

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
  trossen_slate
)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  set(ARCH "x86_64")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  set(ARCH "aarch64")
else()
  message(FATAL_ERROR "Unknown System Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH})

add_library(slate_base
  src/slate_base.cpp
)

ament_target_dependencies(slate_base ${ROS_DEPENDENCIES})

target_link_libraries(slate_base
${trossen_slate_LIBRARIES}
)

add_executable(slate_base_node
  src/slate_base_node.cpp
)

target_link_libraries(slate_base_node
  slate_base
)

install(
  TARGETS slate_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    slate_base_node
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
