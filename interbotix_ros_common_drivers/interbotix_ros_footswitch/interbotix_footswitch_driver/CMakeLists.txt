cmake_minimum_required(VERSION 3.12.0)
project(interbotix_footswitch_driver)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(interbotix_footswitch_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(HIDAPI REQUIRED)
find_package(LIBUSB 1 REQUIRED)

include_directories(
  include
  ${HIDAPI_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
)

set(ROS_DEPENDENCIES
  interbotix_footswitch_msgs
  rclcpp
  rclcpp_components
)

add_library(footswitch_driver
  src/footswitch_driver.cpp
)

ament_target_dependencies(footswitch_driver ${ROS_DEPENDENCIES})

target_link_libraries(footswitch_driver
  ${HIDAPI_LIBRARIES}
  ${LibUSB_LIBRARIES}
)

add_executable(footswitch_driver_node
  src/footswitch_driver_node.cpp
)

target_link_libraries(footswitch_driver_node
  footswitch_driver
)

install(
  TARGETS footswitch_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    footswitch_driver_node
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
