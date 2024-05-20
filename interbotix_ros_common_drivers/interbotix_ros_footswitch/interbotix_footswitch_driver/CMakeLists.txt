cmake_minimum_required(VERSION 3.10.0)
project(interbotix_footswitch_driver)

find_package(ament_cmake REQUIRED)
find_package(interbotix_footswitch_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_subdirectory(dependencies/hidapi)

include_directories(include)

set(ROS_DEPENDENCIES
  interbotix_footswitch_msgs
  rclcpp
)

add_executable(footswitch_driver
  src/footswitch_driver.cpp
)

ament_target_dependencies(footswitch_driver ${ROS_DEPENDENCIES})

target_link_libraries(footswitch_driver hidapi::hidraw)

install(
  TARGETS
    footswitch_driver
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
