cmake_minimum_required(VERSION 3.5)
project(interbotix_xs_sdk)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xseries_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(backward_ros REQUIRED)
include_directories(include)


set (ROS_DEPENDENCIES 
ament_index_cpp
rclcpp
rclcpp_action
control_msgs
dynamixel_workbench_toolbox
sensor_msgs
std_msgs
trajectory_msgs
urdf
xseries_msgs
)
###########
## Build ##
###########

## Declare a C++ executable
## Specify libraries to link a library or executable target against
add_executable(xs_sdk src/xs_sdk.cpp src/xs_sdk_obj.cpp)
ament_target_dependencies(xs_sdk ${ROS_DEPENDENCIES})
target_link_libraries(xs_sdk yaml-cpp)


#############
## Install ##
#############
install(TARGETS
  xs_sdk
  DESTINATION lib/${PROJECT_NAME})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/xs_sdk_sim
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
