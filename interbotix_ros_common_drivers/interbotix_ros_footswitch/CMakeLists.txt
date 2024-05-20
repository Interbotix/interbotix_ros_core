cmake_minimum_required(VERSION 3.10.0)
project(interbotix_ros_footswitch)

find_package(ament_cmake)

add_subdirectory(dependencies/hidapi)

add_executable(footswitch_driver
  src/footswitch_driver.cpp
)

target_link_libraries(footswitch_driver PRIVATE hidapi::hidraw)

install(
  TARGETS
    footswitch_driver
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
