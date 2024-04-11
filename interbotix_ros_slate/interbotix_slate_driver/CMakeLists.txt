cmake_minimum_required(VERSION 3.10.0)
project(interbotix_slate_driver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -g -std=c++11 -O0 ")

set(serial_driver "chassis_driver")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interbotix_slate_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_srvs
  tf
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    interbotix_slate_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(slate_base
  src/slate_base.cpp
  src/base_driver.cpp
)
add_dependencies(slate_base ${slate_base_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(slate_base
  ${serial_driver}
  ${catkin_LIBRARIES}
)

add_executable(slate_base_node
  src/slate_base_node.cpp
)
add_dependencies(slate_base_node ${slate_base_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(slate_base_node
  slate_base
)

add_custom_command(
	OUTPUT copy_lib
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/lib/* ${CATKIN_DEVEL_PREFIX}/lib
)
add_custom_target(COPY_CHASSIS_LIB ALL DEPENDS copy_lib slate_base_node)

install(
  TARGETS
    slate_base_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    lib/lib${serial_driver}.so
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)
