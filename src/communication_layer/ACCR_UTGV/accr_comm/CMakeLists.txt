cmake_minimum_required(VERSION 2.8.3)
project(accr_comm)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  tf
  nav_msgs
  accr_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    accr_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( accr_comm src/accr_comm.cpp)

target_link_libraries( accr_comm
  ${catkin_LIBRARIES}
)

