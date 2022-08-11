project(accr_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
    std_msgs
#    nav_msgs
    geometry_msgs
#    actionlib
    message_generation
    )

##
#add_action_files(
#    DIRECTORY action
#    FILES
#    LocalPlanner.action
#    GlobalPlanner.action
#    ArmorDetection.action
#)

add_message_files(
    DIRECTORY msg
    FILES
    Chassis.msg
    Bucket.msg
    API.msg
)

#add_service_files(
#  DIRECTORY srv
#  FILES
#  GimbalMode.srv
#  FricWhl.srv
#  ShootCmd.srv
#)

#add_service_files(
#  DIRECTORY srv/sim
#  FILES
#  SimCheckBullet.srv
#  SimHasLoS.srv
#  SimReloadCmd.srv
#  SimShootCmd.srv
#)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
#    nav_msgs
)

catkin_package()
