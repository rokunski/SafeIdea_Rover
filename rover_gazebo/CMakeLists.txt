cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(rover_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazebo_ros
  controller_manager
  joint_state_controller
  robot_state_publisher
  diff_drive_controller
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
    rospy
    gazebo_ros
    controller_manager
    joint_state_controller
    robot_state_publisher
    diff_drive_controller
)
