cmake_minimum_required(VERSION 2.8.3)
project(task_manager)

find_package(catkin REQUIRED COMPONENTS
    rospy
    actionlib
    tf
    geometry_msgs
    actionlib_msgs
    nav_msgs
    move_base_msgs
    std_msgs
    message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  addTask.srv
  removeTask.srv
  evaluateTask.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(DEPENDS CATKIN)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
