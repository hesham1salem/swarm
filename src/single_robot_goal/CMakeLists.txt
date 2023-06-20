cmake_minimum_required(VERSION 3.0.2)
project(single_robot_goal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  move_base_msgs
  actionlib
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(single_robot_goal single_robot_goal.cpp)
target_link_libraries(single_robot_goal ${catkin_LIBRARIES})

