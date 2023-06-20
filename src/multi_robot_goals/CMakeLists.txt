cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_goals)

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

add_executable(multi_robot_goals multi_robot_goals.cpp)
target_link_libraries(multi_robot_goals ${catkin_LIBRARIES})

