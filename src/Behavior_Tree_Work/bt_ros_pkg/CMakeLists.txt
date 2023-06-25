cmake_minimum_required(VERSION 3.5)
project(bt_ros_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  behaviortree_cpp_v3
  actionlib
  actionlib_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(subscriber_condition_node src/subscriber_condition_main.cpp src/subscriber_condition_node.cpp)
target_link_libraries(subscriber_condition_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move_base_with_bt src/move_base_with_bt.cpp src/move_base_with_bt_main.cpp)
target_link_libraries(move_base_with_bt ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(multi_goals_bt src/multi_goals_bt.cpp src/multi_goals_bt_main.cpp )
target_link_libraries(multi_goals_bt ${catkin_LIBRARIES} ${Boost_LIBRARIES})