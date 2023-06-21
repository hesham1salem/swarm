cmake_minimum_required(VERSION 3.5)
project(bt_ros_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  behaviortree_cpp_v3
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(bt_node src/bt_node.cpp )
target_link_libraries(bt_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})



add_executable(subscriber_condition_node src/subscriber_condition_main.cpp src/subscriber_condition_node.cpp)
target_link_libraries(subscriber_condition_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})