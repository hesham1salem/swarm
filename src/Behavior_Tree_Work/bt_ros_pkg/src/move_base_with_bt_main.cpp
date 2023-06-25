#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "../include/move_base_with_bt.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_with_bt");
  ros::NodeHandle nh;
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "/home/wafaa/swarm/src/Behavior_Tree_Work/bt_ros_pkg/move_base_bt.xml");

  // Create a Behavior Tree node for the subscriber condition
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SubscriberCondition>("SubscriberCondition");
  factory.registerNodeType<MoveBase>("MoveBase");
  factory.registerNodeType<JustDelay>("JustDelay");

  auto tree = factory.createTreeFromFile(xml_filename);
  BT::PublisherZMQ publisher_zmq(tree);
  // Run the Behavior Tree
  while (ros::ok()) {
    tree.tickRoot();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}