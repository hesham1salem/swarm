#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include "../include/subscriber_condition_node.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber_condition_node");
  ros::NodeHandle nh;
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "/home/wafaa/swarm/src/Behavior_Tree_Work/bt_ros_pkg/condition_tree.xml");

  // Create a Behavior Tree node for the subscriber condition
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SubscriberCondition>("SubscriberCondition");
  factory.registerNodeType<SubscriberConditionNode2>("SubscriberConditionNode2");
  factory.registerNodeType<ApproachBin>("ApproachBin");

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