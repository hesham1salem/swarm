#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "../include/BT_Nodes_Implementation/Move.h"
// #include "../include/BT_Nodes_Implementation/GetGoals.h"
// #include "../include/BT_Nodes_Implementation/ConnectRobots.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_goals_bt");
  ros::NodeHandle nh;
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "/home/wafaa/swarm/src/Behavior_Tree_Work/bt_ros_pkg/multi_goals_bt.xml");

  // Create a Behavior Tree node for the subscriber condition
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<EstablishConnection>("EstablishConnection");
  factory.registerNodeType<GetGoals>("GetGoals");
  factory.registerNodeType<AskForHelp>("AskForHelp");
  factory.registerNodeType<Move>("Move");

  auto tree = factory.createTreeFromFile(xml_filename);
  BT::PublisherZMQ publisher_zmq(tree);
  // Run the Behavior Tree
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (ros::ok()  && status == BT::NodeStatus::RUNNING) {
    tree.tickRoot();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}