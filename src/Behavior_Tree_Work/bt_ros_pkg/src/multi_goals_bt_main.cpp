#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "../include/BT_Nodes_Implementation/Move.h"
#include "wakeb_swarm_msgs/goal_task.h"




bool myServiceCallback(wakeb_swarm_msgs::goal_task::Request &req,
                       wakeb_swarm_msgs::goal_task::Response &res)
{
    goal new_goal;
    new_goal.point.target_pose = req.pose;
    new_goal.point.target_pose.header.frame_id ="map";
    new_goal.priority = req.priority;
    goal_queue.push(new_goal);
    res.success = true; // Populate the response field
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_goals_bt");
  ros::NodeHandle nh;
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "/home/wafaa/swarm/src/Behavior_Tree_Work/bt_ros_pkg/multi_goals_bt.xml");
  ros::ServiceServer service = nh.advertiseService("assign_goal", myServiceCallback);

  // Create a Behavior Tree node for the subscriber condition
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<EstablishConnection>("EstablishConnection");
  factory.registerNodeType<AskForHelp>("AskForHelp");
  // factory.registerNodeType<GetGoals>("GetGoals");
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