#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
class EstablishConnection : public BT::SyncActionNode
{
public:
  EstablishConnection(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),
      ac0("tb3_0/move_base", true),
      ac1("tb3_1/move_base", true),
      ac2("tb3_2/move_base", true)
  {} 
MoveBaseClient ac0 , ac1, ac2  ; 

  static BT::PortsList providedPorts()
  {
    return {};
  }
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  BT::NodeStatus tick() override
  {
    // Wait for the move_base action server to come up
 

    if  ((!ac0.waitForServer(ros::Duration(5.0))) && (!ac1.waitForServer(ros::Duration(5.0))) && (!ac2.waitForServer(ros::Duration(5.0))))
    {
      ROS_INFO("Waiting for the move_base action server to come up...");
      return BT::NodeStatus::FAILURE;
    }
    else {
          ROS_INFO("Connection is done...");
          return BT::NodeStatus::SUCCESS;}}

};


class AskForHelp : public BT::SyncActionNode
{
public:
  AskForHelp(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Robots Are Not Connected" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
};