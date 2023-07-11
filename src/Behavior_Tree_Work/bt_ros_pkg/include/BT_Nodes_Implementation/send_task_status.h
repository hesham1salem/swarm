#include <ros/ros.h>
#include <wakeb_swarm_msgs/task_status.h>
#include "behaviortree_cpp_v3/behavior_tree.h"

class SendTaskStatus : public BT::AsyncActionNode
{
public:
  SendTaskStatus(const std::string& name, const BT::NodeConfiguration& config) : 
    BT::AsyncActionNode(name, config)
  {
    // Create a ROS service client
    client_ = nh_.serviceClient<wakeb_swarm_msgs::task_status>("/task_status_service");
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // Create a service request object
    wakeb_swarm_msgs::task_status::Request req;
    req.task_id = 123;

    // Create a service response object
    wakeb_swarm_msgs::task_status::Response res;

    // Call the service
    if (client_.call(req, res)) {
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_ERROR("Service call failed");
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
};