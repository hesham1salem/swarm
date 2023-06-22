#include <iostream>
#include "../include/move_base_with_bt.h"


SubscriberCondition::SubscriberCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
  , nh_()
  , sub_(nh_.subscribe("topic_1", 1, &SubscriberCondition::callback, this))
  , received_value_(false)
{
}

BT::PortsList SubscriberCondition::providedPorts()
{
  return { BT::InputPort<bool>("expected_value") };
}

BT::NodeStatus SubscriberCondition::tick()
{
  bool expected_value;
  if (!getInput<bool>("expected_value", expected_value)) {
    throw BT::RuntimeError("missing expected_value input port");
  }

  if (received_value_ == expected_value) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

void SubscriberCondition::callback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Received value: [%s]", msg->data ? "true" : "false");
  received_value_ = msg->data;
}



/////////////////send goal ///////////

BT::NodeStatus MoveBase::tick() {
 
  MoveBaseClient _client("tb3_0/move_base", true);
  // if no server is present, fail after 4 seconds
  if (!_client.waitForServer(ros::Duration(4.0))) {
    ROS_ERROR("Can't contact move_base server");
    return BT::NodeStatus::FAILURE;
  }


  // Reset this flag
  _aborted = false;


  // Build the message from Pose2D
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.orientation.w = 1.0;

  _client.sendGoal(goal);

  while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    ROS_ERROR("MoveBase aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveBase failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Target reached");
  return BT::NodeStatus::SUCCESS;
}
