#include <iostream>
#include "subscriber_condition_node.h"

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


/////////////////////////////implementation for second subscriber node /////////////////////////////////////////////////////

SubscriberConditionNode2::SubscriberConditionNode2(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
  , nh_2()
  , sub_2(nh_2.subscribe("topic_2", 1, &SubscriberConditionNode2::callback_2, this))
  , received_value_2(false)
{
}

BT::PortsList SubscriberConditionNode2::providedPorts()
{
  return { BT::InputPort<bool>("expected_value_2") };
}

BT::NodeStatus SubscriberConditionNode2::tick()
{
  bool expected_value_2;
  if (!getInput<bool>("expected_value_2", expected_value_2)) {
    throw BT::RuntimeError("second node missing expected_value input port");
  }

  if (received_value_2 == expected_value_2) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

void SubscriberConditionNode2::callback_2(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("Received value: [%s]", msg->data ? "true" : "false");
  received_value_2 = msg->data;
}