#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>

class SubscriberCondition : public BT::ConditionNode
{
public:
  SubscriberCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  virtual BT::NodeStatus tick() override;

private:
  void callback(const std_msgs::Bool::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  bool received_value_;
};


class SubscriberConditionNode2 : public BT::ConditionNode
{
public:
  SubscriberConditionNode2(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  virtual BT::NodeStatus tick() override;

private:
  void callback_2(const std_msgs::Bool::ConstPtr& msg);

  ros::NodeHandle nh_2;
  ros::Subscriber sub_2;
  bool received_value_2;
};


class ApproachBin : public BT::SyncActionNode
{
public:
  explicit ApproachBin(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    sleep(2);
    std::cout << "Bin approached" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

