#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>



//----------------------------------------------------------------



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




class MoveBase : public BT::AsyncActionNode
{
public:

    MoveBase(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {};
    }

    virtual BT::NodeStatus tick() override;

    virtual void halt() override
    {
        _aborted = true;
    }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    bool _aborted;
};




class JustDelay : public BT::SyncActionNode
{
public:
  explicit JustDelay(const std::string &name) : BT::SyncActionNode(name, {})
  {
    sleep(3);
  }

  BT::NodeStatus tick() override
  {
    sleep(2);
    std::cout << "Bin approached" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

