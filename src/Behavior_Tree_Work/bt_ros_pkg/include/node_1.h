#include<iostream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>


// approach bin 
BT::NodeStatus binClose()
{
  std::cout << "Bin not close" << std::endl;
  return BT::NodeStatus::FAILURE;
}

class ApproachBin : public BT::SyncActionNode
{
public:
  explicit ApproachBin(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    sleep(3);
    std::cout << "Bin approached" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};



//ball placing 
BT::NodeStatus ballPlaced()
{
  std::cout << "Ball not placed" << std::endl;
  return BT::NodeStatus::FAILURE;
}

class PlaceBall : public BT::SyncActionNode
{
public:
  explicit PlaceBall(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    sleep(3);
    std::cout << "Ball placed" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
