#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "common.h"

class GetGoals : public BT::SyncActionNode
{
public:
  GetGoals(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {} 

  static BT::PortsList providedPorts()
  {  return {}; }

  BT::NodeStatus tick() override
  {
    other_goal.target_pose.header.frame_id = "map";
    other_goal.target_pose.pose.position.x = 4;
    other_goal.target_pose.pose.position.y = 4;
    other_goal.target_pose.pose.orientation.w = 1.0;

    other_goal1.target_pose.header.frame_id = "map";
    other_goal1.target_pose.pose.position.x = 5;
    other_goal1.target_pose.pose.position.y = 4;
    other_goal1.target_pose.pose.orientation.w = 1.0;

    goal_queue.emplace(goal{other_goal, 1});
    goal_queue.emplace(goal{other_goal1, 2});

    return BT::NodeStatus::SUCCESS;
  }

//   move_base_msgs::MoveBaseGoal goal0,goal1,goal2,other_goal,other_goal1;
//   struct goal
//   {
//     move_base_msgs::MoveBaseGoal point;
//     int priority;

//     bool operator<(const goal &other) const
//     {
//         return this->priority < other.priority;
//     }
//   };
// //  private :
//   std::priority_queue<goal> goal_queue;
};