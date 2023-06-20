#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Create a goal object for the move_base action
move_base_msgs::MoveBaseGoal goal0;
move_base_msgs::MoveBaseGoal goal1;
move_base_msgs::MoveBaseGoal goal2;
move_base_msgs::MoveBaseGoal other_goal, other_goal1;

struct goal
{
  move_base_msgs::MoveBaseGoal point;
  int priority;
  bool operator<(const goal &other) const
  {
    return this->priority < other.priority;
  }
};

std::priority_queue<goal>
    goal_queue;

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "send_goal_node");
  ros::NodeHandle nh;

  // Create a MoveBaseClient to communicate with the move_base action server
  MoveBaseClient ac0("tb3_0/move_base", true);
  MoveBaseClient ac1("tb3_1/move_base", true);
  MoveBaseClient ac2("tb3_2/move_base", true);

  // Set the frame ID of the goal
  goal0.target_pose.header.frame_id = "map";
  goal0.target_pose.pose.position.x = 1;
  goal0.target_pose.pose.position.y = 2;
  goal0.target_pose.pose.orientation.w = 1.0;

  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.pose.position.x = 2;
  goal1.target_pose.pose.position.y = 2;
  goal1.target_pose.pose.orientation.w = 1.0;

  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.pose.position.x = 3;
  goal2.target_pose.pose.position.y = 3;
  goal2.target_pose.pose.orientation.w = 1.0;

  other_goal.target_pose.header.frame_id = "map";
  other_goal.target_pose.pose.position.x = 4;
  other_goal.target_pose.pose.position.y = 4;
  other_goal.target_pose.pose.orientation.w = 1.0;

  other_goal1.target_pose.header.frame_id = "map";
  other_goal1.target_pose.pose.position.x = 5;
  other_goal1.target_pose.pose.position.y = 4;
  other_goal1.target_pose.pose.orientation.w = 1.0;
  // Wait for the move_base action server to come up
  while ((!ac0.waitForServer(ros::Duration(5.0))) && (!ac1.waitForServer(ros::Duration(5.0))) && (!ac2.waitForServer(ros::Duration(5.0))))
  {
    ROS_INFO("Waiting for the move_base action server to come up...");
  }

  // Send the goal to the move_base action server
  ac0.sendGoal(goal0);
  ac1.sendGoal(goal1);
  ac2.sendGoal(goal2);

  // Wait for the result'

  // ac0.waitForResult();

  // ac1.waitForResult();
  // ac2.waitForResult();

  // Check if the goal was reached

  // fill the priority_queue
  goal_queue.emplace(goal{other_goal, 1});
  goal_queue.emplace(goal{other_goal1, 2});

  while (!goal_queue.empty())
  {

    if (ac0.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("tb3_0 Goal reached!");
      ROS_INFO("send another goal ");
      ac0.sendGoal(goal_queue.top().point);

      goal_queue.pop();
    }
    if (ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("tb3_1 Goal reached!");
      ROS_INFO("send another goal ");
      ac1.sendGoal(goal_queue.top().point);
      goal_queue.pop();
    }
    else if (ac2.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("tb3_2 Goal reached!");
      ROS_INFO("send another goal ");
      ac2.sendGoal(goal_queue.top().point);
      goal_queue.pop();
    }
  }

  return 0;
}
