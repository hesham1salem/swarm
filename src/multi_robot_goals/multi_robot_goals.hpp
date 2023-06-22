#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal goal0,
    goal1,
    goal2,
    other_goal, other_goal1;

struct goal
{
    move_base_msgs::MoveBaseGoal point;
    int priority;
    bool operator<(const goal &other) const
    {
        return this->priority < other.priority;
    }
};

std::priority_queue<goal> goal_queue;

// Set the frame ID of the goal
void fill_goal()
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
}