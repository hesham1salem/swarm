#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "send_goal_node");
    ros::NodeHandle nh;

    // Create a MoveBaseClient to communicate with the move_base action server
    MoveBaseClient ac("tb3_0/move_base", true);

    // Wait for the move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }

    // Create a goal object for the move_base action
    move_base_msgs::MoveBaseGoal goal;

    // Set the frame ID of the goal
    goal.target_pose.header.frame_id = "map";

    // Set the position and orientation of the goal
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 2.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal to the move_base action server
    ac.sendGoal(goal);

    // Wait for the result
    ac.waitForResult();

    // Check if the goal was reached
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal reached!");
    }
    else
    {
        ROS_ERROR("Failed to reach the goal!");
    }

    return 0;
}

