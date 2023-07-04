#include "multi_robot_goals.hpp"
// Create a goal object for the move_base action

double calculateDistance(const move_base_msgs::MoveBaseGoal &goal, const geometry_msgs::PoseWithCovarianceStamped &robotPosition)
{
  double dx = goal.target_pose.pose.position.x - robotPosition.pose.pose.position.x;
  double dy = goal.target_pose.pose.position.y - robotPosition.pose.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::PoseWithCovarianceStamped lastOdomMsg_tb3_0, lastOdomMsg_tb3_1, lastOdomMsg_tb3_2;
// Callback function for receiving feedback odometry  from the tb3_0/move_base
void odomCallback_tb3_0(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_0 = *msg; }
void odomCallback_tb3_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_1 = *msg; }
void odomCallback_tb3_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_2 = *msg; }

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "send_goal_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("assign_goal", myServiceCallback);

  // Create a MoveBaseClient to communicate with the move_base action server
  MoveBaseClient ac0("tb3_0/move_base", true);
  MoveBaseClient ac1("tb3_1/move_base", true);
  MoveBaseClient ac2("tb3_2/move_base", true);
  fill_goal();
  // Wait for the move_base action server to come up
  while ((!ac0.waitForServer(ros::Duration(5.0))) && (!ac1.waitForServer(ros::Duration(5.0))) && (!ac2.waitForServer(ros::Duration(5.0))))
  {
    ROS_INFO("Waiting for the move_base action server to come up...");
  }

  ros::Subscriber sub0 = nh.subscribe("tb3_0/amcl_pose", 1, odomCallback_tb3_0);
  ros::Subscriber sub1 = nh.subscribe("tb3_1/amcl_pose", 1, odomCallback_tb3_1);
  ros::Subscriber sub2 = nh.subscribe("tb3_2/amcl_pose", 1, odomCallback_tb3_2);

  // fill the priority_queue

  MoveBaseClient *selectedClient = nullptr;
  double shortestDistance = std::numeric_limits<double>::max();
  ros::spinOnce();

  while (ros::ok())
  {
    ros::Duration(0.1).sleep();

    ros::spinOnce();
    if (!goal_queue.empty())
    {
      if (ac0.getState() != actionlib::SimpleClientGoalState::ACTIVE)
      {

        auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_0);
        if (current_distance < shortestDistance)
        {

          shortestDistance = current_distance;
          selectedClient = &ac0;
        }
      }
      if (ac1.getState() != actionlib::SimpleClientGoalState::ACTIVE)
      {
        auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_1);
        if (current_distance < shortestDistance)
        {

          shortestDistance = current_distance;
          selectedClient = &ac1;
        }
      }
      if (ac2.getState() != actionlib::SimpleClientGoalState::ACTIVE)
      {
        auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_2);
        if (current_distance < shortestDistance)
        {

          shortestDistance = current_distance;
          selectedClient = &ac2;
        }
      }
      if (selectedClient != nullptr)
      {
        std::cout << "here in selectedClient" << std::endl;

        goal goal = goal_queue.top();
        selectedClient->sendGoal(goal.point);
        goal_queue.pop();
        selectedClient = nullptr;
        shortestDistance = std::numeric_limits<double>::max();
      }
    }
    std::cout << "position.x  :  " << goal_queue.top().point.target_pose.pose.position.x << std::endl;
  }
  return 0;
}
