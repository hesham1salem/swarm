#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "GetGoals.h"
#include "ConnectRobots.h"
#include <iostream>

class Move : public BT::SyncActionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub0;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
      // Reset this flag
    bool _aborted = false;

public:
    Move(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config),
          nh(),
          sub0(nh.subscribe("tb3_0/amcl_pose", 1, &Move::odomCallback_tb3_0, this)),
          sub1(nh.subscribe("tb3_1/amcl_pose", 1, &Move::odomCallback_tb3_1, this)),
          sub2(nh.subscribe("tb3_2/amcl_pose", 1, &Move::odomCallback_tb3_2, this))
    {
    }

    double calculateDistance(const move_base_msgs::MoveBaseGoal& goal, const geometry_msgs::PoseWithCovarianceStamped& robotPosition)
    {
        double dx = goal.target_pose.pose.position.x - robotPosition.pose.pose.position.x;
        double dy = goal.target_pose.pose.position.y - robotPosition.pose.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double shortestDistance = std::numeric_limits<double>::max();
    MoveBaseClient* selectedClient = nullptr;
    geometry_msgs::PoseWithCovarianceStamped lastOdomMsg_tb3_0, lastOdomMsg_tb3_1, lastOdomMsg_tb3_2;

    void odomCallback_tb3_0(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_0 = *msg; }
    void odomCallback_tb3_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_1 = *msg; }
    void odomCallback_tb3_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_2 = *msg; }

    static BT::PortsList providedPorts()
    {
        return {};
    }


    virtual BT::NodeStatus tick() override
    {
        ros::spinOnce();
        MoveBaseClient* selectedClient = nullptr;
        EstablishConnection establish_connection("establish_connection", {});
        if (!goal_queue.empty()){
        if (establish_connection.ac0.getState() != actionlib::SimpleClientGoalState::ACTIVE)
        {
            auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_0);
            if (current_distance < shortestDistance)
            {
                shortestDistance = current_distance;
                selectedClient = &establish_connection.ac0;
            }
        }

        if (establish_connection.ac1.getState() != actionlib::SimpleClientGoalState::ACTIVE)
        {
            auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_1);
            if (current_distance < shortestDistance)
            {
                shortestDistance = current_distance;
                selectedClient = &establish_connection.ac1;
            }
        }

        if (establish_connection.ac2.getState() != actionlib::SimpleClientGoalState::ACTIVE)
        {
            auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_2);
            if (current_distance < shortestDistance)
            {
                shortestDistance = current_distance;
                selectedClient = &establish_connection.ac2;
            }
        }

       
    if (selectedClient != nullptr){
        goal goal = goal_queue.top();
        selectedClient->sendGoal(goal.point);
        goal_queue.pop();
        if (selectedClient->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("MoveBase failed");
        std::cout << selectedClient->getState().state_ <<std::endl ;
        return BT::NodeStatus::FAILURE;
        }}    } 
        
         ROS_INFO("Target reached");
        selectedClient = nullptr;   
        return BT::NodeStatus::SUCCESS;
        }};



        