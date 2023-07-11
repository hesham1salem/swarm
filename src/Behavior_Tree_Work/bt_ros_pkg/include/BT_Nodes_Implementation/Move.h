#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "GetGoals.h"
#include "ConnectRobots.h"
#include <iostream>
#include "wakeb_swarm_msgs/task_status.h"


class Move : public BT::SyncActionNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub0;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::ServiceClient task_status_client;
    ros::ServiceServer task_status_server;

      // Reset this flag
    bool _aborted = false;
    EstablishConnection establish_connection;
    std::string status;


public:
    Move(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config),
        nh(),
        sub0(nh.subscribe("tb3_0/amcl_pose", 1, &Move::odomCallback_tb3_0, this)),
        sub1(nh.subscribe("tb3_1/amcl_pose", 1, &Move::odomCallback_tb3_1, this)),
        sub2(nh.subscribe("tb3_2/amcl_pose", 1, &Move::odomCallback_tb3_2, this)),
        establish_connection("establish_connection", {})
    {
        task_status_client = nh.serviceClient<wakeb_swarm_msgs::task_status>("/test_srv");
        // task_status_server = nh.advertiseService("task_status_service", &send_status_cb,this);

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
    void odomCallback_tb3_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_1 = *msg;  }
    void odomCallback_tb3_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) { lastOdomMsg_tb3_2 = *msg;  }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    // bool send_status_cb(wakeb_swarm_msgs::task_status::Request  &req,
    //                     wakeb_swarm_msgs::task_status::Response &res)
    // {
    //     res.status ="DONE";
    //     status = "DONE";
    //     std::cout <<"_________"<< res.status<<std::endl; 

    //     return true;
    // }


    virtual BT::NodeStatus tick() override
    {
        ros::spinOnce();
        MoveBaseClient* selectedClient = nullptr;

        // while(ros::ok() ){
        ros::Duration(1).sleep();
        ros::spinOnce();
        if (!goal_queue.empty()){
            if (establish_connection.ac0.getState() != actionlib::SimpleClientGoalState::ACTIVE)
            {
                std::cout<<"robot 1 status "<< establish_connection.ac0.getState().toString()<<std::endl; 
               // ROS_INFO("robot 1 status: %d", static_cast<int>(establish_connection.ac0.getState().state_));
                ROS_INFO_STREAM("Robot 1 status: " << establish_connection.ac1.getState().getText());
                auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_0);
                if (current_distance < shortestDistance)
                {
                    shortestDistance = current_distance;
                    selectedClient = &establish_connection.ac0;
                    //std::cout<< "in tb3_0 "<<std::endl;
                }
            }
            else std::cout<<"robot 1 active "<<std::endl ; 

            if (establish_connection.ac1.getState() != actionlib::SimpleClientGoalState::ACTIVE)
            {
                std::cout<<"robot 2 status "<< establish_connection.ac1.getState().toString()<<std::endl; 
                ROS_INFO("robot 2 status: %d", static_cast<int>(establish_connection.ac1.getState().state_));
                // ROS_INFO_STREAM("Robot 2 status: " << establish_connection.ac1.getState().getText());
               if (establish_connection.ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                wakeb_swarm_msgs::task_status::Request req;
                req.task_id = goal_queue.top().task_id;
                req.status = "done" ; 

                // Create a service response object
                wakeb_swarm_msgs::task_status::Response res;

                // // Call the service
                // task_status_client.call(req, res);
                // ROS_INFO("Service call succeeded. Status____________: ");

                if (task_status_client.call(req, res)) {
                std::cout<< "_______"<<res.success <<std::endl;
                // ROS_INFO("Service call succeeded. Status: %s", res.status.c_str());
               // return BT::NodeStatus::SUCCESS;
                } else {
                ROS_ERROR("Service call failed");
                //return BT::NodeStatus::FAILURE;
                 }                  
                    

                }
               
                auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_1);
                if (current_distance < shortestDistance)
                {
                    shortestDistance = current_distance;
                    selectedClient = &establish_connection.ac1;
                }
            }
           
            else {
                std::cout<<"robot 2 active "<<std::endl ;
                } 

            if (establish_connection.ac2.getState() != actionlib::SimpleClientGoalState::ACTIVE)
            {
                std::cout<<"robot 3 status "<< establish_connection.ac2.getState().toString()<<std::endl; 
                // ROS_INFO("robot 3 status: %d", static_cast<int>(establish_connection.ac2.getState().state_));
                ROS_INFO_STREAM("Robot 3 status: " << establish_connection.ac1.getState().getText());
                auto current_distance = calculateDistance(goal_queue.top().point, lastOdomMsg_tb3_2);
                if (current_distance < shortestDistance)
                {
                    shortestDistance = current_distance;
                    selectedClient = &establish_connection.ac2;
                }
            }
            else std::cout<<"robot 3 active "<<std::endl ; 

        
        if (selectedClient != nullptr){
            goal goal = goal_queue.top();
            int priority = goal_queue.top().priority;
            std::cout<< "y of top point in move " <<goal_queue.top().point.target_pose.pose.position.y<<std::endl;
            std::cout<< "priority of the goal is  " <<priority<<std::endl;

            selectedClient->sendGoal(goal.point);
            
            // if (selectedClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            // ROS_INFO("Goal Reached");

            // }    
        
            goal_queue.pop();
            std::cout<< "y after pop" <<goal_queue.top().point.target_pose.pose.position.y<<std::endl;
            std::cout << "supposed that pop is done" <<std::endl ; 
            selectedClient = nullptr;   
            shortestDistance = std::numeric_limits<double>::max();    
            return BT::NodeStatus::SUCCESS;

        }    
    }         
        
        return BT::NodeStatus::SUCCESS;
        }};



        