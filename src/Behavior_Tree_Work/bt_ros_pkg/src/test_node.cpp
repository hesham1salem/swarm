#include <ros/ros.h>
#include <wakeb_swarm_msgs/task_status.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "service_client");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // Create a service client object
  ros::ServiceClient client = nh.serviceClient<wakeb_swarm_msgs::task_status>("/task_status_service");

  // Create a service request object
  wakeb_swarm_msgs::task_status::Request req;
  req.task_id = 123;

  // Create a service response object
  wakeb_swarm_msgs::task_status::Response res;

  // Call the service
  if (client.call(req, res)) {
    // ROS_INFO("Service call succeeded. Status: %s", res.status.c_str());
  } else {
    ROS_ERROR("Service call failed");
    return 1;
  }

  // Shutdown the ROS node
  ros::shutdown();

  return 0;
}