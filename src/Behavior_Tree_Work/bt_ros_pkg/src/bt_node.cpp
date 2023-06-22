#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "../include/node_1.h"


int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "bt_node");
    ros::NodeHandle nh;
    std::string xml_filename;
    nh.param<std::string>("file", xml_filename, "/home/wafaa/swarm/src/Behavior_Tree_Work/bt_ros_pkg/demo_tree.xml");
    
    
    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BinClose", std::bind(binClose));

    factory.registerNodeType<ApproachBin>("ApproachBin");

    factory.registerSimpleCondition("BallPlaced", std::bind(ballPlaced));
    factory.registerNodeType<ApproachBin>("PlaceBall");
    // Create the Behavior Tree
    auto tree = factory.createTreeFromFile(xml_filename);
    BT::PublisherZMQ publisher_zmq(tree);
    sleep(3);

    // Start ticking the tree
    while (ros::ok())
    {
        tree.tickRoot();
        ros::spinOnce();
        usleep(1000);
    }

    return 0;
}