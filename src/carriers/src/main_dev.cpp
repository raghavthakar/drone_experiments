#include "ros/ros.h"
#include <string>
#include <unistd.h>
#include <geometry_msgs/Pose.h>

// THIS FILE CONTROLS THE HIGH LEVEL BEHAVIOUR OF THE SWARM
int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle node_handle;

    ROS_INFO("Handling main\n");

    return 1;
}
