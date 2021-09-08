#include "ros/ros.h"
#include "std_msgs/String.h"
#include "carriers/CentreOfMass.h"
#include <iostream>
#include <string>

bool get_centre_of_mass(carriers::CentreOfMass::Request &request,
                        carriers::CentreOfMass::Response &response)
{
    ROS_INFO("IN THE SERVICE");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centre_of_mass_server");
    ros::NodeHandle node_handle;

    ros::ServiceServer COM=node_handle.advertiseService
                            ("centre_of_mass", get_centre_of_mass);
    ROS_INFO("Determining centre of mass");
    ros::spin();

    return 0;
}
