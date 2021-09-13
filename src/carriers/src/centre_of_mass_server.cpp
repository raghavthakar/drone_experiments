#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "carriers/CentreOfMass.h"
#include <iostream>
#include <string>

bool get_centre_of_mass(carriers::CentreOfMass::Request &request,
                        carriers::CentreOfMass::Response &response)
{
    //Handles the parameter for number of drones
    int drone_count;
    ros::param::get("/drone_count", drone_count);

    ROS_INFO("NO of drones: %d", drone_count);

    std::string uav_globalposition_topic;
    geometry_msgs::PoseConstPtr uav_globalposition;
    for(int i=0; i<drone_count; i++)
    {
        uav_globalposition_topic="/uav"+std::to_string(i)+"/global_position/cartesian";
        ROS_INFO("%s", uav_globalposition_topic.c_str());
        uav_globalposition=ros::topic::waitForMessage<geometry_msgs::Pose>
                                                        (uav_globalposition_topic);

        ROS_INFO("Drone %d x %f and y %f", drone_count,
                uav_globalposition->position.x,uav_globalposition->position.y);

        response.COM.position.x+=uav_globalposition->position.x;
        response.COM.position.y+=uav_globalposition->position.y;
        response.COM.position.z+=uav_globalposition->position.z;
    }

    response.COM.position.x/=drone_count;
    response.COM.position.y/=drone_count;
    response.COM.position.z/=drone_count;

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

    return 1;
}
