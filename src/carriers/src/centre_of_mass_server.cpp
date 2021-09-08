#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
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
    sensor_msgs::NavSatFixConstPtr uav_globalposition;
    for(int i=0; i<drone_count; i++)
    {
        uav_globalposition_topic="/uav"+std::to_string(i)+"/mavros/global_position/global";
        ROS_INFO("%s", uav_globalposition_topic.c_str());
        uav_globalposition=ros::topic::waitForMessage<sensor_msgs::NavSatFix>
                                                        (uav_globalposition_topic);

        ROS_INFO("Drone %d lat %f and long %f", drone_count,
                uav_globalposition->latitude,uav_globalposition->longitude);

        response.COM.latitude+=uav_globalposition->latitude;
        response.COM.longitude+=uav_globalposition->longitude;
        response.COM.altitude+=uav_globalposition->altitude;
    }

    response.COM.latitude/=drone_count;
    response.COM.longitude/=drone_count;
    response.COM.altitude/=drone_count;

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
