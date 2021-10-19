#include "ros/ros.h"
#include <string>
#include <unistd.h>
#include <geometry_msgs/Pose.h>

#define FORMATION_RADIUS 2

// SUPORTED SWARM STATE:
// - HORIZONTAL_FORMATION
// - VERTICAL_FORMATION
// - NAVIGATE
// - DISABLED

// Function to make the formation around a specified point
// maybe impleent this as a service
void moveTo(geometry_msgs::Pose centroid)
{
    // push the centroid to the paramater server
    ros::param::set("/centroid/x", centroid.position.x);
    ros::param::set("/centroid/y", centroid.position.y);
    ros::param::set("/centroid/z", centroid.position.z);
}

// THIS FILE CONTROLS THE HIGH LEVEL BEHAVIOUR OF THE SWARM
int main(int argc, char** argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle node_handle;

    // publisher to publish the centroid of the system, has to be latched
    ros::Publisher centroid_publisher = node_handle.advertise<geometry_msgs::Pose>
    ("centroid", 10, true);

    // Centroid will represent the system as a whole
    geometry_msgs::Pose centroid;
    centroid.position.x=0;
    centroid.position.y=0;
    centroid.position.z=20;

    ros::param::set("/formation_radius", FORMATION_RADIUS);

    std::string swarm_state="HORIZONTAL_FORMATION";
    ros::param::set("/swarm_state", swarm_state);

    // move the system to this point
    moveTo(centroid);

    ros::Rate rate(20);
    while(ros::ok())
    {
        continue;
    }
    sleep(35);

    swarm_state="VERTICAL_FORMATION";
    ros::param::set("/swarm_state", swarm_state);
    sleep(5);

    swarm_state="DISABLED";
    ros::param::set("/swarm_state", swarm_state);

    return 1;
}
