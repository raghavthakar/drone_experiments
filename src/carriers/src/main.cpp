#include "ros/ros.h"
#include <string>
#include <unistd.h>
#include <geometry_msgs/Point.h>

#define FORMATION_RADIUS 2

// SUPORTED SWARM STATE:
// - HORIZONTAL_FORMATION
// - VERTICAL_FORMATION
// - NAVIGATE
// - DISABLED

// THIS FILE CONTROLS THE HIGH LEVEL BEHAVIOUR OF THE SWARM
int main(int argc, char** argv)
{
    geometry_msgs::Point centroid;
    centroid.x=0;
    centroid.y=0;
    centroid.z=10;

    ros::init(argc, argv, "main");
    ros::NodeHandle node_handle;

    ros::param::set("centroid/x", centroid.x);
    ros::param::set("centroid/y", centroid.y);
    ros::param::set("centroid/z", centroid.z);

    ros::param::set("/formation_radius", FORMATION_RADIUS);

    std::string swarm_state="HORIZONTAL_FORMATION";
    ros::param::set("/swarm_state", swarm_state);
    sleep(25);

    swarm_state="VERTICAL_FORMATION";
    ros::param::set("/swarm_state", swarm_state);
    sleep(5);

    swarm_state="DISABLED";
    ros::param::set("/swarm_state", swarm_state);

    return 1;
}
