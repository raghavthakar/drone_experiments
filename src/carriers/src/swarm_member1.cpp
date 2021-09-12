//Controlling the behaviour of every drone in the swarm

// - Make a formation based on the number of drones in the swarm
// - Then follow waypoint navigation in the formation

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "carriers/CentreOfMass.h"
#include "mavros_msgs/State.h"

//Store the current state in a global varibale through callback
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_member");
    ros::NodeHandle nh;

    int drone_count;

    ros::param::get("/drone_count", drone_count);
    // If drone_count is less than 1+drone_number, update it to drone_number
    if(drone_count<(int)(1+*argv[1])-48)
        ros::param::set("/drone_count", (int)(1+*argv[1])-48);
}
