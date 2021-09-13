//Controlling the behaviour of every drone in the swarm

// - Make a formation based on the number of drones in the swarm
// - Then follow waypoint navigation in the formation

#include <ros/ros.h>
#include <cmath>
#include "carriers/CentreOfMass.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define PI 3.14159265359

// Get the COM of the drones, then position drone based on drone_id and no. of drones and formation radius
void horizontalFormation(ros::ServiceClient COM_client, int drone_id,
        int drone_count, ros::Publisher local_pos_pub)
{
    int formation_radius;

    carriers::CentreOfMass COM;
    // Get the centrw of mass of th system
    ROS_INFO("HORIZONTAL_FORMATION");
    if (COM_client.call(COM))
    {
      ROS_INFO("Got COM successfully");
    }
    else
    {
      ROS_ERROR("Failed to call service centre_of_mass_server");
      return;
    }

    // Get the radius of formation
    ros::param::get("/formation_radius", formation_radius);

    // decide the position angle in the formation for the drone
    double formation_position_angle=drone_id*2*PI/drone_count;

    // print the formation position angle
    ROS_INFO("Drone count: %d Drone: %d Formation position angle: %f",
            drone_count, drone_id, formation_position_angle);

}

void verticalFormation(ros::ServiceClient COM_client, int drone_id,
        int drone_count, ros::Publisher local_pos_pub)
{
    ROS_INFO("VERTICAL_FORMATION");
}

//Store the current state in a global varibale through callback
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swarm_member");
    ros::NodeHandle nh;

    // Get the id of the drone by converting from char
    int drone_id=(int)(*argv[1])-48;

    int drone_count;
    ros::param::get("/drone_count", drone_count);
    // If drone_count is less than 1+drone_id, update it to drone_id
    if(drone_count<1+drone_id)
        ros::param::set("/drone_count", 1+drone_id);

    // Mechnaism to handle the current state if the drone
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    // Used to establish stable stream before setting to offboard
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // Arming client
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    // Will let us set the drone to offboard mode
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // Client for getting the COM
    ros::ServiceClient COM_client = nh.serviceClient<carriers::CentreOfMass>
            ("/centre_of_mass");

    // Striing to store the current swarm state
    std::string swarm_state;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    ros::Time last_request = ros::Time::now();
    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Basically set the mode to offboard and arm the drone
    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }
                last_request = ros::Time::now();
            }
        }

        if( arming_client.call(arm_cmd) && arm_cmd.response.success &&
            set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            break;

        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Armed: %d", current_state.armed);

    // State machine
    while(swarm_state!="DISABLED")
    {
        // To keep the drone count updated
        ros::param::get("/drone_count", drone_count);
        ros::param::get("/swarm_state", swarm_state);

        if(swarm_state=="HORIZONTAL_FORMATION")
        {
            horizontalFormation(COM_client, drone_id, drone_count, local_pos_pub);
        }
        else if(swarm_state=="VERTICAL_FORMATION")
        {
            verticalFormation(COM_client, drone_id, drone_count, local_pos_pub);
        }
        else
        {
            break;
        }
    }

    ROS_INFO("All good, exiting!");

    return 0;
}
