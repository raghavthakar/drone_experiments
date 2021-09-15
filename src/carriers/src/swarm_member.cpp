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

// Store the centroid in a global variable
geometry_msgs::Pose centroid;
void centroid_cb(const geometry_msgs::PoseConstPtr& msg){
    centroid = *msg;
    ROS_INFO("centroid z: %f", centroid.position.z);
}

// Get the COM of the drones, then position drone based on drone_id and no. of drones and formation radius
void horizontalFormation(ros::ServiceClient COM_client, int drone_id,
        int drone_count, ros::Publisher local_pos_pub, ros::Subscriber centroid_sub)
{
    int formation_radius;

    // Will be used to give target position to the drone
    geometry_msgs::PoseStamped target_pose;

    carriers::CentreOfMass COM;
    // Get the centrw of mass of th system
    ROS_INFO("HORIZONTAL_FORMATION");
    // if (COM_client.call(COM))
    // {
    //   ROS_INFO("Got COM successfully");
    // }
    // else
    // {
    //   ROS_ERROR("Failed to call service centre_of_mass_server");
    //   return;
    // }

    ros::Duration some_time(0.01);
    centroid=*ros::topic::waitForMessage<geometry_msgs::Pose>("/centroid");

    // Get the radius of formation
    ros::param::get("/formation_radius", formation_radius);

    // decide the position angle in the formation for the drone
    double formation_position_angle=drone_id*2*PI/drone_count;

    // print the formation position angle
    // ROS_INFO("Drone count: %d Drone: %d Formation position angle: %f",
    //         drone_count, drone_id, formation_position_angle);

    // Assign the position
    target_pose.pose.position.x=centroid.position.x+formation_radius*std::cos(formation_position_angle);
    target_pose.pose.position.y=centroid.position.y+formation_radius*std::sin(formation_position_angle);
    target_pose.pose.position.z=centroid.position.z;
    // ROS_INFO("Centroid z: %f", centroid.position.z);
    // ROS_INFO("COM x: %f y: %f z: %f", COM.response.COM.position.x,
    //             COM.response.COM.position.y, COM.response.COM.position.z);

    // ros::Rate rate(30);
    // for(int i=0; i<10; i++)
    // {
    //     local_pos_pub.publish(target_pose);
    //     rate.sleep();
    // }
    local_pos_pub.publish(target_pose);
}

void verticalFormation(ros::ServiceClient COM_client, int drone_id,
        int drone_count, ros::Publisher local_pos_pub, ros::Subscriber centroid_sub)
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

    // Used to keep track of the centroid of the swarm
    ros::Subscriber centroid_sub;
    // ros::Subscriber centroid_sub = nh.subscribe<geometry_msgs::Pose>
    //         ("/centroid", 10, centroid_cb);

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
    pose.pose.position.z = 12;

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

    for(int i=0; i<250; i++)
    {
        local_pos_pub.publish(pose);
        rate.sleep();
    }

    // State machine
    while(swarm_state!="DISABLED")
    {
        // To keep the drone count updated
        ros::param::get("/drone_count", drone_count);
        ros::param::get("/swarm_state", swarm_state);

        if(swarm_state=="HORIZONTAL_FORMATION")
        {
            horizontalFormation(COM_client, drone_id, drone_count, local_pos_pub, centroid_sub);
        }
        else if(swarm_state=="VERTICAL_FORMATION")
        {
            verticalFormation(COM_client, drone_id, drone_count, local_pos_pub, centroid_sub);
        }
        else
        {
            break;
        }
    }

    ROS_INFO("All good, exiting!");

    return 0;
}
