#include <ros/ros.h>
#include <iostream>
#include <string>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#define TAKEOFF_HEIGHT 2
#define FORMATION_RADIUS 2
#define NUM_AGENTS 5
#define PI 3.1415926535897

// will store the orgiin of the agent.Set in main function
geometry_msgs::PoseStamped origin;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

nav_msgs::Odometry global_pos;
void global_pos_cb(const nav_msgs::Odometry::ConstPtr& msg){
    global_pos = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

// generatee the formation point of agent from centre of formation
geometry_msgs::PoseStamped waypointToFormationPose(geometry_msgs::PoseStamped global_waypoint, int drone_number)
{
    float d_theta = (float)(PI/NUM_AGENTS);
    float theta = 2 * d_theta * (drone_number); // anticlockwise measured from positive x axis

    geometry_msgs::PoseStamped formation_point;

    formation_point.pose = global_waypoint.pose;
    formation_point.pose.position.x += FORMATION_RADIUS*std::cos(theta);
    formation_point.pose.position.y += FORMATION_RADIUS*std::sin(theta);

    ROS_INFO("Drone %d global target: %f, %f", drone_number, formation_point.pose.position.x, formation_point.pose.position.y);
    return formation_point;
}

// convert the formation point from global frame to local frame of the agent
geometry_msgs::PoseStamped globalToLocalFrame(geometry_msgs::PoseStamped formation_point_global)
{
    // add the delta to the current local pose of the agent to get target pose in local frame
    geometry_msgs::PoseStamped formation_point_local;
    formation_point_local.pose.position.x = formation_point_global.pose.position.x - origin.pose.position.x;
    formation_point_local.pose.position.y = formation_point_global.pose.position.y - origin.pose.position.y;
    formation_point_local.pose.position.z = formation_point_global.pose.position.z - origin.pose.position.z;

    ROS_INFO("Drone global position: %f %f", global_pos.pose.pose.position.x, global_pos.pose.pose.position.y);
    ROS_INFO("Drone local target: %f, %f", formation_point_local.pose.position.x, formation_point_local.pose.position.y);
    return formation_point_local;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_member");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, global_pos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    // get the process ID from command line
    int drone_number = std::stoi(argv[1]);

    // get the origin of the agent and set it into local variable
    std::string origin_x, origin_y, origin_z;
    nh.getParam("origin/x", origin.pose.position.x);
    nh.getParam("origin/y", origin.pose.position.y);
    nh.getParam("origin/z", origin.pose.position.z);
    
    // origin.pose.position.x = stoi(origin_x);
    // origin.pose.position.y = stoi(origin_y);
    // origin.pose.position.z = stoi(origin_z);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = TAKEOFF_HEIGHT;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // Sets the offboard mode and arms the drone
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
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

    pose.pose.position.y = 7;
    pose.pose.position.x = 7;

    // stores the list of waypoints to traverse
    nav_msgs::Path waypoints;
    waypoints.poses.push_back(pose);

    // iterate through target points. Single point for now
    for(auto global_waypoint:waypoints.poses)
    {
        // convert the target point into formation point for the drone
        geometry_msgs::PoseStamped formation_pose_global = waypointToFormationPose(global_waypoint, drone_number);
        // convert the formation point from global frame to local frame
        geometry_msgs::PoseStamped formation_pose_local = globalToLocalFrame(formation_pose_global);
        // send target point and do whatever else
        while(ros::ok())
        {
            local_pos_pub.publish(formation_pose_local);
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    return 0;
}