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
#include <std_msgs/String.h>

#define TAKEOFF_HEIGHT 2
#define FORMATION_RADIUS 2
#define POSITION_ERROR_LIMIT 0.4
#define NUM_AGENTS 5
#define PI 3.1415926535897

// will store the orgiin of the agent.Set in main function
geometry_msgs::PoseStamped origin;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

std_msgs::String token_message;
void update_newest_token(const std_msgs::String::ConstPtr& msg){
    token_message = *msg;
}

// generatee the formation point of agent from centre of formation
geometry_msgs::PoseStamped waypointToFormationPose(geometry_msgs::PoseStamped global_waypoint, int drone_id)
{
    float d_theta = (float)(PI/NUM_AGENTS);
    float theta = 2 * d_theta * (drone_id); // anticlockwise measured from positive x axis

    geometry_msgs::PoseStamped formation_point;

    formation_point.pose = global_waypoint.pose;
    formation_point.pose.position.x += FORMATION_RADIUS*std::cos(theta);
    formation_point.pose.position.y += FORMATION_RADIUS*std::sin(theta);

    ROS_INFO("Drone %d global target: %f, %f", drone_id, formation_point.pose.position.x, formation_point.pose.position.y);
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

    ROS_INFO("Drone local target: %f, %f", formation_point_local.pose.position.x, formation_point_local.pose.position.y);
    return formation_point_local;
}

bool inPosition(geometry_msgs::PoseStamped formation_point_local)
{
    return sqrt((formation_point_local.pose.position.x-local_pos.pose.position.x)*
                (formation_point_local.pose.position.x-local_pos.pose.position.x)+
                (formation_point_local.pose.position.y-local_pos.pose.position.y)*
                (formation_point_local.pose.position.y-local_pos.pose.position.y)+
                (formation_point_local.pose.position.z-local_pos.pose.position.z)*
                (formation_point_local.pose.position.z-local_pos.pose.position.z)) < POSITION_ERROR_LIMIT;
}

void continueTokenRing(ros::Publisher local_pos_pub, geometry_msgs::PoseStamped formation_pose_local, int drone_id, ros::Rate rate,
                        ros::Publisher token_ring_pub)
{
    ROS_INFO("Drone %d is in token ring", drone_id);
    // maintain old position till the token message is not the current drone_id-1
    while(ros::ok())
    {
        local_pos_pub.publish(formation_pose_local);

        if(token_message.data.compare("TOKEN-"+std::to_string((drone_id+NUM_AGENTS-1)%NUM_AGENTS))==0)
        {
            ROS_INFO("Drone %d received token", drone_id);
            ROS_INFO_STREAM(token_message.data);
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    std_msgs::String token;
    token.data = "TOKEN-"+std::to_string((drone_id+NUM_AGENTS)%NUM_AGENTS);

    token_ring_pub.publish(token);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swarm_member");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber token_ring_sub = nh.subscribe<std_msgs::String>
            ("/token_ring", 10, update_newest_token);
    ros::Publisher token_ring_pub = nh.advertise<std_msgs::String>
            ("/token_ring", 10, true);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    // get the process ID from command line
    int drone_id = std::stoi(argv[1]);

    // get the origin of the agent and set it into global variable
    nh.getParam("origin/x", origin.pose.position.x);
    nh.getParam("origin/y", origin.pose.position.y);
    nh.getParam("origin/z", origin.pose.position.z);

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

    pose.pose.position.y = 2;
    pose.pose.position.x = 2;

    // stores the list of waypoints to traverse
    nav_msgs::Path waypoints;
    waypoints.poses.push_back(pose);

    pose.pose.position.y = 4;
    pose.pose.position.x = 9;
    waypoints.poses.push_back(pose);

    pose.pose.position.y = 4;
    pose.pose.position.x = 14;
    waypoints.poses.push_back(pose);

    pose.pose.position.y = 4;
    pose.pose.position.x = 14;
    pose.pose.position.z = 5;
    waypoints.poses.push_back(pose);

    pose.pose.position.y = 4;
    pose.pose.position.x = 4;
    pose.pose.position.z = 1;
    waypoints.poses.push_back(pose);

    int waypoint_num = 0;
    // iterate through target points.
    for(auto global_waypoint:waypoints.poses)
    {  
        // convert the target point into formation point for the drone
        geometry_msgs::PoseStamped formation_pose_global = waypointToFormationPose(global_waypoint, drone_id);
        // convert the formation point from global frame to local frame
        geometry_msgs::PoseStamped formation_pose_local = globalToLocalFrame(formation_pose_global);
        // send target point and do whatever else until the drone isn't in formation
        while(ros::ok() && !inPosition(formation_pose_local))
        {
            local_pos_pub.publish(formation_pose_local);
            ros::spinOnce();
            rate.sleep();
        }

        // agent with ID = waypoint number % NUM_DRONES initiates token ring algorithm
        // other agents carry forward token ring

        continueTokenRing(local_pos_pub, formation_pose_local, drone_id, rate, token_ring_pub);

        waypoint_num++;
    }
    
    return 0;
}