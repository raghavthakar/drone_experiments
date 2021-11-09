//Controlling the behaviour of every drone in the swarm

// - Make a formation based on the number of drones in the swarm
// - Then follow waypoint navigation in the formation

// Essentially this file controls the low level behaviour of an individual agent
// we determine the target position of the drone based on the formation frame position and heading
// and then control the drone to be there

#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#define FORMATION_RADIUS 2
#define POSITION_ERROR_LIMIT 0.2
#define PI 3.14159265359

//Store the current state in a global varibale through callback
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    ROS_INFO("state callback!");
    current_state = *msg;
}

// store the current position of the drone in global variable thrugh callback
geometry_msgs::PoseStamped current_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO("inside callback!");
    current_position=*msg;
    ROS_INFO("%g", current_position.pose.position.x);
    ROS_INFO("%g", current_position.pose.position.y);
    ROS_INFO("%g", current_position.pose.position.z);
}

// converts the frame of the drone to its local fraeme
geometry_msgs::PoseStamped convertToLocal(geometry_msgs::PoseStamped target_pose)
{
    // convert the drone target position to local frame of drone
    // by subtracting the intiial position of the drone
    int init_position_x;
    int init_position_y;
    int init_position_z;

    // get the intiial position of the drone
    ros::param::get("initial_position/x", init_position_x);
    ros::param::get("initial_position/y", init_position_y);
    ros::param::get("initial_position/z", init_position_z);

    // subtract from target position to convert frames
    target_pose.pose.position.x-=init_position_x;
    target_pose.pose.position.y-=init_position_y;
    target_pose.pose.position.z-=init_position_z;

    return target_pose;
}

// return the euclidean distance between two poses
float euclideanDistance(geometry_msgs::PoseStamped drone_target_pose,
    geometry_msgs::PoseStamped current_position)
{
    return sqrt((drone_target_pose.pose.position.x-current_position.pose.position.x)*
                (drone_target_pose.pose.position.x-current_position.pose.position.x)+
                (drone_target_pose.pose.position.y-current_position.pose.position.y)*
                (drone_target_pose.pose.position.y-current_position.pose.position.y)+
                (drone_target_pose.pose.position.z-current_position.pose.position.z)*
                (drone_target_pose.pose.position.z-current_position.pose.position.z));
}

// returns the target position of the drone in the local frame
geometry_msgs::PoseStamped getDroneTargetPose(geometry_msgs::PoseStamped swarm_centroid,
    int drone_id, int drone_count)
{
    geometry_msgs::PoseStamped target_pose;
    target_pose=swarm_centroid;

    // set position based on drone id
    target_pose.pose.position.x+=FORMATION_RADIUS*cos(drone_id*2*PI/drone_count);
    target_pose.pose.position.y+=FORMATION_RADIUS*sin(drone_id*2*PI/drone_count);

    // convert the positions to local frame of drone
    target_pose=convertToLocal(target_pose);

    return target_pose;
}

// checks to see if all the drones are in the right place for correct formation
bool inFormation(geometry_msgs::PoseStamped drone_target_pose, int waypoint_no,
    int drone_id, int drone_count, ros::NodeHandle &nh)
{
    std_msgs::Float64 euclidean_distance;

    // each drone calculates the error in its position and target individually
    // and updates its paramater
    const std::string current_position_topic="mavros/local_position/pose";
    geometry_msgs::PoseStamped::ConstPtr current_position=ros::topic::waitForMessage<geometry_msgs::PoseStamped>(current_position_topic);

    // get the euclidean distance between currentand target pose and store it
    euclidean_distance.data=euclideanDistance(drone_target_pose, *current_position);

    // put this euclidean dustance on the paramter server
    // ros::param::set("distance_from_target_pose", euclidean_distance.data);

    // ROS_INFO("Drone %d waypoint no %d", drone_id, waypoint_no);

    // check the parameter of each drone and if all are in formation
    // then return true
    if(euclidean_distance.data<1.0)
    {
        std::string other_drone_distance_from_target_pose_param="/uav-/distance_from_target_pose";
        std::string other_drone_waypoint_no_param="/uav-/waypoint_no";
        int other_drone_waypoint_no;
        /*for(int i=0; i<drone_count; i++)
        {
            if(i==drone_id) continue;

            // i think what is happenin right now is that the systwem checks for
            // target reached for older targets as well which causes rest of the
            // formation to go ahead
            // can be solved by adding a target no. as another param to be checked

            // check if all drones have the same waypoint_no
            other_drone_waypoint_no_param[4]=1+'0';
            ros::param::get(other_drone_waypoint_no_param, other_drone_waypoint_no);


            // the drones need to only move on to the next target once all the
            // dornes have reached earlier target/waupoint


            other_drone_distance_from_target_pose_param[4]=i+'0';
            // ROS_INFO_STREAM(other_drone_distance_from_target_pose_param);
            std_msgs::Float64 other_drone_distance_from_target_pose;
            ros::param::get(other_drone_distance_from_target_pose_param,
                other_drone_distance_from_target_pose.data);

            if(other_drone_distance_from_target_pose.data>1.0 || waypoint_no!=other_drone_waypoint_no)
            {
                // ROS_INFO("Drone %d says %d not in formation", drone_id, i);
                // ROS_INFO("My waypoint_no: %d drone %d waypoint_no: %d", waypoint_no, i, other_drone_waypoint_no);
                return false;
            }
        }*/
        return true;
    }

    return false;
}

bool formation(geometry_msgs::PoseStamped centroid_pose, int waypoint_no, int drone_id,
    int drone_count, ros::Publisher local_pos_pub, ros::NodeHandle &nh)
{
    bool in_formation=false;

    // will store the target pose of the drone in formation
    geometry_msgs::PoseStamped drone_target_pose=getDroneTargetPose(centroid_pose,
        drone_id, drone_count);

    // publish the target position to the drone
    local_pos_pub.publish(drone_target_pose);


    return inFormation(drone_target_pose, waypoint_no, drone_id, drone_count, nh);
    // return false;
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

    // track the current position of the drone in global scope
    // ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("mavros/local_position/pose", 10, position_cb);

    // Used to establish stable stream before setting to offboard
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // Arming client
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    // Will let us set the drone to offboard mode
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // Stores the path that the centroid of the system must follow
    nav_msgs::Path path;

    // Striing to store the current swarm state
    std::string formation_state;

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

    // Starting position
    for(int i=0; i<250; i++)
    {
        local_pos_pub.publish(pose);
        rate.sleep();
    }

    ROS_INFO_STREAM(formation_state);

    // make-do path for now with just one pose
    pose.pose.position.z=20;
    path.poses.push_back(pose);

    pose.pose.position.x=25;
    path.poses.push_back(pose);

    pose.pose.position.y=25;
    path.poses.push_back(pose);

    pose.pose.position.y=-25;
    path.poses.push_back(pose);


    int waypoint_no=1; // tracks the waypoint that the system is approaching
    // State machine
    for(auto formation_centroid:path.poses)
    {
        // To keep the drone count updated
        ros::param::get("/drone_count", drone_count);
        ros::param::get("/formation_state", formation_state);

        // update the current waypoint number on the param server
        ros::param::set("waypoint_no", waypoint_no);

        ROS_INFO("Drone %d waypoint num: %d", waypoint_no, drone_id);

        // keep going till the formstion is made around the target
        while(!formation(formation_centroid, waypoint_no, drone_id, drone_count, local_pos_pub, nh)
                &&formation_state!="DISABLED");

        waypoint_no++; // update the waypoint number once it has been reached
    }

    ROS_INFO("All good, exiting!");

    return 0;
}
