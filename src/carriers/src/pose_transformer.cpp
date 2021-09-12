// Transform the pose of drone from its local frame to global coordinate frame
//subscirbe to /uav2/mavros/local_position/pose and publish to topic of choice
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <string>

class PoseTransformer
{
    ros::NodeHandle node_handle;
    std::string local_position_topic_name;
    ros::Subscriber local_position_subscriber;
    std::string global_position_topic_name;
    ros::Publisher global_position_publisher;
    char drone_id;
    std::string uav_ns;//handle the namespace of the drone (/uav0, /uav1 etc)

    //strings to get the initial positions from the paramterr server
    std::string initial_position_x_param_name;
    std::string initial_position_y_param_name;
    std::string initial_position_z_param_name;

    //this structure stores the starting position of the drone
    struct initialPosition
    {
        float x;
        float y;
        float z;
    }initial_position;

public:
    PoseTransformer(char** argv)
    {
        //get the drone id from inline argument
        drone_id=(*argv[1]);

        //construct the uav namespace
        uav_ns="/uav";
        uav_ns.push_back(drone_id);

        //construct the parameter names for initial position
        initial_position_x_param_name=uav_ns+"/initial_position/x";
        initial_position_y_param_name=uav_ns+"/initial_position/y";
        initial_position_z_param_name=uav_ns+"/initial_position/z";

        //get the initial position fro the paramater server
        ros::param::get(initial_position_x_param_name, initial_position.x);
        ros::param::get(initial_position_y_param_name, initial_position.y);
        ros::param::get(initial_position_z_param_name, initial_position.z);

        //Construct the name of the topci to publish global position to
        global_position_topic_name=uav_ns;
        global_position_topic_name+="/global_position/cartesian";

        //Publisher to publish tje global positions
        global_position_publisher=node_handle.advertise<geometry_msgs::Pose>
                                            (global_position_topic_name, 10);

        //Construct the name of the topic to get local position of the drone
        local_position_topic_name=uav_ns;
        local_position_topic_name+="/mavros/local_position/pose";

        //Subscriber to get the local positions
        local_position_subscriber=node_handle.subscribe(local_position_topic_name,
                                                100, &PoseTransformer::transformPose, this);
    }

    //callback when local position is received, Should publish transformed position
    void transformPose(const geometry_msgs::PoseStamped& local_position)
    {
        ROS_INFO("In the callback!Drone id: %c, inital position x:%f y:%f", drone_id, initial_position.x, initial_position.y);

        //Make same as local position
        geometry_msgs::Pose transformed_pose=local_position.pose;

        //Add intial position to give transformed pose
        transformed_pose.position.x+=initial_position.x;
        transformed_pose.position.y+=initial_position.y;
        transformed_pose.position.z+=initial_position.z;

        //Publish the tranformed pose
        global_position_publisher.publish(transformed_pose);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_transformer");
    PoseTransformer pose_transformer(argv);
    ros::spin();
    return 1;
}
