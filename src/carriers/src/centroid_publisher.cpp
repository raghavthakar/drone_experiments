#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centroid_publisher");
    ros::NodeHandle node_handle;

    ros::Publisher centroid_publisher=node_handle.advertise<geometry_msgs::Pose>("centroid", 100);

    //Handles the parameter for number of drones
    int drone_count;
    ros::param::get("/drone_count", drone_count);

    while(ros::ok())
    {
        geometry_msgs::Pose centroid;
        std::string uav_globalposition_topic;
        geometry_msgs::PoseConstPtr uav_globalposition;

        for(int i=0; i<drone_count; i++)
        {
            uav_globalposition_topic="/uav"+std::to_string(i)+"/global_position/cartesian";
            // ROS_INFO("%s", uav_globalposition_topic.c_str());
            uav_globalposition=ros::topic::waitForMessage<geometry_msgs::Pose>
                                                            (uav_globalposition_topic);

            // ROS_INFO("Drone %d x %f and y %f", drone_count,
            //         uav_globalposition->position.x,uav_globalposition->position.y);

            centroid.position.x+=uav_globalposition->position.x;
            centroid.position.y+=uav_globalposition->position.y;
            centroid.position.z+=uav_globalposition->position.z;
        }

        centroid.position.x/=drone_count;
        centroid.position.y/=drone_count;
        centroid.position.z/=drone_count;

        ROS_INFO("centroid: %f %f %f", centroid.position.x, centroid.position.y, centroid.position.z);
        centroid_publisher.publish(centroid);
    }
    ROS_INFO("ENded publishing");
}
