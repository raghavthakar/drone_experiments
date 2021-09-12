## Start a world with drone
In the PX4-Autopilot directory: `make px4_sitl gazebo`

## Launch mavros
mavros communicates via udp port 14557: `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

## Run the test takeoff script
Source the `drone_nav` package: `source devel/setup.bash`

Run the offboard control node: `rosrun carriers offb_node`

## Test out the same script with multiple drones
### Launch the world
Close everything, and in a new terminal:

`cd ~/PX4-Autopilot/`, or wherever the Autopilot installation is in your system

`source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)`

`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo`

Then launch the sitl simulation with multiple drones.

`roslaunch px4 multi_uav_mavros_sitl.launch`

Feel free to modify this launch file to launch as many drones as you wish (max 255). Please note the different namespaces for each drone, and the fact that each drone can be accessed via a different udp port (14540, 14541 and so on. Similarly 14580, 14581...).

### Launch the controlling script
Navigate to the home directory of this repository. Make sure it is built: `catkin_make`.

Source the workspace: `source devel/setup.zsh`

Launch the test takeoff nodes for each drone by launching:

`roslaunch carriers multi_uav_offb.launch`

This launch file launches a node for each drone. Make sure that the namespaces are consistent with `multi_uav_mavros_sitl.launch`. This launch file ___does not___ launch the mavros node. That is again taken care of by `multi_uav_mavros_sitl.launch`.

### Modification to the px4 launch file
To add a little more functionality, modify the file `single_vehicle_spawn.launch` in the autopilot firmware by adding the following lines:

    <!-- Push the starting position to the parameter server -->
    <param name="initial_position/x" value="$(arg x)"/>
    <param name="initial_position/y" value="$(arg y)"/>
    <param name="initial_position/z" value="$(arg z)"/>

This allows the system to know the starting postion of each drone, which can be used in addtiton to the odometry data to accurately identify the
position of each drone in the global frame.
