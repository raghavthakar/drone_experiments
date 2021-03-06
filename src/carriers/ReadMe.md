## Background

This project is an attempt at developing a fully decentralised system for formation and navigation of multiple drone. The motive behind this project is to create an abstraction layer that allows one to control swarm behaviour with simple service calls, with the actual operations taking care of themselves.

Presently the system is capable of gettting into a formation in a decentralised manner and following a set of waypoints, for any number of drones (<10).

The Token Ring Algorithm was used to ensure consistency mutual exclusion as the agents settle into a formation.

Here are 5 drones:
![5 drones making formation](https://github.com/raghavthakar/drone_experiments/blob/main/ReadMe_Resources/5_drone_formation_token_ring.gif)

## Concept

Every drone is assigned a drone ID before the start of the mission. The ID is an integer number that dictates the drone's position in the formation. Each drone identifies the number of drones in the formation and using the ID, calculates where it should place itself to form a symmetric formation around a waypoint.

Once the position has been calculated, each drone converges to the desired position. Starting from the drone with ID equal to `waypoint_num % NUM_AGENTS`, the token ring algorithm is initiated. The selected agent generates a token, and as soon as it takes its place in the formation, it attaches its ID to the token and publishes. Each agent stays in the formation untill it reads a token with ID 1 less than that of itself. This ensures that all agents leading to say, drone ID X, will be in formation.

Presently, every drone has been given the same list of waypoints. In the future I would like to compute the plan onboard and distribute it among the other agents.

#### To do:
- Implement facility to generate formations in any plane

## Try It Yourself

#### What you will need
- MAVROS
- PX4-Autopilot stack
- This repository

#### Setup the repository
    git clone https://github.com/raghavthakar/drone_experiments.git
    cd drone_experiments
    catkin_make
    source devel/setup.bash #or .zsh

#### Setup the PX4-Autopilot

```
cd ~/PX4-Autopilot/ #or wherever the Autopilot installation is in your system
git checkout tags/v1.12.2 #I faced errors with the latest unstable version
DONT_RUN=1 make px4_sitl gazebo #this builds the sitl simulation
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

### Modification to the px4 launch file
To add a little more functionality, modify the file `single_vehicle_spawn.launch` in the autopilot firmware by adding the following lines:

    <!-- Push the starting position to the parameter server -->
    <param name="initial_position/x" value="$(arg x)"/>
    <param name="initial_position/y" value="$(arg y)"/>
    <param name="initial_position/z" value="$(arg z)"/>

This allows the system to know the starting postion of each drone, which can be used in addtiton to the odometry data to accurately identify the
position of each drone in the global frame.

#### Launch the sitl simulation with multiple drones

`roslaunch px4 multi_uav_mavros_sitl.launch`

Feel free to modify this launch file to launch as many drones as you wish (max 255). Please note the different namespaces for each drone, and the fact that each drone can be accessed via a different udp port (14540, 14541 and so on. Similarly 14580, 14581...).

#### Launch the swarm
Navigate to the home directory of this repository. Make sure it is built. Build using command: `catkin_make`.

Source the workspace: `source devel/setup.zsh`

Launch the test takeoff nodes for each drone by launching:

`roslaunch carriers multi_uav_swarm.launch`

This launch file launches a node for each drone. Make sure that the namespaces are consistent with `multi_uav_mavros_sitl.launch`. This launch file ___does not___ launch the mavros node. That is again taken care of by `multi_uav_mavros_sitl.launch`.
