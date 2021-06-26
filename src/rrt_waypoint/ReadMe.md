# Waypoint Navigation for Drones Using RRT
### See the drone in action: https://youtu.be/VgvTWIBN-yA.

Alternatively, view the entire process video [here](https://youtu.be/ASD3BhYQ9qo).

This repository allows a drone to navigate across a room with obstacles, while making sure to pass through the provided waypoints in the correct order. **MAVSDK** API
was utilised along with **PX4-SITL** to control the drone and track its behaviour.

## How It Works
This implementation takes in co-ordinates for the `start` and `target` points, along with several waypoints in a local frame. The RRT algorithm is
then employed to plot a path from `start` to `waypoint#1`, `waypoint#1` to `waypoint#2` and so on until `target` is reached.

Obstacles can be configured as per requirements by uncommenting the relevant code in the `map` class. Right now, they have been assumed to represent the
following configuration:

![visualiser](https://github.com/raghavthakar/drone_nav/blob/main/src/rrt_waypoint/ReadMe_assets/visualiser.png)

Once the path from start to target point has been plotted, it is stored in a local CSV file called `path.csv`. This file is used by `rrt_waypoint` to control
a drone in the local NED co-ordinate system.

`rrt_waypoint` utilises offboard control mode provided by **MAVSDK** to make the drone follow the relevant path.

## Try It Yourself
### Get the Required Code and Applications
Begin by cloning this (**drone_nav**) repository, along with [this](https://github.com/raghavthakar/PX4-Autopilot) fork of the **PX4-Autopilot** stack. 
This particular fork has the required obstacle world already made and set up (`PX4-Autopilot/Tools/sitl_gazebo/worlds/aepl_room.world`).

![aepl_room](https://github.com/raghavthakar/drone_nav/blob/main/src/rrt_waypoint/ReadMe_assets/aepl_room.png)

Install the required PX4 dependencies by running `bash ./PX4-Autopilot/Tools/setup/ubuntu.sh` where you have cloned the repository.

Install **MAVSDK** from [here](https://github.com/mavlink/MAVSDK/releases). Download, and double click to install. Make sure to pick the version
compatible with your system, as your applications just won't build otherwise.

Once the above steps are complete, install a stable build of **QGroundController** from [here](https://github.com/mavlink/qgroundcontrol/releases).

### Set Up the Environment and Test
Navigate to the `drone_nav/src/rrt_waypoint/src/build` directory. 

Run:
```
make
```
This should build the `rrt` and `rrt_waypoint` files, and make them ready to be run.

In a new terminal, run:
```
export PX4_SITL_WORLD=~/PX4-Autopilot/Tools/sitl_gazebo/worlds/empty.world
```
This will configure your simulation to open in the required world wth obstacles.

Then, run:
```
make px4_sitl gazebo_typhoon_h480
```
This might take some time the first time, but ultimately you should be able to see gazebo open up with a drone at point (1, 1) in the room with obstacles.
This drone has an in-built video streaming plugin, which can be toggled with a green button on the top left, that says `video`.

![drone_in_world](https://github.com/raghavthakar/drone_nav/blob/main/src/rrt_waypoint/ReadMe_assets/drone_in_world.png)

Open **QGroundControl** to connect with your drone.
![](https://github.com/raghavthakar/drone_nav/blob/main/src/rrt_waypoint/ReadMe_assets/qgc_home.png)
   
If you cannot see a video stream from your drone, go to `Application Settings`->`General`->`Video` and amke sure it looks like this:
![](https://github.com/raghavthakar/drone_nav/blob/main/src/rrt_waypoint/ReadMe_assets/qgc_settings.png)

### Run the Navigation Code
Once your drone is in the world and the environment is set, you can run the `rrt` executable. In the `drone_nav/src/rrt_waypoint/src` directory run:
```
build/rrt
```
The code should run, and should ask you information about where the starting point is, where target is, and where the waypoints are. Inputs between 0-1000
as row/column number are valid.

### Visualise
Once the path avoiding the obstacles has been plotted, you can visualise the tree by navigating to the `scripts` directory and running the `visualiser.py` file using:
```
./visualiser.py
```
It should show you the generated tree, something like this:
![visualiser](https://github.com/raghavthakar/drone_nav/blob/main/src/rrt_waypoint/ReadMe_assets/visualiser.png)

### Observe the Path on a Drone
The `rrt` executable generates a CSV file called `path.csv` that stores the path for the drone to follow. You can observe this path actually being followed by a drone by running, in the `rrt_waypoint/src` directory:
```
build/rrt_waypoint udp://:14540
```
This should run the `rrt_waypoint` application, and make your drone follow the set path, avoiding any obstacles on its path. This is achieved using the offboard control mode offered by **MAVSDK**.

The co-ordinates system is changed to a local NED system, which also accounts for the starting (1, 1) position of the drone.

While the drone executes the movement along the path, you may view the video feed as well as track the movement of the drone on ***QGroundControl***.
