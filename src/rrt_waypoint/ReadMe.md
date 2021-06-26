# Waypoint Navigation for Drones Using RRT
## rrt visualiser photo, empty aepl world, drone in world, qgc, qgc settings
This repository allows a drone to navigate across a room with obstacles, while making sure to pass through the provided waypoints in the correct order. **MAVSDK** API
was utilised along with **PX4-SITL** to control the drone and track its behaviour.

## How It Works
This implementation takes in co-ordinates for the `start` and `target` points, along with several waypoints in a local frame. The RRT algorithm is
then employed to plot a path from `start` to `waypoint#1`, `waypoint#1` to `waypoint#2` and so on until `target` is reached.

Obstacles can be configured as per requirements by uncommenting the relevant code in the `map` class. Right, they have been assumed to represent the
following configuration:

Once the path from start to target point has been plotted, it is stored in a local CSV file called `path.csv`. This file is used by `rrt_waypoint` to control
a drone in the local NED co-ordinate system.

`rrt_waypoint` utilises offboard control mode provided by **MAVSDK** to make the drone follow the relevant path.

## Try It Yourself
### Get the Required Code and Applications
Begin by cloning this (**drone_nav**) repository, along with [this](https://github.com/raghavthakar/PX4-Autopilot) fork of the **PX4-Autopilot** stack. 
This particular fork has the required obstacle world already made and set up (`PX4-Autopilot/Tools/sitl_gazebo/worlds/aepl_room.world`).

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
This might take some time the first time, but ultimately you should be able to see gazebo open up witha drone at point (1, 1) in the room with obstacles.
This drone has an in-built video streaming plugin, which can be toggled with a green button on the top left, that says `video`.

Open **QGroundControl** to connect with your drone. If you cannot see a video stream from your drone, go to `Application Settings`->`General`->