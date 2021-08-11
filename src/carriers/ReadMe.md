## Start a world with drone
In the PX4-Autopilot directory: `make px4_sitl gazebo`

## Launch mavros
mavros communicates via udp port 14557: `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

## Run the test takeoff script
Source the `drone_nav` package: `source devel/setup.bash`

Run the offboard control node: `rosrun carriers offb_node`
