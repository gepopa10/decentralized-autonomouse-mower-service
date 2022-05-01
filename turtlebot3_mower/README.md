# turtlebot3_mower

This repo is a modified version of the turtlebot3 and turtlebot3_simulations packages that can be downloaded here:

(1) [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) \
(2) [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

The launch files in `/turtlebot3_simulations/turtlebot3_gazebo/launch` and the robot description files in /turtlebot3/turtlebot3_decription/urdf are modified to support simulation with PX4 and the rover_hardware_interface.

To launch: \
`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

PX4 SITL is launching: \
`turtlebot3_world_px4.launch`
