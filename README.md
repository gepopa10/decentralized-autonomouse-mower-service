# Decentralized Autonomous Grass Mower Service

![Simulation Rviz](images/rviz.png?raw=true "Title")

To launch development environnment with rviz:
```
roslaunch turtlebot3_fake turtlebot3_path_dev_rviz.launch
```
To launch development environnment with gazebo:
```
roslaunch turtlebot3_gazebo turtlebot3_world_grass_dev_gazebo.launch
```
To launch development environnment with gazebo and rviz:
```
roslaunch turtlebot3_gazebo turtlebot3_world_grass_dev.launch
```

Call it with the input file with the nb of waypoints that has to be smaller than the total nb of points within the `fullpath_meters.txt`:
```
rosservice call publish_raw_path_from_fullpath_meters "fullpath_meters.txt" 4000
```
