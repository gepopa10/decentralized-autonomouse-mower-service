#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

flask_port=3001
http_port=4443

while getopts p1:p2: flag
do
    case "${flag}" in
        p1) flask_port=${OPTARG};;
        p2) http_port=${OPTARG};;
    esac
done

function ctrl_c() {
  echo "Stopping background ngrok process"
  kill -9 $(ps -ef | grep 'ngrok' | grep -v 'grep' | awk '{print $2}')
  echo "ngrok stopped"
  echo "Killing flask port"
  fuser -k ${flask_port}/tcp
  echo "Flask port $flask_port killed"
  fuser -k ${http_port}/tcp
}

# launch ngrok in a terminal
/bin/sh -ec 'ngrok tcp 22 --log=stdout > ngrok.log &'
# launch robot system in another terminal
/bin/sh -ec "roslaunch turtlebot3_fake turtlebot3_path_dev_rviz.launch flask_app_server_port:=${flask_port}"
# /bin/sh -ec "roslaunch turtlebot3_gazebo turtlebot3_world_grass_dev.launch flask_app_server_port:=${flask_port}"
