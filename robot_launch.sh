#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

flask_port=3001

function ctrl_c() {
  echo "Stopping background ngrok process"
  kill -9 $(ps -ef | grep 'ngrok' | grep -v 'grep' | awk '{print $2}')
  echo "ngrok stopped"
  echo "Killing flask port"
  fuser -k ${flask_port}/tcp
  echo "Flask port $flask_port killed"
}

# launch ngrok in a terminal
/bin/sh -ec 'ngrok tcp 22 --log=stdout > ngrok.log &'
# launch robot system in another terminal
/bin/sh -ec "roslaunch turtlebot3_fake turtlebot3_path_dev_rviz.launch flask_app_server_port:=${flask_port}"
