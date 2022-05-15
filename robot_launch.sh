#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

flask_port=3001
http_port=4443
web_rosbridge_external_address='chainlink-robot.diode.link'
web_rosbridge_external_port=8100
rosbridge_port=9095
image_file='`pwd`/camera/image.jpg'

while getopts p1:p2:wa:wp: flag
do
    case "${flag}" in
        p1) flask_port=${OPTARG};;
        p2) http_port=${OPTARG};;
        wa) web_rosbridge_external_address=${OPTARG};;
        wp) web_rosbridge_external_port=${OPTARG};;
        rp) rosbridge_port=${OPTARG};;
    esac
done

function ctrl_c() {
  echo "Killing flask port"
  fuser -k ${flask_port}/tcp
  echo "Flask port $flask_port killed"
  fuser -k ${http_port}/tcp
}

# -----------------------------------------------------------------------------
# RVIZ
# -----------------------------------------------------------------------------
# /bin/sh -ec "roslaunch turtlebot3_fake turtlebot3_path_dev_rviz.launch \
# flask_app_server_port:=${flask_port} \
# web_rosbridge_external_address:=${web_rosbridge_external_address} \
# web_rosbridge_external_port:=${web_rosbridge_external_port} \
# rosbridge_server_port:=${rosbridge_port}"

# -----------------------------------------------------------------------------
# Gazebo
# -----------------------------------------------------------------------------
# /bin/sh -ec "roslaunch turtlebot3_gazebo turtlebot3_world_grass_dev_gazebo.launch \
# flask_app_server_port:=${flask_port} \
# web_rosbridge_external_address:=${web_rosbridge_external_address} \
# web_rosbridge_external_port:=${web_rosbridge_external_port} \
# rosbridge_server_port:=${rosbridge_port} \
# image_file:=${image_file}"

# -----------------------------------------------------------------------------
# Gazebo + RVIZ
# -----------------------------------------------------------------------------
/bin/sh -ec "roslaunch turtlebot3_gazebo turtlebot3_world_grass_dev.launch \
flask_app_server_port:=${flask_port} \
web_rosbridge_external_address:=${web_rosbridge_external_address} \
web_rosbridge_external_port:=${web_rosbridge_external_port} \
rosbridge_server_port:=${rosbridge_port} \
image_file:=${image_file}"
