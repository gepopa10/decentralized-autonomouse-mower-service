#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

http_port=4443
flask_port=3001
diode_flask_gateway=8050
web_rosbridge_external_address="chainlink-robot.diode.link"
web_rosbridge_external_port=8100
rosbridge_port=9095

function ctrl_c() {
  echo "Stopping background diode process"
  kill -9 $(ps -ef | grep 'diode' | grep -v 'grep' | awk '{print $2}')
  echo "diode stopped"
  fuser -k -n tcp ${http_port}
  fuser -k -n tcp ${flask_port}
}

# launch diode in a terminal
/bin/sh -ec "diode publish -public ${http_port}:80 -public ${flask_port}:${diode_flask_gateway} -public ${rosbridge_port}:${web_rosbridge_external_port} &"

/bin/sh -ec "cd frontend && ./server.sh -p ${http_port} &"
# source frontend/server.sh -p ${http_port}
source robot_launch.sh -p1 ${flask_port} -p2 ${http_port} -wa ${web_rosbridge_external_address} -wp ${web_rosbridge_external_port} -rp ${web_rosbridge_external_port}
