#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

address='0.0.0.0'
port=4443

function ctrl_c() {
  echo "Stopping background diode process"
  kill -9 $(ps -ef | grep 'diode' | grep -v 'grep' | awk '{print $2}')
  echo "diode stopped"
  echo "Stopping https server process"
  pkill -f https
  echo "https stopped"
  fuser ${port}/tcp
}

# launch ngrok in a terminal
/bin/sh -ec "python3 ./https.py --address=${address} --port=${port} &"
# /bin/sh -ec "python3 -m http.server ${port} &"
# launch robot system in another terminal
/bin/sh -ec "diode publish -public ${port}:80"
