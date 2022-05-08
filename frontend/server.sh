#!/bin/bash

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

address='0.0.0.0'
port=4443

while getopts a:p: flag
do
    case "${flag}" in
        a) address=${OPTARG};;
        p) port=${OPTARG};;
    esac
done

function ctrl_c() {
  fuser -k -n tcp ${port}
  echo "Stopping background diode process"
  kill -9 $(ps -ef | grep 'diode' | grep -v 'grep' | awk '{print $2}')
  echo "diode stopped"
  echo "Stopping https server process"
  pkill -f https
  echo "https stopped"
}

# http server
# /bin/sh -ec "python3 ./https.py --address=${address} --port=${port} &" #diode subdomain innacessible via https
/bin/sh -ec "python3 -m http.server ${port} &"
