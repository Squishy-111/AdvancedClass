#!/bin/bash
# Script to run ArduPilot SITL for Webots

# Find Windows IP automatically
WIN_HOST_IP=$(ip route | awk '/^default/ {print $3; exit}')
echo "Targeting Windows host IP: $WIN_HOST_IP"

# Run SITL
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  --model webots-python \
  --add-param-file=/home/camrozendaal/ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm \
  --sim-address=${WIN_HOST_IP} \
  --out=udp:${WIN_HOST_IP}:14551
