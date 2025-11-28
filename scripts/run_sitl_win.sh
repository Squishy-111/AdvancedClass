#!/bin/bash


# This script is supposed to be in the root directory of the ardupilot repository

#grab the window host ip as seen by WSL (default gateway)
WIN_HOST_IP=$(ip route | awk '/^default/ {print $3; exit}')
echo "Using Windows host IP: $WIN_HOST_IP"

# Run SITL and send MAVLink to:
#  - 14550 for Mission Planner
#  - 14551 for whatever python script running
./Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -L UofA \
  --out=udp:${WIN_HOST_IP}:14550 \
  --out=udp:${WIN_HOST_IP}:14551